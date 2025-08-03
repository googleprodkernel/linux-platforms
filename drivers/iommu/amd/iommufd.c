// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2025 Advanced Micro Devices, Inc.
 */

#include <linux/iommu.h>

#include "iommufd.h"
#include "amd_iommu.h"
#include "amd_viommu.h"
#include "amd_iommu_types.h"

static const struct iommufd_viommu_ops amd_viommu_ops;

void *amd_iommufd_hw_info(struct device *dev, u32 *length, u32 *type)
{
	struct iommu_hw_info_amd *hwinfo;

	if (*type != IOMMU_HW_INFO_TYPE_DEFAULT &&
	    *type != IOMMU_HW_INFO_TYPE_AMD)
		return ERR_PTR(-EOPNOTSUPP);

	hwinfo = kzalloc(sizeof(*hwinfo), GFP_KERNEL);
	if (!hwinfo)
		return ERR_PTR(-ENOMEM);

	*length = sizeof(*hwinfo);
	*type = IOMMU_HW_INFO_TYPE_AMD;

	hwinfo->efr = amd_iommu_efr;
	hwinfo->efr2 = amd_iommu_efr2;

	return hwinfo;
}

size_t amd_iommufd_get_viommu_size(struct device *dev, enum iommu_viommu_type viommu_type)
{
	if (viommu_type != IOMMU_VIOMMU_TYPE_AMD)
		return 0;

	return VIOMMU_STRUCT_SIZE(struct amd_iommu_viommu, core);
}

int amd_iommufd_viommu_init(struct iommufd_viommu *viommu, struct iommu_domain *parent,
			    const struct iommu_user_data *user_data)
{
	int ret;
	unsigned long flags;
	struct iommu_viommu_amd data;
	struct protection_domain *pdom = to_pdomain(parent);
	struct amd_iommu_viommu *aviommu = container_of(viommu, struct amd_iommu_viommu, core);

	xa_init(&aviommu->gdomid_array);
	aviommu->parent = pdom;

	if (!user_data)
		return -EINVAL;

	ret = iommu_copy_struct_from_user(&data, user_data,
					  IOMMU_VIOMMU_TYPE_AMD,
					  reserved);
	if (ret)
		return ret;

	aviommu->gid = amd_iommu_gid_alloc();
	if (aviommu->gid < 0)
		return aviommu->gid;
	data.out_gid = aviommu->gid;

	ret = iommu_copy_struct_to_user(user_data, &data,
					IOMMU_VIOMMU_TYPE_AMD,
					reserved);
	if (ret)
		goto err_out;

	aviommu->iommu_devid = data.iommu_devid;
	viommu->ops = &amd_viommu_ops;

	spin_lock_irqsave(&pdom->lock, flags);
	list_add(&aviommu->pdom_list, &pdom->viommu_list);
	spin_unlock_irqrestore(&pdom->lock, flags);

	return 0;

err_out:
	amd_iommu_gid_free(aviommu->gid);
	return ret;
}

static void amd_iommufd_viommu_destroy(struct iommufd_viommu *viommu)
{
	unsigned long flags;
	struct amd_iommu_viommu *entry, *next;
	struct amd_iommu_viommu *aviommu = container_of(viommu, struct amd_iommu_viommu, core);
	struct protection_domain *pdom = aviommu->parent;

	pr_debug("%s: gid:%#x\n", __func__, aviommu->gid);

	spin_lock_irqsave(&pdom->lock, flags);
	list_for_each_entry_safe(entry, next, &pdom->viommu_list, pdom_list) {
		if (entry == aviommu)
			list_del(&entry->pdom_list);
	}
	spin_unlock_irqrestore(&pdom->lock, flags);

	amd_iommu_gid_free(aviommu->gid);
}

static void set_dev_data_viommu(struct amd_iommu *iommu, u16 hDevId, u16 gid, u16 gDevId)
{
	struct iommu_dev_data *dev_data = search_dev_data(iommu, hDevId);

	if (!dev_data) {
		pr_err("%s: Failed to get host devid %#x\n", __func__, hDevId);
		return;
	}

	dev_data->gid = gid;
	dev_data->gDevId = gDevId;
}

/*
 * Program the DevID via VFCTRL registers
 * This function will be called during VM init via VFIO.
 */
static void set_device_mapping(struct amd_iommu *iommu, u16 hDevId,
			       u16 guestId, u16 queueId, u16 gDevId)
{
	u64 val, tmp1, tmp2;
	u8 __iomem *vfctrl;

	pr_debug("%s: iommu_devid=%#x, gid=%#x, hDevId=%#x, gDevId=%#x\n",
		__func__, pci_dev_id(iommu->dev), guestId, hDevId, gDevId);

	tmp1 = gDevId;
	tmp1 = ((tmp1 & 0xFFFFULL) << 46);
	tmp2 = hDevId;
	tmp2 = ((tmp2 & 0xFFFFULL) << 14);
	val = tmp1 | tmp2 | 0x8000000000000001ULL;
	vfctrl = VIOMMU_VFCTRL_MMIO_BASE(iommu, guestId);
	writeq(val, vfctrl + VIOMMU_VFCTRL_GUEST_DID_MAP_CONTROL0_OFFSET);
}

/*
 * Called from drivers/iommu/iommufd/viommu.c: iommufd_vdevice_alloc_ioctl()
 */
static int _amd_viommu_vdevice_init(struct iommufd_vdevice *vdev)
{
	unsigned long flags;
	struct iommu_domain *dom;
	struct protection_domain *pdom;
	struct iommufd_viommu *viommu = vdev->viommu;
	struct amd_iommu_viommu *aviommu = container_of(viommu, struct amd_iommu_viommu, core);
	struct amd_iommu *iommu = container_of(viommu->iommu_dev, struct amd_iommu, iommu);
	struct pci_dev *pdev = to_pci_dev(vdev->dev);
	u16 hdev_id = pci_dev_id(pdev);
	u16 gdev_id = vdev->virt_id;
	struct iommu_dev_data *dev_data;

	if (!pdev) {
		pr_err();
		return -EINVAL;
	}

	dev_data = dev_iommu_priv_get(&pdev->dev);
	if (!dev_data) {
		pr_err("%s: Device not found (devid=%#x)\n",
		       __func__, pci_dev_id(pdev));
		return -EINVAL;
	}

	dom = iommu_get_domain_for_dev(&pdev->dev);
	if (!dom) {
		pr_err("%s: Domain not found (devid=%#x)\n",
		       __func__, pci_dev_id(pdev));
		return -EINVAL;
	}

	pr_debug("%s: gid=%#x, iommu_devid=%#x, hdev_id=%#x, gdev_id=%#x\n",
		 __func__, aviommu->gid, aviommu->iommu_devid, hdev_id, gdev_id);

	/* TODO: Hardcode queueid to 0 for now */
	set_device_mapping(iommu, hdev_id, aviommu->gid, 0, gdev_id);

	set_dev_data_viommu(iommu, dev_data->devid, aviommu->gid, gdev_id);

	pdom = to_pdomain(dom);
	spin_lock_irqsave(&pdom->lock, flags);
	amd_iommu_domain_flush_all(pdom);
	spin_unlock_irqrestore(&pdom->lock, flags);

	return 0;
}

/*
 * See include/linux/iommufd.h
 * struct iommufd_viommu_ops - vIOMMU specific operations
 */
static const struct iommufd_viommu_ops amd_viommu_ops = {
	.alloc_domain_nested = amd_iommu_alloc_domain_nested,
	.destroy = amd_iommufd_viommu_destroy,
	.vdevice_size = VDEVICE_STRUCT_SIZE(struct amd_iommu_vdevice, core),
	.vdevice_init = _amd_viommu_vdevice_init,
};
