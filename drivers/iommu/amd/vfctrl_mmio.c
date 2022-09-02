// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2023 Advanced Micro Devices, Inc.
 * Author: Suravee Suthikulpanit <suravee.suthikulpanit@amd.com>
 */

#define pr_fmt(fmt)     "AMD-Vi: " fmt
#define dev_fmt(fmt)    pr_fmt(fmt)

#include <linux/iommu.h>
#include <linux/amd-iommu.h>
#include <linux/amd-viommu.h>

#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/ioctl.h>
#include <linux/iommufd.h>
#include <uapi/linux/iommufd.h>
#include <linux/mem_encrypt.h>
#include <uapi/linux/amd_viommu.h>

#include <asm/iommu.h>
#include <asm/set_memory.h>

#include "amd_iommu.h"
#include "amd_iommu_types.h"
#include "amd_viommu.h"


#define GET_CTRL_BITS(reg, bit, msk)	(((reg) >> (bit)) & (ULL(msk)))
#define SET_CTRL_BITS(reg, bit1, bit2, msk) \
	((((reg) >> (bit1)) & (ULL(msk))) << (bit2))

int amd_viommu_guest_mmio_read(struct amd_viommu_mmio_data *data)
{
	u8 __iomem *vfctrl, *vf;
	u64 val, tmp = 0;
	int gid = data->gid;
	struct amd_iommu *iommu = get_amd_iommu_from_devid(data->iommu_devid);

	if (!iommu)
		return -ENODEV;

	vf = VIOMMU_VF_MMIO_BASE(iommu, gid);
	vfctrl = VIOMMU_VFCTRL_MMIO_BASE(iommu, gid);

	switch (data->offset) {
	case MMIO_CONTROL_OFFSET:
	{
		/* VFCTRL offset 20h */
		val = readq(vfctrl + 0x20);
		tmp |= SET_CTRL_BITS(val, 8, CONTROL_CMDBUF_EN, 1); // [12]
		tmp |= SET_CTRL_BITS(val, 9, CONTROL_COMWAIT_EN, 1); // [4]

		/* VFCTRL offset 28h */
		val = readq(vfctrl + 0x28);
		tmp |= SET_CTRL_BITS(val, 8, CONTROL_EVT_LOG_EN, 1); // [2]
		tmp |= SET_CTRL_BITS(val, 9, CONTROL_EVT_INT_EN, 1); // [3]
		tmp |= SET_CTRL_BITS(val, 10, CONTROL_DUALEVTLOG_EN, 3); // [33:32]

		/* VFCTRL offset 30h */
		val = readq(vfctrl + 0x30);
		tmp |= SET_CTRL_BITS(val, 8, CONTROL_PPRLOG_EN, 1); // [13]
		tmp |= SET_CTRL_BITS(val, 9, CONTROL_PPRINT_EN, 1); // [14]
		tmp |= SET_CTRL_BITS(val, 10, CONTROL_PPR_EN, 1); // [15]
		tmp |= SET_CTRL_BITS(val, 11, CONTROL_DUALPPRLOG_EN, 3); // [31:30]
		tmp |= SET_CTRL_BITS(val, 13, CONTROL_PPR_AUTO_RSP_EN, 1); // [39]
		tmp |= SET_CTRL_BITS(val, 14, CONTROL_BLKSTOPMRK_EN, 1); // [41]
		tmp |= SET_CTRL_BITS(val, 15, CONTROL_PPR_AUTO_RSP_AON, 1); // [42]

		data->value = tmp;
		break;
	}
	default:
		pr_err("%s: Invalid offset: iommu_devid=%#x, gid=%u, offset=%#x, value=%#llx, mmio_size=%u, is_write=%u\n",
			 __func__, data->iommu_devid, gid, data->offset,
			 data->value, data->mmio_size, data->is_write);
		break;
	}

	pr_debug("%s: iommu_devid=%#x, gid=%u, offset=%#x, value=%#llx, mmio_size=%u, is_write=%u\n",
		 __func__, data->iommu_devid, gid, data->offset,
		 data->value, data->mmio_size, data->is_write);
	return 0;
}
EXPORT_SYMBOL(amd_viommu_guest_mmio_read);

/* Note:
 * This function maps the guest MMIO write to AMD IOMMU MMIO registers
 * into vIOMMU VFCTRL register bits.
 */
int amd_viommu_guest_mmio_write(struct amd_viommu_mmio_data *data)
{
	u8 __iomem *vfctrl, *vf;
	int gid = data->gid;
	u64 val = 0, tmp, ctrl = data->value;
	struct amd_iommu *iommu = get_amd_iommu_from_devid(data->iommu_devid);

	if (!iommu)
		return -ENODEV;

	pr_debug("%s: iommu_devid=%#x, gid=%u, offset=%#x, value=%#llx, mmio_size=%u, is_write=%u\n",
		 __func__, data->iommu_devid, gid, data->offset,
		 ctrl, data->mmio_size, data->is_write);

	vf = VIOMMU_VF_MMIO_BASE(iommu, gid);
	vfctrl = VIOMMU_VFCTRL_MMIO_BASE(iommu, gid);

	switch (data->offset) {
	case MMIO_CONTROL_OFFSET:
	{
		/* VFCTRL offset 20h */
		val = readq(vfctrl + 0x20);
		val &= ~(0x3ULL << 8);
		tmp = GET_CTRL_BITS(ctrl, CONTROL_CMDBUF_EN, 1); // [12]
		val |= (tmp << 8);
		tmp = GET_CTRL_BITS(ctrl, CONTROL_COMWAIT_EN, 1); // [4]
		val |= (tmp << 9);
		writeq(val, vfctrl + 0x20);

		/* VFCTRL offset 28h */
		val = readq(vfctrl + 0x28);
		val &= ~(0xFULL << 8);
		tmp = GET_CTRL_BITS(ctrl, CONTROL_EVT_LOG_EN, 1); // [2]
		val |= (tmp << 8);
		tmp = GET_CTRL_BITS(ctrl, CONTROL_EVT_INT_EN, 1); // [3]
		val |= (tmp << 9);
		tmp = GET_CTRL_BITS(ctrl, CONTROL_DUALEVTLOG_EN, 3); // [33:32]
		val |= (tmp << 10);
		writeq(val, vfctrl + 0x28);

		/* VFCTRL offset 30h */
		val = readq(vfctrl + 0x30);
		val &= ~(0xFFULL << 8);
		tmp = GET_CTRL_BITS(ctrl, CONTROL_PPRLOG_EN, 1); // [13]
		val |= (tmp << 8);
		tmp = GET_CTRL_BITS(ctrl, CONTROL_PPRINT_EN, 1); // [14]
		val |= (tmp << 9);
		tmp = GET_CTRL_BITS(ctrl, CONTROL_PPR_EN, 1); // [15]
		val |= (tmp << 10);
		tmp = GET_CTRL_BITS(ctrl, CONTROL_DUALPPRLOG_EN, 3); // [31:30]
		val |= (tmp << 11);
		tmp = GET_CTRL_BITS(ctrl, CONTROL_PPR_AUTO_RSP_EN, 1); // [39]
		val |= (tmp << 13);
		tmp = GET_CTRL_BITS(ctrl, CONTROL_BLKSTOPMRK_EN, 1); // [41]
		val |= (tmp << 14);
		tmp = GET_CTRL_BITS(ctrl, CONTROL_PPR_AUTO_RSP_AON, 1); // [42]
		val |= (tmp << 15);
		writeq(val, vfctrl + 0x30);
		break;
	}
	default:
		pr_err("%s: Invalid offset: iommu_devid=%#x, gid=%u, offset=%#x, val=%#llx, ctrl=%#llx\n",
			 __func__, data->iommu_devid, gid, data->offset, val, ctrl);
		break;
	}

	return 0;
}
EXPORT_SYMBOL(amd_viommu_guest_mmio_write);
