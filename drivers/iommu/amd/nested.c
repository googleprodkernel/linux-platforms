// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2025 Advanced Micro Devices, Inc.
 */

#define dev_fmt(fmt)	"AMD-Vi: " fmt

#include <linux/iommu.h>
#include <linux/refcount.h>
#include <uapi/linux/iommufd.h>

#include "amd_iommu.h"

static const struct iommu_domain_ops nested_domain_ops;

static inline struct nested_domain *to_ndomain(struct iommu_domain *dom)
{
	return container_of(dom, struct nested_domain, domain);
}

/*
 * Validate guest DTE to make sure that configuration for host (v1)
 * and guest (v2) page tables are valid when allocating nested domain.
 */
static int validate_gdte_nested(struct iommu_hwpt_amd_guest *gdte)
{
	u32 gpt_level = FIELD_GET(DTE_GPT_LEVEL_MASK, gdte->dte[2]);

	/* Must be zero: Mode, Host-TPR */
	if (FIELD_GET(DTE_MODE_MASK, gdte->dte[0]) != 0 ||
	    FIELD_GET(DTE_HOST_TRP, gdte->dte[0]) != 0)
		return -EINVAL;

	/* Must be non-zero: V, GIOV, GV, GCR3 TRP */
	if (FIELD_GET(DTE_FLAG_V, gdte->dte[0]) == 0 ||
	    FIELD_GET(DTE_FLAG_GV, gdte->dte[0]) == 0 ||
	    (FIELD_GET(DTE_GCR3_14_12, gdte->dte[0]) == 0 &&
	     FIELD_GET(DTE_GCR3_30_15, gdte->dte[1]) == 0 &&
	     FIELD_GET(DTE_GCR3_51_31, gdte->dte[1]) == 0))
		return -EINVAL;

	/* Valid Guest Paging Mode values are 0 and 1 */
	if (gpt_level != GUEST_PGTABLE_4_LEVEL &&
	    gpt_level != GUEST_PGTABLE_5_LEVEL)
		return -EINVAL;

	/* GLX = 3 is reserved */
	if (FIELD_GET(DTE_GLX, gdte->dte[0]) == 3)
		return -EINVAL;

	/*
	 * We need to check host capability before setting
	 * the Guest Paging Mode
	 */
	if (gpt_level == GUEST_PGTABLE_5_LEVEL &&
	    amd_iommu_gpt_level < PAGE_MODE_5_LEVEL)
		return -EOPNOTSUPP;

	return 0;
}

/*
 * This function is assigned to struct iommufd_viommu_ops.alloc_domain_nested()
 * during the call to struct iommu_ops.viommu_init().
 */
struct iommu_domain *
amd_iommu_alloc_domain_nested(struct iommufd_viommu *viommu, u32 flags,
			      const struct iommu_user_data *user_data)
{
	int ret;
	struct nested_domain *ndom;
	struct guest_domain_mapping_info *gdom_info, *curr;
	struct amd_iommu_viommu *aviommu = container_of(viommu, struct amd_iommu_viommu, core);

	if (user_data->type != IOMMU_HWPT_DATA_AMD_GUEST)
		return ERR_PTR(-EOPNOTSUPP);

	ndom = kzalloc(sizeof(*ndom), GFP_KERNEL);
	if (!ndom)
		return ERR_PTR(-ENOMEM);

	ret = iommu_copy_struct_from_user(&ndom->gdte, user_data,
					  IOMMU_HWPT_DATA_AMD_GUEST,
					  dte);
	if (ret)
		goto out_err;

	ret = validate_gdte_nested(&ndom->gdte);
	if (ret)
		goto out_err;

	ndom->gdom_id = FIELD_GET(DTE_DOMID_MASK, ndom->gdte.dte[1]);
	ndom->domain.ops = &nested_domain_ops;
	ndom->domain.type = IOMMU_DOMAIN_NESTED;
	ndom->viommu = aviommu;

	gdom_info = kzalloc(sizeof(*gdom_info), GFP_KERNEL);
	if (!gdom_info)
		goto out_err;

	/*
	 * Normally, when a guest has multiple pass-through devices,
	 * the IOMMU driver setup DTEs with the same stage-2 table and
	 * use the same host domain ID (hDomId). In case of nested translation,
	 * if the guest setup different stage-1 tables with same PASID,
	 * IOMMU would use the same TLB tag. This will results in TLB
	 * aliasing issue.
	 *
	 * The guest is assigning gDomIDs based on its own algorithm for managing
	 * cache tags of (DomID, PASID). Within a single viommu, the nest parent domain
	 * (w/ S2 table) is used by all DTEs. But we need to consistently map the gDomID
	 * to a single hDomID. This is done using an xarray in the vIOMMU to
	 * keep track of the gDomID mapping. When the S2 is changed, the INVALIDATE_IOMMU_PAGES
	 * command must be issued for each hDomID in the xarray.
	 */
	curr = xa_cmpxchg(&aviommu->gdomid_array,
			  ndom->gdom_id, NULL, gdom_info, GFP_ATOMIC);
	if (curr) {
		if (xa_err(curr)) {
			ret = -EINVAL;
			goto out_err_gdom_info;
		} else {
			/* The gDomID already exist */
			pr_debug("%s: Found gdom_id=%#x, hdom_id=%#x\n",
				 __func__, ndom->gdom_id, curr->hdom_id);
			refcount_inc(&curr->users);
			ndom->gdom_info = curr;
			kfree(gdom_info);
			return &ndom->domain;
		}
	}

	/* The gDomID does not exist. We allocate new hdom_id */
	gdom_info->hdom_id = amd_iommu_pdom_id_alloc();
	if (gdom_info->hdom_id <= 0) {
		xa_cmpxchg(&aviommu->gdomid_array,
			   ndom->gdom_id, gdom_info, NULL, GFP_ATOMIC);
		ret = -ENOSPC;
		goto out_err_gdom_info;
	}

	refcount_set(&gdom_info->users, 1);
	ndom->gdom_info = gdom_info;
	pr_debug("%s: Allocate gdom_id=%#x, hdom_id=%#x\n",
		 __func__, ndom->gdom_id, gdom_info->hdom_id);

	return &ndom->domain;

out_err_gdom_info:
	kfree(gdom_info);
out_err:
	kfree(ndom);
	return ERR_PTR(ret);
}

static void nested_domain_free(struct iommu_domain *dom)
{
	struct guest_domain_mapping_info *curr;
	struct nested_domain *ndom = to_ndomain(dom);
	struct amd_iommu_viommu *aviommu = ndom->viommu;

	if (!refcount_dec_and_test(&ndom->gdom_info->users))
		return;

	/*
	 * The refcount for the gdom_id to hdom_id mapping is zero.
	 * It is now safe to remove the mapping.
	 */
	curr = xa_cmpxchg(&aviommu->gdomid_array, ndom->gdom_id,
			  ndom->gdom_info, NULL, GFP_ATOMIC);
	if (curr) {
		if (xa_err(curr)) {
			pr_err("%s: Failed to free nested domain gdom_id=%#x\n",
			       __func__, ndom->gdom_id);
			return;
		}

		/* success */
		pr_debug("%s: Free gdom_id=%#x, hdom_id=%#x\n",
			__func__, ndom->gdom_id, curr->hdom_id);
		kfree(curr);
	}

	amd_iommu_pdom_id_free(ndom->gdom_info->hdom_id);
	kfree(ndom);
}

static const struct iommu_domain_ops nested_domain_ops = {
	.free = nested_domain_free,
};
