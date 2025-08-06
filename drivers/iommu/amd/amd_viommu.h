/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (C) 2023 Advanced Micro Devices, Inc.
 * Author: Suravee Suthikulpanit <suravee.suthikulpanit@amd.com>
 */

#ifndef AMD_VIOMMU_H
#define AMD_VIOMMU_H

#define VIOMMU_MAX_GUESTID	(1 << 16)

#define VIOMMU_VF_MMIO_ENTRY_SIZE		4096
#define VIOMMU_VFCTRL_MMIO_ENTRY_SIZE		64

#define VIOMMU_VFCTRL_GUEST_DID_MAP_CONTROL0_OFFSET	0x00
#define VIOMMU_VFCTRL_GUEST_DID_MAP_CONTROL1_OFFSET	0x08
#define VIOMMU_VFCTRL_GUEST_MISC_CONTROL_OFFSET		0x10

#define VIOMMU_VFCTRL_GUEST_CMD_CONTROL_OFFSET	0x20
#define VIOMMU_VFCTRL_GUEST_EVT_CONTROL_OFFSET	0x28
#define VIOMMU_VFCTRL_GUEST_PPR_CONTROL_OFFSET	0x30

#define VIOMMU_VF_MMIO_BASE(iommu, guestId) \
	(iommu->vf_base + (guestId * VIOMMU_VF_MMIO_ENTRY_SIZE))
#define VIOMMU_VFCTRL_MMIO_BASE(iommu, guestId) \
	(iommu->vfctrl_base + (guestId * VIOMMU_VFCTRL_MMIO_ENTRY_SIZE))

#define VIOMMU_GUEST_MMIO_BASE		0
#define VIOMMU_GUEST_MMIO_SIZE		(64 * VIOMMU_MAX_GUESTID)

#define VIOMMU_CMDBUF_DIRTY_STATUS_BASE	0x400000ULL
#define VIOMMU_CMDBUF_DIRTY_STATUS_SIZE	0x2000

#define VIOMMU_DEVID_MAPPING_BASE	0x1000000000ULL
#define VIOMMU_DEVID_MAPPING_ENTRY_SIZE	(1 << 20)

#define VIOMMU_DOMID_MAPPING_BASE	0x2000000000ULL
#define VIOMMU_DOMID_MAPPING_ENTRY_SIZE	(1 << 19)

#define VIOMMU_GUEST_CMDBUF_BASE	0x2800000000ULL
#define VIOMMU_GUEST_CMDBUF_SIZE	(1 << 19)

#define VIOMMU_GUEST_PPR_LOG_BASE	0x3000000000ULL
#define VIOMMU_GUEST_PPR_LOG_SIZE	(1 << 19)

#define VIOMMU_GUEST_PPR_B_LOG_BASE	0x3800000000ULL
#define VIOMMU_GUEST_PPR_B_LOG_SIZE	(1 << 19)

#define VIOMMU_GUEST_EVT_LOG_BASE	0x4000000000ULL
#define VIOMMU_GUEST_EVT_LOG_SIZE	(1 << 19)

#define VIOMMU_GUEST_EVT_B_LOG_BASE	0x4800000000ULL
#define VIOMMU_GUEST_EVT_B_LOG_SIZE	(1 << 19)


#if IS_ENABLED(CONFIG_AMD_IOMMU_IOMMUFD)

int amd_viommu_init(struct amd_iommu *iommu);

int amd_viommu_init_one(struct amd_iommu *iommu, struct amd_iommu_viommu *viommu);

void amd_viommu_uninit_one(struct amd_iommu *iommu, struct amd_iommu_viommu *viommu);

u64 amd_viommu_get_vfmmio_addr(struct iommu_viommu_amd *data);

int amd_viommu_domain_id_update(struct amd_iommu *iommu, u16 gid,
				u16 hdom_id, u16 gdom_id);
#else

static inline int amd_viommu_init(struct amd_iommu *iommu)
{
	return 0;
}

static inline int amd_viommu_init_one(struct amd_iommu *iommu, struct amd_iommu_viommu *viommu)
{
	return -EOPNOTSUPP;
}

static inline u64 amd_viommu_get_vfmmio_addr(struct iommu_viommu_amd *data)
{
	return 0;
}

#endif /* CONFIG_AMD_IOMMU_IOMMUFD */

#endif /* AMD_VIOMMU_H */
