/* SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note */
/*
 * AMD Hardwaer Accelerated Virtualized IOMMU (HW-vIOMMU)
 *
 * Copyright (c) 2023, Advanced Micro Devices, Inc.
 *
 */
#ifndef _UAPI_AMD_VIOMMU_H_
#define _UAPI_AMD_VIOMMU_H_

#include <linux/types.h>
#include <linux/ioctl.h>

/**
 * The ioctl interfaces in this file are specific for AMD HW-vIOMMU.
 * They are an extension of extend the IOMMUFD ioctl interfaces.
 * Please see include/uapi/linux/iommufd.h for more detail.
 */
#include <linux/iommufd.h>

enum iommufd_viommu_cmd {
	IOMMUFD_VIOMMU_CMD_BASE = 0x60,
	IOMMUFD_CMD_MMIO_ACCESS = IOMMUFD_VIOMMU_CMD_BASE,
};

/**
 * struct amd_viommu_mmio_data- ioctl(VIOMMU_MMIO_ACCESS)
 * @size: sizeof(struct amd_viommu_mmio_data)
 * @iommu_id: PCI device ID of the AMD IOMMU instance
 * @gid: guest ID
 * @offset: specify MMIO offset
 * @value: specify MMIO write value or retrieving MMIO read value
 * @mmio_size: specify MMIO size
 * @is_write: specify MMIO read (0) / write (1)
 *
 * - Trap guest IOMMU MMIO write to program HW-vIOMMU for the specified
 *   guest.
 * - Trap guest IOMMU MMIO read to emulate return value for the specified
 *   guest.
 */
struct amd_viommu_mmio_data {
	__u32	size;
	__u32	iommu_devid;
	__u32	gid;
	__u32	offset;
	__u64	value;
	__u32	mmio_size;
	__u8	is_write;
};

#define VIOMMU_MMIO_ACCESS	_IO(IOMMUFD_TYPE, IOMMUFD_CMD_MMIO_ACCESS)

#endif
