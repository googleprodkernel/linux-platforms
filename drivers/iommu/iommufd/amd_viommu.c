// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2023 Advanced Micro Devices, Inc.
 * Author: Suravee Suthikulpanit <suravee.suthikulpanit@amd.com>
 */

#include <linux/file.h>
#include <linux/fs.h>
#include <linux/iommu.h>
#include <linux/iommufd.h>
#include <linux/amd-viommu.h>
#include <uapi/linux/iommufd.h>
#include <uapi/linux/amd_viommu.h>

#include "iommufd_private.h"

union amd_viommu_ucmd_buffer {
	struct amd_viommu_mmio_data mmio;
};

#define IOCTL_OP(_ioctl, _fn, _struct, _last)                                  \
	[_IOC_NR(_ioctl) - IOMMUFD_VIOMMU_CMD_BASE] = {                        \
		.size = sizeof(_struct) +                                      \
			BUILD_BUG_ON_ZERO(sizeof(union amd_viommu_ucmd_buffer) <          \
					  sizeof(_struct)),                    \
		.min_size = offsetofend(_struct, _last),                       \
		.ioctl_num = _ioctl,                                           \
		.execute = _fn,                                                \
	}

static int viommu_mmio_access(struct iommufd_ucmd *ucmd)
{
	int ret;
	struct amd_viommu_mmio_data *data = ucmd->cmd;

	if (data->is_write) {
		ret = amd_viommu_guest_mmio_write(data);
	} else {
		ret = amd_viommu_guest_mmio_read(data);
		if (ret)
			return ret;

		if (copy_to_user(ucmd->ubuffer, data, sizeof(*data)))
			ret = -EFAULT;
	}
	return ret;
}

struct iommufd_ioctl_op viommu_ioctl_ops[] = {
	IOCTL_OP(VIOMMU_MMIO_ACCESS, viommu_mmio_access,
		 struct amd_viommu_mmio_data, is_write),
};

long iommufd_amd_viommu_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct iommufd_ctx *ictx = filp->private_data;
	struct iommufd_ucmd ucmd = {};
	struct iommufd_ioctl_op *op;
	union amd_viommu_ucmd_buffer buf;
	unsigned int nr;
	int ret;

	nr = _IOC_NR(cmd);
	if (nr < IOMMUFD_VIOMMU_CMD_BASE ||
	    (nr - IOMMUFD_VIOMMU_CMD_BASE) >= ARRAY_SIZE(viommu_ioctl_ops))
		return -ENOIOCTLCMD;

	ucmd.ictx = ictx;
	ucmd.ubuffer = (void __user *)arg;
	ret = get_user(ucmd.user_size, (u32 __user *)ucmd.ubuffer);
	if (ret)
		return ret;

	op = &viommu_ioctl_ops[nr - IOMMUFD_VIOMMU_CMD_BASE];
	if (op->ioctl_num != cmd)
		return -ENOIOCTLCMD;
	if (ucmd.user_size < op->min_size)
		return -EOPNOTSUPP;

	ucmd.cmd = &buf;
	ret = copy_struct_from_user(ucmd.cmd, op->size, ucmd.ubuffer,
				    ucmd.user_size);
	if (ret)
		return ret;
	return op->execute(&ucmd);
}
