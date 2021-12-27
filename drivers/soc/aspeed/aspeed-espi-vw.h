// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) ASPEED Technology Inc.
 * Author: Chia-Wei Wang <chiawei_wang@aspeedtech.com>
 *
 * This header includes the data strcuture and the handler
 * for eSPI virtual wire channel. It is part of Aspeed eSPI
 * slave driver implementation
 */
#ifndef _ASPEED_ESPI_VW_H_
#define _ASPEED_ESPI_VW_H_

#define VW_MDEV_NAME	"aspeed-espi-vw"

struct aspeed_espi_vw {
	int irq;
	int irq_reset;

	struct miscdevice mdev;
	struct aspeed_espi_ctrl *ctrl;
};

static long aspeed_espi_vw_ioctl(struct file *fp, unsigned int cmd,
				    unsigned long arg)
{
	uint32_t val;

	struct aspeed_espi_vw *espi_vw = container_of(
			fp->private_data,
			struct aspeed_espi_vw,
			mdev);
	struct aspeed_espi_ctrl *espi_ctrl = espi_vw->ctrl;

	switch (cmd) {
	case ASPEED_ESPI_VW_GET_GPIO_VAL:
		regmap_read(espi_ctrl->map, ESPI_VW_GPIO_VAL, &val);
		if (put_user(val, (uint32_t __user *)arg))
			return -EFAULT;
		break;

	case ASPEED_ESPI_VW_PUT_GPIO_VAL:
		if (get_user(val, (uint32_t __user *)arg))
			return -EFAULT;
		regmap_write(espi_ctrl->map, ESPI_VW_GPIO_VAL, val);
		break;

	default:
		return -EINVAL;
	};

	return 0;
}

static const struct file_operations aspeed_espi_vw_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = aspeed_espi_vw_ioctl,
};

static void aspeed_espi_vw_event(uint32_t sts, struct aspeed_espi_vw *espi_vw)
{
	uint32_t sysevt_sts;
	struct aspeed_espi_ctrl *espi_ctrl = espi_vw->ctrl;

	regmap_read(espi_ctrl->map, ESPI_INT_STS, &sts);

	if (sts & ESPI_INT_STS_VW_SYSEVT) {
		regmap_read(espi_ctrl->map, ESPI_SYSEVT_INT_STS, &sysevt_sts);

		if (espi_ctrl->version == ESPI_AST2500) {
			if (sysevt_sts & ESPI_SYSEVT_INT_STS_HOST_RST_WARN)
				regmap_update_bits(espi_ctrl->map, ESPI_SYSEVT,
						   ESPI_SYSEVT_HOST_RST_ACK,
						   ESPI_SYSEVT_HOST_RST_ACK);

			if (sysevt_sts & ESPI_SYSEVT_INT_STS_OOB_RST_WARN)
				regmap_update_bits(espi_ctrl->map, ESPI_SYSEVT,
						   ESPI_SYSEVT_OOB_RST_ACK,
						   ESPI_SYSEVT_OOB_RST_ACK);
		}

		regmap_write(espi_ctrl->map, ESPI_SYSEVT_INT_STS, sysevt_sts);
	}

	if (sts & ESPI_INT_STS_VW_SYSEVT1) {
		regmap_read(espi_ctrl->map, ESPI_SYSEVT1_INT_STS, &sysevt_sts);

		if (sysevt_sts & ESPI_SYSEVT1_INT_STS_SUSPEND_WARN)
			regmap_update_bits(espi_ctrl->map, ESPI_SYSEVT1,
					   ESPI_SYSEVT1_SUSPEND_ACK,
					   ESPI_SYSEVT1_SUSPEND_ACK);

		regmap_write(espi_ctrl->map, ESPI_SYSEVT1_INT_STS, sysevt_sts);
	}
}

static void aspeed_espi_vw_enable(struct aspeed_espi_vw *espi_vw)
{
	struct aspeed_espi_ctrl *espi_ctrl = espi_vw->ctrl;

	regmap_write(espi_ctrl->map, ESPI_INT_STS,
		     ESPI_INT_STS_VW_BITS);

	regmap_update_bits(espi_ctrl->map, ESPI_INT_EN,
			   ESPI_INT_EN_VW_BITS,
			   ESPI_INT_EN_VW_BITS);

	regmap_update_bits(espi_ctrl->map, ESPI_CTRL,
			   ESPI_CTRL_VW_SW_RDY,
			   ESPI_CTRL_VW_SW_RDY);
}

static void *aspeed_espi_vw_alloc(struct device *dev, struct aspeed_espi_ctrl *espi_ctrl)
{
	int rc;
	struct aspeed_espi_vw *espi_vw;

	espi_vw = devm_kzalloc(dev, sizeof(*espi_vw), GFP_KERNEL);
	if (!espi_vw)
		return ERR_PTR(-ENOMEM);

	espi_vw->ctrl = espi_ctrl;

	espi_vw->mdev.parent = dev;
	espi_vw->mdev.minor = MISC_DYNAMIC_MINOR;
	espi_vw->mdev.name = devm_kasprintf(dev, GFP_KERNEL, "%s", VW_MDEV_NAME);
	espi_vw->mdev.fops = &aspeed_espi_vw_fops;
	rc = misc_register(&espi_vw->mdev);
	if (rc) {
		dev_err(dev, "cannot register device\n");
		return ERR_PTR(rc);
	}

	aspeed_espi_vw_enable(espi_vw);

	return espi_vw;
}

static void aspeed_espi_vw_free(struct device *dev, struct aspeed_espi_vw *espi_vw)
{
	devm_kfree(dev, espi_vw);
}

#endif
