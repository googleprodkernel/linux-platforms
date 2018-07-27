/*
 * Copyright (C) 2016 Google, Inc
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 or later as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/timer.h>
#include <linux/jiffies.h>
#include <linux/sizes.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/uaccess.h>
#include <linux/mutex.h>
#include <linux/of_platform.h>
#include <linux/gpio/consumer.h>

#include "seven_seg_disp.h"

#define DELAY_INTVL_US 1

#define CLOCK_GPIO_NAME "clock"
#define DATA_GPIO_NAME "data"
#define CLEAR_GPIO_NAME "clear"

struct seven_seg_gpio_info {
	u16 curr_disp_value;
	u16 refresh_interval;
	struct timer_list update_timer;
	struct gpio_desc *clock_gpio;
	struct gpio_desc *data_gpio;
	struct gpio_desc *clear_gpio;
};

static void update_seven_seg_gpio_data(struct device *dev, u16 data)
{
	struct platform_device *pdev;
	struct seven_seg_gpio_info *gpio_info;

	pdev = container_of(dev, struct platform_device, dev);
	if (pdev == NULL) {
		pr_err("invalid NULL platform_device\n");
		return;
	}

	gpio_info = platform_get_drvdata(pdev);
	if (gpio_info == NULL) {
		pr_err("invalid NULL gpio_info\n");
		return;
	}

	gpio_info->curr_disp_value = data;
}

static void send_seven_seg_gpio_data(u16 disp_data,
		struct seven_seg_gpio_info *gpio_info)
{
	int i;

	gpiod_set_value(gpio_info->clear_gpio, 0);
	udelay(DELAY_INTVL_US);
	gpiod_set_value(gpio_info->clear_gpio, 1);
	udelay(DELAY_INTVL_US);

	for (i = 0; i < 16; i++) {
		if (disp_data & 0x01)
			gpiod_set_value(gpio_info->data_gpio, 1);
		else
			gpiod_set_value(gpio_info->data_gpio, 0);

		udelay(DELAY_INTVL_US);

		gpiod_set_value(gpio_info->clock_gpio, 0);
		udelay(DELAY_INTVL_US);
		gpiod_set_value(gpio_info->clock_gpio, 1);
		udelay(DELAY_INTVL_US);

		disp_data >>= 1;
	}
}

static void disp_refresh_timer_handler(struct timer_list *t)
{
	u16 disp_data;
	struct seven_seg_gpio_info *gpio_info =
		from_timer(gpio_info, t, update_timer);
	disp_data = gpio_info->curr_disp_value;

	send_seven_seg_gpio_data(disp_data, gpio_info);
	mod_timer(&gpio_info->update_timer,
		jiffies + msecs_to_jiffies(gpio_info->refresh_interval));
}

static const struct of_device_id of_seven_seg_gpio_match[] = {
		{ .compatible = "seven-seg-gpio-dev" },
		{},
};

MODULE_DEVICE_TABLE(of, of_seven_seg_gpio_match);

static int seven_seg_gpio_probe(struct platform_device *pdev)
{
	u16 interval;
	int result;
	struct seven_seg_gpio_info *gpio_info;
	struct device *dev = &pdev->dev;
	struct seven_seg_disp_dev *disp_dev;

	gpio_info = devm_kzalloc(dev,
			sizeof(struct seven_seg_gpio_info),
			GFP_KERNEL);
	if (gpio_info == NULL)
		return -ENOMEM;

	/* Requesting the clock gpio */
	gpio_info->clock_gpio = devm_gpiod_get(dev, CLOCK_GPIO_NAME,
		GPIOD_OUT_HIGH);
	if (IS_ERR(gpio_info->clock_gpio))
		return PTR_ERR(gpio_info->clock_gpio);

	/* Requesting the data gpio */
	gpio_info->data_gpio = devm_gpiod_get(dev, DATA_GPIO_NAME,
		GPIOD_OUT_HIGH);
	if (IS_ERR(gpio_info->data_gpio))
		return PTR_ERR(gpio_info->data_gpio);

	/* Requesting the clear gpio */
	gpio_info->clear_gpio = devm_gpiod_get(dev, CLEAR_GPIO_NAME,
		GPIOD_OUT_HIGH);
	if (IS_ERR(gpio_info->clear_gpio))
		return PTR_ERR(gpio_info->clear_gpio);

	result = of_property_read_u16(pdev->dev.of_node,
		"refresh-interval-ms", &interval);
	gpio_info->refresh_interval = result ? DEFAULT_REFRESH_INTERVAL_MS :
		interval;

	/* Start timer to update seven segment display every second */
	timer_setup(&gpio_info->update_timer, disp_refresh_timer_handler, 0);
	result = mod_timer(&gpio_info->update_timer,
			jiffies +
			msecs_to_jiffies(gpio_info->refresh_interval));
	if (result)
		return result;

	gpio_info->curr_disp_value = 0;

	platform_set_drvdata(pdev, gpio_info);

	disp_dev = devm_kzalloc(dev, sizeof(struct seven_seg_disp_dev),
				GFP_KERNEL);
	disp_dev->parent = *dev;
	seven_seg_setup_cdev(disp_dev, &update_seven_seg_gpio_data);
	return 0;
}

static int seven_seg_gpio_remove(struct platform_device *pdev)
{
	struct seven_seg_gpio_info *gpio_info = platform_get_drvdata(pdev);
	struct seven_seg_disp_dev *disp_dev =
				container_of(&pdev->dev,
				struct seven_seg_disp_dev, parent);
	seven_seg_rem_cdev(disp_dev);
	del_timer_sync(&gpio_info->update_timer);
	platform_set_drvdata(pdev, NULL);
	return 0;
}

static struct platform_driver seven_seg_gpio_driver = {
	.probe		= seven_seg_gpio_probe,
	.remove		= seven_seg_gpio_remove,
	.driver		= {
		.name	= "seven-seg-gpio",
		.of_match_table = of_seven_seg_gpio_match,
	},
};

module_platform_driver(seven_seg_gpio_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Jaghathiswari Rankappagounder Natarajan <jaghu@google.com>");
MODULE_DESCRIPTION("Seven segment display driver using GPIO config");
