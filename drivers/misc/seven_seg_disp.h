/*
 * Copyright (c) 2016 Google, Inc
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 or later as
 * published by the Free Software Foundation.
 */

#ifndef SEVEN_SEG_DISP_H
#define SEVEN_SEG_DISP_H

#include <linux/device.h>
#include <linux/cdev.h>

#define MAX_DISP_CHAR_SIZE 3

#define DEFAULT_REFRESH_INTERVAL_MS 600

struct seven_seg_disp_dev {
	bool disp_data_valid;
	u16 current_seven_seg_disp_data;
	char seven_seg_disp_data_array[MAX_DISP_CHAR_SIZE];
	struct device parent;
	struct device *dev;
	struct cdev cdev;
	void (*update_seven_seg_data)(struct device *, u16 data);
};

int seven_seg_setup_cdev(struct seven_seg_disp_dev *disp_dev,
	void (*update_disp_data)(struct device *, u16 data));

void seven_seg_rem_cdev(struct seven_seg_disp_dev *disp_dev);

#endif
