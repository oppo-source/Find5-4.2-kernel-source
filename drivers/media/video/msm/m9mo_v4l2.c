/* Copyright (c) 2012, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include "msm_sensor.h"
#include "m9mo_v4l2.h"
#include <mach/vreg.h>
#include <linux/spi/spi.h>
#include <linux/module.h>
#include "msm_camera_i2c_mux.h"
#include "msm.h"
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/proc_fs.h> 
#include <linux/pcb_version.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/stat.h>
#include <linux/fs.h>
#include <asm/unistd.h>
#include <asm/uaccess.h>
#include <linux/types.h>
#include <linux/ioctl.h>

#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/gpio_keys.h>
#include <linux/gpio.h>
#include <linux/of_platform.h>
#include <linux/of_gpio.h>
#include <linux/spinlock.h>

#define M9MO_FW_FILE	"/data/m9mo_fw/RS_M9MO.bin"
#define READ_FW_VERSION_SIZE		(4096)
#define M9MO_BUF_SIZE				(512*1024)
#define MAX_TRANSFER_CNT			(M9MO_FW_SIZE/M9MO_BUF_SIZE)
#define MAX_CMD_INDEX				(16)

#define MR_ROTATE_GPIO				(43)

static char m9mo_proc_data[256] = {0x00};
static char m9mo_cmd[16] = {0x00};
static char m9mo_category = 0x00;
static char m9mo_byte = 0x00;
static char m9mo_read_data[16] = {0x00};

static char m9mo_version[2];
static bool update_FW_by_file = false;
static bool now_is_boot = true;

#define SENSOR_NAME "m9mo"
#define PLATFORM_DRIVER_NAME "msm_camera_m9mo"
#define m9mo_obj m9mo_##obj
static struct spi_device* m9mo_spi_dev = NULL;
#define m9m0_bug printk
//#define DOWNLOAD_I2C_MODE
static volatile bool FW_download = true;
static volatile bool need_check_FW = true;
static volatile bool INT_installed = false;
static volatile bool camera_work = false;
static volatile bool g_bCapture = false;
static int queue_id = -1;
//static volatile bool g_bCapture_ready_m9mo = false;
static volatile bool g_bCapture_raw = false;
static struct work_struct m9mo_status_work;
static struct workqueue_struct * m9mo_status_workqueue = NULL;
static struct work_struct caf_work;
static struct workqueue_struct * caf_workqueue = NULL;
static volatile int flash_state = 0;

static volatile bool g_bHDREnable = false;
static volatile bool g_bSlowShutter = false;
static volatile bool g_bRotate = false;
static volatile bool led_is_on = false;

enum m9mo_focus_state{
	FOCUSED_SUCCESS = 0,
	FOCUSING,
	FOCUSED_FAILED,
	FOCUS_MAX,
};
enum m9mo_focus_state current_status,last_status;

struct msm_sensor_ctrl_t* CTRL; 
void m9mo_status_work_callback(struct work_struct *work);
void caf_work_callback(struct work_struct *work);
static irqreturn_t m9mo_irq(int irq, void *dev_id);
int32_t m9mo_power_up(struct msm_sensor_ctrl_t *s_ctrl);
int32_t m9mo_power_down(struct msm_sensor_ctrl_t *s_ctrl);
static int32_t m9mo_change_to_param_set_mode(struct msm_sensor_ctrl_t *s_ctrl);
static int32_t m9mo_set_slow_shutter(struct msm_sensor_ctrl_t *s_ctrl, u_int32_t slow_shutter);
static void m9mo_set_flash_mode(struct msm_sensor_ctrl_t *s_ctrl, int flash_mode);
static struct msm_cam_clk_info cam_8960_clk_info[] = {
	{"cam_clk", MSM_SENSOR_MCLK_24HZ},
};
DEFINE_MUTEX(m9mo_mut);
static struct msm_sensor_ctrl_t m9mo_s_ctrl;
static struct v4l2_subdev_info m9mo_subdev_info[] = {
	{
		.code   = V4L2_MBUS_FMT_YUYV8_2X8,
		.colorspace = V4L2_COLORSPACE_JPEG,
		.fmt    = 1,
		.order    = 0,
	},
};


static struct msm_sensor_output_info_t m9mo_dimensions[] = {
	{
		/* full-size 10fps */
		.x_output = 4128, /* 4208 */
		.y_output = 3096, /* 3120 */
		.line_length_pclk = 4128, /* 4572 */
		.frame_length_lines = 3096, /* 3142 */
		.vt_pixel_clk = 320000000,//316800000
		.op_pixel_clk = 320000000,//316800000
		.binning_factor = 1,
	},
	{
		/* 60 fps  or 24fps for HDR rec */
	    .x_output = 2048, 
		.y_output = 3153, 
		.line_length_pclk = 2048, /* 4572 */
		.frame_length_lines = 3153, /* 1582 */
		.vt_pixel_clk = 320000000,
		.op_pixel_clk = 320000000,
		.binning_factor = 1,
	},
	{
		/* 24fps for FullHD preview */
	    .x_output = 1920, 
		.y_output = 1080, 
		.line_length_pclk = 1920,
		.frame_length_lines = 1080,
		.vt_pixel_clk = 250000000,
		.op_pixel_clk = 250000000,
		.binning_factor = 1,
	},
	{
		#if 1
		/*FullHD 60fps for video*/
	    .x_output = 1920, 
		.y_output = 1080, 
		.line_length_pclk = 1920,
		.frame_length_lines = 1080,
		.vt_pixel_clk = 320000000,
		.op_pixel_clk = 320000000,
		.binning_factor = 1,
		#else
		/*WVGA 100fps for video*/
		.x_output = 800, 
		.y_output = 480, 
		.line_length_pclk = 800,
		.frame_length_lines = 480,
		.vt_pixel_clk = 320000000,
		.op_pixel_clk = 320000000,
		.binning_factor = 1,
		#endif
	},
	{
		/* 30fps for HDR rec */
	    .x_output = 1920, 
		.y_output = 1080, 
		.line_length_pclk = 1920,
		.frame_length_lines = 1080,
		.vt_pixel_clk = 320000000,
		.op_pixel_clk = 320000000,
		.binning_factor = 1,
	},
	{
		/* 120 fps video 480P */
		.x_output = 640, /* 640 */
		.y_output = 480, /* 480 */
		.line_length_pclk = 640,
		.frame_length_lines = 480,
		.vt_pixel_clk = 256000000,
		.op_pixel_clk = 256000000,
		.binning_factor = 1,
	},	
	{
		/* 2064x1548 panorama preview */
	    .x_output = 2064, 
		.y_output = 1548, 
		.line_length_pclk = 2064,
		.frame_length_lines = 1548,
		.vt_pixel_clk = 250000000,
		.op_pixel_clk = 250000000,
		.binning_factor = 1,
	},	
};

static struct msm_sensor_id_info_t m9mo_id_info = {
	.sensor_id_reg_addr = 0x0000,
	.sensor_id = 0000,
};


static const struct i2c_device_id m9mo_i2c_id[] = {
	{SENSOR_NAME, (kernel_ulong_t)&m9mo_s_ctrl},
	{ }
};

static struct i2c_driver m9mo_i2c_driver = {
	.id_table = m9mo_i2c_id,
	.probe  = msm_sensor_i2c_probe,
	.driver = {
		.name = SENSOR_NAME,
	},
};

static struct msm_camera_i2c_client m9mo_sensor_i2c_client = {
	.addr_type = MSM_CAMERA_I2C_WORD_ADDR,
};
struct m9mo_spi_trans {
	struct spi_transfer	spi_xfer;
	struct spi_message spi_msg;
};

int32_t m9mo_sensor_enable_i2c_mux(struct msm_camera_i2c_conf *i2c_conf)
{
	struct v4l2_subdev *i2c_mux_sd =
		dev_get_drvdata(&i2c_conf->mux_dev->dev);
	v4l2_subdev_call(i2c_mux_sd, core, ioctl,
		VIDIOC_MSM_I2C_MUX_INIT, NULL);
	v4l2_subdev_call(i2c_mux_sd, core, ioctl,
		VIDIOC_MSM_I2C_MUX_CFG, (void *)&i2c_conf->i2c_mux_mode);
	return 0;
}

int32_t m9mo_sensor_disable_i2c_mux(struct msm_camera_i2c_conf *i2c_conf)
{
	struct v4l2_subdev *i2c_mux_sd =
		dev_get_drvdata(&i2c_conf->mux_dev->dev);
	v4l2_subdev_call(i2c_mux_sd, core, ioctl,
				VIDIOC_MSM_I2C_MUX_RELEASE, NULL);
	return 0;
}

static void m9mo_set_mirror_flip(struct msm_sensor_ctrl_t *s_ctrl)
{
	char sensor_reverse_h_v_cmd[8] = {0x08,0x02,0x02,0x05,0x01,0x01,0x01,0x01};
	int gpio_value = 0;
	
	/* sensor output reversely H/V */
	gpio_value = gpio_get_value(MR_ROTATE_GPIO);
	if (g_bRotate)
	{
		m9mo_change_to_param_set_mode(s_ctrl);
		g_bRotate = false;
	}
	
	if (1 == gpio_value)
	{
		sensor_reverse_h_v_cmd[4] = 0x01;
		sensor_reverse_h_v_cmd[5] = 0x01;
		sensor_reverse_h_v_cmd[6] = 0x01;
		sensor_reverse_h_v_cmd[7] = 0x01;
	}
	else
	{
		sensor_reverse_h_v_cmd[4] = 0x01;
		sensor_reverse_h_v_cmd[5] = 0x00;
		sensor_reverse_h_v_cmd[6] = 0x01;
		sensor_reverse_h_v_cmd[7] = 0x00;
	}
	msm_camera_i2c_txdata(s_ctrl->sensor_i2c_client,sensor_reverse_h_v_cmd, 8);
}

static void normal_preview_output(struct msm_sensor_ctrl_t *s_ctrl)
{
	char E_cmd[5] = {0x05,0x02,0x01,0x01,0x28};

	//disable zsl
	E_cmd[2] = 0x01;
	E_cmd[3] = 0x6E;
	E_cmd[4] = 0x00;
	msm_camera_i2c_txdata(s_ctrl->sensor_i2c_client,E_cmd,5);

	//disable sensor HDR
	E_cmd[2] = 0x01;
	E_cmd[3] = 0x03;
	E_cmd[4] = 0x00;
	msm_camera_i2c_txdata(s_ctrl->sensor_i2c_client,E_cmd,5);
	
	//set monitor size
	E_cmd[2] = 0x01;
	E_cmd[3] = 0x01;
	E_cmd[4] = 0x28;
	msm_camera_i2c_txdata(s_ctrl->sensor_i2c_client,E_cmd,5);
	
	m9mo_set_mirror_flip(s_ctrl);

	//enable interrupt
	E_cmd[2] = 0x00;
	E_cmd[3] = 0x10;
	E_cmd[4] = 0x01;
	msm_camera_i2c_txdata(s_ctrl->sensor_i2c_client,E_cmd,5);

	//change to monitor mode
	E_cmd[2] = 0x00;
	E_cmd[3] = 0x0b;
	E_cmd[4] = 0x02;
	msm_camera_i2c_txdata(s_ctrl->sensor_i2c_client,E_cmd,5);
	g_bCapture = false;
	m9m0_bug("%s E\n", __func__);
}

//---------------------------------------------------------
/*for YUV 422 out put dimension:1920X1080 no blanks*/
static void ZSL_preview_output(struct msm_sensor_ctrl_t *s_ctrl)
{
	char E_cmd[5] = {0x05,0x02,0x01,0x01,0x28};

	//enable zsl
	E_cmd[2] = 0x01;
	E_cmd[3] = 0x6E;
	E_cmd[4] = 0x01;
	msm_camera_i2c_txdata(s_ctrl->sensor_i2c_client,E_cmd,5);

	//disable sensor HDR
	E_cmd[2] = 0x01;
	E_cmd[3] = 0x03;
	E_cmd[4] = 0x00;
	msm_camera_i2c_txdata(s_ctrl->sensor_i2c_client,E_cmd,5);
	
	//set monitor size
	E_cmd[2] = 0x01;
	E_cmd[3] = 0x01;
	E_cmd[4] = 0x28;
	msm_camera_i2c_txdata(s_ctrl->sensor_i2c_client,E_cmd,5);
	
	m9mo_set_mirror_flip(s_ctrl);

	//enable interrupt
	E_cmd[2] = 0x00;
	E_cmd[3] = 0x10;
	E_cmd[4] = 0x01;
	msm_camera_i2c_txdata(s_ctrl->sensor_i2c_client,E_cmd,5);

	//change to monitor mode
	E_cmd[2] = 0x00;
	E_cmd[3] = 0x0b;
	E_cmd[4] = 0x02;
	msm_camera_i2c_txdata(s_ctrl->sensor_i2c_client,E_cmd,5);
	g_bCapture = false;
	m9m0_bug("%s E\n", __func__);
}

static void ZSL_panorama_preview(struct msm_sensor_ctrl_t *s_ctrl)
{
	char E_cmd[5] = {0x05,0x02,0x01,0x01,0x28};

	//disable zsl
	E_cmd[2] = 0x01;
	E_cmd[3] = 0x6E;
	E_cmd[4] = 0x00;
	msm_camera_i2c_txdata(s_ctrl->sensor_i2c_client,E_cmd,5);

	//disable sensor HDR
	E_cmd[2] = 0x01;
	E_cmd[3] = 0x03;
	E_cmd[4] = 0x00;
	msm_camera_i2c_txdata(s_ctrl->sensor_i2c_client,E_cmd,5);
	
	//set monitor size
	E_cmd[2] = 0x01;
	E_cmd[3] = 0x01;
	E_cmd[4] = 0x3D;
	msm_camera_i2c_txdata(s_ctrl->sensor_i2c_client,E_cmd,5);
	
	m9mo_set_mirror_flip(s_ctrl);

	//enable interrupt
	E_cmd[2] = 0x00;
	E_cmd[3] = 0x10;
	E_cmd[4] = 0x01;
	msm_camera_i2c_txdata(s_ctrl->sensor_i2c_client,E_cmd,5);

	//change to monitor mode
	E_cmd[2] = 0x00;
	E_cmd[3] = 0x0b;
	E_cmd[4] = 0x02;
	msm_camera_i2c_txdata(s_ctrl->sensor_i2c_client,E_cmd,5);
	g_bCapture = false;
	m9m0_bug("%s E\n", __func__);
}

static void ZSL_capture(struct msm_sensor_ctrl_t *s_ctrl)
{
	char E_cmd[5] = {0x05,0x02,0x00,0x10,0x08};
	char resp[5] = {0x05,0x01,0x0A,0x02,0x01};
	int poll_cnt = 0;
	
	//Enable capture INT
	if(g_bCapture)
		return;
	//check if at focusing
	while (poll_cnt++ < 100)
	{
		msm_vendor_i2c_rxdata(s_ctrl->sensor_i2c_client,resp,5,2);
		if (resp[1] != 0x01)
		{
			break;
		}
		msleep(33);
	}
	
	//set capture yuv output
	E_cmd[2] = 0x0B;
	E_cmd[3] = 0x00;
	E_cmd[4] = 0x00;
	msm_camera_i2c_txdata(s_ctrl->sensor_i2c_client,E_cmd,5);

	//set capture mode normal(single capture)
	E_cmd[2] = 0x0C;
	E_cmd[3] = 0x00;
	E_cmd[4] = 0x00;
	msm_camera_i2c_txdata(s_ctrl->sensor_i2c_client,E_cmd,5);

	//Enable capture INT
	E_cmd[2] = 0x00;
	E_cmd[3] = 0x10;
	E_cmd[4] = 0x08;
	msm_camera_i2c_txdata(s_ctrl->sensor_i2c_client,E_cmd,5);
	
	//Change to capture mode
	E_cmd[2] = 0x00;
	E_cmd[3] = 0x0B;
	E_cmd[4] = 0x03;
	msm_camera_i2c_txdata(s_ctrl->sensor_i2c_client,E_cmd,5);
	m9m0_bug("%s E\n", __func__);
	g_bCapture = true;
}
static void raw_capture(struct msm_sensor_ctrl_t *s_ctrl)
{
	char E_cmd[5] = {0x05,0x02,0x0B,0x79,0x01};
	char resp[5] = {0x05,0x01,0x0A,0x02,0x01};
	int poll_cnt = 0;
	
	//Enable raw-jpeg capture
	if(g_bCapture)
		return;
		
	//check if at focusing
	while (poll_cnt++ < 100)
	{
		msm_vendor_i2c_rxdata(s_ctrl->sensor_i2c_client,resp,5,2);
		if (resp[1] != 0x01)
		{
			break;
		}
		msleep(33);
	}
	
	//Enable raw-jpeg capture
	msm_camera_i2c_txdata(s_ctrl->sensor_i2c_client,E_cmd,5);

	E_cmd[2] = 0x00;
	E_cmd[3] = 0x10;
	E_cmd[4] = 0x08;
	//Enable capture INT
	msm_camera_i2c_txdata(s_ctrl->sensor_i2c_client,E_cmd,5);
	
	//Change to capture mode
	E_cmd[2] = 0x00;
	E_cmd[3] = 0x0B;
	E_cmd[4] = 0x03;
	msm_camera_i2c_txdata(s_ctrl->sensor_i2c_client,E_cmd,5);
	m9m0_bug("%s E\n", __func__);
	g_bCapture = true;
}

/*for jpeg 422 out put dimension:1920X1080 no blanks*/
static void ZSL_raw_output(struct msm_sensor_ctrl_t *s_ctrl)
{	
	char E_cmd[5] = {0x05,0x02,0x0B,0x00,0x06};
	m9m0_bug("%s E\n", __func__);
	/* polling data transfer completed INT */
	/* polling capture mode INT */
	/* Select frame main */
	msm_camera_i2c_txdata(s_ctrl->sensor_i2c_client,E_cmd,5);
	/* Setup JPEG max size */
	E_cmd[2] = 0x0B;
	E_cmd[3] = 0x01;
	E_cmd[4] = 0x2C;
	msm_camera_i2c_txdata(s_ctrl->sensor_i2c_client,E_cmd,8);
	/* Setup JPEG min size */
	E_cmd[2] = 0x0C;
	E_cmd[3] = 0x06;
	E_cmd[4] = 0x01;
	msm_camera_i2c_txdata(s_ctrl->sensor_i2c_client,E_cmd,8);
	/* Select main image format YUV422 */
	E_cmd[2] = 0x0C;
	E_cmd[3] = 0x09;
	E_cmd[4] = 0x01;
	msm_camera_i2c_txdata(s_ctrl->sensor_i2c_client,E_cmd,5);
}
/*for YUV 422 out put dimension:4128X3096 no blanks*/
static void ZSL_YUV_output(struct msm_sensor_ctrl_t *s_ctrl)
{ 
	char E_cmd[8] = {0x05,0x02,0x00,0x10,0x88,0x00,0x00,0x00};
	
	//Select frame main
	E_cmd[2] = 0x0C;
	E_cmd[3] = 0x06;
	E_cmd[4] = 0x01;
	msm_camera_i2c_txdata(s_ctrl->sensor_i2c_client,E_cmd,5);
	
	/* Select main image size 4128x3096(13M) */
	E_cmd[0] = 0x05;
	E_cmd[2] = 0x0B;
	E_cmd[3] = 0x01;
	E_cmd[4] = 0x2C;
	msm_camera_i2c_txdata(s_ctrl->sensor_i2c_client,E_cmd,5);
	
	/* Start transfer */
	E_cmd[0] = 0x05;
	E_cmd[2] = 0x0C;
	E_cmd[3] = 0x09;
	E_cmd[4] = 0x01;
	msm_camera_i2c_txdata(s_ctrl->sensor_i2c_client,E_cmd,5);
	m9m0_bug("%s E\n", __func__);
}

static void ZSL_HDR_capture(struct msm_sensor_ctrl_t *s_ctrl)
{
	char E_cmd[5] = {0x05,0x02,0x00,0x10,0x10};
	
	//Enable framesync INT
	msm_camera_i2c_txdata(s_ctrl->sensor_i2c_client,E_cmd,5);
	
	//set YUVOUT_MAIN
	E_cmd[2] = 0x0B;
	E_cmd[3] = 0x00;
	E_cmd[4] = 0x00;
	msm_camera_i2c_txdata(s_ctrl->sensor_i2c_client,E_cmd,5);

	//set capture mode(Auto Bracket capture)
	E_cmd[2] = 0x0C;
	E_cmd[3] = 0x00;
	E_cmd[4] = 0x06;
	msm_camera_i2c_txdata(s_ctrl->sensor_i2c_client,E_cmd,5);

	//set EV offset
	E_cmd[2] = 0x03;
	E_cmd[3] = 0x20;
	E_cmd[4] = 0x32;
	msm_camera_i2c_txdata(s_ctrl->sensor_i2c_client,E_cmd,5);
	
	//main image size setting
	E_cmd[2] = 0x0B;
	E_cmd[3] = 0x01;
	E_cmd[4] = 0x2C;
	msm_camera_i2c_txdata(s_ctrl->sensor_i2c_client,E_cmd,5);
	
	//start multi capture
	E_cmd[2] = 0x00;
	E_cmd[3] = 0x0B;
	E_cmd[4] = 0x03;
	msm_camera_i2c_txdata(s_ctrl->sensor_i2c_client,E_cmd,5);
	g_bCapture = false;
	m9m0_bug("%s E\n", __func__);
}
/*for jpeg out put dimension:1920X1080 no blanks*/
static void ZSL_HDR_yuv_output(struct msm_sensor_ctrl_t *s_ctrl)
{
	//char E_cmd[5] = {0x05,0x02,0x0C,0x09,0x01};

	//msm_camera_i2c_txdata(s_ctrl->sensor_i2c_client,E_cmd,5);
	
	m9m0_bug("%s E\n", __func__);
}
/*for YUV 422 24fps out put dimension:1920X1080 no blanks*/
static void HDR_video_output(struct msm_sensor_ctrl_t *s_ctrl)
{
	char E_cmd[5] = {0x05,0x02,0x00,0x0B,0x01};

	/*disable zsl*/
	E_cmd[2] = 0x01;
	E_cmd[3] = 0x6E;
	E_cmd[4] = 0x00;
	msm_camera_i2c_txdata(s_ctrl->sensor_i2c_client,E_cmd,5);

	/*sensor hdr on*/
	E_cmd[2] = 0x01;
	E_cmd[3] = 0x03;
	E_cmd[4] = 0x01;
	msm_camera_i2c_txdata(s_ctrl->sensor_i2c_client,E_cmd,5);

	/*set outsize FullHD 30fps*/
	E_cmd[2] = 0x01;
	E_cmd[3] = 0x01;
	E_cmd[4] = 0x28;
	msm_camera_i2c_txdata(s_ctrl->sensor_i2c_client,E_cmd,5);

	m9mo_set_mirror_flip(s_ctrl);

	/* enable YUV-output interrupt */
	E_cmd[2] = 0x00;
	E_cmd[3] = 0x10;
	E_cmd[4] = 0x01;
	msm_camera_i2c_txdata(s_ctrl->sensor_i2c_client,E_cmd,5);

	/* change to monitor mode */
	E_cmd[2] = 0x00;
	E_cmd[3] = 0x0b;
	E_cmd[4] = 0x02;
	msm_camera_i2c_txdata(s_ctrl->sensor_i2c_client,E_cmd,5);
	m9m0_bug("%s E\n", __func__);
}
/*for YUV 422 60fps out put dimension:1920X1080 no blanks*/
static void video_output(struct msm_sensor_ctrl_t *s_ctrl)
{
	char E_cmd[5] = {0x05,0x02,0x00,0x0B,0x01};
	
	//disable ZSL
	E_cmd[2] = 0x01;
	E_cmd[3] = 0x6E;
	E_cmd[4] = 0x00;
	msm_camera_i2c_txdata(s_ctrl->sensor_i2c_client,E_cmd,5);

	//disable sensor HDR
	E_cmd[2] = 0x01;
	E_cmd[3] = 0x03;
	E_cmd[4] = 0x00;
	msm_camera_i2c_txdata(s_ctrl->sensor_i2c_client,E_cmd,5);

	//set outsize 1920x1080 FullHD 60fps
	E_cmd[2] = 0x01;
	E_cmd[3] = 0x01;
	E_cmd[4] = 0x2B;
	msm_camera_i2c_txdata(s_ctrl->sensor_i2c_client,E_cmd,5);

	m9mo_set_mirror_flip(s_ctrl);

	/* enable YUV-output interrupt */
	E_cmd[2] = 0x00;
	E_cmd[3] = 0x10;
	E_cmd[4] = 0x01;
	msm_camera_i2c_txdata(s_ctrl->sensor_i2c_client,E_cmd,5);

	/* change to monitor mode */
	E_cmd[2] = 0x00;
	E_cmd[3] = 0x0b;
	E_cmd[4] = 0x02;
	msm_camera_i2c_txdata(s_ctrl->sensor_i2c_client,E_cmd,5);
	
	m9m0_bug("%s E\n", __func__);
}
/*for YUV 422 120fps out put dimension:800X480 no blanks*/
static void WVGA_video_output(struct msm_sensor_ctrl_t *s_ctrl)
{
	char E_cmd[5] = {0x05,0x02,0x01,0x01,0x3B};

	//disable ZSL
	E_cmd[2] = 0x01;
	E_cmd[3] = 0x6E;
	E_cmd[4] = 0x00;
	msm_camera_i2c_txdata(s_ctrl->sensor_i2c_client,E_cmd,5);

	//disable sensor HDR
	E_cmd[2] = 0x01;
	E_cmd[3] = 0x03;
	E_cmd[4] = 0x00;
	msm_camera_i2c_txdata(s_ctrl->sensor_i2c_client,E_cmd,5);
	
	m9mo_set_mirror_flip(s_ctrl);

	/* set outsize 800x480 100fps */
	E_cmd[2] = 0x01;
	E_cmd[3] = 0x01;
	E_cmd[4] = 0x3B;
	msm_camera_i2c_txdata(s_ctrl->sensor_i2c_client,E_cmd,5);

	/* enable YUV-output interrupt */
	E_cmd[2] = 0x00;
	E_cmd[3] = 0x10;
	E_cmd[4] = 0x01;
	msm_camera_i2c_txdata(s_ctrl->sensor_i2c_client,E_cmd,5);

	/* change to monitor mode */
	E_cmd[2] = 0x00;
	E_cmd[3] = 0x0b;
	E_cmd[4] = 0x02;
	msm_camera_i2c_txdata(s_ctrl->sensor_i2c_client,E_cmd,5);
	m9m0_bug("%s E\n", __func__);
}

struct m9mo_action_struct{
	void (*init)(struct msm_sensor_ctrl_t *s_ctrl);
	void (*start)(struct msm_sensor_ctrl_t *s_ctrl);
	void (*output)(struct msm_sensor_ctrl_t *s_ctrl);
	void (*stop)(struct msm_sensor_ctrl_t *s_ctrl);
};
struct m9mo_action_struct m9mo_action[9] = {
	{NULL,ZSL_capture,ZSL_YUV_output,NULL},/*SENSOR_MODE_SNAPSHOT*/
    {NULL,raw_capture,ZSL_raw_output,NULL,},/*SENSOR_MODE_RAW_SNAPSHOT*/
    {NULL,ZSL_preview_output,NULL,NULL},/*SENSOR_MODE_PREVIEW*/
    {NULL,video_output,NULL,NULL},/*SENSOR_MODE_VIDEO*/
    {NULL,HDR_video_output,NULL,NULL},/*SENSOR_MODE_VIDEO_HDR*/
    {NULL,WVGA_video_output,NULL,NULL},/*SENSOR_MODE_HFR_150FPS*/
    {NULL,ZSL_panorama_preview,NULL,NULL},/*SENSOR_MODE_ZSL*/
    {NULL,ZSL_HDR_capture,ZSL_HDR_yuv_output,NULL},
	{NULL,NULL,NULL,NULL},
};

static void focus_done(struct msm_sensor_ctrl_t *s_ctrl)
{
	//m9m0_bug("%s:last_status:%d - current_status:%d\n", __func__,last_status,current_status);
	if(last_status != current_status)
	{
		int rc = 0;
		struct msm_ctrl_cmd ctrlcmd;
		ctrlcmd.type = MSM_V4L2_VENDOR_CMD;
		ctrlcmd.timeout_ms = 2000;
		ctrlcmd.length = 0;
		ctrlcmd.value = NULL;
		ctrlcmd.vnode_id = 0;
		ctrlcmd.queue_idx = queue_id >= 0 ?queue_id:1;
		ctrlcmd.config_ident = current_status;
		rc = msm_server_send_ctrl(&ctrlcmd, MSM_CAM_RESP_V4L2);
		m9m0_bug("%s:msm_server_send_ctrl!current_status[%d] rc:%d\n", __func__,current_status,rc);
	    last_status = current_status;
	}
}

void m9mo_status_work_callback(struct work_struct *work)
{
  
    char  resp[5];
	struct msm_sensor_ctrl_t* s_ctrl = CTRL;
	
	resp[0]= 0x05;
	resp[1]= 0x01;
	resp[2]= 0x00;
	resp[3]= 0x1c;
	resp[4]= 0x01;
	msm_vendor_i2c_rxdata(s_ctrl->sensor_i2c_client,resp,5,2);
	m9m0_bug("%s: resp[1] = {0x%x}\n", __func__, resp[1]);
	if(resp[1] == 0x08 && g_bCapture)
	{
		if(flash_state)
		{
			struct msm_ctrl_cmd ctrlcmd;
			ctrlcmd.type = MSM_V4L2_VENDOR_CMD;
			ctrlcmd.timeout_ms = 2000;
			ctrlcmd.length = 0;
			ctrlcmd.value = NULL;
			ctrlcmd.vnode_id = 0;
			ctrlcmd.queue_idx = queue_id >= 0 ?queue_id:1;
			ctrlcmd.config_ident = 0x08;
	        msm_server_send_ctrl(&ctrlcmd, MSM_CAM_RESP_V4L2);
			if(g_bCapture_raw)
			{
			  msleep(300);
			}else{
			  msleep(30);
			}
		}
		if(m9mo_action[s_ctrl->curr_res].output)
		{
		  m9mo_action[s_ctrl->curr_res].output(s_ctrl);
		}else{
			if(g_bCapture_raw){
				ZSL_raw_output(s_ctrl);
			}else{
			    ZSL_YUV_output(s_ctrl);
			}
		}
		//g_bCapture = false;
    }
	else if (0x10 == resp[1] && g_bHDREnable)
	{
		m9m0_bug("HDR Capture : Frame Sync Coming \r\n");
		if(m9mo_action[7].output)
		{
			m9mo_action[7].output(s_ctrl);
		}
	}
	enable_irq(s_ctrl->sensor_i2c_client->client->irq);
}
void caf_work_callback(struct work_struct *work)
{ 
	char  resp[5];
	struct msm_sensor_ctrl_t* s_ctrl = CTRL;
	
	resp[0]= 0x05;
	resp[1]= 0x01;
	resp[2]= 0x0A;
	resp[3]= 0x03;
	resp[4]= 0x01;
	msm_vendor_i2c_rxdata(s_ctrl->sensor_i2c_client,resp,5,2);
	
	if(resp[1] == 0)
		current_status = FOCUSING;
	
	if((resp[1] == 1)||(resp[1] == 3))
		current_status = FOCUSED_SUCCESS;
	
	if(resp[1] == 2)
		current_status = FOCUSED_FAILED;
	
	focus_done(s_ctrl);
	//m9m0_bug("%s:resp[1]:%d\n", __func__,resp[1]);
}

//---------------------------------------------------------
static irqreturn_t m9mo_irq(int irq, void *dev_id)
{
	struct msm_sensor_ctrl_t *s_ctrl = dev_id;
	
	disable_irq_nosync(s_ctrl->sensor_i2c_client->client->irq);
	queue_work(m9mo_status_workqueue, &m9mo_status_work);
	
	return IRQ_HANDLED;
}

/*---- add by liubin for read M9MO FW from system file start ----*/

static int m9mo_read_FW_from_file(char *pBuf, unsigned long nSize, loff_t position)
{
	struct file *filep = NULL;
	mm_segment_t old_fs;
	
	ssize_t result;

	m9m0_bug("[%s] FW file[%s] \r\n", __func__, M9MO_FW_FILE);
	filep = filp_open(M9MO_FW_FILE, O_RDONLY, 0600);
	if (IS_ERR(filep))
	{
		m9m0_bug("Open file [%s] fail!  errno[%ld] \r\n", M9MO_FW_FILE, IS_ERR(filep));
		return -EIO;
	}

	old_fs=get_fs();
 	set_fs(KERNEL_DS);

	filep->f_pos = position;
	result = vfs_read(filep, pBuf, nSize, &filep->f_pos);
	if (result >= 0)
	{
		m9m0_bug("---get the camera firmware form file succeed !! \r\n");
	}
	else
	{
		m9m0_bug("---read the file failed!! \r\n");
	}
	
	set_fs(old_fs);
	
	filp_close(filep, NULL);

	return 0;
	
}

static int m9mo_get_FW_version_from_file(void)
{
	int nRet = -1;
	
	char *pVerBuf = kmalloc(READ_FW_VERSION_SIZE, GFP_KERNEL);
	if (NULL == pVerBuf)
	{
		m9m0_bug("Allocate pVerBuf FAIL \r\n");
		return -1;
	}
	m9m0_bug("Allocate pVerBuf Success \r\n");

	nRet = m9mo_read_FW_from_file(pVerBuf, READ_FW_VERSION_SIZE, 0);
	if (0 != nRet)
	{
		printk("m9mo_read_FW_from_file fail \r\n");
		return -1;
	}

	m9mo_version[0] = pVerBuf[24];
	m9mo_version[1] = pVerBuf[25];

	if (NULL != pVerBuf)
	{
		kfree(pVerBuf);
	}
	
	return 0;
}

static int m9mo_update_FW_by_file(void)
{
	struct spi_transfer transfer[MAX_TRANSFER_CNT];
	struct spi_message message;
	char *m9mo_fw_buf[MAX_TRANSFER_CNT] = {NULL};
	loff_t pos = 0;
	int transfer_cnt = 0;
	int nRet = -1;

	memset(transfer, 0, sizeof(transfer));
	spi_message_init(&message);
	
	for (transfer_cnt=0; transfer_cnt<MAX_TRANSFER_CNT; transfer_cnt++)
	{
		//allocate buffer
		m9mo_fw_buf[transfer_cnt] = kmalloc(M9MO_BUF_SIZE, GFP_KERNEL);
		if (NULL == m9mo_fw_buf[transfer_cnt])
		{
			m9m0_bug("Allocate m9mo_fw_buf FAIL \r\n");
			nRet = -1;
			goto free_buffer;
		}
		m9m0_bug("Allocate m9mo_fw_buf SUCCESS \r\n");
		
		pos = transfer_cnt * M9MO_BUF_SIZE;
		nRet = m9mo_read_FW_from_file(m9mo_fw_buf[transfer_cnt], M9MO_BUF_SIZE, (loff_t)pos);
		if (0 == nRet)
		{
			m9m0_bug("[%s]---m9mo_read_FW_from_file OK!---pos[%ld] \r\n", __func__, (long)pos);			
			transfer[transfer_cnt].tx_buf = m9mo_fw_buf[transfer_cnt];
			transfer[transfer_cnt].len = M9MO_BUF_SIZE;
			transfer[transfer_cnt].bits_per_word = 8;
			transfer[transfer_cnt].speed_hz = 10000000;
			spi_message_add_tail(&transfer[transfer_cnt], &message);
		}
		else
		{
			m9m0_bug("[%s]---m9mo_read_FW_from_file Fail \r\n", __func__);
			if (NULL != m9mo_fw_buf[transfer_cnt])
			{
				kfree(m9mo_fw_buf[transfer_cnt]);
			}
			goto free_buffer;
		}
	}

	m9mo_spi_dev->mode = SPI_MODE_0;
	spi_sync(m9mo_spi_dev, &message);
	
free_buffer:
	//free allocate buffer
	for (transfer_cnt=0; transfer_cnt<MAX_TRANSFER_CNT; transfer_cnt++)
	{
		if (NULL != m9mo_fw_buf[transfer_cnt])
		{
			kfree(m9mo_fw_buf[transfer_cnt]);
		}
	}

	return nRet;
}
/*---- add by liubin for read M9MO FW from system file end ----*/

static bool need_download_FW(struct msm_sensor_ctrl_t *s_ctrl)
{
	char E_cmd[5] = {0x05,0x02,0x0f,0x12,0x01};
	char resp[5] =  {0x05,0x01,0x00,0x1c,0x01};
	u32 c_cnt = 0;
	#if 1
	if (now_is_boot)
	{
		now_is_boot = false;
		return false;
	}
	#endif
	if(!need_check_FW)
		return false;
	need_check_FW = false;
	msm_camera_i2c_txdata(s_ctrl->sensor_i2c_client,E_cmd,5);
	do
	{
		msleep(300);
		resp[0]= 0x05;
		resp[1]= 0x01;
		resp[2]= 0x00;
		resp[3]= 0x1c;
		msm_vendor_i2c_rxdata(s_ctrl->sensor_i2c_client,resp,5,2);
		m9m0_bug("%s resp[1] = {0x%x}\n", __func__, resp[1]);
		c_cnt++;
	}while((resp[1] != 1)&&(c_cnt < 10));
	
	if(resp[1] == 1)
	{
		resp[0]= 0x05;
		resp[1]= 0x01;
		resp[2]= 0x00;
		resp[3]= 0x02;
		resp[4]= 0x02;
		msm_vendor_i2c_rxdata(s_ctrl->sensor_i2c_client,resp,5,3);	
		if (0 == m9mo_get_FW_version_from_file())
		{
			m9m0_bug("resp[1] = 0x%x, resp[2] = 0x%x \r\n", resp[1], resp[2]);
			m9m0_bug("m9mo_version[0] = 0x%x, m9mo_version[1] = 0x%x \r\n", m9mo_version[0], m9mo_version[1]);
			if ((resp[1] == m9mo_version[0])&&(resp[2] == m9mo_version[1]))
			{
				need_check_FW = false;
				update_FW_by_file = false;
				return false;
			}
			else
			{
				update_FW_by_file = true;
				m9mo_power_down(s_ctrl);
				msleep(200);
				m9mo_power_up(s_ctrl);
				msleep(200);
				return true;
			}
		}
		else
		{
			update_FW_by_file = false;
		}
		m9m0_bug("%s FW_ver[0] = {0x%x} FW_ver[1] = {0x%x}--resp[1] = {0x%x} resp[2] = {0x%x}\n", 
				__func__, m9mo_fw[24], m9mo_fw[25], resp[1],resp[2]);
		if((resp[1] == m9mo_fw[24])&&(resp[2] == m9mo_fw[25]))
		{
			need_check_FW = false;
			return false;
		}else{
			m9mo_power_down(s_ctrl);
			msleep(200);
			m9mo_power_up(s_ctrl);
			msleep(200);
		}
	}
	
	return true;
	
}

static void do_set_port(struct msm_sensor_ctrl_t *s_ctrl)
{
	char byte[8*9] = {0x00,0x08,0x90,0x00,0x12,0x00,0x00,0x40,
		              0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
		              0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
		              0x00,0x00,0x00,0xFF,0x00,0x00,0x00,0x00,
		              0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
		              0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x0F,
		              0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
		              0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x07,
		              0x00,0x00,0x00,0xFF,0x00,0x00,0x00,0xFF,
		             };
	m9m0_bug("%s \n", __func__);
	#ifdef DOWNLOAD_I2C_MODE
	return;
	#endif
	msm_camera_i2c_txdata(s_ctrl->sensor_i2c_client,byte,sizeof(byte));
	byte[1] = 0x08;
	byte[4] = 0x10;
	byte[27] = 0x00;
	byte[47] = 0x00;
	byte[63] = 0x00;
	byte[67] = 0x00;
	byte[71] = 0x00;
	msm_camera_i2c_txdata(s_ctrl->sensor_i2c_client,byte,sizeof(byte));
	byte[4] = 0x11;
	byte[47] = 0x0C;
	byte[51] = 0x20;
	byte[55] = 0x10;
	msm_camera_i2c_txdata(s_ctrl->sensor_i2c_client,byte,sizeof(byte));
}
static void do_sioloader(struct msm_sensor_ctrl_t *s_ctrl)
{
	u32 remain = M9MO_SIO_LOADER_SIZE;
	char byte[8*9] = {0x00,0x08,0x90,0x00,0x12,0x00,0x00,0x40,
		              0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
		              0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
		              0x00,0x00,0x00,0xFF,0x00,0x00,0x00,0x00,
		              0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
		              0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x0F,
		              0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
		              0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x07,
		              0x00,0x00,0x00,0xFF,0x00,0x00,0x00,0xFF,
		             };
	char *S_byte = NULL;
	char *src = sio_loader;
	int i = 0;
	u32 addr = 0x0100;
	int nCnt = 0;
	#ifdef DOWNLOAD_I2C_MODE
	return;
	#endif
	S_byte = kzalloc(256,GFP_KERNEL);   
	if(!S_byte)
	  return;
	m9m0_bug("%s---------->IN \n", __func__);
	msm_camera_i2c_txdata(s_ctrl->sensor_i2c_client,byte,sizeof(byte));
	byte[1] = 0x08;
	byte[4] = 0x10;
	byte[27] = 0x00;
	byte[47] = 0x00;
	byte[63] = 0x00;
	byte[67] = 0x00;
	byte[71] = 0x00;
	msm_camera_i2c_txdata(s_ctrl->sensor_i2c_client,byte,sizeof(byte));
	byte[4] = 0x11;
	byte[47] = 0x0C;
	byte[51] = 0x20;
	byte[55] = 0x10;
	msm_camera_i2c_txdata(s_ctrl->sensor_i2c_client,byte,sizeof(byte));
	
	byte[0] = 0x05;
	byte[1] = 0x01;
	byte[2] = 0x0f;
	byte[3] = 0x1C;
	byte[4] = 0x04;
	msm_vendor_i2c_rxdata(s_ctrl->sensor_i2c_client, byte, 5, 5);
	#if 1
	for (i=1;i<5;i++)
		m9m0_bug("[%s]---byte[%d] = 0x%x \r\n", __func__, i ,byte[i]);
	#endif
	S_byte[0] = 0x00;
    S_byte[1] = 0x04;
    S_byte[2] = 0x01;
    S_byte[3] = 0x00;
	S_byte[4] = 0x01;
	S_byte[5] = 0x00;
	
	while(remain > 0)
	{
	   if(remain > 248)
	   {
	     S_byte[6] = 0x00;
	     S_byte[7] = 0xF8;
		 for(i=0;i<248;i++)
		 {
		 	S_byte[8+i] = src[i];
		 }
		 m9m0_bug("transfer 248-------------[%d] \r\n", nCnt);
		 msm_camera_i2c_txdata(s_ctrl->sensor_i2c_client,S_byte,256);
		 addr += 248;
		 remain -= 248;
		 src += 248;
		 //S_byte[4] = (((addr&0xFF00)>>8)|0x01);
		 S_byte[4] = (addr&0xFF00)>>8;
	     S_byte[5] = addr&0xFF;
	   }else{
	     S_byte[6] = 0x00;
	     S_byte[7] = remain;
		 for(i=0;i<remain;i++)
		 {
		 	S_byte[8+i] = src[i];
		 }
		 m9m0_bug("transfer remain[%d] \r\n", remain);
		 msm_camera_i2c_txdata(s_ctrl->sensor_i2c_client,S_byte,8 + remain);
		 remain = 0; 
	   }
	   
	   nCnt++;
	  
	}
	byte[0] = 0x08;
	byte[1] = 0x02;
	byte[2] = 0x0f;
	byte[3] = 0x0c;
	byte[4] = 0x01;
	byte[5] = 0x00;
	byte[6] = 0x01;
	byte[7] = 0x00;
	msm_camera_i2c_txdata(s_ctrl->sensor_i2c_client,byte,8);
	
	byte[0] = 0x05;
	byte[1] = 0x02;
	byte[2] = 0x0f;
	byte[3] = 0x12;
	byte[4] = 0x02;
	msm_camera_i2c_txdata(s_ctrl->sensor_i2c_client,byte,5);
	kfree(S_byte);
	m9m0_bug("%s -----kfree\n", __func__);
	msleep(200);
	byte[0] = 0x05;
	byte[1] = 0x01;
	byte[2] = 0x0f;
	byte[3] = 0x1C;
	byte[4] = 0x04;
	msm_vendor_i2c_rxdata(s_ctrl->sensor_i2c_client, byte, 5, 5);
	for (i=1;i<5;i++)
		m9m0_bug("[%s]---byte[%d] = 0x%x \r\n", __func__, i ,byte[i]);
	m9m0_bug("%s----------->OUT \n", __func__);
}
static void do_erase(struct msm_sensor_ctrl_t *s_ctrl)
{
	char byte[8] = {0x08,0x02,0x0f,0x00,0x00,0x00,0x00,0x00};
	char E_cmd[5] = {0x05,0x02,0x0f,0x4a,0x01};
	char resp[5] = {0x05,0x01,0x0f,0x06,0x01};
	u32 i = 0;
	msm_camera_i2c_txdata(s_ctrl->sensor_i2c_client,E_cmd,5);
	msleep(5);
	msm_camera_i2c_txdata(s_ctrl->sensor_i2c_client,byte,8);
	E_cmd[3] = 0x06;
	E_cmd[4] = 0x02;
	msm_camera_i2c_txdata(s_ctrl->sensor_i2c_client,E_cmd,5);
	while(i<20)
	{
	  msleep(500);
	  msm_vendor_i2c_rxdata(s_ctrl->sensor_i2c_client,resp,5,2);
	  m9m0_bug("%s resp[1] = [%d] i = %d;\n", __func__,resp[1],i);
	  if(resp[1] == 0)
	  {
	  	m9m0_bug("%s resp[1] = [%d] i = %d;\n", __func__,resp[1],i);
	  	break;
	  }
	  resp[0] = 0x05;
	  resp[1] = 0x01;
	  i++;
	};
}
static void pre_setup(struct msm_sensor_ctrl_t *s_ctrl)
{
	char byte[8] = {0x05,0x02,0x0f,0x4a,0x02,0x00,0x00,0x00};
	//SIO mode start
	#ifdef  DOWNLOAD_I2C_MODE
	return;
	#endif
	m9m0_bug("%s \n", __func__);
	msm_camera_i2c_txdata(s_ctrl->sensor_i2c_client,byte,5);
	msleep(5);
	//set dest address
	byte[0] = 0x08;
	byte[1] = 0x02;
	byte[2] = 0x0f;
	byte[3] = 0x14;
	byte[4] = 0x20;
	byte[5] = 0x00;
	byte[6] = 0x00;
	byte[7] = 0x00;
	msm_camera_i2c_txdata(s_ctrl->sensor_i2c_client,byte,8);
	byte[3] = 0x18;
	byte[4] = 0x00;
	byte[5] = 0x20;
	byte[6] = 0x00;
	byte[7] = 0x00;
	msm_camera_i2c_txdata(s_ctrl->sensor_i2c_client,byte,8);	
}
static void do_i2c_transfer(struct msm_sensor_ctrl_t *s_ctrl)
{
	u32 remain_o = M9MO_FW_SIZE;
	u32 remain_i = 0x10000;
	u32 addr_f = 0x0;
	u32 addr_r = 0x0;
	int poll_cnt = 0;
	char F_addr[8] = {0x08,0x02,0x0f,0x00,0x00,0x00,0x00,0x00}; 
	char W_size[6] = {0x06,0x02,0x0f,0x04,0x00,0x00};
	char W_cmd[5] = {0x05,0x02,0x0f,0x07,0x01};
	char resp[5] = {0x05,0x01,0x0f,0x07,0x01};
	char *S_byte = NULL; 
	char *data_src = NULL;
	
	#ifndef DOWNLOAD_I2C_MODE
	return;
	#endif

	if (update_FW_by_file)
	{
		m9mo_read_FW_from_file(m9mo_fw, M9MO_FW_SIZE, 0);
	}
	
	S_byte = kzalloc(256,GFP_KERNEL);
	data_src = m9mo_fw;
	if(!S_byte)
		    return; 
	
	m9m0_bug("%s \n", __func__); 
	while(remain_o > 0)
	{
	   m9m0_bug("%s addr_f(0x%x) (0x%x) (0x%x) (0x%x) (0x%x)\n", __func__, addr_f, F_addr[4], F_addr[5],F_addr[6],F_addr[7]);
	   msm_camera_i2c_txdata(s_ctrl->sensor_i2c_client,F_addr,sizeof(F_addr)); 
	   msm_camera_i2c_txdata(s_ctrl->sensor_i2c_client,W_size,sizeof(W_size));
	   while(remain_i > 0)
	   {
	   	   int i = 0;
		   S_byte[0] = 0x00;
	       S_byte[1] = 0x04;
	       S_byte[2] = 0x01; 
	   	   S_byte[3] = ((addr_r&0xFF0000)>>16)|0x10;
		   S_byte[4] = (addr_r&0xFF00)>>8;
		   S_byte[5] = (addr_r&0xFF);
		   m9m0_bug("%s addr_r(0x%x) (0x%x) (0x%x) (0x%x)\n", __func__, addr_r, S_byte[3], S_byte[4],S_byte[5]);
		   if(remain_i >= 248)
		   {
		     S_byte[6] = 0x00;
		     S_byte[7] = 0xF8;
			 for(i = 0;i<248;i++)
			 {
			 	S_byte[8+i] = data_src[i];
			 }
			 msm_camera_i2c_txdata(s_ctrl->sensor_i2c_client,S_byte,256);
			 addr_r += 248;
			 data_src += 248;
			 remain_i -= 248;
		   }else{
		     S_byte[6] = 0x00;
		     S_byte[7] = remain_i;
			 for(i = 0;i<remain_i;i++)
			 {
			 	S_byte[8+i] = data_src[i];
			 } 
			 #ifdef CHECK_I2C_TRANSFER
			 for(i = 0;i<remain_i/16;i++)
			 {
				 m9m0_bug("%s 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x\n"
				 	,__func__,S_byte[8+16*i],S_byte[9+16*i],S_byte[10+16*i],S_byte[11+16*i],S_byte[12+16*i],S_byte[13+16*i],S_byte[14+16*i],S_byte[15+16*i],S_byte[16+16*i],S_byte[17+16*i],S_byte[18+16*i],S_byte[19+16*i],S_byte[20+16*i],S_byte[21+16*i],S_byte[22+16*i],S_byte[23+16*i]);
			 }
			 #endif
			 msm_camera_i2c_txdata(s_ctrl->sensor_i2c_client,S_byte,8 + remain_i);
             #ifdef CHECK_I2C_TRANSFER
			   S_byte[0] = 0x00;
		       S_byte[1] = 0x03;
		       S_byte[2] = 0x01; 
		   	   S_byte[3] = ((addr_r&0xFF0000)>>16)|0x10;
			   S_byte[4] = (addr_r&0xFF00)>>8;
			   S_byte[5] = (addr_r&0xFF);
			   S_byte[6] = 0x00;
		       S_byte[7] = remain_i;
			 msm_vendor_i2c_rxdata(s_ctrl->sensor_i2c_client,S_byte,8 ,remain_i+3);
			 for(i = 0;i<remain_i/16;i++)
			 {
				 m9m0_bug("%s 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x\n"
				 	,__func__,S_byte[16*i],S_byte[1+16*i],S_byte[2+16*i],S_byte[3+16*i],S_byte[4+16*i],S_byte[5+16*i],S_byte[6+16*i],S_byte[7+16*i],S_byte[8+16*i],S_byte[9+16*i],S_byte[10+16*i],S_byte[11+16*i],S_byte[12+16*i],S_byte[13+16*i],S_byte[14+16*i],S_byte[15+16*i]);
			 }
			 #endif
			 data_src += remain_i; 
			 remain_i = 0;
			 break;
		   }
		   
	   }
	   msm_camera_i2c_txdata(s_ctrl->sensor_i2c_client,W_cmd,sizeof(W_cmd)); 
	   poll_cnt = 0;
	   do{
	   	 msleep(50);
	   	 resp[0] = 0x05;
		 resp[1] = 0x01;
	   	 msm_vendor_i2c_rxdata(s_ctrl->sensor_i2c_client,resp,5,2);
		 poll_cnt++;
		 m9m0_bug("resp[1] = {%d} poll_cnt = {%d};\n",resp[1],poll_cnt);
	   }while((resp[1] != 0)&&(poll_cnt < 10));
	   addr_f += 0x10000;
	   addr_r = 0x0;
	   remain_o -= 0x10000;
	   remain_i = 0x10000;
	   F_addr[4] = (addr_f&0xFF000000)>>24;
	   F_addr[5] = (addr_f&0xFF0000)>>16;
	   F_addr[6] = (addr_f&0xFF00)>>8;
	   F_addr[7] = (addr_f&0xFF);
	}

}
static void do_setpll(struct msm_sensor_ctrl_t *s_ctrl)
{
	char byte[8] = {0x08,0x02,0x0f,0x1c,0x01,0x2f,0x02,0x52};
	msm_camera_i2c_txdata(s_ctrl->sensor_i2c_client,byte,8);
}
static void do_program(struct msm_sensor_ctrl_t *s_ctrl)
{
	char byte[8] = {0x08,0x02,0x0f,0x00,0x00,0x00,0x00,0x00};
	char resp[5] = {0x05,0x01,0x0f,0x07,0x01};
	u32 poll_cnt = 0;
	m9m0_bug("%s------>IN \n", __func__);
	//set flash rom address
	msm_camera_i2c_txdata(s_ctrl->sensor_i2c_client,byte,8);
	//set programming byte
	byte[0] = 0x06;
	byte[1] = 0x02;
	byte[2] = 0x0f;
	byte[3] = 0x18;
	byte[4] = 0x00;
	byte[5] = 0x20;
	byte[6] = 0x00;
	byte[7] = 0x00;
	msm_camera_i2c_txdata(s_ctrl->sensor_i2c_client,byte,8);
	//program
	byte[0] = 0x05;
	byte[1] = 0x02;
	byte[2] = 0x0f;
	byte[3] = 0x07;
	byte[4] = 0x01;
	msm_camera_i2c_txdata(s_ctrl->sensor_i2c_client,byte,5);
	//check programming process
	do{
	  msleep(100);
	  resp[0] = 0x05;
	  resp[1] = 0x01;
	  msm_vendor_i2c_rxdata(s_ctrl->sensor_i2c_client,resp,5,2);
	  poll_cnt++;
	  m9m0_bug("%s resp[1] = [%d] poll_cnt = %d;\n", __func__,resp[1],poll_cnt);
	}while((poll_cnt<100)&&(resp[1] == 1));
	m9m0_bug("%s------>OUT \n", __func__);
	
}
static void do_transfer(struct msm_sensor_ctrl_t *s_ctrl)
{
	struct m9mo_spi_trans transfer;
	
	#ifdef DOWNLOAD_I2C_MODE
	return;
	#endif
	
	m9m0_bug("%s------>IN \n", __func__);
	if(!m9mo_spi_dev)
	{
		m9m0_bug("m9mo_spi_dev is NULL \r\n");
		FW_download = false;
	    return;
	}
	m9m0_bug("%s \n", __func__);
	gpio_request(12, "spi_cs");
	gpio_direction_output(12, GPIOF_OUT_INIT_HIGH);
	gpio_free(12);
	
	msleep(1);
	
	if (update_FW_by_file)
	{
		m9mo_update_FW_by_file();
	}
	else
	{
		memset(&transfer, 0, sizeof(transfer));
		spi_message_init(&transfer.spi_msg);
		transfer.spi_xfer.tx_buf = m9mo_fw;
		transfer.spi_xfer.len	   = M9MO_FW_SIZE;
		transfer.spi_xfer.bits_per_word = 8;
		spi_message_add_tail(&transfer.spi_xfer, &transfer.spi_msg);
		m9mo_spi_dev->mode = SPI_MODE_0;
		spi_sync(m9mo_spi_dev, &transfer.spi_msg);
	}
	
	gpio_request(12, "spi_cs");
	gpio_direction_output(12, GPIOF_OUT_INIT_LOW);
	gpio_free(12);
	
	do_program(s_ctrl);
	m9m0_bug("%s------>OUT \n", __func__);
}
static bool do_finish(struct msm_sensor_ctrl_t *s_ctrl)
{ 	
	char E_cmd[5] = {0x05,0x02,0x0f,0x09,0x04};
	char resp[5] =  {0x05,0x01,0x0f,0x09,0x01};
	u32 c_cnt = 0; 
	msm_camera_i2c_txdata(s_ctrl->sensor_i2c_client,E_cmd,5); 
	do{
		msleep(10);
		resp[0] = 0x05;
		resp[1] = 0x01;
		resp[2] = 0x0f;
		resp[3] = 0x09;
		resp[4] = 0x01;
		msm_vendor_i2c_rxdata(s_ctrl->sensor_i2c_client,resp,5,2);
		m9m0_bug("%s resp[1] = {0x%d}\n", __func__, resp[1]);
		c_cnt++;
	}while((resp[1]&0x04)&&(c_cnt < 100));
	
	if(!(resp[1]&0x04))
	{
		resp[0] = 0x05;
		resp[1] = 0x01;
		resp[2] = 0x0f;
		resp[3] = 0x0a;
		resp[4] = 0x02;
		msm_vendor_i2c_rxdata(s_ctrl->sensor_i2c_client,resp,5,3);
		m9m0_bug("[%s] checksum resp[1] = {0x%d}resp[2] = {0x%d}\n", __func__, resp[1], resp[2]);
		if((resp[1] == 0)&&(resp[1] == 0))
			return true;
	}
	return false;
}

static int m9mo_start_program(struct msm_sensor_ctrl_t *s_ctrl)
{
	char E_cmd[5] = {0x05,0x02,0x0f,0x12,0x01};
	char resp[5] =  {0x05,0x01,0x00,0x1c,0x01};
	u32 c_cnt = 0;
	
	m9mo_power_down(s_ctrl);
	msleep(200);
	m9mo_power_up(s_ctrl);
	msleep(200);
	
	msm_camera_i2c_txdata(s_ctrl->sensor_i2c_client,E_cmd,5);
	do
	{
		msleep(500);
		resp[0]= 0x05;
		resp[1]= 0x01;
		resp[2]= 0x00;
		resp[3]= 0x1c;
		msm_vendor_i2c_rxdata(s_ctrl->sensor_i2c_client,resp,5,2);
		m9m0_bug("%s resp[1] = {0x%d}\n", __func__, resp[1]);
		c_cnt++;
	}while((resp[1] != 1)&&(c_cnt < 10));

	if (1 == resp[1])
		return 0;
	else
		return -1;
	
}

static void DownloadFW(struct msm_sensor_ctrl_t *s_ctrl)
{
	m9m0_bug("%s \n", __func__);
	if(need_download_FW(s_ctrl))
	{ 
		m9m0_bug("%s do! \n", __func__);
		
		do_set_port(s_ctrl);
		do_sioloader(s_ctrl);
		do_erase(s_ctrl);
		do_setpll(s_ctrl);
		pre_setup(s_ctrl);
		do_transfer(s_ctrl);
		do_i2c_transfer(s_ctrl);
		do_finish(s_ctrl);	
		m9mo_start_program(s_ctrl);
		
	}
	
}

static bool force_update_fw(struct msm_sensor_ctrl_t *s_ctrl)
{
	bool ret = false;
	m9m0_bug("%s \n", __func__);
	update_FW_by_file = true;
	m9mo_power_up(s_ctrl);
	msleep(100);
	do_set_port(s_ctrl);
	do_sioloader(s_ctrl);
	do_erase(s_ctrl);
	do_setpll(s_ctrl);
	pre_setup(s_ctrl);
	do_transfer(s_ctrl);
	do_i2c_transfer(s_ctrl);
	ret = do_finish(s_ctrl);
	m9mo_power_down(s_ctrl);
	update_FW_by_file = false;
	return ret;
}
static bool ctrl_led(struct msm_sensor_ctrl_t *s_ctrl,int onoff)
{
	bool ret = false;

	m9m0_bug("%s \n", __func__); 
	if (led_is_on == onoff)
		return true;
	if(onoff)
	{
    	m9mo_power_up(s_ctrl);
    	msleep(100);
    	need_download_FW(s_ctrl); 
    	ZSL_preview_output(s_ctrl);
    	msleep(100);
    	m9mo_set_flash_mode(s_ctrl,3);
	}
	else  
	{
		m9mo_set_flash_mode(s_ctrl,0);
		m9mo_power_down(s_ctrl);
	}

	led_is_on = onoff;
	
	return ret;
}


/*----------add by liubin for m9mo proc start ---------------*/
static int m9mo_hex_to_char(char *pBuf, unsigned int nHex)
{
	int nHigh = 0;
	int nLow = 0;
	
	pBuf[0] = '0';
	pBuf[1] = 'x';

	nHigh = (nHex&0xF0)>>4;
	nLow = nHex & 0x0F;

	printk("nHigh = %d, nLow = %d \r\n", nHigh, nLow);

	if (nHigh >= 0 && nHigh <= 9)
		pBuf[2] = nHigh + '0';
	else if (nHigh >= 0x0A && nHigh <= 0x0F)
		pBuf[2] = nHigh - 10 + 'a';

	if (nLow >= 0 && nLow <= 9)
		pBuf[3] = nLow + '0';
	else if (nLow >= 10 && nLow <= 15)
		pBuf[3] = nLow - 10 + 'a';

	printk("pBuf[2] = %c, pBuf[3] = %c \r\n", pBuf[2], pBuf[3]);

	return 0;
		
}

static bool m9mo_parse_proc_data(char *proc_data, unsigned long len)
{
	int  i = 0;
	int nCmdIndex = 0;

	//check proc data if valid
	while (proc_data[i] != '\0' && i<len-1)
	{
		printk("proc_data[%d] = %c \r\n", i, proc_data[i]);
		if (!((proc_data[i] >= '0' && proc_data[i] <= '9')
			|| (proc_data[i] >= 'a' && proc_data[i] <= 'f')
			|| (proc_data[i] >= 'A' && proc_data[i] <= 'F')
			|| proc_data[i] == ',' 
			|| proc_data[i] == 'X'
			|| proc_data[i] == 'x'
			|| proc_data[i] == ' '))
		{
			printk("[%s]----m9mo proc input data invalid \r\n", __func__);
			return false;
		}

		if (proc_data[i] >= '0' && proc_data[i] <= '9')
		{
			proc_data[i] = proc_data[i]-'0';
		}
		else if (proc_data[i] >= 'a' && proc_data[i] <= 'f')
			proc_data[i] = proc_data[i]-'a'+10;
		else if (proc_data[i] >= 'A' && proc_data[i] <= 'F')
			proc_data[i] = proc_data[i]-'A'+10;

		if (proc_data[i] == ','
			&& i > 0
			&& nCmdIndex < MAX_CMD_INDEX)
		{
			m9mo_cmd[nCmdIndex] = proc_data[i-2] * 16 + proc_data[i-1];
			printk("[%s]---------m9mo_cmd[%d] = 0x%x \r\n", __func__, nCmdIndex, m9mo_cmd[nCmdIndex]);
			nCmdIndex++;
		}
		
		i++;
	}

	if (i == len -1)
	{
		m9mo_cmd[nCmdIndex] = proc_data[i-2] * 16 + proc_data[i-1];
		printk("[%s]---------m9mo_cmd[%d] = 0x%x \r\n", __func__, nCmdIndex, m9mo_cmd[nCmdIndex]);
	}

	m9mo_cmd[nCmdIndex+1] = '\0';

	return true;
	
}

static int m9mo_proc_read(char *page, char **start, off_t off, int count,
   int *eof, void *data)
{
	
	int len = 0;
	int i = 0;
	
	len = sprintf(page, m9mo_proc_data);
	while (page[i] != '\0')
	{
		printk("page[%d] = %c \r\n", i, page[i]);
		i++;
	}
	return len;
}

static int m9mo_proc_write(struct file *filp, const char __user *buff,
                        	unsigned long len, void *data)
{
	bool bRet = false;
	int nReadLen = 0;
	int i = 0;
	
	struct msm_sensor_ctrl_t *s_ctrl = data;
	
	if (len > sizeof(m9mo_proc_data))
	{
		len = sizeof(m9mo_proc_data) - 1;
	}

	printk("[%s]-----len:%lu \r\n", __func__, len);

	if (copy_from_user(&m9mo_proc_data, buff, len)) {
		m9m0_bug("m9mo read proc input error.\n");
		return -EFAULT;
	}

	m9mo_proc_data[len] = '\0';

	bRet = m9mo_parse_proc_data(m9mo_proc_data, len);
	if (!bRet)
	{
		m9m0_bug("m9mo proc input data is not format.\n");
		return -EFAULT;
	}

	m9mo_category = m9mo_cmd[2];
	m9mo_byte = m9mo_cmd[3];

	if (m9mo_cmd[1] == 0x01)
	{
		nReadLen = m9mo_cmd[4];
		msm_vendor_i2c_rxdata(s_ctrl->sensor_i2c_client, m9mo_cmd, m9mo_cmd[0], m9mo_cmd[4]+1);
		for (i=0; i<nReadLen; i++)
		{
			printk("[%s]----read data m9mo_cmd[%d] 0x%x \r\n", __func__, i+1, m9mo_cmd[i+1]);
			m9mo_read_data[i] = m9mo_cmd[i+1];
		}

		//copy data to user buff
		//Category
		m9mo_proc_data[0] = 'C';
		m9mo_proc_data[1] = 'a';
		m9mo_proc_data[2] = 't';
		m9mo_proc_data[3] = 'e';
		m9mo_proc_data[4] = ':';
		m9mo_hex_to_char(m9mo_proc_data+5, (unsigned int)m9mo_category);
		m9mo_proc_data[9] = '\n';
		//Byte
		m9mo_proc_data[10] = 'B';
		m9mo_proc_data[11] = 'y';
		m9mo_proc_data[12] = 't';
		m9mo_proc_data[13] = 'e';
		m9mo_proc_data[14] = ':';
		m9mo_hex_to_char(m9mo_proc_data+15, (unsigned int)m9mo_byte);
		m9mo_proc_data[19] = '\n';
		//read data num
		m9mo_proc_data[20] = 'N';
		m9mo_proc_data[21] = 'u';
		m9mo_proc_data[22] = 'm';
		m9mo_proc_data[23] = ':';
		m9mo_hex_to_char(m9mo_proc_data+24, (unsigned int)nReadLen);
		m9mo_proc_data[28] = '\n';

		//value
		m9mo_proc_data[29] = 'V';
		m9mo_proc_data[30] = 'a';
		m9mo_proc_data[31] = 'l';
		m9mo_proc_data[32] = 'u';
		m9mo_proc_data[33] = 'e';
		m9mo_proc_data[34] = ':';
		for (i=0; i<nReadLen; i++)
		{
			m9mo_hex_to_char(m9mo_proc_data+35+i*5, (unsigned int)m9mo_read_data[i]);
			if (i == nReadLen-1)
			{
				m9mo_proc_data[39+5*i] = '\n';
				m9mo_proc_data[39+5*i+1] = '\0';
			}
			else
			{
				m9mo_proc_data[39+5*i] = ',';
			}
		}
	}
	else if (m9mo_cmd[1] == 0x02)
	{
		msm_camera_i2c_txdata(s_ctrl->sensor_i2c_client, m9mo_cmd, m9mo_cmd[0]);
	}
	//lanhe@oppo.com add for force update FW
	else if (m9mo_cmd[1] == 0xFF)
	{
		if(force_update_fw(s_ctrl))
		{
			m9m0_bug("m9mo write back success.\n");
		}else{
		   	 m9m0_bug("m9mo write back error.\n");
		}
	} else if (m9mo_cmd[1] == 0xFE)
	{
	    ctrl_led(s_ctrl,m9mo_cmd[2]);
	}
	
	return len;
}

//m9mo proc init
static int m9mo_proc_init(struct msm_sensor_ctrl_t *s_ctrl)
{
	int ret=0;
    
	struct proc_dir_entry *proc_entry = create_proc_entry( "m9mo_v4l2", 0666, NULL);
	proc_entry->data = s_ctrl;

	if (proc_entry == NULL)
	{
		ret = -ENOMEM;
	  	m9m0_bug("[%s]: Error! Couldn't create m9mo_v4l2 proc entry\n", __func__);
	}
	else
	{
		proc_entry->read_proc = m9mo_proc_read;
		proc_entry->write_proc = m9mo_proc_write;
	}
	
	return ret;
	
}
/*----------add by liubin for m9mo proc end ---------------*/

static int __devinit m9mo_spi_probe(struct spi_device *spi)
{
	m9m0_bug("m9mo_spi_probe-----spi = %x \n", (unsigned int)spi);
	m9mo_spi_dev = spi;
	return 0;
}

static struct spi_driver m9mo_spi_driver = {
	.driver	= {
		.name	= "m9mo_spi",
		.owner	= THIS_MODULE,
	},
	.probe	= m9mo_spi_probe,
	.remove	= NULL,
};

struct gpio_button_data {
	const struct gpio_keys_button *button;
	struct input_dev *input;
	struct timer_list timer;
	struct work_struct work;
	unsigned int timer_debounce;	/* in msecs */
	unsigned int irq;
	spinlock_t lock;
	bool disabled;
	bool key_pressed;
};

struct gpio_keys_drvdata {
	struct input_dev *input;
	struct mutex disable_lock;
	unsigned int n_buttons;
	int (*enable)(struct device *dev);
	void (*disable)(struct device *dev);
	struct gpio_button_data data[0];
};

static int gpio_keys_open(struct input_dev *input)
{
	struct gpio_keys_drvdata *ddata = input_get_drvdata(input);

	return ddata->enable ? ddata->enable(input->dev.parent) : 0;
}

static void gpio_keys_close(struct input_dev *input)
{
	struct gpio_keys_drvdata *ddata = input_get_drvdata(input);

	if (ddata->disable)
		ddata->disable(input->dev.parent);
}

static void gpio_keys_gpio_report_event(struct gpio_button_data *bdata)
{
	const struct gpio_keys_button *button = bdata->button;
	struct input_dev *input = bdata->input;
	unsigned int type = button->type ?: EV_KEY;
	int state = (gpio_get_value_cansleep(button->gpio) ? 1 : 0) ^ button->active_low;

	if (type == EV_ABS) {
		if (state)
			input_event(input, type, button->code, button->value);
	} else {
		input_event(input, type, button->code, !!state);
	}
	input_sync(input);
}

static void gpio_keys_gpio_work_func(struct work_struct *work)
{
	int gpio_value = 0;
	
	struct gpio_button_data *bdata =
		container_of(work, struct gpio_button_data, work);

	struct msm_sensor_ctrl_t* s_ctrl = CTRL;

	gpio_keys_gpio_report_event(bdata);
	
	m9m0_bug("[%s]: Camera rotate inturrupt coming \r\n", __func__);
	gpio_value = gpio_get_value(bdata->button->gpio);
	m9m0_bug("[%s]: gpio[%d]--- value[%d] \r\n", __func__, bdata->button->gpio, gpio_value);
	
	if (camera_work && MR_ROTATE_GPIO == bdata->button->gpio)
	{
		g_bRotate = true;
		switch (s_ctrl->curr_res)
		{
			case 2:
			{
				m9mo_change_to_param_set_mode(s_ctrl);
				ZSL_preview_output(s_ctrl);
				g_bRotate = false;
				break;
			}
			case 3:
			{
				m9mo_change_to_param_set_mode(s_ctrl);
				video_output(s_ctrl);
				g_bRotate = false;
				break;
			}
			case 4:
			{
				m9mo_change_to_param_set_mode(s_ctrl);
				HDR_video_output(s_ctrl);
				g_bRotate = false;
				break;
			}
			case 6:
			{
				m9mo_change_to_param_set_mode(s_ctrl);
				ZSL_panorama_preview(s_ctrl);
				g_bRotate = false;
				break;
			}
			default:
				break;
		}
	}
	
	msleep(10);
	enable_irq(bdata->irq);
}

static void gpio_keys_gpio_timer(unsigned long _data)
{
	struct gpio_button_data *bdata = (struct gpio_button_data *)_data;

	schedule_work(&bdata->work);
}

static irqreturn_t gpio_keys_gpio_isr(int irq, void *dev_id)
{
	struct gpio_button_data *bdata = dev_id;

	BUG_ON(irq != bdata->irq);

	if (bdata->timer_debounce)
		mod_timer(&bdata->timer,
			jiffies + msecs_to_jiffies(bdata->timer_debounce));
	else
		schedule_work(&bdata->work);

	return IRQ_HANDLED;
}

static void gpio_keys_irq_timer(unsigned long _data)
{
	struct gpio_button_data *bdata = (struct gpio_button_data *)_data;
	struct input_dev *input = bdata->input;
	unsigned long flags;

	spin_lock_irqsave(&bdata->lock, flags);
	if (bdata->key_pressed) {
		input_event(input, EV_KEY, bdata->button->code, 0);
		input_sync(input);
		bdata->key_pressed = false;
	}
	spin_unlock_irqrestore(&bdata->lock, flags);
}

static irqreturn_t gpio_keys_irq_isr(int irq, void *dev_id)
{
	struct gpio_button_data *bdata = dev_id;
	const struct gpio_keys_button *button = bdata->button;
	struct input_dev *input = bdata->input;
	unsigned long flags;

	BUG_ON(irq != bdata->irq);

	spin_lock_irqsave(&bdata->lock, flags);

	if (!bdata->key_pressed) {
		input_event(input, EV_KEY, button->code, 1);
		input_sync(input);

		if (!bdata->timer_debounce) {
			input_event(input, EV_KEY, button->code, 0);
			input_sync(input);
			goto out;
		}

		bdata->key_pressed = true;
	}

	if (bdata->timer_debounce)
		mod_timer(&bdata->timer,
			jiffies + msecs_to_jiffies(bdata->timer_debounce));
out:
	spin_unlock_irqrestore(&bdata->lock, flags);
	return IRQ_HANDLED;
}

static int __devinit gpio_keys_setup_key(struct platform_device *pdev,
					 struct input_dev *input,
					 struct gpio_button_data *bdata,
					 const struct gpio_keys_button *button)
{
	const char *desc = button->desc ? button->desc : "gpio_keys";
	struct device *dev = &pdev->dev;
	irq_handler_t isr;
	unsigned long irqflags;
	int irq, error;

	bdata->input = input;
	bdata->button = button;
	spin_lock_init(&bdata->lock);

	if (gpio_is_valid(button->gpio)) {

		error = gpio_request(button->gpio, desc);
		if (error < 0) {
			dev_err(dev, "Failed to request GPIO %d, error %d\n",
				button->gpio, error);
			return error;
		}

		error = gpio_direction_input(button->gpio);
		if (error < 0) {
			dev_err(dev,
				"Failed to configure direction for GPIO %d, error %d\n",
				button->gpio, error);
			goto fail;
		}

		if (button->debounce_interval) {
			error = gpio_set_debounce(button->gpio,
					button->debounce_interval * 1000);
			/* use timer if gpiolib doesn't provide debounce */
			if (error < 0)
				bdata->timer_debounce =
						button->debounce_interval;
		}

		irq = gpio_to_irq(button->gpio);
		if (irq < 0) {
			error = irq;
			dev_err(dev,
				"Unable to get irq number for GPIO %d, error %d\n",
				button->gpio, error);
			goto fail;
		}
		bdata->irq = irq;

		INIT_WORK(&bdata->work, gpio_keys_gpio_work_func);
		setup_timer(&bdata->timer,
			    gpio_keys_gpio_timer, (unsigned long)bdata);

		isr = gpio_keys_gpio_isr;
		irqflags = IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING;

	} else {
		if (!button->irq) {
			dev_err(dev, "No IRQ specified\n");
			return -EINVAL;
		}
		bdata->irq = button->irq;

		if (button->type && button->type != EV_KEY) {
			dev_err(dev, "Only EV_KEY allowed for IRQ buttons.\n");
			return -EINVAL;
		}

		bdata->timer_debounce = button->debounce_interval;
		setup_timer(&bdata->timer,
			    gpio_keys_irq_timer, (unsigned long)bdata);

		isr = gpio_keys_irq_isr;
		irqflags = 0;
	}

	input_set_capability(input, button->type ?: EV_KEY, button->code);

	/*
	 * If platform has specified that the button can be disabled,
	 * we don't want it to share the interrupt line.
	 */
	if (!button->can_disable)
		irqflags |= IRQF_SHARED;

	error = request_any_context_irq(bdata->irq, isr, irqflags, desc, bdata);
	if (error < 0) {
		dev_err(dev, "Unable to claim irq %d; error %d\n",
			bdata->irq, error);
		goto fail;
	}

	return 0;

fail:
	if (gpio_is_valid(button->gpio))
		gpio_free(button->gpio);

	return error;
}

static void gpio_remove_key(struct gpio_button_data *bdata)
{
	free_irq(bdata->irq, bdata);
	if (bdata->timer_debounce)
		del_timer_sync(&bdata->timer);
	cancel_work_sync(&bdata->work);
	if (gpio_is_valid(bdata->button->gpio))
		gpio_free(bdata->button->gpio);
}

static int __devinit camera_rotate_keys_probe(struct platform_device *pdev)
{
	const struct gpio_keys_platform_data *pdata = pdev->dev.platform_data;
	struct gpio_keys_drvdata *ddata;
	struct device *dev = &pdev->dev;
	struct input_dev *input;
	int i, error;
	int wakeup = 0;
	
	if (!pdata) {
		m9m0_bug("platform_data is NULL \r\n");
		return -1;
	}

	ddata = kzalloc(sizeof(struct gpio_keys_drvdata) +
			pdata->nbuttons * sizeof(struct gpio_button_data),
			GFP_KERNEL);
	
	input = input_allocate_device();
	if (!ddata || !input) {
		dev_err(dev, "failed to allocate state\n");
		error = -ENOMEM;
		goto fail1;
	}

	ddata->input = input;
	ddata->n_buttons = pdata->nbuttons;
	ddata->enable = pdata->enable;
	ddata->disable = pdata->disable;
	mutex_init(&ddata->disable_lock);

	platform_set_drvdata(pdev, ddata);
	input_set_drvdata(input, ddata);

	input->name = pdata->name ? : pdev->name;
	input->phys = "rotate-keys/input0";
	input->dev.parent = &pdev->dev;
	input->open = gpio_keys_open;
	input->close = gpio_keys_close;

	input->id.bustype = BUS_HOST;
	input->id.vendor = 0x0001;
	input->id.product = 0x0001;
	input->id.version = 0x0100;

	/* Enable auto repeat feature of Linux input subsystem */
	if (pdata->rep)
		__set_bit(EV_REP, input->evbit);

	for (i = 0; i < pdata->nbuttons; i++) {
		const struct gpio_keys_button *button = &pdata->buttons[i];
		struct gpio_button_data *bdata = &ddata->data[i];

		error = gpio_keys_setup_key(pdev, input, bdata, button);
		if (error)
			goto fail2;

		if (button->wakeup)
			wakeup = 1;
	}

	error = input_register_device(input);
	if (error) {
		dev_err(dev, "Unable to register input device, error: %d\n",
			error);
		goto fail2;
	}

	/* get current state of buttons that are connected to GPIOs */
	for (i = 0; i < pdata->nbuttons; i++) {
		struct gpio_button_data *bdata = &ddata->data[i];
		if (gpio_is_valid(bdata->button->gpio))
			gpio_keys_gpio_report_event(bdata);
	}
	input_sync(input);

	device_init_wakeup(&pdev->dev, wakeup);

	return 0;

 fail2:
	while (--i >= 0)
		gpio_remove_key(&ddata->data[i]);

	platform_set_drvdata(pdev, NULL);
 fail1:
	input_free_device(input);
	kfree(ddata);
	/* If we have no platform_data, we allocated buttons dynamically. */
	if (!pdev->dev.platform_data)
		kfree(pdata->buttons);

	return error;
}

static int __devexit camera_rotate_keys_remove(struct platform_device *pdev)
{
	struct gpio_keys_drvdata *ddata = platform_get_drvdata(pdev);
	struct input_dev *input = ddata->input;
	int i;

	device_init_wakeup(&pdev->dev, 0);

	for (i = 0; i < ddata->n_buttons; i++)
		gpio_remove_key(&ddata->data[i]);

	input_unregister_device(input);

	/*
	 * If we had no platform_data, we allocated buttons dynamically, and
	 * must free them here. ddata->data[0].button is the pointer to the
	 * beginning of the allocated array.
	 */
	if (!pdev->dev.platform_data)
		kfree(ddata->data[0].button);

	kfree(ddata);

	return 0;
}


static struct platform_driver camera_rotate_keys_device_driver = {
	.probe		= camera_rotate_keys_probe,
	.remove		= __devexit_p(camera_rotate_keys_remove),
	.driver		= {
		.name	= "camera-rotate-keys",
		.owner	= THIS_MODULE,
	}
};

static int __init m9mo_sensor_init_module(void)
{
	if (get_pcb_version() == PCB_VERSION_EVT_N1)
	{
		platform_driver_register(&camera_rotate_keys_device_driver);
		spi_register_driver(&m9mo_spi_driver);
		return i2c_add_driver(&m9mo_i2c_driver);
	}
	else
	{
		return 0;
	}
}

static struct v4l2_subdev_core_ops m9mo_subdev_core_ops = {
	.ioctl = msm_sensor_subdev_ioctl,
	.s_power = msm_sensor_power,
};

static struct v4l2_subdev_video_ops m9mo_subdev_video_ops = {
	.enum_mbus_fmt = msm_sensor_v4l2_enum_fmt,
};

static struct v4l2_subdev_ops m9mo_subdev_ops = {
	.core = &m9mo_subdev_core_ops,
	.video  = &m9mo_subdev_video_ops,
};
void m9mo_start_stream(struct msm_sensor_ctrl_t *s_ctrl)
{
	m9m0_bug("%s \n", __func__);
	if(m9mo_action[s_ctrl->curr_res].start)
	{
		if (0 == s_ctrl->curr_res && g_bHDREnable)
		{
			m9mo_action[7].start(s_ctrl);
		}
		else
		{
			m9mo_action[s_ctrl->curr_res].start(s_ctrl);
		}
	}
	enable_irq(s_ctrl->sensor_i2c_client->client->irq);
}
void m9mo_stop_stream(struct msm_sensor_ctrl_t *s_ctrl)
{
	FW_download = false;
	if(m9mo_action[s_ctrl->curr_res].stop)
	{
		m9mo_action[s_ctrl->curr_res].stop(s_ctrl);
	}
	m9m0_bug("%s \n", __func__);
}
int32_t m9mo_setting(struct msm_sensor_ctrl_t *s_ctrl,
			int update_type, int res)
{
	int32_t rc = 0;
	m9m0_bug("%s : res [%d] \n", __func__, res);
	
	if (update_type == MSM_SENSOR_REG_INIT) {
		s_ctrl->func_tbl->sensor_stop_stream(s_ctrl);
	} else if (update_type == MSM_SENSOR_UPDATE_PERIODIC) {
		printk("set op_pixel_clk = [%d] \r\n", s_ctrl->msm_sensor_reg->
			output_settings[res].op_pixel_clk);
		v4l2_subdev_notify(&s_ctrl->sensor_v4l2_subdev,
			NOTIFY_PCLK_CHANGE, &s_ctrl->msm_sensor_reg->
			output_settings[res].op_pixel_clk);
	}
	if(m9mo_action[res].init)
	{
		m9mo_action[res].init(s_ctrl);
	}
	return rc;
}

static int32_t m9mo_change_to_param_set_mode(struct msm_sensor_ctrl_t *s_ctrl)
{
	char E_cmd[5] = {0x05,0x02,0x00,0x0B,0x01};
	char resp[5] = {0x05,0x01,0x00,0x0B,0x01};
	u32 i = 0;

	//change to parameter setting mode
	E_cmd[2] = 0x00;
	E_cmd[3] = 0x0B;
	E_cmd[4] = 0x01;
	msm_camera_i2c_txdata(s_ctrl->sensor_i2c_client,E_cmd,5);
	//polling parameter mode
	while(i<20)
	{
		msleep(300);
		msm_vendor_i2c_rxdata(s_ctrl->sensor_i2c_client,resp,5,2);
		m9m0_bug("%s resp[1] = %d, i = %d \n", __func__,resp[1],i);
		if(resp[1] == 0x01)
		{
			m9m0_bug("%s resp[1] = %d, i = %d \n", __func__,resp[1],i);
			break;
		}
		resp[0] = 0x05;
		resp[1] = 0x01;
		i++;
	}
	return 0;
}

//----------------------------------------------------------oppo
int32_t msm_set_m9mo_mode(struct msm_sensor_ctrl_t *s_ctrl,
	int mode, int res)
{
	int32_t rc = 0;
	m9m0_bug("%s res(%d)\n", __func__, res);
	if (s_ctrl->curr_res != res) {
		s_ctrl->curr_frame_length_lines =
			s_ctrl->msm_sensor_reg->
			output_settings[res].frame_length_lines;

		s_ctrl->curr_line_length_pclk =
			s_ctrl->msm_sensor_reg->
			output_settings[res].line_length_pclk;

		if (s_ctrl->is_csic ||
			!s_ctrl->sensordata->csi_if)
			rc = s_ctrl->func_tbl->sensor_csi_setting(s_ctrl,
				MSM_SENSOR_UPDATE_PERIODIC, res);
		else
			rc = s_ctrl->func_tbl->sensor_setting(s_ctrl,
				MSM_SENSOR_UPDATE_PERIODIC, res);
		if (rc < 0)
			return rc;
		
		if (3 == res || 
			(2 == res && 3 == s_ctrl->curr_res) ||
			(4 == res && 2 == s_ctrl->curr_res) ||
			(2 == res && 4 == s_ctrl->curr_res) ||
			(6 == res && 2 == s_ctrl->curr_res) || 
			(2 == res && 6 == s_ctrl->curr_res))
		{
			m9mo_change_to_param_set_mode(s_ctrl);
		}
		
		s_ctrl->curr_res = res;
	}
	return rc;
}
int32_t m9mo_mode_init(struct msm_sensor_ctrl_t *s_ctrl,
			int mode, struct sensor_init_cfg *init_info)
{
	int32_t rc = 0;
	m9m0_bug("%s \n", __func__);
	s_ctrl->fps_divider = Q10;
	s_ctrl->cam_mode = MSM_SENSOR_MODE_INVALID;

	m9m0_bug("[%s]----mode : %d , cam_mode: %d \r\n", __func__, mode, s_ctrl->cam_mode);
	if (mode != s_ctrl->cam_mode) {
		s_ctrl->curr_res = MSM_SENSOR_INVALID_RES;
		s_ctrl->cam_mode = mode;

		if (s_ctrl->is_csic ||
			!s_ctrl->sensordata->csi_if)
			rc = s_ctrl->func_tbl->sensor_csi_setting(s_ctrl,
				MSM_SENSOR_REG_INIT, 0);
		else
			rc = s_ctrl->func_tbl->sensor_setting(s_ctrl,
				MSM_SENSOR_REG_INIT, 0);
		//enable_irq(s_ctrl->sensor_i2c_client->client->irq);
	}
	return rc;
}
int32_t m9mo_power_up(struct msm_sensor_ctrl_t *s_ctrl)
{
	int32_t rc = 0;
	struct msm_camera_sensor_info *data = s_ctrl->sensordata;
	m9m0_bug("%s: %d\n", __func__, __LINE__);
	if (data->sensor_platform_info->ext_power_ctrl != NULL)
		data->sensor_platform_info->ext_power_ctrl(1);
	s_ctrl->reg_ptr = kzalloc(sizeof(struct regulator *)
			* data->sensor_platform_info->num_vreg, GFP_KERNEL);
	
	if (!s_ctrl->reg_ptr) {
		m9m0_bug("%s: could not allocate mem for regulators\n",
			__func__);
		return -ENOMEM;
	}
	
	rc = msm_camera_request_gpio_table(data, 1);
	if (rc < 0) {
		m9m0_bug("%s: request gpio failed\n", __func__);
		goto request_gpio_failed;
	}

	rc = msm_camera_config_vreg(&s_ctrl->sensor_i2c_client->client->dev,
		s_ctrl->sensordata->sensor_platform_info->cam_vreg,
		s_ctrl->sensordata->sensor_platform_info->num_vreg,
		s_ctrl->vreg_seq,
		s_ctrl->num_vreg_seq,
		s_ctrl->reg_ptr, 1);

	if (rc < 0) {
		m9m0_bug("%s: regulator on failed\n", __func__);
		goto config_vreg_failed;
	}
	rc = msm_camera_enable_vreg(&s_ctrl->sensor_i2c_client->client->dev,
			s_ctrl->sensordata->sensor_platform_info->cam_vreg,
			s_ctrl->sensordata->sensor_platform_info->num_vreg,
		    s_ctrl->vreg_seq,
		    s_ctrl->num_vreg_seq,
			s_ctrl->reg_ptr, 1);
	if (rc < 0) {
		m9m0_bug("%s: enable regulator failed\n", __func__);
		goto enable_vreg_failed;
	}

	if (s_ctrl->clk_rate != 0)
		cam_8960_clk_info->clk_rate = s_ctrl->clk_rate;

	rc = msm_cam_clk_enable(&s_ctrl->sensor_i2c_client->client->dev,
		cam_8960_clk_info, s_ctrl->cam_clk, ARRAY_SIZE(cam_8960_clk_info), 1);
	if (rc < 0) {
		m9m0_bug("%s: clk enable failed\n", __func__);
		goto enable_clk_failed;
	}
	
    rc = msm_camera_config_gpio_table(data, 1);
	if (rc < 0) {
		m9m0_bug("%s: config gpio failed\n", __func__);
		goto config_gpio_failed;
	}
	if (!s_ctrl->power_seq_delay)
		usleep_range(1000, 2000);
	else if (s_ctrl->power_seq_delay < 20)
		usleep_range((s_ctrl->power_seq_delay * 1000),
			((s_ctrl->power_seq_delay * 1000) + 1000));
	else
		msleep(s_ctrl->power_seq_delay);

	if (data->sensor_platform_info->i2c_conf &&
		data->sensor_platform_info->i2c_conf->use_i2c_mux)
		m9mo_sensor_enable_i2c_mux(data->sensor_platform_info->i2c_conf);

	camera_work = true;
	return rc;
config_gpio_failed:
	msm_cam_clk_enable(&s_ctrl->sensor_i2c_client->client->dev,
		cam_8960_clk_info, s_ctrl->cam_clk, ARRAY_SIZE(cam_8960_clk_info), 0);
enable_clk_failed:
    msm_camera_enable_vreg(&s_ctrl->sensor_i2c_client->client->dev,
			s_ctrl->sensordata->sensor_platform_info->cam_vreg,
			s_ctrl->sensordata->sensor_platform_info->num_vreg,
		        s_ctrl->vreg_seq,
		        s_ctrl->num_vreg_seq,
			s_ctrl->reg_ptr, 0);
enable_vreg_failed:
	msm_camera_config_vreg(&s_ctrl->sensor_i2c_client->client->dev,
		s_ctrl->sensordata->sensor_platform_info->cam_vreg,
		s_ctrl->sensordata->sensor_platform_info->num_vreg,
		s_ctrl->vreg_seq,
		s_ctrl->num_vreg_seq,
		s_ctrl->reg_ptr, 0);
config_vreg_failed:
	msm_camera_request_gpio_table(data, 0);
request_gpio_failed:
	kfree(s_ctrl->reg_ptr);
	return rc;

}
int32_t m9mo_power_down(struct msm_sensor_ctrl_t *s_ctrl)
{
	struct msm_camera_sensor_info *data = s_ctrl->sensordata;
	m9m0_bug("%s\n", __func__);
	disable_irq_nosync(s_ctrl->sensor_i2c_client->client->irq);
	flash_state = 0;
	if (data->sensor_platform_info->ext_power_ctrl != NULL)
		data->sensor_platform_info->ext_power_ctrl(0);
	if (data->sensor_platform_info->i2c_conf &&
		data->sensor_platform_info->i2c_conf->use_i2c_mux)
		m9mo_sensor_disable_i2c_mux(
			data->sensor_platform_info->i2c_conf);
	msm_cam_clk_enable(&s_ctrl->sensor_i2c_client->client->dev,
		cam_8960_clk_info, s_ctrl->cam_clk, ARRAY_SIZE(cam_8960_clk_info), 0);

	msm_camera_config_gpio_table(data, 0);

	msm_camera_enable_vreg(&s_ctrl->sensor_i2c_client->client->dev,
		s_ctrl->sensordata->sensor_platform_info->cam_vreg,
		s_ctrl->sensordata->sensor_platform_info->num_vreg,
		s_ctrl->vreg_seq,
		s_ctrl->num_vreg_seq,
		s_ctrl->reg_ptr, 0);
	msm_camera_config_vreg(&s_ctrl->sensor_i2c_client->client->dev,
		s_ctrl->sensordata->sensor_platform_info->cam_vreg,
		s_ctrl->sensordata->sensor_platform_info->num_vreg,
		s_ctrl->vreg_seq,
		s_ctrl->num_vreg_seq,
		s_ctrl->reg_ptr, 0);
	msm_camera_request_gpio_table(data, 0);
	if (data->sensor_platform_info->ext_power_ctrl != NULL)
		data->sensor_platform_info->ext_power_ctrl(0);
	if(s_ctrl->reg_ptr)
	   kfree(s_ctrl->reg_ptr);
	need_check_FW = true;
	camera_work = false;
	return 0;
	
}

int32_t m9mo_match_id(struct msm_sensor_ctrl_t *s_ctrl)
{	
	int32_t rc = 0;
	
	DownloadFW(s_ctrl);
	if((s_ctrl->sensor_i2c_client->client->irq) && (!INT_installed))
	{
		//we need irq for sync pipe~
		disable_irq(s_ctrl->sensor_i2c_client->client->irq);
		rc = request_irq(s_ctrl->sensor_i2c_client->client->irq, m9mo_irq,  IRQF_TRIGGER_RISING,
			s_ctrl->sensor_i2c_client->client->name, s_ctrl);
		if(!rc)
		{
			INT_installed = true;
		}
		CTRL = s_ctrl;
		m9mo_status_workqueue = create_singlethread_workqueue("m9mo_status");
		INIT_WORK(&m9mo_status_work, m9mo_status_work_callback);
		caf_workqueue = create_singlethread_workqueue("oppo_caf");
		INIT_WORK(&caf_work, caf_work_callback);
	    //add by liubin for m9mo proc
	    m9mo_proc_init(s_ctrl);
		rc = msm_server_open_client(&queue_id);
	}
		
	return 0;
}

static int32_t m9mo_get_AF_state(struct msm_sensor_ctrl_t *s_ctrl, int *af_state)
{
	char resp[5] = {0x05,0x01,0x0A,0x03,0x01};
	
	//get af state
	msm_vendor_i2c_rxdata(s_ctrl->sensor_i2c_client,resp,5,2);
	switch (resp[1])
	{
		case 0x00:
			*af_state = 0;
			m9m0_bug("Now is at focusing \r\n");
			break;
		case 0x01:
			*af_state = 1;
			m9m0_bug("Focus success! \r\n");
			break;
		case 0x02:
			*af_state = 2;
			m9m0_bug("Focus failed \r\n");
			break;
		case 0x03:
			*af_state = 3;
			m9m0_bug("Focus stop at edge \r\n");
			break;
		default:
			break;
	}

	return 0;
}

static int32_t m9mo_set_AE_area(struct msm_sensor_ctrl_t *s_ctrl, struct cord *ae_area)
{
	char E_cmd[5] = {0x05,0x02,0x02,0x47,0x0F};
	char resp[5] = {0x05,0x01,0x02,0x46,0x01};
	
	uint32_t ae_x = ae_area->x* 256 / 1442;
	uint32_t ae_y = ae_area->y* 256 / 1080;
	uint32_t ae_width = 0x20;
	uint32_t ae_height = 0x20;

	if (ae_area->x > 1442 || ae_area->y > 1080 || ae_area->x <=0 || ae_area->y <=0)
	{
		m9m0_bug("[%s] Invalid ae area \r\n", __func__);
		return 0;
	}

	if (ae_x > 255-ae_width)
		ae_x = 255-ae_width;
	if (ae_y > 255-ae_height)
		ae_y = 255-ae_height;
	
	m9m0_bug("ae_area[%d, %d, %d, %d] \r\n", ae_area->x, ae_area->y, ae_area->dx, ae_area->dy);
	//set AE strength
	E_cmd[2] = 0x02;
	E_cmd[3] = 0x47;
	E_cmd[4] = 0x0F;
	msm_camera_i2c_txdata(s_ctrl->sensor_i2c_client,E_cmd,5);

	//Stop touch AE
	E_cmd[2] = 0x02;
	E_cmd[3] = 0x46;
	E_cmd[4] = 0x00;
	msm_camera_i2c_txdata(s_ctrl->sensor_i2c_client,E_cmd,5);

	//set location of touch AE area
	E_cmd[2] = 0x02;
	E_cmd[3] = 0x3E;
	E_cmd[4] = ae_x;
	msm_camera_i2c_txdata(s_ctrl->sensor_i2c_client,E_cmd,5);
	E_cmd[2] = 0x02;
	E_cmd[3] = 0x3F;
	E_cmd[4] = ae_y;
	msm_camera_i2c_txdata(s_ctrl->sensor_i2c_client,E_cmd,5);

	//set frame size of touch AE area
	E_cmd[2] = 0x02;
	E_cmd[3] = 0x40;
	E_cmd[4] = ae_width;
	msm_camera_i2c_txdata(s_ctrl->sensor_i2c_client,E_cmd,5);
	E_cmd[2] = 0x02;
	E_cmd[3] = 0x41;
	E_cmd[4] = ae_height;
	msm_camera_i2c_txdata(s_ctrl->sensor_i2c_client,E_cmd,5);

	//start touch AE
	E_cmd[2] = 0x02;
	E_cmd[3] = 0x46;
	E_cmd[4] = 0x01;
	msm_camera_i2c_txdata(s_ctrl->sensor_i2c_client,E_cmd,5);

	//check if Touch AE is starting
	msm_vendor_i2c_rxdata(s_ctrl->sensor_i2c_client,resp,5,2);
	if (0x01 == resp[1])
	{
		m9m0_bug("Touch AE is starting \r\n");
		return 0;
	}
	else
	{
		m9m0_bug("Touch AE is NOT starting, Please check the AE area \r\n");
		return -1;
	}
}

static int32_t m9mo_set_AF_area(struct msm_sensor_ctrl_t *s_ctrl, struct cord *af_area)
{
	char E_cmd[5] = {0x05,0x02,0x0A,0x21,0x04};
	char resp[5] = {0x05,0x01,0x02,0x48,0x01};
	
	uint32_t af_x = af_area->x* 256 / 1442;
	uint32_t af_y = af_area->y* 256 / 1080;
	uint32_t af_width = 0x20;
	uint32_t af_height = 0x20;

	if (af_area->x > 1442 || af_area->y > 1080 ||
		af_area->x <= 0 || af_area->y <= 0)
	{
		m9m0_bug("[%s] Invalid af area \r\n", __func__);
		return 0;
	}
	
	if (af_x > 255-af_width)
		af_x = 255-af_width;
	if (af_y > 255-af_height)
		af_y = 255-af_height;
	
	m9m0_bug("af_area[%d, %d, %d, %d] \r\n", af_area->x, af_area->y, af_area->dx, af_area->dy);
	m9m0_bug("af_x [%d], af_y [%d] \r\n", af_x, af_y);
	//set AF window to use touch window
	E_cmd[2] = 0x0A;
	E_cmd[3] = 0x21;
	E_cmd[4] = 0x04;
	msm_camera_i2c_txdata(s_ctrl->sensor_i2c_client,E_cmd,5);

	//Stop touch AF
	E_cmd[2] = 0x02;
	E_cmd[3] = 0x48;
	E_cmd[4] = 0x00;
	msm_camera_i2c_txdata(s_ctrl->sensor_i2c_client,E_cmd,5);

	//set location of touch AF area
	E_cmd[2] = 0x02;
	E_cmd[3] = 0x42;
	E_cmd[4] = af_x;
	msm_camera_i2c_txdata(s_ctrl->sensor_i2c_client,E_cmd,5);
	E_cmd[2] = 0x02;
	E_cmd[3] = 0x43;
	E_cmd[4] = af_y;
	msm_camera_i2c_txdata(s_ctrl->sensor_i2c_client,E_cmd,5);

	//set frame size of touch AF area
	E_cmd[2] = 0x02;
	E_cmd[3] = 0x44;
	E_cmd[4] = af_width;
	msm_camera_i2c_txdata(s_ctrl->sensor_i2c_client,E_cmd,5);
	E_cmd[2] = 0x02;
	E_cmd[3] = 0x45;
	E_cmd[4] = af_height;
	msm_camera_i2c_txdata(s_ctrl->sensor_i2c_client,E_cmd,5);

	//start touch AF
	E_cmd[2] = 0x02;
	E_cmd[3] = 0x48;
	E_cmd[4] = 0x01;
	msm_camera_i2c_txdata(s_ctrl->sensor_i2c_client,E_cmd,5);
	
	//check if Touch AF is starting
	msm_vendor_i2c_rxdata(s_ctrl->sensor_i2c_client,resp,5,2);
	if (0x01 == resp[1])
	{
		m9m0_bug("Touch AF is starting \r\n");
		return 0;
	}
	else
	{
		m9m0_bug("Touch AF is NOT starting, Please check the AF area \r\n");
		return -1;
	}
	
}

static void m9mo_set_flash_mode(struct msm_sensor_ctrl_t *s_ctrl, int flash_mode)
{
	char E_cmd[5] = {0x05,0x02,0x0B,0x1F,0x00};
	m9m0_bug("%s flash_mode = %d\n", __func__,flash_mode);
	switch(flash_mode)
	{
		case 0:
			E_cmd[4] = 0x00;
			msm_camera_i2c_txdata(s_ctrl->sensor_i2c_client,E_cmd,5);
			E_cmd[2] = 0x0D;
			E_cmd[3] = 0x28;
			E_cmd[4] = 0x00;
			msm_camera_i2c_txdata(s_ctrl->sensor_i2c_client,E_cmd,5);
			flash_state = 0;
		  break;
		  
		case 1:
			E_cmd[4] = 0x02;
			msm_camera_i2c_txdata(s_ctrl->sensor_i2c_client,E_cmd,5);
			E_cmd[2] = 0x0D;
			E_cmd[3] = 0x28;
			E_cmd[4] = 0x00;
			msm_camera_i2c_txdata(s_ctrl->sensor_i2c_client,E_cmd,5);
			flash_state = 1;
		  break;
		  
		case 2:
			E_cmd[4] = 0x03;
			msm_camera_i2c_txdata(s_ctrl->sensor_i2c_client,E_cmd,5);
			E_cmd[2] = 0x0D;
			E_cmd[3] = 0x28;
			E_cmd[4] = 0x00;
			msm_camera_i2c_txdata(s_ctrl->sensor_i2c_client,E_cmd,5);
			flash_state = 1;
		   break;
        case 3:
			E_cmd[2] = 0x0D;
			E_cmd[3] = 0x28;
			E_cmd[4] = 0x01;
			msm_camera_i2c_txdata(s_ctrl->sensor_i2c_client,E_cmd,5);
			flash_state = 0;
			break;
		default:
				break;
	}
}
static int32_t m9mo_set_brightness(struct msm_sensor_ctrl_t *s_ctrl, uint8_t val)
{
	m9m0_bug("%s val = %d\n", __func__,val);
	return 0;	
}
static int32_t m9mo_set_contrast(struct msm_sensor_ctrl_t *s_ctrl, uint8_t val)
{
	m9m0_bug("%s val = %d\n", __func__,val);
	return 0;	
}
static int32_t m9mo_set_zoom(struct msm_sensor_ctrl_t *s_ctrl, uint8_t val)
{
	m9m0_bug("%s val = %d\n", __func__,val);
	return 0;	
}

static int32_t m9mo_set_wb(struct msm_sensor_ctrl_t *s_ctrl, uint8_t val)
{
	char E_cmd[5] = {0x05,0x02,0x06,0x02,0x01};
	
	m9m0_bug("%s val = %d\n", __func__,val);
	switch (val)
	{
		case 1:
		{
			m9m0_bug("[%s]-----WB : Auto \r\n", __func__);
			E_cmd[4] = 0x01;
			msm_camera_i2c_txdata(s_ctrl->sensor_i2c_client,E_cmd,5);
			break;
		}
		case 3:
		{
			m9m0_bug("[%s]-----WB : Incandescent light \r\n", __func__);
			E_cmd[4] = 0x02;
			msm_camera_i2c_txdata(s_ctrl->sensor_i2c_client,E_cmd,5);
			E_cmd[2] = 0x06;
			E_cmd[3] = 0x03;
			E_cmd[4] = 0x01;
			msm_camera_i2c_txdata(s_ctrl->sensor_i2c_client,E_cmd,5);
			break;
		}
		case 4:
		{
			m9m0_bug("[%s]-----WB : Fluorescent light \r\n", __func__);
			E_cmd[4] = 0x02;
			msm_camera_i2c_txdata(s_ctrl->sensor_i2c_client,E_cmd,5);
			E_cmd[2] = 0x06;
			E_cmd[3] = 0x03;
			E_cmd[4] = 0x02;
			msm_camera_i2c_txdata(s_ctrl->sensor_i2c_client,E_cmd,5);
			break;
		}
		case 5:
		{
			m9m0_bug("[%s]-----WB : Day light \r\n", __func__);
			E_cmd[4] = 0x02;
			msm_camera_i2c_txdata(s_ctrl->sensor_i2c_client,E_cmd,5);
			E_cmd[2] = 0x06;
			E_cmd[3] = 0x03;
			E_cmd[4] = 0x04;
			msm_camera_i2c_txdata(s_ctrl->sensor_i2c_client,E_cmd,5);
			break;
		}
		case 6:
		{
			m9m0_bug("[%s]-----WB : Cloudy \r\n", __func__);
			E_cmd[4] = 0x02;
			msm_camera_i2c_txdata(s_ctrl->sensor_i2c_client,E_cmd,5);
			E_cmd[2] = 0x06;
			E_cmd[3] = 0x03;
			E_cmd[4] = 0x05;
			msm_camera_i2c_txdata(s_ctrl->sensor_i2c_client,E_cmd,5);
			break;
		}
		default:
			break;
	}
	
	return 0;	
}
static int32_t m9mo_set_antibanding(struct msm_sensor_ctrl_t *s_ctrl, uint16_t val)
{
	m9m0_bug("%s val = %d\n", __func__,val);
	return 0;	
}
static int32_t m9mo_set_saturation(struct msm_sensor_ctrl_t *s_ctrl, uint8_t val)
{
	m9m0_bug("%s val = %d\n", __func__,val);
	return 0;	
}
static int32_t m9mo_set_sharpness(struct msm_sensor_ctrl_t *s_ctrl, int8_t val)
{
	m9m0_bug("%s val = %d\n", __func__,val);
	return 0;	
}
static int32_t m9mo_set_EV(struct msm_sensor_ctrl_t *s_ctrl, int8_t val)
{
	m9m0_bug("%s val = %d\n", __func__,val);
	return 0;	
}
static int32_t m9mo_set_ISO(struct msm_sensor_ctrl_t *s_ctrl, uint8_t val)
{
	char E_cmd[5] = {0x05,0x02,0x03,0x05,0x01};
	if((0 <= val)&&(val <= 7 ))
	{
		E_cmd[4] = val;
	}else{
	    E_cmd[4] = 0;
	}
	msm_camera_i2c_txdata(s_ctrl->sensor_i2c_client,E_cmd,5);
	m9m0_bug("%s val = %d\n", __func__,val);
	return 0;		
}
static int32_t m9mo_set_effect(struct msm_sensor_ctrl_t *s_ctrl, int8_t val)
{
	m9m0_bug("%s val = %d\n", __func__,val);
	return 0;		
}
static int32_t m9mo_set_autofocus(struct msm_sensor_ctrl_t *s_ctrl)
{
	
	char E_cmd[5] = {0x05,0x02,0x0A,0x02,0x01};

	E_cmd[2] = 0x0A;
	E_cmd[3] = 0x02;
	E_cmd[4] = 0x01;
	msm_camera_i2c_txdata(s_ctrl->sensor_i2c_client,E_cmd,5);
	m9m0_bug("%s \n", __func__);
	
	return 0;		
}

static int32_t m9mo_set_slow_shutter(struct msm_sensor_ctrl_t *s_ctrl, u_int32_t slow_shutter)
{
	char E_cmd[5] = {0x05,0x02,0x03,0x0A,0x01};

	if (g_bSlowShutter == slow_shutter)
		return 0;

	m9mo_change_to_param_set_mode(s_ctrl);
	
	if (slow_shutter)
	{
		E_cmd[2] = 0x03;
		E_cmd[3] = 0x0A;
		E_cmd[4] = 0x16;
		msm_camera_i2c_txdata(s_ctrl->sensor_i2c_client,E_cmd,5);
		E_cmd[2] = 0x03;
		E_cmd[3] = 0x0B;
		E_cmd[4] = 0x16;
		msm_camera_i2c_txdata(s_ctrl->sensor_i2c_client,E_cmd,5);
		normal_preview_output(s_ctrl);
	}
	else
	{
		E_cmd[2] = 0x03;
		E_cmd[3] = 0x0A;
		E_cmd[4] = 0x00;
		msm_camera_i2c_txdata(s_ctrl->sensor_i2c_client,E_cmd,5);
		E_cmd[2] = 0x03;
		E_cmd[3] = 0x0B;
		E_cmd[4] = 0x00;
		msm_camera_i2c_txdata(s_ctrl->sensor_i2c_client,E_cmd,5);
		ZSL_preview_output(s_ctrl);
	}
	g_bSlowShutter = slow_shutter;
	m9m0_bug("%s \n", __func__);
	return 0;	
}


int32_t m9mo_config(struct msm_sensor_ctrl_t *s_ctrl, void __user *argp)
{
	struct sensor_cfg_data cdata;
	long   rc = 0;
	if (copy_from_user(&cdata,
		(void *)argp,
		sizeof(struct sensor_cfg_data)))
		return -EFAULT;
	//mutex_lock(s_ctrl->msm_sensor_mutex);
	/*m9m0_bug("%s:%d %s cfgtype = %d\n", __func__, __LINE__,
		s_ctrl->sensordata->sensor_name, cdata.cfgtype);*/
	switch (cdata.cfgtype) {
		case CFG_FRAME_NOTIFICATION:
			if(caf_workqueue)
				queue_work(caf_workqueue, &caf_work);
			break;
		case CFG_SET_AF_MODE:
			break;
		case CFG_SET_FLASH_MODE:
			m9mo_set_flash_mode(s_ctrl, cdata.cfg.flash_mode);
			break;
		case CFG_GET_FLASH_STATE:
			cdata.cfg.flash_mode = flash_state;
			if(flash_state)
			{
				if(g_bCapture_raw)
				{	
				  raw_capture(s_ctrl);
				}else{		
				  ZSL_capture(s_ctrl);
				}
			}
			if (copy_to_user((void *)argp,
		                           &cdata,
		    sizeof(struct sensor_cfg_data)))
	        {
		        rc = -EFAULT;
	        }
			break;
		case CFG_SET_AE_MODE:
			break;
		case CFG_SET_AF_AREA:
			rc = m9mo_set_AF_area(s_ctrl, &(cdata.cfg.af_cord));
			break;
		case CFG_GET_AF_STATE:
			rc = m9mo_get_AF_state(s_ctrl, &(cdata.cfg.af_state));
			break;
		case CFG_SET_AE_AREA:
			rc = m9mo_set_AE_area(s_ctrl, &(cdata.cfg.aec_cord));
			break;
		case CFG_SET_BRIGHTNESS:
			rc = m9mo_set_brightness(s_ctrl, cdata.cfg.brightness);
			break;
		case CFG_SET_CONTRAST:
			rc = m9mo_set_contrast(s_ctrl, cdata.cfg.contrast);
			break;
		case CFG_SET_ZOOM:
			rc = m9mo_set_zoom(s_ctrl, cdata.cfg.zoom);
			break;
		case CFG_SET_WB:
			rc = m9mo_set_wb(s_ctrl, cdata.cfg.wb_val);
			break;
		case CFG_SET_ANTIBANDING:
			rc = m9mo_set_antibanding(s_ctrl, cdata.cfg.antibanding);
			break;
		case CFG_SET_SATURATION:
			rc = m9mo_set_saturation(s_ctrl, cdata.cfg.saturation);
			break;
		case CFG_SET_SHARPNESS:
			rc = m9mo_set_sharpness(s_ctrl, cdata.cfg.sharpness);
			break;
		case CFG_SET_EXPOSURE_COMPENSATION:
			rc = m9mo_set_EV(s_ctrl, cdata.cfg.exp_compensation);
			break;
		case CFG_SET_ISO:
			rc = m9mo_set_ISO(s_ctrl, cdata.cfg.iso_type);
			break;
		case CFG_SET_EFFECT:
			rc = m9mo_set_effect(s_ctrl, cdata.cfg.effect);
			break;
		case CFG_SET_AUTO_FOCUS:
			rc = m9mo_set_autofocus(s_ctrl);
			break;
		case CFG_SET_SLOW_SHUTTER:
			m9m0_bug("[%s] slow shutter : %d \r\n", __func__, cdata.cfg.slow_shutter);
			rc = m9mo_set_slow_shutter(s_ctrl, cdata.cfg.slow_shutter);
			break;
		case CFG_SET_HDR_ENABLE:
			m9m0_bug("[%s] HDR capture enable : %d \r\n", __func__, cdata.cfg.hdr_enable);
			g_bHDREnable = cdata.cfg.hdr_enable;
			break;
		default:
			//mutex_unlock(s_ctrl->msm_sensor_mutex);
			rc = msm_sensor_config(s_ctrl,argp);
	}
	//mutex_unlock(s_ctrl->msm_sensor_mutex);
	return rc;
}
static struct msm_sensor_fn_t m9mo_func_tbl = {
	.sensor_start_stream = m9mo_start_stream,
	.sensor_stop_stream = m9mo_stop_stream,
	.sensor_setting = m9mo_setting,
	.sensor_set_sensor_mode = msm_set_m9mo_mode,
	.sensor_mode_init = m9mo_mode_init,
	.sensor_get_output_info = msm_sensor_get_output_info,
	.sensor_config = m9mo_config,
	.sensor_power_up = m9mo_power_up,
	.sensor_power_down = m9mo_power_down,
	.sensor_get_csi_params = msm_sensor_get_csi_params,
	.sensor_match_id = m9mo_match_id,
};

static struct msm_sensor_reg_t m9mo_regs = {
	.default_data_type = MSM_CAMERA_I2C_BYTE_DATA,
	.start_stream_conf = NULL,
	.start_stream_conf_size = 0,
	.stop_stream_conf = NULL,
	.stop_stream_conf_size = 0,
	.group_hold_on_conf = NULL,
	.group_hold_on_conf_size = 0,
	.group_hold_off_conf = NULL,
	.group_hold_off_conf_size = 0,
	.init_settings = NULL,
	.init_size = 0,
	.mode_settings = 0,
	.output_settings = &m9mo_dimensions[0],
	.num_conf = ARRAY_SIZE(m9mo_dimensions),
};

static struct msm_sensor_ctrl_t m9mo_s_ctrl = {
	.msm_sensor_reg = &m9mo_regs,
	.sensor_i2c_client = &m9mo_sensor_i2c_client,
	.sensor_i2c_addr = 0x3e,
	.sensor_output_reg_addr = NULL,
	.sensor_id_info = &m9mo_id_info,
	.sensor_exp_gain_info = NULL,
	.cam_mode = MSM_SENSOR_MODE_INVALID,
	//.csi_params = &m9mo_csi_params_array[0],
	.msm_sensor_mutex = &m9mo_mut,
	.sensor_i2c_driver = &m9mo_i2c_driver,
	.sensor_v4l2_subdev_info = m9mo_subdev_info,
	.sensor_v4l2_subdev_info_size = ARRAY_SIZE(m9mo_subdev_info),
	.sensor_v4l2_subdev_ops = &m9mo_subdev_ops,
	.func_tbl = &m9mo_func_tbl,
	.clk_rate = MSM_SENSOR_MCLK_24HZ,
};

module_init(m9mo_sensor_init_module);
MODULE_DESCRIPTION("fujitsu common ISP driver");
MODULE_LICENSE("GPL v2");

