/* Copyright (c) 2012, The Linux Foundation. All rights reserved.
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

#include "msm_fb.h"
#include "mipi_dsi.h"
#include "mipi_orise.h"
#include <linux/gpio.h>
#include <mach/device_info.h>

//Neal
#define LCD_DIS_GPIO  70

static char *DEVICE_VERSION = "63311";
static char *DEVICE_MANUFACUTRE_sharp = "sharp";
static char *DEVICE_MANUFACUTRE_jdi = "jdi";


static bool dis = 1;

static struct msm_panel_info pinfo;

bool get_panel_info(void)
{

	return dis;
}

static struct mipi_dsi_phy_ctrl dsi_video_mode_phy_db_1080p = { 
	/* 1920*1200, RGB888, 4 Lane 60 fps video mode */ 
	/* regulator */ 
	{0x09, 0x08, 0x05, 0x00, 0x20},
	/* timing */ 
	{0xe9, 0x5e, 0x2c, 0x00, 0x7b, 0x7c, 0x30, 0x61, 
	0x42, 0x03, 0x04, 0xa0}, 
	/* phy ctrl */ 
	{0x5f, 0x00, 0x00, 0x10}, 
	/* strength */ 
	{0xff, 0x00, 0x06, 0x00}, 
	/* pll control */ 
	{0x0, 0x49, 0x30, 0xc4, 0x00, 0x20, 0x07, 0x62,
	0x41, 0x0f, 0x01, 
	0x00, 0x14, 0x03, 0x00, 0x02, 0x00, 0x20, 0x00, 0x01 }, 
}; 

static int __init mipi_video_orise_720p_pt_init(void)
{
	int ret;
	int rc = 0;
	if (msm_fb_detect_client("mipi_video_orise_720p"))
		return 0;
/*OPPO Neal add for sharp panel*/
	 rc = gpio_request(LCD_DIS_GPIO, "LCD_DIS_GPIO");
        if (rc < 0)
        {
            pr_err("MIPI GPIO LCD_TE_GPIO request failed: %d\n", rc);
            return -ENODEV;
        }
	 gpio_direction_output(LCD_DIS_GPIO,0);
	
	 dis = gpio_get_value(LCD_DIS_GPIO);
	 printk(KERN_ERR "mipi_orise_lcd_probe Neal ******************************: dis = %d\n", dis);
	 gpio_free(LCD_DIS_GPIO);
/*OPPO Neal add end*/

	pinfo.xres = 1080;
	pinfo.yres = 1920;
	pinfo.lcdc.xres_pad = 0;
	pinfo.lcdc.yres_pad = 0;

	pinfo.type = MIPI_VIDEO_PANEL;
	pinfo.pdest = DISPLAY_1;
	pinfo.wait_cycle = 0;
	pinfo.bpp = 24;
#if 1		
	/* OPPO 2013-03-07 Gousj Modify begin for solve the issue of lack of virtical pixel. */	
		//Gousj modified  h_back_porch  from 100 to 101 	
	/*OPPO Neal add for sharp panel*/

	if(get_panel_info() == 0)
		{
			pinfo.lcdc.h_back_porch = 100;//100;//80;
			pinfo.lcdc.v_back_porch = 5;	//must > 4,Otherwise,it will increase the burden of clock	huyu
/* OPPO 2013-11-13 gousj Add begin for device information */
#ifdef CONFIG_VENDOR_EDIT
			register_device_proc("lcd", DEVICE_VERSION, DEVICE_MANUFACUTRE_sharp);
#endif
/* OPPO 2013-11-13 gousj Add end */
			printk(KERN_ERR "%s: register sharp device!\n", __func__);
		}
	else
		{
			pinfo.lcdc.h_back_porch = 101;//100;//80;
			pinfo.lcdc.v_back_porch = 4;	//must > 4,Otherwise,it will increase the burden of clock	huyu
/* OPPO 2013-11-13 gousj Add begin for device information */
#ifdef CONFIG_VENDOR_EDIT
			register_device_proc("lcd", DEVICE_VERSION, DEVICE_MANUFACUTRE_jdi);
#endif
/* OPPO 2013-11-13 gousj Add end */
			printk(KERN_ERR "%s: register jdi device!\n", __func__);
		}	
	/*OPPO Neal add end*/

		pinfo.lcdc.h_front_porch = 130;//120		
		pinfo.lcdc.h_pulse_width = 8;		
		//Modified by Gousj on date 2013-3-4 ,the value of v_back_porch decreased from 5 to 4 .		
				
		pinfo.lcdc.v_front_porch = 3;		
		//Modified by Gousj on date 2013-3-4 ,the value of v_pulse_width decreased from 2 to 1 .		
		pinfo.lcdc.v_pulse_width = 1;//2;		
	/* OPPO 2013-03-07 Gousj Modify end */
#endif
	pinfo.lcdc.border_clr = 0;	/* blk */
	pinfo.lcdc.underflow_clr = 0xff;	/* blue */
	pinfo.lcdc.hsync_skew = 0;
	/* OPPO 2013-04-22 Gousj Modify for black light not light */
	pinfo.bl_max = 127;
	/* OPPO 2013-04-022 Gousj Modify end */
	pinfo.bl_min = 1;
	pinfo.fb_num = 2;

	pinfo.mipi.mode = DSI_VIDEO_MODE;
	pinfo.mipi.pulse_mode_hsa_he = FALSE;
	pinfo.mipi.hfp_power_stop = TRUE;
	pinfo.mipi.hbp_power_stop = FALSE;
	pinfo.mipi.hsa_power_stop = FALSE;
	pinfo.mipi.eof_bllp_power_stop = FALSE;
/* OPPO 2013-10-10 gousj Add begin for sent mipi cmd in HS mode */
#ifdef CONFIG_VENDOR_EDIT
	pinfo.mipi.bllp_power_stop = TRUE;
#endif
/* OPPO 2013-10-10 gousj Add end */
	pinfo.mipi.traffic_mode = DSI_BURST_MODE;
	pinfo.mipi.dst_format = DSI_VIDEO_DST_FORMAT_RGB888;
	pinfo.mipi.vc = 0;
	pinfo.mipi.rgb_swap = DSI_RGB_SWAP_RGB;
	pinfo.mipi.data_lane0 = TRUE;
	pinfo.mipi.data_lane1 = TRUE;
	pinfo.mipi.data_lane2 = TRUE;
	pinfo.mipi.data_lane3 = TRUE;
	pinfo.mipi.t_clk_post = 0x25;//0x19;
	pinfo.mipi.t_clk_pre = 0x36;//0x37;
	pinfo.mipi.stream = 0; /* dma_p */
	pinfo.mipi.mdp_trigger = 0;
	pinfo.mipi.dma_trigger = DSI_CMD_TRIGGER_SW;
	pinfo.mipi.frame_rate = 60;
	pinfo.mipi.dsi_phy_db = &dsi_video_mode_phy_db_1080p;
	pinfo.mipi.tx_eot_append = TRUE;
	pinfo.mipi.esc_byte_ratio = 4;

	ret = mipi_orise_device_register(&pinfo, MIPI_DSI_PRIM,
						MIPI_DSI_PANEL_720P_PT);
	if (ret)
		printk(KERN_ERR "%s: failed to register device!\n", __func__);

	return ret;
}

module_init(mipi_video_orise_720p_pt_init);
