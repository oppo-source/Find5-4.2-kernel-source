/**************************************************************
* Copyright (c)  200X- 2013  Oppo Mobile communication Corp.ltd.¡ê?
* VENDOR_EDIT
*File       : lm3630a.c
* Description: Source file for backlight IC LM630.
*           To control backlight brightness.
* Version   : 1.0
* Date      : 2013-5
* Author    : goushengjun@oppo.com
* ---------------------------------- Revision History: ----------------------------------
*   <version>   <date>      < author >          <desc>
****************************************************************/

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/device.h>
#include <mach/gpio.h>
#include <linux/regulator/consumer.h>
#include <mach/vreg.h>
#include <linux/err.h>
#include <linux/i2c/lm3630_bl.h>
#include <mach/device_info.h>

#define LM3630_DRV_NAME "lm3630"
#define SSL3252_CHIP_ID     0x41
#define LM3630_GP           0x10
#define LM3630_GP_MODE      0xc7
#define LM3630_BMAIN        0xa0
#define LM3630_BMAIN_MAX    127

#define LM3630_I2C_READ

extern int get_boot_mode(void);
enum
{
    MSM_BOOT_MODE__NORMAL,
    MSM_BOOT_MODE__FASTBOOT,
    MSM_BOOT_MODE__RECOVERY,
    MSM_BOOT_MODE__FACTORY,
    MSM_BOOT_MODE__RF,
    MSM_BOOT_MODE__WLAN,
    MSM_BOOT_MODE__CHARGE,
};

static char *DEVICE_VERSION = "3630";
static char *DEVICE_MANUFACUTRE = "ti";
static bool sleep_mode = true;

static struct i2c_client *lm3630_client;

#ifdef LM3630_I2C_READ
static int lm3630_i2c_rxdata(unsigned char saddr, unsigned char *rxdata, int length)
{
    int rc;
    int retry_cnt = 0;
    struct i2c_msg msgs[] =
    {
        {
            .addr  = saddr,
            .flags = 0,
            .len   = 1,
            .buf   = rxdata,
        },
        {
            .addr  = saddr,
            .flags = I2C_M_RD,
            .len   = length,
            .buf   = rxdata,
        },
    };

    do
    {
        rc = i2c_transfer(lm3630_client->adapter, msgs, 2);
        if (rc > 0)
            break;
        retry_cnt++;
    }
    while (retry_cnt < 3);

    if (rc < 0)
    {
        pr_err("%s: failed!:%d %d\n", __func__, rc, retry_cnt);
        return -EIO;
    }

    return 0;
}
#endif

static int32_t lm3630_i2c_txdata(unsigned short saddr,
                                 unsigned char *txdata, int length)
{
    int retry_cnt = 0;
    int rc;

    struct i2c_msg msg[] =
    {
        {
            .addr = saddr,
            .flags = 0,
            .len = length,
            .buf = txdata,
        },
    };

    do
    {
        rc = i2c_transfer(lm3630_client->adapter, msg, 1);
        if (rc > 0)
            break;
        retry_cnt++;
    }
    while (retry_cnt < 3);

    if (rc < 0)
    {
        pr_err("%s :failed: %d %d\n", __func__, rc, retry_cnt);
        return -EIO;
    }

    return 0;
}

#ifdef LM3630_I2C_READ
static int32_t lm3630_i2c_read(unsigned char raddr, unsigned char *rdata)
{
    int32_t rc = -EIO;
    unsigned char buf[1];

    if (!rdata)
        return -EIO;
    memset(buf, 0, sizeof(buf));
    buf[0] = raddr;
    rc = lm3630_i2c_rxdata(lm3630_client->addr, buf, 1);
    if (rc < 0)
    {
        pr_err("%s : read reg addr 0x%x failed!\n", __func__, raddr);
        return rc;
    }
    *rdata = buf[0];

    return rc;
}
#endif

static int lm3630_i2c_write(unsigned char   raddr, unsigned char  rdata)
{
    int32_t rc = -EIO;
    unsigned char buf[2];

    if (!rdata)
        return 0;
    memset(buf, 0, sizeof(buf));
    buf[0] = raddr;
    buf[1] = rdata;
    rc = lm3630_i2c_txdata(lm3630_client->addr , buf, 2);
    if (rc < 0)
        pr_err("%s :failed, addr = 0x%x, val = 0x%x!\n", __func__, raddr, rdata);

    return rc;
}

int lm3630_bkl_control(unsigned char bkl_level)
{
    int rc = 0;


    if (!lm3630_client)
    {
        pr_err("%s: lm3630_client == null!\n", __func__);
        return rc;
    }

    if(bkl_level == 0)
    {
        rc = lm3630_i2c_write(0x03, 0x00);
        rc = lm3630_i2c_write(0x00, 0x9f);
        sleep_mode = true;
        pr_debug("%s: Neal lm3630_client sleep rc = %d\n", __func__,rc);
        return rc;
    }

    if(sleep_mode == true)
    {
        pr_debug("%s Neal backlight IC weke up.\n",__func__);
        rc = lm3630_i2c_write(0x00, 0x1f);
        sleep_mode= false;
        mdelay(10);
        if(get_boot_mode() == MSM_BOOT_MODE__FACTORY)
        {
            rc = lm3630_i2c_write(0x01, 0x18);
        }
        else
        {
            rc = lm3630_i2c_write(0x01, 0x19);
        }
    }
    rc = lm3630_i2c_write(0x03, bkl_level);
    pr_debug("%s: Neal lm3630_client set bkl level = %d, read level after write = %d ,rc = %d\n", __func__,(int)bkl_level,lm3630_bkl_readout(),rc);

    return rc;
}

int lm3630_bkl_readout(void)
{
    int rc = 0;
#ifdef LM3630_I2C_READ
    unsigned char bkl_level = 0;
    rc = lm3630_i2c_read(0x03,&bkl_level);
    rc = bkl_level;
    pr_debug("%s Neal bkl read out = %d\n",__func__,(int)rc);
#endif
    return rc;
}

#define LM3630_ENABLE_GPIO   86
static int lm3630_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    int rc = 0;
    pr_debug("%s: Neal backlight prob.\n", __func__);

/* OPPO 2013-11-13 gousj Add begin for device information */
#ifdef CONFIG_VENDOR_EDIT
	register_device_proc("backlight", DEVICE_VERSION, DEVICE_MANUFACUTRE);
#endif
/* OPPO 2013-11-13 gousj Add end */

    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
    {
        pr_err("%s: i2c_check_functionality failed\n", __func__);
        rc = -EACCES;
        return rc;
    }

    lm3630_client = client;

    if (!lm3630_client)
    {
        pr_err("%s: lm3630_client == null!\n", __func__);
        rc = -ENODEV;
        goto exit;
    }

    rc = gpio_request(LM3630_ENABLE_GPIO, "lm3630_enable");
    if (rc)
    {
        pr_err("lm3630_enable gpio_request failed: %d\n", rc);
        goto exit;
    }
    rc = gpio_direction_output(LM3630_ENABLE_GPIO, 1);
    if (rc)
    {
        pr_err("%s: unable to enable!!!!!!!!!!!!\n", __func__);
        goto exit;
    }

    //initialize bkl to max when system boot

    rc = lm3630_i2c_write(0x00, 0x1f);
    rc = lm3630_i2c_write(0x50, 0x03);
    if(get_boot_mode() == MSM_BOOT_MODE__FACTORY)
    {
        rc = lm3630_i2c_write(0x01, 0x18);
    }
    else
    {
        rc = lm3630_i2c_write(0x01, 0x19);
    }
    rc = lm3630_i2c_write(0x02, 0x79);
    rc = lm3630_i2c_write(0x03, 0xff);
    rc = lm3630_i2c_write(0x05, 0x14);
    rc = lm3630_i2c_write(0x06, 0x14);
    rc = lm3630_i2c_write(0x07, 0x00);
    rc = lm3630_i2c_write(0x08, 0x00);
    rc = lm3630_i2c_write(0x09, 0xe3);
    rc = lm3630_i2c_write(0x10, 0xe3);
    rc = lm3630_i2c_write(0x11, 0xe3);
    rc = lm3630_i2c_write(0x12, 0xe3);
    rc = lm3630_i2c_write(0x13, 0xe3);

exit:
    if (rc)
        lm3630_client = 0;
    return rc;

}

static int lm3630_i2c_remove(struct i2c_client *client)
{
    return 0;
}
static int lm3630_suspend(struct i2c_client *client, pm_message_t mesg)
{
    int rc ;

    pr_debug("%s: Neal backlight suspend.\n", __func__);
    rc = gpio_direction_output(LM3630_ENABLE_GPIO, 0);
    if (rc)
    {
        pr_err("%s: unable to enable!!!!!!!!!!!!\n", __func__);
        return rc;
    }

    return 0;
}
static int lm3630_resume(struct i2c_client *client)
{
    int rc ;
    pr_debug("%s: Neal backlight resume.\n", __func__);

    rc = gpio_direction_output(LM3630_ENABLE_GPIO, 1);
    if (rc)
    {
        pr_err("%s: unable to enable!!!!!!!!!!!!\n", __func__);
        return rc;
    }

    mdelay(10);
    rc = lm3630_i2c_write(0x50, 0x03);
    rc = lm3630_i2c_write(0x01, 0x18);
    rc = lm3630_i2c_write(0x02, 0x79);
    rc = lm3630_i2c_write(0x05, 0x14);
    rc = lm3630_i2c_write(0x06, 0x14);
    rc = lm3630_i2c_write(0x07, 0x00);
    rc = lm3630_i2c_write(0x08, 0x00);

    return 0;
}

static const struct i2c_device_id lm3630_i2c_id[] =
{
    { LM3630_DRV_NAME, 0},
    {}
};

static struct i2c_driver lm3630_i2c_driver =
{
    .id_table = lm3630_i2c_id,
    .probe  = lm3630_i2c_probe,
    .remove = lm3630_i2c_remove,
    .suspend =lm3630_suspend,
    .resume = lm3630_resume,
    .driver = {
        .name = LM3630_DRV_NAME,
    },
};

static int __init lm3630_init(void)
{
    return i2c_add_driver(&lm3630_i2c_driver);
}

static void __exit lm3630_eixt(void)
{
    i2c_del_driver(&lm3630_i2c_driver);
}

module_init(lm3630_init);
module_exit(lm3630_eixt);

MODULE_DESCRIPTION("lm3630 I2C Driver");
MODULE_LICENSE("GPL");
