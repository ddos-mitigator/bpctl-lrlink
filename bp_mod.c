/**************************************************************************
Copyright lr-link.ltd.com
All rights reserved.
***************************************************************************/

#include <linux/version.h>
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,17))
    #include <linux/config.h>
#endif
#if defined(CONFIG_SMP) && ! defined(__SMP__)
    #define __SMP__
#endif

#include <linux/kernel.h> 
#include <linux/module.h> 
#include <linux/fs.h>
#include <linux/pci.h>
#include <linux/delay.h>
#include <linux/netdevice.h>
#include <linux/rtnetlink.h>
#include <linux/rcupdate.h>

#include <asm/uaccess.h>
#include <linux/sched.h>
#include <linux/ethtool.h>
#include <linux/proc_fs.h>
#include <linux/reboot.h>
#include <linux/etherdevice.h>
#include <linux/interrupt.h>
#include <linux/hardirq.h>

#include <linux/types.h>
#include <linux/list.h>
#include <linux/kobject.h>
#include <linux/sysfs.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/file.h>


#include "bp_ioctl.h"
#include "bp_mod.h"
#include "bypass.h"

  
#define SUCCESS 0
#define BP_MOD_VER  VER_STR_SET
#define BP_MOD_DESCR "LR-Link Bypass Nic driver"

static int Device_Open = 0;
static int major_num=0; 
spinlock_t bypass_wr_lock;


MODULE_AUTHOR("Joseph Lan@lr-link.com.cn");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION(BP_MOD_DESCR);
MODULE_VERSION(BP_MOD_VER);

//#define SEARCH_SS_DEVICE_00

#define KPRINTF_MAX 1024

#define LRBP_I350TP4_VENDOR 0x8086
#define LRBP_I350TP4_DEVICE 0x1521
#define LRBP_I350TP4_SUBVENDOR 0x1374
#define LRBP_I350TP4_SUBDEVICE 0x03d0

#define LRBP_X710FP4_VENDOR 0x8086
#define LRBP_X710FP4_DEVICE 0x1572
//#define LRBP_X710FP4_SUBVENDOR 0x1374
//#define LRBP_X710FP4_SUBDEVICE 0x0502
#define LRBP_X710FP4_SUBVENDOR 0x8086
#define LRBP_X710FP4_SUBDEVICE 0x0000

#define REG_X710_GLGEN_GPIO_CTL0     0x88100
#define REG_X710_GLGEN_GPIO_CTL1     0x88104
#define REG_X710_GLGEN_GPIO_CTL2     0x88108
#define REG_X710_GLGEN_GPIO_CTL3     0x8810c
#define REG_X710_GLGEN_GPIO_STAT     0x0008817C
#define REG_X710_GLGEN_GPIO_SET      0x00088184
//bit definition for REG_X710_GLGEN_GPIO_SET
#define GPIO_INDEX_0 0  //bit0-4
#define GPIO_INDEX_1 1
#define GPIO_INDEX_2 2
#define GPIO_INDEX_3 3
#define SDP_DATA_HI  0x20//bit5
#define SDP_DATA_LO  0x00
#define SDP_DRIVE    0x40//bit6
#define REG_X710_GLGEN_GPIO_TRANSIT  0x00088180

#define REG_X710_GLGEN_MDIO_I2C_SEL0 0x000881C0
#define REG_X710_GLGEN_MDIO_I2C_SEL1 0x000881C4
#define REG_X710_GLGEN_MDIO_I2C_SEL2 0x000881C8
#define REG_X710_GLGEN_MDIO_I2C_SEL3 0x000881CC
#define BIT_X710_SEL_MDIO 0x0000
#define BIT_X710_SEL_I2C  0x0001

#define REG_X710_GLGEN_I2C_CMD0 0x000881E0
#define REG_X710_GLGEN_I2C_CMD1 0x000881E4
#define REG_X710_GLGEN_I2C_CMD2 0x000881E8
#define REG_X710_GLGEN_I2C_CMD3 0x000881EC

#define REG_X710_GLGEN_I2C_PARAM0 0x000881AC
#define REG_X710_GLGEN_I2C_PARAM1 0x000881B0
#define REG_X710_GLGEN_I2C_PARAM2 0x000881B4
#define REG_X710_GLGEN_I2C_PARAM3 0x000881B8
#define BIT_X710_I2CBB_DIS      0x0000 //BIT 8
#define BIT_X710_I2CBB_EN       0x0100
#define BIT_X710_I2CCLK_OUT_0   0x0000 //BIT 9
#define BIT_X710_I2CCLK_OUT_1   0x0200 
#define BIT_X710_I2CCLK_OUT     0x0200 
#define BIT_X710_I2CDAT_OUT_0   0x0000 //BIT 10
#define BIT_X710_I2CDAT_OUT_1   0x0400
#define BIT_X710_I2CDAT_OUT     0x0400
#define BIT_X710_I2CDAT_OUT_EN  0x0000 //BIT 11,active low
#define BIT_X710_I2CDAT_OUT_DIS 0x0800 
#define BIT_X710_I2CDAT_IN      0x1000 //BIT 12,RO
#define BIT_X710_I2CCLK_OUT_EN  0x0000 //BIT 13,active low
#define BIT_X710_I2CCLK_OUT_DIS 0x2000 
#define BIT_X710_I2CCLK_IN      0x4000 //BIT 14,RO

#define TRACE(fmt,args...) 
#define trace(fmt,args...) 

#define lock_bpctl() 					\
if (down_interruptible(&bpctl_sema)) {			\
	return -ERESTARTSYS;				\
}							\

#define unlock_bpctl() 					\
	up(&bpctl_sema);

int bp_shutdown = 0;
/* Media Types */
typedef enum {
    bp_copper = 0,
    bp_fiber,
    bp_cx4,
    bp_none,
} bp_media_type;

struct pfs_unit_sd {
    struct proc_dir_entry *proc_entry;
    char proc_name[32];
} ; 

struct bypass_pfs_sd {
    char dir_name[32];
    struct proc_dir_entry *bypass_entry;
    struct pfs_unit_sd bypass_info; 
    struct pfs_unit_sd bypass_slave;  
    struct pfs_unit_sd bypass_caps;   
    struct pfs_unit_sd wd_set_caps;   
    struct pfs_unit_sd bypass;     
    struct pfs_unit_sd bypass_change; 
    struct pfs_unit_sd bypass_wd;     
    struct pfs_unit_sd wd_expire_time;
    struct pfs_unit_sd reset_bypass_wd;   
    struct pfs_unit_sd dis_bypass; 
    struct pfs_unit_sd bypass_pwup; 
    struct pfs_unit_sd bypass_pwoff; 
    struct pfs_unit_sd std_nic;
    struct pfs_unit_sd tap;
    struct pfs_unit_sd dis_tap;
    struct pfs_unit_sd tap_pwup;
    struct pfs_unit_sd tap_change;
    struct pfs_unit_sd wd_exp_mode; 
    struct pfs_unit_sd wd_autoreset;
    struct pfs_unit_sd tpl;

}; 

typedef struct _bpctl_dev {
    unsigned int  vendor;
    unsigned int  device;
    unsigned int  subvendor;
    unsigned int  subdevice;
    unsigned char bus;
    unsigned char slot;
    unsigned char func;
    int ifindex;
    unsigned long mem_map;
    struct pci_dev* pdev;
    struct net_device* ndev;
    spinlock_t bypass_wr_lock;
} bpctl_dev_t;


typedef struct _bp_dev {
    unsigned int  vendor;
    unsigned int  device;
    unsigned int  subvendor;
    unsigned int  subdevice;
    unsigned char bus;
    unsigned char slot;
    unsigned char func;
    int ifindex;
    unsigned long mem_map;
    struct pci_dev* pdev;
    struct net_device* ndev;
    spinlock_t bypass_wr_lock;
    int wdt_status;
    unsigned long bypass_wdt_on_time;
    uint32_t bypass_timer_interval;
    //uint32_t reset_time;
    //atomic_t wdt_busy;
} bp_dev_t;


int x710_set_clk_dat(bp_dev_t* pbp_dev);
int x710_clear_clk_dat(bp_dev_t* pbp_dev);
int x710_set_clk(bp_dev_t* pbp_dev);
int x710_clear_clk(bp_dev_t* pbp_dev);
int x710_set_dat(bp_dev_t* pbp_dev);
int x710_clear_dat(bp_dev_t* pbp_dev);
int x710_set_clk_clear_dat(bp_dev_t* pbp_dev);


int x710_set_clk0_dat0(bp_dev_t* pbp_dev);
int x710_clear_clk0_dat0(bp_dev_t* pbp_dev);
int x710_set_clk0(bp_dev_t* pbp_dev);
int x710_clear_clk0(bp_dev_t* pbp_dev);
int x710_set_dat0(bp_dev_t* pbp_dev);
int x710_clear_dat0(bp_dev_t* pbp_dev);
int x710_set_clk0_clear_dat0(bp_dev_t* pbp_dev);

int x710_gpio_set_sda_scl(bp_dev_t* pbp_dev);
int x710_gpio_clear_sda_scl(bp_dev_t* pbp_dev);
int x710_gpio_set_scl_clear_sda(bp_dev_t* pbp_dev);
int x710_gpio_set_scl(bp_dev_t* pbp_dev);
int x710_gpio_clear_scl(bp_dev_t* pbp_dev);
int x710_gpio_set_sda(bp_dev_t* pbp_dev);
int x710_gpio_clear_sda(bp_dev_t* pbp_dev);
int x710_gpio_sda_input(bp_dev_t* pbp_dev);
int x710_gpio_get_sda(bp_dev_t* pbp_dev,int *sda_stat);
 

#if (LINUX_VERSION_CODE > KERNEL_VERSION(4,15,0))
#if (LINUX_VERSION_CODE < KERNEL_VERSION(5,6,0))

#ifndef do_gettimeofday

#define do_gettimeofday(x) _kc_do_gettimeofday(x)
static inline void _kc_do_gettimeofday(struct timeval *tv)
{
	struct timespec64 now;

	ktime_get_real_ts64(&now);
	tv->tv_sec = now.tv_sec;
	tv->tv_usec = now.tv_nsec/1000;
}

#endif
#endif
#endif

int usec_delay_bp1(unsigned long x) {
#if (LINUX_VERSION_CODE < KERNEL_VERSION(5,6,0))
	struct timeval tv, tv_end;
    do_gettimeofday(&tv);
#endif
    udelay(x);
#if (LINUX_VERSION_CODE < KERNEL_VERSION(5,6,0))
	do_gettimeofday(&tv_end);
    if (tv_end.tv_usec>tv.tv_usec) {
        if ((tv_end.tv_usec-tv.tv_usec)<(x))
            return 0;
    } else if ((~tv.tv_usec+tv_end.tv_usec)<(x))
        return 0;
#endif
    return 1;

}
void usec_delay_bp(unsigned long x) {
#if 1
    int i=2;
    while (i--) {
        if (usec_delay_bp1(x))
            return;
	   /* printk("bpmod: udelay failed!\n");*/
    }
    /*printk("bpmod: udelay failed!!\n");*/
#else
	udelay(x);
#endif
}

void bp_delay_us(unsigned long x) 
{
    int i=2;
    while (i--) {
        if (usec_delay_bp1(x))
            return;
    }

}


void msec_delay_bp(unsigned long x)
{
#ifdef BP_NDELAY_MODE
    return;
#else
    int  i; 
    if (in_interrupt()) {
        for (i = 0; i < 1000; i++) {
            usec_delay_bp(x) ;       
        }                     
    } else {
        msleep(x); 
    }
#endif    
}

void bp_delay_ms(unsigned long x)
{
#ifdef BP_NDELAY_MODE
    return;
#else
    int  i; 
    if (in_interrupt()) {
        for (i = 0; i < 1000; i++) {
            usec_delay_bp(x) ;       
        }                     
    } else {
        msleep(x); 
    }
#endif    
}


static bpctl_dev_t *bpctl_dev_arr;
static bp_dev_t *bp_dev_list;
static struct semaphore bpctl_sema;
static int device_num = 0;

#ifdef ADI_RANGELEY_SUPPORT
static int na_device_num = 0;
#endif


int is_bypass_fn(bp_dev_t *pbpctl_dev);
int get_dev_idx_bsf(int bus, int slot, int func);
int get_bypass_caps_fn(bpctl_dev_t *pbpctl_dev);

int wdt_on(bpctl_dev_t *pbpctl_dev, unsigned int timeout);




static int device_open(struct inode *inode, struct file *file)
{
#ifdef DEBUG
    printk("device_open(%p)\n", file);
#endif
    Device_Open++;
/*
* Initialize the message
*/
    return SUCCESS;
}
static int device_release(struct inode *inode, struct file *file)
{
#ifdef DEBUG
    printk("device_release(%p,%p)\n", inode, file);
#endif
    Device_Open--;
    return SUCCESS;
}


//use scl0 sda0
static void bp_write_bit(bp_dev_t *pbpctl_dev,unsigned int ctrl_ext,unsigned char value,unsigned char len)
{
    unsigned char ctrl_val=0;
    unsigned int i=len;
  
    while (i--) {
        ctrl_val=(value>>i) & 0x1;
        if (ctrl_val) {
            
            x710_gpio_set_sda_scl(pbpctl_dev);

            usec_delay_bp(PULSE_TIME);
            
            x710_gpio_clear_scl(pbpctl_dev);

            usec_delay_bp(PULSE_TIME);

        } else {
 
            x710_gpio_set_scl_clear_sda(pbpctl_dev);

            usec_delay_bp(PULSE_TIME);

            x710_gpio_clear_scl(pbpctl_dev);

            usec_delay_bp(PULSE_TIME);
        } 

    }
}


static int bp_write_wdt_bit(bp_dev_t *pbpctl_dev){
    //uint32_t ctrl_ext=0, ctrl=0;
    unsigned long flags;
    if ((pbpctl_dev->func==1)||(pbpctl_dev->func==3))return -1;

    spin_lock_irqsave(&pbpctl_dev->bypass_wr_lock, flags);

    x710_gpio_clear_sda_scl(pbpctl_dev);
    x710_gpio_set_scl(pbpctl_dev);

    usec_delay_bp(WDT_INTERVAL);
    x710_gpio_clear_sda_scl(pbpctl_dev);

    if ((pbpctl_dev->wdt_status==WDT_STATUS_EN))pbpctl_dev->bypass_wdt_on_time=jiffies;

    spin_unlock_irqrestore(&pbpctl_dev->bypass_wr_lock, flags);

    return 0;
}  




static int bp_read_bit(bp_dev_t *pbpctl_dev, unsigned int ctrl_ext ,unsigned char len)
{
    unsigned char ctrl_val=0;
    unsigned int i=len;
    //unsigned int ctrl= 0;
    int bit_stat;

    while (i--) {
        x710_gpio_sda_input(pbpctl_dev);
        x710_gpio_clear_scl(pbpctl_dev);

        usec_delay_bp(PULSE_TIME);
        x710_gpio_set_scl(pbpctl_dev);

        x710_gpio_get_sda(pbpctl_dev,&bit_stat);
        
        usec_delay_bp(PULSE_TIME);
        if(bit_stat)
            ctrl_val |= 1<<i;
       
    }

    return ctrl_val;
}


//use sck0 sda0
static void bp_write_reg(bp_dev_t* pbpctl_dev, unsigned char value, unsigned char addr) 
{
    unsigned long flags;
    uint32_t ctrl_ext = 0;//, ctrl = 0;

    spin_lock_irqsave(&pbpctl_dev->bypass_wr_lock, flags);

   
    x710_gpio_clear_sda_scl(pbpctl_dev);

    usec_delay_bp(CMND_INTERVAL);

    bp_write_bit(pbpctl_dev, ctrl_ext, SYNC_CMD_VAL, SYNC_CMD_LEN);
    bp_write_bit(pbpctl_dev, ctrl_ext, WR_CMD_VAL, WR_CMD_LEN);
    bp_write_bit(pbpctl_dev, ctrl_ext, addr, ADDR_CMD_LEN);

    bp_write_bit(pbpctl_dev, ctrl_ext, value, WR_DATA_LEN);
  
    x710_gpio_clear_sda_scl(pbpctl_dev);

    usec_delay_bp(CMND_INTERVAL);

    spin_unlock_irqrestore(&pbpctl_dev->bypass_wr_lock, flags);

}


static void bp_write_data(bp_dev_t* pbpctl_dev, unsigned char value) {
    bp_write_reg(pbpctl_dev, value, CMND_REG_ADDR);
}


static int bp_read_reg(bp_dev_t* pbpctl_dev, unsigned char addr) 
{
    unsigned long flags;
    uint32_t ctrl_ext = 0, ctrl_value = 0;

#ifdef BP_SYNC_FLAG//this flag is set
    //trace("bp_read_reg BP_SYNC_FLAG\n");
#endif

    spin_lock_irqsave(&pbpctl_dev->bypass_wr_lock, flags);

    
    x710_gpio_clear_sda_scl(pbpctl_dev);
   
    usec_delay_bp(CMND_INTERVAL);

    bp_write_bit(pbpctl_dev, ctrl_ext, SYNC_CMD_VAL, SYNC_CMD_LEN);
    bp_write_bit(pbpctl_dev, ctrl_ext, RD_CMD_VAL, RD_CMD_LEN);
    bp_write_bit(pbpctl_dev, ctrl_ext, addr, ADDR_CMD_LEN);

    x710_gpio_sda_input(pbpctl_dev);
    x710_gpio_set_scl(pbpctl_dev);

    usec_delay_bp(PULSE_TIME);

    ctrl_value = bp_read_bit(pbpctl_dev, ctrl_ext, RD_DATA_LEN);

   
    x710_gpio_clear_sda_scl(pbpctl_dev);

    usec_delay_bp(CMND_INTERVAL);

    spin_unlock_irqrestore(&pbpctl_dev->bypass_wr_lock, flags);

    return ctrl_value;
}
 

int bp_cmnd_on(bp_dev_t *pbpctl_dev){
    int ret=BP_NOT_CAP;

    if (pbpctl_dev->func==0 || pbpctl_dev->func==2){
        bp_write_data(pbpctl_dev,CMND_ON);
        ret=0;
    }
    return ret;
}

int bp_cmnd_off(bp_dev_t *pbpctl_dev){
    int ret=BP_NOT_CAP;

    if (pbpctl_dev->func==0 || pbpctl_dev->func==2){
        bp_write_data(pbpctl_dev,CMND_OFF);
        ret=0;
    }
    return ret;
}


int bp_bypass_on(bp_dev_t* pbpctl_dev) {
    bp_write_data(pbpctl_dev, BYPASS_ON);
    msec_delay_bp(LATCH_DELAY);
    return 0;
}

int bp_bypass_off(bp_dev_t* pbpctl_dev) {
    bp_write_data(pbpctl_dev, BYPASS_OFF);
    msec_delay_bp(LATCH_DELAY);
    return 0;
}



static int bp_disc_status(bp_dev_t* pbpctl_dev)
{
    bp_dev_t* pbpctl_dev_status = NULL;

    if (pbpctl_dev->func == 0) {
        pbpctl_dev_status = &(bp_dev_list[1]);
    }
    else if (pbpctl_dev->func == 2) {
        pbpctl_dev_status = &(bp_dev_list[3]);
    }

    if (pbpctl_dev_status == NULL)return BP_NOT_CAP;
    if(pbpctl_dev_status->pdev == NULL)return BP_NOT_CAP;

    return(((BPCTL_READ_REG(pbpctl_dev_status, CTRL_EXT)) & BPCTLI_CTRL_EXT_SDP6_DATA)!=0?0:1);

}

int is_bypass_fn(bp_dev_t *pbpctl_dev){


    if (!pbpctl_dev) {
		return -1;
	}

    return(((pbpctl_dev->func == 0) || (pbpctl_dev->func == 2)) ? 1 : 0);
}



int x710_set_clk_dat(bp_dev_t* pbp_dev)
{
     int ret=BP_NOT_CAP;
     //unsigned long flags;

    if (pbp_dev->func==0){

        uint32_t reg_i2c_sel = 0, reg_i2c_param = 0;

        //spin_lock_irqsave(&pbp_dev->bypass_wr_lock, flags);

        reg_i2c_sel = readl((void *)(pbp_dev->mem_map) + REG_X710_GLGEN_MDIO_I2C_SEL1);
        reg_i2c_param = readl((void *)(pbp_dev->mem_map) + REG_X710_GLGEN_I2C_PARAM1);

        writel(reg_i2c_sel|BIT_X710_SEL_I2C, (void *)((pbp_dev->mem_map) + REG_X710_GLGEN_MDIO_I2C_SEL1));
        reg_i2c_sel = readl((void *)(pbp_dev->mem_map) + REG_X710_GLGEN_MDIO_I2C_SEL1);//flush write op
        //trace("reg_i2c_sel 0x%x\n",reg_i2c_sel);
        reg_i2c_param |= BIT_X710_I2CBB_EN;
        reg_i2c_param &= ~BIT_X710_I2CCLK_OUT_DIS;//enable output,active low
        reg_i2c_param &= ~BIT_X710_I2CDAT_OUT_DIS;//enable output,active low
        reg_i2c_param |= BIT_X710_I2CCLK_OUT;
        //reg_i2c_param &= ~BIT_X710_I2CCLK_OUT;
        reg_i2c_param |= BIT_X710_I2CDAT_OUT;
        //reg_i2c_param &= ~BIT_X710_I2CDAT_OUT;
        writel(reg_i2c_param, (void *)((pbp_dev->mem_map) + REG_X710_GLGEN_I2C_PARAM1));
        reg_i2c_param = readl((void *)(pbp_dev->mem_map) + REG_X710_GLGEN_I2C_PARAM1);//flush write op
        //trace("reg_i2c_param 0x%x\n",reg_i2c_param);

        //spin_unlock_irqrestore(&pbp_dev->bypass_wr_lock, flags);

        ret=0;
    }
    else if (pbp_dev->func==2){

        uint32_t reg_i2c_sel = 0, reg_i2c_param = 0;

        //spin_lock_irqsave(&pbp_dev->bypass_wr_lock, flags);

        reg_i2c_sel = readl((void *)(pbp_dev->mem_map) + REG_X710_GLGEN_MDIO_I2C_SEL3);
        reg_i2c_param = readl((void *)(pbp_dev->mem_map) + REG_X710_GLGEN_I2C_PARAM3);

        writel(reg_i2c_sel|BIT_X710_SEL_I2C, (void *)((pbp_dev->mem_map) + REG_X710_GLGEN_MDIO_I2C_SEL3));
        reg_i2c_sel = readl((void *)(pbp_dev->mem_map) + REG_X710_GLGEN_MDIO_I2C_SEL3);//flush write op
        //trace("reg_i2c_sel 0x%x\n",reg_i2c_sel);
        reg_i2c_param |= BIT_X710_I2CBB_EN;
        reg_i2c_param &= ~BIT_X710_I2CCLK_OUT_DIS;//enable output,active low
        reg_i2c_param &= ~BIT_X710_I2CDAT_OUT_DIS;//enable output,active low
        reg_i2c_param |= BIT_X710_I2CCLK_OUT;
        //reg_i2c_param &= ~BIT_X710_I2CCLK_OUT;
        reg_i2c_param |= BIT_X710_I2CDAT_OUT;
        //reg_i2c_param &= ~BIT_X710_I2CDAT_OUT;
        writel(reg_i2c_param, (void *)((pbp_dev->mem_map) + REG_X710_GLGEN_I2C_PARAM3));
        reg_i2c_param = readl((void *)(pbp_dev->mem_map) + REG_X710_GLGEN_I2C_PARAM3);//flush write op
        //trace("reg_i2c_param 0x%x\n",reg_i2c_param);

        //spin_unlock_irqrestore(&pbp_dev->bypass_wr_lock, flags);

        ret=0;
    }
    return ret;
}

//clear clk1 and dat1
int x710_clear_clk_dat(bp_dev_t* pbp_dev)
{
     int ret=BP_NOT_CAP;
     //unsigned long flags;

    if (pbp_dev->func==0){

        uint32_t reg_i2c_sel = 0, reg_i2c_param = 0;

        //spin_lock_irqsave(&pbp_dev->bypass_wr_lock, flags);

        reg_i2c_sel = readl((void *)(pbp_dev->mem_map) + REG_X710_GLGEN_MDIO_I2C_SEL1);
        reg_i2c_param = readl((void *)(pbp_dev->mem_map) + REG_X710_GLGEN_I2C_PARAM1);

        writel(reg_i2c_sel|BIT_X710_SEL_I2C, (void *)((pbp_dev->mem_map) + REG_X710_GLGEN_MDIO_I2C_SEL1));
        reg_i2c_sel = readl((void *)(pbp_dev->mem_map) + REG_X710_GLGEN_MDIO_I2C_SEL1);//flush write op
        //trace("reg_i2c_sel 0x%x\n",reg_i2c_sel);
        reg_i2c_param |= BIT_X710_I2CBB_EN;
        reg_i2c_param &= ~BIT_X710_I2CCLK_OUT_DIS;//enable output,active low
        reg_i2c_param &= ~BIT_X710_I2CDAT_OUT_DIS;//enable output,active low
        //reg_i2c_param |= BIT_X710_I2CCLK_OUT;
        reg_i2c_param &= ~BIT_X710_I2CCLK_OUT;
        //reg_i2c_param |= BIT_X710_I2CDAT_OUT;
        reg_i2c_param &= ~BIT_X710_I2CDAT_OUT;
        writel(reg_i2c_param, (void *)((pbp_dev->mem_map) + REG_X710_GLGEN_I2C_PARAM1));
        reg_i2c_param = readl((void *)(pbp_dev->mem_map) + REG_X710_GLGEN_I2C_PARAM1);//flush write op
        //trace("reg_i2c_param 0x%x\n",reg_i2c_param);

        //spin_unlock_irqrestore(&pbp_dev->bypass_wr_lock, flags);

        ret=0;
    }
    else if (pbp_dev->func==2){

        uint32_t reg_i2c_sel = 0, reg_i2c_param = 0;

        //spin_lock_irqsave(&pbp_dev->bypass_wr_lock, flags);

        reg_i2c_sel = readl((void *)(pbp_dev->mem_map) + REG_X710_GLGEN_MDIO_I2C_SEL3);
        reg_i2c_param = readl((void *)(pbp_dev->mem_map) + REG_X710_GLGEN_I2C_PARAM3);

        writel(reg_i2c_sel|BIT_X710_SEL_I2C, (void *)((pbp_dev->mem_map) + REG_X710_GLGEN_MDIO_I2C_SEL3));
        reg_i2c_sel = readl((void *)(pbp_dev->mem_map) + REG_X710_GLGEN_MDIO_I2C_SEL3);//flush write op
        //trace("reg_i2c_sel 0x%x\n",reg_i2c_sel);
        reg_i2c_param |= BIT_X710_I2CBB_EN;
        reg_i2c_param &= ~BIT_X710_I2CCLK_OUT_DIS;//enable output,active low
        reg_i2c_param &= ~BIT_X710_I2CDAT_OUT_DIS;//enable output,active low
        //reg_i2c_param |= BIT_X710_I2CCLK_OUT;
        reg_i2c_param &= ~BIT_X710_I2CCLK_OUT;
        //reg_i2c_param |= BIT_X710_I2CDAT_OUT;
        reg_i2c_param &= ~BIT_X710_I2CDAT_OUT;
        writel(reg_i2c_param, (void *)((pbp_dev->mem_map) + REG_X710_GLGEN_I2C_PARAM3));
        reg_i2c_param = readl((void *)(pbp_dev->mem_map) + REG_X710_GLGEN_I2C_PARAM3);//flush write op
        //trace("reg_i2c_param 0x%x\n",reg_i2c_param);

        //spin_unlock_irqrestore(&pbp_dev->bypass_wr_lock, flags);

        ret=0;
    }
    return ret;
}

//clear clk0 and dat0
int x710_clear_clk0_dat0(bp_dev_t* pbp_dev)
{
     int ret=BP_NOT_CAP;
     //unsigned long flags;

    if (pbp_dev->func==0){

        uint32_t reg_i2c_sel = 0, reg_i2c_param = 0;

        //spin_lock_irqsave(&pbp_dev->bypass_wr_lock, flags);

        reg_i2c_sel = readl((void *)(pbp_dev->mem_map) + REG_X710_GLGEN_MDIO_I2C_SEL0);
        reg_i2c_param = readl((void *)(pbp_dev->mem_map) + REG_X710_GLGEN_I2C_PARAM0);
        //trace("0 reg_i2c_sel 0x%x\n",reg_i2c_sel);
        //trace("0 reg_i2c_param 0x%x\n",reg_i2c_param);

        writel(reg_i2c_sel|BIT_X710_SEL_I2C, (void *)((pbp_dev->mem_map) + REG_X710_GLGEN_MDIO_I2C_SEL0));
        reg_i2c_sel = readl((void *)(pbp_dev->mem_map) + REG_X710_GLGEN_MDIO_I2C_SEL0);//flush write op
        //trace("1 reg_i2c_sel 0x%x\n",reg_i2c_sel);
        reg_i2c_param |= BIT_X710_I2CBB_EN;
        reg_i2c_param &= ~BIT_X710_I2CCLK_OUT_DIS;//enable output,active low
        reg_i2c_param &= ~BIT_X710_I2CDAT_OUT_DIS;//enable output,active low
        //reg_i2c_param |= BIT_X710_I2CCLK_OUT;
        reg_i2c_param &= ~BIT_X710_I2CCLK_OUT;
        //reg_i2c_param |= BIT_X710_I2CDAT_OUT;
        reg_i2c_param &= ~BIT_X710_I2CDAT_OUT;
        writel(reg_i2c_param, (void *)((pbp_dev->mem_map) + REG_X710_GLGEN_I2C_PARAM0));
        reg_i2c_param = readl((void *)(pbp_dev->mem_map) + REG_X710_GLGEN_I2C_PARAM0);//flush write op
        //trace("1 reg_i2c_param 0x%x\n",reg_i2c_param);

        //spin_unlock_irqrestore(&pbp_dev->bypass_wr_lock, flags);

        ret=0;
    }
    else if (pbp_dev->func==2){

        uint32_t reg_i2c_sel = 0, reg_i2c_param = 0;

        //spin_lock_irqsave(&pbp_dev->bypass_wr_lock, flags);

        reg_i2c_sel = readl((void *)(pbp_dev->mem_map) + REG_X710_GLGEN_MDIO_I2C_SEL2);
        reg_i2c_param = readl((void *)(pbp_dev->mem_map) + REG_X710_GLGEN_I2C_PARAM2);

        writel(reg_i2c_sel|BIT_X710_SEL_I2C, (void *)((pbp_dev->mem_map) + REG_X710_GLGEN_MDIO_I2C_SEL2));
        reg_i2c_sel = readl((void *)(pbp_dev->mem_map) + REG_X710_GLGEN_MDIO_I2C_SEL2);//flush write op
        //trace("reg_i2c_sel 0x%x\n",reg_i2c_sel);
        reg_i2c_param |= BIT_X710_I2CBB_EN;
        reg_i2c_param &= ~BIT_X710_I2CCLK_OUT_DIS;//enable output,active low
        reg_i2c_param &= ~BIT_X710_I2CDAT_OUT_DIS;//enable output,active low
        //reg_i2c_param |= BIT_X710_I2CCLK_OUT;
        reg_i2c_param &= ~BIT_X710_I2CCLK_OUT;
        //reg_i2c_param |= BIT_X710_I2CDAT_OUT;
        reg_i2c_param &= ~BIT_X710_I2CDAT_OUT;
        writel(reg_i2c_param, (void *)((pbp_dev->mem_map) + REG_X710_GLGEN_I2C_PARAM2));
        reg_i2c_param = readl((void *)(pbp_dev->mem_map) + REG_X710_GLGEN_I2C_PARAM2);//flush write op
        //trace("reg_i2c_param 0x%x\n",reg_i2c_param);

        //spin_unlock_irqrestore(&pbp_dev->bypass_wr_lock, flags);

        ret=0;
    }
    return ret;
}



int x710_set_clk_clear_dat(bp_dev_t* pbp_dev)
{
     int ret=BP_NOT_CAP;
     //unsigned long flags;

    if (pbp_dev->func==0){

        uint32_t reg_i2c_sel = 0, reg_i2c_param = 0;

        //spin_lock_irqsave(&pbp_dev->bypass_wr_lock, flags);

        reg_i2c_sel = readl((void *)(pbp_dev->mem_map) + REG_X710_GLGEN_MDIO_I2C_SEL1);
        reg_i2c_param = readl((void *)(pbp_dev->mem_map) + REG_X710_GLGEN_I2C_PARAM1);

        writel(reg_i2c_sel|BIT_X710_SEL_I2C, (void *)((pbp_dev->mem_map) + REG_X710_GLGEN_MDIO_I2C_SEL1));
        reg_i2c_sel = readl((void *)(pbp_dev->mem_map) + REG_X710_GLGEN_MDIO_I2C_SEL1);//flush write op
        //trace("reg_i2c_sel 0x%x\n",reg_i2c_sel);
        reg_i2c_param |= BIT_X710_I2CBB_EN;
        reg_i2c_param &= ~BIT_X710_I2CCLK_OUT_DIS;//enable output,active low
        reg_i2c_param &= ~BIT_X710_I2CDAT_OUT_DIS;//enable output,active low
        reg_i2c_param |= BIT_X710_I2CCLK_OUT;
        //reg_i2c_param &= ~BIT_X710_I2CCLK_OUT;
        //reg_i2c_param |= BIT_X710_I2CDAT_OUT;
        reg_i2c_param &= ~BIT_X710_I2CDAT_OUT;
        writel(reg_i2c_param, (void *)((pbp_dev->mem_map) + REG_X710_GLGEN_I2C_PARAM1));
        reg_i2c_param = readl((void *)(pbp_dev->mem_map) + REG_X710_GLGEN_I2C_PARAM1);//flush write op
        //trace("reg_i2c_param 0x%x\n",reg_i2c_param);

        //spin_unlock_irqrestore(&pbp_dev->bypass_wr_lock, flags);

        ret=0;
    }
    else if (pbp_dev->func==2){

        uint32_t reg_i2c_sel = 0, reg_i2c_param = 0;

        //spin_lock_irqsave(&pbp_dev->bypass_wr_lock, flags);

        reg_i2c_sel = readl((void *)(pbp_dev->mem_map) + REG_X710_GLGEN_MDIO_I2C_SEL3);
        reg_i2c_param = readl((void *)(pbp_dev->mem_map) + REG_X710_GLGEN_I2C_PARAM3);

        writel(reg_i2c_sel|BIT_X710_SEL_I2C, (void *)((pbp_dev->mem_map) + REG_X710_GLGEN_MDIO_I2C_SEL3));
        reg_i2c_sel = readl((void *)(pbp_dev->mem_map) + REG_X710_GLGEN_MDIO_I2C_SEL3);//flush write op
        //trace("reg_i2c_sel 0x%x\n",reg_i2c_sel);
        reg_i2c_param |= BIT_X710_I2CBB_EN;
        reg_i2c_param &= ~BIT_X710_I2CCLK_OUT_DIS;//enable output,active low
        reg_i2c_param &= ~BIT_X710_I2CDAT_OUT_DIS;//enable output,active low
        reg_i2c_param |= BIT_X710_I2CCLK_OUT;
        //reg_i2c_param &= ~BIT_X710_I2CCLK_OUT;
        //reg_i2c_param |= BIT_X710_I2CDAT_OUT;
        reg_i2c_param &= ~BIT_X710_I2CDAT_OUT;
        writel(reg_i2c_param, (void *)((pbp_dev->mem_map) + REG_X710_GLGEN_I2C_PARAM3));
        reg_i2c_param = readl((void *)(pbp_dev->mem_map) + REG_X710_GLGEN_I2C_PARAM3);//flush write op
        //trace("reg_i2c_param 0x%x\n",reg_i2c_param);

        //spin_unlock_irqrestore(&pbp_dev->bypass_wr_lock, flags);

        ret=0;
    }
    return ret;
}

int x710_set_clk0_clear_dat0(bp_dev_t* pbp_dev)
{
     int ret=BP_NOT_CAP;
     //unsigned long flags;

    if (pbp_dev->func==0){

        uint32_t reg_i2c_sel = 0, reg_i2c_param = 0;

        //spin_lock_irqsave(&pbp_dev->bypass_wr_lock, flags);

        reg_i2c_sel = readl((void *)(pbp_dev->mem_map) + REG_X710_GLGEN_MDIO_I2C_SEL0);
        reg_i2c_param = readl((void *)(pbp_dev->mem_map) + REG_X710_GLGEN_I2C_PARAM0);

        writel(reg_i2c_sel|BIT_X710_SEL_I2C, (void *)((pbp_dev->mem_map) + REG_X710_GLGEN_MDIO_I2C_SEL0));
        reg_i2c_sel = readl((void *)(pbp_dev->mem_map) + REG_X710_GLGEN_MDIO_I2C_SEL0);//flush write op
        //trace("reg_i2c_sel 0x%x\n",reg_i2c_sel);
        reg_i2c_param |= BIT_X710_I2CBB_EN;
        reg_i2c_param &= ~BIT_X710_I2CCLK_OUT_DIS;//enable output,active low
        reg_i2c_param &= ~BIT_X710_I2CDAT_OUT_DIS;//enable output,active low
        reg_i2c_param |= BIT_X710_I2CCLK_OUT;
        //reg_i2c_param &= ~BIT_X710_I2CCLK_OUT;
        //reg_i2c_param |= BIT_X710_I2CDAT_OUT;
        reg_i2c_param &= ~BIT_X710_I2CDAT_OUT;
        writel(reg_i2c_param, (void *)((pbp_dev->mem_map) + REG_X710_GLGEN_I2C_PARAM0));
        reg_i2c_param = readl((void *)(pbp_dev->mem_map) + REG_X710_GLGEN_I2C_PARAM0);//flush write op
        //trace("reg_i2c_param 0x%x\n",reg_i2c_param);

        //spin_unlock_irqrestore(&pbp_dev->bypass_wr_lock, flags);

        ret=0;
    }
    else if (pbp_dev->func==2){

        uint32_t reg_i2c_sel = 0, reg_i2c_param = 0;

        //spin_lock_irqsave(&pbp_dev->bypass_wr_lock, flags);

        reg_i2c_sel = readl((void *)(pbp_dev->mem_map) + REG_X710_GLGEN_MDIO_I2C_SEL2);
        reg_i2c_param = readl((void *)(pbp_dev->mem_map) + REG_X710_GLGEN_I2C_PARAM2);

        writel(reg_i2c_sel|BIT_X710_SEL_I2C, (void *)((pbp_dev->mem_map) + REG_X710_GLGEN_MDIO_I2C_SEL2));
        reg_i2c_sel = readl((void *)(pbp_dev->mem_map) + REG_X710_GLGEN_MDIO_I2C_SEL2);//flush write op
        //trace("reg_i2c_sel 0x%x\n",reg_i2c_sel);
        reg_i2c_param |= BIT_X710_I2CBB_EN;
        reg_i2c_param &= ~BIT_X710_I2CCLK_OUT_DIS;//enable output,active low
        reg_i2c_param &= ~BIT_X710_I2CDAT_OUT_DIS;//enable output,active low
        reg_i2c_param |= BIT_X710_I2CCLK_OUT;
        //reg_i2c_param &= ~BIT_X710_I2CCLK_OUT;
        //reg_i2c_param |= BIT_X710_I2CDAT_OUT;
        reg_i2c_param &= ~BIT_X710_I2CDAT_OUT;
        writel(reg_i2c_param, (void *)((pbp_dev->mem_map) + REG_X710_GLGEN_I2C_PARAM2));
        reg_i2c_param = readl((void *)(pbp_dev->mem_map) + REG_X710_GLGEN_I2C_PARAM2);//flush write op
        //trace("reg_i2c_param 0x%x\n",reg_i2c_param);

        //spin_unlock_irqrestore(&pbp_dev->bypass_wr_lock, flags);

        ret=0;
    }
    return ret;
}

//set clk0 and dat0
int x710_set_clk0_dat0(bp_dev_t* pbp_dev)
{
     int ret=BP_NOT_CAP;
     //unsigned long flags;

    if (pbp_dev->func==0){

        uint32_t reg_i2c_sel = 0, reg_i2c_param = 0;

        //spin_lock_irqsave(&pbp_dev->bypass_wr_lock, flags);

        reg_i2c_sel = readl((void *)(pbp_dev->mem_map) + REG_X710_GLGEN_MDIO_I2C_SEL0);
        reg_i2c_param = readl((void *)(pbp_dev->mem_map) + REG_X710_GLGEN_I2C_PARAM0);
        trace("2 reg_i2c_sel 0x%x\n",reg_i2c_sel);
        trace("2 reg_i2c_param 0x%x\n",reg_i2c_param);

        writel(reg_i2c_sel|BIT_X710_SEL_I2C, (void *)((pbp_dev->mem_map) + REG_X710_GLGEN_MDIO_I2C_SEL0));
        reg_i2c_sel = readl((void *)(pbp_dev->mem_map) + REG_X710_GLGEN_MDIO_I2C_SEL0);//flush write op
        trace("3 reg_i2c_sel 0x%x\n",reg_i2c_sel);
        reg_i2c_param |= BIT_X710_I2CBB_EN;
        reg_i2c_param &= ~BIT_X710_I2CCLK_OUT_DIS;//enable output,active low
        reg_i2c_param &= ~BIT_X710_I2CDAT_OUT_DIS;//enable output,active low
        reg_i2c_param |= BIT_X710_I2CCLK_OUT;
        //reg_i2c_param &= ~BIT_X710_I2CCLK_OUT;
        reg_i2c_param |= BIT_X710_I2CDAT_OUT;
        //reg_i2c_param &= ~BIT_X710_I2CDAT_OUT;
        writel(reg_i2c_param, (void *)((pbp_dev->mem_map) + REG_X710_GLGEN_I2C_PARAM0));
        reg_i2c_param = readl((void *)(pbp_dev->mem_map) + REG_X710_GLGEN_I2C_PARAM0);//flush write op
        trace("3 reg_i2c_param 0x%x\n",reg_i2c_param);

        //spin_unlock_irqrestore(&pbp_dev->bypass_wr_lock, flags);

        ret=0;
    }
    else if (pbp_dev->func==2){

        uint32_t reg_i2c_sel = 0, reg_i2c_param = 0;

        //spin_lock_irqsave(&pbp_dev->bypass_wr_lock, flags);

        reg_i2c_sel = readl((void *)(pbp_dev->mem_map) + REG_X710_GLGEN_MDIO_I2C_SEL2);
        reg_i2c_param = readl((void *)(pbp_dev->mem_map) + REG_X710_GLGEN_I2C_PARAM2);

        writel(reg_i2c_sel|BIT_X710_SEL_I2C, (void *)((pbp_dev->mem_map) + REG_X710_GLGEN_MDIO_I2C_SEL2));
        reg_i2c_sel = readl((void *)(pbp_dev->mem_map) + REG_X710_GLGEN_MDIO_I2C_SEL2);//flush write op
        //trace("reg_i2c_sel 0x%x\n",reg_i2c_sel);
        reg_i2c_param |= BIT_X710_I2CBB_EN;
        reg_i2c_param &= ~BIT_X710_I2CCLK_OUT_DIS;//enable output,active low
        reg_i2c_param &= ~BIT_X710_I2CDAT_OUT_DIS;//enable output,active low
        reg_i2c_param |= BIT_X710_I2CCLK_OUT;
        //reg_i2c_param &= ~BIT_X710_I2CCLK_OUT;
        reg_i2c_param |= BIT_X710_I2CDAT_OUT;
        //reg_i2c_param &= ~BIT_X710_I2CDAT_OUT;
        writel(reg_i2c_param, (void *)((pbp_dev->mem_map) + REG_X710_GLGEN_I2C_PARAM2));
        reg_i2c_param = readl((void *)(pbp_dev->mem_map) + REG_X710_GLGEN_I2C_PARAM2);//flush write op
        //trace("reg_i2c_param 0x%x\n",reg_i2c_param);

        //spin_unlock_irqrestore(&pbp_dev->bypass_wr_lock, flags);

        ret=0;
    }
    return ret;
}

//set clk0 and dat0
//gpio_0 : sda
//gpio_1 : scl
int x710_gpio_set_sda_scl(bp_dev_t* pbp_dev)
{
    int ret=BP_NOT_CAP;
    bp_dev_t* pbp_dev_master = NULL;

    if (pbp_dev->func==0){
        //set sda(gpio_0)
        writel(0x3f00030, (void *)((pbp_dev->mem_map) + REG_X710_GLGEN_GPIO_CTL0));
        readl((void *)(pbp_dev->mem_map) + REG_X710_GLGEN_GPIO_STAT);//flush write op
        writel(GPIO_INDEX_0|SDP_DATA_HI|SDP_DRIVE, (void *)((pbp_dev->mem_map) + REG_X710_GLGEN_GPIO_SET));
        readl((void *)(pbp_dev->mem_map) + REG_X710_GLGEN_GPIO_STAT);//flush write op

        //set scl(gpio_1)
        writel(0x3f00030, (void *)((pbp_dev->mem_map) + REG_X710_GLGEN_GPIO_CTL1));
        readl((void *)(pbp_dev->mem_map) + REG_X710_GLGEN_GPIO_STAT);//flush write op
        writel(GPIO_INDEX_1|SDP_DATA_HI|SDP_DRIVE, (void *)((pbp_dev->mem_map) + REG_X710_GLGEN_GPIO_SET));
        readl((void *)(pbp_dev->mem_map) + REG_X710_GLGEN_GPIO_STAT);//flush write op
        ret = 0;
    }
    else if (pbp_dev->func==2){
        pbp_dev_master = &(bp_dev_list[0]);
        //set sda(gpio_2)
        writel(0x3f00030, (void *)((pbp_dev_master->mem_map) + REG_X710_GLGEN_GPIO_CTL2));
        readl((void *)(pbp_dev_master->mem_map) + REG_X710_GLGEN_GPIO_STAT);//flush write op
        writel(GPIO_INDEX_2|SDP_DATA_HI|SDP_DRIVE, (void *)((pbp_dev_master->mem_map) + REG_X710_GLGEN_GPIO_SET));
        readl((void *)(pbp_dev_master->mem_map) + REG_X710_GLGEN_GPIO_STAT);//flush write op

        //set scl(gpio_3)
        writel(0x3f00030, (void *)((pbp_dev_master->mem_map) + REG_X710_GLGEN_GPIO_CTL3));
        readl((void *)(pbp_dev_master->mem_map) + REG_X710_GLGEN_GPIO_STAT);//flush write op
        writel(GPIO_INDEX_3|SDP_DATA_HI|SDP_DRIVE, (void *)((pbp_dev_master->mem_map) + REG_X710_GLGEN_GPIO_SET));
        readl((void *)(pbp_dev_master->mem_map) + REG_X710_GLGEN_GPIO_STAT);//flush write op
        ret = 0;
    }
    return ret;
}

//gpio_0 : sda
//gpio_1 : scl
int x710_gpio_clear_sda_scl(bp_dev_t* pbp_dev)
{
    int ret=BP_NOT_CAP;
    bp_dev_t* pbp_dev_master = NULL;

    if (pbp_dev->func==0){
        //set sda(gpio_0)
        writel(0x3f00030, (void *)((pbp_dev->mem_map) + REG_X710_GLGEN_GPIO_CTL0));
        readl((void *)(pbp_dev->mem_map) + REG_X710_GLGEN_GPIO_STAT);//flush write op
        writel(GPIO_INDEX_0|SDP_DATA_LO|SDP_DRIVE, (void *)((pbp_dev->mem_map) + REG_X710_GLGEN_GPIO_SET));
        readl((void *)(pbp_dev->mem_map) + REG_X710_GLGEN_GPIO_STAT);//flush write op

        //set scl(gpio_1)
        writel(0x3f00030, (void *)((pbp_dev->mem_map) + REG_X710_GLGEN_GPIO_CTL1));
        readl((void *)(pbp_dev->mem_map) + REG_X710_GLGEN_GPIO_STAT);//flush write op
        writel(GPIO_INDEX_1|SDP_DATA_LO|SDP_DRIVE, (void *)((pbp_dev->mem_map) + REG_X710_GLGEN_GPIO_SET));
        readl((void *)(pbp_dev->mem_map) + REG_X710_GLGEN_GPIO_STAT);//flush write op
        ret = 0;
    }
    else if (pbp_dev->func==2){
        pbp_dev_master = &(bp_dev_list[0]);
        //set sda(gpio_2)
        writel(0x3f00030, (void *)((pbp_dev_master->mem_map) + REG_X710_GLGEN_GPIO_CTL2));
        readl((void *)(pbp_dev_master->mem_map) + REG_X710_GLGEN_GPIO_STAT);//flush write op
        writel(GPIO_INDEX_2|SDP_DATA_LO|SDP_DRIVE, (void *)((pbp_dev_master->mem_map) + REG_X710_GLGEN_GPIO_SET));
        readl((void *)(pbp_dev_master->mem_map) + REG_X710_GLGEN_GPIO_STAT);//flush write op

        //set scl(gpio_3)
        writel(0x3f00030, (void *)((pbp_dev_master->mem_map) + REG_X710_GLGEN_GPIO_CTL3));
        readl((void *)(pbp_dev_master->mem_map) + REG_X710_GLGEN_GPIO_STAT);//flush write op
        writel(GPIO_INDEX_3|SDP_DATA_LO|SDP_DRIVE, (void *)((pbp_dev_master->mem_map) + REG_X710_GLGEN_GPIO_SET));
        readl((void *)(pbp_dev_master->mem_map) + REG_X710_GLGEN_GPIO_STAT);//flush write op
        ret = 0;
    }
    return ret;
}

//gpio_0 : sda
//gpio_1 : scl
int x710_gpio_set_scl_clear_sda(bp_dev_t* pbp_dev)
{
    int ret=BP_NOT_CAP;
    bp_dev_t* pbp_dev_master = NULL;

    if (pbp_dev->func==0){
        //clear sda(gpio_0)
        writel(0x3f00030, (void *)((pbp_dev->mem_map) + REG_X710_GLGEN_GPIO_CTL0));
        readl((void *)(pbp_dev->mem_map) + REG_X710_GLGEN_GPIO_STAT);//flush write op
        writel(GPIO_INDEX_0|SDP_DATA_LO|SDP_DRIVE, (void *)((pbp_dev->mem_map) + REG_X710_GLGEN_GPIO_SET));
        readl((void *)(pbp_dev->mem_map) + REG_X710_GLGEN_GPIO_STAT);//flush write op

        //set scl(gpio_1)
        writel(0x3f00030, (void *)((pbp_dev->mem_map) + REG_X710_GLGEN_GPIO_CTL1));
        readl((void *)(pbp_dev->mem_map) + REG_X710_GLGEN_GPIO_STAT);//flush write op
        writel(GPIO_INDEX_1|SDP_DATA_HI|SDP_DRIVE, (void *)((pbp_dev->mem_map) + REG_X710_GLGEN_GPIO_SET));
        readl((void *)(pbp_dev->mem_map) + REG_X710_GLGEN_GPIO_STAT);//flush write op
        ret = 0;
    }
    else if (pbp_dev->func==2){
        pbp_dev_master = &(bp_dev_list[0]);
        //CLEAR sda(gpio_2)
        writel(0x3f00030, (void *)((pbp_dev_master->mem_map) + REG_X710_GLGEN_GPIO_CTL2));
        readl((void *)(pbp_dev_master->mem_map) + REG_X710_GLGEN_GPIO_STAT);//flush write op
        writel(GPIO_INDEX_2|SDP_DATA_LO|SDP_DRIVE, (void *)((pbp_dev_master->mem_map) + REG_X710_GLGEN_GPIO_SET));
        readl((void *)(pbp_dev_master->mem_map) + REG_X710_GLGEN_GPIO_STAT);//flush write op

        //set scl(gpio_3)
        writel(0x3f00030, (void *)((pbp_dev_master->mem_map) + REG_X710_GLGEN_GPIO_CTL3));
        readl((void *)(pbp_dev_master->mem_map) + REG_X710_GLGEN_GPIO_STAT);//flush write op
        writel(GPIO_INDEX_3|SDP_DATA_HI|SDP_DRIVE, (void *)((pbp_dev_master->mem_map) + REG_X710_GLGEN_GPIO_SET));
        readl((void *)(pbp_dev_master->mem_map) + REG_X710_GLGEN_GPIO_STAT);//flush write op
        ret = 0;
    }
    return ret;
}

//gpio_1 : scl
int x710_gpio_set_scl(bp_dev_t* pbp_dev)
{
    int ret=BP_NOT_CAP;
    bp_dev_t* pbp_dev_master = NULL;

    if (pbp_dev->func==0){
        //set scl(gpio_1)
        writel(0x3f00030, (void *)((pbp_dev->mem_map) + REG_X710_GLGEN_GPIO_CTL1));
        readl((void *)(pbp_dev->mem_map) + REG_X710_GLGEN_GPIO_STAT);//flush write op
        writel(GPIO_INDEX_1|SDP_DATA_HI|SDP_DRIVE, (void *)((pbp_dev->mem_map) + REG_X710_GLGEN_GPIO_SET));
        readl((void *)(pbp_dev->mem_map) + REG_X710_GLGEN_GPIO_STAT);//flush write op
        ret = 0;
    }
    else if (pbp_dev->func==2){
        pbp_dev_master = &(bp_dev_list[0]);
        //set scl(gpio_3)
        writel(0x3f00030, (void *)((pbp_dev_master->mem_map) + REG_X710_GLGEN_GPIO_CTL3));
        readl((void *)(pbp_dev_master->mem_map) + REG_X710_GLGEN_GPIO_STAT);//flush write op
        writel(GPIO_INDEX_3|SDP_DATA_HI|SDP_DRIVE, (void *)((pbp_dev_master->mem_map) + REG_X710_GLGEN_GPIO_SET));
        readl((void *)(pbp_dev_master->mem_map) + REG_X710_GLGEN_GPIO_STAT);//flush write op
        ret = 0;
    }
    return ret;
}

//gpio_1 : scl
int x710_gpio_clear_scl(bp_dev_t* pbp_dev)
{
    int ret=BP_NOT_CAP;
    bp_dev_t* pbp_dev_master = NULL;

    if (pbp_dev->func==0){
        //clear scl(gpio_1)
        writel(0x3f00030, (void *)((pbp_dev->mem_map) + REG_X710_GLGEN_GPIO_CTL1));
        readl((void *)(pbp_dev->mem_map) + REG_X710_GLGEN_GPIO_STAT);//flush write op
        writel(GPIO_INDEX_1|SDP_DATA_LO|SDP_DRIVE, (void *)((pbp_dev->mem_map) + REG_X710_GLGEN_GPIO_SET));
        readl((void *)(pbp_dev->mem_map) + REG_X710_GLGEN_GPIO_STAT);//flush write op
        ret = 0;
    }
    else if (pbp_dev->func==2){
        pbp_dev_master = &(bp_dev_list[0]);
        //clear scl(gpio_3)
        writel(0x3f00030, (void *)((pbp_dev_master->mem_map) + REG_X710_GLGEN_GPIO_CTL3));
        readl((void *)(pbp_dev_master->mem_map) + REG_X710_GLGEN_GPIO_STAT);//flush write op
        writel(GPIO_INDEX_3|SDP_DATA_LO|SDP_DRIVE, (void *)((pbp_dev_master->mem_map) + REG_X710_GLGEN_GPIO_SET));
        readl((void *)(pbp_dev_master->mem_map) + REG_X710_GLGEN_GPIO_STAT);//flush write op
        ret = 0;
    }
    return ret;
}

//gpio_0 : sda
int x710_gpio_set_sda(bp_dev_t* pbp_dev)
{
    int ret=BP_NOT_CAP;
    bp_dev_t* pbp_dev_master = NULL;

    if (pbp_dev->func==0){
        //set sda(gpio_0)
        writel(0x3f00030, (void *)((pbp_dev->mem_map) + REG_X710_GLGEN_GPIO_CTL0));
        readl((void *)(pbp_dev->mem_map) + REG_X710_GLGEN_GPIO_STAT);//flush write op
        writel(GPIO_INDEX_0|SDP_DATA_HI|SDP_DRIVE, (void *)((pbp_dev->mem_map) + REG_X710_GLGEN_GPIO_SET));
        readl((void *)(pbp_dev->mem_map) + REG_X710_GLGEN_GPIO_STAT);//flush write op
        ret = 0;
    }
    else if (pbp_dev->func==2){
        pbp_dev_master = &(bp_dev_list[0]);
        //set sda(gpio_2)
        writel(0x3f00030, (void *)((pbp_dev_master->mem_map) + REG_X710_GLGEN_GPIO_CTL2));
        readl((void *)(pbp_dev_master->mem_map) + REG_X710_GLGEN_GPIO_STAT);//flush write op
        writel(GPIO_INDEX_2|SDP_DATA_HI|SDP_DRIVE, (void *)((pbp_dev_master->mem_map) + REG_X710_GLGEN_GPIO_SET));
        readl((void *)(pbp_dev_master->mem_map) + REG_X710_GLGEN_GPIO_STAT);//flush write op
        ret = 0;
    }
    return ret;
}

//gpio_0 : sda
int x710_gpio_clear_sda(bp_dev_t* pbp_dev)
{
    int ret=BP_NOT_CAP;
    bp_dev_t* pbp_dev_master = NULL;

    if (pbp_dev->func==0){
        //set sda(gpio_0)
        writel(0x3f00030, (void *)((pbp_dev->mem_map) + REG_X710_GLGEN_GPIO_CTL0));
        readl((void *)(pbp_dev->mem_map) + REG_X710_GLGEN_GPIO_STAT);//flush write op
        writel(GPIO_INDEX_0|SDP_DATA_LO|SDP_DRIVE, (void *)((pbp_dev->mem_map) + REG_X710_GLGEN_GPIO_SET));
        readl((void *)(pbp_dev->mem_map) + REG_X710_GLGEN_GPIO_STAT);//flush write op
        ret = 0;
    }
    else if (pbp_dev->func==2){
        pbp_dev_master = &(bp_dev_list[0]);
        //set sda(gpio_2)
        writel(0x3f00030, (void *)((pbp_dev_master->mem_map) + REG_X710_GLGEN_GPIO_CTL2));
        readl((void *)(pbp_dev_master->mem_map) + REG_X710_GLGEN_GPIO_STAT);//flush write op
        writel(GPIO_INDEX_2|SDP_DATA_LO|SDP_DRIVE, (void *)((pbp_dev_master->mem_map) + REG_X710_GLGEN_GPIO_SET));
        readl((void *)(pbp_dev_master->mem_map) + REG_X710_GLGEN_GPIO_STAT);//flush write op
        ret = 0;
    }
    return ret;
}

//gpio_0 : sda
int x710_gpio_sda_input(bp_dev_t* pbp_dev)
{
    int ret=BP_NOT_CAP;
    bp_dev_t* pbp_dev_master = NULL;

    if (pbp_dev->func==0){
        //set sda input(gpio_0)
        writel(0x3f00020, (void *)((pbp_dev->mem_map) + REG_X710_GLGEN_GPIO_CTL0));
        readl((void *)(pbp_dev->mem_map) + REG_X710_GLGEN_GPIO_STAT);//flush write op
        ret = 0;
    }
    else if (pbp_dev->func==2){
        pbp_dev_master = &(bp_dev_list[0]);
        //set sda input(gpio_2)
        writel(0x3f00020, (void *)((pbp_dev_master->mem_map) + REG_X710_GLGEN_GPIO_CTL2));
        readl((void *)(pbp_dev_master->mem_map) + REG_X710_GLGEN_GPIO_STAT);//flush write op
        ret = 0;
    }
    return ret;
}

//gpio_0 : sda
int x710_gpio_get_sda(bp_dev_t* pbp_dev,int *sda_stat)
{
    int ret=BP_NOT_CAP;
    int reg_data;
    bp_dev_t* pbp_dev_master = NULL;

    if (pbp_dev->func==0){
        //set sda input(gpio_0)
        //writel(0x3f00020, (void *)((pbp_dev->mem_map) + REG_X710_GLGEN_GPIO_CTL0));
        reg_data = readl((void *)(pbp_dev->mem_map) + REG_X710_GLGEN_GPIO_STAT);
        if(reg_data&0x01) *sda_stat = 1;
        else *sda_stat = 0;
        ret = 0;
    }
    else if (pbp_dev->func==2){
        pbp_dev_master = &(bp_dev_list[0]);
        //set sda input(gpio_2)
        //writel(0x3f00020, (void *)((pbp_dev_master->mem_map) + REG_X710_GLGEN_GPIO_CTL2));
        reg_data = readl((void *)(pbp_dev_master->mem_map) + REG_X710_GLGEN_GPIO_STAT);
        if(reg_data&0x04) *sda_stat = 1;
        else *sda_stat = 0;
        ret = 0;
    }
    return ret;
}


int x710_set_clk(bp_dev_t* pbp_dev)
{
     int ret=BP_NOT_CAP;
     //unsigned long flags;

    if (pbp_dev->func==0){

        uint32_t reg_i2c_sel = 0, reg_i2c_param = 0;

        //spin_lock_irqsave(&pbp_dev->bypass_wr_lock, flags);

        reg_i2c_sel = readl((void *)(pbp_dev->mem_map) + REG_X710_GLGEN_MDIO_I2C_SEL1);
        reg_i2c_param = readl((void *)(pbp_dev->mem_map) + REG_X710_GLGEN_I2C_PARAM1);

        writel(reg_i2c_sel|BIT_X710_SEL_I2C, (void *)((pbp_dev->mem_map) + REG_X710_GLGEN_MDIO_I2C_SEL1));
        reg_i2c_sel = readl((void *)(pbp_dev->mem_map) + REG_X710_GLGEN_MDIO_I2C_SEL1);//flush write op
        //trace("reg_i2c_sel 0x%x\n",reg_i2c_sel);
        reg_i2c_param |= BIT_X710_I2CBB_EN;
        reg_i2c_param &= ~BIT_X710_I2CCLK_OUT_DIS;//enable output,active low
        reg_i2c_param |= BIT_X710_I2CCLK_OUT;
        writel(reg_i2c_param, (void *)((pbp_dev->mem_map) + REG_X710_GLGEN_I2C_PARAM1));
        reg_i2c_param = readl((void *)(pbp_dev->mem_map) + REG_X710_GLGEN_I2C_PARAM1);//flush write op
        //trace("reg_i2c_param 0x%x\n",reg_i2c_param);

        //spin_unlock_irqrestore(&pbp_dev->bypass_wr_lock, flags);

        ret=0;
    }
    else if (pbp_dev->func==2){

        uint32_t reg_i2c_sel = 0, reg_i2c_param = 0;

        //spin_lock_irqsave(&pbp_dev->bypass_wr_lock, flags);

        reg_i2c_sel = readl((void *)(pbp_dev->mem_map) + REG_X710_GLGEN_MDIO_I2C_SEL3);
        reg_i2c_param = readl((void *)(pbp_dev->mem_map) + REG_X710_GLGEN_I2C_PARAM3);

        writel(reg_i2c_sel|BIT_X710_SEL_I2C, (void *)((pbp_dev->mem_map) + REG_X710_GLGEN_MDIO_I2C_SEL3));
        reg_i2c_sel = readl((void *)(pbp_dev->mem_map) + REG_X710_GLGEN_MDIO_I2C_SEL3);//flush write op
        //trace("reg_i2c_sel 0x%x\n",reg_i2c_sel);
        reg_i2c_param |= BIT_X710_I2CBB_EN;
        reg_i2c_param &= ~BIT_X710_I2CCLK_OUT_DIS;//enable output,active low
        reg_i2c_param |= BIT_X710_I2CCLK_OUT;
        writel(reg_i2c_param, (void *)((pbp_dev->mem_map) + REG_X710_GLGEN_I2C_PARAM3));
        reg_i2c_param = readl((void *)(pbp_dev->mem_map) + REG_X710_GLGEN_I2C_PARAM3);//flush write op
        //trace("reg_i2c_param 0x%x\n",reg_i2c_param);

        //spin_unlock_irqrestore(&pbp_dev->bypass_wr_lock, flags);

        ret=0;
    }
    return ret;
}

int x710_set_clk0(bp_dev_t* pbp_dev)
{
     int ret=BP_NOT_CAP;
     //unsigned long flags;

    if (pbp_dev->func==0){

        uint32_t reg_i2c_sel = 0, reg_i2c_param = 0;

        //spin_lock_irqsave(&pbp_dev->bypass_wr_lock, flags);

        reg_i2c_sel = readl((void *)(pbp_dev->mem_map) + REG_X710_GLGEN_MDIO_I2C_SEL0);
        reg_i2c_param = readl((void *)(pbp_dev->mem_map) + REG_X710_GLGEN_I2C_PARAM0);

        writel(reg_i2c_sel|BIT_X710_SEL_I2C, (void *)((pbp_dev->mem_map) + REG_X710_GLGEN_MDIO_I2C_SEL0));
        reg_i2c_sel = readl((void *)(pbp_dev->mem_map) + REG_X710_GLGEN_MDIO_I2C_SEL0);//flush write op
        //trace("reg_i2c_sel 0x%x\n",reg_i2c_sel);
        reg_i2c_param |= BIT_X710_I2CBB_EN;
        reg_i2c_param &= ~BIT_X710_I2CCLK_OUT_DIS;//enable output,active low
        reg_i2c_param |= BIT_X710_I2CCLK_OUT;
        writel(reg_i2c_param, (void *)((pbp_dev->mem_map) + REG_X710_GLGEN_I2C_PARAM0));
        reg_i2c_param = readl((void *)(pbp_dev->mem_map) + REG_X710_GLGEN_I2C_PARAM0);//flush write op
        //trace("reg_i2c_param 0x%x\n",reg_i2c_param);

        //spin_unlock_irqrestore(&pbp_dev->bypass_wr_lock, flags);

        ret=0;
    }
    else if (pbp_dev->func==2){

        uint32_t reg_i2c_sel = 0, reg_i2c_param = 0;

        //spin_lock_irqsave(&pbp_dev->bypass_wr_lock, flags);

        reg_i2c_sel = readl((void *)(pbp_dev->mem_map) + REG_X710_GLGEN_MDIO_I2C_SEL2);
        reg_i2c_param = readl((void *)(pbp_dev->mem_map) + REG_X710_GLGEN_I2C_PARAM2);

        writel(reg_i2c_sel|BIT_X710_SEL_I2C, (void *)((pbp_dev->mem_map) + REG_X710_GLGEN_MDIO_I2C_SEL2));
        reg_i2c_sel = readl((void *)(pbp_dev->mem_map) + REG_X710_GLGEN_MDIO_I2C_SEL2);//flush write op
        //trace("reg_i2c_sel 0x%x\n",reg_i2c_sel);
        reg_i2c_param |= BIT_X710_I2CBB_EN;
        reg_i2c_param &= ~BIT_X710_I2CCLK_OUT_DIS;//enable output,active low
        reg_i2c_param |= BIT_X710_I2CCLK_OUT;
        writel(reg_i2c_param, (void *)((pbp_dev->mem_map) + REG_X710_GLGEN_I2C_PARAM2));
        reg_i2c_param = readl((void *)(pbp_dev->mem_map) + REG_X710_GLGEN_I2C_PARAM2);//flush write op
        //trace("reg_i2c_param 0x%x\n",reg_i2c_param);

        //spin_unlock_irqrestore(&pbp_dev->bypass_wr_lock, flags);

        ret=0;
    }
    return ret;
}

int x710_clear_clk(bp_dev_t* pbp_dev)
{
     int ret=BP_NOT_CAP;
     //unsigned long flags;

    if (pbp_dev->func==0){

        uint32_t reg_i2c_sel = 0, reg_i2c_param = 0;

        //spin_lock_irqsave(&pbp_dev->bypass_wr_lock, flags);

        reg_i2c_sel = readl((void *)(pbp_dev->mem_map) + REG_X710_GLGEN_MDIO_I2C_SEL1);
        reg_i2c_param = readl((void *)(pbp_dev->mem_map) + REG_X710_GLGEN_I2C_PARAM1);

        writel(reg_i2c_sel|BIT_X710_SEL_I2C, (void *)((pbp_dev->mem_map) + REG_X710_GLGEN_MDIO_I2C_SEL1));
        reg_i2c_sel = readl((void *)(pbp_dev->mem_map) + REG_X710_GLGEN_MDIO_I2C_SEL1);//flush write op
        //trace("reg_i2c_sel 0x%x\n",reg_i2c_sel);
        reg_i2c_param |= BIT_X710_I2CBB_EN;
        reg_i2c_param &= ~BIT_X710_I2CCLK_OUT_DIS;//enable output,active low
        reg_i2c_param &= ~BIT_X710_I2CCLK_OUT;
        writel(reg_i2c_param, (void *)((pbp_dev->mem_map) + REG_X710_GLGEN_I2C_PARAM1));
        reg_i2c_param = readl((void *)(pbp_dev->mem_map) + REG_X710_GLGEN_I2C_PARAM1);//flush write op
        //trace("reg_i2c_param 0x%x\n",reg_i2c_param);

        //spin_unlock_irqrestore(&pbp_dev->bypass_wr_lock, flags);

        ret=0;
    }
    else if (pbp_dev->func==2){

        uint32_t reg_i2c_sel = 0, reg_i2c_param = 0;

        //spin_lock_irqsave(&pbp_dev->bypass_wr_lock, flags);

        reg_i2c_sel = readl((void *)(pbp_dev->mem_map) + REG_X710_GLGEN_MDIO_I2C_SEL3);
        reg_i2c_param = readl((void *)(pbp_dev->mem_map) + REG_X710_GLGEN_I2C_PARAM3);

        writel(reg_i2c_sel|BIT_X710_SEL_I2C, (void *)((pbp_dev->mem_map) + REG_X710_GLGEN_MDIO_I2C_SEL3));
        reg_i2c_sel = readl((void *)(pbp_dev->mem_map) + REG_X710_GLGEN_MDIO_I2C_SEL3);//flush write op
        //trace("reg_i2c_sel 0x%x\n",reg_i2c_sel);
        reg_i2c_param |= BIT_X710_I2CBB_EN;
        reg_i2c_param &= ~BIT_X710_I2CCLK_OUT_DIS;//enable output,active low
        reg_i2c_param &= ~BIT_X710_I2CCLK_OUT;
        writel(reg_i2c_param, (void *)((pbp_dev->mem_map) + REG_X710_GLGEN_I2C_PARAM3));
        reg_i2c_param = readl((void *)(pbp_dev->mem_map) + REG_X710_GLGEN_I2C_PARAM3);//flush write op
        //trace("reg_i2c_param 0x%x\n",reg_i2c_param);

        //spin_unlock_irqrestore(&pbp_dev->bypass_wr_lock, flags);

        ret=0;
    }
    return ret;
}

int x710_clear_clk0(bp_dev_t* pbp_dev)
{
     int ret=BP_NOT_CAP;
     //unsigned long flags;

    if (pbp_dev->func==0){

        uint32_t reg_i2c_sel = 0, reg_i2c_param = 0;

        //spin_lock_irqsave(&pbp_dev->bypass_wr_lock, flags);

        reg_i2c_sel = readl((void *)(pbp_dev->mem_map) + REG_X710_GLGEN_MDIO_I2C_SEL0);
        reg_i2c_param = readl((void *)(pbp_dev->mem_map) + REG_X710_GLGEN_I2C_PARAM0);

        writel(reg_i2c_sel|BIT_X710_SEL_I2C, (void *)((pbp_dev->mem_map) + REG_X710_GLGEN_MDIO_I2C_SEL0));
        reg_i2c_sel = readl((void *)(pbp_dev->mem_map) + REG_X710_GLGEN_MDIO_I2C_SEL0);//flush write op
        //trace("reg_i2c_sel 0x%x\n",reg_i2c_sel);
        reg_i2c_param |= BIT_X710_I2CBB_EN;
        reg_i2c_param &= ~BIT_X710_I2CCLK_OUT_DIS;//enable output,active low
        reg_i2c_param &= ~BIT_X710_I2CCLK_OUT;
        writel(reg_i2c_param, (void *)((pbp_dev->mem_map) + REG_X710_GLGEN_I2C_PARAM0));
        reg_i2c_param = readl((void *)(pbp_dev->mem_map) + REG_X710_GLGEN_I2C_PARAM0);//flush write op
        //trace("reg_i2c_param 0x%x\n",reg_i2c_param);

        //spin_unlock_irqrestore(&pbp_dev->bypass_wr_lock, flags);

        ret=0;
    }
    else if (pbp_dev->func==2){

        uint32_t reg_i2c_sel = 0, reg_i2c_param = 0;

        //spin_lock_irqsave(&pbp_dev->bypass_wr_lock, flags);

        reg_i2c_sel = readl((void *)(pbp_dev->mem_map) + REG_X710_GLGEN_MDIO_I2C_SEL2);
        reg_i2c_param = readl((void *)(pbp_dev->mem_map) + REG_X710_GLGEN_I2C_PARAM2);

        writel(reg_i2c_sel|BIT_X710_SEL_I2C, (void *)((pbp_dev->mem_map) + REG_X710_GLGEN_MDIO_I2C_SEL2));
        reg_i2c_sel = readl((void *)(pbp_dev->mem_map) + REG_X710_GLGEN_MDIO_I2C_SEL2);//flush write op
        //trace("reg_i2c_sel 0x%x\n",reg_i2c_sel);
        reg_i2c_param |= BIT_X710_I2CBB_EN;
        reg_i2c_param &= ~BIT_X710_I2CCLK_OUT_DIS;//enable output,active low
        reg_i2c_param &= ~BIT_X710_I2CCLK_OUT;
        writel(reg_i2c_param, (void *)((pbp_dev->mem_map) + REG_X710_GLGEN_I2C_PARAM2));
        reg_i2c_param = readl((void *)(pbp_dev->mem_map) + REG_X710_GLGEN_I2C_PARAM2);//flush write op
        //trace("reg_i2c_param 0x%x\n",reg_i2c_param);

        //spin_unlock_irqrestore(&pbp_dev->bypass_wr_lock, flags);

        ret=0;
    }
    return ret;
}


int x710_set_dat(bp_dev_t* pbp_dev)
{
     int ret=BP_NOT_CAP;
     //unsigned long flags;

    if (pbp_dev->func==0){

        uint32_t reg_i2c_sel = 0, reg_i2c_param = 0;

        //spin_lock_irqsave(&pbp_dev->bypass_wr_lock, flags);

        reg_i2c_sel = readl((void *)(pbp_dev->mem_map) + REG_X710_GLGEN_MDIO_I2C_SEL1);
        reg_i2c_param = readl((void *)(pbp_dev->mem_map) + REG_X710_GLGEN_I2C_PARAM1);

        writel(reg_i2c_sel|BIT_X710_SEL_I2C, (void *)((pbp_dev->mem_map) + REG_X710_GLGEN_MDIO_I2C_SEL1));
        reg_i2c_sel = readl((void *)(pbp_dev->mem_map) + REG_X710_GLGEN_MDIO_I2C_SEL1);//flush write op
        //trace("reg_i2c_sel 0x%x\n",reg_i2c_sel);
        reg_i2c_param |= BIT_X710_I2CBB_EN;
        reg_i2c_param &= ~BIT_X710_I2CDAT_OUT_DIS;//enable output,active low
        reg_i2c_param |= BIT_X710_I2CDAT_OUT;
        //reg_i2c_param &= ~BIT_X710_I2CDAT_OUT;
        writel(reg_i2c_param, (void *)((pbp_dev->mem_map) + REG_X710_GLGEN_I2C_PARAM1));
        reg_i2c_param = readl((void *)(pbp_dev->mem_map) + REG_X710_GLGEN_I2C_PARAM1);//flush write op
        //trace("reg_i2c_param 0x%x\n",reg_i2c_param);

        //spin_unlock_irqrestore(&pbp_dev->bypass_wr_lock, flags);

        ret=0;
    }
    else if (pbp_dev->func==2){

        uint32_t reg_i2c_sel = 0, reg_i2c_param = 0;

        //spin_lock_irqsave(&pbp_dev->bypass_wr_lock, flags);

        reg_i2c_sel = readl((void *)(pbp_dev->mem_map) + REG_X710_GLGEN_MDIO_I2C_SEL3);
        reg_i2c_param = readl((void *)(pbp_dev->mem_map) + REG_X710_GLGEN_I2C_PARAM3);

        writel(reg_i2c_sel|BIT_X710_SEL_I2C, (void *)((pbp_dev->mem_map) + REG_X710_GLGEN_MDIO_I2C_SEL3));
        reg_i2c_sel = readl((void *)(pbp_dev->mem_map) + REG_X710_GLGEN_MDIO_I2C_SEL3);//flush write op
        //trace("reg_i2c_sel 0x%x\n",reg_i2c_sel);
        reg_i2c_param |= BIT_X710_I2CBB_EN;
        reg_i2c_param &= ~BIT_X710_I2CDAT_OUT_DIS;//enable output,active low
        reg_i2c_param |= BIT_X710_I2CDAT_OUT;
        //reg_i2c_param &= ~BIT_X710_I2CDAT_OUT;
        writel(reg_i2c_param, (void *)((pbp_dev->mem_map) + REG_X710_GLGEN_I2C_PARAM3));
        reg_i2c_param = readl((void *)(pbp_dev->mem_map) + REG_X710_GLGEN_I2C_PARAM3);//flush write op
        //trace("reg_i2c_param 0x%x\n",reg_i2c_param);

        //spin_unlock_irqrestore(&pbp_dev->bypass_wr_lock, flags);

        ret=0;
    }
    return ret;
}


int x710_set_dat0(bp_dev_t* pbp_dev)
{
     int ret=BP_NOT_CAP;
     //unsigned long flags;

    if (pbp_dev->func==0){

        uint32_t reg_i2c_sel = 0, reg_i2c_param = 0;

        //spin_lock_irqsave(&pbp_dev->bypass_wr_lock, flags);

        reg_i2c_sel = readl((void *)(pbp_dev->mem_map) + REG_X710_GLGEN_MDIO_I2C_SEL0);
        reg_i2c_param = readl((void *)(pbp_dev->mem_map) + REG_X710_GLGEN_I2C_PARAM0);

        writel(reg_i2c_sel|BIT_X710_SEL_I2C, (void *)((pbp_dev->mem_map) + REG_X710_GLGEN_MDIO_I2C_SEL0));
        reg_i2c_sel = readl((void *)(pbp_dev->mem_map) + REG_X710_GLGEN_MDIO_I2C_SEL0);//flush write op
        //trace("reg_i2c_sel 0x%x\n",reg_i2c_sel);
        reg_i2c_param |= BIT_X710_I2CBB_EN;
        reg_i2c_param &= ~BIT_X710_I2CDAT_OUT_DIS;//enable output,active low
        reg_i2c_param |= BIT_X710_I2CDAT_OUT;
        //reg_i2c_param &= ~BIT_X710_I2CDAT_OUT;
        writel(reg_i2c_param, (void *)((pbp_dev->mem_map) + REG_X710_GLGEN_I2C_PARAM0));
        reg_i2c_param = readl((void *)(pbp_dev->mem_map) + REG_X710_GLGEN_I2C_PARAM0);//flush write op
        //trace("reg_i2c_param 0x%x\n",reg_i2c_param);

        //spin_unlock_irqrestore(&pbp_dev->bypass_wr_lock, flags);

        ret=0;
    }
    else if (pbp_dev->func==2){

        uint32_t reg_i2c_sel = 0, reg_i2c_param = 0;

        //spin_lock_irqsave(&pbp_dev->bypass_wr_lock, flags);

        reg_i2c_sel = readl((void *)(pbp_dev->mem_map) + REG_X710_GLGEN_MDIO_I2C_SEL2);
        reg_i2c_param = readl((void *)(pbp_dev->mem_map) + REG_X710_GLGEN_I2C_PARAM2);

        writel(reg_i2c_sel|BIT_X710_SEL_I2C, (void *)((pbp_dev->mem_map) + REG_X710_GLGEN_MDIO_I2C_SEL2));
        reg_i2c_sel = readl((void *)(pbp_dev->mem_map) + REG_X710_GLGEN_MDIO_I2C_SEL2);//flush write op
        //trace("reg_i2c_sel 0x%x\n",reg_i2c_sel);
        reg_i2c_param |= BIT_X710_I2CBB_EN;
        reg_i2c_param &= ~BIT_X710_I2CDAT_OUT_DIS;//enable output,active low
        reg_i2c_param |= BIT_X710_I2CDAT_OUT;
        //reg_i2c_param &= ~BIT_X710_I2CDAT_OUT;
        writel(reg_i2c_param, (void *)((pbp_dev->mem_map) + REG_X710_GLGEN_I2C_PARAM2));
        reg_i2c_param = readl((void *)(pbp_dev->mem_map) + REG_X710_GLGEN_I2C_PARAM2);//flush write op
        //trace("reg_i2c_param 0x%x\n",reg_i2c_param);

        //spin_unlock_irqrestore(&pbp_dev->bypass_wr_lock, flags);

        ret=0;
    }
    return ret;
}

int x710_clear_dat(bp_dev_t* pbp_dev)
{
     int ret=BP_NOT_CAP;
     //unsigned long flags;

    if (pbp_dev->func==0){

        uint32_t reg_i2c_sel = 0, reg_i2c_param = 0;

        //spin_lock_irqsave(&pbp_dev->bypass_wr_lock, flags);

        reg_i2c_sel = readl((void *)(pbp_dev->mem_map) + REG_X710_GLGEN_MDIO_I2C_SEL1);
        reg_i2c_param = readl((void *)(pbp_dev->mem_map) + REG_X710_GLGEN_I2C_PARAM1);

        writel(reg_i2c_sel|BIT_X710_SEL_I2C, (void *)((pbp_dev->mem_map) + REG_X710_GLGEN_MDIO_I2C_SEL1));
        reg_i2c_sel = readl((void *)(pbp_dev->mem_map) + REG_X710_GLGEN_MDIO_I2C_SEL1);//flush write op
        //trace("reg_i2c_sel 0x%x\n",reg_i2c_sel);
        reg_i2c_param |= BIT_X710_I2CBB_EN;
        reg_i2c_param &= ~BIT_X710_I2CDAT_OUT_DIS;//enable output,active low
        //reg_i2c_param |= BIT_X710_I2CDAT_OUT;
        reg_i2c_param &= ~BIT_X710_I2CDAT_OUT;
        writel(reg_i2c_param, (void *)((pbp_dev->mem_map) + REG_X710_GLGEN_I2C_PARAM1));
        reg_i2c_param = readl((void *)(pbp_dev->mem_map) + REG_X710_GLGEN_I2C_PARAM1);//flush write op
        //trace("reg_i2c_param 0x%x\n",reg_i2c_param);

        //spin_unlock_irqrestore(&pbp_dev->bypass_wr_lock, flags);

        ret=0;
    }
    else if (pbp_dev->func==2){

        uint32_t reg_i2c_sel = 0, reg_i2c_param = 0;

        //spin_lock_irqsave(&pbp_dev->bypass_wr_lock, flags);

        reg_i2c_sel = readl((void *)(pbp_dev->mem_map) + REG_X710_GLGEN_MDIO_I2C_SEL3);
        reg_i2c_param = readl((void *)(pbp_dev->mem_map) + REG_X710_GLGEN_I2C_PARAM3);

        writel(reg_i2c_sel|BIT_X710_SEL_I2C, (void *)((pbp_dev->mem_map) + REG_X710_GLGEN_MDIO_I2C_SEL3));
        reg_i2c_sel = readl((void *)(pbp_dev->mem_map) + REG_X710_GLGEN_MDIO_I2C_SEL3);//flush write op
        //trace("reg_i2c_sel 0x%x\n",reg_i2c_sel);
        reg_i2c_param |= BIT_X710_I2CBB_EN;
        reg_i2c_param &= ~BIT_X710_I2CDAT_OUT_DIS;//enable output,active low
        //reg_i2c_param |= BIT_X710_I2CDAT_OUT;
        reg_i2c_param &= ~BIT_X710_I2CDAT_OUT;
        writel(reg_i2c_param, (void *)((pbp_dev->mem_map) + REG_X710_GLGEN_I2C_PARAM3));
        reg_i2c_param = readl((void *)(pbp_dev->mem_map) + REG_X710_GLGEN_I2C_PARAM3);//flush write op
        //trace("reg_i2c_param 0x%x\n",reg_i2c_param);

        //spin_unlock_irqrestore(&pbp_dev->bypass_wr_lock, flags);

        ret=0;
    }
    return ret;
}

int x710_clear_dat0(bp_dev_t* pbp_dev)
{
     int ret=BP_NOT_CAP;
     //unsigned long flags;

    if (pbp_dev->func==0){

        uint32_t reg_i2c_sel = 0, reg_i2c_param = 0;

        //spin_lock_irqsave(&pbp_dev->bypass_wr_lock, flags);

        reg_i2c_sel = readl((void *)(pbp_dev->mem_map) + REG_X710_GLGEN_MDIO_I2C_SEL0);
        reg_i2c_param = readl((void *)(pbp_dev->mem_map) + REG_X710_GLGEN_I2C_PARAM0);

        writel(reg_i2c_sel|BIT_X710_SEL_I2C, (void *)((pbp_dev->mem_map) + REG_X710_GLGEN_MDIO_I2C_SEL0));
        reg_i2c_sel = readl((void *)(pbp_dev->mem_map) + REG_X710_GLGEN_MDIO_I2C_SEL0);//flush write op
        //trace("reg_i2c_sel 0x%x\n",reg_i2c_sel);
        reg_i2c_param |= BIT_X710_I2CBB_EN;
        reg_i2c_param &= ~BIT_X710_I2CDAT_OUT_DIS;//enable output,active low
        //reg_i2c_param |= BIT_X710_I2CDAT_OUT;
        reg_i2c_param &= ~BIT_X710_I2CDAT_OUT;
        writel(reg_i2c_param, (void *)((pbp_dev->mem_map) + REG_X710_GLGEN_I2C_PARAM0));
        reg_i2c_param = readl((void *)(pbp_dev->mem_map) + REG_X710_GLGEN_I2C_PARAM0);//flush write op
        //trace("reg_i2c_param 0x%x\n",reg_i2c_param);

        //spin_unlock_irqrestore(&pbp_dev->bypass_wr_lock, flags);

        ret=0;
    }
    else if (pbp_dev->func==2){

        uint32_t reg_i2c_sel = 0, reg_i2c_param = 0;

        //spin_lock_irqsave(&pbp_dev->bypass_wr_lock, flags);

        reg_i2c_sel = readl((void *)(pbp_dev->mem_map) + REG_X710_GLGEN_MDIO_I2C_SEL2);
        reg_i2c_param = readl((void *)(pbp_dev->mem_map) + REG_X710_GLGEN_I2C_PARAM2);

        writel(reg_i2c_sel|BIT_X710_SEL_I2C, (void *)((pbp_dev->mem_map) + REG_X710_GLGEN_MDIO_I2C_SEL2));
        reg_i2c_sel = readl((void *)(pbp_dev->mem_map) + REG_X710_GLGEN_MDIO_I2C_SEL2);//flush write op
        //trace("reg_i2c_sel 0x%x\n",reg_i2c_sel);
        reg_i2c_param |= BIT_X710_I2CBB_EN;
        reg_i2c_param &= ~BIT_X710_I2CDAT_OUT_DIS;//enable output,active low
        //reg_i2c_param |= BIT_X710_I2CDAT_OUT;
        reg_i2c_param &= ~BIT_X710_I2CDAT_OUT;
        writel(reg_i2c_param, (void *)((pbp_dev->mem_map) + REG_X710_GLGEN_I2C_PARAM2));
        reg_i2c_param = readl((void *)(pbp_dev->mem_map) + REG_X710_GLGEN_I2C_PARAM2);//flush write op
        //trace("reg_i2c_param 0x%x\n",reg_i2c_param);

        //spin_unlock_irqrestore(&pbp_dev->bypass_wr_lock, flags);

        ret=0;
    }
    return ret;
}



int bp_set_bypass_fn(bp_dev_t* pbp_dev, int bypass_mode) {
    int ret = 0;
    trace("bp_set_bypass_fn\n");//lan

    if ((ret = bp_cmnd_on(pbp_dev)) < 0)
        return ret;

    if (!bypass_mode)
        ret = bp_bypass_off(pbp_dev);
    else
        ret = bp_bypass_on(pbp_dev);

    bp_cmnd_off(pbp_dev);

    return ret;
}



int bp_get_bypass_fn(bp_dev_t* pbp_dev) 
{
    //return(bp_bypass_status(pbp_dev));
    int reg_data;
    if ((pbp_dev->func == 0) || (pbp_dev->func == 2)){
        reg_data = bp_read_reg(pbp_dev, STATUS_REG_ADDR);
        TRACE("read status reg 0x%x\n",reg_data);
        return(((reg_data & 0x04) == 0x04) ? 1 : 0);
    }else
        return BP_NOT_CAP;
}


int bp_set_bypass_at_power_off(bp_dev_t* pdev, int bypass_mode) {
    int ret = 0;

    if (!pdev) {
        return -1;
    }

    if ((ret = bp_cmnd_on(pdev)) < 0) {
        return ret;
    }
    if (bypass_mode) {
        bp_write_data(pdev, BYPASS_STATE_PWROFF);
        msec_delay_bp(EEPROM_WR_DELAY);
    }
    else {
        bp_write_data(pdev, NORMAL_STATE_PWROFF);
        msec_delay_bp(EEPROM_WR_DELAY);
    }
    ret = bp_cmnd_off(pdev);
    return ret;

}


int bp_get_bypass_at_power_off(bp_dev_t* pdev)
{
    if ((pdev->func == 0) || (pdev->func == 2)){
        return((((bp_read_reg(pdev, STATUS_REG_ADDR)) & DFLT_PWROFF_MASK) == DFLT_PWROFF_MASK) ? 0 : 1);
    }else
        return BP_NOT_CAP;
}


int bp_set_bypass_at_power_on(bp_dev_t* pdev, int bypass_mode) {
    int ret = 0;

    if (!pdev) {
        return -1;
    }

    if ((ret = bp_cmnd_on(pdev)) < 0) {
        return ret;
    }
    if (bypass_mode) {
        bp_write_data(pdev, BYPASS_STATE_PWRON);
        msec_delay_bp(EEPROM_WR_DELAY);
    }
    else {
        bp_write_data(pdev, NORMAL_STATE_PWRON);
        msec_delay_bp(EEPROM_WR_DELAY);
    }
    ret = bp_cmnd_off(pdev);
    return ret;

}


int bp_get_bypass_at_power_on(bp_dev_t* pdev)
{
    if ((pdev->func == 0) || (pdev->func == 2)) {
        return((((bp_read_reg(pdev, STATUS_REG_ADDR)) & DFLT_PWRON_MASK) == DFLT_PWRON_MASK) ? 0 : 1);
    }
    else
        return BP_NOT_CAP;
}


int bp_get_dev_index(bp_dev_t *pdev)
{
    int index;
    if (pdev == 0)return -1;
    for (index = 0; index < device_num; index++) {
        if (pdev == (bp_dev_t *)&bp_dev_list[index]) {
            return index;
        }
    }
    return -1;
}


int get_dev_idx_bsf(int bus, int slot, int func){
    int idx_dev=0;
    //if_scan();
    for (idx_dev = 0; ((idx_dev<device_num)&&(bpctl_dev_arr[idx_dev].pdev!=NULL)); idx_dev++) {
        if ((bus==bpctl_dev_arr[idx_dev].bus) &&
            (slot==bpctl_dev_arr[idx_dev].slot) &&
            (func==bpctl_dev_arr[idx_dev].func) ) {
			return idx_dev;
		}
    }
    return -1;
}

int bp_bsf_to_idx(int bus, int slot, int func) {
    int idx_dev = 0;
    //if_scan();
    for (idx_dev = 0; ((idx_dev < device_num) && (bp_dev_list[idx_dev].pdev != NULL)); idx_dev++) {
        if ((bus == bp_dev_list[idx_dev].bus) &&
            (slot == bp_dev_list[idx_dev].slot) &&
            (func == bp_dev_list[idx_dev].func)) {
            return idx_dev;
        }
    }
    return -1;
}



static int bp_if_to_idx(int ifindex) {
    int idx_dev = 0;

    for (idx_dev = 0; ((idx_dev < device_num) && (bp_dev_list[idx_dev].pdev != NULL)); idx_dev++) {
        if (ifindex == bp_dev_list[idx_dev].ifindex) {
            return idx_dev;
        }
    }

    return -1;
}


bp_dev_t* get_bp_dev(int bus, int slot, int func, int ifindex) 
{
    int index;
    if (bus||slot||func)
    {
        for (index = 0; index < device_num; index++) {
            if ((bus == bp_dev_list[index].bus) &&
                (slot == bp_dev_list[index].slot) &&
                (func == bp_dev_list[index].func)) {
                return &bp_dev_list[index];
            }
        }
    }

    if (ifindex)
    {
        for (index = 0; index < device_num; index++) {
            if (ifindex == bp_dev_list[index].ifindex) {
                return &bp_dev_list[index];
            }
        }
    }

    return 0;
}


int bp_set_std_nic(bp_dev_t *pbpctl_dev, int nic_mode){
    int ret=0;

    if (!pbpctl_dev) {
        return -1;
    }
 

    if ((ret=bp_cmnd_on(pbpctl_dev))<0) {
        return ret;
    }
    if (nic_mode) {
        bp_write_data(pbpctl_dev,STD_NIC_ON);
        msec_delay_bp(BYPASS_CAP_DELAY);
    } else {
        bp_write_data(pbpctl_dev,STD_NIC_OFF);
        msec_delay_bp(BYPASS_CAP_DELAY);
    }
    bp_cmnd_off(pbpctl_dev);
    return ret;
}

int bp_get_std_nic(bp_dev_t *pdev){
    if (!pdev) {
        return -1;
    }

    if ((pdev->func == 0) || (pdev->func == 2)) {
        return((((bp_read_reg(pdev, STATUS_DISC_REG_ADDR)) & STD_NIC_ON_MASK) == STD_NIC_ON_MASK) ? 1 : 0);
    }
    else
        return BP_NOT_CAP;
}

int bp_get_bypass_slave(bp_dev_t *pbpctl_dev,bp_dev_t **pbpctl_dev_slave)
{
    int idx_dev=0;

    if (!pbpctl_dev) {
        return -1;
    }

    //trace("bp_get_bypass_slave func %d",pbpctl_dev->func);

    if ((pbpctl_dev->func==0)||(pbpctl_dev->func==2)) {
        for (idx_dev = 0; ((idx_dev<device_num)&&(bp_dev_list[idx_dev].pdev!=NULL)); idx_dev++) {
            if ((bp_dev_list[idx_dev].bus==pbpctl_dev->bus)&&
                (bp_dev_list[idx_dev].slot==pbpctl_dev->slot)) {
                if ((pbpctl_dev->func==0)&&
                    (bp_dev_list[idx_dev].func==1)) {
                    *pbpctl_dev_slave=&bp_dev_list[idx_dev];
                    return 1;
                }
                if ((pbpctl_dev->func==2)&&
                    (bp_dev_list[idx_dev].func==3)) {
                    *pbpctl_dev_slave=&bp_dev_list[idx_dev];
                    return 1;
                }
            }
        }
        return -1;
    } else {
        return 0;
    }
}

int bp_set_bypass_wd(bp_dev_t *pbpctl_dev, int timeout)
{
    int ret=0;
    unsigned int pulse=0, temp_value=0, temp_cnt=0;

    pbpctl_dev->wdt_status=0; 

    if (!pbpctl_dev) {
        return -1;
    }

    if ((ret=bp_cmnd_on(pbpctl_dev))<0) {
        return ret;
    }
    if (!timeout) {
         bp_write_data(pbpctl_dev,WDT_OFF);
         pbpctl_dev->wdt_status=WDT_STATUS_DIS;
         ret=0;
    } else {
        timeout=(timeout<TIMEOUT_UNIT?TIMEOUT_UNIT:(timeout>WDT_TIMEOUT_MAX?WDT_TIMEOUT_MAX:timeout));
        temp_value=timeout/100;
        while ((temp_value>>=1))
            temp_cnt++;
        if (timeout > ((1<<temp_cnt)*100))
            temp_cnt++;
        pbpctl_dev->bypass_wdt_on_time=jiffies;
        pulse=(WDT_ON | temp_cnt);
        bp_write_data(pbpctl_dev,pulse);
        pbpctl_dev->bypass_timer_interval=(1<<temp_cnt)*100;
        pbpctl_dev->wdt_status=WDT_STATUS_EN;
        ret = pbpctl_dev->bypass_timer_interval;
    }
    bp_cmnd_off(pbpctl_dev);
    return ret;
}



int bp_get_bypass_wd(bp_dev_t *pbpctl_dev, int *timeout)
{
    int ret=0;
    u8 wdt_val;
    if (!pbpctl_dev) {
        return -1;
    }

    if ((pbpctl_dev->func==0)||(pbpctl_dev->func==2)) {
        if ((bp_read_reg(pbpctl_dev,STATUS_REG_ADDR))&WDT_EN_MASK) {
            wdt_val=bp_read_reg(pbpctl_dev,WDT_REG_ADDR);
            *timeout=  (1<<wdt_val)*100;
        } else *timeout=0;
        ret = 0;
    }else 
        ret=BP_NOT_CAP;

    return ret;
}

int bp_reset_bypass_wd(bp_dev_t *pbpctl_dev)
{
    int ret=0;
    if (!pbpctl_dev)return -1;

    if (((pbpctl_dev->func==0)||(pbpctl_dev->func==2))&&(pbpctl_dev->wdt_status!=WDT_STATUS_UNKNOWN)){
        ret= bp_write_wdt_bit(pbpctl_dev);
        return 1;
    }
    return BP_NOT_CAP; 
}

int bp_set_disc(bp_dev_t *pbpctl_dev, int disc_mode)
{

    if (!pbpctl_dev) {
        return -1;
    }

    if ((bp_cmnd_on(pbpctl_dev)>=0)) {
        if (!disc_mode) {
            bp_write_data(pbpctl_dev,DISC_OFF);
            msec_delay_bp(LATCH_DELAY);
        } else {
            bp_write_data(pbpctl_dev,DISC_ON);
            msec_delay_bp(LATCH_DELAY);
        }
        bp_cmnd_off(pbpctl_dev);

        return BP_OK;
    }
    return BP_NOT_CAP;

}

int bp_get_disc(bp_dev_t *pbpctl_dev)
{
    int ret=0;

    if (!pbpctl_dev) {
        return -1;
    }

    ret = bp_disc_status(pbpctl_dev);

    return ret;

}

#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,30))
static int bp_device_ioctl(struct inode *inode, /* see include/linux/fs.h */
                        struct file *file, /* ditto */
                        unsigned int ioctl_num, /* number and param for ioctl */
                        unsigned long ioctl_param)
#else
static long bp_device_ioctl(struct file *file, /* ditto */
                         unsigned int ioctl_num, /* number and param for ioctl */
                         unsigned long ioctl_param)

#endif
{
    struct bpctl_cmd bpctl_cmd;
    int dev_idx=0;
    //bpctl_dev_t *pbpctl_dev_out;
    void __user *argp = (void __user *)ioctl_param; 
    int ret=0;
    //bpctl_dev_t *pbpctl_dev;
    bp_dev_t* pbp_dev;
    bp_dev_t *pbp_dev_slave;
    //trace("bp_device_ioctl entry\n");
    lock_bpctl();

    if (copy_from_user(&bpctl_cmd, argp, sizeof(struct bpctl_cmd))) {

        ret= -EFAULT; 
        goto bp_exit;
    }

    if (ioctl_num==IOCTL_TX_MSG(GET_DEV_NUM)) {
        bpctl_cmd.out_param[0]= device_num;
        goto bpcmd_exit;

    }

    if (ioctl_num==IOCTL_TX_MSG(RESET_BYPASS_WD_TIMER)) {
        if (bpctl_cmd.in_param[0]==-1) {
            int idx_dev=0;
            for (idx_dev = 0; ((idx_dev<device_num)&&(bp_dev_list[idx_dev].pdev!=NULL));
                 idx_dev++) {
                if (is_bypass_fn(&bp_dev_list[idx_dev])) {
                    bp_reset_bypass_wd(&bp_dev_list[idx_dev]);
                }
            }
            bpctl_cmd.status=1;
            goto bpcmd_exit;
        }
    }

    if ((bpctl_cmd.in_param[5])||
        (bpctl_cmd.in_param[6])||
        (bpctl_cmd.in_param[7]))
        dev_idx= bp_bsf_to_idx(bpctl_cmd.in_param[5],
                                bpctl_cmd.in_param[6],
                                bpctl_cmd.in_param[7]);
    else if (bpctl_cmd.in_param[1]==0)
        dev_idx= bpctl_cmd.in_param[0];
    else dev_idx= bp_if_to_idx(bpctl_cmd.in_param[1]);

    //trace("bp_device_ioctl dev_idx %d\n",dev_idx);

    if (dev_idx<0||dev_idx>device_num) {
        ret= -EOPNOTSUPP;
        goto bp_exit;
    }

    bpctl_cmd.out_param[0]= bp_dev_list[dev_idx].bus;
    bpctl_cmd.out_param[1]= bp_dev_list[dev_idx].slot;
    bpctl_cmd.out_param[2]= bp_dev_list[dev_idx].func;
    bpctl_cmd.out_param[3]= bp_dev_list[dev_idx].ifindex;

    

    if ((dev_idx<0)||(dev_idx>device_num)||(bp_dev_list[dev_idx].pdev==NULL)) {
        bpctl_cmd.status=-1;
        goto bpcmd_exit;
    }

    pbp_dev=&bp_dev_list[dev_idx];

    //trace("ioctl_num 0x%x\n,IOCTL_TX_MSG 0x%x\n",ioctl_num,IOCTL_TX_MSG(SET_BYPASS)); 

    switch (ioctl_num) {

    case IOCTL_TX_MSG(GET_BYPASS):
        bpctl_cmd.status= bp_get_bypass_fn(pbp_dev);
        break;

    case IOCTL_TX_MSG(SET_BYPASS):
        bpctl_cmd.status= bp_set_bypass_fn(pbp_dev, bpctl_cmd.in_param[2]);
        break;

    case IOCTL_TX_MSG(SET_BYPASS_PWOFF):
        bpctl_cmd.status = bp_set_bypass_at_power_off(pbp_dev, bpctl_cmd.in_param[2]);
        break;

    case IOCTL_TX_MSG(GET_BYPASS_PWOFF):
        bpctl_cmd.status = bp_get_bypass_at_power_off(pbp_dev);
        break;

    case IOCTL_TX_MSG(SET_BYPASS_PWUP):
        bpctl_cmd.status = bp_set_bypass_at_power_on(pbp_dev, bpctl_cmd.in_param[2]);
        break;

    case IOCTL_TX_MSG(GET_BYPASS_PWUP):
        bpctl_cmd.status = bp_get_bypass_at_power_on(pbp_dev);
        break;

    case IOCTL_TX_MSG(IS_BYPASS):
        bpctl_cmd.status= is_bypass_fn(pbp_dev);
        //trace("is bypass cmd status %d,dev_idx %d\n", bpctl_cmd.status, dev_idx);
        break;

    case IOCTL_TX_MSG(SET_STD_NIC):
        bpctl_cmd.status= bp_set_std_nic(pbp_dev, bpctl_cmd.in_param[2]);
        break;

    case IOCTL_TX_MSG(GET_STD_NIC):
        bpctl_cmd.status= bp_get_std_nic(pbp_dev);
        break;   

    case IOCTL_TX_MSG(GET_BYPASS_SLAVE):
        bpctl_cmd.status= bp_get_bypass_slave(pbp_dev, &pbp_dev_slave);
        if (bpctl_cmd.status==1) {
            bpctl_cmd.out_param[4]= pbp_dev_slave->bus;
            bpctl_cmd.out_param[5]= pbp_dev_slave->slot;
            bpctl_cmd.out_param[6]= pbp_dev_slave->func;
            bpctl_cmd.out_param[7]= pbp_dev_slave->ifindex;
        }
        break;

    case IOCTL_TX_MSG(SET_BYPASS_WD) :
        bpctl_cmd.status= bp_set_bypass_wd(pbp_dev, bpctl_cmd.in_param[2]);
        break;

    case IOCTL_TX_MSG(GET_BYPASS_WD) :
        bpctl_cmd.status= bp_get_bypass_wd(pbp_dev,(int *)&(bpctl_cmd.data[0]));
        break;   

    case IOCTL_TX_MSG(RESET_BYPASS_WD_TIMER) :
        bpctl_cmd.status= bp_reset_bypass_wd(pbp_dev);
        break;  

    case IOCTL_TX_MSG(SET_DISC) :
        bpctl_cmd.status=bp_set_disc(pbp_dev,bpctl_cmd.in_param[2]);
        break;
        
    case IOCTL_TX_MSG(GET_DISC) :
        bpctl_cmd.status=bp_get_disc(pbp_dev);
        break;      
 
    default:

        ret= -EOPNOTSUPP;
        goto bp_exit;
    }
    bpcmd_exit:
    if (copy_to_user(argp, (void *)&bpctl_cmd, sizeof(struct bpctl_cmd))) ret= -EFAULT;
    ret= SUCCESS;
    bp_exit:
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,30))
#endif
    unlock_bpctl();
    return ret;
}


struct file_operations Fops = {
    .owner = THIS_MODULE,
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,30)) 
    .ioctl = bp_device_ioctl,
#else
    .unlocked_ioctl = bp_device_ioctl,
#endif

    .open = device_open,
    .release = device_release, 
};

#ifndef PCI_DEVICE
    #define PCI_DEVICE(vend,dev) \
	.vendor = (vend), .device = (dev), \
	.subvendor = PCI_ANY_ID, .subdevice = PCI_ANY_ID
#endif


#define SILICOM_BPCTLBP_ETHERNET_DEVICE(device_id) {\
	PCI_DEVICE(SILICOM_VID, device_id)}



typedef struct _bpmod_info_t {
    unsigned int vendor;
    unsigned int device;
    unsigned int subvendor;
    unsigned int subdevice;
    unsigned int index;
    char *bp_name;

} bpmod_info_t;


typedef struct _dev_desc {
    char *name;
} dev_desc_t;



static void __exit bypass_cleanup_module(void)
{
    int i ;
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,23))      
    int ret;
#endif
 


    for (i = 0; i < device_num; i++) {
        iounmap ((void *)(bp_dev_list[i].mem_map));
    }

    if (bp_dev_list) {
		kfree (bp_dev_list);
	}



#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,23))
    ret = unregister_chrdev(major_num, DEVICE_NAME);

    if (ret < 0) {
		printk("Error in module_unregister_chrdev: %d\n", ret);
	}
#else
    unregister_chrdev(major_num, DEVICE_NAME);

#endif
}



static int bp_get_dev_number(void)
{
    int dev_num = 0;
    struct pci_dev* p_pci_dev = NULL;
    unsigned int vendor = LRBP_X710FP4_VENDOR;
    unsigned int device = LRBP_X710FP4_DEVICE;
    unsigned int ss_vendor = LRBP_X710FP4_SUBVENDOR;
    unsigned int ss_device = LRBP_X710FP4_SUBDEVICE;
    while ((p_pci_dev = pci_get_subsys(vendor, device, ss_vendor, ss_device, p_pci_dev))) {
        dev_num++;
    }
#ifdef SEARCH_SS_DEVICE_00  
    if(ss_device){  
        while ((p_pci_dev = pci_get_subsys(vendor, device, ss_vendor, 0, p_pci_dev))) {
            dev_num++;
        }
    }
#endif
    trace("!!! get_bp_dev_number %d\n", dev_num);
    return dev_num;
}



static int bp_get_net_device_bsf(struct net_device* dev, int* pbus, int* pslot, int* pfunc)
{
    struct ethtool_drvinfo drvinfo = { 0 };
    char* p;
    int bus, slot, func;

    if (dev->ethtool_ops && dev->ethtool_ops->get_drvinfo)
        dev->ethtool_ops->get_drvinfo(dev, &drvinfo);
    else
        return -1;

    if (!drvinfo.bus_info)
        return -1;

    if (!strcmp(drvinfo.bus_info, "N/A"))return -1;

    p = strchr(drvinfo.bus_info, ':');
    if (!p) return -1;

    p++;
    if (sscanf(p, "%x:%x.%x", &bus, &slot, &func) != 3)return -1;

    *pbus = bus;
    *pslot = slot;
    *pfunc = func;
    return 0;
}


static int bp_get_ifindex(bp_dev_t* bp_dev)
{
    int ifindex = 0;
    int bus, slot, func;
    struct net_device* dev;

    for_each_netdev(&init_net, dev)
    {
        if (bp_get_net_device_bsf(dev, &bus, &slot, &func) == 0)
        {
            if (bus == bp_dev->bus && slot == bp_dev->slot && func == bp_dev->func)
            {
                ifindex = dev->ifindex;
            }
        }
    }

    return ifindex;
}

static int bp_probe_dev(bp_dev_t* bp_dev_array, int dev_list_size)
{
    int dev_idx = 0;
    unsigned long mmio_start, mmio_len;
    struct pci_dev* p_pci_dev = NULL;
    unsigned int vendor = LRBP_X710FP4_VENDOR;
    unsigned int device = LRBP_X710FP4_DEVICE;
    unsigned int ss_vendor = LRBP_X710FP4_SUBVENDOR;
    unsigned int ss_device = LRBP_X710FP4_SUBDEVICE;
    while ((p_pci_dev = pci_get_subsys(vendor, device, ss_vendor, ss_device, p_pci_dev))) {
        mmio_start = pci_resource_start(p_pci_dev, 0);
        mmio_len = pci_resource_len(p_pci_dev, 0);
        if ((!mmio_len) || (!mmio_start))continue;
        bp_dev_array[dev_idx].mem_map = (unsigned long)ioremap(mmio_start, mmio_len);
        bp_dev_array[dev_idx].bus = p_pci_dev->bus->number;
        bp_dev_array[dev_idx].slot = PCI_SLOT(p_pci_dev->devfn);
        bp_dev_array[dev_idx].func = PCI_FUNC(p_pci_dev->devfn);
        bp_dev_array[dev_idx].vendor = vendor;
        bp_dev_array[dev_idx].device = device;
        bp_dev_array[dev_idx].subvendor = ss_vendor;
        bp_dev_array[dev_idx].subdevice = ss_device;
        bp_dev_array[dev_idx].ifindex = bp_get_ifindex(&bp_dev_array[dev_idx]);
        bp_dev_array[dev_idx].pdev = p_pci_dev;
        spin_lock_init(&bp_dev_array[dev_idx].bypass_wr_lock);
        trace("dev_idx %d,bus %d,slot %d,func %d,ifindex %d\n", dev_idx,bp_dev_array[dev_idx].bus, bp_dev_array[dev_idx].slot, bp_dev_array[dev_idx].func, bp_dev_array[dev_idx].ifindex);
        trace("mem_map 0x%x\n",bp_dev_array[dev_idx].mem_map);
        dev_idx++;
        if (dev_idx >= dev_list_size)break;
    }
#ifdef SEARCH_SS_DEVICE_00
    if(ss_device){
        while ((p_pci_dev = pci_get_subsys(vendor, device, ss_vendor, 0, p_pci_dev))) {
            mmio_start = pci_resource_start(p_pci_dev, 0);
            mmio_len = pci_resource_len(p_pci_dev, 0);
            if ((!mmio_len) || (!mmio_start))continue;
            bp_dev_array[dev_idx].mem_map = (unsigned long)ioremap(mmio_start, mmio_len);
            bp_dev_array[dev_idx].bus = p_pci_dev->bus->number;
            bp_dev_array[dev_idx].slot = PCI_SLOT(p_pci_dev->devfn);
            bp_dev_array[dev_idx].func = PCI_FUNC(p_pci_dev->devfn);
            bp_dev_array[dev_idx].vendor = vendor;
            bp_dev_array[dev_idx].device = device;
            bp_dev_array[dev_idx].subvendor = ss_vendor;
            bp_dev_array[dev_idx].subdevice = ss_device;
            bp_dev_array[dev_idx].ifindex = bp_get_ifindex(&bp_dev_array[dev_idx]);
            bp_dev_array[dev_idx].pdev = p_pci_dev;
            spin_lock_init(&bp_dev_array[dev_idx].bypass_wr_lock);
            trace("dev_idx %d,bus %d,slot %d,func %d,ifindex %d\n", dev_idx,bp_dev_array[dev_idx].bus, bp_dev_array[dev_idx].slot, bp_dev_array[dev_idx].func, bp_dev_array[dev_idx].ifindex);
            trace("mem_map 0x%x\n",bp_dev_array[dev_idx].mem_map);
            dev_idx++;
            if (dev_idx >= dev_list_size)break;
        }
    }
#endif

    trace("bp_probe_dev dev_idx %d\n",dev_idx);
    printk("bpctl_dev_num %d\n",dev_idx);
    return dev_idx;
}

int bp_is_master(bp_dev_t* pdev) 
{
    if (!pdev)return 0;

    if (pdev->func == 0 || pdev->func == 2)return 1;
    else return 0;
}


static int bypass_driver_init(void)
{
    int ret_val; //int idx, idx_dev = 0;
    int i;
    
    trace("LINUX_VERSION_CODE 0x%x\n", LINUX_VERSION_CODE);
#ifdef BP_SELF_TEST
    trace("BP_SELF_TEST\n");
#endif

    ret_val = register_chrdev (major_num, DEVICE_NAME, &Fops);
    if (ret_val < 0) {
        printk("%s failed with %d\n",DEVICE_NAME,ret_val);
        return ret_val;
    }
    major_num = ret_val; 

    device_num = bp_get_dev_number();
    if (!device_num) {
        printk("No such device\n"); 
        unregister_chrdev(major_num, DEVICE_NAME);
        return -1;
    }


    bp_dev_list = kmalloc((device_num) * sizeof(bp_dev_t), GFP_KERNEL);
    if (!bp_dev_list) {
        printk("Allocation bp_dev_list error\n");
        unregister_chrdev(major_num, DEVICE_NAME);
        return -1;
    }
    memset(bp_dev_list, 0, ((device_num) * sizeof(bp_dev_t)));


    bp_probe_dev(bp_dev_list, device_num);
    

    sema_init (&bpctl_sema, 1); 

    for(i=0;i<device_num;i++){
        if ((bp_dev_list[i].func == 0) || (bp_dev_list[i].func == 2)){
            ret_val = bp_read_reg(&bp_dev_list[i], STATUS_REG_ADDR);
        }
    }
    

    return 0;
}


static void __exit bypass_driver_exit(void)
{
    bypass_cleanup_module();
}

module_exit(bypass_driver_exit);
module_init(bypass_driver_init);













