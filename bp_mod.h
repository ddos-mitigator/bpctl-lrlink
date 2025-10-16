
#ifndef BP_MOD_H
#define BP_MOD_H
#include "bits.h"

#ifndef KERNEL_VERSION
    #define KERNEL_VERSION(a,b,c) (((a) << 16) + ((b) << 8) + (c))
#endif


#if ( LINUX_VERSION_CODE < KERNEL_VERSION(2,6,8) )
    #define msleep(x)	do { if(in_interrupt()) { \
				/* Don't mdelay in interrupt context! */ \
	                	BUG(); \
			} else { \
				set_current_state(TASK_UNINTERRUPTIBLE); \
				schedule_timeout((x * HZ)/1000 + 2); \
			} } while(0)
#endif

#if (LINUX_VERSION_CODE > KERNEL_VERSION(2,6,10))
    #define EXPORT_SYMBOL_NOVERS EXPORT_SYMBOL
#endif



#ifndef jiffies_to_msecs
    #define jiffies_to_msecs(x) _kc_jiffies_to_msecs(x)
static inline unsigned int jiffies_to_msecs(const unsigned long j)
{
#if HZ <= 1000 && !(1000 % HZ)
    return(1000 / HZ) * j;
#elif HZ > 1000 && !(HZ % 1000)
    return(j + (HZ / 1000) - 1)/(HZ / 1000);
#else
    return(j * 1000) / HZ;
#endif
}
#endif

#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,10))

    #define pci_get_class pci_find_class
    #define pci_get_subsys pci_find_subsys
    #define pci_get_device pci_find_device

#endif

#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,4))
    #define MODULE_VERSION(_version)
#endif



#define BPCTLI_CTRL          0x00000  
#define BPCTLI_LEDCTL         0x00E00
#define BPCTLI_CTRL_SWDPIO0  0x00400000 
#define BPCTLI_CTRL_SWDPIN0  0x00040000


#define BPCTLI_CTRL_EXT 0x00018  
#define BPCTLI_STATUS   0x00008  

#define BPCTLI_EERD     0x14 
#define BPCTLI_EEWR     0x102c 

#define BPCTLI_CTRL_EXT_SDP6_DATA 0x00000040 
#define BPCTLI_CTRL_EXT_SDP7_DATA 0x00000080 
#define BPCTLI_CTRL_SDP0_DATA     0x00040000 
#define BPCTLI_CTRL_SDP1_DATA     0x00080000

#define BPCTLI_CTRL_EXT_SDP6_DIR  0x00000400 
#define BPCTLI_CTRL_EXT_SDP7_DIR  0x00000800 
#define BPCTLI_CTRL_SDP0_DIR      0x00400000 
#define BPCTLI_CTRL_SWDPIN0       0x00040000
#define BPCTLI_CTRL_SWDPIN1       0x00080000
#define BPCTLI_CTRL_SDP1_DIR      0x00800000


#define BPCTLI_CTRL_EXT_MCLK_DIR  BPCTLI_CTRL_EXT_SDP7_DIR
#define BPCTLI_CTRL_EXT_MCLK_DATA BPCTLI_CTRL_EXT_SDP7_DATA
#define BPCTLI_CTRL_EXT_MDIO_DIR  BPCTLI_CTRL_EXT_SDP6_DIR
#define BPCTLI_CTRL_EXT_MDIO_DATA BPCTLI_CTRL_EXT_SDP6_DATA

#define BPCTLI_CTRL_EXT_MCLK_DIR5  BPCTLI_CTRL_SDP1_DIR
#define BPCTLI_CTRL_EXT_MCLK_DATA5 BPCTLI_CTRL_SWDPIN1
#define BPCTLI_CTRL_EXT_MCLK_DIR80  BPCTLI_CTRL_EXT_SDP6_DIR
#define BPCTLI_CTRL_EXT_MCLK_DATA80 BPCTLI_CTRL_EXT_SDP6_DATA
#define BPCTLI_CTRL_EXT_MDIO_DIR5  BPCTLI_CTRL_SWDPIO0
#define BPCTLI_CTRL_EXT_MDIO_DATA5 BPCTLI_CTRL_SWDPIN0
#define BPCTLI_CTRL_EXT_MDIO_DIR80  BPCTLI_CTRL_SWDPIO0
#define BPCTLI_CTRL_EXT_MDIO_DATA80 BPCTLI_CTRL_SWDPIN0



#define BPCTL_WRITE_REG(a, reg, value) \
	(writel((value), (void *)(((a)->mem_map) + BPCTLI_##reg)))

#define BPCTL_READ_REG(a, reg) ( \
        readl((void *)((a)->mem_map) + BPCTLI_##reg))


#define BPCTL_WRITE_FLUSH(a) BPCTL_READ_REG(a, STATUS)

#define BPCTL_BP_WRITE_REG(a, reg, value) ({ \
        BPCTL_WRITE_REG(a, reg, value); \
        BPCTL_WRITE_FLUSH(a);})
        





#endif
