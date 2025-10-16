
#ifndef BYPASS_H
#define BYPASS_H


#define SYNC_CMD_VAL               2     
#define SYNC_CMD_LEN               2

#define WR_CMD_VAL                 2      
#define WR_CMD_LEN                 2 

#define RD_CMD_VAL                 1     
#define RD_CMD_LEN                 2 

#define ADDR_CMD_LEN               4 

#define WR_DATA_LEN                8
#define RD_DATA_LEN                8


#define PIC_SIGN_REG_ADDR          0x7
#define PIC_SIGN_VALUE         0xcd


#define STATUS_REG_ADDR           0
#define WDT_EN_MASK            0x01    
#define CMND_EN_MASK           0x02    
#define DIS_BYPASS_CAP_MASK    0x04    
#define DFLT_PWRON_MASK        0x08   
#define BYPASS_OFF_MASK        0x10    
#define BYPASS_FLAG_MASK       0x20    
#define STD_NIC_MASK           (DIS_BYPASS_CAP_MASK | BYPASS_OFF_MASK | DFLT_PWRON_MASK)
#define WD_EXP_FLAG_MASK       0x40    
#define DFLT_PWROFF_MASK       0x80   
#define STD_NIC_PWOFF_MASK     (DIS_BYPASS_CAP_MASK | BYPASS_OFF_MASK | DFLT_PWRON_MASK | DFLT_PWROFF_MASK)


#define STATUS_DISC_REG_ADDR    13
    #define WDTE_DISC_BPN_MASK      0x01    
    #define STD_NIC_ON_MASK         0x02    
    #define DIS_DISC_CAP_MASK       0x04    
    #define DFLT_PWRON_DISC_MASK    0x08   
    #define DISC_OFF_MASK           0x10    
    #define DISC_FLAG_MASK          0x20    
    #define TPL2_FLAG_MASK          0x40   
    #define STD_NIC_DISC_MASK       DIS_DISC_CAP_MASK
    
#define CONT_CONFIG_REG_ADDR    12
    #define EN_HW_RESET_MASK       0x2  
    #define WAIT_AT_PWUP_MASK      0x1  
    



#define CMND_REG_ADDR              10    
#define WDT_REG_ADDR               4
#define TMRL_REG_ADDR              2
#define TMRH_REG_ADDR              3

#define WDT_INTERVAL               5 
#define WDT_CMND_INTERVAL          200 
#define CMND_INTERVAL              200 
#define PULSE_TIME                 100  

#define INIT_CMND_INTERVAL         40
#define PULSE_INTERVAL             5
#define WDT_TIME_CNT               3


#define CMND_OFF_INT               0xf
#define PWROFF_BYPASS_ON_INT       0x5
#define BYPASS_ON_INT              0x6
#define DIS_BYPASS_CAP_INT         0x4
#define RESET_WDT_INT              0x1


#define BYPASS_DELAY_INT           4     
#define CMND_INTERVAL_INT          2     
             
#define CMND_ON                    0x4
#define CMND_OFF                   0x2 
#define BYPASS_ON                  0xa
#define BYPASS_OFF                 0x8
#define PORT_LINK_EN               0xe
#define PORT_LINK_DIS              0xc
#define WDT_ON                     0x10                
#define TIMEOUT_UNIT           100
#define TIMEOUT_MAX_STEP       15
#define WDT_TIMEOUT_MIN        100                 
#define WDT_TIMEOUT_MAX        3276800              
#define WDT_AUTO_MIN_INT           500
#define WDT_TIMEOUT_DEF        WDT_TIMEOUT_MIN
#define WDT_OFF                    0x6
#define WDT_RELOAD                 0x9
#define RESET_CONT                 0x20
#define DIS_BYPASS_CAP             0x22
#define EN_BYPASS_CAP              0x24
#define BYPASS_STATE_PWRON         0x26
#define NORMAL_STATE_PWRON         0x28
#define BYPASS_STATE_PWROFF        0x27
#define NORMAL_STATE_PWROFF        0x29
#define TAP_ON                     0xb
#define TAP_OFF                    0x9
#define TAP_STATE_PWRON            0x2a
#define DIS_TAP_CAP                0x2c
#define EN_TAP_CAP                 0x2e
#define STD_NIC_OFF       0x86
#define STD_NIC_ON       0x84
#define DISC_ON           0x85    
#define DISC_OFF          0x8a    
#define DISC_STATE_PWRON  0x87    
#define DIS_DISC_CAP      0x88    
#define EN_DISC_CAP       0x89   
#define TPL2_ON                    0x8c
#define TPL2_OFF                   0x8b
#define BP_WAIT_AT_PWUP_EN        0x80    
#define BP_WAIT_AT_PWUP_DIS       0x81    
#define BP_HW_RESET_EN             0x82    
#define BP_HW_RESET_DIS            0x83 



#define TX_DISA_PWRUP          0xA2    
#define TX_DISB_PWRUP          0xA3    
#define TX_ENA_PWRUP           0xA4    
#define TX_ENB_PWRUP           0xA5    
 

#define BYPASS_CAP_DELAY           35    
#define DFLT_PWRON_DELAY           10    
#define LATCH_DELAY                15     
#define EEPROM_WR_DELAY            20    

#define BP_LINK_MON_DELAY          4 


#define BP_FW_EXT_VER0                 0xa0
#define BP_FW_EXT_VER1                 0xa1
#define BP_FW_EXT_VER2                0xb1 

#define BP_OK        0
#define BP_NOT_CAP  -1
#define WDT_STATUS_EXP -2
#define WDT_STATUS_UNKNOWN -1
#define WDT_STATUS_EN 1
#define WDT_STATUS_DIS 0


#define ETH_P_BPTEST 0xabba


#define BPTEST_DATA_LEN 60




#endif   


