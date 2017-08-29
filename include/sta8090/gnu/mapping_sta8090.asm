@ _____________________________________________________________________________
@| FILE,         mapping_sta8088.s
@| PROJECT,      CRM Teseo 2 - STA8088
@|_____________________________________________________________________________
@| DESCRIPTION,  Memory map
@|_____________________________________________________________________________
@| COPYRIGHT,    (c) 2009 STMicroelectronics, Agrate Brianza (MI) (ITALY)
@|
@| HISTORY,
@| Date        | Modification               | Author
@|_____________________________________________________________________________
@| 2009.06.26  | Initial revision           | FB
@|_____________________________________________________________________________

@--------------------------------------------------------------
@ AHB address mapping
@--------------------------------------------------------------

.equ  ITCM_START_ADDR                 ,   0x00000000    @ I-TCM (always at 0)
.equ  DTCM_START_ADDR                 ,   0x00100000    @ D-TCM (logic addressing)
.equ  SQI_START_ADDR                  ,   0x10000000    @ SQI
.equ  FSMC_BANK0_START_ADDR           ,   0x20000000    @ FSMC Bank 0
.equ  FSMC_BANK1_START_ADDR           ,   0x24000000    @ FSMC Bank 1
.equ  FSMC_BANK2_START_ADDR           ,   0x28000000    @ FSMC Bank 2
.equ  ROM_START_ADDR                  ,   0x30000000    @ ROM
.equ  BACKUP_RAM_START_ADDR           ,   0x40000000    @ Backup RAM
.equ  VIC_REG_START_ADDR              ,   0x50000000    @ VIC Controller
.equ  SQI_REG_START_ADDR              ,   0x50010000    @ SQI Controller
.equ  FSMC_REG_START_ADDR             ,   0x50020000    @ FSMC Controller
.equ  USB_OTG_FS_START_ADDR           ,   0x60000000    @ USB-OTG Full Speed
.equ  G3_REG_START_ADDR               ,   0x70000000    @ G3 Controller

@--------------------------------------------------------------
@ APB peripheral address mapping
@--------------------------------------------------------------
@ APB Bridge 1
.equ  PRCC_BCK_START_ADDR             ,   0x51000000    @ Power, Reset and Clock Controller on backup
.equ  RTC_RTT_PWL_REG_START_ADDR      ,   0x51001000    @ RTC and RTT registers

@ APB Bridge 2

.equ  WDT_REG_START_ADDR              ,   0x52000000    @ Watchdog Timer
.equ  EFT_REG_START_ADDR              ,   0x52001000    @ Extended Function Timer
.equ  SD_SDIO_MMC_REG_START_ADDR      ,   0x52002000    @ Secure Digital Card, SD IO, Multi-Media Card
.equ  SSP_REG_START_ADDR              ,   0x52003000    @ Synchronous Serial Port
.equ  MTU_REG_START_ADDR              ,   0x52004000    @ Multi Timer Unit
.equ  ADC_REG_START_ADDR              ,   0x52005000    @ Analog/digital converter
.equ  MSP_REG_START_ADDR              ,   0x52006000    @ Multi Serial Port port
.equ  I2C_REG_START_ADDR              ,   0x52007000    @ Inter-Integrated Circuit
.equ  GPIO0_REG_START_ADDR            ,   0x52008000    @ General Purpose I/Os Bank 0
.equ  GPIO1_REG_START_ADDR            ,   0x52009000    @ General Purpose I/Os Bank 1
.equ  UART0_REG_START_ADDR            ,   0x5200A000    @ Asynchronous Serial Port 0
.equ  UART1_REG_START_ADDR            ,   0x5200B000    @ Asynchronous Serial Port 1
.equ  UART2_REG_START_ADDR            ,   0x5200C000    @ Asynchronous Serial Port 2
.equ  PRCC_REG_START_ADDR             ,   0x5200F000    @ Power, Reset and Clock Controller
.equ  MTU1_REG_START_ADDR             ,   0x52014000    @ Multi Timer Unit

@ APB Bridge 3

.equ  CAN0_REG_START_ADDR             ,   0x5200d000    @ Controller Area Network 0
.equ  CAN1_REG_START_ADDR             ,   0x5200e000    @ Controller Area Network 1

@--------------------------------------------------------------
@ STA8088 TCM specific constants
@--------------------------------------------------------------

.equ ITCM_SIZE_MIN                    ,   0x00004000    @ Minimum ITCM size
.equ DTCM_SIZE_MIN                    ,   0x00020000    @ Minimum DTCM size
.equ TCM_BLOCK_SIZE                   ,   0x00004000    @ TCM block size

@ End of file - mapping_sta8088.h

