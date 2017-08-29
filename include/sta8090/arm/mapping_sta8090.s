; _____________________________________________________________________________
;| FILE,         mapping_sta8089.s
;| PROJECT,      CRM Teseo 3 - STA8089
;|_____________________________________________________________________________
;| DESCRIPTION,  Memory map
;|_____________________________________________________________________________
;| COPYRIGHT,    (c) 2009 STMicroelectronics, Agrate Brianza (MI) (ITALY)
;|
;| HISTORY,
;| Date        | Modification               | Author
;|_____________________________________________________________________________
;| 2012.01.03  | Initial revision           | FB
;|_____________________________________________________________________________

;--------------------------------------------------------------
; AHB address mapping
;--------------------------------------------------------------

ITCM_START_ADDR                 EQU   0x00000000    ; I-TCM (always at 0)
DTCM_START_ADDR                 EQU   0x00100000    ; D-TCM (logic addressing)
SQI_START_ADDR                  EQU   0x10000000    ; SQI
FSMC_BANK0_START_ADDR           EQU   0x20000000    ; FSMC Bank 0
FSMC_BANK1_START_ADDR           EQU   0x24000000    ; FSMC Bank 1
FSMC_BANK2_START_ADDR           EQU   0x28000000    ; FSMC Bank 2
ROM_START_ADDR                  EQU   0x30000000    ; ROM
BACKUP_RAM_START_ADDR           EQU   0x40000000    ; Backup RAM
VIC_REG_START_ADDR              EQU   0x50000000    ; VIC Controller
SQI_REG_START_ADDR              EQU   0x50010000    ; SQI Controller
FSMC_REG_START_ADDR             EQU   0x50020000    ; FSMC Controller
USB_OTG_FS_START_ADDR           EQU   0x60000000    ; USB-OTG Full Speed
G3EP_REG_START_ADDR             EQU   0x70000000    ; G3 Controller

;--------------------------------------------------------------
; APB peripheral address mapping
;--------------------------------------------------------------
; APB Bridge 1
PRCC_BCK_START_ADDR             EQU   0x51000000    ; Power, Reset and Clock Controller on backup
RTC_RTT_PWL_REG_START_ADDR      EQU   0x51001000    ; RTC and RTT registers

; APB Bridge 2

WDT_REG_START_ADDR              EQU   0x52000000    ; Watchdog Timer
EFT_REG_START_ADDR              EQU   0x52001000    ; Extended Function Timer
SD_SDIO_MMC_REG_START_ADDR      EQU   0x52002000    ; Secure Digital Card, SD IO, Multi-Media Card
SSP_REG_START_ADDR              EQU   0x52003000    ; Synchronous Serial Port
MTU0_REG_START_ADDR             EQU   0x52004000    ; Multi Timer Unit
ADC_REG_START_ADDR              EQU   0x52005000    ; Analog/digital converter
MSP_REG_START_ADDR              EQU   0x52006000    ; Multi Serial Port port
I2C_REG_START_ADDR              EQU   0x52007000    ; Inter-Integrated Circuit
GPIO0_REG_START_ADDR            EQU   0x52008000    ; General Purpose I/Os Bank 0
GPIO1_REG_START_ADDR            EQU   0x52009000    ; General Purpose I/Os Bank 1
UART0_REG_START_ADDR            EQU   0x5200A000    ; Asynchronous Serial Port 0
UART1_REG_START_ADDR            EQU   0x5200B000    ; Asynchronous Serial Port 1
UART2_REG_START_ADDR            EQU   0x5200C000    ; Asynchronous Serial Port 2
PRCC_REG_START_ADDR             EQU   0x5200F000    ; Power, Reset and Clock Controller
MTU1_REG_START_ADDR             EQU   0x52014000    ; Multi Timer Unit

; APB Bridge 3

CAN0_REG_START_ADDR             EQU   0x5200d000    ; Controller Area Network 0
CAN1_REG_START_ADDR             EQU   0x5200e000    ; Controller Area Network 1

;--------------------------------------------------------------
; STA8088 TCM specific constants
;--------------------------------------------------------------

ITCM_SIZE_MIN                   EQU   0x00004000    ; Minimum ITCM size
DTCM_SIZE_MIN                   EQU   0x00020000    ; Minimum DTCM size
TCM_BLOCK_SIZE                  EQU   0x00004000    ; TCM block size

; End of file - mapping_sta8090.h

        END
