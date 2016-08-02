# bootloader
This is the bootloader for my project mini-sys

* This bootloader can use the first 16k flash pages starting from 0x8000000
* Offset address for the application is then 0x8004000

If not already done in application setup do:

* edit linker script STM32F103C8Tx_FLASH.ld. Change flash origin and length to:
FLASH (rx)      : ORIGIN = 0x8004000, LENGTH = 112K

* edit Drivers/CMSIS/Device/ST/STM32F1xx/Source/Templates/system_stm32f1xx.c. Change VECT_TAB_OFFSET to
VECT_TAB_OFFSET 0x4000


The loader checks if GPIOA, GPIO_PIN_8 is low while power on to enter DFU mode. Else it jumps to 0x8004000

In DFU mode one can do:

Upload: dfu-util -a 0 -s 0x08004000 -U app.bin
Download:  dfu-util -a 0 -s 0x08004000 -D app.bin

 
  