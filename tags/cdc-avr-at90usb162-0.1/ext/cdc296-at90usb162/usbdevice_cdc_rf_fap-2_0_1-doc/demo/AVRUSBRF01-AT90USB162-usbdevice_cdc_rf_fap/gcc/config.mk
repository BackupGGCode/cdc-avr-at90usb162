
# Project name
PROJECT = AVRUSBRF01-AT90USB162-usbdevice_cdc_rf_fap

# CPU architecture : {avr0|...|avr6}
# Parts : {at90usb646|at90usb647|at90usb1286|at90usb1287|at90usb162|at90usb82}
MCU = at90usb162

# Source files
CSRCS = \
  ../../../lib_board/nrf24l01/nRF_API.c\
  ../cdc_rf_task.c\
  ../main.c\
  ../../../lib_mcu/power/power_drv.c\
  ../../../modules/scheduler/scheduler.c\
  ../../../lib_mcu/spi/spi_lib.c\
  ../usb_descriptors.c\
  ../../../modules/usb/device_chap9/usb_device_task.c\
  ../../../lib_mcu/usb/usb_drv.c\
  ../../../modules/usb/device_chap9/usb_standard_request.c\
  ../usb_specific_request.c\
  ../../../modules/usb/usb_task.c\
  ../fap.c\
  ../FAPtoNRF_API.c\
# Assembler source files
ASSRCS = \
  ../../../lib_mcu/flash/flash_drv.s\

