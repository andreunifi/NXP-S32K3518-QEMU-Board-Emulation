# group9


Clone with git clone --recurse-submodules https://baltig.polito.it/eos2024/group9.git
Build QEMU and run firmware using the provided script ./build_and_run.sh after having given it execute permissions (chmod +x ./build_and_run.sh)

## Added and modified QEMU files
### MCU and BOARD
- qemu/hw/arm/Kconfig
- qemu/hw/arm/meson.build
- qemu/hw/arm/s32k3x8evb.c
- qemu/hw/arm/s32k3x8evb_mcu.c
- qemu/include/hw/arm/s32k3x8evb.h
- qemu/include/hw/arm/s32k3x8evb_mcu.h

### UART
- qemu/hw/char/meson.build
- qemu/hw/char/nxps32k3x8evb_uart.c
- qemu/include/hw/char/nxps32k3x8evb_uart.h
- qemu/hw/char/trace-events

### ADC
- qemu/hw/adc/meson.build
- qemu/hw/adc/nxps32k3x8evb_adc.c
- qemu/include/hw/adc/nxps32k3x8evb_adc.h
