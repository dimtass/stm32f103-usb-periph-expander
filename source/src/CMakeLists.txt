cmake_minimum_required(VERSION 2.8)

project(stm32f103_usb-peripheral-expander)

set(PP_DEFINES " -DSTM32F1 -DSTM32F103C8Tx -DSTM32 -DUSE_STDPERIPH_DRIVER -DSTM32F10X_MD")
set(PP_DEFINES "${PP_DEFINES} -DIVECT_TAB_OFFSET=0x0")


include_directories(
    ${CMSIS_SOURCE_DIR}/core
    ${CMSIS_SOURCE_DIR}/device
    ${StdPeriph_Driver_SOURCE_DIR}/inc
    ${STM32_USB-FS-Device_Driver_SOURCE_DIR}/inc
    inc
)

# uncomment/ comment as required
file(GLOB STM_STD_PERIPH_LIB_SRC
    ${StdPeriph_Driver_SOURCE_DIR}/src/misc.c
    ${StdPeriph_Driver_SOURCE_DIR}/src/stm32f10x_adc.c
    ${StdPeriph_Driver_SOURCE_DIR}/src/stm32f10x_bkp.c
#    ${StdPeriph_Driver_SOURCE_DIR}/src/stm32f10x_can.c
#    ${StdPeriph_Driver_SOURCE_DIR}/src/stm32f10x_cec.c
#    ${StdPeriph_Driver_SOURCE_DIR}/src/stm32f10x_crc.c
#    ${StdPeriph_Driver_SOURCE_DIR}/src/stm32f10x_dac.c
#    ${StdPeriph_Driver_SOURCE_DIR}/src/stm32f10x_dbgmcu.c
    ${StdPeriph_Driver_SOURCE_DIR}/src/stm32f10x_dma.c
    ${StdPeriph_Driver_SOURCE_DIR}/src/stm32f10x_exti.c
    ${StdPeriph_Driver_SOURCE_DIR}/src/stm32f10x_flash.c
#    ${StdPeriph_Driver_SOURCE_DIR}/src/stm32f10x_fsmc.c
    ${StdPeriph_Driver_SOURCE_DIR}/src/stm32f10x_gpio.c
    ${StdPeriph_Driver_SOURCE_DIR}/src/stm32f10x_i2c.c
    ${StdPeriph_Driver_SOURCE_DIR}/src/stm32f10x_iwdg.c
    ${StdPeriph_Driver_SOURCE_DIR}/src/stm32f10x_pwr.c
    ${StdPeriph_Driver_SOURCE_DIR}/src/stm32f10x_rcc.c
#    ${StdPeriph_Driver_SOURCE_DIR}/src/stm32f10x_rtc.c
#    ${StdPeriph_Driver_SOURCE_DIR}/src/stm32f10x_sdio.c
    ${StdPeriph_Driver_SOURCE_DIR}/src/stm32f10x_spi.c
#    ${StdPeriph_Driver_SOURCE_DIR}/src/stm32f10x_tim.c
    ${StdPeriph_Driver_SOURCE_DIR}/src/stm32f10x_usart.c
#    ${StdPeriph_Driver_SOURCE_DIR}/src/stm32f10x_wwdg.c
)

file(GLOB STM32_USB_DEVICE_SRC   
    ${STM32_USB-FS-Device_Driver_SOURCE_DIR}/src/usb_core.c
    ${STM32_USB-FS-Device_Driver_SOURCE_DIR}/src/usb_init.c
    ${STM32_USB-FS-Device_Driver_SOURCE_DIR}/src/usb_int.c
    ${STM32_USB-FS-Device_Driver_SOURCE_DIR}/src/usb_mem.c
    ${STM32_USB-FS-Device_Driver_SOURCE_DIR}/src/usb_regs.c
    ${STM32_USB-FS-Device_Driver_SOURCE_DIR}/src/usb_sil.c
)

file(GLOB C_SOURCE
    syscalls.c
#    cmd_parser.inc
    common_tools.c
    comm_protocol.c
    dev_adc.c
    dev_gpio.c
    dev_i2c.c
    dev_led.c
    dev_spi.c
    dev_uart.c
    dev_usb_uart.c
    hw_config.c
    main.c
    platform_devices.c
    stm32f10x_it.c
    system_stm32f10x.c
    tiny_printf.c
    usb_desc.c
    usb_endp.c
    usb_istr.c
    usb_prop.c
    usb_pwr.c
    
    ${STM_STD_PERIPH_LIB_SRC}
    ${STM32_USB_DEVICE_SRC}
)

set_source_files_properties(${C_SOURCE}
    PROPERTIES COMPILE_FLAGS ${PP_DEFINES}
)
    
add_executable(${PROJECT_NAME}.elf
    ${STARTUP_ASM_FILE}
    ${C_SOURCE}
)

target_link_libraries(${PROJECT_NAME}.elf)

add_custom_target ( ${PROJECT_NAME}.hex ALL
    DEPENDS ${PROJECT_NAME}.elf 
    COMMAND ${CMAKE_OBJCOPY} -O ihex ${PROJECT_NAME}.elf ${PROJECT_NAME}.hex
    COMMENT "Generating ${PROJECT_NAME}.hex"
)
add_custom_target ( ${PROJECT_NAME}.bin ALL
    DEPENDS ${PROJECT_NAME}.elf 
    COMMAND ${CMAKE_OBJCOPY} -O binary ${PROJECT_NAME}.elf ${PROJECT_NAME}.bin
    COMMENT "Generating ${PROJECT_NAME}.bin"
)

add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
    COMMAND ${CMAKE_SIZE} --format=berkeley "${PROJECT_NAME}.elf")
