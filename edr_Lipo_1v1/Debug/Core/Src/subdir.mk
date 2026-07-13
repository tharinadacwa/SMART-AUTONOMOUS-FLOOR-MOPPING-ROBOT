################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/adc.c \
../Core/Src/adc_interface.c \
../Core/Src/app_freertos.c \
../Core/Src/battery.c \
../Core/Src/bq25703a_regulator.c \
../Core/Src/dma.c \
../Core/Src/error.c \
../Core/Src/gpio.c \
../Core/Src/i2c.c \
../Core/Src/main.c \
../Core/Src/printf.c \
../Core/Src/stm32g0xx_hal_msp.c \
../Core/Src/stm32g0xx_hal_timebase_tim.c \
../Core/Src/stm32g0xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32g0xx.c \
../Core/Src/tim.c \
../Core/Src/usart.c \
../Core/Src/usbpd.c \
../Core/Src/usbpd_dpm_core.c \
../Core/Src/usbpd_dpm_user.c \
../Core/Src/usbpd_pwr_if.c \
../Core/Src/usbpd_pwr_user.c \
../Core/Src/usbpd_vdm_user.c 

OBJS += \
./Core/Src/adc.o \
./Core/Src/adc_interface.o \
./Core/Src/app_freertos.o \
./Core/Src/battery.o \
./Core/Src/bq25703a_regulator.o \
./Core/Src/dma.o \
./Core/Src/error.o \
./Core/Src/gpio.o \
./Core/Src/i2c.o \
./Core/Src/main.o \
./Core/Src/printf.o \
./Core/Src/stm32g0xx_hal_msp.o \
./Core/Src/stm32g0xx_hal_timebase_tim.o \
./Core/Src/stm32g0xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32g0xx.o \
./Core/Src/tim.o \
./Core/Src/usart.o \
./Core/Src/usbpd.o \
./Core/Src/usbpd_dpm_core.o \
./Core/Src/usbpd_dpm_user.o \
./Core/Src/usbpd_pwr_if.o \
./Core/Src/usbpd_pwr_user.o \
./Core/Src/usbpd_vdm_user.o 

C_DEPS += \
./Core/Src/adc.d \
./Core/Src/adc_interface.d \
./Core/Src/app_freertos.d \
./Core/Src/battery.d \
./Core/Src/bq25703a_regulator.d \
./Core/Src/dma.d \
./Core/Src/error.d \
./Core/Src/gpio.d \
./Core/Src/i2c.d \
./Core/Src/main.d \
./Core/Src/printf.d \
./Core/Src/stm32g0xx_hal_msp.d \
./Core/Src/stm32g0xx_hal_timebase_tim.d \
./Core/Src/stm32g0xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32g0xx.d \
./Core/Src/tim.d \
./Core/Src/usart.d \
./Core/Src/usbpd.d \
./Core/Src/usbpd_dpm_core.d \
./Core/Src/usbpd_dpm_user.d \
./Core/Src/usbpd_pwr_if.d \
./Core/Src/usbpd_pwr_user.d \
./Core/Src/usbpd_vdm_user.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G071xx -DUSE_FULL_LL_DRIVER -DUSBPD_PORT_COUNT=1 -DUSBPDCORE_LIB_PD3_FULL -D_RTOS -D_SNK -c -I../Core/Inc -I../Middlewares/Third_Party/FreeRTOS_CLI/Source/include -I../Utilities/GUI_INTERFACE -I../Utilities/TRACER_EMB -I../Drivers/STM32G0xx_HAL_Driver/Inc -I../Drivers/STM32G0xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G0xx/Include -I../Drivers/CMSIS/Include -I../USBPD/App -I../USBPD -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM0 -I../Middlewares/ST/STM32_USBPD_Library/Core/inc -I../Middlewares/ST/STM32_USBPD_Library/Devices/STM32G0XX/inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/adc.cyclo ./Core/Src/adc.d ./Core/Src/adc.o ./Core/Src/adc.su ./Core/Src/adc_interface.cyclo ./Core/Src/adc_interface.d ./Core/Src/adc_interface.o ./Core/Src/adc_interface.su ./Core/Src/app_freertos.cyclo ./Core/Src/app_freertos.d ./Core/Src/app_freertos.o ./Core/Src/app_freertos.su ./Core/Src/battery.cyclo ./Core/Src/battery.d ./Core/Src/battery.o ./Core/Src/battery.su ./Core/Src/bq25703a_regulator.cyclo ./Core/Src/bq25703a_regulator.d ./Core/Src/bq25703a_regulator.o ./Core/Src/bq25703a_regulator.su ./Core/Src/dma.cyclo ./Core/Src/dma.d ./Core/Src/dma.o ./Core/Src/dma.su ./Core/Src/error.cyclo ./Core/Src/error.d ./Core/Src/error.o ./Core/Src/error.su ./Core/Src/gpio.cyclo ./Core/Src/gpio.d ./Core/Src/gpio.o ./Core/Src/gpio.su ./Core/Src/i2c.cyclo ./Core/Src/i2c.d ./Core/Src/i2c.o ./Core/Src/i2c.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/printf.cyclo ./Core/Src/printf.d ./Core/Src/printf.o ./Core/Src/printf.su ./Core/Src/stm32g0xx_hal_msp.cyclo ./Core/Src/stm32g0xx_hal_msp.d ./Core/Src/stm32g0xx_hal_msp.o ./Core/Src/stm32g0xx_hal_msp.su ./Core/Src/stm32g0xx_hal_timebase_tim.cyclo ./Core/Src/stm32g0xx_hal_timebase_tim.d ./Core/Src/stm32g0xx_hal_timebase_tim.o ./Core/Src/stm32g0xx_hal_timebase_tim.su ./Core/Src/stm32g0xx_it.cyclo ./Core/Src/stm32g0xx_it.d ./Core/Src/stm32g0xx_it.o ./Core/Src/stm32g0xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32g0xx.cyclo ./Core/Src/system_stm32g0xx.d ./Core/Src/system_stm32g0xx.o ./Core/Src/system_stm32g0xx.su ./Core/Src/tim.cyclo ./Core/Src/tim.d ./Core/Src/tim.o ./Core/Src/tim.su ./Core/Src/usart.cyclo ./Core/Src/usart.d ./Core/Src/usart.o ./Core/Src/usart.su ./Core/Src/usbpd.cyclo ./Core/Src/usbpd.d ./Core/Src/usbpd.o ./Core/Src/usbpd.su ./Core/Src/usbpd_dpm_core.cyclo ./Core/Src/usbpd_dpm_core.d ./Core/Src/usbpd_dpm_core.o ./Core/Src/usbpd_dpm_core.su ./Core/Src/usbpd_dpm_user.cyclo ./Core/Src/usbpd_dpm_user.d ./Core/Src/usbpd_dpm_user.o ./Core/Src/usbpd_dpm_user.su ./Core/Src/usbpd_pwr_if.cyclo ./Core/Src/usbpd_pwr_if.d ./Core/Src/usbpd_pwr_if.o ./Core/Src/usbpd_pwr_if.su ./Core/Src/usbpd_pwr_user.cyclo ./Core/Src/usbpd_pwr_user.d ./Core/Src/usbpd_pwr_user.o ./Core/Src/usbpd_pwr_user.su ./Core/Src/usbpd_vdm_user.cyclo ./Core/Src/usbpd_vdm_user.d ./Core/Src/usbpd_vdm_user.o ./Core/Src/usbpd_vdm_user.su

.PHONY: clean-Core-2f-Src

