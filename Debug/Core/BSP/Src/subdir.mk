################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/BSP/Src/CRC8_CRC16.c \
../Core/BSP/Src/bsp_buzzer.c \
../Core/BSP/Src/bsp_can.c \
../Core/BSP/Src/bsp_dbus_input.c \
../Core/BSP/Src/bsp_gpio.c \
../Core/BSP/Src/bsp_imu.c \
../Core/BSP/Src/bsp_led.c \
../Core/BSP/Src/bsp_micros_timer.c \
../Core/BSP/Src/bsp_oled.c \
../Core/BSP/Src/bsp_queue.c \
../Core/BSP/Src/bsp_referee.c \
../Core/BSP/Src/bsp_usart.c \
../Core/BSP/Src/bsp_usb_redir.c 

OBJS += \
./Core/BSP/Src/CRC8_CRC16.o \
./Core/BSP/Src/bsp_buzzer.o \
./Core/BSP/Src/bsp_can.o \
./Core/BSP/Src/bsp_dbus_input.o \
./Core/BSP/Src/bsp_gpio.o \
./Core/BSP/Src/bsp_imu.o \
./Core/BSP/Src/bsp_led.o \
./Core/BSP/Src/bsp_micros_timer.o \
./Core/BSP/Src/bsp_oled.o \
./Core/BSP/Src/bsp_queue.o \
./Core/BSP/Src/bsp_referee.o \
./Core/BSP/Src/bsp_usart.o \
./Core/BSP/Src/bsp_usb_redir.o 

C_DEPS += \
./Core/BSP/Src/CRC8_CRC16.d \
./Core/BSP/Src/bsp_buzzer.d \
./Core/BSP/Src/bsp_can.d \
./Core/BSP/Src/bsp_dbus_input.d \
./Core/BSP/Src/bsp_gpio.d \
./Core/BSP/Src/bsp_imu.d \
./Core/BSP/Src/bsp_led.d \
./Core/BSP/Src/bsp_micros_timer.d \
./Core/BSP/Src/bsp_oled.d \
./Core/BSP/Src/bsp_queue.d \
./Core/BSP/Src/bsp_referee.d \
./Core/BSP/Src/bsp_usart.d \
./Core/BSP/Src/bsp_usb_redir.d 


# Each subdirectory must supply rules for building sources it contributes
Core/BSP/Src/%.o Core/BSP/Src/%.su Core/BSP/Src/%.cyclo: ../Core/BSP/Src/%.c Core/BSP/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DDEBUG -DSTM32F407xx -c -I../Core/Inc -I../lib -I../Core/Tasks/Inc -I../Core/BSP/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -IC:/Users/wx/STM32Cube/Repository/STM32Cube_FW_F4_V1.25.2/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -IC:/Users/wx/STM32Cube/Repository/STM32Cube_FW_F4_V1.25.2/Drivers/STM32F4xx_HAL_Driver/Inc -IC:/Users/wx/STM32Cube/Repository/STM32Cube_FW_F4_V1.25.2/Drivers/CMSIS/Device/ST/STM32F4xx/Include -IC:/Users/wx/STM32Cube/Repository/STM32Cube_FW_F4_V1.25.2/Middlewares/Third_Party/FreeRTOS/Source/include -IC:/Users/wx/STM32Cube/Repository/STM32Cube_FW_F4_V1.25.2/Drivers/CMSIS/Include -IC:/Users/wx/STM32Cube/Repository/STM32Cube_FW_F4_V1.25.2/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -IC:/Users/wx/STM32Cube/Repository/STM32Cube_FW_F4_V1.25.2/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-BSP-2f-Src

clean-Core-2f-BSP-2f-Src:
	-$(RM) ./Core/BSP/Src/CRC8_CRC16.cyclo ./Core/BSP/Src/CRC8_CRC16.d ./Core/BSP/Src/CRC8_CRC16.o ./Core/BSP/Src/CRC8_CRC16.su ./Core/BSP/Src/bsp_buzzer.cyclo ./Core/BSP/Src/bsp_buzzer.d ./Core/BSP/Src/bsp_buzzer.o ./Core/BSP/Src/bsp_buzzer.su ./Core/BSP/Src/bsp_can.cyclo ./Core/BSP/Src/bsp_can.d ./Core/BSP/Src/bsp_can.o ./Core/BSP/Src/bsp_can.su ./Core/BSP/Src/bsp_dbus_input.cyclo ./Core/BSP/Src/bsp_dbus_input.d ./Core/BSP/Src/bsp_dbus_input.o ./Core/BSP/Src/bsp_dbus_input.su ./Core/BSP/Src/bsp_gpio.cyclo ./Core/BSP/Src/bsp_gpio.d ./Core/BSP/Src/bsp_gpio.o ./Core/BSP/Src/bsp_gpio.su ./Core/BSP/Src/bsp_imu.cyclo ./Core/BSP/Src/bsp_imu.d ./Core/BSP/Src/bsp_imu.o ./Core/BSP/Src/bsp_imu.su ./Core/BSP/Src/bsp_led.cyclo ./Core/BSP/Src/bsp_led.d ./Core/BSP/Src/bsp_led.o ./Core/BSP/Src/bsp_led.su ./Core/BSP/Src/bsp_micros_timer.cyclo ./Core/BSP/Src/bsp_micros_timer.d ./Core/BSP/Src/bsp_micros_timer.o ./Core/BSP/Src/bsp_micros_timer.su ./Core/BSP/Src/bsp_oled.cyclo ./Core/BSP/Src/bsp_oled.d ./Core/BSP/Src/bsp_oled.o ./Core/BSP/Src/bsp_oled.su ./Core/BSP/Src/bsp_queue.cyclo ./Core/BSP/Src/bsp_queue.d ./Core/BSP/Src/bsp_queue.o ./Core/BSP/Src/bsp_queue.su ./Core/BSP/Src/bsp_referee.cyclo ./Core/BSP/Src/bsp_referee.d ./Core/BSP/Src/bsp_referee.o ./Core/BSP/Src/bsp_referee.su ./Core/BSP/Src/bsp_usart.cyclo ./Core/BSP/Src/bsp_usart.d ./Core/BSP/Src/bsp_usart.o ./Core/BSP/Src/bsp_usart.su ./Core/BSP/Src/bsp_usb_redir.cyclo ./Core/BSP/Src/bsp_usb_redir.d ./Core/BSP/Src/bsp_usb_redir.o ./Core/BSP/Src/bsp_usb_redir.su

.PHONY: clean-Core-2f-BSP-2f-Src

