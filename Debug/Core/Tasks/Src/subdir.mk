################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Tasks/Src/buzzing_task.c \
../Core/Tasks/Src/can_msg_processor.c \
../Core/Tasks/Src/control_input_task.c \
../Core/Tasks/Src/control_keyboard.c \
../Core/Tasks/Src/control_remote.c \
../Core/Tasks/Src/gimbal_control_task.c \
../Core/Tasks/Src/imu_processing_task.c \
../Core/Tasks/Src/launcher_control_task.c \
../Core/Tasks/Src/motor_config.c \
../Core/Tasks/Src/motor_control.c \
../Core/Tasks/Src/movement_control_task.c \
../Core/Tasks/Src/referee_processing_task.c \
../Core/Tasks/Src/startup_task.c 

OBJS += \
./Core/Tasks/Src/buzzing_task.o \
./Core/Tasks/Src/can_msg_processor.o \
./Core/Tasks/Src/control_input_task.o \
./Core/Tasks/Src/control_keyboard.o \
./Core/Tasks/Src/control_remote.o \
./Core/Tasks/Src/gimbal_control_task.o \
./Core/Tasks/Src/imu_processing_task.o \
./Core/Tasks/Src/launcher_control_task.o \
./Core/Tasks/Src/motor_config.o \
./Core/Tasks/Src/motor_control.o \
./Core/Tasks/Src/movement_control_task.o \
./Core/Tasks/Src/referee_processing_task.o \
./Core/Tasks/Src/startup_task.o 

C_DEPS += \
./Core/Tasks/Src/buzzing_task.d \
./Core/Tasks/Src/can_msg_processor.d \
./Core/Tasks/Src/control_input_task.d \
./Core/Tasks/Src/control_keyboard.d \
./Core/Tasks/Src/control_remote.d \
./Core/Tasks/Src/gimbal_control_task.d \
./Core/Tasks/Src/imu_processing_task.d \
./Core/Tasks/Src/launcher_control_task.d \
./Core/Tasks/Src/motor_config.d \
./Core/Tasks/Src/motor_control.d \
./Core/Tasks/Src/movement_control_task.d \
./Core/Tasks/Src/referee_processing_task.d \
./Core/Tasks/Src/startup_task.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Tasks/Src/%.o Core/Tasks/Src/%.su Core/Tasks/Src/%.cyclo: ../Core/Tasks/Src/%.c Core/Tasks/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DDEBUG -DSTM32F407xx -c -I../Core/Inc -I../lib -I../Core/Tasks/Inc -I../Core/BSP/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -IC:/Users/wx/STM32Cube/Repository/STM32Cube_FW_F4_V1.25.2/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -IC:/Users/wx/STM32Cube/Repository/STM32Cube_FW_F4_V1.25.2/Drivers/STM32F4xx_HAL_Driver/Inc -IC:/Users/wx/STM32Cube/Repository/STM32Cube_FW_F4_V1.25.2/Drivers/CMSIS/Device/ST/STM32F4xx/Include -IC:/Users/wx/STM32Cube/Repository/STM32Cube_FW_F4_V1.25.2/Middlewares/Third_Party/FreeRTOS/Source/include -IC:/Users/wx/STM32Cube/Repository/STM32Cube_FW_F4_V1.25.2/Drivers/CMSIS/Include -IC:/Users/wx/STM32Cube/Repository/STM32Cube_FW_F4_V1.25.2/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -IC:/Users/wx/STM32Cube/Repository/STM32Cube_FW_F4_V1.25.2/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Tasks-2f-Src

clean-Core-2f-Tasks-2f-Src:
	-$(RM) ./Core/Tasks/Src/buzzing_task.cyclo ./Core/Tasks/Src/buzzing_task.d ./Core/Tasks/Src/buzzing_task.o ./Core/Tasks/Src/buzzing_task.su ./Core/Tasks/Src/can_msg_processor.cyclo ./Core/Tasks/Src/can_msg_processor.d ./Core/Tasks/Src/can_msg_processor.o ./Core/Tasks/Src/can_msg_processor.su ./Core/Tasks/Src/control_input_task.cyclo ./Core/Tasks/Src/control_input_task.d ./Core/Tasks/Src/control_input_task.o ./Core/Tasks/Src/control_input_task.su ./Core/Tasks/Src/control_keyboard.cyclo ./Core/Tasks/Src/control_keyboard.d ./Core/Tasks/Src/control_keyboard.o ./Core/Tasks/Src/control_keyboard.su ./Core/Tasks/Src/control_remote.cyclo ./Core/Tasks/Src/control_remote.d ./Core/Tasks/Src/control_remote.o ./Core/Tasks/Src/control_remote.su ./Core/Tasks/Src/gimbal_control_task.cyclo ./Core/Tasks/Src/gimbal_control_task.d ./Core/Tasks/Src/gimbal_control_task.o ./Core/Tasks/Src/gimbal_control_task.su ./Core/Tasks/Src/imu_processing_task.cyclo ./Core/Tasks/Src/imu_processing_task.d ./Core/Tasks/Src/imu_processing_task.o ./Core/Tasks/Src/imu_processing_task.su ./Core/Tasks/Src/launcher_control_task.cyclo ./Core/Tasks/Src/launcher_control_task.d ./Core/Tasks/Src/launcher_control_task.o ./Core/Tasks/Src/launcher_control_task.su ./Core/Tasks/Src/motor_config.cyclo ./Core/Tasks/Src/motor_config.d ./Core/Tasks/Src/motor_config.o ./Core/Tasks/Src/motor_config.su ./Core/Tasks/Src/motor_control.cyclo ./Core/Tasks/Src/motor_control.d ./Core/Tasks/Src/motor_control.o ./Core/Tasks/Src/motor_control.su ./Core/Tasks/Src/movement_control_task.cyclo ./Core/Tasks/Src/movement_control_task.d ./Core/Tasks/Src/movement_control_task.o ./Core/Tasks/Src/movement_control_task.su ./Core/Tasks/Src/referee_processing_task.cyclo ./Core/Tasks/Src/referee_processing_task.d ./Core/Tasks/Src/referee_processing_task.o ./Core/Tasks/Src/referee_processing_task.su ./Core/Tasks/Src/startup_task.cyclo ./Core/Tasks/Src/startup_task.d ./Core/Tasks/Src/startup_task.o ./Core/Tasks/Src/startup_task.su

.PHONY: clean-Core-2f-Tasks-2f-Src

