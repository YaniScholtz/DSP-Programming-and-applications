################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/BSP/STM32F401-Discovery/stm32f401_discovery.c \
../Drivers/BSP/STM32F401-Discovery/stm32f401_discovery_accelerometer.c \
../Drivers/BSP/STM32F401-Discovery/stm32f401_discovery_audio.c \
../Drivers/BSP/STM32F401-Discovery/stm32f401_discovery_gyroscope.c 

OBJS += \
./Drivers/BSP/STM32F401-Discovery/stm32f401_discovery.o \
./Drivers/BSP/STM32F401-Discovery/stm32f401_discovery_accelerometer.o \
./Drivers/BSP/STM32F401-Discovery/stm32f401_discovery_audio.o \
./Drivers/BSP/STM32F401-Discovery/stm32f401_discovery_gyroscope.o 

C_DEPS += \
./Drivers/BSP/STM32F401-Discovery/stm32f401_discovery.d \
./Drivers/BSP/STM32F401-Discovery/stm32f401_discovery_accelerometer.d \
./Drivers/BSP/STM32F401-Discovery/stm32f401_discovery_audio.d \
./Drivers/BSP/STM32F401-Discovery/stm32f401_discovery_gyroscope.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/BSP/STM32F401-Discovery/%.o Drivers/BSP/STM32F401-Discovery/%.su Drivers/BSP/STM32F401-Discovery/%.cyclo: ../Drivers/BSP/STM32F401-Discovery/%.c Drivers/BSP/STM32F401-Discovery/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F429xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-BSP-2f-STM32F401-2d-Discovery

clean-Drivers-2f-BSP-2f-STM32F401-2d-Discovery:
	-$(RM) ./Drivers/BSP/STM32F401-Discovery/stm32f401_discovery.cyclo ./Drivers/BSP/STM32F401-Discovery/stm32f401_discovery.d ./Drivers/BSP/STM32F401-Discovery/stm32f401_discovery.o ./Drivers/BSP/STM32F401-Discovery/stm32f401_discovery.su ./Drivers/BSP/STM32F401-Discovery/stm32f401_discovery_accelerometer.cyclo ./Drivers/BSP/STM32F401-Discovery/stm32f401_discovery_accelerometer.d ./Drivers/BSP/STM32F401-Discovery/stm32f401_discovery_accelerometer.o ./Drivers/BSP/STM32F401-Discovery/stm32f401_discovery_accelerometer.su ./Drivers/BSP/STM32F401-Discovery/stm32f401_discovery_audio.cyclo ./Drivers/BSP/STM32F401-Discovery/stm32f401_discovery_audio.d ./Drivers/BSP/STM32F401-Discovery/stm32f401_discovery_audio.o ./Drivers/BSP/STM32F401-Discovery/stm32f401_discovery_audio.su ./Drivers/BSP/STM32F401-Discovery/stm32f401_discovery_gyroscope.cyclo ./Drivers/BSP/STM32F401-Discovery/stm32f401_discovery_gyroscope.d ./Drivers/BSP/STM32F401-Discovery/stm32f401_discovery_gyroscope.o ./Drivers/BSP/STM32F401-Discovery/stm32f401_discovery_gyroscope.su

.PHONY: clean-Drivers-2f-BSP-2f-STM32F401-2d-Discovery

