################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/BSP/Components/ls016b8uy/ls016b8uy.c 

OBJS += \
./Drivers/BSP/Components/ls016b8uy/ls016b8uy.o 

C_DEPS += \
./Drivers/BSP/Components/ls016b8uy/ls016b8uy.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/BSP/Components/ls016b8uy/%.o Drivers/BSP/Components/ls016b8uy/%.su Drivers/BSP/Components/ls016b8uy/%.cyclo: ../Drivers/BSP/Components/ls016b8uy/%.c Drivers/BSP/Components/ls016b8uy/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F429xx -c -I"C:/Users/User/STM32CubeIDE/workspace_1.17.0/ESP411_P1/Drivers/BSP/STM32F429I-Discovery" -I"C:/Users/User/STM32CubeIDE/workspace_1.17.0/ESP411_P1/Drivers/BSP/Components" -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -u _printf_float -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-BSP-2f-Components-2f-ls016b8uy

clean-Drivers-2f-BSP-2f-Components-2f-ls016b8uy:
	-$(RM) ./Drivers/BSP/Components/ls016b8uy/ls016b8uy.cyclo ./Drivers/BSP/Components/ls016b8uy/ls016b8uy.d ./Drivers/BSP/Components/ls016b8uy/ls016b8uy.o ./Drivers/BSP/Components/ls016b8uy/ls016b8uy.su

.PHONY: clean-Drivers-2f-BSP-2f-Components-2f-ls016b8uy

