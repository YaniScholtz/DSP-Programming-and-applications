################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/BSP/STM324x9I_EVAL/stm324x9i_eval.c \
../Drivers/BSP/STM324x9I_EVAL/stm324x9i_eval_audio.c \
../Drivers/BSP/STM324x9I_EVAL/stm324x9i_eval_camera.c \
../Drivers/BSP/STM324x9I_EVAL/stm324x9i_eval_eeprom.c \
../Drivers/BSP/STM324x9I_EVAL/stm324x9i_eval_io.c \
../Drivers/BSP/STM324x9I_EVAL/stm324x9i_eval_lcd.c \
../Drivers/BSP/STM324x9I_EVAL/stm324x9i_eval_nor.c \
../Drivers/BSP/STM324x9I_EVAL/stm324x9i_eval_sd.c \
../Drivers/BSP/STM324x9I_EVAL/stm324x9i_eval_sdram.c \
../Drivers/BSP/STM324x9I_EVAL/stm324x9i_eval_sram.c \
../Drivers/BSP/STM324x9I_EVAL/stm324x9i_eval_ts.c 

OBJS += \
./Drivers/BSP/STM324x9I_EVAL/stm324x9i_eval.o \
./Drivers/BSP/STM324x9I_EVAL/stm324x9i_eval_audio.o \
./Drivers/BSP/STM324x9I_EVAL/stm324x9i_eval_camera.o \
./Drivers/BSP/STM324x9I_EVAL/stm324x9i_eval_eeprom.o \
./Drivers/BSP/STM324x9I_EVAL/stm324x9i_eval_io.o \
./Drivers/BSP/STM324x9I_EVAL/stm324x9i_eval_lcd.o \
./Drivers/BSP/STM324x9I_EVAL/stm324x9i_eval_nor.o \
./Drivers/BSP/STM324x9I_EVAL/stm324x9i_eval_sd.o \
./Drivers/BSP/STM324x9I_EVAL/stm324x9i_eval_sdram.o \
./Drivers/BSP/STM324x9I_EVAL/stm324x9i_eval_sram.o \
./Drivers/BSP/STM324x9I_EVAL/stm324x9i_eval_ts.o 

C_DEPS += \
./Drivers/BSP/STM324x9I_EVAL/stm324x9i_eval.d \
./Drivers/BSP/STM324x9I_EVAL/stm324x9i_eval_audio.d \
./Drivers/BSP/STM324x9I_EVAL/stm324x9i_eval_camera.d \
./Drivers/BSP/STM324x9I_EVAL/stm324x9i_eval_eeprom.d \
./Drivers/BSP/STM324x9I_EVAL/stm324x9i_eval_io.d \
./Drivers/BSP/STM324x9I_EVAL/stm324x9i_eval_lcd.d \
./Drivers/BSP/STM324x9I_EVAL/stm324x9i_eval_nor.d \
./Drivers/BSP/STM324x9I_EVAL/stm324x9i_eval_sd.d \
./Drivers/BSP/STM324x9I_EVAL/stm324x9i_eval_sdram.d \
./Drivers/BSP/STM324x9I_EVAL/stm324x9i_eval_sram.d \
./Drivers/BSP/STM324x9I_EVAL/stm324x9i_eval_ts.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/BSP/STM324x9I_EVAL/%.o Drivers/BSP/STM324x9I_EVAL/%.su Drivers/BSP/STM324x9I_EVAL/%.cyclo: ../Drivers/BSP/STM324x9I_EVAL/%.c Drivers/BSP/STM324x9I_EVAL/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F429xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-BSP-2f-STM324x9I_EVAL

clean-Drivers-2f-BSP-2f-STM324x9I_EVAL:
	-$(RM) ./Drivers/BSP/STM324x9I_EVAL/stm324x9i_eval.cyclo ./Drivers/BSP/STM324x9I_EVAL/stm324x9i_eval.d ./Drivers/BSP/STM324x9I_EVAL/stm324x9i_eval.o ./Drivers/BSP/STM324x9I_EVAL/stm324x9i_eval.su ./Drivers/BSP/STM324x9I_EVAL/stm324x9i_eval_audio.cyclo ./Drivers/BSP/STM324x9I_EVAL/stm324x9i_eval_audio.d ./Drivers/BSP/STM324x9I_EVAL/stm324x9i_eval_audio.o ./Drivers/BSP/STM324x9I_EVAL/stm324x9i_eval_audio.su ./Drivers/BSP/STM324x9I_EVAL/stm324x9i_eval_camera.cyclo ./Drivers/BSP/STM324x9I_EVAL/stm324x9i_eval_camera.d ./Drivers/BSP/STM324x9I_EVAL/stm324x9i_eval_camera.o ./Drivers/BSP/STM324x9I_EVAL/stm324x9i_eval_camera.su ./Drivers/BSP/STM324x9I_EVAL/stm324x9i_eval_eeprom.cyclo ./Drivers/BSP/STM324x9I_EVAL/stm324x9i_eval_eeprom.d ./Drivers/BSP/STM324x9I_EVAL/stm324x9i_eval_eeprom.o ./Drivers/BSP/STM324x9I_EVAL/stm324x9i_eval_eeprom.su ./Drivers/BSP/STM324x9I_EVAL/stm324x9i_eval_io.cyclo ./Drivers/BSP/STM324x9I_EVAL/stm324x9i_eval_io.d ./Drivers/BSP/STM324x9I_EVAL/stm324x9i_eval_io.o ./Drivers/BSP/STM324x9I_EVAL/stm324x9i_eval_io.su ./Drivers/BSP/STM324x9I_EVAL/stm324x9i_eval_lcd.cyclo ./Drivers/BSP/STM324x9I_EVAL/stm324x9i_eval_lcd.d ./Drivers/BSP/STM324x9I_EVAL/stm324x9i_eval_lcd.o ./Drivers/BSP/STM324x9I_EVAL/stm324x9i_eval_lcd.su ./Drivers/BSP/STM324x9I_EVAL/stm324x9i_eval_nor.cyclo ./Drivers/BSP/STM324x9I_EVAL/stm324x9i_eval_nor.d ./Drivers/BSP/STM324x9I_EVAL/stm324x9i_eval_nor.o ./Drivers/BSP/STM324x9I_EVAL/stm324x9i_eval_nor.su ./Drivers/BSP/STM324x9I_EVAL/stm324x9i_eval_sd.cyclo ./Drivers/BSP/STM324x9I_EVAL/stm324x9i_eval_sd.d ./Drivers/BSP/STM324x9I_EVAL/stm324x9i_eval_sd.o ./Drivers/BSP/STM324x9I_EVAL/stm324x9i_eval_sd.su ./Drivers/BSP/STM324x9I_EVAL/stm324x9i_eval_sdram.cyclo ./Drivers/BSP/STM324x9I_EVAL/stm324x9i_eval_sdram.d ./Drivers/BSP/STM324x9I_EVAL/stm324x9i_eval_sdram.o ./Drivers/BSP/STM324x9I_EVAL/stm324x9i_eval_sdram.su ./Drivers/BSP/STM324x9I_EVAL/stm324x9i_eval_sram.cyclo ./Drivers/BSP/STM324x9I_EVAL/stm324x9i_eval_sram.d ./Drivers/BSP/STM324x9I_EVAL/stm324x9i_eval_sram.o ./Drivers/BSP/STM324x9I_EVAL/stm324x9i_eval_sram.su ./Drivers/BSP/STM324x9I_EVAL/stm324x9i_eval_ts.cyclo ./Drivers/BSP/STM324x9I_EVAL/stm324x9i_eval_ts.d ./Drivers/BSP/STM324x9I_EVAL/stm324x9i_eval_ts.o ./Drivers/BSP/STM324x9I_EVAL/stm324x9i_eval_ts.su

.PHONY: clean-Drivers-2f-BSP-2f-STM324x9I_EVAL

