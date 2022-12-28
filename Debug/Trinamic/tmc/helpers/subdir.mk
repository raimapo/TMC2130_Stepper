################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Trinamic/tmc/helpers/CRC.c \
../Trinamic/tmc/helpers/Functions.c 

OBJS += \
./Trinamic/tmc/helpers/CRC.o \
./Trinamic/tmc/helpers/Functions.o 

C_DEPS += \
./Trinamic/tmc/helpers/CRC.d \
./Trinamic/tmc/helpers/Functions.d 


# Each subdirectory must supply rules for building sources it contributes
Trinamic/tmc/helpers/%.o Trinamic/tmc/helpers/%.su: ../Trinamic/tmc/helpers/%.c Trinamic/tmc/helpers/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../TMC2130 -I../Trinamic/tmc -I../Trinamic/tmc/ramp -I../Trinamic/tmc/helpers -I../Trinamic/tmc/ic/TMC2130 -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Trinamic-2f-tmc-2f-helpers

clean-Trinamic-2f-tmc-2f-helpers:
	-$(RM) ./Trinamic/tmc/helpers/CRC.d ./Trinamic/tmc/helpers/CRC.o ./Trinamic/tmc/helpers/CRC.su ./Trinamic/tmc/helpers/Functions.d ./Trinamic/tmc/helpers/Functions.o ./Trinamic/tmc/helpers/Functions.su

.PHONY: clean-Trinamic-2f-tmc-2f-helpers

