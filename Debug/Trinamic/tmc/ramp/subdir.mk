################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Trinamic/tmc/ramp/LinearRamp.c \
../Trinamic/tmc/ramp/LinearRamp1.c \
../Trinamic/tmc/ramp/Ramp.c 

OBJS += \
./Trinamic/tmc/ramp/LinearRamp.o \
./Trinamic/tmc/ramp/LinearRamp1.o \
./Trinamic/tmc/ramp/Ramp.o 

C_DEPS += \
./Trinamic/tmc/ramp/LinearRamp.d \
./Trinamic/tmc/ramp/LinearRamp1.d \
./Trinamic/tmc/ramp/Ramp.d 


# Each subdirectory must supply rules for building sources it contributes
Trinamic/tmc/ramp/%.o Trinamic/tmc/ramp/%.su: ../Trinamic/tmc/ramp/%.c Trinamic/tmc/ramp/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../TMC2130 -I../Trinamic/tmc -I../Trinamic/tmc/ramp -I../Trinamic/tmc/helpers -I../Trinamic/tmc/ic/TMC2130 -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Trinamic-2f-tmc-2f-ramp

clean-Trinamic-2f-tmc-2f-ramp:
	-$(RM) ./Trinamic/tmc/ramp/LinearRamp.d ./Trinamic/tmc/ramp/LinearRamp.o ./Trinamic/tmc/ramp/LinearRamp.su ./Trinamic/tmc/ramp/LinearRamp1.d ./Trinamic/tmc/ramp/LinearRamp1.o ./Trinamic/tmc/ramp/LinearRamp1.su ./Trinamic/tmc/ramp/Ramp.d ./Trinamic/tmc/ramp/Ramp.o ./Trinamic/tmc/ramp/Ramp.su

.PHONY: clean-Trinamic-2f-tmc-2f-ramp

