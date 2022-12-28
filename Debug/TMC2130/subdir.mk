################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../TMC2130/tmc2130.c \
../TMC2130/tmc2130_io.c \
../TMC2130/tmc2130_step_generator.c 

OBJS += \
./TMC2130/tmc2130.o \
./TMC2130/tmc2130_io.o \
./TMC2130/tmc2130_step_generator.o 

C_DEPS += \
./TMC2130/tmc2130.d \
./TMC2130/tmc2130_io.d \
./TMC2130/tmc2130_step_generator.d 


# Each subdirectory must supply rules for building sources it contributes
TMC2130/%.o TMC2130/%.su: ../TMC2130/%.c TMC2130/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../TMC2130 -I../Trinamic/tmc -I../Trinamic/tmc/ramp -I../Trinamic/tmc/helpers -I../Trinamic/tmc/ic/TMC2130 -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-TMC2130

clean-TMC2130:
	-$(RM) ./TMC2130/tmc2130.d ./TMC2130/tmc2130.o ./TMC2130/tmc2130.su ./TMC2130/tmc2130_io.d ./TMC2130/tmc2130_io.o ./TMC2130/tmc2130_io.su ./TMC2130/tmc2130_step_generator.d ./TMC2130/tmc2130_step_generator.o ./TMC2130/tmc2130_step_generator.su

.PHONY: clean-TMC2130

