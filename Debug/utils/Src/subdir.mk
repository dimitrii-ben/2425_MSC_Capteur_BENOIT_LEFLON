################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../utils/Src/functions.c \
../utils/Src/tools.c 

OBJS += \
./utils/Src/functions.o \
./utils/Src/tools.o 

C_DEPS += \
./utils/Src/functions.d \
./utils/Src/tools.d 


# Each subdirectory must supply rules for building sources it contributes
utils/Src/%.o utils/Src/%.su utils/Src/%.cyclo: ../utils/Src/%.c utils/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G431xx -c -I../Core/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-utils-2f-Src

clean-utils-2f-Src:
	-$(RM) ./utils/Src/functions.cyclo ./utils/Src/functions.d ./utils/Src/functions.o ./utils/Src/functions.su ./utils/Src/tools.cyclo ./utils/Src/tools.d ./utils/Src/tools.o ./utils/Src/tools.su

.PHONY: clean-utils-2f-Src

