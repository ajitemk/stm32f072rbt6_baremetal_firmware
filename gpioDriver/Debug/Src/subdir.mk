################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/buttonInterrupt.c \
../Src/syscalls.c \
../Src/sysmem.c 

OBJS += \
./Src/buttonInterrupt.o \
./Src/syscalls.o \
./Src/sysmem.o 

C_DEPS += \
./Src/buttonInterrupt.d \
./Src/syscalls.d \
./Src/sysmem.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o Src/%.su: ../Src/%.c Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=gnu11 -g3 -DDEBUG -DSTM32 -DSTM32F0 -DSTM32F072RBTx -c -I../Inc -I"E:/Ajitem Kudtarkar/STM32/Ajitem_Projects/STM32_Projects/STM32F072RBT6/gpioDriver/Drivers/driver_INC" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Src

clean-Src:
	-$(RM) ./Src/buttonInterrupt.d ./Src/buttonInterrupt.o ./Src/buttonInterrupt.su ./Src/syscalls.d ./Src/syscalls.o ./Src/syscalls.su ./Src/sysmem.d ./Src/sysmem.o ./Src/sysmem.su

.PHONY: clean-Src

