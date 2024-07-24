################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/driver_SRC/i2c_Driver.c \
../Drivers/driver_SRC/stm32f072rbt6_GPIO_Driver.c \
../Drivers/driver_SRC/usart_driver.c 

OBJS += \
./Drivers/driver_SRC/i2c_Driver.o \
./Drivers/driver_SRC/stm32f072rbt6_GPIO_Driver.o \
./Drivers/driver_SRC/usart_driver.o 

C_DEPS += \
./Drivers/driver_SRC/i2c_Driver.d \
./Drivers/driver_SRC/stm32f072rbt6_GPIO_Driver.d \
./Drivers/driver_SRC/usart_driver.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/driver_SRC/%.o Drivers/driver_SRC/%.su: ../Drivers/driver_SRC/%.c Drivers/driver_SRC/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=gnu11 -g3 -DDEBUG -DSTM32 -DSTM32F0 -DSTM32F072RBTx -c -I../Inc -I"E:/Ajitem Kudtarkar/STM32/Ajitem_Projects/STM32_Projects/STM32F072RBT6/gpioDriver/Drivers/driver_INC" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Drivers-2f-driver_SRC

clean-Drivers-2f-driver_SRC:
	-$(RM) ./Drivers/driver_SRC/i2c_Driver.d ./Drivers/driver_SRC/i2c_Driver.o ./Drivers/driver_SRC/i2c_Driver.su ./Drivers/driver_SRC/stm32f072rbt6_GPIO_Driver.d ./Drivers/driver_SRC/stm32f072rbt6_GPIO_Driver.o ./Drivers/driver_SRC/stm32f072rbt6_GPIO_Driver.su ./Drivers/driver_SRC/usart_driver.d ./Drivers/driver_SRC/usart_driver.o ./Drivers/driver_SRC/usart_driver.su

.PHONY: clean-Drivers-2f-driver_SRC

