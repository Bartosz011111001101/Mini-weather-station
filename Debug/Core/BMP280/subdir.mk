################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/BMP280/bme280.c 

OBJS += \
./Core/BMP280/bme280.o 

C_DEPS += \
./Core/BMP280/bme280.d 


# Each subdirectory must supply rules for building sources it contributes
Core/BMP280/%.o Core/BMP280/%.su Core/BMP280/%.cyclo: ../Core/BMP280/%.c Core/BMP280/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G474xx -c -I../Core/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -I../FATFS/Target -I../FATFS/App -I../Middlewares/Third_Party/FatFs/src -I../Core/BMP280 -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-BMP280

clean-Core-2f-BMP280:
	-$(RM) ./Core/BMP280/bme280.cyclo ./Core/BMP280/bme280.d ./Core/BMP280/bme280.o ./Core/BMP280/bme280.su

.PHONY: clean-Core-2f-BMP280

