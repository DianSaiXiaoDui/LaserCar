################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/24l01.c \
../Core/Src/Angle_PID.c \
../Core/Src/CCD.c \
../Core/Src/CCR_PID.c \
../Core/Src/DC.c \
../Core/Src/Location_PID.c \
../Core/Src/Velocity_PID.c \
../Core/Src/Vision.c \
../Core/Src/adc.c \
../Core/Src/dwt_delay.c \
../Core/Src/gpio.c \
../Core/Src/main.c \
../Core/Src/servo.c \
../Core/Src/spi.c \
../Core/Src/stm32h7xx_hal_msp.c \
../Core/Src/stm32h7xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/tim.c \
../Core/Src/usart.c 

OBJS += \
./Core/Src/24l01.o \
./Core/Src/Angle_PID.o \
./Core/Src/CCD.o \
./Core/Src/CCR_PID.o \
./Core/Src/DC.o \
./Core/Src/Location_PID.o \
./Core/Src/Velocity_PID.o \
./Core/Src/Vision.o \
./Core/Src/adc.o \
./Core/Src/dwt_delay.o \
./Core/Src/gpio.o \
./Core/Src/main.o \
./Core/Src/servo.o \
./Core/Src/spi.o \
./Core/Src/stm32h7xx_hal_msp.o \
./Core/Src/stm32h7xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/tim.o \
./Core/Src/usart.o 

C_DEPS += \
./Core/Src/24l01.d \
./Core/Src/Angle_PID.d \
./Core/Src/CCD.d \
./Core/Src/CCR_PID.d \
./Core/Src/DC.d \
./Core/Src/Location_PID.d \
./Core/Src/Velocity_PID.d \
./Core/Src/Vision.d \
./Core/Src/adc.d \
./Core/Src/dwt_delay.d \
./Core/Src/gpio.d \
./Core/Src/main.d \
./Core/Src/servo.d \
./Core/Src/spi.d \
./Core/Src/stm32h7xx_hal_msp.d \
./Core/Src/stm32h7xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/tim.d \
./Core/Src/usart.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DCORE_CM7 -DUSE_HAL_DRIVER -DSTM32H745xx -c -I../Core/Inc -I../../Drivers/STM32H7xx_HAL_Driver/Inc -I../../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/24l01.cyclo ./Core/Src/24l01.d ./Core/Src/24l01.o ./Core/Src/24l01.su ./Core/Src/Angle_PID.cyclo ./Core/Src/Angle_PID.d ./Core/Src/Angle_PID.o ./Core/Src/Angle_PID.su ./Core/Src/CCD.cyclo ./Core/Src/CCD.d ./Core/Src/CCD.o ./Core/Src/CCD.su ./Core/Src/CCR_PID.cyclo ./Core/Src/CCR_PID.d ./Core/Src/CCR_PID.o ./Core/Src/CCR_PID.su ./Core/Src/DC.cyclo ./Core/Src/DC.d ./Core/Src/DC.o ./Core/Src/DC.su ./Core/Src/Location_PID.cyclo ./Core/Src/Location_PID.d ./Core/Src/Location_PID.o ./Core/Src/Location_PID.su ./Core/Src/Velocity_PID.cyclo ./Core/Src/Velocity_PID.d ./Core/Src/Velocity_PID.o ./Core/Src/Velocity_PID.su ./Core/Src/Vision.cyclo ./Core/Src/Vision.d ./Core/Src/Vision.o ./Core/Src/Vision.su ./Core/Src/adc.cyclo ./Core/Src/adc.d ./Core/Src/adc.o ./Core/Src/adc.su ./Core/Src/dwt_delay.cyclo ./Core/Src/dwt_delay.d ./Core/Src/dwt_delay.o ./Core/Src/dwt_delay.su ./Core/Src/gpio.cyclo ./Core/Src/gpio.d ./Core/Src/gpio.o ./Core/Src/gpio.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/servo.cyclo ./Core/Src/servo.d ./Core/Src/servo.o ./Core/Src/servo.su ./Core/Src/spi.cyclo ./Core/Src/spi.d ./Core/Src/spi.o ./Core/Src/spi.su ./Core/Src/stm32h7xx_hal_msp.cyclo ./Core/Src/stm32h7xx_hal_msp.d ./Core/Src/stm32h7xx_hal_msp.o ./Core/Src/stm32h7xx_hal_msp.su ./Core/Src/stm32h7xx_it.cyclo ./Core/Src/stm32h7xx_it.d ./Core/Src/stm32h7xx_it.o ./Core/Src/stm32h7xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/tim.cyclo ./Core/Src/tim.d ./Core/Src/tim.o ./Core/Src/tim.su ./Core/Src/usart.cyclo ./Core/Src/usart.d ./Core/Src/usart.o ./Core/Src/usart.su

.PHONY: clean-Core-2f-Src

