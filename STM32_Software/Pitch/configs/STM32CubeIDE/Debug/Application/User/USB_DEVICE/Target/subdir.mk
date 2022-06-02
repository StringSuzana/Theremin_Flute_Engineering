################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
C:/Users/Suz/STM32/configs/USB_DEVICE/Target/usbd_conf.c 

OBJS += \
./Application/User/USB_DEVICE/Target/usbd_conf.o 

C_DEPS += \
./Application/User/USB_DEVICE/Target/usbd_conf.d 


# Each subdirectory must supply rules for building sources it contributes
Application/User/USB_DEVICE/Target/usbd_conf.o: C:/Users/Suz/STM32/configs/USB_DEVICE/Target/usbd_conf.c Application/User/USB_DEVICE/Target/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../../Core/Inc -I../../Drivers/STM32F1xx_HAL_Driver/Inc -I../../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../../Drivers/CMSIS/Include -I../../USB_DEVICE/App -I../../USB_DEVICE/Target -I../../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Application-2f-User-2f-USB_DEVICE-2f-Target

clean-Application-2f-User-2f-USB_DEVICE-2f-Target:
	-$(RM) ./Application/User/USB_DEVICE/Target/usbd_conf.d ./Application/User/USB_DEVICE/Target/usbd_conf.o ./Application/User/USB_DEVICE/Target/usbd_conf.su

.PHONY: clean-Application-2f-User-2f-USB_DEVICE-2f-Target

