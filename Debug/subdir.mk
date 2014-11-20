################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../mavlink_serial.cpp \
../serial_port.cpp 

OBJS += \
./mavlink_serial.o \
./serial_port.o 

CPP_DEPS += \
./mavlink_serial.d \
./serial_port.d 


# Each subdirectory must supply rules for building sources it contributes
%.o: ../%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	g++ -I"/media/trent/bigdata/Dropbox/Research-cloud/Projects/PX4/Examples/c_uart_interface_example/mavlink" -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


