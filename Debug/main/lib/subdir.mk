################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../main/lib/counter.cpp 

OBJS += \
./main/lib/counter.o 

CPP_DEPS += \
./main/lib/counter.d 


# Each subdirectory must supply rules for building sources it contributes
main/lib/%.o: ../main/lib/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	xtensa-esp32-elf-g++ -I"/Users/fap/esp/esp-idf/components/freertos/include" -I"/Users/fap/esp/esp-idf/components/esp32/include" -I"/Users/fap/esp/esp-idf/components/spi_flash/include" -I"/Users/fap/esp/esp-idf/components/driver/include" -I"/Users/fap/esp/esp-idf/components/soc/esp32/include" -I../build/include -I"/Users/fap/esp/esp-idf/components/heap/include" -I"/Users/fap/esp/esp-idf/components/soc/include" -I"/Users/fap/esp/esp-idf/components/log/include" -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


