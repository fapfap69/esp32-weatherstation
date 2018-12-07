################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../main/WeatherStation.cpp \
../main/main.cpp 

OBJS += \
./main/WeatherStation.o \
./main/main.o 

CPP_DEPS += \
./main/WeatherStation.d \
./main/main.d 


# Each subdirectory must supply rules for building sources it contributes
main/%.o: ../main/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	xtensa-esp32-elf-g++ -I"/Users/fap/esp/esp-idf/components/freertos/include" -I"/Users/fap/esp/esp-idf/components/esp32/include" -I"/Users/fap/esp/esp-idf/components/spi_flash/include" -I"/Users/fap/esp/esp-idf/components/driver/include" -I"/Users/fap/esp/esp-idf/components/soc/esp32/include" -I../build/include -I"/Users/fap/esp/esp-idf/components/heap/include" -I"/Users/fap/esp/esp-idf/components/soc/include" -I"/Users/fap/esp/esp-idf/components/log/include" -I"/Users/fap/esp/esp-idf/components/mdns/include" -I"/Users/fap/esp/esp-idf/components/lwip/lwip/src/include/" -I"/Users/fap/esp/esp-idf/components/tcpip_adapter/include" -I"/Users/fap/esp/esp-idf/components/nvs_flash/include" -I"/Users/fap/esp/esp-idf/components/mqtt/esp-mqtt/include" -I"/Users/fap/esp/esp-idf/components/app_update/include" -I"/Users/fap/esp/esp-idf/components/esp_http_client/include" -I"/Users/fap/esp/esp-idf/components/bootloader_support/include" -I"/Users/fap/esp/esp-idf/components/esp_event/include" -I"/Users/fap/esp/esp-idf/components/bt/include" -I"/Users/fap/esp/esp-idf/components/ulp/include" -I../main/components -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


