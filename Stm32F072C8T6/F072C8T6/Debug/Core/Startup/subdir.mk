################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_UPPER_SRCS += \
../Core/Startup/startup_stm32f072c8tx.S 

OBJS += \
./Core/Startup/startup_stm32f072c8tx.o 

S_UPPER_DEPS += \
./Core/Startup/startup_stm32f072c8tx.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Startup/startup_stm32f072c8tx.o: ../Core/Startup/startup_stm32f072c8tx.S
	arm-none-eabi-gcc -mcpu=cortex-m0 -g3 -c -x assembler-with-cpp -MMD -MP -MF"Core/Startup/startup_stm32f072c8tx.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@" "$<"

