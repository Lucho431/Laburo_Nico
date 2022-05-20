################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Drivers/STM32F1xx_HAL_Driver/Src/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
CajaDeCarga_STM32F103C8T6.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
CajaDeCarga_STM32F103C8T6.list \


# All Target
all: CajaDeCarga_STM32F103C8T6.elf secondary-outputs

# Tool invocations
CajaDeCarga_STM32F103C8T6.elf: $(OBJS) $(USER_OBJS) /home/nicolas/STM32CubeIDE/workspace_1.0.0/CajaDeCarga_STM32F103C8T6/STM32F103C8TX_FLASH.ld
	arm-none-eabi-gcc -o "CajaDeCarga_STM32F103C8T6.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m3 -T"/home/nicolas/STM32CubeIDE/workspace_1.0.0/CajaDeCarga_STM32F103C8T6/STM32F103C8TX_FLASH.ld" --specs=nosys.specs -Wl,-Map="CajaDeCarga_STM32F103C8T6.map" -Wl,--gc-sections -static --specs=nano.specs -mfloat-abi=soft -mthumb -u _printf_float -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

CajaDeCarga_STM32F103C8T6.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "CajaDeCarga_STM32F103C8T6.list"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) *
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
