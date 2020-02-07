################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Startup/subdir.mk
-include Src/subdir.mk
-include Drivers/STM32F4xx_HAL_Driver/Src/subdir.mk
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
mouse2020.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
mouse2020.list \

OBJCOPY_HEX += \
mouse2020.hex \

OBJCOPY_BIN += \
mouse2020.bin \


# All Target
all: mouse2020.elf secondary-outputs

# Tool invocations
mouse2020.elf: $(OBJS) $(USER_OBJS) C:\Users\Hatanaka\STM32CubeIDE\workspace_1.0.1\mouse2020\STM32F405RGTX_FLASH.ld
	arm-none-eabi-gcc -o "mouse2020.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"C:\Users\Hatanaka\STM32CubeIDE\workspace_1.0.1\mouse2020\STM32F405RGTX_FLASH.ld" --specs=nosys.specs -Wl,-Map="mouse2020.map" -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

mouse2020.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "mouse2020.list"
	@echo 'Finished building: $@'
	@echo ' '

mouse2020.hex: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O ihex $(EXECUTABLES) "mouse2020.hex"
	@echo 'Finished building: $@'
	@echo ' '

mouse2020.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "mouse2020.bin"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) *
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_HEX) $(OBJCOPY_BIN)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
