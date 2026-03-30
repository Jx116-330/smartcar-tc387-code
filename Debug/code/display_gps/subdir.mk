################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
"../code/display_gps/display_gps.c" 

COMPILED_SRCS += \
"code/display_gps/display_gps.src" 

C_DEPS += \
"./code/display_gps/display_gps.d" 

OBJS += \
"code/display_gps/display_gps.o" 


# Each subdirectory must supply rules for building sources it contributes
"code/display_gps/display_gps.src":"../code/display_gps/display_gps.c" "code/display_gps/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2004 "-fE:/ads/Seekfree_TC387_Opensource_Library/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc38x -Y0 -N0 -Z0 -o "$@" "$<"
"code/display_gps/display_gps.o":"code/display_gps/display_gps.src" "code/display_gps/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"

clean: clean-code-2f-display_gps

clean-code-2f-display_gps:
	-$(RM) ./code/display_gps/display_gps.d ./code/display_gps/display_gps.o ./code/display_gps/display_gps.src

.PHONY: clean-code-2f-display_gps

