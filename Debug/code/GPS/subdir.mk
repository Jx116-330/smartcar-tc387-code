################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
"../code/GPS/display_gps.c" \
"../code/GPS/path_display.c" \
"../code/GPS/path_recorder.c" 

COMPILED_SRCS += \
"code/GPS/display_gps.src" \
"code/GPS/path_display.src" \
"code/GPS/path_recorder.src" 

C_DEPS += \
"./code/GPS/display_gps.d" \
"./code/GPS/path_display.d" \
"./code/GPS/path_recorder.d" 

OBJS += \
"code/GPS/display_gps.o" \
"code/GPS/path_display.o" \
"code/GPS/path_recorder.o" 


# Each subdirectory must supply rules for building sources it contributes
"code/GPS/display_gps.src":"../code/GPS/display_gps.c" "code/GPS/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2004 "-fE:/ads/Seekfree_TC387_Opensource_Library/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc38x -Y0 -N0 -Z0 -o "$@" "$<"
"code/GPS/display_gps.o":"code/GPS/display_gps.src" "code/GPS/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
"code/GPS/path_display.src":"../code/GPS/path_display.c" "code/GPS/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2004 "-fE:/ads/Seekfree_TC387_Opensource_Library/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc38x -Y0 -N0 -Z0 -o "$@" "$<"
"code/GPS/path_display.o":"code/GPS/path_display.src" "code/GPS/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
"code/GPS/path_recorder.src":"../code/GPS/path_recorder.c" "code/GPS/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2004 "-fE:/ads/Seekfree_TC387_Opensource_Library/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc38x -Y0 -N0 -Z0 -o "$@" "$<"
"code/GPS/path_recorder.o":"code/GPS/path_recorder.src" "code/GPS/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"

clean: clean-code-2f-GPS

clean-code-2f-GPS:
	-$(RM) ./code/GPS/display_gps.d ./code/GPS/display_gps.o ./code/GPS/display_gps.src ./code/GPS/path_display.d ./code/GPS/path_display.o ./code/GPS/path_display.src ./code/GPS/path_recorder.d ./code/GPS/path_recorder.o ./code/GPS/path_recorder.src

.PHONY: clean-code-2f-GPS

