################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
"../code/menu/menu.c" \
"../code/menu/menu_gps.c" \
"../code/menu/menu_params.c" \
"../code/menu/menu_pid.c" \
"../code/menu/wifi_menu.c" 

COMPILED_SRCS += \
"code/menu/menu.src" \
"code/menu/menu_gps.src" \
"code/menu/menu_params.src" \
"code/menu/menu_pid.src" \
"code/menu/wifi_menu.src" 

C_DEPS += \
"./code/menu/menu.d" \
"./code/menu/menu_gps.d" \
"./code/menu/menu_params.d" \
"./code/menu/menu_pid.d" \
"./code/menu/wifi_menu.d" 

OBJS += \
"code/menu/menu.o" \
"code/menu/menu_gps.o" \
"code/menu/menu_params.o" \
"code/menu/menu_pid.o" \
"code/menu/wifi_menu.o" 


# Each subdirectory must supply rules for building sources it contributes
"code/menu/menu.src":"../code/menu/menu.c" "code/menu/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2004 "-fE:/ads/Seekfree_TC387_Opensource_Library/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc38x -Y0 -N0 -Z0 -o "$@" "$<"
"code/menu/menu.o":"code/menu/menu.src" "code/menu/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
"code/menu/menu_gps.src":"../code/menu/menu_gps.c" "code/menu/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2004 "-fE:/ads/Seekfree_TC387_Opensource_Library/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc38x -Y0 -N0 -Z0 -o "$@" "$<"
"code/menu/menu_gps.o":"code/menu/menu_gps.src" "code/menu/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
"code/menu/menu_params.src":"../code/menu/menu_params.c" "code/menu/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2004 "-fE:/ads/Seekfree_TC387_Opensource_Library/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc38x -Y0 -N0 -Z0 -o "$@" "$<"
"code/menu/menu_params.o":"code/menu/menu_params.src" "code/menu/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
"code/menu/menu_pid.src":"../code/menu/menu_pid.c" "code/menu/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2004 "-fE:/ads/Seekfree_TC387_Opensource_Library/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc38x -Y0 -N0 -Z0 -o "$@" "$<"
"code/menu/menu_pid.o":"code/menu/menu_pid.src" "code/menu/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
"code/menu/wifi_menu.src":"../code/menu/wifi_menu.c" "code/menu/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2004 "-fE:/ads/Seekfree_TC387_Opensource_Library/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc38x -Y0 -N0 -Z0 -o "$@" "$<"
"code/menu/wifi_menu.o":"code/menu/wifi_menu.src" "code/menu/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"

clean: clean-code-2f-menu

clean-code-2f-menu:
	-$(RM) ./code/menu/menu.d ./code/menu/menu.o ./code/menu/menu.src ./code/menu/menu_gps.d ./code/menu/menu_gps.o ./code/menu/menu_gps.src ./code/menu/menu_params.d ./code/menu/menu_params.o ./code/menu/menu_params.src ./code/menu/menu_pid.d ./code/menu/menu_pid.o ./code/menu/menu_pid.src ./code/menu/wifi_menu.d ./code/menu/wifi_menu.o ./code/menu/wifi_menu.src

.PHONY: clean-code-2f-menu

