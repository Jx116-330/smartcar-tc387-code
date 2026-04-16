# Compact State Snapshot
Generated: 2026-04-13 19:04:31

## Git State
- Branch: main
- Last commit: 390f841 refactor(menu): 提取共享UI工具 + 统一视图上下文 + 消除屏幕刷新闪烁

## Modified files (unstaged):
Debug/Seekfree_TC387_Opensource_Library.elf
Debug/Seekfree_TC387_Opensource_Library.hex
Debug/Seekfree_TC387_Opensource_Library.map
Debug/Seekfree_TC387_Opensource_Library.mdf
Debug/code/Beep/Beep.o
Debug/code/Beep/Beep.src
Debug/code/GPS/display_gps.o
Debug/code/GPS/display_gps.src
Debug/code/GPS/path_display.o
Debug/code/GPS/path_display.src
Debug/code/GPS/path_recorder.o
Debug/code/GPS/path_recorder.src
Debug/code/MyEncoder/MyEncoder.o
Debug/code/MyEncoder/MyEncoder.src
Debug/code/MyKey/MyKey.o
Debug/code/MyKey/MyKey.src
Debug/code/menu/menu.d
Debug/code/menu/menu.o
Debug/code/menu/menu.src
Debug/libraries/infineon_libraries/Configurations/Ifx_Cfg_Ssw.o

## Staged files:


## Key project context
- TC387 embedded firmware (Infineon TriCore, TASKING compiler, ADS IDE)
- Display: IPS200 (320x240 SPI LCD)
- Input: 4 keys (P20_2/8/6/7) + rotary encoder (P20_0/3)
- Menu system: code/menu/ directory
- Shared UI utils: menu_ui_utils.c/h (menu_view_ctx_t, menu_step_desc_t)
- Build system: Debug/code/menu/subdir.mk
