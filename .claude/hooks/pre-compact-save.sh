#!/bin/bash
# Pre-Compact hook: save current work state before context compression
# Writes a snapshot so Claude can recover context after compaction

INPUT=$(cat)
PROJECT_DIR="E:/ads/Seekfree_TC387_Opensource_Library"
STATE_FILE="$PROJECT_DIR/.claude/compact-state.md"

# Capture current git state
BRANCH=$(cd "$PROJECT_DIR" && git branch --show-current 2>/dev/null || echo "unknown")
LAST_COMMIT=$(cd "$PROJECT_DIR" && git log --oneline -1 2>/dev/null || echo "unknown")
MODIFIED=$(cd "$PROJECT_DIR" && git diff --name-only 2>/dev/null | head -20)
STAGED=$(cd "$PROJECT_DIR" && git diff --cached --name-only 2>/dev/null | head -20)

cat > "$STATE_FILE" << EOF
# Compact State Snapshot
Generated: $(date '+%Y-%m-%d %H:%M:%S')

## Git State
- Branch: $BRANCH
- Last commit: $LAST_COMMIT

## Modified files (unstaged):
$MODIFIED

## Staged files:
$STAGED

## Key project context
- TC387 embedded firmware (Infineon TriCore, TASKING compiler, ADS IDE)
- Display: IPS200 (320x240 SPI LCD)
- Input: 4 keys (P20_2/8/6/7) + rotary encoder (P20_0/3)
- Menu system: code/menu/ directory
- Shared UI utils: menu_ui_utils.c/h (menu_view_ctx_t, menu_step_desc_t)
- Build system: Debug/code/menu/subdir.mk
EOF

exit 0
