#!/bin/bash
# Pre-commit hook: verify all .c files in code/menu/ are in subdir.mk
# Runs as a PreToolUse hook on Bash commands matching "git commit"

INPUT=$(cat)

# Extract command - pure bash, no jq
COMMAND=$(echo "$INPUT" | grep -o '"command"[[:space:]]*:[[:space:]]*"[^"]*"' | head -1 | sed 's/.*"command"[[:space:]]*:[[:space:]]*"//;s/"$//')

# Only check git commit commands
case "$COMMAND" in
    git\ commit*) ;;
    *) exit 0 ;;
esac

PROJECT_DIR="E:/ads/Seekfree_TC387_Opensource_Library"
MENU_DIR="$PROJECT_DIR/code/menu"
SUBDIR_MK="$PROJECT_DIR/Debug/code/menu/subdir.mk"

if [ ! -f "$SUBDIR_MK" ]; then
    exit 0  # No subdir.mk, can't check
fi

MISSING=""
for src in "$MENU_DIR"/*.c; do
    [ -f "$src" ] || continue
    basename=$(basename "$src")
    if ! grep -q "$basename" "$SUBDIR_MK"; then
        MISSING="$MISSING $basename"
    fi
done

if [ -n "$MISSING" ]; then
    echo "subdir.mk missing source files:$MISSING -- add them to Debug/code/menu/subdir.mk before committing" >&2
    exit 2  # Block commit
fi

exit 0
