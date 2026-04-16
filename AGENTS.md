# AGENTS.md

Use the local skill `jarvis-Codex-pack` as the default workflow guide for engineering tasks.

## Operating style

- Be concise, structured, and execution-oriented.
- Prefer small-step changes over broad refactors.
- For non-trivial work, plan first, then execute.
- Before writing new code, inspect existing project implementation and reuse patterns where possible.
- For bug fixes and build failures, make the smallest viable correction first.
- After code changes, review the result before handoff.
- Do not claim success without concrete verification.

## Preferred workflow

### 1. Search first
Before creating new code, check:
- existing files and nearby modules
- established project patterns
- existing drivers, helpers, and middleware
- external docs only after local inspection is insufficient

### 2. Plan small
For multi-step work, provide:
- files affected
- change intent
- step order
- verification method
- current risk / unknowns

### 3. Fix minimally
When resolving compile, link, type, or integration failures:
- avoid refactoring
- avoid unrelated renames
- avoid speculative cleanup
- fix one blocker at a time
- re-verify after each fix

### 4. Review before handoff
Check:
- scope stayed bounded
- no unrelated behavior was altered
- verification is concrete
- remaining risks are stated clearly

## Embedded / ADS / firmware preferences

- Prefer hardware-path proof before software speculation.
- Separate raw acquisition, parsing, control, and display layers.
- Verify signal presence before higher-level algorithm changes.
- When runtime behavior does not change after flashing, verify final build artifacts and link outputs.
- If evidence isolates the issue to wiring, connector, or board path, stop churning business logic and say so directly.

## Recommended output sections

- Findings
- Planned change
- Files touched
- Verification
- Risks / next step

## Default constraints for coding tasks

- Keep edits local and minimal.
- Do not create extra metadata directories unless explicitly asked.
- Do not move or restructure files unless necessary for the requested task.
- Preserve the existing project conventions unless a change is explicitly requested.
