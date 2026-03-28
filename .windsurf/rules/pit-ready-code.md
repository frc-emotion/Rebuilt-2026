---
trigger: always_on
---
# Pit-Ready Code

This is FRC robot code. A human needs to read and change logic quickly between competition matches.

- Flatten logic. No nested ternaries, no chained conditionals beyond 2 levels. Use early returns.
- If a lambda has more than one statement, extract it to a named method.
- One idea per method. If you scroll to understand it, split it.
- No magic numbers. Every value a human might change should be a named constant.
- Write the simplest code that works. No extra layers, abstractions, or patterns for future use.
- No boilerplate comments. Comment only when the WHY is non-obvious.
- Remove dead code. Never comment it out.
- When refactoring, read the full file first. Change one thing at a time. Verify it compiles.
- always use the context7 mcp server whenever you believe documentation would be even slightly helpful or help prevent errors down the line
