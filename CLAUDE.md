# Isaac Sim — Claude Code Guide

The agent guide lives in [`AGENTS.md`](AGENTS.md). It is the single source of
truth for every agent platform (Cursor, Codex CLI, Claude Code, ...). `skills/`
is the canonical, agent-agnostic location for SKILL.md-format workflow guides.

At session start, read in full:

1. [`AGENTS.md`](AGENTS.md) — request loop, skill layout, library pointers.
2. All `SKILL.md` files anywhere under `skills/`. Use a traversal that includes
   every nested directory even when search tools would normally skip ignored
   paths, for example:
   ```bash
   find skills -type f -name SKILL.md -print
   ```
3. All `*.mdc` files in [`.cursor/rules/`](.cursor/rules/) — repo-wide style and
   policy rules (C++ / Python codestyle, docs style, extension structure, build
   instructions, pip packaging, ...). Platform-independent.
4. [`skills/SKILLS.md`](skills/SKILLS.md) — categorized index of all skills with
   one-line descriptions, read order, pipeline diagrams, and the env-var
   contract. Use it to route to individual skills on demand.

Skills are authored once under `skills/`. `.claude/skills/` contains symlinks
into `skills/` so Claude Code's native skill discovery works without
duplication. `.cursor/skills/` is retired.
