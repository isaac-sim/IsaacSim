# Isaac Sim — Agent Guide

`skills/` is the canonical, agent-agnostic location for SKILL.md-format workflow guides.

Route to skills by the `description` field in each `SKILL.md` frontmatter, which states when that skill applies. Load only the skills the task needs; do not bulk-load the library.

Read [`skills/SKILLS.md`](skills/SKILLS.md) when the task spans several skills, or when you need the composition order between them. It is a routing map, not a prerequisite for every conversation.

Cursor and Claude Code load the same skills via their respective pointer mechanisms (`.cursor/rules/agent_skills.mdc`, `CLAUDE.md`). For Codex (this file), the skills are background context — refer to them when the user's task overlaps a skill's subject.

### Repo-wide rules

[`.cursor/rules/`](.cursor/rules/) holds the `*.mdc` style and policy rules (C++ codestyle, Python codestyle, docs style, extension structure, build instructions, pip packaging, ...). Load them on demand when editing code that falls under their domain.

Choose the ownership model before selecting build, test, packaging, logging, or release instructions:

- Kit-independent API or implementation: `source/libraries` and `library_module_structure.mdc`.
- Kit lifecycle, UI, settings, OmniGraph, or application adapter: `source/extensions` and `extension_structure.mdc`.
- Delivery of a library through Kit: stage its wheel in a thin extension; do not duplicate the library implementation.

Use the owning build: the repository Repo/Premake flow for extensions and applications, and the standalone CMake/Pixi
flow under `source/libraries` for library modules, wheels, SDKs, and `source/examples`.

Skills are authored once under `skills/`. `.claude/skills/` contains symlinks into `skills/` so Claude Code's native skill discovery works without duplication. `.cursor/skills/` is retired.
