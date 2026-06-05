# Isaac Sim — Agent Guide

At the start of every conversation, read the following in full to load all project rules and skills:

- All `SKILL.md` files anywhere under `skills/`. Use a traversal that includes every nested directory even when search tools would normally skip ignored paths, for example:
  ```bash
  find skills -type f -name SKILL.md -print
  ```

`skills/` is the canonical, agent-agnostic location for SKILL.md-format workflow guides.

Cursor and Claude Code load the same skills via their respective pointer mechanisms (`.cursor/rules/agent_skills.mdc`, `CLAUDE.md`). For Codex (this file), the skills are background context — refer to them when the user's task overlaps a skill's subject.

### Repo-native skills

| Skill | What it does |
|---|---|
| [`isaac-sim-remote`](skills/isaac-sim-remote/SKILL.md) | Drive a running Isaac Sim via the `isaacsim.code_editor.python_server` TCP socket (port 8226). Headless OK. |
| [`profile-isaac-sim`](skills/profile-isaac-sim/SKILL.md) | Profile and optimize with the in-repo benchmark scripts + Tracy; compare runs, diff frame times. |
| [`validation-diff-gifs`](skills/validation-diff-gifs/SKILL.md) | Pixel-diff GIFs of validation captures vs golden data; fastest path to triage benchmark image failures. |

### Robotics-sim skills (load on demand)

Additional skills covering orchestrator + meta, URDF -> USD, physics, navigation/fleets, manipulation, sensors, rendering, USD pipeline, ROS 2 bridge, and operational meta-skills. They assume you are driving a built Isaac Sim from a script (`SimulationApp`, `isaacsim.core.experimental.*`, USD authoring, RL with Isaac Lab).

Do not load all at once. Use the categorized index in [`skills/SKILLS.md`](skills/SKILLS.md) for pipeline diagrams, priority read order, the `$ISAAC_SIM_DIR` / `$ISAAC_LAB_DIR` / `$WORKSPACE_DIR` contract, and "when stuck" pointers.

### Repo-wide rules

[`.cursor/rules/`](.cursor/rules/) holds the `*.mdc` style and policy rules (C++ codestyle, Python codestyle, docs style, extension structure, build instructions, pip packaging, ...). Cursor loads them automatically; other platforms read them at session start.

Skills are authored once under `skills/`. `.claude/skills/` contains symlinks into `skills/` so Claude Code's native skill discovery works without duplication. `.cursor/skills/` is retired.
