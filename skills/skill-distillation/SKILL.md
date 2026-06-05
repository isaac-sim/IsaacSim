---
name: skill-distillation
description: >
  Step 5 of the request loop (`ORIENT -> PLAN -> EXECUTE -> VALIDATE ->
  DISTILL -> DELIVER`). Before delivering any response, extract new skills,
  skill updates, or procedure refinements and write them to the library.
  Triggers: user correction, novel problem solved, undocumented workaround,
  >3-iteration debugging, draft skill created mid-task, heartbeat
  retrospective, every request closure.
---

# Skill Distillation

Step 5 of the universal request loop (see `AGENTS.md` and `meta-skills`). Every user request ends with a distillation pass before delivery. Skip it and the library decays; the next session rediscovers the same lessons.

```
ORIENT -> PLAN -> EXECUTE -> VALIDATE -> DISTILL -> DELIVER
```

## When to run

Run automatically when any of these happen:

1. A task required > 3 iterations.
2. The user corrected you ("that's wrong", "you missed X", "it should be Y").
3. You discovered a workaround not in any existing skill.
4. A sub-agent hit a failure not covered by an existing skill.
5. You solved something that would break again next session without docs.

Also run during heartbeats (every 2-3 days): review recent `memory/` files for undistilled lessons.

## Loop

```
INTERACTION -> TRIGGER CHECK -> EXTRACT -> CLASSIFY -> WRITE -> VERIFY
```

### 1. Trigger check

Ask:

- Did I learn something not in any skill file?
- Did the user correct my approach?
- Did I waste time on something a future agent would also waste time on?
- Was there a procedure that worked but isn't documented?

Any yes -> extract.

### 2. Extract

| What happened | What to extract |
|---|---|
| Debugging session found root cause | Decision tree (symptom -> diagnosis -> fix) |
| User said "do it this way instead" | Procedure update or new rule |
| Tried 5 approaches, only 1 worked | Anti-patterns + the working pattern |
| Used a tool in a new way | New workflow or technique |
| Combined skills in a novel way | Integration pattern |
| Task required a specific order | Phase-gated procedure |

### 3. Classify

```
New knowledge
├── Fits existing skill        -> UPDATE that skill
│   ├── New rule               -> Key Rules
│   ├── New failure mode       -> Failure Modes
│   ├── New procedure          -> Key Workflows
│   └── Correction             -> fix in place
├── Cross-cutting pattern      -> UPDATE meta-skills
├── Entirely new capability    -> CREATE new skill (MSF Phase 1-3)
└── One-off fact               -> MEMORY.md, not skills
```

Rule: if the knowledge helps solve a different future problem, it belongs in a skill. If it's specific to this one situation, it belongs in memory.

### 4. Write

- Skill updates: edit the specific section; bump the Iteration Log.
- New skills: `skills/<name>/SKILL.md` from the meta-skilling template.
- Procedure changes: update the orchestrator or relevant workflow skill.
- Always: date and context ("Learned from SO-101 session 2026-04-08").

### 5. Verify

- Re-read the file. Does it stand on its own?
- Could a fresh agent follow it?
- Is the lesson a procedure, not a fact?

## What good distillation looks like

Bad: "The table had dual RigidBodyAPI which made it explode." This is a fact about one table; useless next session.

Good: "Before using any USD asset in physics, run the Asset Stability Check: load in isolation -> simulate 2 s -> if unstable, scan child meshes for rogue `RigidBodyAPI`." Reusable procedure.

Bad: "SO-101 needs hybrid IK + joint-space." Specific to one robot.

Good: "Arms with < 6 DOF cannot do lateral transport with pure differential IK. Use hybrid: IK for precision positioning, joint-space for large moves." Generalizes to any small-DOF arm.

## The Generalization Rule

Always ask: *"What's the general principle behind this specific fix?"*

| Specific fix | Generalized skill |
|-------------|-------------------|
| Stripped RigidBodyAPI from table legs | Asset stability check procedure |
| Used file writes instead of print() | Headless debugging: stdout is unreliable |
| Added PhysxArticulationAPI.Apply() | Articulation setup checklist |
| Robot moved but didn't grasp | Visual validation must confirm task outcome, not just motion |
| User had to give detailed plan | Auto-decomposition: agent breaks down goals into phases |

## Integration with Orchestrator

The orchestrator's workflow should include distillation as a final step:

```
Phase 1: Verify Foundations
Phase 2: Incremental Integration
Phase 3: Polish & Deliver
Phase 4: Distill ← NEW (automatic, after delivery)
  - What went wrong during this task?
  - What workarounds were discovered?
  - What feedback did the user give?
  - Which skills need updating?
  - Should a new skill be created?
```

## Anti-patterns

- Waiting to be asked. If you learned it, write it now.
- Capturing facts instead of procedures. "X broke" is not a skill; "check X before running" is.
- Over-generalizing from a single data point unless the user confirmed the pattern.
- Under-generalizing: writing a per-asset fix when the pattern applies to all USD assets.
- Skipping verification. An unreadable update is worse than none.
- Duplicating across skills. One canonical location per procedure; cross-reference from others.
- Embedding code in skills. See below.

## Code in Skills — Reference, Don't Embed

Skills should reference canonical sources rather than embed code directly. Embedded code becomes stale when the upstream library or API changes — silently giving future agents wrong patterns to follow.

### Rule
For any code pattern in a skill:
- *If it comes from Isaac Sim examples or docs:* link to the doc page and/or local example file path. Describe the pattern in prose; let the agent read the actual file.
- *If it's a custom utility not in any upstream source:* put it in a versioned script file in the workspace (e.g., `isaac-sim/utils/grasp_utils.py`), reference it by path, and note the version it was written for.
- *If it's a tiny one-liner or config snippet* (e.g., a settings dict): embedding is acceptable, but add a comment with the Isaac Sim version it was validated against.

### Why
- Isaac Sim APIs change between versions. A skill with embedded code can silently guide agents down wrong paths.
- Local example files (`$ISAAC_SIM_DIR/source/standalone_examples/...`) always reflect the installed/built version — they are the most accurate reference available.
- Docs at `docs.isaacsim.omniverse.nvidia.com` are version-tagged — link to `latest` unless pinning to a specific release is intentional.

### What to Write Instead of Code
- The *pattern name* (e.g., "damped least-squares IK")
- *Where to find the canonical implementation* (doc URL + local file path)
- *Key parameters* and how to tune them (table form is fine)
- *What can go wrong* and how to detect it
- *The conceptual flow* in prose: what the code does step by step, without being the code itself

## Library-Health Check (Run During Distillation)

Distillation isn't only "add new knowledge." Each pass should also detect when an *existing* skill has grown stale, bloated, or duplicate. Catch these at write time or they accrete until a full consolidation pass is needed.

### Bloat detection

Before saving a skill update, eyeball the target file:

| Cue | Action |
|---|---|
| Body > **500 lines** after your edit | Offload to sidecar (see "Sidecar Offload" below). Anthropic spec hard limit. |
| Description > **1024 chars** | Compress; move detail to the body. Hard limit. |
| ≥ 50% of the file is fenced code blocks | Extract `≥ 20-line` blocks to `scripts/<name>.py`. |
| 5+ dated sections (`## Learned 2026-MM-DD`, `## Lessons …`) | Move them to a `lessons.md` sidecar. The main file should describe the *current* procedure, not the journey. |
| You're adding a new "Patch 2026-MM-DD" section to fix earlier guidance | **Don't.** Edit the original section in place. Add a 1-line iteration log entry. |

### Sidecar Offload (when SKILL.md is too long)

| What | Where |
|---|---|
| Reusable Python helpers (≥ 20 lines) | `scripts/<name>.py` — import & call from inline 1-liners |
| Long worked examples / case studies | `examples.md` |
| Lessons / dated discoveries / iteration notes | `lessons.md` |
| Deep API reference, parameter tables, config schemas | `reference.md` or `<topic>.md` |
| Multi-step workflows that dominate the file | `workflow.md` |

**Rule:** one level deep only. `scripts/foo.py` and `examples.md` are valid; `scripts/utils/foo.py` is not (Anthropic spec: one-level refs).

### Duplication check

Before adding to a skill, search the library for the same concept:

```
grep -rE "<your-new-concept>" --include=SKILL.md skills/
```

If another skill already documents it:
- **Add to the existing skill**, not your current one. Single source of truth.
- **Cross-link** from your current skill: "See `<other-skill>` for X."
- If the existing skill's coverage is wrong/outdated → fix it there, don't fork.

### Staleness check

Each distillation, ask of the skill you're touching:
- Is the framework version reference still current? (Isaac Sim 5.x → 6.0+, PhysX → Newton)
- Are the hardcoded paths really paths, or should they be `$ENV_VARS`?
- Are there time-sensitive phrases ("before August 2025", "as of 2026-04-04")? Replace with version-anchored or "sample-run" framing.
- Does any "lessons learned" section reference a project that no longer exists?

## Cadence

| Trigger | Action |
|---------|--------|
| After every task with user feedback | Immediate distillation pass |
| After every task with >3 iterations | Immediate distillation pass |
| During heartbeat (every 2-3 days) | Review memory/ for undistilled lessons |
| After a PR is merged | Check if merged work revealed patterns worth capturing |
| User says "remember this" | Write to memory AND check if it's a skill-level lesson |
| New skill created mid-task (no prior skill existed) | Flag as HIGH PRIORITY — see below |
| You added content to a skill that now exceeds 500 lines | Offload to sidecar BEFORE delivering response |

## Draft Skills — High Priority Iteration

When a skill is created *during* a task (i.e., no existing skill covered the feature), it is by definition unproven. Treat it differently:

### Frontmatter Flag
Add to the new skill's YAML frontmatter:
```yaml
status: draft
priority: high
created_from: <task description>
```

### Behavior While Draft Skill Is Active
- *Tell the user immediately:* "I'm working with an untested skill for [feature] — I'll check in after each step rather than at the end."
- *Shorten iteration cycles:* share intermediate results after each meaningful step, not just at delivery
- *Ask early, not late:* if anything about the goal is ambiguous, surface it before implementation, not after
- *Validate direction before depth:* confirm the approach is right before building it out fully
- *Checkpoint messages should include:* what was just completed, what comes next, and any open questions

### Promotion to Stable
After a draft skill has been used successfully on at least one task with user confirmation:
1. Remove `status: draft` and `priority: high` from frontmatter
2. Add `validated: <date>` and a note on what the validation task was
3. Run a full distillation pass to incorporate any corrections from the task
4. Update the orchestrator's feature→skill map if one exists

### Anti-patterns for draft skills

- Running a draft skill to completion without checking in. Direction may be wrong.
- Skipping the user notice. They need to know confidence is lower.
- Promoting to stable after one pass with no user feedback.
