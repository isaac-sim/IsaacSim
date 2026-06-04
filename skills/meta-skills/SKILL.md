---
name: meta-skills
description: >
  Patterns for structuring, composing, and improving agent skills, plus the
  Meta-Skilling Framework (MSF) for developing new skills from zero to competent.
  Covers skill routing, chunked processing, collaboration protocols, quality gates,
  MSF five phases (Discovery, Practice, Capture, Validation, Iteration), the
  SKILL.md template, and stackability patterns. Use when creating new skills,
  improving existing ones, designing multi-skill workflows, building a skill
  hierarchy, reviewing whether an existing skill needs iteration, or deciding how
  to decompose a domain into sub-skills.
---

# Meta-Skills — Patterns for Authoring Agent Skills

Two parts:

1. Composition patterns: structuring and orchestrating skills.
2. Meta-Skilling Framework (MSF): building new skills from zero.

This is not the skill glossary. The categorized inventory, pipeline diagrams, and tier rankings live in [`SKILLS.md`](../../SKILLS.md).

## Core Operating Principle: Distillation Is Not Optional

Every user request follows a six-step loop: ORIENT → PLAN → EXECUTE → VALIDATE → **DISTILL** → DELIVER. The DISTILL step is mandatory, not aspirational. If a task produced a new insight, workaround, correction, or procedure, the relevant skill file (or this skill, or a new skill) must be updated **before** the response is delivered. Skill files ARE persistent memory — what isn't written down doesn't exist next session.

See:
- [`AGENTS.md`](../../AGENTS.md) — the universal request handling loop and skill index
- [`skills/skill-distillation/SKILL.md`](../skill-distillation/SKILL.md) — the full distillation procedure, generalization rule, classification table, and draft-skill promotion gates

---

# Part 1: Composition Patterns

> **Provenance note:** Several patterns below attribute their origin to skills (`ai-advisor`, `training-ops`, `pbt-system`, `llm-review-protocol`, `collaboration`, `multi-robot-franka`, `sortation-diverter`) from the upstream isaac-claw library that are **not imported into this repo**. The pattern itself is reusable — treat the "Source:" line as historical context, not a navigation pointer.

## 1. Skill Routing

**Pattern:** A top-level orchestrator skill routes to specific sub-skills based on task category.

```
User request → Orchestrator reads context → Routes to specific skill
                                          → Provides shared patterns
```

**Key insight:** The orchestrator doesn't do the work — it identifies which specialist skill should handle it and provides cross-cutting concerns (units, naming, validation).

**Example in this library:** `isaac-sim-orchestrator` — turns natural-language requests into runnable sims and routes to `usd-pipeline`, `isaac-sim-rendering`, `isaac-sim-validator`, `physics-simulation`.

## 2. Chunked Processing

**Pattern:** Prevent context overflow by enforcing batch limits and tier-by-tier processing.

Rules:
- Never process more than N items at once (N=10-20 for complex assets)
- Process in tiers: leaf assets first, then composites, then top-level
- After each chunk, summarize and checkpoint before continuing
- Use explicit "processing X of Y" progress tracking

**When to use:** any time the input set is large enough that bulk-loading risks blowing the context window.

## 3. Quality Gates (from Collaboration / LLM Review)

**Pattern:** Mandatory external review before publishing any output.

Steps:
1. Generate output (render, code, asset)
2. Upload for review (e.g., image to local host at `$IMAGE_SERVER_URL` — typically `http://localhost:8092` — for vision LLM ingestion)
3. Flood context — include full setup, known issues, what was tried
4. Require minimum score (e.g., 7/10) before sharing
5. Iterate based on prioritized improvement list

**Source:** `llm-review-protocol`, `collaboration` — Grok vision review before posting renders

## 4. Safety Rails for Autonomous Modification (from AI Advisor)

**Pattern:** When an AI agent autonomously modifies configuration:

- Max N× baseline per parameter (e.g., 10×)
- Absolute caps (e.g., 5000)
- No sign flips (penalty can't become reward)
- Audit trail of all changes with before/after
- Drift detection against baseline

**Source:** `ai-advisor` — SafetyRails for reward weight modification

## 5. Multi-Skill Composition (from Warehouse Builder)

**Pattern:** Complex tasks declare skill dependencies explicitly.

```yaml
Dependencies:
  - spatial-reasoning            # all transform math
  - urdf-mjcf-to-usd-conversion       # robot asset import
  - usd-articulation             # multi-link validation
  - physics-simulation           # scene + per-prim physics
  - isaac-sim-rendering          # headless render + tone mapping
```

Each dependency is a complete skill that can be used independently. The composite skill orchestrates when to invoke each one.

## 6. Fleet Orchestration (from Training Ops + PBT)

**Pattern:** Manage distributed compute resources as a fleet.

- Pre-flight validation before any operation
- Zombie process cleanup is mandatory
- SSH-based deployment with git commit tracking
- Centralized monitoring via dashboard
- Population-based training for hyperparameter search

**Source:** `training-ops`, `pbt-system`

## 7. Progressive Disclosure (from Multiple Skills)

**Pattern:** Skill files should front-load the most common operations and put deep reference material later.

Structure:
1. Quick decision guide (table: symptom → action)
2. Most common operations with code
3. Configuration reference
4. Troubleshooting
5. Deep reference / edge cases

**Source:** Observed in the best-structured skills (`spatial-reasoning`, `isaac-sim-troubleshooting`)

---

# Part 1b: Concise Skills (Anthropic-Compliant)

Hard constraints (Anthropic skill spec):
- `SKILL.md` body ≤ **500 lines**
- `description` ≤ **1024 chars**, third-person, includes WHAT + WHEN + trigger terms
- Cross-references **one level deep** only (no `subdir/subdir/file.md`)
- No hardcoded user paths, IPs, or time-sensitive phrases ("before August 2025")

The patterns below were extracted from a 170 → 132 library consolidation. They are the operational rules to keep the body ≤ 500 lines without losing knowledge.

## 9. Separation of Concerns — Three-File Pattern

A skill that grows past 300 lines almost always has three kinds of content intermixed. Split them into sidecar files.

| Content | Lives in | Rule of thumb |
|---|---|---|
| **What / When / How (procedure)** | `SKILL.md` | Stays in the main file. ≤ 500 lines hard limit. |
| **Executable code** (≥ 20-line blocks, reusable helpers) | `scripts/<name>.py` | Importable module with docstrings. SKILL.md keeps a 1-line call-site example. |
| **Deep reference / case studies / lessons** | `<topic>.md` siblings | E.g. `lessons.md`, `examples.md`, `advanced.md`, `recipes.md`, `workflow.md`. Cross-link from SKILL.md once. |

**Trigger:** if SKILL.md exceeds 500 lines, ≥ 50% of content is code blocks, OR the file has 5+ dated "Learned YYYY-MM-DD" sections → split.

**Naming convention:** sidecars get a single descriptive name (`examples.md`, not `hard_lessons_2026_05.md`). One sidecar per concern; don't fragment further.

## 10. Code-as-Reference, Not Code-as-Skill

A skill's body should describe **what the code does and when to call it**, not be the code. Concrete rules:

- **≥ 20-line code blocks** → extract to `scripts/<name>.py` as a callable function. Replace in SKILL.md with: function signature + 1-line description + path link.
- **5–20-line blocks** → keep inline if they're the *primary* illustration. Drop them if redundant with a sidecar function.
- **≤ 5-line one-liners and config snippets** → embedded is fine. Add a comment with the API version they were validated against.
- **Tables, decision trees, parameter ranges** → keep inline (they're the agent-facing decision support, not the implementation).

Reasoning: a sidecar script can be imported, version-pinned, and tested independently. Embedded code becomes silently stale as upstream APIs evolve. See `skill-distillation` Part 3 ("Code in Skills — Reference, Don't Embed") for the full policy.

## 11. Description Budget — 1024 Chars Is Not Aspirational

The description is what the agent's discovery system uses to decide whether to load the skill. Long descriptions:
1. Hit the 1024-char hard limit and get truncated by tooling
2. Bury the trigger terms in prose

**Structure that fits comfortably:**
- Sentence 1 (≤ 200 chars): what the skill produces or covers
- Sentence 2 (≤ 200 chars): the key concepts/APIs it owns
- Sentence 3 (≤ 200 chars): "Use when …" with 3–5 specific trigger phrases
- Optional: a "See also …" pointer to sibling skills

**Anti-pattern:** description as a TOC. If you find yourself enumerating every section, you're writing the body twice. Save the detail for the body.

## 12. When to Split vs Merge vs Cross-Link

Decision tree for the most common library-evolution question:

```
Two skills overlap. What now?
│
├── One is a strict subset of the other?
│      → DELETE the subset; absorb unique bits into the superset.
│
├── They share substrate but specialize differently?
│      → EXTRACT shared substrate to a third skill; both depend on it.
│         (e.g. `navigation-primitives` shared by runtime nav + SDG)
│
├── They cover the same domain at different abstraction levels?
│      → MERGE into one with a "When to use this skill (vs siblings)" section.
│         (e.g. `physics-simulation` absorbing 10 PhysX/Newton skills)
│
└── They're distinct but frequently used together?
       → CROSS-LINK with explicit RECEIVES from / PRODUCES for sections.
          (e.g. `urdf-mjcf-to-usd-conversion` ↔ `usd-articulation`)
```

**Anti-pattern: narrow-scope project artifacts.** If a skill is named after a specific asset/task (e.g. `multi-robot-franka`, `sortation-diverter`) and its "reusable principle" is "do exactly this thing" — that's a project note, not a skill. Either fold its content into a general parent or delete.

## 13. Environment Variables, Not User Paths

Skills run on different machines. Every absolute path is a portability bug.

| Replace | With |
|---|---|
| hardcoded Isaac Sim checkout path | `$ISAAC_SIM_DIR` |
| hardcoded SimReady asset path | `$SIMREADY_ASSETS/...` |
| `localhost:8095` | `$KB_SERVICE_URL` (default `http://localhost:8095`) |
| hardcoded image review endpoint | `$IMAGE_SERVER_URL` (placeholder OK; never bake the endpoint) |
| `C:\_Data\exts\content-pipeline\cip.bat` | `$CIP_ROOT/exts/content-pipeline/cip.bat` (Windows skills declare platform in a preamble) |

The orchestrator skill owns the env-var contract; per-skill files reference vars, never declare values. See `isaac-sim-orchestrator` § Environment.

---

# Part 2: Meta-Skilling Framework (MSF)

A systematic, repeatable process for developing new skills from zero to competent in any domain. Designed for an AI agent with session-based memory loss where skill files ARE the persistent memory.

## When to Use MSF
- Starting to learn any new tool, domain, or capability
- Building a skill hierarchy (e.g., Blender → modeling, materials, rigging...)
- Reviewing whether an existing skill needs iteration

## The Five Phases

### Phase 1: Discovery — define what you need to learn and why
1. Identify the need — what problem does this skill solve?
2. Define scope — boundaries; one skill or a hierarchy?
3. Break down sub-skills — list components, create hierarchy if complex
4. Research resources — official docs, API references, tutorials
5. Set competence goals — specific tasks you should be able to perform when done
6. Check existing skills — do any of your skills overlap? Link and reuse.

**Output:** Initial SKILL.md with purpose, scope, hierarchy, resources, goals.

### Phase 2: Practice — build hands-on competence through structured experimentation
1. Follow the learning path — foundational, then advance
2. Do small practical tasks — don't just read; build
3. Take notes AS you go — commands, workflows, gotchas
4. Identify patterns — what's repeatable, what can be scripted
5. Track challenges — tag what's hard; these become iteration targets

**Output:** Raw practice notes, initial scripts, identified patterns.

### Phase 3: Capture — convert raw learnings into structured, reusable knowledge
1. Organize — convert practice notes into structured sections
2. Standardize — follow the SKILL.md template (below)
3. Save scripts — working code in `scripts/`. Reference docs in `references/`.
4. Version — start at v0.1, increment on significant updates
5. Link related skills — cross-reference parent/child/sibling skills

**Critical Rule:** If it's not in the file, it doesn't exist next session.

### Phase 4: Validation — prove the skill actually works
1. Test it — perform a real task using only what's documented
2. Check against goals — can you do what Discovery said you should?
3. Identify gaps — tag as "Validation Gaps" for iteration
4. Get feedback — from user, validator, or test results

### Phase 5: Iteration — refine and expand based on real usage
1. Review gaps at session start
2. Research solutions
3. Update and version
4. Expand scope — add sub-skills once core is solid
5. Re-validate

## Skill Hierarchy Pattern

```
domain/
├── SKILL.md              # Parent: overview, skill tree, progress tracker
├── sub-skill-1/
│   ├── SKILL.md          # Sub-skill: focused docs
│   ├── scripts/
│   └── references/
└── sub-skill-2/...
```

Rules: parent has the full skill tree with status indicators; sub-skills link back to parent; track dependencies; master foundational sub-skills before dependent ones.

## SKILL.md Template

```markdown
---
name: skill-name
description: >
  [1-3 sentences: what, when to use, triggers]
---

# [Skill Name]

## When to Use This Skill
- [task 1]
- [task 2]

## Related / Integration Points
- RECEIVES from: [skill] → [data]
- PRODUCES for: [skill] → [data]

## Core Concepts
[Organized knowledge]

## Key Workflows
[Step-by-step procedures with code]

## Decision Trees
[Branching logic for "it depends" situations]

## Failure Modes & Recovery
For each major workflow:
- FAILURE: [what goes wrong]
- SYMPTOMS: [how you notice]
- CAUSE: [why]
- RECOVERY: [fix]
- PREVENTION: [how to avoid]

## Expert Intuition
- Rules of thumb
- Smell tests
- Proportional judgments

## Context Matrix
| Context | Behavior Change |
|---------|----------------|

## Environment Requirements
Pre-flight checks and tool versions.

## Iteration Log
- v0.1 [date]: Initial capture
```

## Cold-Start Expertise Patterns

Goal: BE competent the moment you read the skill file.

- **Executable Knowledge Blocks (EKBs)**: `IF [condition] THEN [action] BECAUSE [reasoning]`
- **Decision Trees**: embed branching logic directly
- **Failure Mode Catalog**: FAILURE/SYMPTOMS/CAUSE/RECOVERY/PREVENTION
- **Worked Examples**: at least 2 per workflow, input→decisions→output
- **Context Matrix**: same skill across different situations
- **Confidence Tags**: `verified`, `probable`, `theoretical`

## Anti-Patterns

- Learning without capturing — you WILL forget
- Starting too broad — focus on one sub-skill at a time
- Skipping validation — untested = unreliable
- Copy-pasting tutorials without adapting to YOUR context
- Ignoring existing skills — always check for overlap first
- Perfectionism before practice — rough working skill beats perfect plan

## Session Start Protocol

1. Read this file to remember the framework
2. Read the PARENT skill for the domain you're working in
3. Read the specific SUB-SKILL you're focusing on
4. Check Challenges & Gaps — that's your TODO
5. Pick up where the last session left off
