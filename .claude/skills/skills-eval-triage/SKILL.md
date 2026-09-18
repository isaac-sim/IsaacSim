---
name: skills-eval-triage
description: "Interpret SkillEvaluator T1/T2/T3 reports and apply targeted fixes to skills. Use when a skill is blocking or has actionable findings after an eval run."
license: Apache-2.0
metadata:
  author: Renato Gasoto <info@nvidia.com>
  tags: [ci, skills, triage, quality]
---

# Skills Eval Triage

## Purpose

Read the committed evaluation report, identify actionable findings per skill, apply targeted
fixes to SKILL.md or scripts, and verify locally — without re-running the full eval suite.

Scope: T1 (schema / security / quality / lint) and T2 (context dedup). T3 neutral verdicts
have an interpretation guide but no automated fix path; they require authoring judgment.

## Prerequisites

- Repo checkout with `_logs/` populated from a recent local run, **or** the committed
  `tools/ci/gitlab/skills/report/` tree from the last pipeline.
- `skillevaluator` installed (for local re-validation): `pip install skillevaluator==0.8.3`
- `NVIDIA_INFERENCE_KEY` set (needed only if re-running T1 with `--llm-verify`).

## Reading the report

Locate the report using the first source that exists:

| Source | When it exists |
|---|---|
| `tools/ci/gitlab/skills/report/index.html` | After a committed pipeline run or a local run where `write_run_reports` completed. |
| `_logs/consolidated-report-*.html` (latest by mtime) | After any local `validate-skills-local.sh` run, even before the first commit. |
| `_logs/<skill>/skillevaluator-output-*.html` | Individual T1 reports — always present after a local run; use directly if consolidation did not finish. |
| `_logs/tier3/<skill>/skillevaluator-output-*.html` | Individual T3 reports — present per-skill after Tier 3 ran locally. |

If `tools/ci/gitlab/skills/report/` does not exist yet (first ever run), regenerate it from
the local logs:

```bash
python3 tools/ci/gitlab/skills/update_scorecard.py \
  --logs-dir _logs --skills-root skills \
  --state tools/ci/gitlab/skills/report/SKILL_HEALTH.json \
  --origin local --commit "$(git rev-parse HEAD)"

python3 tools/ci/gitlab/skills/consolidate_reports.py \
  --logs-dir _logs \
  --state tools/ci/gitlab/skills/report/SKILL_HEALTH.json \
  -o tools/ci/gitlab/skills/report/index.html
```

Then open `tools/ci/gitlab/skills/report/index.html`. Sort by **Block** descending to
surface blocking skills first. For each blocking or failing skill:

1. Click its **T1** link → per-skill static report with full finding list and fix suggestions.
2. Click its **T3** link → agent eval report with per-dimension scores and trial traces.
3. Cross-reference **SKILL_HEALTH.json** for the last-known commit and execution status.

Key columns:
- **Block** — `YES (T1)` means a critical or high T1 finding; `YES (T3)` means T3 execution
  failed or verdict is `fail`. Both gate a merge.
- **T1 findings** — worst-severity pill. `clean` means no C/H findings; advisories are shown
  in the sub-label (`3M 2L`).
- **T3 verdict** — `pass` / `neutral` / `fail` / `⚠ failed` (infrastructure error).
- **T3 score / lift** — `neutral` with a good score (>80%) means the skill is valid but did
  not measurably beat the no-skill baseline.

## T1 triage decision table

Work findings in severity order: Critical → High → Medium. Low findings are advisory only.

### Schema (SCHEMA-*)

| Finding | Action |
|---|---|
| `Missing recommended section: '## Instructions'` | Add `## Instructions` heading with the skill's step-by-step guidance. |
| `Missing recommended section: '## Examples'` | Add `## Examples` with 2-3 concrete prompt/response pairs. |
| `metadata.tags missing` | Add `tags: [topic1, topic2]` under `metadata:` in frontmatter (1-5 kebab-case tags). |
| `author_format` | Fix `author:` to `Firstname Lastname <email>` format. |
| `Unexpected nesting depth` | Advisory only for internal skills under `skills/_internal/`; no fix required. |
| `line_count exceeded` | Trim SKILL.md below 500 lines; move reference tables to linked docs. |

### Security (SECURITY-HIGH → blocking)

Security findings that survive `--llm-verify` are more likely genuine. Read the **Fix**
block in the T1 report — it is generated per-instance and names the exact file and line.

**Before fixing, classify the finding:**

| Signature | Classification | Action |
|---|---|---|
| `eval()` on user-supplied or model-generated input | Genuine | Replace with `ast.literal_eval()` for literals; `json.loads()` for JSON. |
| `os.system(cmd)` where `cmd` is constructed from arguments | Genuine | Replace with `subprocess.run([...], check=True)` using a list, never a string. |
| `subprocess.run(["bash", script, user_arg])` | Genuine | Validate `user_arg` is a safe path before passing: `os.path.abspath`, check against allowed roots. |
| Validation-bypass flag (e.g. a CLI flag that suppresses a safety check) with no documentation | Genuine | Document the flag's purpose and the conditions under which bypassing validation is safe, or remove it if no legitimate use exists. |
| Finding describes the **intentional design** of the skill | False positive | The skill IS about executing code in a running sim / sending commands to a server. Add a comment in the SKILL.md `## Security` section acknowledging the design characteristic. `--llm-verify` should have caught this — if it didn't, note it as a known false positive and move on. |
| `Personal Linux home directory path` (PII Scan HIGH) | False positive | Check if the path is in a comment or example. Replace with `/path/to/...` placeholder. |

**Common false-positive signatures for Isaac Sim skills:**

- "The script's core function is to send arbitrary Python code to a running server" — design characteristic of `isaac-sim-remote`.
- "executes any registered omni.kit.commands command by name" — design characteristic of command-dispatching skills.
- "accepts and executes arbitrary Python" — design characteristic of the python_server architecture.

These should have been downgraded by `--llm-verify`. If they weren't, add a `## Security` section to the SKILL.md explaining the threat model and why the design is intentional, then re-run.

### Code Risk (Code Risk Analysis)

| Finding | Action |
|---|---|
| `CWE-377: insecure temp file` | Use `tempfile.NamedTemporaryFile(delete=False)` or `tempfile.mkstemp()`. |
| `insecure function: eval()` | Same as Security → eval above. |
| `shell injection via subprocess` | Use list form: `subprocess.run(["cmd", arg1, arg2])`, never `subprocess.run(f"cmd {arg}")`. |

### Quality (QUALITY-MEDIUM → advisory, not blocking)

Quality advisories reduce the T1 grade but don't block unless the overall grade drops to D/F.
Address them when the grade is B or lower or when fixing other issues in the same skill.

| Finding | Action |
|---|---|
| `metadata.tags missing` | Same as Schema → tags. |
| `Instructions don't mention 'run_script'` | Reference the relevant script in the Instructions section. |
| `No documented scripts in table format` | Add a scripts table: `\| Script \| Purpose \|` listing each file under `scripts/`. |
| `has_instructions: false` | The SKILL.md has no `## Instructions` (or equivalent) heading. |
| `has_examples: false` | The SKILL.md has no `## Examples` section. |

### Script lint (SCRIPT_LINT-MEDIUM → advisory)

These never block a merge. Fix when they appear alongside other issues in the same script.

| Finding | Action |
|---|---|
| `deeply nested code (depth N, max 6)` | Extract the inner block into a named helper function. |
| `no function definitions (flat script)` | Wrap the script body in `def main(): ...` and `if __name__ == "__main__": main()`. |

## T2 triage

### T2A — intra-skill context dedup

`DUPLICATE-HIGH` in the context-optimization-check log means two sections of the SKILL.md
are semantically near-identical. Consolidate: keep the more detailed version, remove or
link the duplicate.

### T2B — inter-skill similarity

`SIMILARITY-HIGH` in the Tier 2B log means this skill overlaps heavily with another.
Options:
- Add a `## Related Skills` section naming the overlapping skill and explaining when to
  choose one over the other.
- If the overlap is substantial, consider merging the skills or making one call the other
  via `meta-skills` composition patterns.

## T3 interpretation

T3 verdicts are not auto-fixed — they require authoring judgment. Use this table to decide
whether to act and what to change.

| Verdict | Score | Lift | Interpretation | Action |
|---|---|---|---|---|
| `pass` | ≥ 85% | positive | Skill is effective. | None. |
| `neutral` | ≥ 70% | ≈ 0 | Skill is valid but agents perform similarly without it. | Strengthen: add concrete examples, sharpen negative routing (when NOT to use), clarify the distinctive value the skill adds. |
| `neutral` | < 70% | ≈ 0 | Skill may confuse agents on these tasks. | Audit instructions for ambiguity; check if evals test the skill's actual domain. |
| `pass` / `neutral` | any | negative | Skill is hurting agents compared to no-skill. | Review instructions for overreach or conflicting guidance. |
| `⚠ failed` | — | — | Trials were invalidated (timeout, connection reset, non-zero exit). | Infrastructure issue — check eval config and harbor logs. Do NOT treat as a quality signal. |
| `❌ fail` | 0% | none | All trials scored 0 — usually invalidated, not quality failure. | Check the T3 detail report for `AgentTimeoutError` or exit codes before concluding quality is bad. |

**Dimension scores** (visible in the T3 detail report): focus on the lowest-scoring dimension.

| Dimension | Low score means | Fix |
|---|---|---|
| `goal_accuracy` | Agent is completing tasks but not achieving the skill's stated goals. | Rewrite the goal statement; add expected-output examples with goal-level assertions. |
| `skill_execution` | Agent is not invoking the skill's recommended workflow. | Make the trigger clearer; add a `## When to Use` section. |
| `skill_efficiency` | Agent is using the skill but taking unnecessary extra steps. | Add a `## Common Mistakes` or `## Anti-patterns` section. |
| `behavior_check` | Agent output fails behavioral assertions (output format, side-effects). | Add an `## Output Format` section; tighten expected-output descriptions in evals. |
| `security` | Agent is producing outputs that fail security checks. | Add a `## Security` note; ensure examples don't demonstrate unsafe patterns. |

## Verification workflow

After applying fixes, validate locally without running the full suite:

```bash
# T1 only — fast (<2 min per skill), no GPU needed
SKILL_FILTER=<skill-name> SKIP_TIER2=1 RUN_TIER3=0 \
  tools/ci/gitlab/skills/validate-skills-local.sh

# T1 + T2A — catches context dedup regressions
SKILL_FILTER=<skill-name> RUN_TIER3=0 \
  tools/ci/gitlab/skills/validate-skills-local.sh

# Multiple skills at once
SKILL_FILTER=isaac-sim-installation,motion-generation SKIP_TIER2=1 RUN_TIER3=0 \
  tools/ci/gitlab/skills/validate-skills-local.sh
```

The per-skill T1 report is written to `tools/ci/gitlab/skills/report/tier1/<skill>.html`.
Open it to confirm the specific finding is resolved.

**Do not re-run T3 to verify T1 fixes** — T3 takes hours and T1 is independent.
Re-run T3 only when instructions or examples changed substantively.

## Handoff pattern

After T1/T2 fixes are applied and verified:

1. If the fixes revealed a generalizable pattern (e.g., all skills need a `## Security`
   section for design-level notes), invoke `skill-distillation` to capture that as a
   library-wide convention.
2. If a T3 neutral verdict led to authoring changes, re-run T3 for that skill alone:
   ```bash
   SKILL_FILTER=<skill-name> SKIP_TIER1=1 SKIP_TIER2=1 FORCE_TIER3_ALL=1 \
     tools/ci/gitlab/skills/validate-skills-local.sh
   ```
3. Commit changes to the skill files; the next MR pipeline will update the report.

## Inputs

| Input | Source | Required |
|---|---|---|
| Skill name(s) to triage | User prompt or report **Block** column | Yes |
| `tools/ci/gitlab/skills/report/index.html` | Committed report tree | Preferred; fall back to `_logs/` |
| `tools/ci/gitlab/skills/report/SKILL_HEALTH.json` | Committed report tree | For commit context |
| Per-skill T1 HTML (`report/tier1/<skill>.html`) | Committed or `_logs/<skill>/`) | For finding details |
| Per-skill T3 HTML (`report/tier3/<skill>.html`) | Committed or `_logs/tier3/<skill>/` | For T3 scores and trial traces |

## Output Format

For each triaged skill, produce:

1. **Finding summary** — validator, severity, finding code, affected file and line.
2. **Classification** — Genuine / False positive / Advisory (non-blocking).
3. **Applied fix** — what was changed and why, or why no change was made.
4. **Verification result** — output of the targeted `validate-skills-local.sh` re-run confirming the finding is resolved.

## Limitations

- LLM verify (`--llm-verify`) is required to reliably downgrade Isaac Sim architecture false positives (python_server, command dispatch). Without `NVIDIA_INFERENCE_KEY` the fallback keeps them as HIGH and they will block.
- T3 neutral verdicts require authoring judgment — there is no deterministic fix path.
- T3 execution failures (`⚠ failed`) cannot be diagnosed from the SKILL.md alone; they require the Harbor job log.
- This skill covers public and internal skills equally; `_internal/` schema advisories (nesting depth) are expected and inert.

## Troubleshooting

| Symptom | Cause | Action |
|---|---|---|
| `report/index.html` missing or empty | First local run; `write_run_reports` did not complete | Run `update_scorecard.py` + `consolidate_reports.py` from `_logs/` (see Reading the report) |
| Security HIGH survives after SKILL.md fix | `--llm-verify` fell back (no `NVIDIA_INFERENCE_KEY`) | Set `NVIDIA_INFERENCE_KEY` and re-run; or add a `## Security` section explaining the design characteristic |
| Targeted re-run still shows the finding | Fix was incomplete or the finding has multiple instances | Check the finding's **File** and **Content** fields for all occurrences |
| T3 shows `⚠ failed` after a SKILL.md fix | Infrastructure issue unrelated to the fix | Check the Harbor job log; the SKILL.md change did not cause this |
| `SKILL_FILTER` re-run produces different findings than the full run | Per-skill vs. catalog-bundled scans differ slightly | Run the full `validate_catalog` pass to confirm before marking a finding resolved |

## What not to fix

- **T3 execution failures** (`⚠ failed`): these are infra/config issues, not authoring
  problems. Check the Harbor job log, not the SKILL.md.
- **SCHEMA-MEDIUM: Unexpected nesting depth** for internal skills: internal skills live
  under `skills/_internal/` which is a non-standard hierarchy; this is expected and inert.
- **Security findings describing Isaac Sim's architecture**: the python_server, command
  dispatch, and remote-control patterns are inherent to the domain. Document, don't remove.
- **Low SCRIPT_LINT advisories on scripts that already work**: flat scripts and deep nesting
  are advisory only. Fix only when the script is already being edited for another reason.
