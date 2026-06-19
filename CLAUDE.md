<!-- gitnexus:start -->
# GitNexus — Code Intelligence

This project is indexed by GitNexus as **kalibr-docker** (20119 symbols, 30000 relationships, 239 execution flows). Use the GitNexus MCP tools to understand code, assess impact, and navigate safely.

> Index stale? Run `node .gitnexus/run.cjs analyze` from the project root — it auto-selects an available runner. No `.gitnexus/run.cjs` yet? `npx gitnexus analyze` (npm 11 crash → `npm i -g gitnexus`; #1939).

## Always Do

- **MUST run impact analysis before editing any symbol.** Before modifying a function, class, or method, run `impact({target: "symbolName", direction: "upstream"})` and report the blast radius (direct callers, affected processes, risk level) to the user.
- **MUST run `detect_changes()` before committing** to verify your changes only affect expected symbols and execution flows. For regression review, compare against the default branch: `detect_changes({scope: "compare", base_ref: "master"})`.
- **MUST warn the user** if impact analysis returns HIGH or CRITICAL risk before proceeding with edits.
- When exploring unfamiliar code, use `query({query: "concept"})` to find execution flows instead of grepping. It returns process-grouped results ranked by relevance.
- When you need full context on a specific symbol — callers, callees, which execution flows it participates in — use `context({name: "symbolName"})`.

## Never Do

- NEVER edit a function, class, or method without first running `impact` on it.
- NEVER ignore HIGH or CRITICAL risk warnings from impact analysis.
- NEVER rename symbols with find-and-replace — use `rename` which understands the call graph.
- NEVER commit changes without running `detect_changes()` to check affected scope.

## Resources

| Resource | Use for |
|----------|---------|
| `gitnexus://repo/kalibr-docker/context` | Codebase overview, check index freshness |
| `gitnexus://repo/kalibr-docker/clusters` | All functional areas |
| `gitnexus://repo/kalibr-docker/processes` | All execution flows |
| `gitnexus://repo/kalibr-docker/process/{name}` | Step-by-step execution trace |

## CLI

| Task | Read this skill file |
|------|---------------------|
| Understand architecture / "How does X work?" | `.claude/skills/gitnexus/gitnexus-exploring/SKILL.md` |
| Blast radius / "What breaks if I change X?" | `.claude/skills/gitnexus/gitnexus-impact-analysis/SKILL.md` |
| Trace bugs / "Why is X failing?" | `.claude/skills/gitnexus/gitnexus-debugging/SKILL.md` |
| Rename / extract / split / refactor | `.claude/skills/gitnexus/gitnexus-refactoring/SKILL.md` |
| Tools, resources, schema reference | `.claude/skills/gitnexus/gitnexus-guide/SKILL.md` |
| Index, status, clean, wiki CLI commands | `.claude/skills/gitnexus/gitnexus-cli/SKILL.md` |
| Work in the Test area (176 symbols) | `.claude/skills/generated/test/SKILL.md` |
| Work in the Implementation area (151 symbols) | `.claude/skills/generated/implementation/SKILL.md` |
| Work in the Backend area (89 symbols) | `.claude/skills/generated/backend/SKILL.md` |
| Work in the Camera_calibration area (62 symbols) | `.claude/skills/generated/camera-calibration/SKILL.md` |
| Work in the Tools area (56 symbols) | `.claude/skills/generated/tools/SKILL.md` |
| Work in the Io area (53 symbols) | `.claude/skills/generated/io/SKILL.md` |
| Work in the Kalibr_common area (49 symbols) | `.claude/skills/generated/kalibr-common/SKILL.md` |
| Work in the Kinematics area (44 symbols) | `.claude/skills/generated/kinematics/SKILL.md` |
| Work in the Optimizer area (41 symbols) | `.claude/skills/generated/optimizer/SKILL.md` |
| Work in the Residuals area (35 symbols) | `.claude/skills/generated/residuals/SKILL.md` |
| Work in the Algorithms area (34 symbols) | `.claude/skills/generated/algorithms/SKILL.md` |
| Work in the Kalibr_camera_calibration area (33 symbols) | `.claude/skills/generated/kalibr-camera-calibration/SKILL.md` |
| Work in the Interp_rotation area (29 symbols) | `.claude/skills/generated/interp-rotation/SKILL.md` |
| Work in the Variables area (22 symbols) | `.claude/skills/generated/variables/SKILL.md` |
| Work in the Apps area (20 symbols) | `.claude/skills/generated/apps/SKILL.md` |
| Work in the Aslam area (17 symbols) | `.claude/skills/generated/aslam/SKILL.md` |
| Work in the Sm area (16 symbols) | `.claude/skills/generated/sm/SKILL.md` |
| Work in the Base area (16 symbols) | `.claude/skills/generated/base/SKILL.md` |
| Work in the Apriltags area (16 symbols) | `.claude/skills/generated/apriltags/SKILL.md` |
| Work in the Statistics area (14 symbols) | `.claude/skills/generated/statistics/SKILL.md` |

<!-- gitnexus:end -->
