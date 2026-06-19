---
name: tools
description: "Skill for the Tools area of kalibr-docker. 56 symbols across 6 files."
---

# Tools

56 symbols | 6 files | Cohesion: 78%

## When to Use

- Working with code in `kalibr-camimu-ceres/`
- Understanding how repo_dir, quote_command, run work
- Modifying tools-related functionality

## Key Files

| File | Symbols |
|------|---------|
| `kalibr-camimu-ceres/tools/run_ceres_sweep.py` | sanitize_name, key_values_from_line, parse_solver_options_line, parse_camchain_init_line, parse_orientation_init_line (+15) |
| `kalibr-camimu-ceres/tools/prepare_ceres_inputs.py` | repo_dir, quote_command, run, add_path_mounts, base_docker_command (+11) |
| `kalibr-camimu-ceres/tools/run_ceres_two_stage.py` | append_common_args, build_stage1_command, build_stage2_command, main, repo_dir (+2) |
| `kalibr-camimu-ceres/tools/run_kalibr_docker.py` | _container_path, build_kalibr_args, build_docker_command, _extract_summary, parse_args (+1) |
| `kalibr-camimu-ceres/tools/export_kalibr_bag_to_ceres.py` | _timestamp_ns, _write_corners, _write_poses, _write_imu, main |
| `kalibr-camimu-ceres/tools/export_kalibr_corners.py` | export, main |

## Entry Points

Start here when exploring this area:

- **`repo_dir`** (Function) — `kalibr-camimu-ceres/tools/prepare_ceres_inputs.py:36`
- **`quote_command`** (Function) — `kalibr-camimu-ceres/tools/prepare_ceres_inputs.py:40`
- **`run`** (Function) — `kalibr-camimu-ceres/tools/prepare_ceres_inputs.py:44`
- **`add_path_mounts`** (Function) — `kalibr-camimu-ceres/tools/prepare_ceres_inputs.py:67`
- **`base_docker_command`** (Function) — `kalibr-camimu-ceres/tools/prepare_ceres_inputs.py:82`

## Key Symbols

| Symbol | Type | File | Line |
|--------|------|------|------|
| `repo_dir` | Function | `kalibr-camimu-ceres/tools/prepare_ceres_inputs.py` | 36 |
| `quote_command` | Function | `kalibr-camimu-ceres/tools/prepare_ceres_inputs.py` | 40 |
| `run` | Function | `kalibr-camimu-ceres/tools/prepare_ceres_inputs.py` | 44 |
| `add_path_mounts` | Function | `kalibr-camimu-ceres/tools/prepare_ceres_inputs.py` | 67 |
| `base_docker_command` | Function | `kalibr-camimu-ceres/tools/prepare_ceres_inputs.py` | 82 |
| `prepare_pkl` | Function | `kalibr-camimu-ceres/tools/prepare_ceres_inputs.py` | 99 |
| `prepare_bag` | Function | `kalibr-camimu-ceres/tools/prepare_ceres_inputs.py` | 126 |
| `prepare_euroc` | Function | `kalibr-camimu-ceres/tools/prepare_ceres_inputs.py` | 166 |
| `parse_args` | Function | `kalibr-camimu-ceres/tools/prepare_ceres_inputs.py` | 374 |
| `main` | Function | `kalibr-camimu-ceres/tools/prepare_ceres_inputs.py` | 440 |
| `sanitize_name` | Function | `kalibr-camimu-ceres/tools/run_ceres_sweep.py` | 252 |
| `key_values_from_line` | Function | `kalibr-camimu-ceres/tools/run_ceres_sweep.py` | 285 |
| `parse_solver_options_line` | Function | `kalibr-camimu-ceres/tools/run_ceres_sweep.py` | 294 |
| `parse_camchain_init_line` | Function | `kalibr-camimu-ceres/tools/run_ceres_sweep.py` | 302 |
| `parse_orientation_init_line` | Function | `kalibr-camimu-ceres/tools/run_ceres_sweep.py` | 333 |
| `parse_pose_init_line` | Function | `kalibr-camimu-ceres/tools/run_ceres_sweep.py` | 345 |
| `parse_iteration_state_line` | Function | `kalibr-camimu-ceres/tools/run_ceres_sweep.py` | 352 |
| `update_iteration_trace` | Function | `kalibr-camimu-ceres/tools/run_ceres_sweep.py` | 375 |
| `parse_summary_text` | Function | `kalibr-camimu-ceres/tools/run_ceres_sweep.py` | 397 |
| `main` | Function | `kalibr-camimu-ceres/tools/export_kalibr_bag_to_ceres.py` | 127 |

## Execution Flows

| Flow | Type | Steps |
|------|------|-------|
| `Main → _container_path` | cross_community | 4 |
| `Main → Repo_dir` | intra_community | 4 |
| `Main → Quote_command` | intra_community | 4 |
| `Main → Sanitize_name` | cross_community | 4 |
| `Main → Require_file` | cross_community | 4 |
| `Main → File_arg` | cross_community | 4 |
| `Main → _repo_dir` | intra_community | 3 |
| `Main → Add_path_mounts` | intra_community | 3 |
| `Main → Repo_dir` | intra_community | 3 |
| `Main → Option_present` | cross_community | 3 |

## How to Explore

1. `context({name: "repo_dir"})` — see callers and callees
2. `query({query: "tools"})` — find related execution flows
3. Read key files listed above for implementation details
