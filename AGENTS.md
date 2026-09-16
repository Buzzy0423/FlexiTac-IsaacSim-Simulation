# Working in this repository

## Read only what the task needs

1. Read `docs/STATUS.md` for current progress and the next unfinished step.
2. Use `docs/README.md` to select **one relevant guide**; read more only when needed.
3. Read `docs/decisions.md` before changing geometry, coordinate conventions or observation semantics.

Do not load all documentation, historical reports, mesh files or the vendored Isaac Lab tree
into context at startup. Use scoped `rg` searches and inspect the owning files.
Keep `STATUS.md` short (about 70 lines); move detailed results into dated reports.

## Ownership and entrypoints

- `Isaacsim_tactile_env/dexmate_workspace.json`: table and robot placement configuration.
- `Isaacsim_tactile_env/dexmate_env.py`: native single-scene reset/step and timed observations.
- `tools/run_dexmate_episodes.py`: repeated grasp validation and synchronized recording.
- `tools/*dexmate*`, `tools/convert_vention_step.py`: local asset preparation and validation tools.
- `Isaacsim_tactile_env/assets/{dexmate,vention_312098_v6}/`: maintained robot/table assets.
- `source/`: upstream Isaac Lab/FlexiTac implementation; modify only when the task requires it.
- The original ALOHA example remains available. Dexmate has a native single-scene environment;
  current task coverage is the scripted left-hand grasp and a small deterministic pose sweep,
  not a general training suite. See STATUS.md for measured outcomes.

## Preserve evidence and sources

- Keep supplied STEP/STL inputs and `assets/dexmate/upstream/` unchanged.
- Asset changes must keep their provenance and manifest hashes consistent; Markdown notes are excluded from asset hashes.
- Generate each validation run in a new `Isaacsim_tactile_env/output/` directory.
- `passed` applies only to the report's declared scope. Self-collision-disabled tests do not validate finger contact.
- Never describe provisional camera parameters, actuator values or geometry checks as calibrated physical measurements.

## Environment and checks

- Simulation/CAD: `conda activate dexmate_isaacsim`, then `./tools/run_dexmate_sim.sh ...`.
- Older offline finger tools use the existing `dex_wire` environment; see `docs/guides/assets.md`.
- Run focused checks for changed behavior. Documentation-only changes need links, provenance,
  `.gitignore` and staged-diff checks, not another GPU simulation.
- For local Python changes run Ruff on the changed scripts. Check shell syntax for launcher changes.
- Before committing inspect staged files and `git diff --cached --check`; keep outputs, caches and secrets out of Git.

## Handoff updates

Update `docs/STATUS.md` with the current result, limitation and next concrete step.
Put durable user decisions in `docs/decisions.md`, commands in guides, and measured results in dated reports.
Avoid duplicating the current task list across READMEs or copying full logs into documentation.
