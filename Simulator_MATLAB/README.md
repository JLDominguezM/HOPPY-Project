# MATLAB reference simulator

The original MATLAB simulator of HOPPY from the paper (Ramos, Ding, Kim, Murphy,
Block). It is the reference this project validates the MuJoCo port against: the
hybrid controller in `mujoco/controller.py` is a faithful port of this code.

## Run

```bash
cd Simulator_MATLAB
matlab        # then, inside MATLAB, run: MAIN
```

`MAIN.m` runs the hybrid hopping loop with animation and writes a video.
`video.mp4` is an example output.

## Validation against MuJoCo

`export_ref.m` runs the same loop without animation and writes the reference
trajectory to `../mujoco/ref_matlab.csv`. The MuJoCo suite (`mujoco/verify.py`)
compares against that file.

```bash
matlab -batch "export_ref"
```

## Layout

- `MAIN.m` — entry point (hybrid hopping loop and animation).
- `export_ref.m` — dumps the reference trajectory for the MuJoCo comparison.
- `fcns/` — dynamics, contact events, impact map, Bezier utilities and the
  physical parameters (`get_params.m`).
- `gen/` — generated kinematics and dynamics functions.
- `Code_Instructions.pdf` — the upstream walkthrough of the simulator.

Requires MATLAB (tested on R2026a). This folder is upstream material kept for
reproducibility and credit; see [../NOTICE](../NOTICE).
