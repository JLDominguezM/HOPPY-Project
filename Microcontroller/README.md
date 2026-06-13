# Firmware (TI C2000)

Real-time control firmware for the physical HOPPY, running on a TI LaunchPad
F28379D (C2000 Delfino) under SYS/BIOS. The control loop runs at 1 kHz and is
the same controller as the simulation (see the top-level README and `mujoco/`).

## Layout

- `blinky_rtos_flash/` — the Code Composer Studio project (CPU1 + CPU2).
  - `cpu01/cpu01_main.c` — **the active file**: the full HOPPY controller.
  - `cpu02/` — second core, unused by the controller.
- `cpu01_main_EXAMPLE.c` — the original kit example (motor and sensor test).
- `cpu01_main_PRUEBA_LENTA.c` — a slow bench-test variant, kept for reference.

## Toolchain and hardware

- Code Composer Studio 12.8, TI C2000 compiler 22.6, SYS/BIOS.
- 2x goBILDA 5202 motors driven by VNH5019 H-bridges, quadrature encoders on
  EQEP1/EQEP2, and a SoftPot foot sensor on ADC-D0.

## Build and flash

1. In Code Composer Studio, import `blinky_rtos_flash/` as an existing CCS
   project (File > Import > CCS Projects).
2. Work in `cpu01/cpu01_main.c`. Build it and flash it to the board.
3. Read the live telemetry through CCS Expressions: hop counter, last flight
   time, last stance time, and the PWM and knee-travel peaks.

## What the controller does

`cpu01_main.c` runs a FLIGHT/STANCE state machine at 1 kHz:

- **Flight**: a Cartesian foot PD places the foot at the landing pose.
- **Stance**: a Bezier reaction-force profile pushes off. Touchdown and lift-off
  come from the SoftPot foot sensor (threshold 2048).
- Desired torque is turned into voltage with back-EMF compensation and clipped
  to the 12 V bus, the same actuator model as the simulation.
- Continuous hopping is enabled with `JUMP_MODE`.

## Safety

Read this before powering the motors.

- `MOTORS_OFF` starts at **1**: the controller computes everything but sends
  nothing to the motors. Set it to 0 (live, in Expressions) only when ready.
- `u_lim_test` caps the PWM. Raise it gradually.
- Clamp the leg, or start with a low push force, as the upstream kit warns.
