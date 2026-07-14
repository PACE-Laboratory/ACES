# ACES (Aircraft Control and Estimation Simulator)

ACES (Aircraft Control and Estimation Simulator) is a modular simulation package for aircraft flight dynamics, control, and estimation in Julia.

## Commands

- Instantiate: `julia --project=. -e 'using Pkg; Pkg.instantiate()'`
- Run tests: `julia --project=. -e 'using Pkg; Pkg.test()'`
- Load package: `julia --project=. -e 'using AircraftSim'`

Before finishing a code change, run the narrowest relevant tests and then the complete package test suite.

## Architecture

- `types.jl`: shared aliases, enums, and abstract interfaces
- `airframe.jl`: inertial and geometric aircraft data
- `attitude.jl`: rigid body dynamics and attitude representations
- `environment.jl`: gravity and atmospheric density
- `wind.jl`: wind fields, gradients, and turbulence
- `aerodynamics.jl`: aerodynamic force and moment models
- `actuators.jl`: actuator interfaces and dynamics
- `control.jl`: controller interfaces and implementations
- `estimation.jl`: estimation interfaces and implementations
- `core.jl`: simulation layout

Keep `ACES.jl` limited to imports, includes, and exports.

## Coding Conventions

- Public types and functions require docstrings.
- Concrete models implement the abstract interface defined for their subsystem.

## Mathematical Background

Subsystem mathematics are defined by the nearest applicable `THEORY.md`.

Before modifying a mathematical model, numerical implementation, state layout, or subsystem interface:

1. Read `NOTATION.md`. Preserve Unicode mathematical identifiers where they improve correspondence with equations.
2. Read the applicable subsystem `THEORY.md`.
3. Read any `THEORY.md` governing the assembly layer through which the subsystem is evaluated.
4. Identify the equations and invariants affected by the change.
5. Update theory, implementation, and tests together when the mathematical formulation changes.

Treat the following as blocking inconsistencies that must be reported rather than silently resolved:

- theory and implementation disagree;
- theory and an interface contract disagree;
- a test encodes behavior inconsistent with the documented mathematics.

Do not infer missing conventions from common aerospace practice.


