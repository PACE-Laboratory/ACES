# ACES (Aircraft Control and Estimation Simulator)

ACES (Aircraft Control and Estimation Simulator) is a modular simulation package for aircraft flight dynamics, control, and estimation in Julia.

## Commands

- Instantiate: `julia --project=. -e 'using Pkg; Pkg.instantiate()'`
- Run tests: `julia --project=. -e 'using Pkg; Pkg.test()'`
- Load package: `julia --project=. -e 'using ACES'`

## Documentation

Documentation is built with Documenter.jl.

- Instantiate documentation dependencies: `julia --project=docs -e 'using Pkg; Pkg.instantiate()'`
- Build documentation: `julia --project=docs docs/make.jl`
- Open `docs/build/index.html` to inspect the generated site directly.

The subsystem `THEORY.md` files and `NOTATION.md` are the canonical documentation sources. Keep theory files beside their corresponding subsystems.

- Do not edit or commit `docs/src/notation.md` or `docs/src/theory/`; `docs/make.jl` generates them from the canonical files.
- Do not commit `docs/build/`.
- When adding, removing, or renaming a theory file, update `THEORY_PAGES` in `docs/make.jl`.
- Use `$...$` for inline mathematics without padding inside the delimiters.
- Use fenced `math` blocks for display equations; do not use `$$...$$`.
- Link to the notation reference with `[canonical notation](@ref notation)`.
- Preserve `prettyurls = false` while the generated site is intended to work when opened directly from the filesystem.
- Before finishing any documentation or theory change, build the complete Documenter site and resolve all warnings, broken references, and rendering errors.

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


