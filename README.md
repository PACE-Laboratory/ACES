# ACES

ACES (Aircraft Control and Estimation Simulator) is a modular Julia package for aircraft flight dynamics, control, and estimation. It assembles interchangeable rigid-body, environment, wind, aerodynamic, actuator, measurement, estimator, and controller models into an ODE or SDE simulation.

ACES is currently under active development. The public interfaces are usable for experiments and aircraft-specific model development, but may change before the first stable release.

## Installation

Until ACES is registered, install the development version directly from GitHub:

```julia
using Pkg
Pkg.add(url="https://github.com/PACE-Laboratory/ACES.git")
```

To work from a local checkout instead:

```sh
git clone https://github.com/PACE-Laboratory/ACES.git
cd ACES
julia --project=. -e 'using Pkg; Pkg.instantiate()'
```

## Quick start

The following example constructs a deterministic, zero-aerodynamic-load aircraft and simulates its motion under gravity:

```julia
using ACES
using LinearAlgebra

rigid_body = RigidBody(1.0, Matrix{Float64}(I, 3, 3))

zero_aerodynamics = QuasiSteadyAerodynamics(
    (v_r, ω_r, x_aero, δ, ρ) -> zeros(eltype(v_r), 3),
    (v_r, ω_r, x_aero, δ, ρ) -> zeros(eltype(v_r), 3),
)

# This measurement returns the 13 rigid-body states. Because the rigid-body
# block is first in the extended state, its range is 1:13 in this model.
measurement = IdealMeasurement((x, t) -> copy(x[1:13]))
estimator = IdentityEstimator()

# With actuator=nothing, both controller callables must ignore δ. ACES then
# evaluates the command once and applies the ideal relation δ = u.
controller = FunctionController(
    0,
    (x_ctrl, x_rb, δ, ϖ, xhat, t) -> empty_state(eltype(x_ctrl)),
    (x_ctrl, x_rb, δ, ϖ, xhat, t) -> [0.0],
)

aircraft = AircraftModel(
    rigid_body=rigid_body,
    environment=Environment(),
    wind=ConstantWind(),
    aerodynamics=Aerodynamics(zero_aerodynamics),
    actuator=nothing,
    measurement=measurement,
    estimator=estimator,
    controller=controller,
)

# Unit-quaternion rigid-body state:
# [position; quaternion; body velocity; body angular velocity]
x_rb0 = [
    0.0, 0.0, 0.0,
    1.0, 0.0, 0.0, 0.0,
    20.0, 0.0, 0.0,
    0.0, 0.0, 0.0,
]
x0 = initial_state(aircraft; x_rb=x_rb0)

# Preflight the selected interfaces and initial dimensions. This validation is
# also performed automatically by simulation_problem and simulate.
validate(aircraft, x0)

solution = simulate(
    aircraft,
    x0,
    (0.0, 5.0);
    abstol=1e-9,
    reltol=1e-9,
)

final_states = state_components(aircraft, solution.u[end])
final_rigid_body = state_view(aircraft.rigid_body, final_states.x_rb)
```

`state_layout(aircraft)` reports the range assigned to every subsystem, while
`state_components(aircraft, x)` returns non-copying named views of those
blocks.

## Aircraft-specific models

Reusable baseline models live under `src/models/`. A model specific to an aircraft should normally live in the user's workspace or in a separate Julia package. Define a concrete subtype and extend the ACES interface functions:

```julia
module MyAircraftModels

import ACES

struct MyAerodynamics <: ACES.AbstractAerodynamicModel
    reference_area::Float64
end

ACES.state_dimension(::MyAerodynamics) = 0

ACES.F_model(model::MyAerodynamics, v_r, ω_r, x_aero, δ, ρ) = begin
    speed_squared = sum(abs2, v_r)
    drag = 0.5 * ρ * model.reference_area * 0.02 * speed_squared
    [-drag, zero(drag), zero(drag)]
end

ACES.M_model(model::MyAerodynamics, v_r, ω_r, x_aero, δ, ρ) = begin
    zeros(eltype(v_r), 3)
end

ACES.f_aero(::MyAerodynamics, x_aero, v_r, ω_r, δ, ρ) =
    ACES.empty_state(eltype(x_aero))

end
```

Load that file and select the model explicitly:

```julia
include("MyAircraftModels.jl")
using .MyAircraftModels

aerodynamics = Aerodynamics(MyAircraftModels.MyAerodynamics(16.2))
```

The same pattern applies to custom wind, actuator, measurement, estimator, controller, environment, and rigid-body models. The `validate` preflight is intended to catch missing methods, incorrect state dimensions, invalid diffusion blocks, and unsupported controller/actuator signal dependencies before integration starts.

## Documentation

The latest development documentation is available at the [ACES documentation site](https://pace-laboratory.github.io/ACES/dev/). It contains the canonical notation, subsystem theory, Julia code documentation, and simulation-assembly contract.

To build the complete Documenter.jl site locally:

```sh
julia --project=docs -e 'using Pkg; Pkg.instantiate()'
julia --project=docs docs/make.jl
```

The generated site is written to `docs/build/`. Because the site uses concrete HTML links, `docs/build/index.html` can be opened directly without a local web server.

## Testing

Run the package test suite with:

```sh
julia --project=. -e 'using Pkg; Pkg.test()'
```

## License

ACES is distributed under the GNU General Public License, version 3. See `LICENSE` for the full terms.
