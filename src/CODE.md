# Julia Code: Simulation Assembly

This page documents the Julia interface that combines subsystem models into a simulation. See the [simulation assembly theory](../theory/simulation.md) for the state ordering, signal dependencies, and assembled ODE/SDE equations.

```@docs
ACES
```

## Constructing an Aircraft Model

`AircraftModel` stores one concrete implementation of every subsystem. Model selection is explicit: construct the subsystem instances and pass them to the keyword constructor.

```julia
model = AircraftModel(
    rigid_body=RigidBody(m, inertia),
    environment=Environment(),
    wind=ConstantWind(),
    aerodynamics=Aerodynamics(nominal_aerodynamics),
    actuator=nothing,
    measurement=IdealMeasurement(:rigid_body),
    estimator=estimator_model,
    controller=controller_model,
)
```

Set `actuator=nothing` when the ideal relation `δ = u` should be used without an actuator state. In that configuration, both controller methods must ignore their `δ` argument.

The keyword constructor resolves `IdealMeasurement(:rigid_body)` from this aircraft's `state_layout` during construction, including Euler, quaternion, and custom rigid-body state dimensions. The resulting measurement returns a view of the raw rigid-body coordinates, which consumers must treat as read-only; `IdentityEstimator` returns a copy. Other measurement models and `IdealMeasurement(h)` callbacks pass through unchanged. See the [measurement documentation](estimation.md) for standalone evaluation and aliasing details.

`simulation_signals(model, x, t)` evaluates the environment through `environment(model.environment, rb_state.s)`, using only the current inertial position. Custom environment models must implement this two-argument interface and return `(g=..., ρ=...)`. Environment outputs have no explicit time dependence; they can change as the aircraft position changes.

## State Assembly

`state_layout(model)` reports the range occupied by each subsystem in the flat integration state. Use `initial_state` to assemble initial conditions and `state_components` to recover non-copying views:

```julia
x0 = initial_state(
    model;
    x_rb=x_rb0,
    x_aero=x_aero0,
    x_act=x_act0,
)

layout = state_layout(model)
states = state_components(model, x0)
```

Keywords for zero-dimensional states may be omitted. Keywords for nonzero-dimensional optional states default to zero vectors of the required length.

## Validation and Simulation

Call `validate(model, x0)` before a simulation when constructing or debugging a custom model. `simulation_problem` and `simulate` perform the same initial preflight automatically.

`simulation_problem` returns an `ODEProblem` when the total noise dimension is zero and an `SDEProblem` otherwise. With no explicit `solver`, `simulate` uses SciML's standard automatic ODE algorithm for deterministic problems and adaptive Euler-Maruyama for general-noise stochastic problems. An explicit solver and all other solver keywords are forwarded to SciML:

```julia
validate(model, x0)
solution = simulate(model, x0, (0.0, 10.0); abstol=1e-9, reltol=1e-9)
```

The in-place drift and diffusion functions are public for advanced workflows such as constructing a customized SciML problem. Most users should prefer `simulation_problem` or `simulate`.

## Saved Results

Plotting belongs to downstream aircraft projects. ACES has no plotting API or plotting dependencies. Use `state_components(model, x)` and `state_view(model.rigid_body, x_rb)` to decode saved states, `Θ(rb.R_IB)` for principal Euler angles, and `simulation_signals(model, x, t)` to recover controller commands and actuator outputs at saved times. These public data interfaces support the downstream project's choice of plotting backend; no integration or interpolation is needed to inspect saved histories.

## Shared Model Interface

Every stateful subsystem implements `state_dimension`. Stochastic wind, aerodynamic-residual, and measurement models also implement `noise_dimension`. A stateless implementation should return zero and may use `empty_state` for its drift.

```@docs
ACES.RealT
ACES.Vec3
ACES.Vec4
ACES.Vec6
ACES.Mat3
ACES.empty_state
ACES.state_dimension
ACES.noise_dimension
```

## Composition and State API

```@docs
ACES.AircraftModel
ACES.StateLayout
ACES.state_layout
ACES.state_components
ACES.initial_state
ACES.simulation_signals
ACES.validate
```

## Numerical Problem API

```@docs
ACES.simulation_drift!
ACES.simulation_diffusion!
ACES.simulation_problem
ACES.simulate
```
