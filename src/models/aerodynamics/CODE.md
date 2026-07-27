# Julia Code: Aerodynamics

This page documents nominal aerodynamic models, stochastic load residuals, and derived fixed-wing air data. See the [aerodynamics theory](../theory/aerodynamics.md) for the force, moment, and residual equations.

## Composing Nominal and Residual Models

`Aerodynamics` combines one nominal model with an optional stochastic residual:

```julia
nominal = QuasiSteadyAerodynamics(force_function, moment_function)
aerodynamics = Aerodynamics(nominal)

residual = LinearAerodynamicResidual(A_res, L_res, G_res)
stochastic_aerodynamics = Aerodynamics(nominal, residual)
```

The nominal and residual models receive the same air-relative velocity, air-relative angular velocity, actuator configuration, and density. The residual output matrix has six rows: the first three perturb force and the last three perturb moment.

## Defining a Custom Nominal Model

Aircraft-specific aerodynamic models should normally be defined outside ACES:

```julia
struct MyAerodynamics <: ACES.AbstractAerodynamicModel
    # geometry, coefficient tables, and other aircraft data
end

ACES.state_dimension(model::MyAerodynamics) = n_aero
ACES.F_model(model::MyAerodynamics, v_r, ω_r, x_aero, δ, ρ) = ...
ACES.M_model(model::MyAerodynamics, v_r, ω_r, x_aero, δ, ρ) = ...
ACES.f_aero(model::MyAerodynamics, x_aero, v_r, ω_r, δ, ρ) = ...
```

`F_model` and `M_model` each return a body-frame three-vector. `f_aero` returns an `n_aero`-vector. For a stateless model, return zero from `state_dimension` and use `empty_state(eltype(x_aero))` for the drift.

## Defining a Custom Residual Model

A stochastic residual subtypes `AbstractAerodynamicResidualModel` and implements `state_dimension`, `noise_dimension`, `f_res`, `σ_res`, and `g_res`. The diffusion must have size `n_res × m_res`, and `g_res` must return a `6 × n_res` matrix.

## Model Types

```@docs
ACES.AbstractAerodynamicModel
ACES.AbstractAerodynamicResidualModel
ACES.Aerodynamics
ACES.QuasiSteadyAerodynamics
ACES.NoAerodynamicResidual
ACES.LinearAerodynamicResidual
```

## Nominal and Residual Interfaces

```@docs
ACES.F_model
ACES.M_model
ACES.f_aero
ACES.f_res
ACES.σ_res
ACES.g_res
ACES.aerodynamic_loads
```

## Derived Air Data

```@docs
ACES.fixed_wing_air_data
```
