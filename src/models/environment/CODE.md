# Julia Code: Environment

This page documents gravity, atmospheric-density, and composite environment models. See the [environment theory](../theory/environment.md) for the NED altitude convention and atmosphere equations.

## Built-In Models

The default `Environment()` combines uniform positive-down gravity with the NASA metric atmosphere:

```julia
default_environment = Environment()

custom_environment = Environment(
    UniformGravity(9.80665),
    ExponentialDensity(1.225, 0.0, 8_500.0);
    h0=250.0,
)
```

The `h0` keyword is the mean-sea-level altitude of the NED-frame origin. Calling `environment(model, s, t)` returns a named tuple `(g=..., ρ=...)`.

## Defining Custom Environment Models

A complete application-specific environment can implement the top-level interface directly:

```julia
struct TabulatedEnvironment <: ACES.AbstractEnvironmentModel
    # tables and interpolation data
end

ACES.environment(model::TabulatedEnvironment, s, t) = (
    g=...,
    ρ=...,
)
```

The returned `g` must be a three-vector expressed in the inertial frame, and `ρ` must be a nonnegative scalar.

Reusable gravity or density components can instead subtype `AbstractGravityModel` or `AbstractDensityModel` and extend `gravity` or `density`. They can then be composed with `Environment`.

## Model Types

```@docs
ACES.AbstractEnvironmentModel
ACES.AbstractGravityModel
ACES.AbstractDensityModel
ACES.Environment
ACES.UniformGravity
ACES.ConstantDensity
ACES.ExponentialDensity
ACES.NASAMetricAtmosphere
```

## Evaluation API

```@docs
ACES.altitude
ACES.gravity
ACES.density
ACES.environment
```
