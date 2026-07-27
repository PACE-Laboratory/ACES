# Julia Code: Wind

This page documents deterministic wind fields, stochastic shaping filters, and the wind triangle. See the [wind theory](../theory/wind.md) for frame conventions and the generalized wind definition.

## Built-In Models

Use `ConstantWind` for spatially uniform translational and angular wind:

```julia
wind = ConstantWind([5.0, 0.0, 0.0], zeros(3))
```

Use `FrozenWindField` when inertial-frame wind and its spatial gradient are available as functions:

```julia
wind = FrozenWindField(
    (s, t) -> wind_velocity(s, t),
    (s, t) -> wind_gradient(s, t),
)
```

`ShapingFilterWind` represents continuous-time stochastic turbulence. Its matrix dimensions define both the wind-state dimension and Wiener-process dimension.

## Defining a Custom Wind Model

A user-defined model subtypes `AbstractWindModel` and implements:

```julia
ACES.state_dimension(model::MyWind) = n_w
ACES.noise_dimension(model::MyWind) = m_w
ACES.f_w(model::MyWind, x_w, x_rb, t) = ...
ACES.σ_w(model::MyWind, x_w, x_rb, t) = ...
ACES.h_w(model::MyWind, x_w, x_rb, t) = ...
```

`f_w` returns an `n_w`-vector, `σ_w` returns an `n_w × m_w` matrix, and `h_w` returns the six-vector `[w; ω_w]`. The decoded rigid-body argument exposes `s`, `R_IB`, `v`, and `ω`. A deterministic model may omit `noise_dimension` because its default is zero, but it must still provide a correctly sized zero-column diffusion when the method is called.

## Model Types

```@docs
ACES.AbstractWindModel
ACES.ConstantWind
ACES.FrozenWindField
ACES.ShapingFilterWind
```

## Wind Interface

```@docs
ACES.f_w
ACES.σ_w
ACES.h_w
ACES.wind_triangle
```

## Gradient Utilities

```@docs
ACES.body_wind_gradient
ACES.wind_angular_velocity
ACES.fixed_wing_angular_velocity
```
