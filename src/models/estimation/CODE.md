# Julia Code: Measurement and Estimation

This page documents measurement-process and estimator models. See the [estimation theory](../theory/estimation.md) for the continuous-time measurement and estimator equations.

## Built-In Measurement Models

`IdealMeasurement` evaluates a deterministic map of the complete extended state:

```julia
measurement = IdealMeasurement((x, t) -> measured_quantities(x, t))
```

`OrnsteinUhlenbeckMeasurement` adds a colored disturbance state to a nominal measurement:

```julia
measurement = OrnsteinUhlenbeckMeasurement(
    0.25,
    sqrt_R,
    (x, t) -> measured_quantities(x, t),
)
```

The nominal callable must return the same number of outputs as the row count of `sqrt_R`.

## Built-In Estimators

`IdentityEstimator` returns a copy of the measurement output. For general continuous-time estimators, `FunctionEstimator` accepts state dimension, drift, and output callables:

```julia
estimator = FunctionEstimator(
    n_est,
    (x_est, y_meas, x, t) -> estimator_drift(x_est, y_meas, x, t),
    (x_est, y_meas, x, t) -> estimator_output(x_est, y_meas, x, t),
)
```

## Defining Custom Models

A custom measurement type implements `state_dimension`, `noise_dimension`, `f_meas`, `σ_meas`, and `h_meas`. A custom estimator implements `state_dimension`, `f_est`, and `h_est`.

All measurement and estimator methods receive the complete extended state `x`. If a model needs actuator configuration or another derived signal, it owns the aircraft-specific map from `x` to that signal. Core does not pass `δ` as a separate measurement or estimator argument.

## Measurement Models and Interface

```@docs
ACES.AbstractMeasurementModel
ACES.IdealMeasurement
ACES.OrnsteinUhlenbeckMeasurement
ACES.f_meas
ACES.σ_meas
ACES.h_meas
```

## Estimator Models and Interface

```@docs
ACES.AbstractEstimator
ACES.IdentityEstimator
ACES.FunctionEstimator
ACES.f_est
ACES.h_est
```
