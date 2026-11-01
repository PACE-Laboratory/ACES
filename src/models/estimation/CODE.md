# Julia Code: Measurement and Estimation

This page documents measurement-process and estimator models. See the [estimation theory](../theory/estimation.md) for the continuous-time measurement and estimator equations.

The model types are exported. The evaluation and extension interfaces `f_meas`, `σ_meas`, `h_meas`, `f_est`, and `h_est` are public but unexported; use `ACES.` qualification or import individual names explicitly.

## Built-In Measurement Models

`IdealMeasurement` evaluates a deterministic map of the complete extended state:

```julia
measurement = IdealMeasurement((x, t) -> measured_quantities(x, t))
```

For ideal measurement of just the rigid-body state, use `measurement=IdealMeasurement(:rigid_body)` in the keyword `AircraftModel` constructor. The constructor resolves the selector using its own `state_layout`, so the rigid-body model can be constructed inline and no callback, dimension lookup, or explicit copy is needed. The output retains the raw integration coordinates (including the selected attitude representation), excludes all other subsystem states, and is a view into the supplied simulation state; consumers must treat it as read-only. `IdentityEstimator` continues to return an independent copy. Other symbol selectors throw `ArgumentError` immediately.

The selector is a configuration request until aircraft construction. For standalone evaluation, call `h_meas(aircraft.measurement, x_meas, x, t)` on the resolved measurement; calling `h_meas` on an unresolved selector throws `ArgumentError`. A resolved measurement is bound to that aircraft's layout; use a fresh `IdealMeasurement(:rigid_body)` request when constructing an aircraft with a different layout. A state shorter than the resolved rigid-body range throws `DimensionMismatch`. The existing callable form retains its callable's output and aliasing behavior unchanged.

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
