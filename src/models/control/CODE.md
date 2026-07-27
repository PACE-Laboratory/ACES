# Julia Code: Control

This page documents static, dynamic, and function-defined controllers. See the [control theory](../theory/control.md) for the controller equations and the [simulation theory](../theory/simulation.md#Controller-and-Actuator-Causality) for controller/actuator evaluation order.

## Built-In Controllers

`LinearStateFeedback` is stateless and acts on the estimator output:

```julia
controller = LinearStateFeedback(K, u_ref, x_ref)
```

`LinearOutputFeedback` carries a dynamic controller state. Its optional `measurement` callback maps `(x_rb, δ, ϖ, xhat, t)` to the feedback quantity used by the state-space controller:

```julia
controller = LinearOutputFeedback(
    A_ctrl,
    B_ctrl,
    C_ctrl,
    D_ctrl,
    u_ref,
    y_ref;
    measurement=(x_rb, δ, ϖ, xhat, t) -> xhat,
)
```

`FunctionController` supports an arbitrary continuous-time controller without defining a new Julia type:

```julia
controller = FunctionController(
    n_ctrl,
    (x_ctrl, x_rb, δ, ϖ, xhat, t) -> controller_drift(...),
    (x_ctrl, x_rb, δ, ϖ, xhat, t) -> control_output(...),
)
```

## Defining a Custom Controller

A custom controller subtypes `AbstractController` and implements `state_dimension`, `f_ctrl`, and `h_ctrl`. Both interface functions retain the full argument list `(model, x_ctrl, x_rb, δ, ϖ, xhat, t)`.

The selected actuator configuration determines which arguments may be used:

- With `actuator=nothing`, both `f_ctrl` and `h_ctrl` must ignore `δ`.
- With a direct-feedthrough actuator, `h_ctrl` must ignore `δ`; `f_ctrl` may use the actuator output after it is evaluated.
- With an actuator without direct feedthrough, both controller methods may use `δ`.

ACES supplies an unavailable-signal sentinel when an argument must be ignored, so an invalid dependency fails during validation.

## Controller Models

```@docs
ACES.AbstractController
ACES.LinearStateFeedback
ACES.LinearOutputFeedback
ACES.FunctionController
```

## Controller Interface

```@docs
ACES.f_ctrl
ACES.h_ctrl
```
