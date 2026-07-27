# Julia Code: Actuators

This page documents ideal, first-order, and custom actuator models. See the [actuator theory](../theory/actuators.md) and the controller/actuator causality section of the [simulation theory](../theory/simulation.md#Controller-and-Actuator-Causality) for the signal-order contract.

## Selecting Actuator Behavior

There are three common configurations:

```julia
# No actuator state; core applies δ = u.
actuator = nothing

# Explicit stateless actuator model with δ = u.
actuator = DirectActuator()

# One or more first-order channels with δ = x_act.
actuator = FirstOrderActuator(0.1, 4)
```

`nothing` and `DirectActuator()` both produce an ideal input-output relation, but they have different controller causality contracts. With `nothing`, both controller methods must ignore `δ`. `DirectActuator` has direct feedthrough, so `h_ctrl` must ignore `δ`, while `f_ctrl` may use the resulting actuator output.

## Defining a Custom Actuator Model

```julia
struct MyActuator <: ACES.AbstractActuatorModel
    # actuator parameters
end

ACES.state_dimension(model::MyActuator) = n_act
ACES.actuator_direct_feedthrough(model::MyActuator) = false
ACES.f_act(model::MyActuator, x_act, v_r, ω_r, u) = ...
ACES.h_act(model::MyActuator, x_act, v_r, ω_r, u) = ...
```

`f_act` returns an `n_act`-vector and `h_act` returns the physical actuator configuration `δ`. The default direct-feedthrough trait is `true`. Return `false` only when `h_act` can be evaluated without inspecting the current `u`; ACES enforces this assumption with an unavailable-signal sentinel.

## Model Types and Causality Trait

```@docs
ACES.AbstractActuatorModel
ACES.DirectActuator
ACES.FirstOrderActuator
ACES.actuator_direct_feedthrough
```

## Actuator Interface

```@docs
ACES.f_act
ACES.h_act
```
