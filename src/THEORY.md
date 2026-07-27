# Theory: Simulation Assembly

Symbols and Julia identifiers follow the [canonical notation](@ref notation). This page defines how the subsystem models are composed into the numerical simulation implemented by `core.jl`. The subsystem theory pages define the individual maps used below.

## Model Composition

An aircraft simulation selects one concrete model for each of the following subsystems:

- rigid-body dynamics;
- environment;
- wind;
- nominal and residual aerodynamics;
- optional actuator dynamics;
- measurement;
- estimation; and
- control.

The selected model instances and their parameters are fixed during a simulation. Aircraft-specific models may be defined outside ACES as long as they implement the corresponding abstract subsystem interface.

## Extended Simulation State

The numerical integrator advances one flat extended state,

```math
\bm{x} = \begin{bmatrix}
    \bm{x}_\mathrm{rb} \\
    \bm{x}_\mathrm{aero} \\
    \bm{x}_\mathrm{act} \\
    \bm{x}_\mathrm{est} \\
    \bm{x}_\mathrm{ctrl} \\
    \bm{x}_w \\
    \bm{x}_\mathrm{res} \\
    \bm{x}_\mathrm{meas}
\end{bmatrix} \in \mathbb{R}^{n_x}
```

The block order is invariant. The deterministic rigid-body, nominal-aerodynamic, actuator, estimator, and controller states precede the stochastic wind, aerodynamic-residual, and measurement states. A subsystem with zero state dimension occupies an empty range and contributes no entries to $\bm{x}$. If no actuator model is selected, $n_\mathrm{act}=0$ and $\bm{x}_\mathrm{act}$ is empty.

The total state dimension is

```math
n_x = n_\mathrm{rb} + n_\mathrm{aero} + n_\mathrm{act} + n_\mathrm{est} + n_\mathrm{ctrl} + n_w + n_\mathrm{res} + n_\mathrm{meas}
```

`state_layout(model)` supplies the ranges associated with these blocks, `initial_state(model; ...)` assembles them, and `state_components(model, x)` returns non-copying views.

## Derived Signal Evaluation

At a state $\bm{x}$ and time $t$, the simulation evaluates derived signals in the following dependency order.

First, the rigid-body state is decoded and the environment and wind outputs are evaluated:

```math
\begin{aligned}
    (\bm{g},\rho) &= \bm{h}_\mathrm{env}(\bm{s},t) \\
    \bm{\varpi} &= \bm{h}_w(\bm{x}_w,\bm{x}_\mathrm{rb},t)
\end{aligned}
```

The wind triangle then gives the body-frame air-relative velocities:

```math
\begin{aligned}
    \bm{v}_r &= \bm{v}-\bm{R}_\mathrm{IB}^{\mathsf{T}}\bm{w} \\
    \bm{\omega}_r &= \bm{\omega}-\bm{\omega}_w
\end{aligned}
```

Measurement and estimation outputs depend on the complete extended state:

```math
\begin{aligned}
    \bm{y}_\mathrm{meas} &= \bm{h}_\mathrm{meas}(\bm{x}_\mathrm{meas},\bm{x},t) \\
    \hat{\bm{x}} &= \bm{h}_\mathrm{est}(\bm{x}_\mathrm{est},\bm{y}_\mathrm{meas},\bm{x},t)
\end{aligned}
```

Any application-specific mapping from $\bm{x}$ to a known estimator signal, including a reconstructed actuator configuration, is implemented inside the measurement or estimator model.

## Controller and Actuator Causality

Controller and actuator methods always retain their complete interface signatures. In particular, ACES calls `h_ctrl(model, x_ctrl, x_rb, δ, ϖ, xhat, t)` and `h_act(model, x_act, v_r, ω_r, u)` in every applicable case. Saying that a method must not *depend* on an argument means that the method must ignore that argument; it does not mean that the argument is removed from the call.

ACES evaluates each controller and actuator output once per simulation state and does not solve simultaneous direct-feedthrough equations. The required evaluation order and dependency assumptions are:

1. **No actuator model:** ACES evaluates `h_ctrl` first and then applies the ideal relation $\bm{\delta}=\bm{u}$. Both `h_ctrl` and `f_ctrl` must ignore their $\bm{\delta}$ argument.
2. **Actuator with direct feedthrough:** ACES evaluates `h_ctrl` to obtain $\bm{u}$ and then evaluates `h_act` to obtain $\bm{\delta}$. Because $\bm{\delta}$ is not yet available when `h_ctrl` is called, `h_ctrl` must ignore its $\bm{\delta}$ argument. After both outputs are known, `f_ctrl` may depend on the resulting $\bm{\delta}$.
3. **Actuator without direct feedthrough:** ACES evaluates `h_act` to obtain $\bm{\delta}$ and then evaluates `h_ctrl` to obtain $\bm{u}$. Because $\bm{u}$ is not yet available when `h_act` is called, `h_act` must ignore its $\bm{u}$ argument. Both `h_ctrl` and `f_ctrl` may depend on the resulting $\bm{\delta}$. This case includes the first-order actuator model for which $\bm{\delta}=\bm{x}_\mathrm{act}$.

Once $\bm{u}$ and $\bm{\delta}$ are known, the actuator and controller drift functions are evaluated for the assembled state derivative. Unavailable-signal sentinels enforce the dependency assumptions above during validation and simulation.

## Aerodynamic Loads

After $\bm{\delta}$ is known, nominal and residual aerodynamic loads are evaluated at the common flow and density conditions:

```math
\begin{aligned}
    \bm{F} &= \bm{F}_\mathrm{model}(\bm{v}_r,\bm{\omega}_r,\bm{x}_\mathrm{aero},\bm{\delta},\rho) + \delta\bm{F} \\
    \bm{M} &= \bm{M}_\mathrm{model}(\bm{v}_r,\bm{\omega}_r,\bm{x}_\mathrm{aero},\bm{\delta},\rho) + \delta\bm{M}
\end{aligned}
```

These loads and the gravitational acceleration $\bm{g}$ drive the rigid-body dynamics.

## Assembled Drift

The deterministic part of the extended-state dynamics is the block vector

```math
\bm{f}(\bm{x},t) = \begin{bmatrix}
    \bm{f}_\mathrm{rb} \\
    \bm{f}_\mathrm{aero} \\
    \bm{f}_\mathrm{act} \\
    \bm{f}_\mathrm{est} \\
    \bm{f}_\mathrm{ctrl} \\
    \bm{f}_w \\
    \bm{f}_\mathrm{res} \\
    \bm{f}_\mathrm{meas}
\end{bmatrix}
```

When no actuator model is selected, the actuator block is empty. All other blocks retain the same order as the corresponding state blocks.

## Assembled Diffusion

Only the wind, aerodynamic-residual, and measurement models contribute Wiener process inputs. Their diffusion matrices occupy separate row and column blocks:

```math
\bm{G}(\bm{x},t) = \begin{bmatrix}
    \bm{0} & \bm{0} & \bm{0} \\
    \bm{0} & \bm{0} & \bm{0} \\
    \bm{0} & \bm{0} & \bm{0} \\
    \bm{0} & \bm{0} & \bm{0} \\
    \bm{0} & \bm{0} & \bm{0} \\
    \bm{\sigma}_w & \bm{0} & \bm{0} \\
    \bm{0} & \bm{\sigma}_\mathrm{res} & \bm{0} \\
    \bm{0} & \bm{0} & \bm{\sigma}_\mathrm{meas}
\end{bmatrix}
```

The column blocks correspond respectively to $\bm{W}_w$, $\bm{W}_\mathrm{res}$, and $\bm{W}_\mathrm{meas}$. These Wiener processes are independent unless a user-defined subsystem embeds a different correlation structure in its own diffusion matrix.

If every selected subsystem has zero noise dimension, the assembled problem is the ODE

```math
\mathrm{d}\bm{x}=\bm{f}(\bm{x},t)\mathrm{d}t.
```

Otherwise, the assembled problem is the SDE

```math
\mathrm{d}\bm{x} = \bm{f}(\bm{x},t)\mathrm{d}t + \bm{G}(\bm{x},t) \begin{bmatrix}
    \mathrm{d}\bm{W}_w \\
    \mathrm{d}\bm{W}_\mathrm{res} \\
    \mathrm{d}\bm{W}_\mathrm{meas}
\end{bmatrix}
```

`simulation_problem` selects the ODE or SDE representation from the total noise dimension.

## Preflight Validation

Before constructing a numerical problem, `validate(model, x0)` evaluates the assembled model at the initial state. Validation requires:

- nonnegative integer state and noise dimensions;
- an initial state with the assembled dimension and finite real entries;
- finite drift and diffusion outputs with the required block dimensions; and
- satisfaction of the controller/actuator causality restrictions above.

Validation catches interface and dimension errors before they are nested inside a numerical-solver stack trace. It verifies the model at the supplied initial state; it cannot guarantee that a user-defined model remains valid at every future state.
