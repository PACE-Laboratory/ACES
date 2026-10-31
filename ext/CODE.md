# Julia Code: Optional Plotting

The `ACESPlotsExt` package extension implements `plot_rigid_body` and `plot_controls`. Plots is a weak dependency: installing or loading ACES for simulation does not require Plots. The public entry points and docstrings live in `src/plotting.jl`; calling them before loading Plots raises an `ArgumentError` explaining how to enable plotting.

## Loading and Saving

Install Plots in the environment used for plotting, then load both packages in either order:

```julia
using ACES, Plots

body_plot = plot_rigid_body(model, solution; size=(1100, 750))
controls_plot = plot_controls(model, solution)

# Explicit customization, display, and saving remain under the caller's control.
plot!(body_plot; linewidth=2)
savefig(body_plot, "rigid-body.png")
```

Both functions return a `Plots.Plot`. They do not call `display` or `savefig`, and force `show=false` even when a global Plots default or supplied keyword requests display. Other keyword arguments are passed to the figure constructor; panel titles, units, and series labels are set when the histories are added. Use standard Plots operations on the returned object for further customization.

## Saved Histories

The solution must expose nonempty `t` and `u` collections with equal lengths, containing saved times in seconds and complete simulation states. Both functions use those samples directly, preserving saved time order and repeated timestamps. They do not interpolate, rerun integration, evaluate drift or diffusion functions, or draw random samples. Invalid histories raise `ArgumentError` for an empty history or `DimensionMismatch` for unequal lengths.

Private helpers in `ext/ACESPlotsExt.jl` validate the saved history, collect channel matrices, and validate labels and units. Channel assembly allocates an ordinary dense `Matrix` with one row per saved sample and one column per channel, promoting element types across samples. It copies static or dynamic vectors with indexed loops in time and storage proportional to the number of saved channel values; the history length is never encoded in a static array type or a splatted concatenation call. Both plotting helpers retain every sample without downsampling. These helpers are internal to the extension; no new public history type is introduced.

## Rigid-Body Histories

`plot_rigid_body(model, solution; kwargs...)` supports built-in `RigidBody` models with `EulerAngles` or `UnitQuaternion` attitudes. It uses `state_components` to locate the rigid-body block and `state_view` to decode position and body velocities. Native attitude coordinates are taken from the pose portion of that rigid-body block using `pose_dimension`, without assuming indices in the complete simulation state.

The four default panels show:

- Inertial position: North, East, and Down in meters; Down retains its positive-down sign.
- Native attitude: roll, pitch, and yaw in radians for Euler models, or scalar-first `q₀`, `q₁`, `q₂`, `q₃` for quaternion models with dimensionless unit `1`.
- Inertial translational velocity expressed in body axes, in meters per second.
- Body angular velocity expressed in body axes, in radians per second.

These follow the [rigid-body theory](../theory/rigid-body.md) and [canonical notation](@ref notation). Attitude samples are plotted as saved, without angle unwrapping, quaternion normalization, or quaternion sign changes. The default layout is a two-by-two grid with time in seconds on the horizontal axes. Custom rigid-body models or attitude representations raise `ArgumentError`.

## Control and Actuator Histories

`plot_controls` evaluates `simulation_signals(model, x, t)` once at each saved pair `(x, t)` and copies the resulting controller commands `u`. Set `actuator_outputs=true` to copy and append panels for actuator outputs `δ` from the same evaluations:

```julia
controls_plot = plot_controls(
    model, solution;
    actuator_outputs=true,
    labels=["Pitch command", "Throttle command"],
    units=["rad", "1"],
    actuator_labels=["Elevator", "Throttle"],
    actuator_units=["rad", "1"],
)
```

Labels in this example describe an application with two channels; no aircraft-specific labels or channel counts are built into ACES. By default, labels are `u[1]`, `u[2]`, … and `δ[1]`, `δ[2]`, …. Commands and actuator outputs have independent channel counts and metadata. Each channel gets a separate panel so units can differ between channels and between commands and physical outputs. Without actuator dynamics, the outputs satisfy the assembly's ideal relation `δ = u`.

`labels` and `actuator_labels` accept a vector or tuple with exactly one entry per corresponding channel. `units` and `actuator_units` accept a string for all corresponding channels, or a vector or tuple of strings with one entry per channel. Units affect labels only and do not convert values; unspecified units are omitted. Metadata with the wrong length raises `DimensionMismatch`; unsupported metadata types raise `ArgumentError`. Each output's channel count must remain constant across samples, or `DimensionMismatch` is raised. A zero-channel model returns an empty figure.

Commands are recomputed from the saved state, time, and model through the assembly interface, never inferred from controller states. Custom output callbacks must be deterministic functions of their inputs and use any saved stochastic state rather than drawing fresh noise. Model parameters and callbacks must match those used during simulation for the reconstructed commands to represent that run.

## Plotting API

```@docs
ACES.plot_rigid_body
ACES.plot_controls
```
