"""
    plot_rigid_body(model, solution; kwargs...)

Plot saved rigid-body position, attitude, body translational velocity, and body
angular velocity. Load `Plots` to enable this optional extension.

For built-in `RigidBody` models, attitude always shows roll, pitch, and yaw in
radians, computed by `Θ` from the decoded body-to-inertial rotation for both
`EulerAngles` and `UnitQuaternion` states. The principal branch has pitch in
[-π/2, π/2] and roll/yaw in [-π, π], without temporal unwrapping. At gimbal lock,
yaw is zero and roll represents the remaining rotation. Saved states are not
modified. Position uses North-East-Down coordinates in meters, body velocity
uses m/s, angular velocity uses rad/s, and time uses seconds.

`solution` must expose nonempty, equally sized `t` and `u` collections of saved
times and complete simulation states. No integration or interpolation occurs.
Return a `Plots.Plot` with four panels; `kwargs` customize the figure using
Plots attributes. The function does not display the plot or save files.
"""
plot_rigid_body(model, solution; kwargs...) =
    throw(ArgumentError("load Plots with `using Plots` to enable plot_rigid_body"))

"""
    plot_controls(model, solution; actuator_outputs=false, labels=nothing,
                  units=nothing, actuator_labels=nothing,
                  actuator_units=nothing, kwargs...)

Plot controller commands `u` at saved times. Set `actuator_outputs=true` to
append panels for physical actuator outputs `δ`. Load `Plots` to enable this
optional extension. Each channel has its own panel; channel counts are inferred
from `simulation_signals`, independently for commands and actuator outputs.

`labels` and `actuator_labels` are collections with one name per corresponding
channel; defaults are `u[i]` and `δ[i]`. `units` and `actuator_units` accept a
unit string for all corresponding channels or a collection of unit strings.
Units are labels only: values are not converted. Unspecified units are omitted.

`solution` must expose nonempty, equally sized `t` and `u` collections of saved
times (seconds) and complete simulation states. `simulation_signals` evaluates
each saved state once; it does not integrate, interpolate, or sample noise.
Custom output callbacks must be deterministic functions of their saved inputs.
Channel counts must remain constant throughout the history.

Return a `Plots.Plot` for customization and `Plots.savefig`; `kwargs` are
forwarded as Plots figure attributes. No windows are opened or files written.
"""
plot_controls(model, solution; kwargs...) =
    throw(ArgumentError("load Plots with `using Plots` to enable plot_controls"))
