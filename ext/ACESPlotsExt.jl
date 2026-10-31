module ACESPlotsExt

using ACES
import Plots

# Histories are evaluated exclusively at saved states, including repeated times
# from solver callbacks. Never invoke solution(t), solve, or a random sampler.
function _saved_history(solution)
    length(solution.t) == length(solution.u) ||
        throw(DimensionMismatch("saved times and states must have equal lengths"))
    isempty(solution.t) && throw(ArgumentError("solution must contain saved states"))
    return solution.t, solution.u
end

function _channel_matrix(values, name)
    n = length(first(values))
    all(value -> value isa AbstractVector && length(value) == n, values) ||
        throw(DimensionMismatch("$name must have a constant channel count"))
    # Keep the sample count out of both array types and vararg call signatures.
    # In particular, hcat of static vectors builds a history-sized SMatrix.
    T = mapreduce(eltype, promote_type, values)
    channels = Matrix{T}(undef, length(values), n)
    for (sample, value) in enumerate(values)
        for (channel, entry) in enumerate(value)
            channels[sample, channel] = entry
        end
    end
    return channels
end

function ACES.plot_rigid_body(model::AircraftModel, solution; kwargs...)
    rb = model.rigid_body
    rb isa RigidBody && rb.attitude isa Union{EulerAngles,UnitQuaternion} ||
        throw(ArgumentError("plot_rigid_body supports built-in Euler-angle and quaternion RigidBody models"))
    t, x = _saved_history(solution)
    states = [state_components(model, saved).x_rb for saved in x]
    body = [state_view(rb, saved) for saved in states]
    attitude = [Θ(saved.R_IB) for saved in body]
    histories = (
        _channel_matrix([saved.s for saved in body], "position"),
        _channel_matrix(attitude, "attitude"),
        _channel_matrix([saved.v for saved in body], "body velocity"),
        _channel_matrix([saved.ω for saved in body], "body angular velocity"),
    )
    labels = (
        ["North", "East", "Down"],
        ["Roll φ", "Pitch θ", "Yaw ψ"],
        ["v₁", "v₂", "v₃"],
        ["ω₁", "ω₂", "ω₃"],
    )
    titles = ("Position (NED)", "Attitude", "Body translational velocity", "Body angular velocity")
    units = ("m", "rad", "m/s", "rad/s")
    p = Plots.plot(; layout=(2, 2), size=(1000, 700), link=:x,
                   xlabel="Time (s)", kwargs..., show=false)
    for panel in eachindex(histories)
        Plots.plot!(p, t, histories[panel]; subplot=panel,
                    label=permutedims(labels[panel]), title=titles[panel],
                    ylabel=units[panel], show=false)
    end
    return p
end

function _channel_labels(labels, n, prefix)
    isnothing(labels) && return ["$prefix[$i]" for i in 1:n]
    labels isa Union{AbstractVector,Tuple} ||
        throw(ArgumentError("$prefix labels must be a collection of channel names"))
    length(labels) == n ||
        throw(DimensionMismatch("expected $n labels for $prefix"))
    return string.(collect(labels))
end

function _channel_units(units, n, prefix)
    isnothing(units) && return fill("", n)
    units isa AbstractString && return fill(units, n)
    units isa Union{AbstractVector,Tuple} && all(unit -> unit isa AbstractString, units) ||
        throw(ArgumentError("$prefix units must be a string or a collection of strings"))
    length(units) == n ||
        throw(DimensionMismatch("expected $n units for $prefix"))
    return collect(units)
end

function ACES.plot_controls(
    model::AircraftModel, solution;
    actuator_outputs::Bool=false, labels=nothing, units=nothing,
    actuator_labels=nothing, actuator_units=nothing, kwargs...,
)
    t, x = _saved_history(solution)
    # Copy each output immediately: user callbacks can reuse output buffers.
    signals = map(zip(x, t)) do (saved, time)
        output = simulation_signals(model, saved, time)
        (u=copy(output.u), δ=actuator_outputs ? copy(output.δ) : nothing)
    end
    commands = _channel_matrix([saved.u for saved in signals], "controller commands")
    n_u = size(commands, 2)
    names = _channel_labels(labels, n_u, "u")
    channel_units = _channel_units(units, n_u, "u")
    histories = commands
    kinds = fill("Command", n_u)
    if actuator_outputs
        outputs = _channel_matrix([saved.δ for saved in signals], "actuator outputs")
        n_δ = size(outputs, 2)
        names = vcat(names, _channel_labels(actuator_labels, n_δ, "δ"))
        channel_units = vcat(channel_units, _channel_units(actuator_units, n_δ, "δ"))
        histories = hcat(histories, outputs)
        kinds = vcat(kinds, fill("Actuator output", n_δ))
    end
    n = size(histories, 2)
    title = actuator_outputs ? "Controller commands and actuator outputs" : "Controller commands"
    p = Plots.plot(; layout=(max(n, 1), 1), size=(800, max(250, 200n)),
                   link=:x, xlabel="Time (s)", legend=false, title=title,
                   kwargs..., show=false)
    for channel in 1:n
        unit = channel_units[channel]
        guide = isempty(unit) ? names[channel] : "$(names[channel]) ($unit)"
        Plots.plot!(p, t, histories[:, channel]; subplot=channel,
                    label=names[channel], title="$(kinds[channel]): $(names[channel])",
                    ylabel=guide, show=false)
    end
    return p
end

end
