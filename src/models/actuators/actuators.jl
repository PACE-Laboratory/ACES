# -----------------------------------------------------------------------------
# Actuator model definitions
# -----------------------------------------------------------------------------
#
# Extension point: aircraft-specific actuator dynamics may be defined in a user
# file by subtyping AbstractActuatorModel and implementing state_dimension,
# f_act, h_act, and (when appropriate) actuator_direct_feedthrough. Pass the
# instance as `actuator` to AircraftModel. Pass `actuator=nothing` when no
# actuator dynamics are modeled; core then enforces the documented direct
# relation δ = u.

"""
    actuator_direct_feedthrough(model)

Return `true` when `h_act` depends on the current control input `u`.

The simulation uses this trait to choose an acyclic signal order. The
conservative default is `true`: the controller output is evaluated first and
must not use `δ`. Models whose output is determined entirely by actuator state
should return `false`; their `δ` is evaluated before the controller and their
`h_act` implementation must not use `u`.
"""
actuator_direct_feedthrough(::AbstractActuatorModel) = true

"""Ideal actuator model with no state and direct feedthrough `δ = u`."""
struct DirectActuator <: AbstractActuatorModel end

state_dimension(::DirectActuator) = 0
actuator_direct_feedthrough(::DirectActuator) = true

"""
    FirstOrderActuator(τ, channels=1)
    FirstOrderActuator(τ_vector)

Independent first-order actuator channels with positive time constants.
"""
struct FirstOrderActuator{T<:Real} <: AbstractActuatorModel
    τ::Vector{T}
end

function FirstOrderActuator(τ::Real, channels::Integer=1)
    τ > 0 || throw(ArgumentError("τ must be positive"))
    channels > 0 || throw(ArgumentError("channels must be positive"))
    T = typeof(float(τ))
    return FirstOrderActuator{T}(fill(T(τ), channels))
end

function FirstOrderActuator(τ::AbstractVector{<:Real})
    isempty(τ) && throw(ArgumentError("τ must contain at least one channel"))
    all(>(0), τ) || throw(ArgumentError("all time constants must be positive"))
    T = float(eltype(τ))
    return FirstOrderActuator{T}(Vector{T}(τ))
end

state_dimension(model::FirstOrderActuator) = length(model.τ)
actuator_direct_feedthrough(::FirstOrderActuator) = false

# -----------------------------------------------------------------------------
# Actuator dynamics
# -----------------------------------------------------------------------------

"""Evaluate actuator state drift `f_act`."""
function f_act(
    ::DirectActuator,
    x_act::AbstractVector,
    v_r::AbstractVector,
    ω_r::AbstractVector,
    u::AbstractVector,
)
    isempty(x_act) || throw(DimensionMismatch("direct actuators have no state"))
    return empty_state(eltype(x_act))
end

function f_act(
    model::FirstOrderActuator,
    x_act::AbstractVector,
    v_r::AbstractVector,
    ω_r::AbstractVector,
    u::AbstractVector,
)
    n = state_dimension(model)
    length(x_act) == length(u) == n ||
        throw(DimensionMismatch("x_act and u must have length $n"))
    # Broadcasting permits a distinct time constant for every actuator channel.
    return (u - x_act) ./ model.τ
end

# -----------------------------------------------------------------------------
# Physical actuator outputs
# -----------------------------------------------------------------------------

"""Evaluate physical actuator output `h_act`."""
function h_act(
    ::DirectActuator,
    x_act::AbstractVector,
    v_r::AbstractVector,
    ω_r::AbstractVector,
    u::AbstractVector,
)
    isempty(x_act) || throw(DimensionMismatch("direct actuators have no state"))
    return copy(u)
end

function h_act(
    model::FirstOrderActuator,
    x_act::AbstractVector,
    v_r::AbstractVector,
    ω_r::AbstractVector,
    u::AbstractVector,
)
    length(x_act) == state_dimension(model) ||
        throw(DimensionMismatch("x_act has the wrong length"))
    return copy(x_act)
end
