# -----------------------------------------------------------------------------
# Controller model definitions
# -----------------------------------------------------------------------------
#
# Extension point: an aircraft-specific controller can live in the end user's
# workspace and subtype AbstractController. Implement state_dimension, f_ctrl,
# and h_ctrl, then pass its instance as `controller` to AircraftModel. Keep only
# general controller architectures in this package file.
#
# If AircraftModel is constructed with `actuator=nothing`, both controller
# methods must ignore their `δ` argument. Core passes an unavailable sentinel
# in that case, so attempting to read `δ` produces a clear ArgumentError.

"""
Linear static feedback `u = u_ref - K*(xhat - x_ref)`.

The general controller interface uses the estimator output `xhat` as the
feedback state.
"""
struct LinearStateFeedback{T<:Real} <: AbstractController
    K::Matrix{T}
    u_ref::Vector{T}
    x_ref::Vector{T}
end

function LinearStateFeedback(
    K::AbstractMatrix{<:Real},
    u_ref::AbstractVector{<:Real},
    x_ref::AbstractVector{<:Real},
)
    size(K) == (length(u_ref), length(x_ref)) ||
        throw(DimensionMismatch("K must be length(u_ref) × length(x_ref)"))
    T = promote_type(float(eltype(K)), float(eltype(u_ref)), float(eltype(x_ref)))
    return LinearStateFeedback{T}(Matrix{T}(K), Vector{T}(u_ref), Vector{T}(x_ref))
end

state_dimension(::LinearStateFeedback) = 0

"""
Linear dynamic output-feedback controller.

`measurement(x_rb, δ, ϖ, xhat, t)` supplies `y`; by default it returns
`xhat`.
"""
struct LinearOutputFeedback{T<:Real,H} <: AbstractController
    A_ctrl::Matrix{T}
    B_ctrl::Matrix{T}
    C_ctrl::Matrix{T}
    D_ctrl::Matrix{T}
    u_ref::Vector{T}
    y_ref::Vector{T}
    measurement::H
end

function LinearOutputFeedback(
    A_ctrl::AbstractMatrix{<:Real},
    B_ctrl::AbstractMatrix{<:Real},
    C_ctrl::AbstractMatrix{<:Real},
    D_ctrl::AbstractMatrix{<:Real},
    u_ref::AbstractVector{<:Real},
    y_ref::AbstractVector{<:Real};
    measurement=(x_rb, δ, ϖ, xhat, t) -> xhat,
)
    n = size(A_ctrl, 1)
    p = length(u_ref)
    n_y = length(y_ref)
    size(A_ctrl, 2) == n || throw(DimensionMismatch("A_ctrl must be square"))
    size(B_ctrl) == (n, n_y) ||
        throw(DimensionMismatch("B_ctrl must be $n × $n_y"))
    size(C_ctrl) == (p, n) ||
        throw(DimensionMismatch("C_ctrl must be $p × $n"))
    size(D_ctrl) == (p, n_y) ||
        throw(DimensionMismatch("D_ctrl must be $p × $n_y"))
    T = promote_type(
        float(eltype(A_ctrl)),
        float(eltype(B_ctrl)),
        float(eltype(C_ctrl)),
        float(eltype(D_ctrl)),
        float(eltype(u_ref)),
        float(eltype(y_ref)),
    )
    return LinearOutputFeedback{T,typeof(measurement)}(
        Matrix{T}(A_ctrl),
        Matrix{T}(B_ctrl),
        Matrix{T}(C_ctrl),
        Matrix{T}(D_ctrl),
        Vector{T}(u_ref),
        Vector{T}(y_ref),
        measurement,
    )
end

state_dimension(model::LinearOutputFeedback) = size(model.A_ctrl, 1)

"""
    FunctionController(n_ctrl, f_ctrl, h_ctrl)

General controller defined by user-supplied dynamics and output callables.

When no actuator model is selected, both callables must ignore their `δ`
argument.
"""
struct FunctionController{F,H} <: AbstractController
    n_ctrl::Int
    drift::F
    output::H
    function FunctionController(n_ctrl::Integer, drift::F, output::H) where {F,H}
        n_ctrl >= 0 || throw(ArgumentError("n_ctrl must be nonnegative"))
        new{F,H}(n_ctrl, drift, output)
    end
end

state_dimension(model::FunctionController) = model.n_ctrl

# -----------------------------------------------------------------------------
# Controller dynamics
# -----------------------------------------------------------------------------

"""Evaluate controller state drift `f_ctrl`."""
function f_ctrl(
    ::LinearStateFeedback,
    x_ctrl::AbstractVector,
    x_rb,
    δ::AbstractVector,
    ϖ::AbstractVector,
    xhat::AbstractVector,
    t::Real,
)
    isempty(x_ctrl) || throw(DimensionMismatch("static feedback has no state"))
    return empty_state(eltype(x_ctrl))
end

function f_ctrl(
    model::LinearOutputFeedback,
    x_ctrl::AbstractVector,
    x_rb,
    δ::AbstractVector,
    ϖ::AbstractVector,
    xhat::AbstractVector,
    t::Real,
)
    length(x_ctrl) == state_dimension(model) ||
        throw(DimensionMismatch("x_ctrl has the wrong length"))
    # The measurement callback isolates controller design from the complete
    # simulation-state layout.
    y = model.measurement(x_rb, δ, ϖ, xhat, t)
    length(y) == length(model.y_ref) ||
        throw(DimensionMismatch("measurement has the wrong length"))
    return model.A_ctrl * x_ctrl + model.B_ctrl * (y - model.y_ref)
end

function f_ctrl(
    model::FunctionController,
    x_ctrl::AbstractVector,
    x_rb,
    δ::AbstractVector,
    ϖ::AbstractVector,
    xhat::AbstractVector,
    t::Real,
)
    length(x_ctrl) == state_dimension(model) ||
        throw(DimensionMismatch("x_ctrl has the wrong length"))
    return model.drift(x_ctrl, x_rb, δ, ϖ, xhat, t)
end

# -----------------------------------------------------------------------------
# Controller outputs
# -----------------------------------------------------------------------------

"""Evaluate control input `h_ctrl`."""
function h_ctrl(
    model::LinearStateFeedback,
    x_ctrl::AbstractVector,
    x_rb,
    δ::AbstractVector,
    ϖ::AbstractVector,
    xhat::AbstractVector,
    t::Real,
)
    isempty(x_ctrl) || throw(DimensionMismatch("static feedback has no state"))
    length(xhat) == length(model.x_ref) ||
        throw(DimensionMismatch("xhat has the wrong length"))
    # Static feedback operates on the estimator output rather than directly on
    # the truth state x_rb.
    return model.u_ref - model.K * (xhat - model.x_ref)
end

function h_ctrl(
    model::LinearOutputFeedback,
    x_ctrl::AbstractVector,
    x_rb,
    δ::AbstractVector,
    ϖ::AbstractVector,
    xhat::AbstractVector,
    t::Real,
)
    length(x_ctrl) == state_dimension(model) ||
        throw(DimensionMismatch("x_ctrl has the wrong length"))
    y = model.measurement(x_rb, δ, ϖ, xhat, t)
    length(y) == length(model.y_ref) ||
        throw(DimensionMismatch("measurement has the wrong length"))
    return model.u_ref +
           model.C_ctrl * x_ctrl +
           model.D_ctrl * (y - model.y_ref)
end

function h_ctrl(
    model::FunctionController,
    x_ctrl::AbstractVector,
    x_rb,
    δ::AbstractVector,
    ϖ::AbstractVector,
    xhat::AbstractVector,
    t::Real,
)
    length(x_ctrl) == state_dimension(model) ||
        throw(DimensionMismatch("x_ctrl has the wrong length"))
    return model.output(x_ctrl, x_rb, δ, ϖ, xhat, t)
end
