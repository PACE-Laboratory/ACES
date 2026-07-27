# -----------------------------------------------------------------------------
# Measurement-model definitions
# -----------------------------------------------------------------------------
#
# Extension point: sensor suites and estimators are usually application
# specific. User-defined measurement types implement state_dimension,
# noise_dimension, f_meas, σ_meas, and h_meas; estimator types implement
# state_dimension, f_est, and h_est. These interfaces receive only the complete
# simulation state x and time. If a design needs actuator configuration δ, the
# model owns and applies its documented x -> δ map internally instead of
# receiving δ as a separate core input. Pass instances as `measurement` and
# `estimator` to AircraftModel.

"""
Stateless, deterministic measurement `y_meas = h(x, t)`.

The callable `h` owns any application-specific mapping from the complete
simulation state to quantities such as actuator configuration.
"""
struct IdealMeasurement{H} <: AbstractMeasurementModel
    h::H
end

state_dimension(::IdealMeasurement) = 0
noise_dimension(::IdealMeasurement) = 0

"""
Ornstein-Uhlenbeck colored measurement disturbance added to `h(x, t)`.

The state drift is `-x_meas/τ`, diffusion is `sqrt_R/τ`, and `h` is called as
`h(x, t)`.
"""
struct OrnsteinUhlenbeckMeasurement{T<:Real,H} <: AbstractMeasurementModel
    τ::T
    sqrt_R::Matrix{T}
    h::H
end

function OrnsteinUhlenbeckMeasurement(
    τ::Real,
    sqrt_R::AbstractMatrix{<:Real},
    h,
)
    τ > 0 || throw(ArgumentError("τ must be positive"))
    size(sqrt_R, 1) > 0 ||
        throw(ArgumentError("sqrt_R must have at least one output row"))
    T = promote_type(typeof(float(τ)), float(eltype(sqrt_R)))
    return OrnsteinUhlenbeckMeasurement{T,typeof(h)}(
        T(τ),
        Matrix{T}(sqrt_R),
        h,
    )
end

state_dimension(model::OrnsteinUhlenbeckMeasurement) = size(model.sqrt_R, 1)
noise_dimension(model::OrnsteinUhlenbeckMeasurement) = size(model.sqrt_R, 2)

# -----------------------------------------------------------------------------
# Estimator definitions
# -----------------------------------------------------------------------------

"""Estimator that directly reports the current measurement."""
struct IdentityEstimator <: AbstractEstimator end

state_dimension(::IdentityEstimator) = 0

"""
    FunctionEstimator(n_est, f, h)

General estimator with user-supplied callables `f(x_est, y_meas, x, t)` and
`h(x_est, y_meas, x, t)`.
"""
struct FunctionEstimator{F,H} <: AbstractEstimator
    n_est::Int
    drift::F
    output::H
    function FunctionEstimator(n_est::Integer, drift::F, output::H) where {F,H}
        n_est >= 0 || throw(ArgumentError("n_est must be nonnegative"))
        new{F,H}(n_est, drift, output)
    end
end

state_dimension(model::FunctionEstimator) = model.n_est

# -----------------------------------------------------------------------------
# Measurement SDE interface
# -----------------------------------------------------------------------------

"""Evaluate measurement-model drift `f_meas`."""
function f_meas(
    ::IdealMeasurement,
    x_meas::AbstractVector,
    x::AbstractVector,
    t::Real,
)
    isempty(x_meas) || throw(DimensionMismatch("ideal measurements have no state"))
    return empty_state(eltype(x_meas))
end

function f_meas(
    model::OrnsteinUhlenbeckMeasurement,
    x_meas::AbstractVector,
    x::AbstractVector,
    t::Real,
)
    length(x_meas) == state_dimension(model) ||
        throw(DimensionMismatch("x_meas has the wrong length"))
    return -x_meas / model.τ
end

"""Evaluate measurement-model diffusion `σ_meas`."""
σ_meas(
    ::IdealMeasurement,
    x_meas::AbstractVector{T},
    x::AbstractVector,
    t::Real,
) where {T} = zeros(T, 0, 0)

σ_meas(
    model::OrnsteinUhlenbeckMeasurement,
    x_meas::AbstractVector,
    x::AbstractVector,
    t::Real,
) = model.sqrt_R / model.τ

"""Evaluate measurement-model output `h_meas`."""
function h_meas(
    model::IdealMeasurement,
    x_meas::AbstractVector,
    x::AbstractVector,
    t::Real,
)
    isempty(x_meas) || throw(DimensionMismatch("ideal measurements have no state"))
    return model.h(x, t)
end

function h_meas(
    model::OrnsteinUhlenbeckMeasurement,
    x_meas::AbstractVector,
    x::AbstractVector,
    t::Real,
)
    length(x_meas) == state_dimension(model) ||
        throw(DimensionMismatch("x_meas has the wrong length"))
    nominal = model.h(x, t)
    length(nominal) == length(x_meas) ||
        throw(DimensionMismatch("h must return $(length(x_meas)) outputs"))
    return nominal + x_meas
end

# -----------------------------------------------------------------------------
# Estimator dynamics and output interface
# -----------------------------------------------------------------------------

"""Evaluate estimator state drift `f_est`."""
function f_est(
    ::IdentityEstimator,
    x_est::AbstractVector,
    y_meas::AbstractVector,
    x::AbstractVector,
    t::Real,
)
    isempty(x_est) || throw(DimensionMismatch("the identity estimator has no state"))
    return empty_state(eltype(x_est))
end

function f_est(
    model::FunctionEstimator,
    x_est::AbstractVector,
    y_meas::AbstractVector,
    x::AbstractVector,
    t::Real,
)
    length(x_est) == state_dimension(model) ||
        throw(DimensionMismatch("x_est has the wrong length"))
    return model.drift(x_est, y_meas, x, t)
end

"""Evaluate extended state estimate `h_est`."""
function h_est(
    ::IdentityEstimator,
    x_est::AbstractVector,
    y_meas::AbstractVector,
    x::AbstractVector,
    t::Real,
)
    isempty(x_est) || throw(DimensionMismatch("the identity estimator has no state"))
    return copy(y_meas)
end

function h_est(
    model::FunctionEstimator,
    x_est::AbstractVector,
    y_meas::AbstractVector,
    x::AbstractVector,
    t::Real,
)
    length(x_est) == state_dimension(model) ||
        throw(DimensionMismatch("x_est has the wrong length"))
    return model.output(x_est, y_meas, x, t)
end
