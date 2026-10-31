# -----------------------------------------------------------------------------
# Gravity and density model types and constructors
# -----------------------------------------------------------------------------
#
# Extension point: reusable atmosphere/gravity models may be added here.
# Project-specific environments can instead subtype AbstractEnvironmentModel in
# a user file and implement `environment(model, s) -> (g=..., ρ=...)`.
# Select one by passing its instance as `environment` to AircraftModel.

"""
Uniform positive-down gravitational acceleration in the NED frame.
"""
struct UniformGravity{T<:Real} <: AbstractGravityModel
    g::T
    function UniformGravity(g::T=RealT(9.81)) where {T<:Real}
        g >= zero(T) || throw(ArgumentError("g must be nonnegative"))
        new{T}(g)
    end
end

"""
Constant air density.
"""
struct ConstantDensity{T<:Real} <: AbstractDensityModel
    ρ::T
    function ConstantDensity(ρ::T=RealT(1.225)) where {T<:Real}
        ρ >= zero(T) || throw(ArgumentError("ρ must be nonnegative"))
        new{T}(ρ)
    end
end

"""
Exponential atmospheric density model referenced to altitude `h0`.
"""
struct ExponentialDensity{T<:Real} <: AbstractDensityModel
    ρ0::T
    h0::T
    h_scale::T
    function ExponentialDensity(ρ0::T, h0::T, h_scale::T) where {T<:Real}
        ρ0 >= zero(T) || throw(ArgumentError("ρ0 must be nonnegative"))
        h_scale > zero(T) || throw(ArgumentError("h_scale must be positive"))
        new{T}(ρ0, h0, h_scale)
    end
end

# Outer constructor that ensures float types
function ExponentialDensity(ρ0::Real, h0::Real, h_scale::Real)
    T = promote_type(typeof(float(ρ0)), typeof(float(h0)), typeof(float(h_scale)))
    return ExponentialDensity(T(ρ0), T(h0), T(h_scale))
end

"""NASA Glenn metric standard atmosphere model."""
struct NASAMetricAtmosphere <: AbstractDensityModel end

# -----------------------------------------------------------------------------
# Composite environment model
# -----------------------------------------------------------------------------

"""
    Environment(gravity, density; h0=0)

Compose gravity and density models for an NED frame whose origin is at MSL
altitude `h0`.
"""
struct Environment{G<:AbstractGravityModel,D<:AbstractDensityModel,T<:Real} <:
       AbstractEnvironmentModel
    gravity_model::G
    density_model::D
    h0::T
end

# Environment (default) constructor
function Environment(
    gravity_model::AbstractGravityModel=UniformGravity(),
    density_model::AbstractDensityModel=ConstantDensity();
    h0::Real=0.0,
)
    return Environment(gravity_model, density_model, float(h0))
end

# -----------------------------------------------------------------------------
# Helpers
# -----------------------------------------------------------------------------

"""Return altitude above mean sea level from an NED position."""
function altitude(s::AbstractVector, h0::Real=0.0)
    length(s) == 3 || throw(DimensionMismatch("s must have length 3"))
    return h0 - s[3]
end

# -----------------------------------------------------------------------------
# Gravity interfaces
# -----------------------------------------------------------------------------

"""Evaluate inertial frame gravitational acceleration."""
function gravity(model::UniformGravity{T}, s::AbstractVector) where {T}
    length(s) == 3 || throw(DimensionMismatch("s must have length 3"))
    return Vec3{T}(zero(T), zero(T), model.g)
end

"""Evaluate gravitational acceleration from an environment model."""
gravity(model::Environment, s::AbstractVector) = gravity(model.gravity_model, s)

# -----------------------------------------------------------------------------
# Atmospheric density interfaces
# -----------------------------------------------------------------------------

"""Evaluate constant atmospheric density at MSL altitude `h`."""
density(model::ConstantDensity, h::Real) = model.ρ

"""Evaluate exponential atmospheric density at MSL altitude `h`."""
density(model::ExponentialDensity, h::Real) =
    model.ρ0 * exp(-(h - model.h0) / model.h_scale)

"""Evaluate the NASA Glenn metric atmosphere density at MSL altitude `h`."""
function density(::NASAMetricAtmosphere, h::Real)
    # NASA's metric curve fit is piecewise in geometric altitude. Temperature
    # is in degrees Celsius and pressure is in kilopascals in all three zones.
    if h < 11_000
        T = 15.04 - 0.00649h
        p = 101.29 * ((T + 273.1) / 288.08)^5.256
    elseif h < 25_000
        T = -56.46
        p = 22.65 * exp(1.73 - 0.000157h)
    else
        T = -131.21 + 0.00299h
        p = 2.488 * ((T + 273.1) / 216.6)^(-11.388)
    end
    # The constant 0.2869 is the metric specific-gas constant in the units used
    # by the source curve fit.
    ρ = p / (0.2869 * (T + 273.1))
    return max(zero(ρ), ρ)
end

"""Evaluate atmospheric density from an environment model."""
function density(model::Environment, s::AbstractVector)
    ρ = density(model.density_model, altitude(s, model.h0))
    ρ >= zero(ρ) || throw(DomainError(ρ, "density models must return ρ ≥ 0"))
    return ρ
end

# -----------------------------------------------------------------------------
# Environment model interface
# -----------------------------------------------------------------------------

"""
    environment(model, s)

Return `(g=..., ρ=...)` at inertial position `s`, without explicit time
dependence. `g` is inertial-frame gravitational acceleration and `ρ` is
nonnegative atmospheric density. Custom `AbstractEnvironmentModel` subtypes
must implement this two-argument interface.
"""
environment(model::Environment, s::AbstractVector) = (g=gravity(model, s), ρ=density(model, s))
