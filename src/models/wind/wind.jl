# -----------------------------------------------------------------------------
# Wind-model definitions
# -----------------------------------------------------------------------------
#
# Extension point: an aircraft- or site-specific wind model can live outside
# ACES and subtype AbstractWindModel. Implement state_dimension, noise_dimension
# (when stochastic), f_w, σ_w, and h_w, then pass the instance as `wind` to
# AircraftModel. Add a model here only when it is generally reusable.

"""Spatially uniform and stationary generalized wind velocity."""
struct ConstantWind{T<:Real} <: AbstractWindModel
    w::Vec3{T}
    ω_w::Vec3{T}
end

function ConstantWind(
    w::AbstractVector{<:Real}=zeros(3),
    ω_w::AbstractVector{<:Real}=zeros(3),
)
    length(w) == length(ω_w) == 3 ||
        throw(DimensionMismatch("w and ω_w must have length 3"))
    T = promote_type(float(eltype(w)), float(eltype(ω_w)))
    return ConstantWind(Vec3{T}(w), Vec3{T}(ω_w))
end

state_dimension(::ConstantWind) = 0
noise_dimension(::ConstantWind) = 0

"""
    FrozenWindField(W, ∇W; gradient_to_angular=fixed_wing_angular_velocity)

Analytic or numerical frozen wind field and its inertial-frame spatial
gradient. `gradient_to_angular` maps the body-frame gradient to `ω_w`.
"""
struct FrozenWindField{FW,FG,FA} <: AbstractWindModel
    W::FW
    ∇W::FG
    gradient_to_angular::FA
end

FrozenWindField(W, ∇W; gradient_to_angular=fixed_wing_angular_velocity) =
    FrozenWindField(W, ∇W, gradient_to_angular)

state_dimension(::FrozenWindField) = 0
noise_dimension(::FrozenWindField) = 0

"""
Continuous-time stochastic wind shaping filter.

The filter has drift `A_w*x_w`, diffusion `B_w`, body-frame turbulent output
`C_δw_b*x_w`, and apparent angular-wind output `C_ω_w*x_w`.
"""
struct ShapingFilterWind{T<:Real} <: AbstractWindModel
    A_w::Matrix{T}
    B_w::Matrix{T}
    C_δw_b::Matrix{T}
    C_ω_w::Matrix{T}
    w_bar::Vec3{T}
end

function ShapingFilterWind(
    A_w::AbstractMatrix{<:Real},
    B_w::AbstractMatrix{<:Real},
    C_δw_b::AbstractMatrix{<:Real},
    C_ω_w::AbstractMatrix{<:Real},
    w_bar::AbstractVector{<:Real}=zeros(3),
)
    n = size(A_w, 1)
    size(A_w, 2) == n || throw(DimensionMismatch("A_w must be square"))
    size(B_w, 1) == n || throw(DimensionMismatch("B_w must have $n rows"))
    size(C_δw_b) == (3, n) ||
        throw(DimensionMismatch("C_δw_b must be 3 × $n"))
    size(C_ω_w) == (3, n) ||
        throw(DimensionMismatch("C_ω_w must be 3 × $n"))
    length(w_bar) == 3 || throw(DimensionMismatch("w_bar must have length 3"))
    T = promote_type(
        float(eltype(A_w)),
        float(eltype(B_w)),
        float(eltype(C_δw_b)),
        float(eltype(C_ω_w)),
        float(eltype(w_bar)),
    )
    return ShapingFilterWind{T}(
        Matrix{T}(A_w),
        Matrix{T}(B_w),
        Matrix{T}(C_δw_b),
        Matrix{T}(C_ω_w),
        Vec3{T}(w_bar),
    )
end

state_dimension(model::ShapingFilterWind) = size(model.A_w, 1)
noise_dimension(model::ShapingFilterWind) = size(model.B_w, 2)

# -----------------------------------------------------------------------------
# Wind-gradient geometry
# -----------------------------------------------------------------------------

"""Transform an inertial-frame wind gradient into the body frame."""
function body_wind_gradient(∇W::AbstractMatrix, R_IB::AbstractMatrix)
    size(∇W) == size(R_IB) == (3, 3) ||
        throw(DimensionMismatch("∇W and R_IB must be 3 × 3"))
    return transpose(R_IB) * ∇W * R_IB
end

"""Return angular wind from the skew-symmetric part of a body-frame gradient."""
function wind_angular_velocity(Φ_W::AbstractMatrix)
    size(Φ_W) == (3, 3) || throw(DimensionMismatch("Φ_W must be 3 × 3"))
    # Only local rigid rotation is retained; the symmetric part represents
    # strain and therefore does not contribute to ω_W.
    return cpeminv((Φ_W - transpose(Φ_W)) / 2)
end

"""Return the fixed-wing apparent angular-wind approximation."""
function fixed_wing_angular_velocity(Φ_W::AbstractMatrix{T}) where {T<:Real}
    size(Φ_W) == (3, 3) || throw(DimensionMismatch("Φ_W must be 3 × 3"))
    return Vec3{T}(Φ_W[3, 2], -Φ_W[3, 1], Φ_W[2, 1])
end

# -----------------------------------------------------------------------------
# Stochastic-system interface: drift, diffusion, and output
# -----------------------------------------------------------------------------

"""Evaluate wind-model drift `f_w`."""
f_w(::ConstantWind, x_w::AbstractVector, x_rb, t::Real) = empty_state(eltype(x_w))
f_w(::FrozenWindField, x_w::AbstractVector, x_rb, t::Real) = empty_state(eltype(x_w))
function f_w(model::ShapingFilterWind, x_w::AbstractVector, x_rb, t::Real)
    length(x_w) == state_dimension(model) ||
        throw(DimensionMismatch("x_w has the wrong length"))
    return model.A_w * x_w
end

"""Evaluate wind-model diffusion `σ_w`."""
σ_w(model::ConstantWind{T}, x_w::AbstractVector, x_rb, t::Real) where {T} =
    zeros(T, 0, 0)
σ_w(::FrozenWindField, x_w::AbstractVector{T}, x_rb, t::Real) where {T} =
    zeros(T, 0, 0)
σ_w(model::ShapingFilterWind, x_w::AbstractVector, x_rb, t::Real) = model.B_w

"""Evaluate generalized wind output `h_w = [w; ω_w]`."""
function h_w(model::ConstantWind, x_w::AbstractVector, x_rb, t::Real)
    isempty(x_w) || throw(DimensionMismatch("constant wind has no state"))
    return vcat(model.w, model.ω_w)
end

function h_w(model::FrozenWindField, x_w::AbstractVector, x_rb, t::Real)
    isempty(x_w) || throw(DimensionMismatch("a frozen wind field has no state"))
    s = getproperty(x_rb, :s)
    R = getproperty(x_rb, :R_IB)
    # W and ∇W are defined in inertial coordinates. The aircraft-dependent
    # angular-wind mapping acts on the gradient after its body transformation.
    w = model.W(s, t)
    length(w) == 3 || throw(DimensionMismatch("W must return a three-vector"))
    Φ_W = body_wind_gradient(model.∇W(s, t), R)
    ω_w = model.gradient_to_angular(Φ_W)
    return vcat(w, ω_w)
end

function h_w(model::ShapingFilterWind, x_w::AbstractVector, x_rb, t::Real)
    length(x_w) == state_dimension(model) ||
        throw(DimensionMismatch("x_w has the wrong length"))
    R = getproperty(x_rb, :R_IB)
    δw_b = model.C_δw_b * x_w
    ω_w = model.C_ω_w * x_w
    # Turbulence-filter velocity outputs are body-frame fluctuations, whereas
    # the generalized wind vector stores translational wind in inertial axes.
    return vcat(model.w_bar + R * δw_b, ω_w)
end

# -----------------------------------------------------------------------------
# Wind triangle
# -----------------------------------------------------------------------------

"""
    wind_triangle(x_rb, ϖ)

Return body-frame air-relative translational and angular velocities.
"""
function wind_triangle(x_rb, ϖ::AbstractVector)
    length(ϖ) == 6 || throw(DimensionMismatch("ϖ must have length 6"))
    R = getproperty(x_rb, :R_IB)
    v = getproperty(x_rb, :v)
    ω = getproperty(x_rb, :ω)
    # Translational wind is stored in inertial axes and must be rotated before
    # subtraction. Apparent angular wind is already expressed in body axes.
    v_r = v - transpose(R) * @view(ϖ[1:3])
    ω_r = ω - @view(ϖ[4:6])
    return (v_r=v_r, ω_r=ω_r)
end
