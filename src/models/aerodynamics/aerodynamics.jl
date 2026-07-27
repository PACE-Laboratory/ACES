# -----------------------------------------------------------------------------
# Nominal and stochastic-residual model definitions
# -----------------------------------------------------------------------------
#
# Extension point: aerodynamic models are the subsystem most likely to be
# aircraft-specific. Define those types in the end user's workspace (or a
# separate aircraft package), subtype AbstractAerodynamicModel, and implement
# state_dimension, F_model, M_model, and f_aero. A custom stochastic residual
# similarly subtypes AbstractAerodynamicResidualModel and implements
# state_dimension, noise_dimension, f_res, σ_res, and g_res. Wrap the selected
# instances in `Aerodynamics(nominal, residual)` and pass that to AircraftModel.
# Only reusable, aircraft-independent formulations should be added to this file.

"""
    QuasiSteadyAerodynamics(F_model, M_model)

Stateless nominal aerodynamic force and moment functions. Each function is
called as `(v_r, ω_r, x_aero, δ, ρ)`.
"""
struct QuasiSteadyAerodynamics{FF,FM} <: AbstractAerodynamicModel
    force::FF
    moment::FM
end

state_dimension(::QuasiSteadyAerodynamics) = 0

"""Aerodynamic residual model with no state, noise, or output."""
struct NoAerodynamicResidual <: AbstractAerodynamicResidualModel end

state_dimension(::NoAerodynamicResidual) = 0
noise_dimension(::NoAerodynamicResidual) = 0

"""
Linear stochastic aerodynamic residual with drift `A_res*x_res`, diffusion
`L_res`, and load output `G_res*x_res`.
"""
struct LinearAerodynamicResidual{T<:Real} <: AbstractAerodynamicResidualModel
    A_res::Matrix{T}
    L_res::Matrix{T}
    G_res::Matrix{T}
end

function LinearAerodynamicResidual(
    A_res::AbstractMatrix{<:Real},
    L_res::AbstractMatrix{<:Real},
    G_res::AbstractMatrix{<:Real},
)
    n = size(A_res, 1)
    size(A_res, 2) == n || throw(DimensionMismatch("A_res must be square"))
    size(L_res, 1) == n || throw(DimensionMismatch("L_res must have $n rows"))
    size(G_res) == (6, n) || throw(DimensionMismatch("G_res must be 6 × $n"))
    T = promote_type(
        float(eltype(A_res)),
        float(eltype(L_res)),
        float(eltype(G_res)),
    )
    return LinearAerodynamicResidual{T}(
        Matrix{T}(A_res),
        Matrix{T}(L_res),
        Matrix{T}(G_res),
    )
end

state_dimension(model::LinearAerodynamicResidual) = size(model.A_res, 1)
noise_dimension(model::LinearAerodynamicResidual) = size(model.L_res, 2)

"""Nominal and residual aerodynamic models evaluated as one subsystem."""
struct Aerodynamics{
    N<:AbstractAerodynamicModel,
    R<:AbstractAerodynamicResidualModel,
}
    nominal::N
    residual::R
end

Aerodynamics(nominal::AbstractAerodynamicModel) =
    Aerodynamics(nominal, NoAerodynamicResidual())

state_dimension(model::Aerodynamics) =
    state_dimension(model.nominal) + state_dimension(model.residual)
noise_dimension(model::Aerodynamics) = noise_dimension(model.residual)

# -----------------------------------------------------------------------------
# Nominal aerodynamic interface
# -----------------------------------------------------------------------------

"""Evaluate nominal body-frame aerodynamic force."""
function F_model(
    model::QuasiSteadyAerodynamics,
    v_r::AbstractVector,
    ω_r::AbstractVector,
    x_aero::AbstractVector,
    δ::AbstractVector,
    ρ::Real,
)
    isempty(x_aero) || throw(DimensionMismatch("quasi-steady aerodynamics has no state"))
    F = model.force(v_r, ω_r, x_aero, δ, ρ)
    length(F) == 3 || throw(DimensionMismatch("F_model must return a three-vector"))
    return F
end

"""Evaluate nominal body-frame aerodynamic moment."""
function M_model(
    model::QuasiSteadyAerodynamics,
    v_r::AbstractVector,
    ω_r::AbstractVector,
    x_aero::AbstractVector,
    δ::AbstractVector,
    ρ::Real,
)
    isempty(x_aero) || throw(DimensionMismatch("quasi-steady aerodynamics has no state"))
    M = model.moment(v_r, ω_r, x_aero, δ, ρ)
    length(M) == 3 || throw(DimensionMismatch("M_model must return a three-vector"))
    return M
end

"""Evaluate unsteady-aerodynamics state drift."""
f_aero(
    ::QuasiSteadyAerodynamics,
    x_aero::AbstractVector,
    v_r::AbstractVector,
    ω_r::AbstractVector,
    δ::AbstractVector,
    ρ::Real,
) = isempty(x_aero) ? empty_state(eltype(x_aero)) :
    throw(DimensionMismatch("quasi-steady aerodynamics has no state"))

# -----------------------------------------------------------------------------
# Residual SDE interface
# -----------------------------------------------------------------------------

"""Evaluate aerodynamic-residual state drift."""
f_res(
    ::NoAerodynamicResidual,
    x_res::AbstractVector,
    v_r::AbstractVector,
    ω_r::AbstractVector,
    x_aero::AbstractVector,
    δ::AbstractVector,
    ρ::Real,
) = isempty(x_res) ? empty_state(eltype(x_res)) :
    throw(DimensionMismatch("the selected residual model has no state"))

function f_res(
    model::LinearAerodynamicResidual,
    x_res::AbstractVector,
    v_r::AbstractVector,
    ω_r::AbstractVector,
    x_aero::AbstractVector,
    δ::AbstractVector,
    ρ::Real,
)
    length(x_res) == state_dimension(model) ||
        throw(DimensionMismatch("x_res has the wrong length"))
    return model.A_res * x_res
end

"""Evaluate aerodynamic-residual diffusion."""
σ_res(
    ::NoAerodynamicResidual,
    x_res::AbstractVector{T},
    v_r::AbstractVector,
    ω_r::AbstractVector,
    x_aero::AbstractVector,
    δ::AbstractVector,
    ρ::Real,
) where {T} = zeros(T, 0, 0)

σ_res(
    model::LinearAerodynamicResidual,
    x_res::AbstractVector,
    v_r::AbstractVector,
    ω_r::AbstractVector,
    x_aero::AbstractVector,
    δ::AbstractVector,
    ρ::Real,
) = model.L_res

"""Evaluate the 6 × `n_res` aerodynamic-residual output matrix."""
g_res(
    ::NoAerodynamicResidual,
    x_res::AbstractVector{T},
    v_r::AbstractVector,
    ω_r::AbstractVector,
    x_aero::AbstractVector,
    δ::AbstractVector,
    ρ::Real,
) where {T} = zeros(T, 6, 0)

g_res(
    model::LinearAerodynamicResidual,
    x_res::AbstractVector,
    v_r::AbstractVector,
    ω_r::AbstractVector,
    x_aero::AbstractVector,
    δ::AbstractVector,
    ρ::Real,
) = model.G_res

# -----------------------------------------------------------------------------
# Combined aerodynamic loads
# -----------------------------------------------------------------------------

"""
    aerodynamic_loads(model, x_aero, x_res, v_r, ω_r, δ, ρ)

Return simulated body-frame force and moment, including residual loads.
"""
function aerodynamic_loads(
    model::Aerodynamics,
    x_aero::AbstractVector,
    x_res::AbstractVector,
    v_r::AbstractVector,
    ω_r::AbstractVector,
    δ::AbstractVector,
    ρ::Real,
)
    # The nominal and residual models intentionally receive the same flow,
    # unsteady-state, and actuator context.
    F = F_model(model.nominal, v_r, ω_r, x_aero, δ, ρ)
    M = M_model(model.nominal, v_r, ω_r, x_aero, δ, ρ)
    # g_res is a 6 × n_res map. Its first and last three outputs perturb force
    # and moment, respectively.
    residual = g_res(model.residual, x_res, v_r, ω_r, x_aero, δ, ρ) * x_res
    length(residual) == 6 ||
        throw(DimensionMismatch("the residual output must have length 6"))
    return (F=F + residual[1:3], M=M + residual[4:6])
end

# -----------------------------------------------------------------------------
# Fixed-wing derived air data
# -----------------------------------------------------------------------------

"""
    fixed_wing_air_data(v_r)

Return airspeed `V`, angle of attack `α`, and sideslip angle `β`. At zero
airspeed both angles are defined as zero.
"""
function fixed_wing_air_data(v_r::AbstractVector{T}) where {T<:Real}
    length(v_r) == 3 || throw(DimensionMismatch("v_r must have length 3"))
    V = norm(v_r)
    if iszero(V)
        return (V=V, α=zero(T), β=zero(T))
    end
    α = atan(v_r[3], v_r[1])
    # Clamp protects asin from roundoff that can place v_r[2]/V just outside
    # the closed interval [-1, 1].
    β = asin(clamp(v_r[2] / V, -one(T), one(T)))
    return (V=V, α=α, β=β)
end
