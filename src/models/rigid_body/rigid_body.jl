# -----------------------------------------------------------------------------
# Attitude representations and rigid-body model data
# -----------------------------------------------------------------------------
#
# Extension point: keep broadly reusable rigid-body formulations in this file.
# Aircraft-specific rigid-body types normally belong in the user's workspace
# and subtype AbstractRigidBodyModel. Core assembly requires implementations of
# state_dimension, state_view, and rigid_body_dynamics for that custom type.
# Pass the resulting instance as `rigid_body` when constructing AircraftModel.

"""Roll-pitch-yaw attitude coordinates using the documented 1-2-3 sequence."""
struct EulerAngles <: AbstractAttitudeRepresentation end

"""Scalar-first unit-quaternion attitude coordinates."""
struct UnitQuaternion <: AbstractAttitudeRepresentation end

"""Return the number of attitude coordinates used by `representation`."""
pose_dimension(::EulerAngles) = 6
pose_dimension(::UnitQuaternion) = 7

"""
    RigidBody(m, 𝐈; attitude=UnitQuaternion())

Rigid-body mass and body-frame inertia about the center of gravity.
"""
struct RigidBody{T<:Real,R<:AbstractAttitudeRepresentation} <: AbstractRigidBodyModel
    m::T
    𝐈::Mat3{T}
    attitude::R
end

function RigidBody(
    m::Real,
    𝐈::AbstractMatrix{<:Real};
    attitude::AbstractAttitudeRepresentation=UnitQuaternion(),
)
    size(𝐈) == (3, 3) || throw(DimensionMismatch("𝐈 must be 3 × 3"))
    m > 0 || throw(ArgumentError("mass must be positive"))
    T = promote_type(typeof(float(m)), float(eltype(𝐈)))
    inertia = Mat3{T}(𝐈)
    isapprox(inertia, transpose(inertia)) ||
        throw(ArgumentError("𝐈 must be symmetric"))
    isposdef(Symmetric(Matrix(inertia))) ||
        throw(ArgumentError("𝐈 must be positive definite"))
    return RigidBody{T,typeof(attitude)}(T(m), inertia, attitude)
end

pose_dimension(model::RigidBody) = pose_dimension(model.attitude)
# The integration state is x_rb = [η; ν], with six generalized velocities.
state_dimension(model::RigidBody) = pose_dimension(model) + 6

"""
Kinematic values decoded from a rigid-body integration state.

`s` and `R_IB` describe pose, while `v` and `ω` are expressed in the body
frame.
"""
struct RigidBodyState{T<:Real}
    s::Vec3{T}
    R_IB::Mat3{T}
    v::Vec3{T}
    ω::Vec3{T}
end

# -----------------------------------------------------------------------------
# SO(3) utilities
# -----------------------------------------------------------------------------

"""Return the cross-product equivalent matrix `S(a)`."""
function cpem(a::AbstractVector{T}) where {T<:Real}
    length(a) == 3 || throw(DimensionMismatch("a must have length 3"))
    z = zero(T)
    return @SMatrix [
        z -a[3] a[2]
        a[3] z -a[1]
        -a[2] a[1] z
    ]
end

"""Return the vector associated with the skew-symmetric part of `A`."""
function cpeminv(A::AbstractMatrix{T}) where {T<:Real}
    size(A) == (3, 3) || throw(DimensionMismatch("A must be 3 × 3"))
    two = one(T) + one(T)
    return Vec3{T}(
        (A[3, 2] - A[2, 3]) / two,
        (A[1, 3] - A[3, 1]) / two,
        (A[2, 1] - A[1, 2]) / two,
    )
end

"""
    R_IB(EulerAngles(), Θ)
    R_IB(Θ)

Return the body-to-inertial rotation matrix as an `SMatrix{3,3}` for the
three roll-pitch-yaw angles `Θ` in radians.
"""
function R_IB(::EulerAngles, Θ::AbstractVector{T}) where {T<:Real}
    length(Θ) == 3 || throw(DimensionMismatch("Θ must have length 3"))
    φ, θ, ψ = Θ
    sφ, cφ = sincos(φ)
    sθ, cθ = sincos(θ)
    sψ, cψ = sincos(ψ)
    return @SMatrix [
        cψ*cθ cψ*sφ*sθ-cφ*sψ sφ*sψ+cφ*cψ*sθ
        cθ*sψ cφ*cψ+sφ*sψ*sθ cφ*sψ*sθ-cψ*sφ
        -sθ cθ*sφ cφ*cθ
    ]
end

"""
    R_IB(UnitQuaternion(), q_IB)
    R_IB(q_IB)

Return the body-to-inertial rotation matrix as an `SMatrix{3,3}` for a nonzero,
scalar-first quaternion of length four. The input is normalized before
evaluation. A zero quaternion throws `ArgumentError`.
"""
function R_IB(::UnitQuaternion, q_IB::AbstractVector{T}) where {T<:Real}
    length(q_IB) == 4 || throw(DimensionMismatch("q_IB must have length 4"))
    q_norm = norm(q_IB)
    iszero(q_norm) && throw(ArgumentError("q_IB must be nonzero"))
    q0, q1, q2, q3 = q_IB / q_norm
    two = one(T) + one(T)
    return @SMatrix [
        q0*q0+q1*q1-q2*q2-q3*q3 two*(q1*q2-q0*q3) two*(q0*q2+q1*q3)
        two*(q0*q3+q1*q2) q0*q0-q1*q1+q2*q2-q3*q3 two*(q2*q3-q0*q1)
        two*(q1*q3-q0*q2) two*(q0*q1+q2*q3) q0*q0-q1*q1-q2*q2+q3*q3
    ]
end

"""
    R_IB(attitude::AbstractVector{<:Real})

Return the body-to-inertial rotation matrix as an `SMatrix{3,3}`.

Interpret a length-three vector as 1-2-3 roll-pitch-yaw Euler angles
`[φ, θ, ψ]` in radians, and a length-four vector as a scalar-first
quaternion `[q0, q1, q2, q3]`. Three-component inputs are always
interpreted as Euler angles.

Quaternion inputs are normalized; a zero quaternion throws `ArgumentError`.
Any other vector length throws `DimensionMismatch`.
"""
function R_IB(attitude::AbstractVector{<:Real})
    length(attitude) == 3 && return R_IB(EulerAngles(), attitude)
    length(attitude) == 4 && return R_IB(UnitQuaternion(), attitude)
    throw(DimensionMismatch("attitude must have length 3 (Euler angles) or 4 (quaternion)"))
end
R_IB(Θ::SVector{3,<:Real}) = R_IB(EulerAngles(), Θ)
R_IB(q_IB::SVector{4,<:Real}) = R_IB(UnitQuaternion(), q_IB)


"""
    q_IB(EulerAngles(), Θ)
    q_IB(Θ)

Convert the three roll-pitch-yaw Euler angles `Θ` in radians to a scalar-first
unit quaternion `[q0, q1, q2, q3]`, returned as an `SVector{4}`. The quaternion
represents the body-to-inertial rotation `R_IB(EulerAngles(), Θ)`, with its sign
chosen so that `q0 ≥ 0`.
"""
function q_IB(::EulerAngles, Θ::AbstractVector{T}) where {T<:Real}
    length(Θ) == 3 || throw(DimensionMismatch("Θ must have length 3"))
    φ, θ, ψ = Θ
    sφ, cφ = sincos(φ / 2)
    sθ, cθ = sincos(θ / 2)
    sψ, cψ = sincos(ψ / 2)
    # Compose the same R3(ψ) R2(θ) R1(φ) rotation as the Euler-angle matrix.
    q = @SVector [
        cφ*cθ*cψ + sφ*sθ*sψ,
        sφ*cθ*cψ - cφ*sθ*sψ,
        cφ*sθ*cψ + sφ*cθ*sψ,
        cφ*cθ*sψ - sφ*sθ*cψ,
    ]
    return q[1] < 0 ? -q : q
end

q_IB(Θ::AbstractVector{<:Real}) = q_IB(EulerAngles(), Θ)

"""
    Θ(R_IB)
    Θ(EulerAngles(), angles)
    Θ(UnitQuaternion(), q_IB)
    Θ(attitude)

Return principal roll-pitch-yaw Euler angles in radians as an `SVector{3}`,
using the documented body-to-inertial rotation `R3(ψ) R2(θ) R1(φ)`.
Accept a real 3 × 3 rotation matrix, three Euler angles, or a nonzero,
scalar-first quaternion of length four. The vector-only form selects the
representation by length, as `R_IB(attitude)` does. Quaternion inputs are
normalized through `R_IB`; inputs are never modified.

Pitch lies in [-π/2, π/2], and roll/yaw lie in [-π, π], without temporal
unwrapping. At numerical gimbal lock (cosine of pitch no greater than eight
machine epsilons), choose zero yaw and recover the remaining rotation in roll.
Matrix inputs must already be rotations; they are not projected onto SO(3).
Invalid input dimensions throw `DimensionMismatch`; a zero quaternion throws
`ArgumentError`.
"""
function Θ(R_IB::AbstractMatrix{T}) where {T<:Real}
    size(R_IB) == (3, 3) || throw(DimensionMismatch("R_IB must be 3 × 3"))
    cθ = hypot(R_IB[1, 1], R_IB[2, 1])
    θ = atan(-R_IB[3, 1], cθ)
    if cθ > 8eps(float(T))
        ψ = atan(R_IB[2, 1], R_IB[1, 1])
    else
        # At gimbal lock only a combination of roll and yaw is observable.
        ψ = zero(θ)
    end
    # Remove yaw before extracting roll to retain rotation accuracy near
    # pitch ±π/2, where R32 and R33 both approach zero.
    sψ, cψ = sincos(ψ)
    φ = atan(sψ * R_IB[1, 3] - cψ * R_IB[2, 3], cψ * R_IB[2, 2] - sψ * R_IB[1, 2])
    return Vec3(φ, θ, ψ)
end

Θ(::EulerAngles, angles::AbstractVector{<:Real}) = Θ(R_IB(EulerAngles(), angles))
Θ(::UnitQuaternion, q_IB::AbstractVector{<:Real}) = Θ(R_IB(UnitQuaternion(), q_IB))

function Θ(attitude::AbstractVector{<:Real})
    length(attitude) == 3 && return Θ(EulerAngles(), attitude)
    length(attitude) == 4 && return Θ(UnitQuaternion(), attitude)
    throw(DimensionMismatch("attitude must have length 3 (Euler angles) or 4 (quaternion)"))
end

# -----------------------------------------------------------------------------
# Pose kinematics
# -----------------------------------------------------------------------------

"""Return the body-angular-velocity to Euler-angle-rate map."""
function L_IB(Θ::AbstractVector{T}) where {T<:Real}
    length(Θ) == 3 || throw(DimensionMismatch("Θ must have length 3"))
    φ, θ = Θ[1], Θ[2]
    sφ, cφ = sincos(φ)
    cθ = cos(θ)
    abs(cθ) > sqrt(eps(float(T))) ||
        throw(DomainError(θ, "Euler-angle kinematics are singular at ±π/2"))
    tθ = tan(θ)
    return @SMatrix [
        one(T) sφ*tθ cφ*tθ
        zero(T) cφ -sφ
        zero(T) sφ/cθ cφ/cθ
    ]
end

"""Return the quaternion kinematics matrix `Ξ(q_IB)`."""
function Ξ(q_IB::AbstractVector{T}) where {T<:Real}
    length(q_IB) == 4 || throw(DimensionMismatch("q_IB must have length 4"))
    q0, q1, q2, q3 = q_IB
    return @SMatrix [
        -q1 -q2 -q3
        q0 -q3 q2
        q3 q0 -q1
        -q2 q1 q0
    ]
end

"""Return the quaternion rate matrix `Ω(ω)`."""
function Ω(ω::AbstractVector{T}) where {T<:Real}
    length(ω) == 3 || throw(DimensionMismatch("ω must have length 3"))
    p, q, r = ω
    z = zero(T)
    return @SMatrix [
        z -p -q -r
        p z r -q
        q -r z p
        r q -p z
    ]
end

"""Return the rigid-body kinematics matrix `J_η(η)`."""
function J_η(representation::AbstractAttitudeRepresentation, η::AbstractVector{T}) where {T<:Real}
    n_pose = pose_dimension(representation)
    length(η) == n_pose || throw(DimensionMismatch("η must have length $n_pose"))
    J = zeros(T, n_pose, 6)
    # Translational velocity is stored in body axes and must be rotated into
    # inertial axes before updating the inertial position s.
    J[1:3, 1:3] .= R_IB(representation, @view η[4:end])
    # Only the rotational block changes with the attitude representation.
    if representation isa EulerAngles
        J[4:6, 4:6] .= L_IB(@view η[4:6])
    else
        J[4:7, 4:6] .= Ξ(@view η[4:7]) / 2
    end
    return J
end

J_η(model::RigidBody, η::AbstractVector) = J_η(model.attitude, η)

"""Evaluate the rigid-body kinematics vector field."""
f_η(representation::AbstractAttitudeRepresentation, η::AbstractVector, ν::AbstractVector) =
    J_η(representation, η) * ν
f_η(model::RigidBody, η::AbstractVector, ν::AbstractVector) =
    f_η(model.attitude, η, ν)

# -----------------------------------------------------------------------------
# Newton-Euler dynamics and integration-state helpers
# -----------------------------------------------------------------------------

"""Return the 6 × 6 generalized mass matrix."""
function ℳ(model::RigidBody{T}) where {T}
    matrix = zeros(T, 6, 6)
    matrix[1:3, 1:3] .= model.m * Matrix{T}(I, 3, 3)
    matrix[4:6, 4:6] .= model.𝐈
    return SMatrix{6,6,T,36}(matrix)
end

"""
    f_ν(model, η, ν, g)

Evaluate the rigid-body velocity dynamics excluding aerodynamic force and
moment. `g` is expressed in the inertial frame.
"""
function f_ν(
    model::RigidBody,
    η::AbstractVector,
    ν::AbstractVector,
    g::AbstractVector,
)
    length(ν) == 6 || throw(DimensionMismatch("ν must have length 6"))
    length(g) == 3 || throw(DimensionMismatch("g must have length 3"))
    v = @view ν[1:3]
    ω = @view ν[4:6]
    R = R_IB(model.attitude, @view η[4:end])
    # Gravity is supplied in inertial axes; all terms in v_dot are body-frame
    # components. The cross products follow the signs in the canonical theory.
    v_dot = cross(v, ω) + transpose(R) * g
    ω_dot = model.𝐈 \ cross(model.𝐈 * ω, ω)
    return vcat(v_dot, ω_dot)
end

"""Decode an integration vector into a `RigidBodyState`."""
function state_view(model::RigidBody, x_rb::AbstractVector{T}) where {T<:Real}
    length(x_rb) == state_dimension(model) ||
        throw(DimensionMismatch("x_rb has the wrong length"))
    n_pose = pose_dimension(model)
    # Avoid copies while splitting x_rb = [η; v; ω].
    η = @view x_rb[1:n_pose]
    ν = @view x_rb[n_pose+1:n_pose+6]
    return RigidBodyState(
        Vec3{T}(η[1:3]),
        Mat3{T}(R_IB(model.attitude, @view η[4:end])),
        Vec3{T}(ν[1:3]),
        Vec3{T}(ν[4:6]),
    )
end

"""
    rigid_body_dynamics(model, x_rb, F, M, g)

Evaluate the full rigid-body state derivative for body-frame force `F`,
body-frame moment `M`, and inertial-frame gravitational acceleration `g`.
"""
function rigid_body_dynamics(
    model::RigidBody,
    x_rb::AbstractVector,
    F::AbstractVector,
    M::AbstractVector,
    g::AbstractVector,
)
    length(x_rb) == state_dimension(model) ||
        throw(DimensionMismatch("x_rb has the wrong length"))
    length(F) == length(M) == 3 ||
        throw(DimensionMismatch("F and M must have length 3"))
    n_pose = pose_dimension(model)
    # The pose representation changes n_pose, but ν always occupies the final
    # six entries of the rigid-body state.
    η = @view x_rb[1:n_pose]
    ν = @view x_rb[n_pose+1:n_pose+6]
    # Generalized loads use the same translational-then-rotational ordering as
    # the block-diagonal generalized mass matrix.
    ℱ = vcat(F, M)
    return vcat(f_η(model, η, ν), f_ν(model, η, ν, g) + ℳ(model) \ ℱ)
end
