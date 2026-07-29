# -----------------------------------------------------------------------------
# Shared scalar and static-array aliases
# -----------------------------------------------------------------------------

"""Default floating-point scalar used by ACES constructors."""
const RealT = Float64

"""Statically sized three-vector."""
const Vec3{T} = SVector{3,T}

"""Statically sized four-vector."""
const Vec4{T} = SVector{4,T}

"""Statically sized six-vector."""
const Vec6{T} = SVector{6,T}

"""Statically sized 3 × 3 matrix."""
const Mat3{T} = SMatrix{3,3,T,9}

# -----------------------------------------------------------------------------
# Subsystem interface hierarchy
#
# Concrete models subtype one of these interfaces and implement the applicable
# drift, diffusion, output, and dimension methods in their subsystem file.
# -----------------------------------------------------------------------------

"""Abstract interface for rigid-body models."""
abstract type AbstractRigidBodyModel end

"""Abstract interface for attitude-coordinate representations."""
abstract type AbstractAttitudeRepresentation end

"""Abstract interface for complete environment models."""
abstract type AbstractEnvironmentModel end

"""Abstract interface for gravity models."""
abstract type AbstractGravityModel end

"""Abstract interface for atmospheric-density models."""
abstract type AbstractDensityModel end

"""Abstract interface for wind models."""
abstract type AbstractWindModel end

"""Abstract interface for nominal aerodynamic models."""
abstract type AbstractAerodynamicModel end

"""Abstract interface for stochastic aerodynamic-residual models."""
abstract type AbstractAerodynamicResidualModel end

"""Abstract interface for actuator models."""
abstract type AbstractActuatorModel end

"""Abstract interface for measurement models."""
abstract type AbstractMeasurementModel end

"""Abstract interface for state estimators."""
abstract type AbstractEstimator end

"""Abstract interface for feedback controllers."""
abstract type AbstractController end

# -----------------------------------------------------------------------------
# Traits shared by deterministic and stochastic models
# -----------------------------------------------------------------------------

"""Return an empty statically sized state vector with scalar type `T`."""
empty_state(::Type{T}=RealT) where {T<:Real} = SVector{0,T}()

"""Return the number of states of `model`."""
state_dimension(model) = throw(MethodError(state_dimension, (model,)))

"""Return the number of independent Wiener process inputs carried by `model`."""
noise_dimension(::Any) = 0
