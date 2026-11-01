"""Aircraft Control and Estimation Simulator."""
module ACES

using StaticArrays
using LinearAlgebra
using OrdinaryDiffEqDefault: DefaultODEAlgorithm
using SciMLBase: ODEProblem, SDEProblem, solve
using StochasticDiffEqLowOrder: LambaEM

# Model files share the ACES module. Keep this order aligned with the physical
# signal flow: rigid-body/environment definitions precede wind and aerodynamic
# models, followed by effectors, estimation, control, and final assembly.
include("types.jl")
include("models/rigid_body/rigid_body.jl")
include("models/environment/environment.jl")
include("models/wind/wind.jl")
include("models/aerodynamics/aerodynamics.jl")
include("models/actuators/actuators.jl")
include("models/estimation/estimation.jl")
include("models/control/control.jl")
include("core.jl")

# Public API grouped by subsystem in the same order as the includes above.
export empty_state, state_dimension, noise_dimension
export AbstractRigidBodyModel, AbstractAttitudeRepresentation,
       EulerAngles, UnitQuaternion, RigidBody, RigidBodyState,
       state_view, q_IB, R_IB, Θ
export AbstractEnvironmentModel, AbstractGravityModel, AbstractDensityModel,
       Environment, UniformGravity, ConstantDensity, ExponentialDensity,
       NASAMetricAtmosphere, altitude
export AbstractWindModel, ConstantWind, FrozenWindField, ShapingFilterWind
export AbstractAerodynamicModel, AbstractAerodynamicResidualModel,
       Aerodynamics, QuasiSteadyAerodynamics, NoAerodynamicResidual,
       LinearAerodynamicResidual, fixed_wing_air_data
export AbstractActuatorModel, DirectActuator, FirstOrderActuator
export AbstractMeasurementModel, AbstractEstimator, IdealMeasurement,
       OrnsteinUhlenbeckMeasurement, IdentityEstimator, FunctionEstimator
export AbstractController, LinearStateFeedback, LinearOutputFeedback,
       FunctionController
export AircraftModel, StateLayout, state_layout, state_components,
       initial_state, validate, simulation_signals, simulation_problem, simulate

# Supported low-level interfaces require qualification or an explicit import.
public RealT, Vec3, Vec4, Vec6, Mat3
public pose_dimension, cpem, cpeminv, L_IB, Ξ, Ω, J_η, f_η, ℳ, f_ν,
       rigid_body_dynamics
public gravity, density, environment
public body_wind_gradient, wind_angular_velocity, fixed_wing_angular_velocity,
       wind_triangle, f_w, σ_w, h_w
public F_model, M_model, f_aero, f_res, σ_res, g_res, aerodynamic_loads
public actuator_direct_feedthrough, f_act, h_act
public f_meas, σ_meas, h_meas, f_est, h_est
public f_ctrl, h_ctrl
public simulation_drift!, simulation_diffusion!

end
