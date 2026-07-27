using ACES
using DifferentialEquations
using LinearAlgebra
using Test

# This module deliberately lives outside ACES and demonstrates the supported
# end-user extension pattern for an aircraft-specific model.
module TestAircraftModels
import ACES

struct DensityScaledForce <: ACES.AbstractAerodynamicModel end

ACES.state_dimension(::DensityScaledForce) = 0
ACES.F_model(::DensityScaledForce, v_r, ω_r, x_aero, δ, ρ) =
    [ρ * δ[1], 0.0, 0.0]
ACES.M_model(::DensityScaledForce, v_r, ω_r, x_aero, δ, ρ) = zeros(3)
ACES.f_aero(::DensityScaledForce, x_aero, v_r, ω_r, δ, ρ) =
    ACES.empty_state(eltype(x_aero))

"""One-state nominal model used to exercise the complete core state layout."""
struct StatefulForce <: ACES.AbstractAerodynamicModel end

ACES.state_dimension(::StatefulForce) = 1
ACES.F_model(::StatefulForce, v_r, ω_r, x_aero, δ, ρ) =
    zeros(eltype(x_aero), 3)
ACES.M_model(::StatefulForce, v_r, ω_r, x_aero, δ, ρ) =
    zeros(eltype(x_aero), 3)
ACES.f_aero(::StatefulForce, x_aero, v_r, ω_r, δ, ρ) = -x_aero

end

@testset "Core assembly and simulation" begin
    rigid_body = RigidBody(1.0, Matrix{Float64}(I, 3, 3))
    environment_model = Environment(
        UniformGravity(0.0),
        ExponentialDensity(1.0, 0.0, 8_500.0),
    )
    aerodynamics = Aerodynamics(TestAircraftModels.DensityScaledForce())
    measurement = IdealMeasurement((x, t) -> [0.0])
    estimator = IdentityEstimator()
    controller = FunctionController(
        0,
        (x_ctrl, x_rb, δ, ϖ, xhat, t) -> empty_state(eltype(x_ctrl)),
        (x_ctrl, x_rb, δ, ϖ, xhat, t) -> [1.0],
    )

    aircraft = AircraftModel(
        rigid_body=rigid_body,
        environment=environment_model,
        wind=ConstantWind(),
        aerodynamics=aerodynamics,
        actuator=nothing,
        measurement=measurement,
        estimator=estimator,
        controller=controller,
    )

    x_rb = [0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    x0 = initial_state(aircraft; x_rb=x_rb)
    @test state_dimension(aircraft) == 13
    @test length(state_layout(aircraft)) == 13
    @test state_layout(aircraft).actuator == 14:13
    @test state_components(aircraft, x0).x_rb == x_rb
    @test isnothing(validate(aircraft, x0))
    @test_throws DimensionMismatch validate(aircraft, x0[1:end-1])
    nonfinite_x0 = copy(x0)
    nonfinite_x0[1] = NaN
    @test_throws ArgumentError validate(aircraft, nonfinite_x0)

    signals = simulation_signals(aircraft, x0, 0.0)
    @test signals.u == [1.0]
    @test signals.δ == signals.u
    @test signals.y_meas == [0.0]
    @test signals.xhat == [0.0]
    @test signals.F == [1.0, 0.0, 0.0]

    dx = similar(x0)
    simulation_drift!(dx, x0, aircraft, 0.0)
    @test dx[8:10] == [1.0, 0.0, 0.0]

    deterministic_problem = simulation_problem(aircraft, x0, (0.0, 0.01))
    @test deterministic_problem isa ODEProblem
    solution = simulate(aircraft, x0, (0.0, 0.01); abstol=1e-9, reltol=1e-9)
    @test solution.t[end] == 0.01

    colored_measurement = OrnsteinUhlenbeckMeasurement(
        0.5,
        reshape([0.1], 1, 1),
        (x, t) -> [0.0],
    )
    stochastic_aircraft = AircraftModel(
        rigid_body=rigid_body,
        environment=environment_model,
        wind=ConstantWind(),
        aerodynamics=aerodynamics,
        actuator=nothing,
        measurement=colored_measurement,
        estimator=estimator,
        controller=controller,
    )
    stochastic_x0 = initial_state(stochastic_aircraft; x_rb=x_rb)
    @test isnothing(validate(stochastic_aircraft, stochastic_x0))
    stochastic_problem =
        simulation_problem(stochastic_aircraft, stochastic_x0, (0.0, 0.01))
    @test stochastic_problem isa SDEProblem
    G = zeros(length(stochastic_x0), noise_dimension(stochastic_aircraft))
    simulation_diffusion!(G, stochastic_x0, stochastic_aircraft, 0.0)
    @test G[state_layout(stochastic_aircraft).measurement, :] ≈
          reshape([0.2], 1, 1)

    invalid_controller = FunctionController(
        0,
        (x_ctrl, x_rb, δ, ϖ, xhat, t) -> empty_state(eltype(x_ctrl)),
        (x_ctrl, x_rb, δ, ϖ, xhat, t) -> [δ[1]],
    )
    invalid_aircraft = AircraftModel(
        rigid_body=rigid_body,
        environment=environment_model,
        wind=ConstantWind(),
        aerodynamics=aerodynamics,
        actuator=nothing,
        measurement=measurement,
        estimator=estimator,
        controller=invalid_controller,
    )
    invalid_x0 = initial_state(invalid_aircraft; x_rb=x_rb)
    @test_throws ArgumentError simulation_signals(invalid_aircraft, invalid_x0, 0.0)
    @test_throws ArgumentError validate(invalid_aircraft, invalid_x0)

    invalid_drift_controller = FunctionController(
        0,
        (x_ctrl, x_rb, δ, ϖ, xhat, t) -> δ[1:0],
        (x_ctrl, x_rb, δ, ϖ, xhat, t) -> [1.0],
    )
    invalid_drift_aircraft = AircraftModel(
        rigid_body=rigid_body,
        environment=environment_model,
        wind=ConstantWind(),
        aerodynamics=aerodynamics,
        actuator=nothing,
        measurement=measurement,
        estimator=estimator,
        controller=invalid_drift_controller,
    )
    invalid_drift_x0 = initial_state(invalid_drift_aircraft; x_rb=x_rb)
    invalid_dx = similar(invalid_drift_x0)
    @test_throws ArgumentError validate(invalid_drift_aircraft, invalid_drift_x0)
    @test_throws ArgumentError simulation_drift!(
        invalid_dx,
        invalid_drift_x0,
        invalid_drift_aircraft,
        0.0,
    )

    actuator_controller = FunctionController(
        0,
        (x_ctrl, x_rb, δ, ϖ, xhat, t) -> empty_state(eltype(x_ctrl)),
        (x_ctrl, x_rb, δ, ϖ, xhat, t) -> [1.0 + δ[1]],
    )
    actuated_aircraft = AircraftModel(
        rigid_body=rigid_body,
        environment=environment_model,
        wind=ConstantWind(),
        aerodynamics=aerodynamics,
        actuator=FirstOrderActuator(0.5),
        measurement=measurement,
        estimator=estimator,
        controller=actuator_controller,
    )
    actuated_x0 = initial_state(actuated_aircraft; x_rb=x_rb, x_act=[0.25])
    actuated_signals = simulation_signals(actuated_aircraft, actuated_x0, 0.0)
    @test actuated_signals.δ == [0.25]
    @test actuated_signals.u == [1.25]

    # Exercise every state and noise block at once. Distinct initial values
    # make the invariant flat-state order visible without relying only on
    # ranges whose neighboring zero-dimensional blocks can share endpoints.
    ordered_aerodynamics = Aerodynamics(
        TestAircraftModels.StatefulForce(),
        LinearAerodynamicResidual(
            reshape([-1.0], 1, 1),
            reshape([0.2], 1, 1),
            zeros(6, 1),
        ),
    )
    ordered_wind = ShapingFilterWind(
        reshape([-1.0], 1, 1),
        reshape([0.1], 1, 1),
        zeros(3, 1),
        zeros(3, 1),
    )
    ordered_measurement = OrnsteinUhlenbeckMeasurement(
        1.0,
        reshape([0.3], 1, 1),
        (x, t) -> [0.0],
    )
    ordered_estimator = FunctionEstimator(
        1,
        (x_est, y_meas, x, t) -> -x_est,
        (x_est, y_meas, x, t) -> copy(x_est),
    )
    ordered_controller = FunctionController(
        1,
        (x_ctrl, x_rb, δ, ϖ, xhat, t) -> -x_ctrl,
        (x_ctrl, x_rb, δ, ϖ, xhat, t) -> [1.0],
    )
    ordered_aircraft = AircraftModel(
        rigid_body=rigid_body,
        environment=environment_model,
        wind=ordered_wind,
        aerodynamics=ordered_aerodynamics,
        actuator=FirstOrderActuator(0.5),
        measurement=ordered_measurement,
        estimator=ordered_estimator,
        controller=ordered_controller,
    )
    ordered_x0 = initial_state(
        ordered_aircraft;
        x_rb=x_rb,
        x_aero=[14.0],
        x_act=[15.0],
        x_est=[16.0],
        x_ctrl=[17.0],
        x_w=[18.0],
        x_res=[19.0],
        x_meas=[20.0],
    )
    ordered_layout = state_layout(ordered_aircraft)
    @test ordered_layout.rigid_body == 1:13
    @test ordered_layout.aerodynamics == 14:14
    @test ordered_layout.actuator == 15:15
    @test ordered_layout.estimator == 16:16
    @test ordered_layout.controller == 17:17
    @test ordered_layout.wind == 18:18
    @test ordered_layout.residual == 19:19
    @test ordered_layout.measurement == 20:20
    @test ordered_x0[14:20] == collect(14.0:20.0)

    ordered_states = state_components(ordered_aircraft, ordered_x0)
    @test ordered_states.x_aero == [14.0]
    @test ordered_states.x_act == [15.0]
    @test ordered_states.x_est == [16.0]
    @test ordered_states.x_ctrl == [17.0]
    @test ordered_states.x_w == [18.0]
    @test ordered_states.x_res == [19.0]
    @test ordered_states.x_meas == [20.0]

    ordered_dx = similar(ordered_x0)
    simulation_drift!(ordered_dx, ordered_x0, ordered_aircraft, 0.0)
    @test ordered_dx[ordered_layout.aerodynamics] == [-14.0]
    @test ordered_dx[ordered_layout.actuator] == [-28.0]
    @test ordered_dx[ordered_layout.estimator] == [-16.0]
    @test ordered_dx[ordered_layout.controller] == [-17.0]
    @test ordered_dx[ordered_layout.wind] == [-18.0]
    @test ordered_dx[ordered_layout.residual] == [-19.0]
    @test ordered_dx[ordered_layout.measurement] == [-20.0]

    ordered_G = zeros(
        state_dimension(ordered_aircraft),
        noise_dimension(ordered_aircraft),
    )
    simulation_diffusion!(ordered_G, ordered_x0, ordered_aircraft, 0.0)
    @test ordered_G[ordered_layout.wind, 1:1] == reshape([0.1], 1, 1)
    @test ordered_G[ordered_layout.residual, 2:2] == reshape([0.2], 1, 1)
    @test ordered_G[ordered_layout.measurement, 3:3] == reshape([0.3], 1, 1)
end
