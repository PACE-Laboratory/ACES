using ACES
using ACES: f_meas, σ_meas, h_meas, f_est, h_est
using LinearAlgebra
using Test

@testset "Estimation" begin
    ideal = IdealMeasurement((x, t) -> x[1:2])
    @test h_meas(ideal, empty_state(), [1.0, 2.0], 0.0) == [1.0, 2.0]
    @test isempty(f_meas(ideal, empty_state(), zeros(2), 0.0))
    @test_throws ArgumentError IdealMeasurement(:unknown)
    selector = IdealMeasurement(:rigid_body)
    @test state_dimension(selector) == 0
    @test noise_dimension(selector) == 0
    @test_throws ArgumentError h_meas(selector, empty_state(), zeros(13), 0.0)

    colored = OrnsteinUhlenbeckMeasurement(
        0.5,
        Matrix{Float64}(I, 2, 2),
        (x, t) -> x,
    )
    @test f_meas(colored, [1.0, -1.0], zeros(2), 0.0) == [-2.0, 2.0]
    @test σ_meas(colored, zeros(2), zeros(2), 0.0) ≈ 2I
    @test h_meas(colored, [0.1, -0.1], [1.0, 2.0], 0.0) == [1.1, 1.9]

    estimator = IdentityEstimator()
    @test h_est(estimator, empty_state(), [4.0, 5.0], zeros(2), 0.0) ==
          [4.0, 5.0]

    functional = FunctionEstimator(
        1,
        (x_est, y, x, t) -> -x_est + y,
        (x_est, y, x, t) -> copy(x_est),
    )
    @test f_est(functional, [1.0], [3.0], zeros(1), 0.0) == [2.0]
    @test h_est(functional, [1.0], [3.0], zeros(1), 0.0) == [1.0]
end
