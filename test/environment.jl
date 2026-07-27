using ACES
using StaticArrays
using Test

@testset "Environment" begin
    s = @SVector [0.0, 0.0, -1_000.0]
    @test altitude(s, 100.0) == 1_100.0
    @test gravity(UniformGravity(), s, 0.0) == [0.0, 0.0, 9.81]

    exponential = ExponentialDensity(1.225, 0.0, 8_500.0)
    @test density(exponential, 0.0) == 1.225
    @test density(exponential, 8_500.0) ≈ 1.225 / ℯ
    @test density(NASAMetricAtmosphere(), 0.0) ≈ 1.225 atol=0.01

    env = Environment(UniformGravity(), exponential; h0=0.0)
    output = environment(env, s, 2.0)
    @test output.g == [0.0, 0.0, 9.81]
    @test output.ρ < 1.225
end
