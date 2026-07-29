using ACES
using StaticArrays
using Test

@testset "Environment" begin
    # Arbitrary NED position
    s = @SVector [100.0, 500.0, -1000.0]

    # Altitude from position vector
    @test altitude(s) == 1000.0
    @test altitude(s, 100.0) == 1100.0

    # Uniform gravity model
    @test gravity(UniformGravity(), s, 0.0) == [0.0, 0.0, 9.81]
    @test gravity(UniformGravity(9.807), s, 0.0) == [0.0, 0.0, 9.807]

    # Constant density model
    @test density(ConstantDensity(), 0.0) == 1.225
    @test density(ConstantDensity(1.1), 0.0) == 1.1

    # Exponential density model
    exponential = ExponentialDensity(1.225, 0.0, 8500.0)
    @test density(exponential, 0.0) == 1.225
    @test density(exponential, 8500.0) ≈ 1.225 / ℯ

    # NASA Glenn atmospheric model [metric]
    @test density(NASAMetricAtmosphere(), 0.0) ≈ 1.225 atol=0.01

    # Environment constructor and interface
    env = Environment(UniformGravity(), exponential; h0=0.0)
    output = environment(env, s, 2.0)
    @test output.g == [0.0, 0.0, 9.81]
    @test output.ρ < 1.225
end
