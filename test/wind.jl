using ACES
using LinearAlgebra
using StaticArrays
using Test

@testset "Wind" begin
    rb_state = RigidBodyState(
        (@SVector [1.0, 2.0, 3.0]),
        SMatrix{3,3,Float64,9}(I),
        (@SVector [10.0, 0.0, 0.0]),
        (@SVector [0.1, 0.2, 0.3]),
    )

    constant = ConstantWind([2.0, 0.0, 0.0], [0.0, 0.0, 0.1])
    ϖ = h_w(constant, empty_state(), rb_state, 0.0)
    relative = wind_triangle(rb_state, ϖ)
    @test relative.v_r ≈ [8.0, 0.0, 0.0]
    @test relative.ω_r ≈ [0.1, 0.2, 0.2]

    Φ = cpem((@SVector [1.0, 2.0, 3.0]))
    @test wind_angular_velocity(Φ) ≈ [1.0, 2.0, 3.0]
    @test body_wind_gradient(Φ, Matrix{Float64}(I, 3, 3)) ≈ Φ

    frozen = FrozenWindField(
        (s, t) -> s + (@SVector [t, 0.0, 0.0]),
        (s, t) -> zeros(3, 3),
    )
    @test h_w(frozen, empty_state(), rb_state, 2.0) ≈
          [3.0, 2.0, 3.0, 0.0, 0.0, 0.0]

    shaping = ShapingFilterWind(
        [-1.0 0.0; 0.0 -2.0],
        Matrix{Float64}(I, 2, 2),
        [1.0 0.0; 0.0 1.0; 0.0 0.0],
        [0.0 0.0; 0.0 0.0; 1.0 1.0],
        [1.0, 0.0, 0.0],
    )
    @test f_w(shaping, [2.0, 3.0], rb_state, 0.0) ≈ [-2.0, -6.0]
    @test σ_w(shaping, [2.0, 3.0], rb_state, 0.0) ≈ I
    @test h_w(shaping, [2.0, 3.0], rb_state, 0.0) ≈
          [3.0, 3.0, 0.0, 0.0, 0.0, 5.0]
end
