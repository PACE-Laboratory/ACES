using ACES
using StaticArrays
using Test

@testset "Aerodynamics" begin
    nominal = QuasiSteadyAerodynamics(
        (v_r, ω_r, x_aero, δ, ρ) -> -ρ * v_r,
        (v_r, ω_r, x_aero, δ, ρ) -> -ρ * ω_r,
    )
    residual = LinearAerodynamicResidual(
        reshape([-1.0], 1, 1),
        reshape([0.5], 1, 1),
        reshape(collect(1.0:6.0), 6, 1),
    )
    model = Aerodynamics(nominal, residual)
    loads = aerodynamic_loads(
        model,
        empty_state(),
        [2.0],
        [3.0, 0.0, 4.0],
        [0.1, 0.2, 0.3],
        [0.0],
        2.0,
    )
    @test loads.F ≈ [-4.0, 4.0, -2.0]
    @test loads.M ≈ [7.8, 9.6, 11.4]
    @test f_res(
        residual,
        [2.0],
        zeros(3),
        zeros(3),
        empty_state(),
        [0.0],
        2.0,
    ) == [-2.0]
    @test size(g_res(
        residual,
        [2.0],
        zeros(3),
        zeros(3),
        empty_state(),
        [0.0],
        2.0,
    )) == (6, 1)

    air = fixed_wing_air_data((@SVector [3.0, 0.0, 4.0]))
    @test air.V == 5.0
    @test air.α ≈ atan(4, 3)
    @test air.β == 0.0
    @test fixed_wing_air_data(zeros(3)) == (V=0.0, α=0.0, β=0.0)
end
