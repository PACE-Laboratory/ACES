using ACES
using StaticArrays
using Test

module TestExportSurface
using ACES
end

@testset "Public namespace" begin
    # Ordinary workflows remain available through `using ACES` alone.
    for name in (:RigidBody, :AircraftModel, :simulate, :state_view, :R_IB, :Θ)
        @test isdefined(TestExportSurface, name)
        @test Base.isexported(ACES, name)
    end
    # Extension hooks and mathematical helpers stay supported without entering
    # the caller's namespace. Subsystem tests explicitly import what they use.
    for name in (:Vec3, :ℳ, :environment, :f_w, :F_model, :h_act, :h_meas,
                 :h_est, :h_ctrl, :simulation_drift!)
        @test Base.ispublic(ACES, name)
        @test !Base.isexported(ACES, name)
        @test !isdefined(TestExportSurface, name)
    end
    for name in (:_resolve_measurement, :_controller_actuator_outputs, :solve)
        @test !Base.ispublic(ACES, name)
        @test !isdefined(TestExportSurface, name)
    end
end

@testset "Shared types" begin
    @test empty_state() isa SVector{0,Float64}
    @test noise_dimension(DirectActuator()) == 0
end
