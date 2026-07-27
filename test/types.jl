using ACES
using StaticArrays
using Test

@testset "Shared types" begin
    @test empty_state() isa SVector{0,Float64}
    @test noise_dimension(DirectActuator()) == 0
end
