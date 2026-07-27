using ACES
using Test

@testset "Actuators" begin
    direct = DirectActuator()
    @test actuator_direct_feedthrough(direct)
    @test isempty(f_act(direct, empty_state(), zeros(3), zeros(3), [0.2, -0.1]))
    @test h_act(direct, empty_state(), zeros(3), zeros(3), [0.2, -0.1]) ==
          [0.2, -0.1]

    first_order = FirstOrderActuator([0.5, 1.0])
    @test !actuator_direct_feedthrough(first_order)
    @test state_dimension(first_order) == 2
    @test f_act(first_order, [0.0, 0.5], zeros(3), zeros(3), [1.0, 1.0]) ==
          [2.0, 0.5]
    @test h_act(first_order, [0.0, 0.5], zeros(3), zeros(3), [1.0, 1.0]) ==
          [0.0, 0.5]
end
