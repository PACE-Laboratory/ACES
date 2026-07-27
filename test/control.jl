using ACES
using Test

@testset "Control" begin
    static_controller = LinearStateFeedback([2.0 0.0], [0.5], [1.0, 2.0])
    @test h_ctrl(
        static_controller,
        empty_state(),
        nothing,
        zeros(1),
        zeros(6),
        [2.0, 4.0],
        0.0,
    ) == [-1.5]
    @test isempty(f_ctrl(
        static_controller,
        empty_state(),
        nothing,
        zeros(1),
        zeros(6),
        [2.0, 4.0],
        0.0,
    ))

    dynamic = LinearOutputFeedback(
        reshape([-1.0], 1, 1),
        reshape([2.0], 1, 1),
        reshape([3.0], 1, 1),
        reshape([4.0], 1, 1),
        [0.5],
        [1.0],
    )
    @test f_ctrl(dynamic, [2.0], nothing, zeros(1), zeros(6), [3.0], 0.0) ==
          [2.0]
    @test h_ctrl(dynamic, [2.0], nothing, zeros(1), zeros(6), [3.0], 0.0) ==
          [14.5]

    functional = FunctionController(
        1,
        (x_ctrl, x_rb, δ, ϖ, xhat, t) -> -x_ctrl,
        (x_ctrl, x_rb, δ, ϖ, xhat, t) -> x_ctrl + xhat,
    )
    @test f_ctrl(functional, [2.0], nothing, zeros(1), zeros(6), [3.0], 0.0) ==
          [-2.0]
    @test h_ctrl(functional, [2.0], nothing, zeros(1), zeros(6), [3.0], 0.0) ==
          [5.0]
end
