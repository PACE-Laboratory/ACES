using ACES
using LinearAlgebra
using StaticArrays
using Test

@testset "Rigid body" begin
    a = @SVector [1.0, 2.0, 3.0]
    b = @SVector [4.0, 5.0, 6.0]
    @test cpem(a) * b ≈ cross(a, b)
    @test cpeminv(cpem(a)) ≈ a

    Θ = @SVector [0.2, -0.1, 0.3]
    R = R_IB(EulerAngles(), Θ)
    @test transpose(R) * R ≈ I
    @test det(R) ≈ 1

    q_IB = @SVector [cos(0.1), sin(0.1), 0.0, 0.0]
    @test transpose(R_IB(UnitQuaternion(), q_IB)) * R_IB(UnitQuaternion(), q_IB) ≈ I
    ω = @SVector [0.2, 0.3, 0.4]
    @test Ξ(q_IB) * ω ≈ Ω(ω) * q_IB

    rb = RigidBody(2.0, Diagonal([1.0, 2.0, 3.0]))
    @test pose_dimension(rb) == 7
    @test state_dimension(rb) == 13
    @test ℳ(rb) ≈ Diagonal([2.0, 2.0, 2.0, 1.0, 2.0, 3.0])

    x_rb = [0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
            0.0, 10.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    rb_state = state_view(rb, x_rb)
    @test rb_state.s == zeros(3)
    @test rb_state.v == [10.0, 0.0, 0.0]
    x_dot = rigid_body_dynamics(
        rb,
        x_rb,
        (@SVector [2.0, 0.0, 0.0]),
        zeros(3),
        (@SVector [0.0, 0.0, 9.81]),
    )
    @test x_dot[1:3] ≈ [10.0, 0.0, 0.0]
    @test x_dot[8:10] ≈ [1.0, 0.0, 9.81]

    rb_euler = RigidBody(1.0, Matrix{Float64}(I, 3, 3); attitude=EulerAngles())
    @test size(J_η(rb_euler, zeros(6))) == (6, 6)
    @test_throws DomainError L_IB((@SVector [0.0, π / 2, 0.0]))
end
