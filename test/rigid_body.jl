using ACES
using ACES: pose_dimension, cpem, cpeminv, L_IB, Ξ, Ω, J_η, ℳ, rigid_body_dynamics
using LinearAlgebra
using StaticArrays
using Test

@testset "Rigid body" begin
    # Cross product equivalent matrix
    a = @SVector [1.0, 2.0, 3.0]
    b = @SVector [4.0, 5.0, 6.0]
    @test cpem(a)*b ≈ cross(a, b)
    @test cpeminv(cpem(a)) ≈ a

    # Rotation matrix from Euler angles
    Θ = @SVector [0.2, -0.1, 0.3]
    R = R_IB(EulerAngles(), Θ)
    @test transpose(R)*R ≈ I
    @test det(R) ≈ 1

    @testset "Attitude conversions" begin
        @test ACES.q_IB(zeros(3)) ≈ [1, 0, 0, 0]
        for axis in 1:3
            angles = zeros(3)
            angles[axis] = π / 2
            expected = zeros(4)
            expected[1] = cos(π / 4)
            expected[axis + 1] = sin(π / 4)
            @test ACES.q_IB(angles) ≈ expected
        end

        # Include half turns, Euler rate singularities, and a negative scalar
        # before sign selection to check rotation order and the sign convention.
        for angles in ([0.2, -0.1, 0.3], [π, 0, 0], [0, π, 0], [0, 0, π],
                       [0.4, π / 2, -0.8], [-0.7, -π / 2, 0.9],
                       [3π / 2, 0, 0], [2π, -3π, 4π])
            q = ACES.q_IB(EulerAngles(), angles)
            expected = R_IB(EulerAngles(), angles)
            @test q isa SVector{4}
            @test norm(q) ≈ 1
            @test q[1] >= 0
            @test R_IB(UnitQuaternion(), q) ≈ expected
            @test R_IB(q) ≈ expected
            @test R_IB(collect(q)) ≈ expected
            @test R_IB(-3 .* collect(q)) ≈ expected
            @test R_IB(angles) ≈ expected
            @test R_IB(SVector{3}(angles)) ≈ expected
            @test ACES.q_IB(SVector{3}(angles)) ≈ q
            @test ACES.q_IB(angles) ≈ q
        end

        @test ACES.q_IB(Float32[0.2, -0.1, 0.3]) isa SVector{4,Float32}
        @test ACES.q_IB(EulerAngles(), Float32[0.2, -0.1, 0.3]) isa SVector{4,Float32}
        @test R_IB(Float32[1, 0, 0, 0]) isa SMatrix{3,3,Float32}
        for n in (0, 2, 4, 5)
            @test_throws DimensionMismatch ACES.q_IB(zeros(n))
            @test_throws DimensionMismatch ACES.q_IB(EulerAngles(), zeros(n))
        end
        for n in (0, 2, 5)
            @test_throws DimensionMismatch R_IB(zeros(n))
        end
        @test_throws ArgumentError R_IB(zeros(4))
    end

    # Rotation matrix from a unit quaternion
    q_IB = @SVector [4.0, 3.0, 2.0, 1.0]
    q_IB = normalize(q_IB)
    R = R_IB(UnitQuaternion(), q_IB)
    @test transpose(R)*R ≈ I
    @test det(R) ≈ 1

    # Equivalence of quaternion kinematics formulations
    ω = @SVector [0.2, 0.3, 0.4]
    @test Ξ(q_IB)*ω ≈ Ω(ω)*q_IB

    # Dimensions
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

@testset "Euler-angle conversions" begin
    cases = ([0.0, 0.0, 0.0], [0.2, -0.3, 0.4], [π, 0.0, 0.0],
             [0.0, 0.0, π], [4.2, 0.3, -4.5], [1.0, 2.1, -2.0],
             [0.7, π/2, -0.8], [0.7, -π/2, -0.8],
             [0.7, π/2 - 1e-7, -0.8], [0.7, -π/2 + 1e-7, -0.8])
    for T in (Float32, Float64), (i, value) in enumerate(cases)
        angles = T.(value)
        q = ACES.q_IB(EulerAngles(), angles)
        expected = R_IB(EulerAngles(), angles)
        for input in (expected, Matrix(expected), angles, SVector{3}(angles), q, collect(q))
            before = copy(input)
            recovered = ACES.Θ(input)
            @test recovered isa SVector{3,T}
            @test all(isfinite, recovered)
            @test abs(recovered[1]) <= π + 4eps(T)
            @test abs(recovered[2]) <= π/2 + 4eps(T)
            @test abs(recovered[3]) <= π + 4eps(T)
            @test R_IB(EulerAngles(), recovered) ≈ expected atol=50eps(T) rtol=50eps(T)
            @test input == before
            if i in (7, 8)
                @test recovered[3] == 0
            end
        end
        @test ACES.Θ(EulerAngles(), angles) ≈ ACES.Θ(angles)
        @test ACES.Θ(UnitQuaternion(), collect(q)) ≈ ACES.Θ(q)
        for scale in (T(-1), T(2), T(-3))
            recovered = ACES.Θ(UnitQuaternion(), scale .* q)
            @test recovered isa SVector{3,T}
            @test R_IB(EulerAngles(), recovered) ≈ expected atol=50eps(T) rtol=50eps(T)
        end
    end
    angles = [0.2, -0.3, 0.4]
    @test ACES.Θ(R_IB(EulerAngles(), angles)) ≈ angles
    @test ACES.Θ(UnitQuaternion(), ACES.q_IB(angles)) ≈ angles
    @test ACES.Θ(Matrix{Int}(I, 3, 3)) isa SVector{3,Float64}
    @test ACES.Θ(Matrix{Int}(I, 3, 3)) == zeros(3)
    for n in (0, 1, 2, 5)
        @test_throws DimensionMismatch ACES.Θ(zeros(n))
    end
    for n in (0, 2, 4, 5)
        @test_throws DimensionMismatch ACES.Θ(EulerAngles(), zeros(n))
    end
    for n in (0, 2, 3, 5)
        @test_throws DimensionMismatch ACES.Θ(UnitQuaternion(), zeros(n))
    end
    for shape in ((0, 0), (2, 2), (3, 4), (4, 3), (4, 4))
        @test_throws DimensionMismatch ACES.Θ(zeros(shape))
    end
    @test_throws ArgumentError ACES.Θ(zeros(4))
    @test_throws ArgumentError ACES.Θ(UnitQuaternion(), zeros(4))
end
