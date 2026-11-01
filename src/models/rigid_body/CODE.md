# Julia Code: Rigid-Body Dynamics

This page documents the rigid-body model and attitude-coordinate utilities. See the [rigid-body theory](../theory/rigid-body.md) for frame conventions, kinematics, and Newton–Euler equations.

## Selecting an Attitude Representation

`RigidBody` supports scalar-first unit quaternions by default and roll-pitch-yaw Euler angles as an alternative:

```julia
using LinearAlgebra

inertia = Matrix{Float64}(I, 3, 3)
quaternion_body = RigidBody(1.0, inertia)
euler_body = RigidBody(1.0, inertia; attitude=EulerAngles())
```

The attitude choice changes the rigid-body state dimension. The unit-quaternion state has 13 entries, while the Euler-angle state has 12.

`state_view(model, x_rb)` decodes either representation into a common `RigidBodyState` containing position, rotation matrix, body velocity, and body angular velocity. Other subsystems consume this representation and therefore do not need to know which attitude coordinates are integrated.

## Defining a Custom "Rigid"-Body Model

Aircraft-specific rigid-body formulations normally live in the user's workspace or a separate Julia package:

```julia
struct FlexibleBody <: ACES.AbstractRigidBodyModel
    # application-specific parameters
end

ACES.state_dimension(model::FlexibleBody) = ...
ACES.state_view(model::FlexibleBody, x_rb) = ...
ACES.rigid_body_dynamics(model::FlexibleBody, x_rb, F, M, g) = ...
```

The custom `state_view` result must expose the properties `s`, `R_IB`, `v`, and `ω`, because the environment, wind, and core assembly read those properties. `rigid_body_dynamics` must return a vector with `state_dimension(model)` entries.

## Model Types

```@docs
ACES.AbstractRigidBodyModel
ACES.AbstractAttitudeRepresentation
ACES.EulerAngles
ACES.UnitQuaternion
ACES.RigidBody
ACES.RigidBodyState
ACES.pose_dimension
ACES.state_view
```

## Rotation and Kinematics Utilities

The function names `R_IB`, `q_IB`, and `Θ` intentionally follow the [canonical notation](@ref notation). When a local variable or argument uses the same mathematical name, qualify the function call as `ACES.R_IB(...)`, `ACES.q_IB(...)`, or `ACES.Θ(...)`. The notation reference includes an example that preserves these names for both functions and computed values.

`q_IB(EulerAngles(), Θ)` converts a real three-vector of roll, pitch, and yaw angles in radians to a scalar-first unit quaternion stored as an `SVector{4}`. The convenience form `q_IB(Θ)` delegates to this explicit representation method. It follows the documented rotation order and chooses the sign with a nonnegative scalar component. The conversion remains defined at pitch `±π/2`, where the Euler-angle rate map is singular.

`R_IB(attitude)` accepts a real vector of length three for Euler angles in radians or length four for a scalar-first quaternion, and returns the body-to-inertial rotation as an `SMatrix{3,3}`. The explicit forms `R_IB(EulerAngles(), Θ)` and `R_IB(UnitQuaternion(), q)` remain available. Quaternion inputs are normalized; zero quaternions throw `ArgumentError`. Invalid vector lengths throw `DimensionMismatch` in both `q_IB` and `R_IB`.

`Θ(R_IB)` converts a real three-by-three body-to-inertial rotation matrix to principal Euler roll, pitch, and yaw in radians as an `SVector{3}`. The overloads `Θ(EulerAngles(), angles)` and `Θ(UnitQuaternion(), q)` first use the corresponding `R_IB` method, and `Θ(attitude)` selects between them by vector length, following the same dispatch pattern as `R_IB`. Static and ordinary vectors and matrices are accepted without modifying their inputs. Quaternion normalization, zero-quaternion errors, and vector-length errors follow `R_IB`; matrices with dimensions other than three-by-three raise `DimensionMismatch`. Matrix inputs are assumed to be rotations and are not projected onto SO(3). Floating-point inputs retain their floating-point element type.

The conversion follows the inverse defined in the [rigid-body theory](../theory/rigid-body.md), with pitch in `[-π/2, π/2]` and roll/yaw in `[-π, π]`, without temporal unwrapping. It computes pitch with `atan(-R[3,1], hypot(R[1,1], R[2,1]))`, selects yaw from the first column when the cosine of pitch exceeds eight machine epsilons of the floating-point element type, and otherwise chooses zero yaw. Roll is recovered after removing yaw, avoiding loss of rotation accuracy near pitch `±π/2`. Downstream plotting code can use this public conversion without implementing its own attitude conversion.

```julia
angles = [0.2, -0.1, 0.3]
q = q_IB(EulerAngles(), angles)
R = R_IB(q)
@assert R ≈ R_IB(EulerAngles(), angles)
@assert Θ(R) ≈ angles
@assert Θ(UnitQuaternion(), q) ≈ angles
```

```@docs
ACES.cpem
ACES.cpeminv
ACES.q_IB
ACES.R_IB
ACES.Θ
ACES.L_IB
ACES.Ξ
ACES.Ω
ACES.J_η
ACES.f_η
```

## Dynamics

```@docs
ACES.ℳ
ACES.f_ν
ACES.rigid_body_dynamics
```
