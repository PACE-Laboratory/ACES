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

## Defining a Custom Rigid-Body Model

Aircraft-specific rigid-body formulations normally live in the user's workspace or a separate Julia package:

```julia
struct FlexibleMassRigidBody <: ACES.AbstractRigidBodyModel
    # application-specific parameters
end

ACES.state_dimension(model::FlexibleMassRigidBody) = ...
ACES.state_view(model::FlexibleMassRigidBody, x_rb) = ...
ACES.rigid_body_dynamics(model::FlexibleMassRigidBody, x_rb, F, M, g) = ...
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

```@docs
ACES.cpem
ACES.cpeminv
ACES.R_IB
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
