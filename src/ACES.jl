module ACES

using StaticArrays
using ComponentArrays
using DifferentialEquations
using Interpolations
using LinearAlgebra

# Include files in dependency order:
include("types.jl")
include("rigid_body.jl")
include("enviroment.jl")
include("wind.jl")
include("aerodynamics.jl")
include("actuators.jl")
include("estimation.jl")
include("control.jl")
include("core.jl")
