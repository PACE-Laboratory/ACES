# =============================================================================
# Simulation assembly and numerical integration
# =============================================================================
#
# ACES keeps reusable subsystem interfaces and baseline implementations under
# `src/models/`. Aircraft-specific models should normally remain in the end
# user's workspace (or in a separate Julia package), not be copied into ACES.
#
# A typical user file extends the public interface:
#
#     # MyAircraft.jl
#     struct MyAerodynamics <: ACES.AbstractAerodynamicModel
#         # aircraft geometry and coefficient data
#     end
#     ACES.state_dimension(::MyAerodynamics) = 0
#     ACES.F_model(model::MyAerodynamics, v_r, ω_r, x_aero, δ, ρ) = ...
#     ACES.M_model(model::MyAerodynamics, v_r, ω_r, x_aero, δ, ρ) = ...
#     ACES.f_aero(::MyAerodynamics, x_aero, v_r, ω_r, δ, ρ) =
#         ACES.empty_state(eltype(x_aero))
#
# The simulation script loads that file and selects concrete model instances:
#
#     using ACES
#     include("MyAircraft.jl")
#     aircraft = AircraftModel(
#         rigid_body = RigidBody(...),
#         environment = Environment(...),
#         wind = ConstantWind(),
#         aerodynamics = Aerodynamics(MyAerodynamics(...)),
#         actuator = nothing,              # core uses δ = u
#         measurement = MyMeasurement(...),
#         estimator = MyEstimator(...),
#         controller = MyController(...),
#     )
#
# With `actuator=nothing`, both controller methods must ignore their `δ`
# argument. Core checks this assumption by passing a sentinel that throws if
# the controller attempts to inspect it.
#
# This explicit composition is the only model-selection mechanism: no global
# registry or source-file editing is required.

# -----------------------------------------------------------------------------
# Model composition
# -----------------------------------------------------------------------------

"""
    AircraftModel(; rigid_body, environment, wind, aerodynamics, actuator,
                    measurement, estimator, controller)

Concrete selection of every subsystem used by one simulation.

Set `actuator=nothing` to omit actuator dynamics; core then uses `δ = u` and
allocates no actuator state. In that configuration, `f_ctrl` and `h_ctrl` must
not use their `δ` argument. Core enforces the assumption by passing a sentinel
that throws `ArgumentError` if inspected.
"""
struct AircraftModel{RB,E,W,A,ACT,MEAS,EST,CTRL}
    rigid_body::RB
    environment::E
    wind::W
    aerodynamics::A
    actuator::ACT
    measurement::MEAS
    estimator::EST
    controller::CTRL
end

function AircraftModel(
    ;
    rigid_body::AbstractRigidBodyModel,
    environment::AbstractEnvironmentModel=Environment(),
    wind::AbstractWindModel=ConstantWind(),
    aerodynamics::Aerodynamics,
    actuator::Union{Nothing,AbstractActuatorModel}=nothing,
    measurement::AbstractMeasurementModel,
    estimator::AbstractEstimator,
    controller::AbstractController,
)
    return AircraftModel(
        rigid_body,
        environment,
        wind,
        aerodynamics,
        actuator,
        measurement,
        estimator,
        controller,
    )
end

# -----------------------------------------------------------------------------
# Flat integration-state layout
# -----------------------------------------------------------------------------

"""Ranges identifying each subsystem inside the flat simulation state `x`."""
struct StateLayout
    rigid_body::UnitRange{Int}
    aerodynamics::UnitRange{Int}
    actuator::UnitRange{Int}
    estimator::UnitRange{Int}
    controller::UnitRange{Int}
    wind::UnitRange{Int}
    residual::UnitRange{Int}
    measurement::UnitRange{Int}
    length::Int
end

Base.length(layout::StateLayout) = layout.length

@inline _state_range(first::Int, count::Integer) =
    first:(first + Int(count) - 1)

"""Return the canonical flat-state layout implied by an `AircraftModel`."""
function state_layout(model::AircraftModel)
    first = 1

    # Deterministic state blocks always precede the stochastic state blocks.
    # Keep this allocation order synchronized with `initial_state`,
    # `state_components`, and the assembled drift documented in `THEORY.md`.
    rigid_body = _state_range(first, state_dimension(model.rigid_body))
    first = last(rigid_body) + 1
    aerodynamics = _state_range(first, state_dimension(model.aerodynamics.nominal))
    first = last(aerodynamics) + 1

    n_act = isnothing(model.actuator) ? 0 : state_dimension(model.actuator)
    actuator = _state_range(first, n_act)
    first = last(actuator) + 1
    estimator = _state_range(first, state_dimension(model.estimator))
    first = last(estimator) + 1
    controller = _state_range(first, state_dimension(model.controller))
    first = last(controller) + 1

    # Wind, aerodynamic-residual, and measurement states are grouped last
    # because these are the only subsystem blocks driven by Wiener processes.
    wind = _state_range(first, state_dimension(model.wind))
    first = last(wind) + 1
    residual = _state_range(first, state_dimension(model.aerodynamics.residual))
    first = last(residual) + 1
    measurement = _state_range(first, state_dimension(model.measurement))
    first = last(measurement) + 1

    return StateLayout(
        rigid_body,
        aerodynamics,
        actuator,
        estimator,
        controller,
        wind,
        residual,
        measurement,
        first - 1,
    )
end

state_dimension(model::AircraftModel) = state_layout(model).length
noise_dimension(model::AircraftModel) =
    noise_dimension(model.wind) +
    noise_dimension(model.aerodynamics.residual) +
    noise_dimension(model.measurement)

"""Return non-copying, named views of every subsystem state in `x`."""
function state_components(model::AircraftModel, x::AbstractVector)
    layout = state_layout(model)
    length(x) == layout.length ||
        throw(DimensionMismatch("x must have length $(layout.length)"))
    return (
        x_rb=@view(x[layout.rigid_body]),
        x_aero=@view(x[layout.aerodynamics]),
        x_act=@view(x[layout.actuator]),
        x_est=@view(x[layout.estimator]),
        x_ctrl=@view(x[layout.controller]),
        x_w=@view(x[layout.wind]),
        x_res=@view(x[layout.residual]),
        x_meas=@view(x[layout.measurement]),
    )
end

@inline function _check_initial_state(name::Symbol, value, expected::Int)
    length(value) == expected ||
        throw(DimensionMismatch("$name must have length $expected"))
    return value
end

"""
    initial_state(model; x_rb, x_aero, x_act, x_est, x_ctrl,
                         x_w, x_res, x_meas)

Assemble subsystem initial conditions in the canonical flat-state order.
Zero-state subsystems default to empty vectors; nonzero optional subsystem
states default to zero vectors of the required size.
"""
function initial_state(
    model::AircraftModel;
    x_rb::AbstractVector,
    x_aero::AbstractVector=zeros(RealT, state_dimension(model.aerodynamics.nominal)),
    x_act::AbstractVector=zeros(
        RealT,
        isnothing(model.actuator) ? 0 : state_dimension(model.actuator),
    ),
    x_est::AbstractVector=zeros(RealT, state_dimension(model.estimator)),
    x_ctrl::AbstractVector=zeros(RealT, state_dimension(model.controller)),
    x_w::AbstractVector=zeros(RealT, state_dimension(model.wind)),
    x_res::AbstractVector=zeros(RealT, state_dimension(model.aerodynamics.residual)),
    x_meas::AbstractVector=zeros(RealT, state_dimension(model.measurement)),
)
    _check_initial_state(:x_rb, x_rb, state_dimension(model.rigid_body))
    _check_initial_state(:x_aero, x_aero, state_dimension(model.aerodynamics.nominal))
    _check_initial_state(
        :x_act,
        x_act,
        isnothing(model.actuator) ? 0 : state_dimension(model.actuator),
    )
    _check_initial_state(:x_est, x_est, state_dimension(model.estimator))
    _check_initial_state(:x_ctrl, x_ctrl, state_dimension(model.controller))
    _check_initial_state(:x_w, x_w, state_dimension(model.wind))
    _check_initial_state(:x_res, x_res, state_dimension(model.aerodynamics.residual))
    _check_initial_state(:x_meas, x_meas, state_dimension(model.measurement))
    return vcat(x_rb, x_aero, x_act, x_est, x_ctrl, x_w, x_res, x_meas)
end

# -----------------------------------------------------------------------------
# Single-pass controller and actuator outputs
# -----------------------------------------------------------------------------

struct _UnavailableSignal{T} <: AbstractVector{T}
    message::String
end

Base.size(signal::_UnavailableSignal) = throw(ArgumentError(signal.message))
Base.getindex(signal::_UnavailableSignal, indices...) =
    throw(ArgumentError(signal.message))

@inline function _unavailable_actuator_output(::Type{T}) where {T}
    return _UnavailableSignal{T}(
        "the controller may not use δ when no actuator output is available",
    )
end

@inline function _unavailable_control_input(::Type{T}) where {T}
    return _UnavailableSignal{T}(
        "h_act may not use u when actuator_direct_feedthrough(model) is false",
    )
end

@inline function _require_vector(name, value)
    value isa AbstractVector ||
        throw(ArgumentError("$name must return an AbstractVector"))
    return value
end

function _controller_actuator_outputs(model, states, xhat, v_r, ω_r, ϖ, T, t)
    if isnothing(model.actuator)
        # With no actuator model there is no independently available δ. The
        # controller is evaluated without it, then the ideal relation δ = u is
        # applied. The sentinel turns the modeling assumption into a runtime
        # check instead of silently supplying a guessed value.
        unavailable_δ = _unavailable_actuator_output(T)
        u = _require_vector(
            "h_ctrl",
            h_ctrl(
                model.controller,
                states.x_ctrl,
                states.x_rb,
                unavailable_δ,
                ϖ,
                xhat,
                t,
            ),
        )
        return (u=u, δ=copy(u))
    end

    if actuator_direct_feedthrough(model.actuator)
        # A direct-feedthrough actuator needs the current command, so h_ctrl
        # must not use δ. Evaluate u first and δ second.
        unavailable_δ = _unavailable_actuator_output(T)
        u = _require_vector(
            "h_ctrl",
            h_ctrl(
                model.controller,
                states.x_ctrl,
                states.x_rb,
                unavailable_δ,
                ϖ,
                xhat,
                t,
            ),
        )
        δ = _require_vector(
            "h_act",
            h_act(model.actuator, states.x_act, v_r, ω_r, u),
        )
        return (u=u, δ=δ)
    end

    # A state-output actuator supplies δ without the current command. Evaluate
    # δ first so controllers may use the physical actuator configuration.
    unavailable_u = _unavailable_control_input(T)
    δ = _require_vector(
        "h_act",
        h_act(model.actuator, states.x_act, v_r, ω_r, unavailable_u),
    )
    u = _require_vector(
        "h_ctrl",
        h_ctrl(
            model.controller,
            states.x_ctrl,
            states.x_rb,
            δ,
            ϖ,
            xhat,
            t,
        ),
    )
    return (u=u, δ=δ)
end

"""
    simulation_signals(model, x, t)

Evaluate derived environment, wind, actuator, estimation, control, and
aerodynamic signals at one simulation state and time.
"""
function simulation_signals(model::AircraftModel, x::AbstractVector, t::Real)
    states = state_components(model, x)
    rb_state = state_view(model.rigid_body, states.x_rb)
    env = environment(model.environment, rb_state.s, t)
    ϖ = h_w(model.wind, states.x_w, rb_state, t)
    relative = wind_triangle(rb_state, ϖ)

    # Estimation depends only on the simulation state and time. Aircraft-
    # specific x -> δ reconstruction, when required, is encapsulated inside
    # these model methods rather than represented as a core argument.
    y_meas = h_meas(model.measurement, states.x_meas, x, t)
    xhat = h_est(model.estimator, states.x_est, y_meas, x, t)

    outputs = _controller_actuator_outputs(
        model,
        states,
        xhat,
        relative.v_r,
        relative.ω_r,
        ϖ,
        eltype(x),
        t,
    )
    loads = aerodynamic_loads(
        model.aerodynamics,
        states.x_aero,
        states.x_res,
        relative.v_r,
        relative.ω_r,
        outputs.δ,
        env.ρ,
    )

    return (
        rb_state=rb_state,
        g=env.g,
        ρ=env.ρ,
        ϖ=ϖ,
        v_r=relative.v_r,
        ω_r=relative.ω_r,
        u=outputs.u,
        δ=outputs.δ,
        y_meas=y_meas,
        xhat=xhat,
        F=loads.F,
        M=loads.M,
    )
end

# -----------------------------------------------------------------------------
# ODE/SDE drift and diffusion assembly
# -----------------------------------------------------------------------------

"""In-place drift function used by `ODEProblem` and `SDEProblem`."""
function simulation_drift!(
    dx::AbstractVector,
    x::AbstractVector,
    model::AircraftModel,
    t::Real,
)
    layout = state_layout(model)
    states = state_components(model, x)
    signals = simulation_signals(model, x, t)

    dx[layout.rigid_body] .= rigid_body_dynamics(
        model.rigid_body,
        states.x_rb,
        signals.F,
        signals.M,
        signals.g,
    )
    dx[layout.aerodynamics] .= f_aero(
        model.aerodynamics.nominal,
        states.x_aero,
        signals.v_r,
        signals.ω_r,
        signals.δ,
        signals.ρ,
    )

    if !isnothing(model.actuator)
        dx[layout.actuator] .= f_act(
            model.actuator,
            states.x_act,
            signals.v_r,
            signals.ω_r,
            signals.u,
        )
    end

    dx[layout.estimator] .= f_est(
        model.estimator,
        states.x_est,
        signals.y_meas,
        x,
        t,
    )
    # The no-actuator assumption applies to both controller methods. Passing
    # the same sentinel here ensures f_ctrl cannot read δ after h_ctrl has
    # already established the command used by δ = u.
    δ_ctrl = isnothing(model.actuator) ?
             _unavailable_actuator_output(eltype(x)) :
             signals.δ
    dx[layout.controller] .= f_ctrl(
        model.controller,
        states.x_ctrl,
        states.x_rb,
        δ_ctrl,
        signals.ϖ,
        signals.xhat,
        t,
    )

    # Stochastic subsystem drifts occupy the final three state blocks, in the
    # same order as their independent Wiener-process column blocks.
    dx[layout.wind] .= f_w(model.wind, states.x_w, signals.rb_state, t)
    dx[layout.residual] .= f_res(
        model.aerodynamics.residual,
        states.x_res,
        signals.v_r,
        signals.ω_r,
        states.x_aero,
        signals.δ,
        signals.ρ,
    )
    dx[layout.measurement] .= f_meas(
        model.measurement,
        states.x_meas,
        x,
        t,
    )
    return nothing
end

@inline function _write_diffusion_block!(G, rows, columns, block, name)
    size(block) == (length(rows), length(columns)) ||
        throw(DimensionMismatch(
            "$name diffusion must be $(length(rows)) × $(length(columns))",
        ))
    G[rows, columns] .= block
    return nothing
end

"""In-place block diffusion function used by `SDEProblem`."""
function simulation_diffusion!(
    G::AbstractMatrix,
    x::AbstractVector,
    model::AircraftModel,
    t::Real,
)
    layout = state_layout(model)
    size(G) == (layout.length, noise_dimension(model)) ||
        throw(DimensionMismatch("G has the wrong size"))
    fill!(G, zero(eltype(G)))

    states = state_components(model, x)
    signals = simulation_signals(model, x, t)
    first_column = 1

    n_wiener = noise_dimension(model.wind)
    columns = _state_range(first_column, n_wiener)
    _write_diffusion_block!(
        G,
        layout.wind,
        columns,
        σ_w(model.wind, states.x_w, signals.rb_state, t),
        "wind",
    )
    first_column = last(columns) + 1

    n_wiener = noise_dimension(model.aerodynamics.residual)
    columns = _state_range(first_column, n_wiener)
    _write_diffusion_block!(
        G,
        layout.residual,
        columns,
        σ_res(
            model.aerodynamics.residual,
            states.x_res,
            signals.v_r,
            signals.ω_r,
            states.x_aero,
            signals.δ,
            signals.ρ,
        ),
        "aerodynamic residual",
    )
    first_column = last(columns) + 1

    n_wiener = noise_dimension(model.measurement)
    columns = _state_range(first_column, n_wiener)
    _write_diffusion_block!(
        G,
        layout.measurement,
        columns,
        σ_meas(model.measurement, states.x_meas, x, t),
        "measurement",
    )
    return nothing
end

# -----------------------------------------------------------------------------
# Model and initial-state preflight validation
# -----------------------------------------------------------------------------

@inline function _validate_dimension(name, kind, dimension)
    dimension isa Integer ||
        throw(ArgumentError("$name $kind dimension must be an integer"))
    dimension >= 0 ||
        throw(ArgumentError("$name $kind dimension must be nonnegative"))
    return nothing
end

function _validate_model_dimensions(model::AircraftModel)
    state_models = (
        ("rigid-body", model.rigid_body),
        ("wind", model.wind),
        ("nominal aerodynamic", model.aerodynamics.nominal),
        ("aerodynamic residual", model.aerodynamics.residual),
        ("measurement", model.measurement),
        ("estimator", model.estimator),
        ("controller", model.controller),
    )
    for (name, subsystem) in state_models
        _validate_dimension(name, "state", state_dimension(subsystem))
    end
    if !isnothing(model.actuator)
        _validate_dimension(
            "actuator",
            "state",
            state_dimension(model.actuator),
        )
        actuator_direct_feedthrough(model.actuator) isa Bool ||
            throw(ArgumentError(
                "actuator_direct_feedthrough(model) must return Bool",
            ))
    end

    noise_models = (
        ("wind", model.wind),
        ("aerodynamic residual", model.aerodynamics.residual),
        ("measurement", model.measurement),
    )
    for (name, subsystem) in noise_models
        _validate_dimension(name, "noise", noise_dimension(subsystem))
    end
    return nothing
end

"""
    validate(model, x0; t=0)

Preflight an assembled aircraft model at initial state `x0` and time `t`.

Validation checks subsystem state and noise dimensions, requires finite real
initial-state entries, and evaluates the assembled drift and any diffusion
blocks. This catches missing methods, incompatible output dimensions,
non-finite initial outputs, and controller/actuator causality violations before
the numerical problem is constructed. It returns `nothing` on success.

Validation establishes consistency only at the supplied state and time;
user-defined models remain responsible for valid behavior elsewhere.
"""
function validate(
    model::AircraftModel,
    x0::AbstractVector;
    t::Real=0,
)
    _validate_model_dimensions(model)
    isfinite(t) || throw(ArgumentError("validation time t must be finite"))
    eltype(x0) <: Real ||
        throw(ArgumentError("x0 must have a real scalar element type"))

    expected = state_dimension(model)
    length(x0) == expected ||
        throw(DimensionMismatch("x0 must have length $expected"))
    all(isfinite, x0) ||
        throw(ArgumentError("x0 must contain only finite values"))

    state = collect(x0)
    dx = similar(state)
    simulation_drift!(dx, state, model, t)
    all(isfinite, dx) ||
        throw(ArgumentError(
            "the assembled drift must contain only finite values",
        ))

    n_noise = noise_dimension(model)
    if !iszero(n_noise)
        G = zeros(eltype(state), length(state), n_noise)
        simulation_diffusion!(G, state, model, t)
        all(isfinite, G) ||
            throw(ArgumentError(
                "the assembled diffusion must contain only finite values",
            ))
    end
    return nothing
end

# -----------------------------------------------------------------------------
# SciML problem construction and solve
# -----------------------------------------------------------------------------

"""
    simulation_problem(model, x0, tspan)

Construct an `ODEProblem` when all selected models are deterministic, or a
general-noise `SDEProblem` when any wind, residual, or measurement model has
nonzero diffusion dimension. The assembled model is validated at `x0` and the
initial time before the problem is returned.
"""
function simulation_problem(
    model::AircraftModel,
    x0::AbstractVector,
    tspan::Tuple{<:Real,<:Real},
)
    all(isfinite, tspan) ||
        throw(ArgumentError("tspan endpoints must be finite"))
    validate(model, x0; t=first(tspan))
    state = collect(x0)
    if iszero(noise_dimension(model))
        return ODEProblem(simulation_drift!, state, tspan, model)
    end

    prototype = zeros(eltype(state), length(state), noise_dimension(model))
    return SDEProblem(
        simulation_drift!,
        simulation_diffusion!,
        state,
        tspan,
        model;
        noise_rate_prototype=prototype,
    )
end

"""
    simulate(model, x0, tspan; solver=nothing, kwargs...)

Build and solve the appropriate SciML problem. By default, deterministic
problems use the standard automatic ODE algorithm and general-noise stochastic
problems use adaptive Euler-Maruyama. Supplying `solver` overrides that choice;
all other keywords are forwarded to `solve`.
"""
function simulate(
    model::AircraftModel,
    x0::AbstractVector,
    tspan::Tuple{<:Real,<:Real};
    solver=nothing,
    kwargs...,
)
    problem = simulation_problem(model, x0, tspan)
    if !isnothing(solver)
        return solve(problem, solver; kwargs...)
    end
    default_solver =
        problem isa ODEProblem ? DefaultODEAlgorithm() : LambaEM()
    return solve(problem, default_solver; kwargs...)
end
