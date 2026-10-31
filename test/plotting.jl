using ACES
using LinearAlgebra
using StaticArrays
using Test

@testset "Plotting is optional" begin
    @test Base.get_extension(ACES, :ACESPlotsExt) === nothing
    @test_throws ArgumentError plot_rigid_body(nothing, nothing)
    @test_throws ArgumentError plot_controls(nothing, nothing)
end

# Use an off-screen backend for rendering tests; the public functions themselves
# never change the caller's backend or display settings.
ENV["GKSwstype"] = "100"
using Plots

# Forward rotation composition for inputs, independent of the inverse plotting
# conversion under test.
function plotting_quaternion(angles)
    φ, θ, ψ = angles
    sφ, cφ = sincos(φ / 2)
    sθ, cθ = sincos(θ / 2)
    sψ, cψ = sincos(ψ / 2)
    return [cφ*cθ*cψ + sφ*sθ*sψ, sφ*cθ*cψ - cφ*sθ*sψ,
            cφ*sθ*cψ + sφ*cθ*sψ, cφ*cθ*sψ - sφ*sθ*cψ]
end

function plotting_fixture(attitude; channels=2, actuator=nothing, calls=Ref(0), output=identity)
    rigid_body = RigidBody(1.0, Matrix{Float64}(I, 3, 3); attitude=attitude)
    controller = FunctionController(
        1,
        (args...) -> error("plotting must not evaluate controller drift"),
        (x_ctrl, x_rb, δ, ϖ, xhat, t) -> begin
            calls[] += 1
            output([10i + t + xhat[1] for i in 1:channels])
        end,
    )
    nominal = QuasiSteadyAerodynamics(
        (args...) -> zeros(3),
        (args...) -> zeros(3),
    )
    # Colored measurement output depends on the saved stochastic state.
    measurement = OrnsteinUhlenbeckMeasurement(
        1.0, reshape([0.1], 1, 1), (x, t) -> [t],
    )
    model = AircraftModel(
        rigid_body=rigid_body,
        aerodynamics=Aerodynamics(nominal),
        actuator=actuator,
        measurement=measurement,
        estimator=IdentityEstimator(),
        controller=controller,
    )
    angles = [0.2, -0.3, 0.4]
    native = attitude isa EulerAngles ? angles : plotting_quaternion(angles)
    x_rb = vcat([10.0, 20.0, -30.0], native, [4.0, 5.0, 6.0], [0.1, 0.2, 0.3])
    x_act = isnothing(actuator) ? Float64[] : fill(0.25, state_dimension(actuator))
    x0 = initial_state(model; x_rb=x_rb, x_act=x_act, x_ctrl=[-999.0], x_meas=[0.75])
    x1 = copy(x0)
    state_components(model, x1).x_rb[1] = 15.0
    state_components(model, x1).x_meas[1] = 1.25
    # The last two states have equal timestamps but different saved noise.
    return model, (t=[0.0, 0.5, 0.5], u=[x0, x0, x1]), native
end

@testset "Saved-state plotting" begin
    @test Base.get_extension(ACES, :ACESPlotsExt) !== nothing

    for attitude in (EulerAngles(), UnitQuaternion())
        calls = Ref(0)
        model, solution, _ = plotting_fixture(attitude; calls=calls)
        before = deepcopy(solution)
        body = plot_rigid_body(model, solution; size=(900, 600))
        @test body isa Plots.Plot
        @test length(body.subplots) == 4
        @test body.attr[:size] == (900, 600)
        @test body.attr[:show] == false
        @test calls[] == 0
        @test body[1].series_list[1][:x] == solution.t
        @test body[1].series_list[1][:y] == [10.0, 10.0, 15.0]
        @test body[1].series_list[3][:y] == fill(-30.0, 3)
        @test length(body[2].series_list) == 3
        for (i, angle) in enumerate([0.2, -0.3, 0.4])
            @test body[2].series_list[i][:y] ≈ fill(angle, 3)
            @test body[2].series_list[i][:label] == ("Roll φ", "Pitch θ", "Yaw ψ")[i]
        end
        @test body[2][:yaxis][:guide] == "rad"
        @test body[3].series_list[1][:y] == fill(4.0, 3)
        @test body[3][:yaxis][:guide] == "m/s"
        @test body[4].series_list[3][:y] == fill(0.3, 3)
        @test body[4][:yaxis][:guide] == "rad/s"
        @test solution == before

        commands = plot_controls(model, solution)
        @test commands isa Plots.Plot
        @test length(commands.subplots) == 2
        @test commands[1].series_list[1][:y] == [10.75, 11.75, 12.25]
        @test commands[2].series_list[1][:y] == [20.75, 21.75, 22.25]
        @test commands[1][:yaxis][:guide] == "u[1]"
        @test calls[] == 3
        @test solution == before

        both = plot_controls(
            model, solution; actuator_outputs=true,
            labels=["Pitch", "Thrust"], units=["rad", "N"],
            actuator_labels=("Surface", "Motor"), actuator_units="1",
        )
        @test calls[] == 6
        @test length(both.subplots) == 4
        @test both[1][:yaxis][:guide] == "Pitch (rad)"
        @test both[2][:yaxis][:guide] == "Thrust (N)"
        @test both[3][:yaxis][:guide] == "Surface (1)"
        @test both[1].series_list[1][:y] == both[3].series_list[1][:y]

        # Standard saving remains available, solely on explicit caller request.
        mktempdir() do directory
            filename = joinpath(directory, "rigid-body.svg")
            savefig(body, filename)
            @test filesize(filename) > 0
        end
    end

    model, solution, _ = plotting_fixture(UnitQuaternion(); channels=3, actuator=FirstOrderActuator(0.5, 3))
    both = plot_controls(model, solution; actuator_outputs=true)
    @test length(both.subplots) == 6
    @test both[3][:yaxis][:guide] == "u[3]"
    @test both[6][:yaxis][:guide] == "δ[3]"
    @test both[4].series_list[1][:y] == fill(0.25, 3)
    @test both[1].series_list[1][:y] != both[4].series_list[1][:y]

    # Empty and single-channel controls are valid, independent of controller state size.
    for channels in (0, 1), output in (identity, SVector{channels})
        small_model, small_solution, _ = plotting_fixture(EulerAngles(); channels=channels, output=output)
        figure = plot_controls(small_model, small_solution)
        @test length(figure.subplots) == max(1, channels)
        @test length(figure.series_list) == channels
        both = plot_controls(small_model, small_solution; actuator_outputs=true)
        @test length(both.subplots) == max(1, 2channels)
        @test length(both.series_list) == 2channels
    end

    # Callbacks may reuse a mutable command buffer; each saved output is copied.
    buffer = zeros(2)
    buffered_model, buffered_solution, _ = plotting_fixture(
        EulerAngles(); output=values -> (buffer .= values; buffer),
    )
    buffered = plot_controls(buffered_model, buffered_solution; actuator_outputs=true)
    @test buffered[1].series_list[1][:y] == [10.75, 11.75, 12.25]
    @test buffered[3].series_list[1][:y] == buffered[1].series_list[1][:y]

    empty_solution = (t=Float64[], u=Vector{Float64}[])
    mismatched_solution = (t=[0.0], u=solution.u)
    @test_throws ArgumentError plot_rigid_body(model, empty_solution)
    @test_throws ArgumentError plot_controls(model, empty_solution)
    @test_throws DimensionMismatch plot_rigid_body(model, mismatched_solution)
    @test_throws DimensionMismatch plot_controls(model, mismatched_solution)
    @test_throws DimensionMismatch plot_controls(model, solution; labels=["one"])
    @test_throws DimensionMismatch plot_controls(model, solution; units=["rad"])
    @test_throws ArgumentError plot_controls(model, solution; labels="one")
    @test_throws ArgumentError plot_controls(model, solution; units=1)
    @test_throws DimensionMismatch plot_controls(model, solution; actuator_outputs=true, actuator_labels=["one"])
    @test_throws DimensionMismatch plot_controls(model, solution; actuator_outputs=true, actuator_units=["rad"])

    # Replaying variable-size command outputs must fail clearly.
    variable_controller = FunctionController(
        0, (args...) -> error("unused"),
        (x_ctrl, x_rb, δ, ϖ, xhat, t) -> zeros(t == 0 ? 1 : 2),
    )
    variable_model = AircraftModel(
        rigid_body=model.rigid_body, aerodynamics=model.aerodynamics,
        measurement=model.measurement, estimator=model.estimator,
        controller=variable_controller,
    )
    variable_solution = (
        t=solution.t,
        u=[initial_state(variable_model; x_rb=copy(state_components(model, x).x_rb)) for x in solution.u],
    )
    @test_throws DimensionMismatch plot_controls(variable_model, variable_solution)

    # A global request to show figures must not open windows from these helpers.
    previous_show = Plots.default(:show)
    try
        Plots.default(show=true)
        @test plot_controls(model, solution; show=true).attr[:show] == false
        @test plot_rigid_body(model, solution; show=true).attr[:show] == false
    finally
        Plots.default(show=previous_show)
    end
end

@testset "Euler attitude plotting" begin
    cases = ([0.0, 0.0, 0.0], [0.2, -0.3, 0.4], [π, 0.0, 0.0],
             [0.0, 0.0, π], [4.2, 0.3, -4.5], [1.0, 2.1, -2.0],
             [0.7, π/2, -0.8], [0.7, -π/2, -0.8],
             [0.7, π/2 - 1e-7, -0.8], [0.7, -π/2 + 1e-7, -0.8])
    for T in (Float32, Float64), attitude in (EulerAngles(), UnitQuaternion())
        model, short, _ = plotting_fixture(attitude)
        angles = [T.(value) for value in cases]
        states = map(angles) do value
            native = attitude isa EulerAngles ? value : plotting_quaternion(value)
            T.(vcat([10, 20, -30], native, [4, 5, 6], [0.1, 0.2, 0.3],
                    short.u[1][(state_dimension(model.rigid_body) + 1):end]))
        end
        history = (t=fill(0.0, length(states)), u=states)
        before = deepcopy(history)
        figure = plot_rigid_body(model, history)
        @test length(figure[2].series_list) == 3
        @test figure[2][:yaxis][:guide] == "rad"
        for (i, value) in enumerate(angles)
            plotted = [series[:y][i] for series in figure[2].series_list]
            @test all(isfinite, plotted)
            @test abs(plotted[1]) <= π + 4eps(T)
            @test abs(plotted[2]) <= π/2 + 4eps(T)
            @test abs(plotted[3]) <= π + 4eps(T)
            @test R_IB(EulerAngles(), plotted) ≈ R_IB(EulerAngles(), value) atol=50eps(T) rtol=50eps(T)
        end
        # Both pitch singularities select zero yaw, without losing the rotation.
        @test figure[2].series_list[3][:y][7:8] == zeros(2)
        @test history == before
    end

    # Equivalent quaternion signs/scales must give the same Euler histories.
    model, short, _ = plotting_fixture(UnitQuaternion())
    q = plotting_quaternion([0.2, -0.3, 0.4])
    states = map((1.0, -1.0, 2.0, -3.0)) do scale
        state = copy(first(short.u))
        state_components(model, state).x_rb[4:7] .= scale .* q
        state
    end
    history = (t=[0.0, 0.5, 0.5, 1.0], u=collect(states))
    before = deepcopy(history)
    figure = plot_rigid_body(model, history)
    for (channel, expected) in enumerate([0.2, -0.3, 0.4])
        @test figure[2].series_list[channel][:x] == history.t
        @test figure[2].series_list[channel][:y] ≈ fill(expected, 4)
    end
    @test history == before
end

@testset "Dynamic saved-history assembly" begin
    assemble = Base.get_extension(ACES, :ACESPlotsExt)._channel_matrix
    # Guard the representation itself as well as the public plots below: the
    # number of samples must not become part of a StaticArray type.
    for n in (3001, 10001), vector in (identity, SVector{3})
        values = [vector([Float32(i), -Float32(i), Float32(i) / 2]) for i in 1:n]
        channels = assemble(values, "test channels")
        @test channels isa Matrix{Float32}
        @test size(channels) == (n, 3)
        for j in 1:3
            @test channels[:, j] == [value[j] for value in values]
        end
    end
    mixed = AbstractVector[SVector(1, 2), [3.5, 4.5]]
    @test assemble(mixed, "mixed") == [1.0 2.0; 3.5 4.5]
    @test assemble(mixed, "mixed") isa Matrix{Float64}
    @test size(assemble([SVector{0,Float64}()], "empty channels")) == (1, 0)
    @test assemble([[2.0]], "single channel") == fill(2.0, 1, 1)
    @test_throws DimensionMismatch assemble([[1.0], [2.0, 3.0]], "changing channels")
end

@testset "Long saved-state plotting" begin
    for n in (3001, 10001), attitude in (EulerAngles(), UnitQuaternion()), static in (false, true)
        @testset "$n samples, $(typeof(attitude)), static=$static" begin
            calls = Ref(0)
            model, short, _ = plotting_fixture(
                attitude; calls=calls, output=static ? SVector{2} : identity,
            )
            n_pose = pose_dimension(model.rigid_body)
            # Adjacent equal timestamps have distinct states and commands.
            times = [(i - 1) ÷ 2 / 100 for i in 1:n]
            angles = [[0.2 + 0.1i / n, -0.3 + 0.2i / n, 0.4 - 0.1i / n] for i in 1:n]
            states = map(1:n) do i
                state = copy(first(short.u))
                parts = state_components(model, state)
                parts.x_rb[1:3] .= [i, -i, -2i]
                parts.x_rb[4:n_pose] .= attitude isa EulerAngles ?
                    angles[i] :
                    plotting_quaternion(angles[i]) .* ((isodd(i) ? -1 : 1) * (1 + i / n))
                parts.x_rb[(n_pose + 1):(n_pose + 6)] .= [j + i / n for j in 1:6]
                parts.x_meas[1] = i / n
                static ? SVector{length(state)}(state) : state
            end
            solution = (t=times, u=states)
            before = deepcopy(solution)
            body = plot_rigid_body(model, solution)
            @test calls[] == 0
            @test length(body.subplots) == 4
            @test body.attr[:show] == false
            indices = ((1, 1:3), (3, (n_pose + 1):(n_pose + 3)), (4, (n_pose + 4):(n_pose + 6)))
            for (panel, coordinates) in indices
                @test length(body[panel].series_list) == length(coordinates)
                for (channel, coordinate) in enumerate(coordinates)
                    series = body[panel].series_list[channel]
                    @test series[:x] == times
                    @test series[:y] == [state[coordinate] for state in states]
                end
            end
            @test length(body[2].series_list) == 3
            @test body[2][:yaxis][:guide] == "rad"
            for channel in 1:3
                @test body[2].series_list[channel][:x] == times
                @test body[2].series_list[channel][:y] ≈ [value[channel] for value in angles]
            end

            # Ideal actuator outputs retain the controller's static/dynamic
            # vector representation and exercise the shared assembly twice.
            controls = plot_controls(model, solution; actuator_outputs=true)
            @test calls[] == n
            @test length(controls.subplots) == 4
            @test controls.attr[:show] == false
            for channel in 1:2, panel in (channel, channel + 2)
                series = controls[panel].series_list[1]
                @test series[:x] == times
                @test series[:y] == [10channel + times[i] + (times[i] + i / n) for i in 1:n]
            end
            @test solution == before
        end
    end
end
