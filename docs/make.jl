using Documenter

const REPOSITORY_ROOT = normpath(joinpath(@__DIR__, ".."))
const DOCUMENTATION_SOURCE = joinpath(@__DIR__, "src")

# Load the package directly from this checkout so `@docs` blocks always
# describe the same source revision as the canonical Markdown files.
push!(LOAD_PATH, REPOSITORY_ROOT)
using ACES

const THEORY_PAGES = [
    ("Simulation Assembly", "src/THEORY.md", "theory/simulation.md"),
    ("Rigid-Body Dynamics", "src/models/rigid_body/THEORY.md", "theory/rigid-body.md"),
    ("Environment", "src/models/environment/THEORY.md", "theory/environment.md"),
    ("Wind", "src/models/wind/THEORY.md", "theory/wind.md"),
    ("Aerodynamics", "src/models/aerodynamics/THEORY.md", "theory/aerodynamics.md"),
    ("Actuators", "src/models/actuators/THEORY.md", "theory/actuators.md"),
    ("Estimation", "src/models/estimation/THEORY.md", "theory/estimation.md"),
    ("Control", "src/models/control/THEORY.md", "theory/control.md"),
]

const CODE_PAGES = [
    ("Simulation Assembly", "src/CODE.md", "code/simulation.md"),
    ("Rigid-Body Dynamics", "src/models/rigid_body/CODE.md", "code/rigid-body.md"),
    ("Environment", "src/models/environment/CODE.md", "code/environment.md"),
    ("Wind", "src/models/wind/CODE.md", "code/wind.md"),
    ("Aerodynamics", "src/models/aerodynamics/CODE.md", "code/aerodynamics.md"),
    ("Actuators", "src/models/actuators/CODE.md", "code/actuators.md"),
    ("Estimation", "src/models/estimation/CODE.md", "code/estimation.md"),
    ("Control", "src/models/control/CODE.md", "code/control.md"),
]

"""Stage a canonical repository Markdown file for processing by Documenter."""
function stage_page(source_path::AbstractString, destination_path::AbstractString)
    source = joinpath(REPOSITORY_ROOT, source_path)
    destination = joinpath(DOCUMENTATION_SOURCE, destination_path)
    mkpath(dirname(destination))

    edit_url = relpath(source, dirname(destination))
    open(destination, "w") do io
        println(io, "```@meta")
        println(io, "EditURL = ", repr(edit_url))
        println(io, "```")
        println(io)
        write(io, read(source, String))
    end
end

stage_page("NOTATION.md", "notation.md")
foreach(THEORY_PAGES) do (_, source, destination)
    stage_page(source, destination)
end
foreach(CODE_PAGES) do (_, source, destination)
    stage_page(source, destination)
end

const THEORY_PAGE_PATHS = last.(THEORY_PAGES)
const CODE_PAGE_PATHS = last.(CODE_PAGES)

makedocs(
    sitename = "ACES",
    authors = "Jeremy W. Hopwood",
    modules = [ACES],
    checkdocs = :exports,
    source = "src",
    build = "build",
    pages = [
        "Home" => "index.md",
        "Notation" => "notation.md",
        "Theory" => [title => destination for (title, _, destination) in THEORY_PAGES],
        "Code Documentation" => [
            title => destination for (title, _, destination) in CODE_PAGES
        ],
    ],
    # Emit links to concrete HTML files so the site also works when opened
    # directly from docs/build/ without a local web server.
    format = Documenter.HTML(prettyurls = false),
)

deploydocs(
    repo = "github.com/PACE-Laboratory/ACES.git",
    devbranch = "main",
)
