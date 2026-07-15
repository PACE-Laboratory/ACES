using Documenter

const REPOSITORY_ROOT = normpath(joinpath(@__DIR__, ".."))
const DOCUMENTATION_SOURCE = joinpath(@__DIR__, "src")

const THEORY_PAGES = [
    ("Rigid-Body Dynamics", "src/models/rigid_body/THEORY.md", "theory/rigid-body.md"),
    ("Environment", "src/models/enviroment/THEORY.md", "theory/environment.md"),
    ("Wind", "src/models/wind/THEORY.md", "theory/wind.md"),
    ("Aerodynamics", "src/models/aerodynamics/THEORY.md", "theory/aerodynamics.md"),
    ("Actuators", "src/models/actuators/THEORY.md", "theory/actuators.md"),
    ("Estimation", "src/models/estimation/THEORY.md", "theory/estimation.md"),
    ("Control", "src/models/control/THEORY.md", "theory/control.md"),
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

const THEORY_PAGE_PATHS = last.(THEORY_PAGES)

makedocs(
    sitename = "ACES",
    authors = "Jeremy W. Hopwood",
    source = "src",
    build = "build",
    pages = [
        "Home" => "index.md",
        "Notation" => "notation.md",
        "Theory" => [title => destination for (title, _, destination) in THEORY_PAGES],
    ],
    # Emit links to concrete HTML files so the site also works when opened
    # directly from docs/build/ without a local web server.
    format = Documenter.HTML(prettyurls = false),
)
