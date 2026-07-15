# ACES
Aircraft Control and Estimation Simulator

*A modular simulation package for aircraft flight dynamics, control, and estimation in Julia*

## Documentation

The latest development documentation is available at the [ACES documentation site](https://pace-laboratory.github.io/ACES/dev/).

To build the Documenter.jl site locally, instantiate the documentationenvironment and run the build script:

```sh
julia --project=docs -e 'using Pkg; Pkg.instantiate()'
julia --project=docs docs/make.jl
```

The generated site is written to `docs/build/`. Open `docs/build/index.html` in a web browser to preview.
