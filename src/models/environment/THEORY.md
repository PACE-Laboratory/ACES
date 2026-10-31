# Theory: Environmental Model

Symbols and Julia identifiers follow the [canonical notation](@ref notation).

## Assumptions

- Gravity and atmospheric density depend only on inertial position, with no explicit time dependence.
- Atmospheric density is a scalar field. The default atmosphere models below are horizontally uniform and stationary, so density depends only on altitude.
- Wind is handled by the wind model, not by this environmental model.

## Inputs and Outputs

**Inputs:**
- Inertial position, $\bm{s} \in \mathbb{R}^3$

**Outputs:**
- Gravitational acceleration, $\bm{g} \in \mathbb{R}^3$
- Atmospheric density, $\rho \in \mathbb{R}_{\geq 0}$

## Gravity Model

In general, the acceration due to gravity expressed in the inertial frame is
```math
    \bm{g} = \bm{h}_g(\bm{s})
```

The default gravity model is uniform and points in the $\bm{i}_3$ direction:
```math
    \bm{g} = \begin{bmatrix}
        0 \\
        0 \\
        g
    \end{bmatrix}
```
where $g$ is the gravitational acceleration magnitude, nominally equal to $9.81~\mathrm{m}/\mathrm{s}^2$.

!!! note Remark
    The need to account for the spatial variation of gravity generally indicates that the local NED frame is no longer sufficiently accurate as an inertial approximation. At this level of fidelity, the rigid-body dynamics should be formulated in an ECI frame, or in an ECEF frame with the associated non-inertial terms, including the Coriolis and centrifugal accelerations.

## Density Model

The most general density model is
```math
    \rho = h_\rho(\bm{s})
```
and, for horizontally uniform stationary atmosphere models,
```math
    \rho = h_\rho(h)
```
where $h$ denotes the altitude above mean sea level:
```math
    h = h_0 - s_D
```
where $h_0$ is the mean-sea-level (MSL) altitude of the inerial frame origin and $s_D$ is the third component of $\bm{s}$.

All density models must satisfy $\rho \ge 0$ and return density in $\mathrm{kg}/\mathrm{m}^3$.

### Exponential Density

A simple altitude-only density model is
```math
    \rho(h) = \rho_0 \exp\left(-\frac{h - h_0}{h_\mathrm{scale}}\right)
```
where $\rho_0 = \rho(h_0)$ and $h_\mathrm{scale} > 0$ is the density scale height. This model is useful when only monotonic density decay with altitude is needed.

### NASA Standard Atmosphere

The existing standard-atmosphere formula comes from the [NASA Glenn metric atmosphere](https://www.grc.nasa.gov/www/k-12/airplane/atmosmet.html) curve fits. The model is defined by altitude zones.

For the troposphere, from the surface of the Earth to $11{,}000~\mathrm{m}$,
```math
    T = 15.04 - 0.00649 h
```
```math
    p = 101.29 \left(\frac{T + 273.1}{288.08}\right)^{5.256}
```

For the lower stratosphere, from $11{,}000~\mathrm{m}$ to $25{,}000~\mathrm{m}$,
```math
    T = -56.46
```
```math
    p = 22.65 \exp(1.73 - 0.000157 h)
```

For the upper stratosphere, above $25{,}000~\mathrm{m}$,
```math
    T = -131.21 + 0.00299 h
```
```math
    p = 2.488
        \left(\frac{T + 273.1}{216.6}\right)^{-11.388}
```

In all zones, density is computed from
```math
    \rho = \frac{p}{0.2869(T + 273.1)}
```
where $h$ is in meters, $T$ is in degrees Celsius, $p$ is in kilopascals, and $\rho$ is in $\mathrm{kg}/\mathrm{m}^3$.
