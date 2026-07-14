# Theory: Environmental Model

## Assumptions

- Atmospheric density is a scalar field. The default atmosphere models below are horizontally uniform and stationary, so density depends only on altitude.
- Wind is handled by the wind model, not by this environmental model.

## Inputs and Outputs

**Inputs:**
- Time, $t$
- Inertial position, $\bm{s} \in \mathbb{R}^3$

**Outputs:**
- Gravitational acceleration, $\bm{g} \in \mathbb{R}^3$
- Atmospheric density, $\rho \in \mathbb{R}_{\ge 0}$

## Altitude

The NED frame uses a positive-down third coordinate. Therefore, altitude above mean sea level is
$$
    h = h_0 - s_D
$$
where $h_0$ is the mean-sea-level (MSL) altitude of the inerial frame origin and $s_D$ is the third component of $\bm{s}$.

## Gravity Model

The most general environmental gravity output is
$$
    \bm{g} = \bm{h}_g(\bm{s},t)
$$
with components expressed in the inertial frame.

For local aircraft simulations, the default gravity model is uniform and points in the positive-down NED direction:
$$
    \bm{g} = \begin{bmatrix}
        0 \\
        0 \\
        g
    \end{bmatrix}
$$
where $g$ is the gravitational acceleration magnitude, nominally equal to $9.81~\mathrm{m}/\mathrm{s}^2$.

## Density Models

The most general density model is
$$
    \rho = h_\rho(\bm{s},t)
$$
and, for horizontally uniform stationary atmosphere models,
$$
    \rho = h_\rho(h)
$$

All density models must satisfy $\rho \ge 0$ and return density in $\mathrm{kg}/\mathrm{m}^3$.

### Exponential Density

A simple altitude-only density model is
$$
    \rho(h) = \rho_0 \exp\left(-\frac{h - h_0}{h_\mathrm{scale}}\right)
$$
where $\rho_0 = \rho(h_0)$ and $h_\mathrm{scale} > 0$ is the density scale height. This model is useful when only monotonic density decay with altitude is needed.

### NASA Metric Standard Atmosphere

The existing standard-atmosphere formula comes from the NASA Glenn metric atmosphere curve fits. The model is defined by altitude zones.

For the troposphere, from the surface of the Earth to $11{,}000~\mathrm{m}$,
$$
    T = 15.04 - 0.00649 h
$$
$$
    p = 101.29 \left(\frac{T + 273.1}{288.08}\right)^{5.256}
$$

For the lower stratosphere, from $11{,}000~\mathrm{m}$ to $25{,}000~\mathrm{m}$,
$$
    T = -56.46
$$
$$
    p = 22.65 \exp(1.73 - 0.000157 h)
$$

For the upper stratosphere, above $25{,}000~\mathrm{m}$,
$$
    T = -131.21 + 0.00299 h
$$
$$
    p = 2.488
        \left(\frac{T + 273.1}{216.6}\right)^{-11.388}
$$

In all zones, density is computed from
$$
    \rho = \frac{p}{0.2869(T + 273.1)}
$$
where $h$ is in meters, $T$ is in degrees Celsius, $p$ is in kilopascals, and $\rho$ is in $\mathrm{kg}/\mathrm{m}^3$.

Reference: https://www.grc.nasa.gov/www/k-12/airplane/atmosmet.html
