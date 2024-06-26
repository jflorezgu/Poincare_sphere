# Wave plate in the Poincaré sphere
Poincaré sphere representation of a Jones vector under the action of either a half- or a quarter-wave plate

## Requirements
This code requires ipympl to dynamically interact with the Poincaré sphere. You can install ipympl via pip
```
pip install ipympl
```
or conda
```
conda install -c conda-forge ipympl
```

## Convention
There are multiple conventions for the Jones vectors and matrices. However, we follow the Wikipedia [Jones calculus](https://en.wikipedia.org/wiki/Jones_calculus) convention because it is easily available to everyone.

The orthogonal polarisation vectors are:

$$ H = \begin{bmatrix}1 \cr 0\end{bmatrix} $$

$$ V = \begin{bmatrix}0 \cr 1\end{bmatrix} $$

$$ D = \frac{1}{\sqrt{2}}\begin{bmatrix}1 \cr 1\end{bmatrix} $$

$$ A = \frac{1}{\sqrt{2}}\begin{bmatrix}1 \cr -1\end{bmatrix} $$

$$ R = \frac{1}{\sqrt{2}}\begin{bmatrix}1 \cr -i\end{bmatrix} $$

$$ L = \frac{1}{\sqrt{2}}\begin{bmatrix}1 \cr i\end{bmatrix} $$

For the half- and quarter-wave plates (HWP and QWP, respectively) we use the Jones matrix for an arbitrary linear phase retarder (LPR)

$$ LPR(\eta,\vartheta,\varphi) = \begin{bmatrix} \cos(\eta/2)-i\sin(\eta/2)\cos(2\vartheta) & -\sin(\eta/2)\sin(\varphi)\sin(2\vartheta)-i\sin(\eta/2)\cos(\varphi)\sin(2\vartheta) \cr \sin(\eta/2)\sin(\varphi)\sin(2\vartheta)-i\sin(\eta/2)\cos(\varphi)\sin(2\vartheta) & \cos(\eta/2)+i\sin(\eta/2)\cos(2\vartheta)\end{bmatrix} $$

with $\eta = \pi$ ($\eta = \pi/2$) for a HWP (QWP). The angle $\varphi$ identically vanishes for both a HWP and a QWP, but we keep it to introduce the action of a liquid crystal in a future release.

## Effect of a wave plate on an arbitrary Jones vector

We assume an initial Jones vector of the form

$$ \begin{bmatrix}\cos(\theta/2) \cr e^{i\phi}\sin(\theta/2)\end{bmatrix} $$

The action of $LPR(\eta,\vartheta,\varphi)$ on this initial Jones vector leads to

$$ LPR(\eta,\vartheta,\varphi)\begin{bmatrix}\cos(\theta/2) \cr e^{i\phi}\sin(\theta/2)\end{bmatrix}=\begin{bmatrix}\alpha \cr \beta\end{bmatrix} $$

with

$$ \alpha = \[\cos(\eta/2)-i\sin(\eta/2)\cos(2\vartheta)\]\cos(\theta/2)+\[-\sin(\eta/2)\sin(\varphi)\sin(2\vartheta)-i\sin(\eta/2)\cos(\varphi)\sin(2\vartheta)\]e^{i\phi}\sin(\theta/2) $$

$$ \beta = \[\sin(\eta/2)\sin(\varphi)\sin(2\vartheta)-i\sin(\eta/2)\cos(\varphi)\sin(2\vartheta)\]\cos(\theta/2)+\[\cos(\eta/2)+i\sin(\eta/2)\cos(2\vartheta)\]e^{i\phi}\sin(\theta/2) $$

To represent Jones vectors in the Poincaré sphere we calculate the Stokes parameters as

$$ S_1 = |\alpha|^2-|\beta|^2 $$

$$ S_2 = 2\mathrm{Re}(\alpha\beta^*) $$

$$ S_3 = 2\mathrm{Im}(\alpha\beta^*) $$

where $^*$ is the complex conjugate.

## Screenshots

A HWP at 22.5° transforms H into D:

<div align="center">

![HWPat22 5_H](https://github.com/jflorezgu/Poincare_sphere/assets/29898626/d035b9de-8da3-44e0-9c5c-33a4391b2587)

</div>

A QWP at 0° transforms R into D:

<div align="center">

![QWPat0_R](https://github.com/jflorezgu/Poincare_sphere/assets/29898626/c957e61f-97a6-4ad7-9acb-05fd47acedf0)

</div>

## Suggested exercises

1. Verify that a HWP at an angle $\vartheta$ rotates an arbitrary vector in the Poincaré sphere by 180° around an axis on the equator at $2\vartheta$ w.r.t. the "H" axis.

2. Verify that a QWP at an angle $\vartheta$ rotates an arbitrary vector in the Poincaré sphere by 90° around an axis on the equator at $2\vartheta$ w.r.t. the "H" axis following the left-hand rule (palm pointing towards the initial vector, thumb parallel to the QWP rotation axis).

3. Verify that the wave plate angles to produce the orthogonal polarisation vectors – up to a global phase – in a PBS $\to$ QWP $\to$ HWP configuration are:

<div align="center">

| Vector | QWP | HWP   |
|:-----:|:---:|:-----:|
| H     | 0°  | 0°    |
| V     | 0°  | 45°   |
| D     | 0°  | 22.5° |
| A     | 0°  | -22.5°|
| R     | -45° | 0°    |
| L     | 45°| 0°    |

</div>

4. Verify that the wave plate angles to project the orthogonal polarisation vectors onto the transmission port of a PBS in a QWP $\to$ HWP $\to$ PBS configuration are:

<div align="center">

| Vector | QWP | HWP   |
|:-----:|:---:|:-----:|
| H     | 0°  | 0°    |
| V     | 0°  | 45°   |
| D     | 45°  | 22.5° |
| A     | 45°  | -22.5°|
| R     | 0° | 22.5°    |
| L     | 0°| -22.5°    |

</div>
