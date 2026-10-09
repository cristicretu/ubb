# Mathematical Analysis

Semester 1 · Year 1 · Python (NumPy, Matplotlib), Jupyter, LaTeX

Sequences and series, derivatives, Taylor expansions, integrals and functions of several variables. The repo has the optional programming homeworks from the seminars, mostly numerical experiments with plots.

## Contents

| Folder | What it is |
| --- | --- |
| [Seminars/Seminar_04](Seminars/Seminar_04) | Alternating harmonic series: partial sums and how rearranging the terms (p positive, q negative) changes the sum, with plots |
| [Seminars/Seminar_05](Seminars/Seminar_05) | Gradient descent on a convex and a concave function for several learning rates (`main.py`); Blancmange curve, continuous but nowhere differentiable (`extra.py`) |
| [Seminars/Seminar_06](Seminars/Seminar_06) | Notebook: first and second order finite difference approximations of the derivative, error vs step size |
| [Seminars/Seminar_07](Seminars/Seminar_07) | Notebook: trapezoidal rule for the integral of e^(-x^2), compared with sqrt(pi) |
| [Seminars/Seminar_08](Seminars/Seminar_08) | Notebook: unit balls of p-norms for several p, plotted |
| [Seminars/Seminar_09](Seminars/Seminar_09) | Notebook: tangent plane to the sphere, gradient descent with exact step on a quadratic |
| [Seminars/Seminar_12](Seminars/Seminar_12) | Notebook: ridge regression on the scikit-learn breast cancer dataset |
| [Exam](Exam) | Photos of past exam subjects |

Each notebook folder also has its LaTeX export and, for most, the submitted PDF.

## How to run

```sh
cd Seminars/Seminar_04
pip install -r requirements.txt
python3 alternating_sum.py
```

Seminar_05 needs only `numpy` and `matplotlib`. For the notebooks:

```sh
pip install numpy matplotlib scikit-learn jupyter
jupyter notebook
```
