# Probability and Statistics

Semester 3 · Year 2 · MATLAB

Probability distributions, simulation of random variables, and statistical inference (confidence intervals and hypothesis tests). All lab work is MATLAB scripts.

## Contents

| Folder | What it is |
| --- | --- |
| [Labs/Lab_01](Labs/Lab_01) | MATLAB basics: matrix operations and plotting |
| [Labs/Lab_02](Labs/Lab_02) | Binomial distribution: pdf/cdf with `binopdf`, `binocdf` and plots |
| [Labs/Lab_03](Labs/Lab_03) | Continuous models (Normal, Student, Chi2, Fisher) and approximating the binomial with normal and Poisson |
| [Labs/Lab_04](Labs/Lab_04) | Simulations with `rand`: binomial, geometric and Pascal (negative binomial) variables; `PS_Lab_HW.pdf` is a table of distributions with mean and variance |
| [Labs/Lab_05](Labs/Lab_05) | Descriptive statistics and confidence intervals for mean, variance, difference of means and ratio of variances |
| [Labs/Practice](Labs/Practice) | Practical exam practice: two-sample t-tests and F-tests (`ttest`, `ttest2`, `vartest2`) and confidence intervals on given data |
| [Labs/test](Labs/test) | Practical exam: comparing two samples (steel vs glass) at 5% significance |

## How to run

Open the folder in MATLAB and run a script:

```matlab
>> run('Lab_05/1.m')
```

Most scripts call `input(...)` and wait for values in the Command Window.

## Notes

- The scripts use the Statistics and Machine Learning Toolbox (`binopdf`, `norminv`, `tinv`, `ttest2`, ...).
- GNU Octave works for most of them after `pkg load statistics`.
- Scripts start with `clear all`, so they wipe your workspace.
