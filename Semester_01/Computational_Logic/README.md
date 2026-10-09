# Computational Logic

Semester 1 · Year 1 · Python

Propositional and predicate logic, proof methods (semantic tableaux, resolution), Boolean functions and number bases. The only code is an optional homework on arithmetic and conversions between bases.

## Contents

| Folder | What it is |
| --- | --- |
| [HW_Conversions](HW_Conversions) | Console app for addition, subtraction, multiplication and division by one digit in bases 2-10 and 16, plus base conversions (substitution, successive divisions, base 10 as intermediate, rapid conversions between 2, 4, 8, 16). Includes tests and generated HTML docs |
| [Exam](Exam) | Photos of past written exam subjects (semantic tableaux, soundness/completeness, Karnaugh diagrams) |

## How to run

```sh
cd HW_Conversions
python3 main.py
```

The function docs are in `HW_Conversions/docs`; open `index.html` in a browser.

## Notes

- All the logic is in `lib.py`; `ui.py` is the menu. `lib.py` also has `test_all_functions()` with asserts for every operation.
