# Fundamentals of Programming

Semester 1 · Year 1 · Python

Python from scratch: sorting, complexity, backtracking and dynamic programming, then layered applications (domain, repository, service, UI) with unit tests, file and database persistence, undo and simple GUIs.

## Contents

| Folder | What it is |
| --- | --- |
| [Labs/Lab_02](Labs/Lab_02) | Exchange sort and heap sort on a random list |
| [Labs/Lab_03](Labs/Lab_03) | Same sorts, timed on best/average/worst cases (uses `termtables`) |
| [Labs/Lab_04](Labs/Lab_04) | Backtracking (collinear point subsets, iterative and recursive) and DP (maximize `A[m]-A[n]+A[p]-A[q]`, naive vs DP) |
| [Labs/Lab_05](Labs/Lab_05) | Complex numbers in list or dict representation: longest subarray of distinct numbers, max subarray sum of real parts |
| [Labs/Lab_06](Labs/Lab_06) | Command-based bank transactions manager with filters and undo, plus tests |
| [Labs/Lab_07](Labs/Lab_07) | Expenses manager with memory, text, pickle, JSON and SQLite repositories, picked from `settings.properties` |
| [Labs/Lab_08](Labs/Lab_08) | Library app (books, clients, rentals) with console and Tkinter UI; `mock_test/` has practice test solutions |
| [Labs/Lab_09](Labs/Lab_09) | Library app extended with text/pickle repositories and undo/redo |
| [Labs/Lab_13](Labs/Lab_13) | Practical exam practice: bus routes, student grades, tennis tournament, taxi simulation |
| [Labs/Lab_14](Labs/Lab_14) | Battleship against the computer, console or Pygame GUI |
| [Seminars/Seminar_02](Seminars/Seminar_02) | Menu app managing a list of complex numbers |
| [Seminars/Seminar_03](Seminars/Seminar_03) | Time complexity exercises |
| [Seminars/Seminar_04](Seminars/Seminar_04) | Divide and conquer, backtracking |
| [Seminars/Seminar_05](Seminars/Seminar_05) | Greedy and dynamic programming (Fibonacci memo, max subarray, knapsack) |
| [Seminars/Seminar_06](Seminars/Seminar_06) | Menu app managing rectangles |
| [Seminars/Seminar_10](Seminars/Seminar_10) | Car rental domain classes with unit tests |
| [Exam/game-of-life](Exam/game-of-life) | Game of Life on an 8x8 board with patterns and save file |
| [Exam/hammurabi](Exam/hammurabi) | Hammurabi city management game |
| [Exam/hangman](Exam/hangman) | Small hangman script |
| [Exam/practic_sol](Exam/practic_sol) | Practical exam solution: board game defending Earth from alien ships |
| [Exam/reservation](Exam/reservation) | Hotel room reservations backed by text files |
| [Exam/snake](Exam/snake) | Snake on a grid in the console |
| [Exam/students](Exam/students) | Students and grades backed by text files, with tests |
| [Exam](Exam) | `practic.jpeg` and `written.jpeg`: photos of the practical and written exam subjects |

## How to run

Run each app from the folder that holds `start.py` (imports are relative to it):

```sh
cd Labs/Lab_14/src
pip install pygame texttable
python3 start.py          # console
python3 start.py --gui    # Pygame window
```

Single-file labs run directly, e.g. `python3 Labs/Lab_04/p1.py` from inside `Labs/Lab_04`. Lab_08 takes `gui` (not `--gui`) as argument for the Tkinter UI.

## Notes

- Extra packages used: `texttable` (Lab_14, several Exam apps), `termtables` (Lab_03, Lab_06), `pygame` (Lab_14).
- Lab_08 and Lab_09 import `lib/helpers.py`, which is not in the repo. Add `src/lib/helpers.py` with the input helpers they import (`read_date`, `read_string`, `read_valid_integer`, `random_date` and the Tkinter readers) before running them.
- Lab_04 and Lab_07 have the original assignment statement in their own README.
