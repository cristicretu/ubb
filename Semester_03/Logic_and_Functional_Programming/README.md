# Logic and Functional Programming

Semester 3 · Year 2 · Prolog, Common Lisp

Recursive programming in two languages: Prolog for the first half (lists, flow models, backtracking) and Lisp for the second (lists, non-linear lists, trees, map functions). Each lab is one numbered problem from the course problem sheet.

## Contents

| Folder | What it is |
| --- | --- |
| [Labs/Lab_01](Labs/Lab_01) | Warm-up in Python: recursive linked list operations |
| [Labs/Lab_02](Labs/Lab_02) | Prolog problem 9: insert an element at a position (`9a.pl`), gcd of all numbers in a list (`9b.pl`) |
| [Labs/Lab_03](Labs/Lab_03) | Prolog problem 12: add each number's divisors after it in a list (`12a.pl`, `12b.pl` for heterogeneous lists) |
| [Labs/Lab_04](Labs/Lab_04) | Prolog backtracking: all subsets of collinear points (`2.pl`), with a Python checker (`a.py`) |
| [Labs/Lab_05](Labs/Lab_05) | Lisp problem 1: n-th element, membership in a non-linear list, all sublists, list to set |
| [Labs/Lab_06](Labs/Lab_06) | Lisp tree problem 6: in-order traversal of a tree stored as (node children-count ...) |
| [Labs/Lab_07](Labs/Lab_07) | Lisp map functions, problem 15: reverse a list on every level with `mapcar` |
| [Labs/Practice](Labs/Practice) | Exam practice: numbered Prolog (`.pl`) and Lisp (`.lsp`) problems, `treeN.lsp` tree problems, sorting in Prolog |
| [Labs/test](Labs/test) | Practical test (Prolog list problem) |
| [Practice](Practice) | More loose exam practice in Prolog and Lisp (backtracking, list problems) |
| [Seminars/Seminar_01](Seminars/Seminar_01) | First Prolog predicates: filter even/prime, lucky numbers, sum to n |
| [Seminars/Seminar_03](Seminars/Seminar_03) | Prolog backtracking (balanced parentheses) and mountain-shaped lists |
| [Seminars/Seminar_04](Seminars/Seminar_04) | Prolog backtracking: valley-shaped subsets, subsets with sum divisible by N |
| [Seminars/Seminar_05](Seminars/Seminar_05) | Lisp: merge sorted lists, nodes on level k of a tree |
| [Seminars/Seminar_06](Seminars/Seminar_06) | Lisp map functions (flatten) |
| [Seminars/Seminar_07](Seminars/Seminar_07) | Prolog with cut, combinations |

## How to run

Prolog (SWI-Prolog):

```sh
swipl 9a.pl
?- insert([1,2,3], 2, 9, R).
```

Lisp (SBCL or CLISP):

```sh
sbcl --load 1.lisp     # or: clisp -i 1.lisp
* (n_th_element '(a b c) 2)
```

## Notes

- Prolog files usually start with a `% flow(i, i, o)` comment that tells you which arguments are inputs and outputs.
- Some seminar `.pl` files use `#` for comments, which SWI-Prolog rejects. Change them to `%` before consulting.
- File names in `Practice/` (e.g. `messi.pl`, `varza.lsp`) say nothing about the content; open the file.
