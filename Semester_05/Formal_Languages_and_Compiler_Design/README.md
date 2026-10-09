# Formal Languages and Compiler Design

Semester 5 · Year 3 · C, C++, Flex, ANTLR

You design a small programming language, then build its front end step by step: a BNF spec, a scanner with a symbol table, finite automata for tokens, and an LL(1) parser that builds a parse tree.

## Contents

All lab work lives in one folder (copied from the GitHub Classroom repo): [Labs/flcd-lab-2025-cristicretu](Labs/flcd-lab-2025-cristicretu). It has one folder per lab:

| Folder | What it is |
| --- | --- |
| `lab1` | Language spec in BNF (`program ... end`, `let`, `read`, `say`, `if`, `for`, `fn`, arrays) |
| `lab2` | Example programs written in that language |
| `lab3` | Flex scanner (`scanner.l`) with a symbol table in C |
| `lab4` | Finite automaton processor (`fa_processor.c`) that reads FA definitions for identifiers, numbers and strings |
| `lab5` | Scanner that uses the finite automata to classify tokens, with sample program outputs |
| `lab6` | ANTLR grammar (`syntax.g4`) with a C++ driver that prints the productions used, plus a Flex lexer that writes the PIF |
| `lab7` | LL(1) parser in C: FIRST/FOLLOW, parse table, parse tree (`grammar.txt` holds the grammar) |
| `lab8` | Outputs from the ANTLR parser and the LL(1) parser on the same test program (factorial) |
| `lab9` | Optimized LL(1) parser (hash-table symbol lookup and more, see `OPTIMIZATIONS.md`) |
| `lab10` | Only the lab statement (`.docx`), no code |

## How to run

Each code lab has a `Makefile`:

```sh
cd Labs/flcd-lab-2025-cristicretu/lab7
make
./ll1parser
```

- `lab3`/`lab5` need `flex`.
- `lab6` needs the ANTLR 4 C++ runtime installed.

## Notes

- Some compiled binaries and `.o` files are committed (built on macOS arm64). Rebuild with `make` on your machine.
