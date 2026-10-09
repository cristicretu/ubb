# Formal Languages and Compiler Design

Semester 5 · Year 3 · C, C++, Flex, ANTLR

You design a small programming language, then build its front end step by step: a BNF spec, a scanner with a symbol table, finite automata for tokens, and an LL(1) parser that builds a parse tree.

## Contents

| Folder | What it is |
| --- | --- |
| [Labs/Lab_01](Labs/Lab_01) | Language spec in BNF (`program ... end`, `let`, `read`, `say`, `if`, `for`, `fn`, arrays) |
| [Labs/Lab_02](Labs/Lab_02) | Example programs written in that language |
| [Labs/Lab_03](Labs/Lab_03) | Flex scanner (`scanner.l`) with a symbol table in C |
| [Labs/Lab_04](Labs/Lab_04) | Finite automaton processor (`fa_processor.c`) that reads FA definitions for identifiers, numbers and strings |
| [Labs/Lab_05](Labs/Lab_05) | Scanner that uses the finite automata to classify tokens, with sample program outputs |
| [Labs/Lab_06](Labs/Lab_06) | ANTLR grammar (`syntax.g4`) with a C++ driver that prints the productions used, plus a Flex lexer that writes the PIF |
| [Labs/Lab_07](Labs/Lab_07) | LL(1) parser in C: FIRST/FOLLOW, parse table, parse tree (`grammar.txt` holds the grammar) |
| [Labs/Lab_08](Labs/Lab_08) | Outputs from the ANTLR parser and the LL(1) parser on the same test program (factorial) |
| [Labs/Lab_09](Labs/Lab_09) | Optimized LL(1) parser (hash-table symbol lookup and more, see `OPTIMIZATIONS.md`) |
| [Labs/Lab_10](Labs/Lab_10) | Only the lab statement (`.docx`), no code |

## How to run

Each code lab has a `Makefile`:

```sh
cd Labs/Lab_07
make
./ll1parser
```

- Lab_03 and Lab_05 need `flex`.
- Lab_06 needs the ANTLR 4 C++ runtime installed.

## Notes

- Lab_07 and Lab_09 read the PIF from `../Lab_06/output/pif.txt`, so run Lab_06 first.
