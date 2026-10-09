# Computer System Architecture

Semester 1 · Year 1 · x86 assembly (NASM, 32-bit, Windows)

Data representation, x86 registers and flags, and 32-bit assembly programming. The labs go from arithmetic expressions and bit manipulation to strings, `printf`/`scanf` calls and file I/O through `msvcrt.dll`. File names like `18.asm` are the problem numbers from the lab sheets.

## Contents

| Folder | What it is |
| --- | --- |
| [Labs/Lab_02](Labs/Lab_02) | Arithmetic expression on bytes and words (`f+(c-2)*(3+a)/(d-4)`) |
| [Labs/Lab_03](Labs/Lab_03) | Mixed-size expression in signed and unsigned versions; `lab3.asm` is notes on conversions and the stack |
| [Labs/Lab_04](Labs/Lab_04) | Bit operations: build a doubleword from bit ranges of given words (problems 18-21) |
| [Labs/Lab_05](Labs/Lab_05) | Byte strings: keep the odd positive elements of two strings, plus a class exercise |
| [Labs/Lab_06](Labs/Lab_06) | String instructions on arrays of doublewords (sums of packed bytes, extracting bytes by a rule) |
| [Labs/Lab_08](Labs/Lab_08) | `printf`/`scanf`/`fopen`/`fprintf`: read numbers, base 16 to base 10, write text and filtered words to a file |
| [Labs/Lab_09](Labs/Lab_09) | Reads a file in chunks, XORs every byte with 5 and appends the result to another file |
| [Labs/Lab_11](Labs/Lab_11) | Reads a file and prints the length of each word |
| [Labs/Lab_13](Labs/Lab_13) | File processing: letter substitution cipher, keep lowercase letters, print text reversed, count words, mock test |
| [Labs/Lab_14](Labs/Lab_14) | Practical exam solution: read `input.txt`, write the text, its length and other results to an output file |
| [Seminars/Seminar_01](Seminars/Seminar_01) | First `mov`/`add` exercises |
| [Exam](Exam) | `18feb2019.asm` (solved past subject: count set bits in high bytes of a doubleword array) and a photo of the January 2024 written exam (in Romanian) |

## How to run

The programs use the NASM `obj` format with `import ... msvcrt.dll`, the setup given in the course (NASM + ALINK on Windows, or SASM). For example:

```sh
nasm -fobj lab2.asm
alink -oPE -subsys console -entry start lab2.obj
lab2.exe
```

Most programs have no output; step through them in a debugger (OllyDbg or the SASM debugger) and watch the registers.

## Notes

- Windows only: the programs call `exit`, `printf`, `fopen` etc. from `msvcrt.dll`.
- File programs expect their input file (e.g. `input.txt`, `afara.txt`, `messi.txt`) in the current folder. Those input files are not in the repo, create them yourself.
- Some comments and strings are in Romanian.
