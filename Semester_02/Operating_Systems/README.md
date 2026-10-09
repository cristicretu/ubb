# Operating Systems

Semester 2 · Year 1 · Bash, grep/sed/awk, C (POSIX processes and pthreads)

Unix from the user side (regular expressions, grep, sed, awk, shell scripting) and from the programmer side (processes with `fork`/`exec`, pipes and FIFOs, signals, threads, mutexes, condition variables, barriers, semaphores). Files are mostly named after the problem numbers from the lab sheets.

## Contents

| Folder | What it is |
| --- | --- |
| [Labs/Lab_01](Labs/Lab_01) | C warm-up: 2D arrays with `malloc`, printing `argv` |
| [Labs/Lab_02](Labs/Lab_02) | Reading a file with `fread` |
| [Labs/Lab_03](Labs/Lab_03) | grep and sed one-liners on `passwd.fake` |
| [Labs/Lab_04](Labs/Lab_04) | grep, sed and awk exercises (line/word counting, averages, `ps`/`last`/`ls -l` processing) with sample input files |
| [Labs/Lab_05](Labs/Lab_05) | Bash scripts: walking directories with `find`, filtering files, summing PIDs, counting words in files |
| [Labs/Lab_06](Labs/Lab_06) | Bash scripts: directory reports, reading file names until "stop", building an email list from usernames |
| [Labs/Lab_07](Labs/Lab_07) | Bash scripts with argument checks (users, strings, directory/file triplets, line counts); `a/`, `b/`, `c/` are test files |
| [Labs/Lab_08](Labs/Lab_08) | One Bash script: directories vs regular files given as arguments |
| [Labs/Lab_09](Labs/Lab_09) | `fork` and `wait`; parent and children exchanging random numbers through pipes |
| [Labs/Lab_10](Labs/Lab_10) | Pipes between parent and child; guessing game between two programs over FIFOs (`4a.c`, `4b.c`) |
| [Labs/Lab_12](Labs/Lab_12) | Threads: sequential vs threaded sums, mutex and barrier |
| [Labs/Lab_13](Labs/Lab_13) | Threads with mutexes, condition variables and barriers on N threads from the command line |
| [Labs/bash_practice](Labs/bash_practice) | Bash practice problems (files, directories) with test files |
| [Labs/grep_sed_awk](Labs/grep_sed_awk) | More grep/sed/awk practice on `passwd.fake`, `ps.fake`, `last.fake` |
| [Labs/c-test-practice](Labs/c-test-practice) | C file I/O practice: matrix from a text file, to and from a binary file |
| [Labs/processes](Labs/processes) | Large set of practice problems: `fork`, `exec`, pipes, FIFOs, signals, threads, barriers, semaphores, condition variables, plus past exam problems (`exam_epr.c`, `exam_threads_2023.c`) |
| [Labs/test_epr](Labs/test_epr) | Lab test practice: two processes A and B talking through a pipe |
| [Lectures](Lectures) | Code from lectures: shell loops, `fork`, pipes with `dup2`/`exec`, `popen`, mutex vs spin lock, condition variables |
| [Seminars/Seminar_02](Seminars/Seminar_02) | Shell scripts on files and directories |
| [Seminars/Seminar_03](Seminars/Seminar_03) | `fork`, the `exec` family, timing a command, signal handling |
| [Seminars/Seminar_04](Seminars/Seminar_04) | Pipes and FIFOs |
| [Seminars/Seminar_05](Seminars/Seminar_05) | Threads, mutexes, read-write locks; notes on fork vs threads |
| [Seminars/Seminar_06](Seminars/Seminar_06) | Barriers, condition variables, semaphores |
| [Seminars/Seminar_07](Seminars/Seminar_07) | Exam recap notes (regex, processes, threads) |
| [Exam](Exam) | Written exam subject from 13.06.2024 as text (Romanian and English) |

## How to run

Shell scripts take their input file or directory as an argument:

```sh
cd Labs/Lab_04
bash 13.sh
cd ../Lab_05 && bash 1.sh .
```

C programs (Linux or macOS):

```sh
gcc -Wall -o prog Labs/Lab_09/c.c && ./prog
gcc -Wall -pthread -o prog Labs/Lab_13/4.c Labs/Lab_13/pthread_barrier.c && ./prog Labs/Lab_13/messi.txt 4
```

## Notes

- macOS has no `pthread_barrier_t`. The `pthread_barrier.c/.h` files next to the barrier programs add one (active only on `__APPLE__`); compile them together with the program. On Linux they are not needed.
- The `*.fake` files are sample `/etc/passwd`, `ps` and `last` outputs used by the grep/sed/awk exercises, since the real ones differ per machine.
- Some binaries (`Lectures/a`, `Lectures/Lect_6/b`, `Seminars/Seminar_04/b`) are macOS arm64 builds; recompile them.
- Some comments and file names are in Romanian.
