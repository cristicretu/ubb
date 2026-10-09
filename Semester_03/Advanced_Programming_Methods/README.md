# Advanced Programming Methods

Semester 3 · Year 2 · Java, JavaFX

Object-oriented design in Java built around one project: a toy language interpreter. Each lab extends the previous one with new statements, types and features, ending with threads (`fork`), a type checker and a JavaFX GUI.

## Contents

| Folder | What it is |
| --- | --- |
| [Labs/Lab_01](Labs/Lab_01) | Java warm-up: classes, inheritance, static members, exceptions |
| [Labs/Lab_02](Labs/Lab_02) | Interpreter v1: `PrgState` (execution stack, symbol table, output), MVC layers, int/bool values, assign/if/print statements |
| [Labs/Lab_03](Labs/Lab_03) | Adds strings, relational expressions, file statements (`openRFile`, `readFile`, `closeRFile`), a text menu and per-program log files |
| [Labs/Lab_04](Labs/Lab_04) | Adds the heap: reference types, `new`, `rH`/`wH`, `while`, and a garbage collector |
| [Labs/Lab_05](Labs/Lab_05) | Adds `fork` and runs program states concurrently with an `ExecutorService` |
| [Labs/Lab_06](Labs/Lab_06) | Adds a static type checker (`typecheck`) run before execution |
| [Labs/Lab_07](Labs/Lab_07) | Final version with a JavaFX GUI (program list window + main window showing heap, stack, symbol table, output), Maven build |
| [Seminars/Seminar_02](Seminars/Seminar_02) | Layered app (model/repository/service/ui) for vehicles with custom exceptions |
| [Seminars/Seminar_04](Seminars/Seminar_04) | Interpreter skeleton written in the seminar |
| [Seminars/Seminar_05](Seminars/Seminar_05) | Interpreter skeleton with expressions and print |
| [Seminars/Seminar_06](Seminars/Seminar_06) | Interpreter with if, logic and arithmetic expressions |
| [Seminars/Seminar_07](Seminars/Seminar_07) | Interpreter with file statements; `Main.java` is a Java streams/lambdas exercise |
| [Exam](Exam) | Checklist of concurrency topics to study (locks, latch, semaphore, barrier, atomics) |

`logN.txt` files in each lab are the execution logs of the example programs. `test.in` is the file read by the `readFile` examples.

## How to run

Lab_02 to Lab_06 are plain Java sources with no build file. Open the lab folder in IntelliJ and mark `src` as the sources root, or compile by hand:

```sh
cd Labs/Lab_06
javac -d out $(find src -name "*.java")
java -cp out Main
```

Lab_07 uses Maven and JavaFX 21:

```sh
cd Labs/Lab_07
mvn javafx:run
```

## Notes

- Lab_07 declares `public class Main` in a file named `main.java`. This compiles on macOS/Windows, but on Linux rename the file to `Main.java`.
- In Lab_07 the `pom.xml` sets `mainClass` to `main.Main`, while `Main` is in the default package. If `mvn javafx:run` cannot find it, change `mainClass` to `Main`.
- Run from the lab folder so the log files and `test.in` resolve correctly.
