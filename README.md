# ubb

Every lab, seminar, project and exam I did for the Computer Science bachelor at Babeș-Bolyai University, Cluj. 2023 to 2026. 6 semesters, 27 courses, 3,600 files. Nothing hidden.

**[cristicretu.github.io/ubb](https://cristicretu.github.io/ubb)**: the same thing, searchable.

## Read this first

The code is here so you can read it. Copy-pasting it into your submission is the worst possible use of it. Teachers know this repo exists, the written exam doesn't care what you pasted, and you learn nothing.

Use it to unblock yourself, then close the tab and write your own. Run it, break it, rewrite it better, send a PR.

## Layout

```
Semester_0N/
└── Course_Name/
    ├── README.md      what every lab does, how to run it
    ├── Labs/Lab_NN    one folder per lab
    ├── Seminars/
    └── Exam/          past subjects, practice, solutions
```

Every course has a README. If something doesn't run, the README says why.

## Year 1

| Sem | Course | Stack |
| --- | --- | --- |
| 1 | [Fundamentals of Programming](Semester_01/Fundamentals_of_Programming) | Python |
| 1 | [Computer System Architecture](Semester_01/Computer_System_Architecture) | x86 asm (NASM) |
| 1 | [Computational Logic](Semester_01/Computational_Logic) | Python |
| 1 | [Linear Algebra](Semester_01/Linear_Algebra) | Python, C++ |
| 1 | [Mathematical Analysis](Semester_01/Mathematical_Analysis) | Python, Jupyter |
| 2 | [Data Structures and Algorithms](Semester_02/Data_Structures_and_Algorithms) | C++ |
| 2 | [Object-Oriented Programming](Semester_02/Object_Oriented_Programming) | C, C++, Qt |
| 2 | [Operating Systems](Semester_02/Operating_Systems) | C, Bash |
| 2 | [Graph Algorithms](Semester_02/Graph_Algorithms) | C++, Python |
| 2 | [Dynamical Systems](Semester_02/Dynamical_Systems) | Maple |

## Year 2

| Sem | Course | Stack |
| --- | --- | --- |
| 3 | [Advanced Programming Methods](Semester_03/Advanced_Programming_Methods) | Java, JavaFX |
| 3 | [Databases](Semester_03/Databases) | T-SQL |
| 3 | [Computer Networks](Semester_03/Computer_Networks) | C, Python, Packet Tracer |
| 3 | [Logic and Functional Programming](Semester_03/Logic_and_Functional_Programming) | Prolog, Lisp |
| 3 | [Probability and Statistics](Semester_03/Probability_and_Statistics) | MATLAB |
| 4 | [Artificial Intelligence](Semester_04/Artificial_Intelligence) | Python, PyTorch |
| 4 | [Database Management Systems](Semester_04/Database_Management_Systems) | C#, T-SQL |
| 4 | [Systems for Design and Implementation (MPP)](Semester_04/Systems_for_Design_and_Implementation) | TypeScript, Next.js, Prisma |
| 4 | [Web Programming](Semester_04/Web_Programming) | PHP, JSP, ASP.NET, Angular |
| 4 | [Software Engineering](#software-engineering) | C#, WinUI 3 |

## Year 3

| Sem | Course | Stack |
| --- | --- | --- |
| 5 | [Formal Languages and Compiler Design](Semester_05/Formal_Languages_and_Compiler_Design) | C, C++, Flex, ANTLR |
| 5 | [Parallel and Distributed Programming](Semester_05/Parallel_and_Distributed_Programming) | C++, C#, Java, MPI |
| 5 | [Mobile Application Programming](Semester_05/Mobile_Application_Programming) | React Native, Expo |
| 5 | [Public Key Cryptography](Semester_05/Public_Key_Cryptography) | Python |
| 5 | [Robotics](Semester_05/Robotics) | Python, ROS, OpenCV |
| 6 | [Software Verification and Validation](Semester_06/Software_Verification_and_Validation) | Java, JUnit, Serenity, Playwright |
| 6 | [Large Language Models](Semester_06/Large_Language_Models) | Python, Jupyter |
| 6 | [Natural Language Processing](Semester_06/Natural_Language_Processing) | Python |

## Software Engineering

One team project, swapped and merged with other teams every few weeks. Four repos, one per phase:

1. [Duo](https://github.com/cristicretu/UBB-SE-2025-Messi/tree/main/Duo): what we started with, a forum for a learning platform
2. [MarketMinds](https://github.com/cristicretu/UBB-SE-2025-MarketMinds): another team's project, handed to us
3. [MarketMessi](https://github.com/cristicretu/UBB-SE-2025-MarketMessi): first merge
4. [Marketplace](https://github.com/cristicretu/UBB-SE-2025-Marketplace): final merge, web + desktop

## Tools

- [ubb-schedule.vercel.app](https://ubb-schedule.vercel.app): your timetable in Google Calendar, one click
- [cn-exam-sand.vercel.app](https://cn-exam-sand.vercel.app): every question from the Computer Networks Moodle final

## Things that will waste your afternoon

- **Qt + CMake** on a Mac: [gist](https://gist.github.com/cristicretu/ceceeff14ff6335959274dfe8b4e7061)
- **JavaFX** never finds its modules: [gist](https://gist.github.com/cristicretu/b9853999dc825f83d65442d6264ecf78)
- **Assembly** needs Windows. On Apple Silicon, run a [Windows 11 ARM VM](https://princessdharmy.medium.com/installing-windows-11-on-macbook-m1-arm64-e1e7e0f52ce0)
- **No valgrind on macOS.** Use `MallocStackLogging=YES leaks -quiet -atExit --list -- ./a.out`
- **No `pthread_barrier_t` on macOS.** [Operating Systems](Semester_02/Operating_Systems) ships a drop-in
- **Packet Tracer** is behind a [NetAcad login](https://www.netacad.com/resources/lab-downloads?courseLang=en-US)

## Contributing

Found a bug? PR. Better solution? PR. Your year has a different subject? PR it into `Exam/`.

[MIT](LICENSE). Take it.
