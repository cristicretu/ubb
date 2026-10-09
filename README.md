# UBB Computer Science

My labs, seminars, projects and exam prep from the Computer Science bachelor at Babeș-Bolyai University, Cluj-Napoca (English line, 2023–2026).

**Browse it as a website: [cristicretu.github.io/ubb](https://cristicretu.github.io/ubb)**

Every course folder has its own README that says what each lab is and how to run it. Start there.

## How the repo is organized

```
Semester_04/
└── Web_Programming/
    ├── README.md        what's in here, how to run it
    ├── Labs/Lab_01 ...  weekly lab assignments
    ├── Seminars/        seminar code (when there is any)
    └── Exam/            practice subjects, solved past exams
```

Semesters 1–2 are Year 1, 3–4 are Year 2 and 5–6 are Year 3.

## Year 1

| Semester | Course | Languages |
| --- | --- | --- |
| 1 | [Fundamentals of Programming](Semester_01/Fundamentals_of_Programming) | Python |
| 1 | [Computer System Architecture](Semester_01/Computer_System_Architecture) | x86 Assembly (NASM) |
| 1 | [Computational Logic](Semester_01/Computational_Logic) | Python |
| 1 | [Linear Algebra](Semester_01/Linear_Algebra) | Python, C++ |
| 1 | [Mathematical Analysis](Semester_01/Mathematical_Analysis) | Python, Jupyter |
| 2 | [Data Structures and Algorithms](Semester_02/Data_Structures_and_Algorithms) | C++ |
| 2 | [Object-Oriented Programming](Semester_02/Object_Oriented_Programming) | C, C++, Qt |
| 2 | [Operating Systems](Semester_02/Operating_Systems) | C, Bash |
| 2 | [Graph Algorithms](Semester_02/Graph_Algorithms) | C++, Python |
| 2 | [Dynamical Systems](Semester_02/Dynamical_Systems) | Maple |

## Year 2

| Semester | Course | Languages |
| --- | --- | --- |
| 3 | [Advanced Programming Methods](Semester_03/Advanced_Programming_Methods) | Java, JavaFX |
| 3 | [Databases](Semester_03/Databases) | T-SQL (SQL Server) |
| 3 | [Computer Networks](Semester_03/Computer_Networks) | C, Python, Packet Tracer |
| 3 | [Logic and Functional Programming](Semester_03/Logic_and_Functional_Programming) | Prolog, Lisp |
| 3 | [Probability and Statistics](Semester_03/Probability_and_Statistics) | MATLAB |
| 4 | [Artificial Intelligence](Semester_04/Artificial_Intelligence) | Python, Jupyter |
| 4 | [Database Management Systems](Semester_04/Database_Management_Systems) | C#, T-SQL |
| 4 | [Systems for Design and Implementation (MPP)](Semester_04/Systems_for_Design_and_Implementation) | TypeScript, Next.js, Prisma |
| 4 | [Web Programming](Semester_04/Web_Programming) | PHP, JSP, ASP.NET, Angular |
| 4 | Software Engineering | C#, WinUI 3 ([separate repos](#software-engineering-projects)) |

## Year 3

| Semester | Course | Languages |
| --- | --- | --- |
| 5 | [Formal Languages and Compiler Design](Semester_05/Formal_Languages_and_Compiler_Design) | C, C++, Flex, ANTLR |
| 5 | [Parallel and Distributed Programming](Semester_05/Parallel_and_Distributed_Programming) | C++, C#, Java, MPI |
| 5 | [Mobile Application Programming](Semester_05/Mobile_Application_Programming) | React Native, Expo |
| 5 | [Public Key Cryptography](Semester_05/Public_Key_Cryptography) | Python |
| 5 | [Robotics](Semester_05/Robotics) | Python, ROS, OpenCV |
| 6 | [Software Verification and Validation](Semester_06/Software_Verification_and_Validation) | Java, JUnit, Serenity, Playwright |
| 6 | [Large Language Models](Semester_06/Large_Language_Models) | Python, Jupyter |
| 6 | [Natural Language Processing](Semester_06/Natural_Language_Processing) | Python |

## Software Engineering projects

The SE course (semester 4) is a team project that gets swapped and merged between teams during the semester:

1. [Duo](https://github.com/cristicretu/UBB-SE-2025-Messi/tree/main/Duo): the first project, a community forum for a learning platform
2. [MarketMinds](https://github.com/cristicretu/UBB-SE-2025-MarketMinds): after swapping projects with another team
3. [MarketMessi](https://github.com/cristicretu/UBB-SE-2025-MarketMessi): first merge
4. [Marketplace](https://github.com/cristicretu/UBB-SE-2025-Marketplace): final merge

## Tools I made for UBB students

- [UBB schedule → Google Calendar](https://ubb-schedule.vercel.app): import your timetable into your calendar
- [Computer Networks exam practice](https://cn-exam-sand.vercel.app): every question from the CN Moodle final

## Setup guides

- **C++ / Qt (OOP):** configure CMake and Qt with [this gist](https://gist.github.com/cristicretu/ceceeff14ff6335959274dfe8b4e7061)
- **JavaFX (MAP):** follow [this gist](https://gist.github.com/cristicretu/b9853999dc825f83d65442d6264ecf78)
- **Assembly on an M1/M2 Mac (ASC):** run a [Windows 11 ARM VM](https://princessdharmy.medium.com/installing-windows-11-on-macbook-m1-arm64-e1e7e0f52ce0)
- **No valgrind on macOS (OS):** `MallocStackLogging=YES leaks -quiet -atExit --list -- ./a.out`
- **Packet Tracer (Networks):** download it from [Cisco NetAcad](https://www.netacad.com/resources/lab-downloads?courseLang=en-US)

## Using this code

Read it, run it, learn from it. If you hand it in as your own, it's on you: teachers at UBB know this repo exists, and copied labs are easy to spot. Code is [MIT licensed](LICENSE).

Found a bug or have a better solution? Open an issue or a PR.
