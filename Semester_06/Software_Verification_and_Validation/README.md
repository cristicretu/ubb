# Software Systems Verification and Validation

Semester 6 · Year 3 · Java, Maven, JUnit, Mockito, Serenity BDD, TypeScript, Vitest, Playwright

The software testing course (SSVV). You write black-box and white-box test cases, do integration testing with stubs, drivers and mocks, automate web UI tests, and study exploratory testing. It ends with a take-home exam where you build a small app and test it at every level.

## Contents

| Folder | What it is |
| --- | --- |
| [Labs/Lab_02](Labs/Lab_02) | Black-box testing (BBT). `GiveBonus`: bonus calculation for employees of the department with the biggest sale, with JUnit 5 tests and a `Jenkinsfile`. Test case tables in `.xlsx` (GiveBonus and an EvenNumbers exercise) |
| [Labs/Lab_03](Labs/Lab_03) | White-box testing (WBT). `GiveNextDate` with JUnit 4 tests and a WBT form. `Lab03Files/` has the provided in-class (`Set`) and take-home sources |
| [Labs/Lab_04](Labs/Lab_04) | Integration testing with stubs, drivers and Mockito. `StubsIsPrime` is the reference example (longest prime sequence). `DaysBetween2Dates` is the take-home: days between two dates using a mocked/stubbed `isLeapYear`, applied to a library return system. See [its README](Labs/Lab_04/DaysBetween2Dates/README.md) |
| [Labs/Lab_05](Labs/Lab_05) | Web UI testing with Serenity BDD + JUnit + Selenium. The root project is the in-class demo (keyword search, data-driven from CSV). `take_home/` tests Wikipedia search and opening articles by URL, data-driven from CSV. |
| [Seminars/Seminar_03](Seminars/Seminar_03) | Exploratory testing portfolio: summary of Afzal et al. (2015), "An experiment on the effectiveness and efficiency of exploratory testing", plus a proposed new study (`.md`, `.docx`, `.pdf`) |
| [Take_Home_Exam](Take_Home_Exam) | FIFA World Cup 2026 tracker (Express + TypeScript backend, React + Vite frontend) with BBT/WBT unit tests, integration tests, Playwright end-to-end tests and the written reports. See [its README](Take_Home_Exam/README.md) |

## How to run

Java labs (each folder with a `pom.xml`):

```sh
cd Labs/Lab_04/DaysBetween2Dates
mvn test
```

Serenity web tests (Lab_05):

```sh
cd Labs/Lab_05/take_home
mvn clean verify
```

Reports go to `target/site/serenity/index.html`.

Take-home exam: follow [Take_Home_Exam/README.md](Take_Home_Exam/README.md) (`npm install`, `npm run dev`, `npx vitest run`, `npx playwright test`).

## Notes

- Lab_05 expects ChromeDriver at `src/test/resources/webdriver/mac/chromedriver-mac-arm64/chromedriver` (see `serenity.properties`). The binary is not committed. Download the version matching your Chrome, or change the path.
- Lab_05 tests hit live sites (Wikipedia, cs.ubbcluj.ro), so they can break when those pages change.
- Lab_05 root has both a `pom.xml` and a `build.gradle`.
- The `.xlsx`/`.xls` test case tables are part of the deliverables, not generated.
