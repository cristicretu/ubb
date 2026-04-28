# Lab 04 - Take-Home: Integration testing, stubs and drivers

Project for the Take-Home part of Lab 04. The goal is to compute the
number of days between two given dates, where the computation depends
on a leap-year checker (`isLeapYear`). The lab requires:

1. Implementation of `isLeapYear` and `daysBetween2Dates` (which uses
   `isLeapYear`).
2. Integration testing of `daysBetween2Dates` in the context of one of
   the suggested applications, using **mocks for `isLeapYear`** and a
   **driver that uses `isLeapYear` as a stub**.
3. Simulation of the chosen application via a driver in the same
   integration-testing setup.

The structure intentionally mirrors the `StubsIsPrime` example
(`LongSeq` / `LongSeq_VerifyIsPrimeMock` + driver/mock test classes).

## Chosen application example

**Library Borrowing - Return System** (option 3 in the brief).

A library computes how long a book was kept past the due date and
turns that into an overdue fine. Correctly counting days across leap
years matters a lot here (incorrect penalties are exactly the
"typical bug" called out in the assignment).

## Source layout

```
src/main/java/DaysBetween/
    MyDate.java                          plain Year/Month/Day with validation
    MyValueException.java                domain exception (mirrors the example)
    DaysBetween.java                     real isLeapYear + isLeapYearStub.
                                         daysBetween2Dates uses the real
                                         predicate by default and works for
                                         arbitrary years; constructing with
                                         `new DaysBetween(true)` switches it
                                         to stub mode for driver tests.
    VerifyIsLeapYear.java                hollow seam, intended to be Mockito-mocked
    DaysBetween_VerifyIsLeapYearMock.java   same daysBetween2Dates logic, but
                                         delegates the leap-year check to an
                                         injected VerifyIsLeapYear
    LibraryReturnSystem.java             application wired against DaysBetween (stub)
    LibraryReturnSystem_Mock.java        application wired against the mock variant
    App.java                             tiny demo entry point

src/test/java/DaysBetween/
    AppTest_IsLeapYear_BBT.java                              EC + BVA tests for the real isLeapYear
    AppTest_DaysBetween2Dates_General.java                   regression tests: production daysBetween2Dates works for arbitrary years
    AppTest_DaysBetween2Dates_IsLeapYearStub.java            driver tests through isLeapYearStub (stub mode)
    AppTest_DaysBetween2Dates_IsLeapYearMock.java            Mockito tests for daysBetween2Dates
    AppTest_LibraryReturnSystem_IsLeapYearStub.java          app integration test (stub-based)
    AppTest_LibraryReturnSystem_IsLeapYearMock.java          app integration test (Mockito-mocked)
    AppTest_MyDate_Validation.java                           regression tests: MyDate rejects impossible dates
```

The test class names start with `AppTest_` to match the example's
convention; the `pom.xml` widens the Surefire `<includes>` filter so
those files are picked up.

## Building and running

The project requires Java 8+. The bundled tests have been verified on
JDK 25; Mockito is told to run in experimental mode for that JDK via
`-Dnet.bytebuddy.experimental=true` in the Surefire configuration.

```
mvn test
```

Expected output (counts):

```
AppTest_IsLeapYear_BBT                              -> 11 tests
AppTest_DaysBetween2Dates_General                   ->  7 tests
AppTest_DaysBetween2Dates_IsLeapYearStub            ->  9 tests
AppTest_DaysBetween2Dates_IsLeapYearMock            ->  6 tests
AppTest_LibraryReturnSystem_IsLeapYearStub          ->  8 tests
AppTest_LibraryReturnSystem_IsLeapYearMock          ->  8 tests
AppTest_MyDate_Validation                           -> 18 tests

Tests run: 67, Failures: 0, Errors: 0, Skipped: 0
```

## How each lab requirement is covered

| Requirement                                                            | Where                                                                                  |
| ---------------------------------------------------------------------- | -------------------------------------------------------------------------------------- |
| 1. `IsLeapYear` implementation                                         | `DaysBetween#isLeapYear`                                                               |
| 1. `DaysBetween2Dates` using leap-year check                           | `DaysBetween#daysBetween2Dates` and `DaysBetween_VerifyIsLeapYearMock#daysBetween2Dates` |
| 2. Driver for `DaysBetween2Dates` using `IsLeapYear` as **stub**       | `AppTest_DaysBetween2Dates_IsLeapYearStub` calls `daysBetween2Dates` which routes through `isLeapYearStub` |
| 2. **Mock** for `IsLeapYear` (Mockito)                                 | `AppTest_DaysBetween2Dates_IsLeapYearMock`, `AppTest_LibraryReturnSystem_IsLeapYearMock` |
| 3. Driver for `DaysBetween2Dates` in the **chosen application** context | `AppTest_LibraryReturnSystem_IsLeapYearStub` (stub) and `AppTest_LibraryReturnSystem_IsLeapYearMock` (mock) |
