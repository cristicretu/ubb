# Task 2 — Summary Document: Testing I (BBT, WBT, Integration)

**Group 933** — Cretu Cristian (uploader), Grancea Alexandru, Deaconu Victor, Draguta Vasile
**Application under test:** FIFA World Cup 2026 Tracker (this repo)
**Tech stack of the SUT:** TypeScript on Node 24 — Express 4 backend (port 2070) + Vite/React/Tailwind frontend (port 2069). In-memory repository with three CRUD entities (Team, Player, Match). Test runner: **Vitest** with v8 coverage.

> This document indexes the three testing techniques required by Task 2 and explains which functions were chosen as System Under Test (SUT) for each technique, why, and where to find the artefacts.

## 1. Black-Box Testing (BBT)

**SUT chosen:** `computeGoalBonus(position, goals, marketValueMillions)` in `app/backend/src/services/bonusService.ts`.

**Why this function:** rich input domain (a string enum, an integer in [0, 10], a number in [0, 500]), four equally-spaced positional rates and two market-value multipliers — directly comparable to the *GiveBonus* example we used in Laboratory 2. It maps cleanly to equivalence partitioning, BVA, and a small all-executed combined set.

**Artefacts**

- `Task2_BBT_ComputeGoalBonus.xlsx` — 6 sheets: `Problem`, `EC`, `TC based on EP`, `BVA`, `TC based on BVA`, `EP_BVA_all_executed`. Layout mirrors `lab_02/GiveBonus_BBT_TestCases.xlsx`.
- `app/backend/tests/unit/bonusService.bbt.test.ts` — Vitest mirror of the xlsx, parameterised with `test.each`. **32 / 32 tests pass.**

**Coverage of the input domain**

| Sheet | Test cases | Notes |
|---|---|---|
| TC based on EP | 15 | Covers all 15 ECs: 4 valid positions + 1 invalid; goals = 0, in-range, < 0, > 10, non-integer; the three market-value sub-ranges and the two out-of-range cases. |
| TC based on BVA | 17 | Boundaries on goals ({-1, 0, 1, 10, 11}), marketValue ({-1, 0, 10, 10.001, 100, 100.001, 500, 500.001}), and position membership. |
| EP_BVA_all_executed | 30 (after deduplication; 2 dup flagged) | Combined matrix. |
| Vitest run | 32 / 32 pass | `cd app/backend && npx vitest run tests/unit/bonusService.bbt.test.ts` |

**Sample finding (carried to the inspection report):** the validator accepts `Infinity` for `marketValueMillions`; the BVA suite intentionally documents this in a Remarks cell rather than patching the code so that the artefacts and the source stay aligned for grading. See `Task3_InspectionReview.md` F-06.

## 2. White-Box Testing (WBT)

**SUT chosen:** `computeStandingsRow(teamId, matches)` in `app/backend/src/services/standingsService.ts`.

**Why this function:** a small but realistic loop with guard `continue`s, four branching decisions (`m.status`, `m.stage`, team membership, null score), and a 3-way comparison (`own > opp` / `===` / `<`). Cyclomatic complexity is in the same ballpark as *GiveNextDate* from Laboratory 3, so the artefacts mirror that lab's structure directly.

**Artefacts**

- `Task2_WBT_ComputeStandingsRow.xlsx` — 4 sheets: `Problem`, `Req_CFG_CC_Paths`, `Req_TC_coverage`, `Req_Statistics`. Layout mirrors `lab_03/Lab03_WBT_GiveNextDate.xlsx`.
- `app/backend/tests/wbt/standingsService.wbt.test.ts` — Vitest mirror of the xlsx, one `it` per row of `Req_TC_coverage`. **14 / 14 tests pass.**

**Cyclomatic complexity (cross-checked three ways, all agree at CC = 13)**

| Method | Value |
|---|---|
| Regions (planar CFG) | 12 enclosed + 1 outer = 13 |
| E − N + 2 | 36 − 25 + 2 = 13 |
| Predicates + 1 | 12 atomic predicates + 1 = 13 |

Exact edge/node counts and predicate enumeration are in the `Req_CFG_CC_Paths` sheet.

**Path / decision / loop coverage**

The `Req_TC_coverage` sheet uses the same matrix as lab_03. Statement, decision and loop coverage all reach 100 %:

- 0 iterations: TC-4 (empty matches array)
- 1 iteration counted: TC-10..TC-12 (single completed Group match, exercising each outcome arm)
- 1 iteration skipped: TC-5..TC-9 (each `continue` guard exercised — wrong status, wrong stage, wrong team, null homeScore, null awayScore)
- > 1 iterations: TC-13, TC-14 (mixed matches; full standings)
- input-validation branches: TC-1..TC-3 (teamId not positive int, matches not array, default OK)

Branch coverage on `own > opp`, `own === opp`, `own < opp` is achieved with three dedicated test cases.

## 3. Integration testing

**Strategy:** all three Lab 4 patterns (top-down with stubs, bottom-up with drivers, mocks) plus an end-to-end HTTP layer with **supertest**.

**Files** under `app/backend/tests/integration/`:

| Strategy | File | What is real / stubbed / mocked |
|---|---|---|
| Top-down with stubs | `topdown_recordResult_withRepoStub.test.ts` | Real `recordMatchResult`; the repository is replaced by a hand-written object stub returning canned `Team`/`Player`/`Match` data. Exercises all error branches without depending on the repo. |
| Bottom-up with drivers | `bottomup_repository_withDrivers.test.ts` | Drivers (test wrappers) exercise the real `InMemoryRepository` through CRUD sequences and assert cascade-block invariants (delete-team-with-players, delete-team-with-matches, delete-player-with-goals). |
| Mocks (Vitest) | `mock_reportService_withVitestMocks.test.ts` | `vi.fn()` mocks of `listMatches` / `listPlayers` / `listTeams` feed `topScorersReport` and `groupStandings`; mocks are verified to be called as expected. |
| HTTP integration | `api_http_supertest.test.ts` | Real `createApp(new InMemoryRepository())` driven through `supertest` over every REST route — happy paths and key 4xx error paths. |

**Total: 58 `it` blocks across 4 files, all passing in ≈560 ms** (11 top-down stub + 19 bottom-up driver + 10 mock + 18 HTTP). Run with:

```bash
cd app/backend
npx vitest run tests/integration
```

## 4. How to reproduce all of Task 2

```bash
cd app/backend
npm install
npx vitest run           # runs unit + wbt + integration suites together
npx vitest run --coverage  # adds v8 coverage; report under coverage/
```

**Last full run:** `Test Files 6 passed (6) — Tests 104 passed (104)` — 32 (BBT) + 14 (WBT) + 58 (integration: 11 + 19 + 10 + 18).

The Excel artefacts (`Task2_BBT_*.xlsx`, `Task2_WBT_*.xlsx`) are static and live under `docs/`.

## 5. Mapping from PDF requirements

| PDF requirement | Deliverable |
|---|---|
| BBT | `docs/Task2_BBT_ComputeGoalBonus.xlsx` + `tests/unit/bonusService.bbt.test.ts` |
| WBT | `docs/Task2_WBT_ComputeStandingsRow.xlsx` + `tests/wbt/standingsService.wbt.test.ts` |
| Integration testing | `tests/integration/*.test.ts` (top-down stub, bottom-up driver, mocks, HTTP) |
| Summary document | this file |
| Test code | the `tests/` tree above |
