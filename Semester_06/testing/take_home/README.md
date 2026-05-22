# SSVV — Exam Take-Home — Group 933

**Members (4):** Cretu Cristian (uploader), Grancea Alexandru, Deaconu Victor, Draguta Vasile

**Application:** *FIFA World Cup 2026 Tracker* — a small tournament-management web app built with Vite + React + Tailwind (frontend, port `2069`) and Express + TypeScript (backend, port `2070`) backed by an in-memory repository. MVC layout: models (`Team`, `Player`, `Match`) → repository → controllers → REST API → React views.

## Layout

```
take_home/
├── README.md                 (this file — deliverables index)
├── team_members.txt          (PDF-required: list of contributors)
├── 2025_2026_SSVV_Exam_TakeHome.pdf
├── app/
│   ├── backend/              Task 1 — source code (Express + TS)
│   │   ├── src/
│   │   │   ├── models/       Team, Player, Match + their validators
│   │   │   ├── repository/   InMemoryRepository + seed
│   │   │   ├── services/     bonusService (BBT SUT), standingsService (WBT SUT),
│   │   │   │                 reportService (Top-Scorers report), matchResultService
│   │   │   ├── controllers/  team / player / match / report
│   │   │   ├── app.ts, server.ts
│   │   └── tests/            Task 2 test code
│   │       ├── unit/         bonusService.bbt.test.ts          (BBT)
│   │       ├── wbt/          standingsService.wbt.test.ts      (WBT)
│   │       └── integration/  topdown / bottomup / mocks / HTTP
│   └── frontend/             Task 1 — Vite + React + Tailwind UI
│       └── src/pages/        Teams, Players, Matches, RecordResult, TopScorers
├── e2e/                      Task 3 — Playwright GUI/Web tests
│   ├── playwright.config.ts
│   └── tests/                teams · players · matches · record_result · top_scorers
└── docs/                     Task 2 / Task 3 documentation
    ├── SummaryDocument_Task2.md
    ├── SummaryDocument_Task3.md
    ├── Task2_BBT_ComputeGoalBonus.xlsx
    ├── Task2_WBT_ComputeStandingsRow.xlsx
    ├── Task3_ExploratoryTesting.md
    ├── Task3_InspectionReview.md
    └── screenshot_*.png      (UI evidence)
```

## Task 1 — Application (300 XP)

- 3 CRUD entities: **Team**, **Player**, **Match** (each with full Create/Read/Update/Delete via REST + UI).
- 1 functionality using **all 3 entities**: *Record Match Result* (page `/record`) — picks a scheduled Match, lists Players from each Team, assigns scorers with minute, validates that the scorer belongs to the home or away Team, then writes the result back to the Match.
- 1 functionality that generates a report: *Top Scorers* (page `/top-scorers`) — aggregates goals across all Completed Matches, joins Player and Team data, ranks by goal count with `?limit=` and `?minGoals=` query parameters. Endpoint: `GET /api/reports/top-scorers`.
- MVC: models in `src/models/`, controllers in `src/controllers/`, views in `frontend/src/pages/`. Repository sits between controller and model.

## Run

```bash
# backend (port 2070)
cd app/backend && npm install && PORT=2070 npm run dev
# frontend (port 2069)
cd app/frontend && npm install && npm run dev
# open http://localhost:2069
```

## Test (Task 2 + Task 3)

```bash
# Vitest — Unit (BBT) + WBT + Integration (104 tests)
cd app/backend && npx vitest run
# Playwright — GUI / Web (10 tests)
cd e2e && npm install && npx playwright install chromium && npx playwright test
```

## Most recent run results

| Suite | Files | Tests | Outcome |
|---|---|---|---|
| BBT (Vitest) | 1 | 32 | ✓ pass |
| WBT (Vitest) | 1 | 14 | ✓ pass |
| Integration (Vitest + supertest) | 4 | 58 | ✓ pass |
| **Backend total** | **6** | **104** | ✓ pass (≈270 ms) |
| GUI/Web (Playwright, chromium) | 5 | 10 | ✓ pass (≈2.3 s) |

See `docs/SummaryDocument_Task2.md` and `docs/SummaryDocument_Task3.md` for what each test covers and the per-technique artefacts.
