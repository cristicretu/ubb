# PDF requirement vs delivered artefacts — final checklist

Cross-referenced against `2025_2026_SSVV_Exam_TakeHome.pdf`.

## Prerequisite

- [x] **Team of 5 (max 6)** — Group 933 has 4 members (Cretu, Grancea, Deaconu, Draguta). Consistent with the team used for Seminar 3 portfolio and Lab 5. *(Note: the PDF target is 5 — we will add a fifth member if required.)*
- [x] Team named in `team_members.txt`, with the uploader (Cretu Cristian) flagged explicitly.

## Task 1 — Application Development (300 XP)

| PDF item | Status | Deliverable |
|---|---|---|
| 3 CRUD entities | ✅ | `Team`, `Player`, `Match` — full create/read/update/delete via REST and UI |
| 1 functionality that uses all 3 entities | ✅ | *Record Match Result* (page `/record`, service `matchResultService.ts`) |
| 1 functionality that generates a report | ✅ | *Top Scorers* (page `/top-scorers`, service `reportService.topScorersReport`, endpoint `GET /api/reports/top-scorers`) |
| MVC architecture | ✅ | Models in `app/backend/src/models/`, controllers in `app/backend/src/controllers/`, views in `app/frontend/src/pages/`. Repository sits between controller and models. |
| Source code | ✅ | `app/backend`, `app/frontend` |
| 5 min video | ⏳ | To be recorded by the team after submission walk-through. Screenshots from a real session captured under `docs/screenshot_*.png` as evidence the app works. |

## Task 2 — Testing I (300 XP)

| PDF item | Status | Deliverable |
|---|---|---|
| Black-Box Testing | ✅ | `docs/Task2_BBT_ComputeGoalBonus.xlsx` (6 sheets, 32 deduplicated TCs) + `app/backend/tests/unit/bonusService.bbt.test.ts` (32 / 32 pass) |
| White-Box Testing | ✅ | `docs/Task2_WBT_ComputeStandingsRow.xlsx` (4 sheets — Problem, Req_CFG_CC_Paths, Req_TC_coverage, Req_Statistics; CC=13 cross-checked) + `app/backend/tests/wbt/standingsService.wbt.test.ts` (14 / 14 pass) |
| Integration testing | ✅ | 4 files under `app/backend/tests/integration/` covering top-down stubs, bottom-up drivers, Vitest mocks, and HTTP supertest (58 / 58 pass) |
| Summary document | ✅ | `docs/SummaryDocument_Task2.md` |
| Documents specific to testing technique | ✅ | the two `.xlsx` files (lab_02 / lab_03 layout) |
| Test code | ✅ | the `tests/` tree above |

## Task 3 — Testing II (300 XP)

| PDF item | Status | Deliverable |
|---|---|---|
| Inspection / Review | ✅ | `docs/Task3_InspectionReview.md` — Fagan-style, 18 defects logged with severity, checklist-driven |
| Exploratory testing | ✅ | `docs/Task3_ExploratoryTesting.md` — 60-min charter, 36 ideas, 7 bugs |
| GUI / Web testing | ✅ | `e2e/` — Playwright config + 5 spec files + support helpers (10 / 10 pass on chromium) |
| Summary document | ✅ | `docs/SummaryDocument_Task3.md` |
| Documents specific to testing technique | ✅ | the three Task3_*.md files plus Playwright spec sources |
| Test code | ✅ | `e2e/tests/*.spec.ts` |

## Turn-in package

- [x] `team_members.txt` with the 4 contributors, uploader flagged.
- [x] All Task 1 / Task 2 / Task 3 artefacts under the take-home directory.
- [ ] Zip the directory as `ExamTakeHome_933_CretuGranceaDeaconuDraguta.zip` (excluding `node_modules/` and `2025_2026_SSVV_Exam_TakeHome.pdf` from the archive if the upload portal already has the spec).

## Final test totals

| Suite | Files | Tests | Outcome | Wallclock |
|---|---|---|---|---|
| BBT (Vitest) | 1 | 32 | ✅ | ~2 ms |
| WBT (Vitest) | 1 | 14 | ✅ | ~2 ms |
| Integration (Vitest + supertest) | 4 | 58 | ✅ | ~70 ms |
| **Backend total** | **6** | **104** | ✅ | ~255 ms |
| GUI/Web (Playwright) | 5 | 10 | ✅ | ~2.2 s |
