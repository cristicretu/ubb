# Task 3 — Summary Document: Testing II (Inspection/Review, Exploratory, GUI/Web)

**Group 933** — Cretu Cristian (uploader), Grancea Alexandru, Deaconu Victor, Draguta Vasile
**Application under test:** FIFA World Cup 2026 Tracker (this repo)
**Frontend:** `http://localhost:2069` (Vite + React + Tailwind).
**Backend:** `http://localhost:2070` (Express + TypeScript). The backend exposes `POST /api/reset` to reseed deterministic test data.

This document indexes the three Task-3 techniques and points to the artefacts.

## 1. Inspection / Review

**Approach:** Fagan-style informal asynchronous inspection with a written checklist (C1 functional correctness, C2 data validation, C3 boundaries, C4 state / referential integrity, C5 error handling, C6 security, C7 readability, C8 testability). Four reviewers across two roles (functional/data and security/integrity).

**Scope:** all backend source files under `app/backend/src/` (~520 LOC, 10 files).

**Artefact:** `Task3_InspectionReview.md`.

**Headline numbers**

- 18 defects logged (2 High, 4 Medium, 7 Low, 5 Trivial).
- Defect density ≈ 3.5 / 100 LOC.
- Inspection effort ≈ 3 person-hours; detection rate ≈ 6 defects / person-hour.
- The two High-severity findings overlap with the exploratory-testing pass: F-01 (no year-range check on match dates) and F-02 (editing a Completed match keeps stale results).

## 2. Exploratory testing

**Approach:** session-based exploratory testing (SBTM, Bach), one 60-minute charter, freeform exploration over both UI (Chrome DevTools MCP) and REST API (curl). Followed Afzal et al. (2015) — the paper we covered in the Seminar-3 portfolio — for session-log structure and bug-categorisation discipline.

**Charter:** *Test the input domain and the cross-entity consistency of the three CRUD entities and the Record-Match-Result flow. Look for: validation gaps, boundary leaks, broken referential integrity, report inconsistencies.*

**Artefact:** `Task3_ExploratoryTesting.md`.

**Headline numbers**

- 36 distinct test ideas executed in 60 minutes.
- 28 confirmed-OK ✓ / 7 bugs ✗ / 1 suspicious ?.
- Bug clusters: 2 × state-transition (B-001, B-005), 2 × uniqueness (B-003, B-004), 3 × validation-edge (B-002, B-006, B-007).
- All 7 bugs are reproducible from the running app with no source access required.

The exploratory bugs are cross-linked with the inspection report by ID — for example exploratory **B-005** ↔ inspection **F-02**. No bug was found by exploratory alone *and* missed by inspection (and vice-versa) for the High-severity items, which is the desired triangulation.

## 3. GUI / Web testing

**Tool:** **Playwright** (TypeScript, chromium project) — chosen as the modern equivalent of the Serenity/JUnit Web-UI setup we used in Laboratory 5. Playwright's `webServer` block starts both backend (2070) and frontend (2069) before the run and reuses them on re-runs.

**Project layout:** `e2e/` next to `app/`. Page selectors use `data-testid` attributes added to the React components plus `getByRole` / `getByText` for stable locators. Each test resets the data via `request.post('/api/reset')` before exercising the UI, mirroring the Serenity `@Steps` style with deterministic preconditions.

**Spec files** under `e2e/tests/`:

| Spec | Covers |
|---|---|
| `teams.spec.ts` | CRUD for Team via the UI; validation error path; cascade-block message |
| `players.spec.ts` | Create / edit / filter / delete a player; team-filter dropdown |
| `matches.spec.ts` | Schedule a new match; edit venue; verify the row in the matches table |
| `record_result.spec.ts` | The cross-entity Record-Match-Result flow: pick a scheduled match, set scores, assign scorers from each team, submit |
| `top_scorers.spec.ts` | Verify the report joins all three entities; verify the `limit` input filters the table |

**How to reproduce**

```bash
cd e2e
npm install
npx playwright install chromium
npx playwright test
```

**Result of last run:** `10 passed (2.3 s)` on chromium headless.

> Accessibility note (forwarded to inspection report): the React forms use `<label>` siblings without `htmlFor` / `id`, so screen-reader programmatic association is broken and the e2e spec relies on a custom `fieldByLabel` helper (`e2e/tests/support/fields.ts`) rather than Playwright's idiomatic `page.getByLabel`. Worth fixing for a11y.

## 4. Mapping from PDF requirements

| PDF requirement | Deliverable |
|---|---|
| Inspection / Review | `docs/Task3_InspectionReview.md` |
| Exploratory testing | `docs/Task3_ExploratoryTesting.md` |
| GUI / Web testing | `e2e/` (Playwright project) — config + 5 spec files |
| Summary document | this file |
| Test code | `e2e/tests/*.spec.ts` |

## 5. Cross-technique observations

- Inspection caught most of the *Low* and *Trivial* issues (style, naming, dead-code-ish smells); exploratory testing caught most of the *Medium* and *High* issues (state / consistency). Same finding as Afzal et al. 2015 — different techniques surface different defect classes.
- Playwright's E2E suite passes against the **current** source even though F-01 and F-02 are still open — those bugs require *legal-but-perverse* API sequences that aren't on the happy-path UI flow. This is why having all three (inspection + exploratory + GUI) is necessary; none subsumes the others.
