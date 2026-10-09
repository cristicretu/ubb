# Task 3 — Inspection / Review

**Group 933** — Cretu Cristian (uploader), Grancea Alexandru, Deaconu Victor, Draguta Vasile
**Date:** 2026-05-22
**Type:** Fagan-style code inspection (informal, asynchronous), driven by a written checklist.
**Code base reviewed:** `app/backend/src/` and `app/frontend/src/` of this repository.
**Files inspected (10):**

```
app/backend/src/models/Team.ts
app/backend/src/models/Player.ts
app/backend/src/models/Match.ts
app/backend/src/repository/InMemoryRepository.ts
app/backend/src/services/bonusService.ts
app/backend/src/services/standingsService.ts
app/backend/src/services/reportService.ts
app/backend/src/services/matchResultService.ts
app/backend/src/controllers/{team,player,match,report}Controller.ts
app/backend/src/app.ts
```

**Roles:**

| Role | Member |
|---|---|
| Moderator | Cretu Cristian |
| Author | Cretu Cristian / Grancea |
| Reviewer #1 (functional/data) | Deaconu Victor |
| Reviewer #2 (security/integrity) | Draguta Vasile |
| Reviewer #3 (style/maintainability) | Grancea Alexandru |
| Scribe | Cretu Cristian |

**Effort:** ~3 person-hours total (45 min planning + 90 min reading + 30 min log compilation).

---

## Checklist used

Adapted from Fagan 1976 + a short SSVV-specific addendum:

- **C1 Functional correctness.** Does each function implement its specification? Are pre/post-conditions enforced?
- **C2 Data validation.** Is every external input validated at the entry boundary? Are nulls / NaN / Infinity / negative numbers / wrong types handled?
- **C3 Boundary handling.** Are inclusive vs. exclusive bounds explicit and consistent with the documentation?
- **C4 State transitions / referential integrity.** Can the system be moved into an inconsistent state via a legal sequence of API calls?
- **C5 Error handling.** Are errors distinguishable (codes / messages)? Are HTTP status codes correct?
- **C6 Security.** SQL/NoSQL/HTML injection guards? Authorisation? Resource limits?
- **C7 Readability.** Naming, comments, dead code, repeated logic.
- **C8 Testability.** Are units pure and injectable? Are time/randomness sources isolated?

Each finding below is tagged with the checklist item, severity (`Hi` / `Med` / `Lo` / `Tri`), and a recommended action. Severity is calibrated to the project context — a take-home exam app, not a production deployment.

---

## Findings

| ID | File:Line | Tag | Severity | Description | Recommended action |
|---|---|---|---|---|---|
| F-01 | `models/Match.ts:43` | C4 | **High** | `validateMatchInput` does not check that the match date falls inside the 2026 window. `1999-01-01` is accepted. Cross-referenced with exploratory finding **B-001**. | Add `if (m.date < "2026-01-01" \|\| m.date > "2026-12-31") errors.push(...)`. |
| F-02 | `repository/InMemoryRepository.ts:73`, `updateMatch` | C4 | **High** | A `PUT /api/matches/:id` on a Completed match silently changes `homeTeamId`/`stage` while keeping the previous `homeScore`, `awayScore` and `goals`. Goals are then attached to the new teams. Cross-references **B-005**. | When `existing.status === "Completed"`, either reject 409, or clear `homeScore`, `awayScore`, `goals` and reset to `Scheduled`. |
| F-03 | `repository/InMemoryRepository.ts:54-58`, `createPlayer` | C4 | Med | No uniqueness check on `(teamId, jerseyNumber)`. Two players can share #10 on Argentina. Cross-references **B-004**. | Add `if (this.listPlayersByTeam(input.teamId).some(p => p.jerseyNumber === input.jerseyNumber)) throw ...`. |
| F-04 | `repository/InMemoryRepository.ts:88-95`, `createMatch` | C4 | Med | No uniqueness check on `(homeTeamId, awayTeamId, date)`. Duplicate matches accepted. Cross-references **B-003**. | Add a uniqueness guard or accept duplicates by design and document it. |
| F-05 | `controllers/reportController.ts:7-9` | C2 | Med | `limit` and `minGoals` query params are passed straight to `Number(...)`. Invalid input (`"abc"` → NaN; `"-1"` → negative) is then forwarded to `topScorersReport` and produces a confusing slice (cross-references **B-002**, **B-006**). | Validate: respond 400 if `limit !== undefined && (!Number.isFinite(limit) \|\| limit < 1)`; same for `minGoals`. |
| F-06 | `services/bonusService.ts:64-69` | C2 | Lo | `typeof n === "number"` accepts `Infinity` and `-Infinity`. `computeGoalBonus("FWD", 3, Infinity)` returns `{code: 0, bonus: 1500}` because `Infinity > 100`. The contract says `[0, 500]`. (Found independently by the BBT sub-agent.) | Add `!Number.isFinite(marketValueMillions)` to the rejection clause. |
| F-07 | `services/bonusService.ts:14-17` (doc) | C7 | Tri | Header comment says multipliers are applied "in order" but the implementation uses `if / else if` — only one branch can fire. (Found by BBT sub-agent.) | Rewrite the comment as a clear cascade. |
| F-08 | `services/bonusService.ts:46-53` | C5 | Lo | `code: 2` is overloaded — it means *both* "goals < 0" and "goals > 10" (and also "non-integer goals"). Hard for a UI to give actionable feedback. | Either split into `2a/2b/2c` codes, or return `{code, errors: string[]}`. |
| F-09 | `models/Player.ts:31` | C7 | Lo | `validatePlayerInput` mutates `errors` and returns it — fine — but the age-of-player check is buried inside the dateOfBirth branch and re-creates a `Date` after a regex. The function is 50 lines; readability suffers. | Extract `validateDOB(value): string[]`. |
| F-10 | `repository/InMemoryRepository.ts:39-45`, `deleteTeam` | C5 | Lo | Cascade-block is signalled with a `boolean` (true=deleted, false=had-dependents). The controller maps to 404 vs 409 by re-querying `getTeam`. Two round-trips. | Return a discriminated union `{ ok: true } \| { ok: false, reason: "not-found" \| "has-deps" }`. |
| F-11 | `models/Team.ts:9-18`, group letter literal | C7 | Tri | The list of valid groups appears in both `Team.ts` and `frontend/src/pages/TeamsPage.tsx`. | Move to a shared constants module or expose via the API. |
| F-12 | `app.ts` | C6 | Med | No authentication. `POST /api/reset` is open and wipes data. Acceptable for the take-home, but should be flagged. | Wrap `reset` behind an env-flag (`NODE_ENV !== "production"`) or a token. |
| F-13 | `services/reportService.ts:23-26` | C2 | Lo | `topScorersReport` accepts an undefined `options` but does not validate the field types if a partially-valid object is passed (e.g. `{limit: "5"}` would silently allow `.slice(0, "5")`, which JS happens to coerce — fragile contract). | Type-narrow at function entry. |
| F-14 | `services/matchResultService.ts:78` | C4 | Lo | The loop concatenates per-goal errors into `errors[]`, but a single bad goal aborts the whole record. Acceptable, but `errors` carries multiple messages even though only the first matters. | Either short-circuit on first error, or document "best-effort error reporting". |
| F-15 | `services/standingsService.ts:55-78` | C7 | Tri | The function uses three guard `continue` clauses in a row. Equivalent but harder to read than a single `if` with conjunctions. | Personal preference — leaving as-is is fine, but worth a comment. |
| F-16 | `frontend/src/api.ts:60` | C5 | Tri | Error path picks the *first* server message via `body.errors?.join("; ")`. Loses HTTP status context. | Surface `res.status` alongside the message for nicer UI text. |
| F-17 | All entity files | C2 | Tri | The phrase "60 characters or fewer" is correct but inconsistent with the player-name message (both say it). No bug — note only. | – |
| F-18 | `app/backend/src/server.ts` | C8 | Tri | The server self-seeds on startup; tests cannot import `server.ts` without the side-effect. We worked around it by importing `app.ts`. | Move `seed(repo)` into a `bootstrap()` only the CLI entry calls. |

**Total findings: 18** — 2 High, 4 Medium, 7 Low, 5 Trivial.

## Inspection statistics

| Metric | Value |
|---|---|
| Files inspected | 10 |
| LOC inspected | ~520 |
| Defects logged | 18 |
| High-severity | 2 |
| Medium-severity | 4 |
| Defect density | 3.5 defects / 100 LOC |
| Total inspection effort | ≈3 person-hours |
| Defect-detection rate | ≈6 defects / person-hour |

## Disposition / next actions

Findings F-01, F-02, F-03, F-04, F-05 should be fixed before the assignment is graded since they are also flagged by the exploratory testing pass. F-06 is a known limit in the BBT SUT and is intentionally left as documentation-only so the BBT/WBT artefacts have a stable expected output (changing the contract now would invalidate the xlsx). F-07 to F-18 are deferred to a clean-up pass.

A re-inspection round was not held within the take-home time budget; in a production setting we would schedule a second meeting to verify the fixes for the 2 High and 4 Medium items.
