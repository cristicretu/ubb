# Task 3 — Exploratory Testing (session-based)

**Group 933** — Cretu Cristian (uploader), Grancea Alexandru, Deaconu Victor, Draguta Vasile
**Application under test:** FIFA World Cup 2026 Tracker (this repo, port 2069 frontend / 2070 backend)
**Session date:** 2026-05-22
**Approach:** session-based exploratory testing (SBTM) following Itkonen / Bach. One time-boxed session, one charter, freeform exploration, all findings logged inline.

---

## Charter

> *Test the **input domain and the cross-entity consistency** of the three CRUD entities (Team, Player, Match) and the **Record Match Result** flow. Look for: validation gaps, boundary leaks, broken referential integrity, and report inconsistencies. Out of scope: visual/UX polish, performance.*

**Time-box:** 60 minutes
**Tester:** rotating — Cretu (lead), Grancea/Deaconu/Draguta on follow-up probes
**Heuristics used:** SFDIPOT (Structure–Function–Data–Interfaces–Platform–Operations–Time), CRUD-Create/Read/Update/Delete pairs, boundary value analysis on every numeric field, "what if the user does the dumb thing?"

## Test environment

- macOS 14 / Chromium via Chrome DevTools MCP for UI probes.
- `curl` + the live REST API on `http://localhost:2070` for fast input-domain sweeps.
- Seed data: 6 teams, 9 players, 4 matches (3 completed) — restored with `POST /api/reset` before and after the session.

---

## Session log

Each row is one tested behaviour. **Status**: ✓ (works as expected), ✗ (bug — written up below), ? (suspicious; needs follow-up).

| # | What we tried | Path | Status | Note |
|---|---|---|---|---|
| T1 | Submit empty Team form via UI | UI `/teams` | ✓ | Inline error: "name is required; country is required; coach is required" |
| T2 | Delete Argentina (has players + matches) | UI `/teams` | ✓ | 409 with cascade message |
| T3 | Create team via UI then delete it | UI `/teams` | ✓ | Romania round-trip |
| T4 | Match with same home + away team | API | ✓ | "homeTeamId and awayTeamId must differ" |
| T5 | jerseyNumber = 0 (BVA below) | API | ✓ | rejected |
| T6 | jerseyNumber = 100 (BVA above) | API | ✓ | rejected |
| T7 | Player born 2015 (age ~11) | API | ✓ | "player age must be between 15 and 50 years" |
| T8 | marketValue = -1 | API | ✓ | rejected |
| T9 | marketValue = 501 | API | ✓ | rejected |
| T10 | Team name = "   " (whitespace) | API | ✓ | "name is required" — trim used |
| T11 | group = "M" | API | ✓ | rejected (A..L valid) |
| T12 | fifaRanking = 0 | API | ✓ | rejected |
| T13 | fifaRanking = 211 | API | ✓ | rejected |
| T14 | name = `<script>alert(1)</script>` | API + UI | ? | Backend stores it raw. React escapes on render, so no XSS in DOM — but no defense-in-depth on the backend. |
| T15 | Record result on already-completed match | API | ✓ | "match already completed" |
| T16 | Scorer player belongs to wrong team | API | ✓ | "player Kylian Mbappe does not play for Argentina" |
| T17 | Goal minute = 0 | API | ✓ | rejected |
| T18 | Goal minute = 121 | API | ✓ | rejected |
| T19 | homeScore != homeGoals.length | API | ✓ | rejected |
| T20 | homeScore = -1 | API | ✓ | rejected |
| T21 | Unknown playerId in goals | API | ✓ | "player 9999 not found" |
| T22 | Match id 777 not found | API | ✓ | "match not found" |
| T23 | top-scorers ?limit=0 | API | ✓ | returns [] |
| T24 | top-scorers ?limit=-1 | API | ✗ | **B-002** — returns wrong slice (`slice(0,-1)`); should reject negative limit |
| T25 | fifaRanking = 1e9 | API | ✓ | rejected |
| T26 | fifaRanking = 5.5 (float) | API | ✓ | rejected (Number.isInteger guard) |
| T27 | Match date = 1999-01-01 | API | ✗ | **B-001** — accepted; should be inside 2026 window |
| T28 | Team name length = 61 chars | API | ✓ | rejected |
| T29 | Team name length = 60 chars (cap) | API | ✓ | accepted (on-boundary) |
| T30 | Delete player who has scored goals | API | ✓ | cascade-blocked |
| T31 | PUT a Completed match, change stage to R16 | API | ✗ | **B-005** — change applied; result not reset, goals kept; integrity at risk |
| T32 | Create duplicate match (same home,away,date) | API | ✗ | **B-003** — accepted, second copy created |
| T33 | Verify top-scorers consistency | API | ✓ | output matches seed |
| T34 | top-scorers ?minGoals=2 filter | API | ✓ | only 2+ |
| T35 | top-scorers ?limit=abc (NaN) | API | ✗ | **B-006** — silently returns [] instead of 400 |
| T36 | Two players, same teamId + same jerseyNumber | API | ✗ | **B-004** — accepted; no jersey-uniqueness rule |

**Outcome:** 28 expectations confirmed (✓), 6 bugs (✗), 1 suspicious finding (?). Charter was satisfied within 60 minutes; no follow-up charter required.

---

## Bug list

See `Task3_InspectionReview.md` for the full inspection-review write-up; these are the items surfaced *exclusively* by exploratory testing and not by static code review.

| ID | Severity | Title | Repro | Notes |
|---|---|---|---|---|
| B-001 | Medium | Match date is not constrained to 2026 | `POST /api/matches {date:"1999-01-01", ...}` returns 201 | Trivial fix: add a range check (`>= "2026-01-01" && <= "2026-12-31"`) in `validateMatchInput` |
| B-002 | Low | Negative `limit` on top-scorers returns "all-but-last" | `GET /api/reports/top-scorers?limit=-1` | Validate `limit > 0` in controller; default to all |
| B-003 | Medium | Duplicate matches allowed | Two `POST /api/matches` with same home, away, date → both stored | Add uniqueness check in repo |
| B-004 | Medium | Two players in same team can share jerseyNumber | `POST /api/players` twice with same teamId+jerseyNumber → both 201 | Add uniqueness check in repo |
| B-005 | High | Editing a Completed match keeps stale result | `PUT /api/matches/1` with new teams/stage; previous goals stay attached | Either forbid PUT on Completed or reset score+goals when teams change |
| B-006 | Low | Garbage query params silently swallowed | `?limit=abc` → returns [] | Validate query parsing and respond 400 |
| B-007 | Trivial | No backend HTML/script sanitization | T14 | Rendered safely by React, but defense-in-depth missing |

All bugs reproduced ≥2 times during the session. None require source-code access to reproduce.

---

## Tool note

A short demo of **Bug Magnet** (used in the Seminar 3 portfolio) was *not* re-run here — the previously delivered video covers it. For this session we used:

- **curl** as the input-domain probe — fastest way to enumerate 30 boundaries.
- **Chrome DevTools MCP** (`navigate_page`, `click`, `fill`, `take_snapshot`) for the UI-driven path.

---

## Reflection

Most of our validation lives in the `validate*Input` functions and is well-covered. The bugs we found cluster around **state transitions** (B-001, B-005) and **uniqueness** (B-003, B-004), which we historically don't test as part of input validation. This matches the Afzal et al. (2015) finding from our Seminar-3 portfolio paper: ET surfaces a different *kind* of defect than scripted TCT — fewer trivial input-validation misses, more transition / consistency issues. The plain seven-bug yield in a 60-minute window is consistent with the paper's efficiency claim.
