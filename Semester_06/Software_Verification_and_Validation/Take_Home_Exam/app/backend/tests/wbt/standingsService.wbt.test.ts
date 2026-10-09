import { describe, it, expect } from "vitest";
import {
  computeStandingsRow,
  emptyRow,
  type StandingsRow,
} from "../../src/services/standingsService.js";
import type { Match } from "../../src/models/Match.js";

/**
 * White-Box Testing — computeStandingsRow(teamId, matches)
 * --------------------------------------------------------
 * Each `it(...)` mirrors one row of docs/Task2_WBT_ComputeStandingsRow.xlsx
 * (sheet Req_TC_coverage). Test numbering (TC #) matches the workbook.
 *
 * CC = 13 (regions = E-N+2 = predicates+1). 14 test cases satisfy every
 * basis path plus statement, decision and loop coverage.
 */

/** Compact Match factory with sensible defaults. */
function makeMatch(overrides: Partial<Match> & { id: number }): Match {
  return {
    id: overrides.id,
    homeTeamId: overrides.homeTeamId ?? 1,
    awayTeamId: overrides.awayTeamId ?? 2,
    stage: overrides.stage ?? "Group",
    date: overrides.date ?? "2026-06-12",
    venue: overrides.venue ?? "Stadium",
    homeScore: overrides.homeScore ?? null,
    awayScore: overrides.awayScore ?? null,
    status: overrides.status ?? "Scheduled",
    goals: overrides.goals ?? [],
  };
}

/** Convenience: a clean zero row at errorCode=0. */
const ZERO_OK: StandingsRow = emptyRow(0);

describe("computeStandingsRow — input validation (Req paths 1-3)", () => {
  it("TC 1: teamId is not an integer (1.5) -> errorCode 1, all counters 0", () => {
    // Path 1: 1(T) -> 3 -> EXIT
    const actual = computeStandingsRow(1.5, []);
    expect(actual).toEqual(emptyRow(1));
  });

  it("TC 2: teamId is 0 (integer but <= 0) -> errorCode 1", () => {
    // Path 2: 1(F) -> 2(T) -> 3 -> EXIT
    const actual = computeStandingsRow(0, []);
    expect(actual).toEqual(emptyRow(1));
  });

  it("TC 3: matches is not an array (null) -> errorCode 2", () => {
    // Path 3: 1(F) -> 2(F) -> 4(T) -> 5 -> EXIT
    // Intentionally bypass the TS type via `as unknown as Match[]`.
    const actual = computeStandingsRow(1, null as unknown as Match[]);
    expect(actual).toEqual(emptyRow(2));
  });
});

describe("computeStandingsRow — loop coverage: zero iterations (Req path 4)", () => {
  it("TC 4: empty matches array -> empty row, errorCode 0", () => {
    // Path 4: 1(F) -> 2(F) -> 4(F) -> 6 -> 7(F) -> 24 -> EXIT
    expect(computeStandingsRow(1, [])).toEqual(ZERO_OK);
  });
});

describe("computeStandingsRow — loop coverage: 1 iter, all `continue` branches (Req paths 5-10)", () => {
  it("TC 5: status === 'Scheduled' is skipped (D4 TRUE)", () => {
    const matches: Match[] = [
      makeMatch({ id: 101, homeTeamId: 1, awayTeamId: 2, stage: "Group", status: "Scheduled" }),
    ];
    expect(computeStandingsRow(1, matches)).toEqual(ZERO_OK);
  });

  it("TC 6: stage === 'R16' is skipped (D5 TRUE)", () => {
    const matches: Match[] = [
      makeMatch({
        id: 102, homeTeamId: 1, awayTeamId: 2, stage: "R16",
        status: "Completed", homeScore: 2, awayScore: 1,
      }),
    ];
    expect(computeStandingsRow(1, matches)).toEqual(ZERO_OK);
  });

  it("TC 7: team not involved in match is skipped (D6a TRUE & D6b TRUE)", () => {
    const matches: Match[] = [
      makeMatch({
        id: 103, homeTeamId: 1, awayTeamId: 2, stage: "Group",
        status: "Completed", homeScore: 2, awayScore: 1,
      }),
    ];
    expect(computeStandingsRow(9, matches)).toEqual(ZERO_OK);
  });

  it("TC 8: team is HOME but homeScore is null (D6a FALSE -> D7a TRUE)", () => {
    const matches: Match[] = [
      makeMatch({
        id: 104, homeTeamId: 1, awayTeamId: 2, stage: "Group",
        status: "Completed", homeScore: null, awayScore: 2,
      }),
    ];
    expect(computeStandingsRow(1, matches)).toEqual(ZERO_OK);
  });

  it("TC 9: team is AWAY (D6a TRUE, D6b FALSE) but awayScore is null (D7b TRUE)", () => {
    const matches: Match[] = [
      makeMatch({
        id: 105, homeTeamId: 1, awayTeamId: 2, stage: "Group",
        status: "Completed", homeScore: 3, awayScore: null,
      }),
    ];
    expect(computeStandingsRow(2, matches)).toEqual(ZERO_OK);
  });
});

describe("computeStandingsRow — 1 iter, counted (Req paths 11-13)", () => {
  it("TC 10: team HOME wins 3-1 (D8 TRUE)", () => {
    const matches: Match[] = [
      makeMatch({
        id: 106, homeTeamId: 1, awayTeamId: 2, stage: "Group",
        status: "Completed", homeScore: 3, awayScore: 1,
      }),
    ];
    expect(computeStandingsRow(1, matches)).toEqual({
      errorCode: 0, played: 1, won: 1, drawn: 0, lost: 0,
      goalsFor: 3, goalsAgainst: 1, goalDifference: 2, points: 3,
    });
  });

  it("TC 11: team AWAY loses 1-3 (D8 FALSE, D9 FALSE)", () => {
    const matches: Match[] = [
      makeMatch({
        id: 107, homeTeamId: 1, awayTeamId: 2, stage: "Group",
        status: "Completed", homeScore: 3, awayScore: 1,
      }),
    ];
    expect(computeStandingsRow(2, matches)).toEqual({
      errorCode: 0, played: 1, won: 0, drawn: 0, lost: 1,
      goalsFor: 1, goalsAgainst: 3, goalDifference: -2, points: 0,
    });
  });

  it("TC 12: draw 2-2 for team HOME (D8 FALSE, D9 TRUE)", () => {
    const matches: Match[] = [
      makeMatch({
        id: 108, homeTeamId: 1, awayTeamId: 2, stage: "Group",
        status: "Completed", homeScore: 2, awayScore: 2,
      }),
    ];
    expect(computeStandingsRow(1, matches)).toEqual({
      errorCode: 0, played: 1, won: 0, drawn: 1, lost: 0,
      goalsFor: 2, goalsAgainst: 2, goalDifference: 0, points: 1,
    });
  });
});

describe("computeStandingsRow — loop coverage: > 1 iterations (Req paths 11+12+13 / mixed)", () => {
  it("TC 13: three group matches — 1W (3-1), 1D (2-2), 1L (0-1)", () => {
    const matches: Match[] = [
      makeMatch({
        id: 201, homeTeamId: 1, awayTeamId: 2, stage: "Group",
        status: "Completed", homeScore: 3, awayScore: 1,
      }),
      makeMatch({
        id: 202, homeTeamId: 1, awayTeamId: 3, stage: "Group",
        status: "Completed", homeScore: 2, awayScore: 2,
      }),
      makeMatch({
        id: 203, homeTeamId: 4, awayTeamId: 1, stage: "Group",
        status: "Completed", homeScore: 1, awayScore: 0,
      }),
    ];
    expect(computeStandingsRow(1, matches)).toEqual({
      errorCode: 0, played: 3, won: 1, drawn: 1, lost: 1,
      goalsFor: 5, goalsAgainst: 4, goalDifference: 1, points: 4,
    });
  });

  it("TC 14: 5 matches — 1 counted win + 4 skipped (status, stage, team, null score)", () => {
    const matches: Match[] = [
      makeMatch({
        id: 301, homeTeamId: 1, awayTeamId: 2, stage: "Group",
        status: "Completed", homeScore: 2, awayScore: 0,
      }), // counts as W
      makeMatch({
        id: 302, homeTeamId: 1, awayTeamId: 3, stage: "R16",
        status: "Completed", homeScore: 4, awayScore: 0,
      }), // stage skip
      makeMatch({
        id: 303, homeTeamId: 1, awayTeamId: 4, stage: "Group",
        status: "Scheduled",
      }), // status skip
      makeMatch({
        id: 304, homeTeamId: 5, awayTeamId: 6, stage: "Group",
        status: "Completed", homeScore: 1, awayScore: 1,
      }), // team skip
      makeMatch({
        id: 305, homeTeamId: 1, awayTeamId: 7, stage: "Group",
        status: "Completed", homeScore: null, awayScore: 2,
      }), // null-score skip
    ];
    expect(computeStandingsRow(1, matches)).toEqual({
      errorCode: 0, played: 1, won: 1, drawn: 0, lost: 0,
      goalsFor: 2, goalsAgainst: 0, goalDifference: 2, points: 3,
    });
  });
});
