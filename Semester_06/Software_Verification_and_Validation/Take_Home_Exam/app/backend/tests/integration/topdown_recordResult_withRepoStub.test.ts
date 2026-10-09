/**
 * Integration strategy: TOP-DOWN with a hand-written REPOSITORY STUB.
 *   - Real (under test): src/services/matchResultService.ts -> recordMatchResult
 *   - Stubbed:           InMemoryRepository (object literal exposing only
 *                        getMatch, getTeam, getPlayer, recordMatchResult)
 * Mirrors AppTest_DaysBetween2Dates_IsLeapYearStub from Lab 04: drivers feed
 * the SUT but the lower layer is replaced by a canned stub so we exercise
 * the service's branching alone.
 */
import { describe, it, expect, beforeEach } from "vitest";
import {
  recordMatchResult,
  type RecordResultInput,
} from "../../src/services/matchResultService.js";
import type { Match } from "../../src/models/Match.js";
import type { Team } from "../../src/models/Team.js";
import type { Player } from "../../src/models/Player.js";
import type { InMemoryRepository } from "../../src/repository/InMemoryRepository.js";

type RepoStub = Pick<
  InMemoryRepository,
  "getMatch" | "getTeam" | "getPlayer" | "recordMatchResult"
>;

interface StubCanned {
  match?: Match;
  teams: Record<number, Team>;
  players: Record<number, Player>;
}

function makeStub(canned: StubCanned): {
  repo: RepoStub;
  recordedCalls: Array<{
    id: number;
    homeScore: number;
    awayScore: number;
    goals: Match["goals"];
  }>;
} {
  const recordedCalls: Array<{
    id: number;
    homeScore: number;
    awayScore: number;
    goals: Match["goals"];
  }> = [];
  const repo: RepoStub = {
    getMatch: (id: number) =>
      canned.match && canned.match.id === id ? canned.match : undefined,
    getTeam: (id: number) => canned.teams[id],
    getPlayer: (id: number) => canned.players[id],
    recordMatchResult: (id, homeScore, awayScore, goals) => {
      recordedCalls.push({ id, homeScore, awayScore, goals });
      return canned.match;
    },
  };
  return { repo, recordedCalls };
}

const sampleHomeTeam: Team = {
  id: 1,
  name: "USA",
  country: "USA",
  group: "A",
  coach: "Pochettino",
  fifaRanking: 16,
};
const sampleAwayTeam: Team = {
  id: 2,
  name: "Mexico",
  country: "Mexico",
  group: "A",
  coach: "Aguirre",
  fifaRanking: 19,
};
const samplePulisic: Player = {
  id: 10,
  teamId: 1,
  name: "Pulisic",
  position: "FWD",
  jerseyNumber: 10,
  dateOfBirth: "1998-09-18",
  marketValueMillions: 28,
};
const sampleLozano: Player = {
  id: 20,
  teamId: 2,
  name: "Lozano",
  position: "FWD",
  jerseyNumber: 22,
  dateOfBirth: "1995-07-30",
  marketValueMillions: 18,
};
const sampleScheduledMatch: Match = {
  id: 99,
  homeTeamId: 1,
  awayTeamId: 2,
  stage: "Group",
  date: "2026-06-12",
  venue: "MetLife",
  homeScore: null,
  awayScore: null,
  status: "Scheduled",
  goals: [],
};

let canned: StubCanned;

beforeEach(() => {
  canned = {
    match: { ...sampleScheduledMatch, goals: [] },
    teams: { 1: sampleHomeTeam, 2: sampleAwayTeam },
    players: { 10: samplePulisic, 20: sampleLozano },
  };
});

describe("recordMatchResult (TOP-DOWN with repo stub)", () => {
  it("TC_1_happy_path_records_and_returns_ok", () => {
    const { repo, recordedCalls } = makeStub(canned);
    const input: RecordResultInput = {
      homeScore: 2,
      awayScore: 1,
      homeGoals: [
        { playerId: 10, minute: 22 },
        { playerId: 10, minute: 67 },
      ],
      awayGoals: [{ playerId: 20, minute: 81 }],
    };
    const out = recordMatchResult(repo as InMemoryRepository, 99, input);
    expect(out.ok).toBe(true);
    expect(out.errors).toEqual([]);
    expect(recordedCalls).toHaveLength(1);
    expect(recordedCalls[0].id).toBe(99);
    expect(recordedCalls[0].homeScore).toBe(2);
    expect(recordedCalls[0].awayScore).toBe(1);
    expect(recordedCalls[0].goals).toHaveLength(3);
    expect(recordedCalls[0].goals[0]).toEqual({
      playerId: 10,
      teamId: 1,
      minute: 22,
    });
  });

  it("TC_2_match_not_found_returns_error_and_does_not_persist", () => {
    canned.match = undefined;
    const { repo, recordedCalls } = makeStub(canned);
    const out = recordMatchResult(repo as InMemoryRepository, 404, {
      homeScore: 0,
      awayScore: 0,
      homeGoals: [],
      awayGoals: [],
    });
    expect(out.ok).toBe(false);
    expect(out.errors).toContain("match not found");
    expect(recordedCalls).toHaveLength(0);
  });

  it("TC_3_already_completed_match_is_rejected", () => {
    canned.match = { ...sampleScheduledMatch, status: "Completed" };
    const { repo, recordedCalls } = makeStub(canned);
    const out = recordMatchResult(repo as InMemoryRepository, 99, {
      homeScore: 1,
      awayScore: 0,
      homeGoals: [{ playerId: 10, minute: 30 }],
      awayGoals: [],
    });
    expect(out.ok).toBe(false);
    expect(out.errors).toContain("match already completed");
    expect(recordedCalls).toHaveLength(0);
  });

  it("TC_4_negative_home_score_fails_basic_validation", () => {
    const { repo, recordedCalls } = makeStub(canned);
    const out = recordMatchResult(repo as InMemoryRepository, 99, {
      homeScore: -1,
      awayScore: 0,
      homeGoals: [],
      awayGoals: [],
    });
    expect(out.ok).toBe(false);
    expect(out.errors).toContain("homeScore must be a non-negative integer");
    expect(recordedCalls).toHaveLength(0);
  });

  it("TC_5_non_integer_away_score_fails", () => {
    const { repo } = makeStub(canned);
    const out = recordMatchResult(repo as InMemoryRepository, 99, {
      homeScore: 0,
      awayScore: 1.5,
      homeGoals: [],
      awayGoals: [{ playerId: 20, minute: 10 }],
    });
    expect(out.ok).toBe(false);
    expect(out.errors).toContain("awayScore must be a non-negative integer");
  });

  it("TC_6_homeGoals_count_must_equal_homeScore", () => {
    const { repo, recordedCalls } = makeStub(canned);
    const out = recordMatchResult(repo as InMemoryRepository, 99, {
      homeScore: 2,
      awayScore: 0,
      homeGoals: [{ playerId: 10, minute: 30 }], // mismatch
      awayGoals: [],
    });
    expect(out.ok).toBe(false);
    expect(out.errors).toContain("homeGoals length must equal homeScore");
    expect(recordedCalls).toHaveLength(0);
  });

  it("TC_7_unknown_scorer_id_is_rejected_with_player_not_found", () => {
    const { repo, recordedCalls } = makeStub(canned);
    const out = recordMatchResult(repo as InMemoryRepository, 99, {
      homeScore: 1,
      awayScore: 0,
      homeGoals: [{ playerId: 9999, minute: 30 }],
      awayGoals: [],
    });
    expect(out.ok).toBe(false);
    expect(out.errors.some((e) => e.includes("player 9999 not found"))).toBe(
      true,
    );
    expect(recordedCalls).toHaveLength(0);
  });

  it("TC_8_scorer_belongs_to_wrong_team_is_rejected", () => {
    const { repo, recordedCalls } = makeStub(canned);
    // Lozano (id 20, teamId 2) is credited as a HOME goal — wrong team.
    const out = recordMatchResult(repo as InMemoryRepository, 99, {
      homeScore: 1,
      awayScore: 0,
      homeGoals: [{ playerId: 20, minute: 30 }],
      awayGoals: [],
    });
    expect(out.ok).toBe(false);
    expect(
      out.errors.some((e) => e.includes("Lozano") && e.includes("USA")),
    ).toBe(true);
    expect(recordedCalls).toHaveLength(0);
  });

  it("TC_9_minute_below_range_is_rejected", () => {
    const { repo } = makeStub(canned);
    const out = recordMatchResult(repo as InMemoryRepository, 99, {
      homeScore: 1,
      awayScore: 0,
      homeGoals: [{ playerId: 10, minute: 0 }],
      awayGoals: [],
    });
    expect(out.ok).toBe(false);
    expect(out.errors.some((e) => e.includes("invalid minute 0"))).toBe(true);
  });

  it("TC_10_minute_above_range_is_rejected", () => {
    const { repo } = makeStub(canned);
    const out = recordMatchResult(repo as InMemoryRepository, 99, {
      homeScore: 0,
      awayScore: 1,
      homeGoals: [],
      awayGoals: [{ playerId: 20, minute: 121 }],
    });
    expect(out.ok).toBe(false);
    expect(out.errors.some((e) => e.includes("invalid minute 121"))).toBe(true);
  });

  it("TC_11_deleted_home_team_returns_refresh_error", () => {
    canned.teams = { 2: sampleAwayTeam }; // home (id 1) is gone
    const { repo, recordedCalls } = makeStub(canned);
    const out = recordMatchResult(repo as InMemoryRepository, 99, {
      homeScore: 0,
      awayScore: 0,
      homeGoals: [],
      awayGoals: [],
    });
    expect(out.ok).toBe(false);
    expect(out.errors).toContain(
      "home/away team has been deleted; refresh match",
    );
    expect(recordedCalls).toHaveLength(0);
  });
});
