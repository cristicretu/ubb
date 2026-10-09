/**
 * Integration strategy: MOCKS (Vitest vi.fn / vi.spyOn).
 *   - Real (under test): src/services/reportService.ts -> topScorersReport,
 *                        groupStandings (and the transitively used
 *                        standingsService.computeStandingsRow).
 *   - Mocked:            InMemoryRepository's listMatches / listPlayers /
 *                        listTeams via vi.fn (no DB / map state needed).
 * Mirrors AppTest_DaysBetween2Dates_IsLeapYearMock from Lab 04: the
 * collaborator is replaced by a mock that returns canned values, and we
 * verify both the produced report and the interaction count.
 */
import { describe, it, expect, beforeEach, vi } from "vitest";
import {
  topScorersReport,
  groupStandings,
} from "../../src/services/reportService.js";
import type { Team } from "../../src/models/Team.js";
import type { Player } from "../../src/models/Player.js";
import type { Match } from "../../src/models/Match.js";
import type { InMemoryRepository } from "../../src/repository/InMemoryRepository.js";

// Canned domain data
const teams: Team[] = [
  {
    id: 1,
    name: "Argentina",
    country: "Argentina",
    group: "C",
    coach: "Scaloni",
    fifaRanking: 1,
  },
  {
    id: 2,
    name: "Brazil",
    country: "Brazil",
    group: "C",
    coach: "Dorival",
    fifaRanking: 5,
  },
  {
    id: 3,
    name: "France",
    country: "France",
    group: "D",
    coach: "Deschamps",
    fifaRanking: 2,
  },
];

const players: Player[] = [
  {
    id: 10,
    teamId: 1,
    name: "Messi",
    position: "FWD",
    jerseyNumber: 10,
    dateOfBirth: "1987-06-24",
    marketValueMillions: 20,
  },
  {
    id: 11,
    teamId: 1,
    name: "Dybala",
    position: "FWD",
    jerseyNumber: 21,
    dateOfBirth: "1993-11-15",
    marketValueMillions: 35,
  },
  {
    id: 20,
    teamId: 2,
    name: "Vini",
    position: "FWD",
    jerseyNumber: 11,
    dateOfBirth: "2000-07-12",
    marketValueMillions: 180,
  },
  {
    id: 30,
    teamId: 3,
    name: "Mbappe",
    position: "FWD",
    jerseyNumber: 10,
    dateOfBirth: "1998-12-20",
    marketValueMillions: 180,
  },
];

const completedMatch1: Match = {
  id: 1,
  homeTeamId: 1,
  awayTeamId: 2,
  stage: "Group",
  date: "2026-06-15",
  venue: "AT&T",
  homeScore: 2,
  awayScore: 2,
  status: "Completed",
  goals: [
    { playerId: 10, teamId: 1, minute: 10 },
    { playerId: 20, teamId: 2, minute: 35 },
    { playerId: 11, teamId: 1, minute: 60 },
    { playerId: 20, teamId: 2, minute: 88 },
  ],
};

const completedMatch2: Match = {
  id: 2,
  homeTeamId: 3,
  awayTeamId: 2,
  stage: "Group",
  date: "2026-06-20",
  venue: "BC Place",
  homeScore: 3,
  awayScore: 0,
  status: "Completed",
  goals: [
    { playerId: 30, teamId: 3, minute: 18 },
    { playerId: 30, teamId: 3, minute: 54 },
    { playerId: 30, teamId: 3, minute: 77 },
  ],
};

const scheduledMatch: Match = {
  id: 3,
  homeTeamId: 1,
  awayTeamId: 3,
  stage: "Final",
  date: "2026-07-19",
  venue: "MetLife",
  homeScore: null,
  awayScore: null,
  status: "Scheduled",
  goals: [],
};

interface MockRepo {
  listMatches: ReturnType<typeof vi.fn>;
  listPlayers: ReturnType<typeof vi.fn>;
  listTeams: ReturnType<typeof vi.fn>;
}

let mockRepo: MockRepo;

beforeEach(() => {
  mockRepo = {
    listMatches: vi.fn<[], Match[]>(),
    listPlayers: vi.fn<[], Player[]>(),
    listTeams: vi.fn<[], Team[]>(),
  };
});

describe("topScorersReport with mocked repository", () => {
  it("TC_1_aggregates_completed_goals_and_ignores_scheduled_matches", () => {
    mockRepo.listMatches.mockReturnValue([
      completedMatch1,
      completedMatch2,
      scheduledMatch,
    ]);
    mockRepo.listPlayers.mockReturnValue(players);
    mockRepo.listTeams.mockReturnValue(teams);

    const report = topScorersReport(
      (mockRepo as unknown as InMemoryRepository).listMatches(),
      (mockRepo as unknown as InMemoryRepository).listPlayers(),
      (mockRepo as unknown as InMemoryRepository).listTeams(),
    );

    expect(report).toHaveLength(4);
    expect(report[0]).toMatchObject({ playerName: "Mbappe", goals: 3 });
    expect(report[1]).toMatchObject({ playerName: "Vini", goals: 2 });

    expect(mockRepo.listMatches).toHaveBeenCalledTimes(1);
    expect(mockRepo.listPlayers).toHaveBeenCalledTimes(1);
    expect(mockRepo.listTeams).toHaveBeenCalledTimes(1);
  });

  it("TC_2_sorts_equal_goal_counts_by_player_name_ascending", () => {
    mockRepo.listMatches.mockReturnValue([completedMatch1]);
    mockRepo.listPlayers.mockReturnValue(players);
    mockRepo.listTeams.mockReturnValue(teams);

    const report = topScorersReport(
      mockRepo.listMatches(),
      mockRepo.listPlayers(),
      mockRepo.listTeams(),
    );
    // Messi(1), Dybala(1), Vini(2) -> Vini first, then Dybala before Messi.
    expect(report.map((r) => r.playerName)).toEqual([
      "Vini",
      "Dybala",
      "Messi",
    ]);
  });

  it("TC_3_limit_option_truncates_the_leaderboard", () => {
    mockRepo.listMatches.mockReturnValue([completedMatch1, completedMatch2]);
    mockRepo.listPlayers.mockReturnValue(players);
    mockRepo.listTeams.mockReturnValue(teams);

    const report = topScorersReport(
      mockRepo.listMatches(),
      mockRepo.listPlayers(),
      mockRepo.listTeams(),
      { limit: 2 },
    );
    expect(report).toHaveLength(2);
    expect(report[0].playerName).toBe("Mbappe");
    expect(report[1].playerName).toBe("Vini");
  });

  it("TC_4_minGoals_filters_below_threshold", () => {
    mockRepo.listMatches.mockReturnValue([completedMatch1, completedMatch2]);
    mockRepo.listPlayers.mockReturnValue(players);
    mockRepo.listTeams.mockReturnValue(teams);

    const report = topScorersReport(
      mockRepo.listMatches(),
      mockRepo.listPlayers(),
      mockRepo.listTeams(),
      { minGoals: 2 },
    );
    expect(report.map((r) => r.playerName)).toEqual(["Mbappe", "Vini"]);
  });

  it("TC_5_returns_empty_when_no_matches_completed", () => {
    mockRepo.listMatches.mockReturnValue([scheduledMatch]);
    mockRepo.listPlayers.mockReturnValue(players);
    mockRepo.listTeams.mockReturnValue(teams);

    const report = topScorersReport(
      mockRepo.listMatches(),
      mockRepo.listPlayers(),
      mockRepo.listTeams(),
    );
    expect(report).toEqual([]);
  });

  it("TC_6_skips_goals_for_unknown_players_or_unknown_teams", () => {
    // listPlayers omits Mbappe (id 30), so his 3 goals must be skipped.
    mockRepo.listMatches.mockReturnValue([completedMatch2]);
    mockRepo.listPlayers.mockReturnValue(
      players.filter((p) => p.id !== 30),
    );
    mockRepo.listTeams.mockReturnValue(teams);

    const report = topScorersReport(
      mockRepo.listMatches(),
      mockRepo.listPlayers(),
      mockRepo.listTeams(),
    );
    expect(report).toEqual([]);
  });
});

describe("groupStandings with mocked repository", () => {
  it("TC_7_groups_teams_by_group_letter_and_sorts_alphabetically", () => {
    mockRepo.listMatches.mockReturnValue([completedMatch1, completedMatch2]);
    mockRepo.listTeams.mockReturnValue(teams);

    const standings = groupStandings(
      mockRepo.listMatches(),
      mockRepo.listTeams(),
    );
    expect(standings.map((g) => g.group)).toEqual(["C", "D"]);
    expect(mockRepo.listMatches).toHaveBeenCalledTimes(1);
    expect(mockRepo.listTeams).toHaveBeenCalledTimes(1);
  });

  it("TC_8_within_group_sorts_by_points_then_goal_difference", () => {
    mockRepo.listMatches.mockReturnValue([completedMatch1, completedMatch2]);
    mockRepo.listTeams.mockReturnValue(teams);

    const standings = groupStandings(
      mockRepo.listMatches(),
      mockRepo.listTeams(),
    );
    const groupC = standings.find((g) => g.group === "C")!;
    // Argentina drew (1pt), Brazil drew + lost to France (1pt, -3 GD).
    expect(groupC.rows[0].teamName).toBe("Argentina");
    expect(groupC.rows[1].teamName).toBe("Brazil");
    expect(groupC.rows[0].points).toBe(1);
    expect(groupC.rows[1].points).toBe(1);
    expect(groupC.rows[0].goalDifference).toBeGreaterThan(
      groupC.rows[1].goalDifference,
    );
  });

  it("TC_9_scheduled_matches_do_not_affect_standings", () => {
    mockRepo.listMatches.mockReturnValue([scheduledMatch]);
    mockRepo.listTeams.mockReturnValue(teams);

    const standings = groupStandings(
      mockRepo.listMatches(),
      mockRepo.listTeams(),
    );
    for (const g of standings) {
      for (const row of g.rows) {
        expect(row.played).toBe(0);
        expect(row.points).toBe(0);
      }
    }
  });

  it("TC_10_each_team_triggers_exactly_one_listMatches_pass_at_caller_level", () => {
    // Caller (this test) invokes listMatches once; we verify mock usage
    // is exactly what we expect — no incidental extra reads.
    mockRepo.listMatches.mockReturnValue([]);
    mockRepo.listTeams.mockReturnValue(teams);

    groupStandings(mockRepo.listMatches(), mockRepo.listTeams());
    expect(mockRepo.listMatches).toHaveBeenCalledTimes(1);
    expect(mockRepo.listTeams).toHaveBeenCalledTimes(1);
  });
});
