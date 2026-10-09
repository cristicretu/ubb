/**
 * Integration strategy: BOTTOM-UP with TEST DRIVERS.
 *   - Real (under test): src/repository/InMemoryRepository.ts
 *   - Drivers:           wrapper helpers below (seedTeams, seedPlayers,
 *                        attemptDelete, etc.) that sequence repo calls and
 *                        assert invariants without any higher layer.
 * Mirrors AppTest_DaysBetween2Dates_IsLeapYearStub in spirit — focus on
 * the lowest layer in isolation, walked through a sequence of CRUD
 * operations to expose contract-level behaviour (sequential ids,
 * cascade-block on referenced entities).
 */
import { describe, it, expect, beforeEach } from "vitest";
import { InMemoryRepository } from "../../src/repository/InMemoryRepository.js";
import type { Team } from "../../src/models/Team.js";
import type { Player } from "../../src/models/Player.js";

let repo: InMemoryRepository;

beforeEach(() => {
  repo = new InMemoryRepository();
});

// ---- Test drivers (analogue of LongestSeq's "wrapper" helpers) -----------
function driver_createTeams(count: number): Team[] {
  const out: Team[] = [];
  for (let i = 0; i < count; i++) {
    out.push(
      repo.createTeam({
        name: `Team ${i}`,
        country: `Country ${i}`,
        group: "A",
        coach: `Coach ${i}`,
        fifaRanking: i + 1,
      }),
    );
  }
  return out;
}

function driver_createPlayer(teamId: number, name = "Player X"): Player {
  return repo.createPlayer({
    teamId,
    name,
    position: "FWD",
    jerseyNumber: 9,
    dateOfBirth: "1995-01-01",
    marketValueMillions: 10,
  });
}

function driver_createScheduledMatch(homeId: number, awayId: number) {
  return repo.createMatch({
    homeTeamId: homeId,
    awayTeamId: awayId,
    stage: "Group",
    date: "2026-06-12",
    venue: "Stadium",
  });
}

describe("InMemoryRepository (BOTTOM-UP with drivers)", () => {
  describe("Teams", () => {
    it("TC_1_create_assigns_sequential_ids_starting_at_one", () => {
      const teams = driver_createTeams(3);
      expect(teams.map((t) => t.id)).toEqual([1, 2, 3]);
    });

    it("TC_2_listTeams_returns_sorted_by_id", () => {
      driver_createTeams(3);
      const listed = repo.listTeams();
      expect(listed.map((t) => t.id)).toEqual([1, 2, 3]);
    });

    it("TC_3_getTeam_unknown_id_returns_undefined", () => {
      driver_createTeams(1);
      expect(repo.getTeam(99)).toBeUndefined();
    });

    it("TC_4_updateTeam_overwrites_payload_but_keeps_id", () => {
      const [t] = driver_createTeams(1);
      const updated = repo.updateTeam(t.id, {
        name: "Renamed",
        country: "X",
        group: "B",
        coach: "New",
        fifaRanking: 50,
      });
      expect(updated).toBeDefined();
      expect(updated!.id).toBe(t.id);
      expect(updated!.name).toBe("Renamed");
      expect(updated!.group).toBe("B");
    });

    it("TC_5_updateTeam_unknown_id_returns_undefined", () => {
      expect(
        repo.updateTeam(42, {
          name: "X",
          country: "X",
          group: "A",
          coach: "X",
          fifaRanking: 1,
        }),
      ).toBeUndefined();
    });

    it("TC_6_deleteTeam_unknown_id_returns_false", () => {
      expect(repo.deleteTeam(42)).toBe(false);
    });

    it("TC_7_deleteTeam_with_players_is_blocked", () => {
      const [t1] = driver_createTeams(1);
      driver_createPlayer(t1.id);
      expect(repo.deleteTeam(t1.id)).toBe(false);
      expect(repo.getTeam(t1.id)).toBeDefined();
    });

    it("TC_8_deleteTeam_with_match_is_blocked", () => {
      const [t1, t2] = driver_createTeams(2);
      driver_createScheduledMatch(t1.id, t2.id);
      expect(repo.deleteTeam(t1.id)).toBe(false);
      expect(repo.deleteTeam(t2.id)).toBe(false);
    });

    it("TC_9_deleteTeam_succeeds_when_no_referencing_entities", () => {
      const [t1, t2] = driver_createTeams(2);
      expect(repo.deleteTeam(t1.id)).toBe(true);
      expect(repo.getTeam(t1.id)).toBeUndefined();
      expect(repo.getTeam(t2.id)).toBeDefined();
    });
  });

  describe("Players", () => {
    it("TC_10_createPlayer_assigns_sequential_ids_independent_of_teams", () => {
      const [t1] = driver_createTeams(1);
      const p1 = driver_createPlayer(t1.id, "A");
      const p2 = driver_createPlayer(t1.id, "B");
      expect(p1.id).toBe(1);
      expect(p2.id).toBe(2);
    });

    it("TC_11_listPlayersByTeam_filters_correctly", () => {
      const [t1, t2] = driver_createTeams(2);
      driver_createPlayer(t1.id, "T1-p1");
      driver_createPlayer(t1.id, "T1-p2");
      driver_createPlayer(t2.id, "T2-p1");
      expect(repo.listPlayersByTeam(t1.id).map((p) => p.name)).toEqual([
        "T1-p1",
        "T1-p2",
      ]);
      expect(repo.listPlayersByTeam(t2.id)).toHaveLength(1);
    });

    it("TC_12_updatePlayer_unknown_id_returns_undefined", () => {
      const [t1] = driver_createTeams(1);
      expect(
        repo.updatePlayer(99, {
          teamId: t1.id,
          name: "X",
          position: "FWD",
          jerseyNumber: 9,
          dateOfBirth: "1995-01-01",
          marketValueMillions: 10,
        }),
      ).toBeUndefined();
    });

    it("TC_13_deletePlayer_with_recorded_goal_is_blocked", () => {
      const [t1, t2] = driver_createTeams(2);
      const p = driver_createPlayer(t1.id);
      const m = driver_createScheduledMatch(t1.id, t2.id);
      repo.recordMatchResult(m.id, 1, 0, [
        { playerId: p.id, teamId: t1.id, minute: 10 },
      ]);
      expect(repo.deletePlayer(p.id)).toBe(false);
      expect(repo.getPlayer(p.id)).toBeDefined();
    });

    it("TC_14_deletePlayer_no_goals_succeeds", () => {
      const [t1] = driver_createTeams(1);
      const p = driver_createPlayer(t1.id);
      expect(repo.deletePlayer(p.id)).toBe(true);
      expect(repo.getPlayer(p.id)).toBeUndefined();
    });
  });

  describe("Matches", () => {
    it("TC_15_createMatch_defaults_status_to_scheduled_and_clears_scores", () => {
      const [t1, t2] = driver_createTeams(2);
      const m = driver_createScheduledMatch(t1.id, t2.id);
      expect(m.id).toBe(1);
      expect(m.status).toBe("Scheduled");
      expect(m.homeScore).toBeNull();
      expect(m.awayScore).toBeNull();
      expect(m.goals).toEqual([]);
    });

    it("TC_16_recordMatchResult_completes_and_persists_goals", () => {
      const [t1, t2] = driver_createTeams(2);
      const p = driver_createPlayer(t1.id);
      const m = driver_createScheduledMatch(t1.id, t2.id);
      const updated = repo.recordMatchResult(m.id, 1, 0, [
        { playerId: p.id, teamId: t1.id, minute: 5 },
      ]);
      expect(updated).toBeDefined();
      expect(updated!.status).toBe("Completed");
      expect(updated!.homeScore).toBe(1);
      expect(updated!.goals).toHaveLength(1);
    });

    it("TC_17_recordMatchResult_unknown_id_returns_undefined", () => {
      expect(repo.recordMatchResult(404, 0, 0, [])).toBeUndefined();
    });

    it("TC_18_deleteMatch_drops_the_record_and_frees_cascade", () => {
      const [t1, t2] = driver_createTeams(2);
      const m = driver_createScheduledMatch(t1.id, t2.id);
      expect(repo.deleteMatch(m.id)).toBe(true);
      // After match removal, both teams must now be deletable.
      expect(repo.deleteTeam(t1.id)).toBe(true);
      expect(repo.deleteTeam(t2.id)).toBe(true);
    });
  });

  describe("Reset", () => {
    it("TC_19_reset_clears_collections_and_rewinds_id_counters", () => {
      const [t1, t2] = driver_createTeams(2);
      driver_createPlayer(t1.id);
      driver_createScheduledMatch(t1.id, t2.id);
      repo.reset();
      expect(repo.listTeams()).toEqual([]);
      expect(repo.listPlayers()).toEqual([]);
      expect(repo.listMatches()).toEqual([]);
      const fresh = repo.createTeam({
        name: "Fresh",
        country: "Z",
        group: "A",
        coach: "Z",
        fifaRanking: 1,
      });
      expect(fresh.id).toBe(1);
    });
  });
});
