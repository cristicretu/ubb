/**
 * Integration strategy: FULL HTTP-LEVEL with supertest.
 *   - Real (under test): app.ts + controllers + services + InMemoryRepository
 *                        (the whole vertical slice, instantiated per test).
 *   - No stubs, no mocks. Each `it` starts from a fresh repository so tests
 *     are independent.
 * This is the application-context integration test akin to
 * AppTest_LibraryReturnSystem_IsLeapYearStub from Lab 04, but over HTTP.
 */
import { describe, it, expect, beforeEach } from "vitest";
import request from "supertest";
import type { Express } from "express";
import { createApp } from "../../src/app.js";
import { InMemoryRepository } from "../../src/repository/InMemoryRepository.js";

let app: Express;
let repo: InMemoryRepository;

const teamUSA = {
  name: "United States",
  country: "USA",
  group: "A",
  coach: "Pochettino",
  fifaRanking: 16,
};
const teamMex = {
  name: "Mexico",
  country: "Mexico",
  group: "A",
  coach: "Aguirre",
  fifaRanking: 19,
};
const playerPulisic = {
  teamId: 1,
  name: "Christian Pulisic",
  position: "FWD",
  jerseyNumber: 10,
  dateOfBirth: "1998-09-18",
  marketValueMillions: 28,
};
const playerLozano = {
  teamId: 2,
  name: "Hirving Lozano",
  position: "FWD",
  jerseyNumber: 22,
  dateOfBirth: "1995-07-30",
  marketValueMillions: 18,
};

beforeEach(() => {
  repo = new InMemoryRepository();
  app = createApp(repo);
});

describe("/api/teams", () => {
  it("TC_1_post_creates_a_team_and_returns_201_with_id", async () => {
    const res = await request(app).post("/api/teams").send(teamUSA);
    expect(res.status).toBe(201);
    expect(res.body).toMatchObject({ id: 1, ...teamUSA });
  });

  it("TC_2_get_lists_teams_in_id_order", async () => {
    await request(app).post("/api/teams").send(teamUSA);
    await request(app).post("/api/teams").send(teamMex);
    const res = await request(app).get("/api/teams");
    expect(res.status).toBe(200);
    expect(res.body.map((t: { id: number }) => t.id)).toEqual([1, 2]);
  });

  it("TC_3_put_updates_the_team_record", async () => {
    await request(app).post("/api/teams").send(teamUSA);
    const res = await request(app)
      .put("/api/teams/1")
      .send({ ...teamUSA, coach: "Berhalter" });
    expect(res.status).toBe(200);
    expect(res.body.coach).toBe("Berhalter");
  });

  it("TC_4_post_invalid_payload_returns_400_with_errors", async () => {
    const res = await request(app)
      .post("/api/teams")
      .send({ ...teamUSA, group: "Z" });
    expect(res.status).toBe(400);
    expect(res.body.errors.length).toBeGreaterThan(0);
  });

  it("TC_5_delete_team_with_players_returns_409", async () => {
    await request(app).post("/api/teams").send(teamUSA);
    await request(app).post("/api/players").send(playerPulisic);
    const res = await request(app).delete("/api/teams/1");
    expect(res.status).toBe(409);
    expect(res.body.error).toContain("players or matches");
  });

  it("TC_6_delete_team_unknown_id_returns_404", async () => {
    const res = await request(app).delete("/api/teams/999");
    expect(res.status).toBe(404);
  });
});

describe("/api/players", () => {
  beforeEach(async () => {
    await request(app).post("/api/teams").send(teamUSA);
    await request(app).post("/api/teams").send(teamMex);
  });

  it("TC_7_post_creates_a_player_and_returns_201", async () => {
    const res = await request(app).post("/api/players").send(playerPulisic);
    expect(res.status).toBe(201);
    expect(res.body.id).toBe(1);
    expect(res.body.name).toBe("Christian Pulisic");
  });

  it("TC_8_post_player_referencing_unknown_team_returns_400", async () => {
    const res = await request(app)
      .post("/api/players")
      .send({ ...playerPulisic, teamId: 999 });
    expect(res.status).toBe(400);
    expect(res.body.errors[0]).toContain("non-existent team");
  });

  it("TC_9_list_filters_by_teamId_query_param", async () => {
    await request(app).post("/api/players").send(playerPulisic);
    await request(app).post("/api/players").send(playerLozano);
    const onlyUSA = await request(app).get("/api/players?teamId=1");
    expect(onlyUSA.status).toBe(200);
    expect(onlyUSA.body).toHaveLength(1);
    expect(onlyUSA.body[0].name).toBe("Christian Pulisic");
  });
});

describe("/api/matches and record result", () => {
  beforeEach(async () => {
    await request(app).post("/api/teams").send(teamUSA);
    await request(app).post("/api/teams").send(teamMex);
    await request(app).post("/api/players").send(playerPulisic);
    await request(app).post("/api/players").send(playerLozano);
  });

  const scheduledMatch = {
    homeTeamId: 1,
    awayTeamId: 2,
    stage: "Group",
    date: "2026-06-12",
    venue: "MetLife Stadium",
  };

  it("TC_10_post_creates_a_match_with_scheduled_status", async () => {
    const res = await request(app).post("/api/matches").send(scheduledMatch);
    expect(res.status).toBe(201);
    expect(res.body.status).toBe("Scheduled");
    expect(res.body.homeScore).toBeNull();
  });

  it("TC_11_post_match_with_unknown_home_team_returns_400", async () => {
    const res = await request(app)
      .post("/api/matches")
      .send({ ...scheduledMatch, homeTeamId: 99 });
    expect(res.status).toBe(400);
    expect(res.body.errors[0]).toContain("homeTeamId");
  });

  it("TC_12_record_result_happy_path_marks_match_completed", async () => {
    const created = await request(app).post("/api/matches").send(scheduledMatch);
    const matchId = created.body.id;
    const res = await request(app)
      .post(`/api/matches/${matchId}/result`)
      .send({
        homeScore: 2,
        awayScore: 1,
        homeGoals: [
          { playerId: 1, minute: 22 },
          { playerId: 1, minute: 67 },
        ],
        awayGoals: [{ playerId: 2, minute: 81 }],
      });
    expect(res.status).toBe(200);
    expect(res.body.status).toBe("Completed");
    expect(res.body.goals).toHaveLength(3);
  });

  it("TC_13_record_result_with_wrong_team_scorer_returns_400", async () => {
    const created = await request(app).post("/api/matches").send(scheduledMatch);
    const res = await request(app)
      .post(`/api/matches/${created.body.id}/result`)
      .send({
        homeScore: 1,
        awayScore: 0,
        // Lozano (id=2) plays for Mexico but is being credited as a USA goal.
        homeGoals: [{ playerId: 2, minute: 30 }],
        awayGoals: [],
      });
    expect(res.status).toBe(400);
    expect(res.body.errors.length).toBeGreaterThan(0);
  });

  it("TC_14_record_result_on_unknown_match_returns_400", async () => {
    const res = await request(app)
      .post("/api/matches/999/result")
      .send({ homeScore: 0, awayScore: 0, homeGoals: [], awayGoals: [] });
    expect(res.status).toBe(400);
    expect(res.body.errors).toContain("match not found");
  });

  it("TC_15_recording_result_twice_is_rejected", async () => {
    const created = await request(app).post("/api/matches").send(scheduledMatch);
    const body = {
      homeScore: 1,
      awayScore: 0,
      homeGoals: [{ playerId: 1, minute: 22 }],
      awayGoals: [],
    };
    const first = await request(app)
      .post(`/api/matches/${created.body.id}/result`)
      .send(body);
    expect(first.status).toBe(200);
    const second = await request(app)
      .post(`/api/matches/${created.body.id}/result`)
      .send(body);
    expect(second.status).toBe(400);
    expect(second.body.errors).toContain("match already completed");
  });
});

describe("/api/reports", () => {
  beforeEach(async () => {
    await request(app).post("/api/teams").send(teamUSA);
    await request(app).post("/api/teams").send(teamMex);
    await request(app).post("/api/players").send(playerPulisic);
    await request(app).post("/api/players").send(playerLozano);
    const m = await request(app).post("/api/matches").send({
      homeTeamId: 1,
      awayTeamId: 2,
      stage: "Group",
      date: "2026-06-12",
      venue: "MetLife Stadium",
    });
    await request(app)
      .post(`/api/matches/${m.body.id}/result`)
      .send({
        homeScore: 2,
        awayScore: 1,
        homeGoals: [
          { playerId: 1, minute: 22 },
          { playerId: 1, minute: 67 },
        ],
        awayGoals: [{ playerId: 2, minute: 81 }],
      });
  });

  it("TC_16_top_scorers_returns_aggregated_leaderboard", async () => {
    const res = await request(app).get("/api/reports/top-scorers");
    expect(res.status).toBe(200);
    expect(res.body).toHaveLength(2);
    expect(res.body[0]).toMatchObject({
      playerName: "Christian Pulisic",
      goals: 2,
      teamName: "United States",
    });
    expect(res.body[1]).toMatchObject({
      playerName: "Hirving Lozano",
      goals: 1,
    });
  });

  it("TC_17_top_scorers_with_limit_query_param_truncates", async () => {
    const res = await request(app).get("/api/reports/top-scorers?limit=1");
    expect(res.status).toBe(200);
    expect(res.body).toHaveLength(1);
    expect(res.body[0].playerName).toBe("Christian Pulisic");
  });

  it("TC_18_standings_returns_one_group_with_two_teams", async () => {
    const res = await request(app).get("/api/reports/standings");
    expect(res.status).toBe(200);
    expect(res.body).toHaveLength(1);
    expect(res.body[0].group).toBe("A");
    expect(res.body[0].rows).toHaveLength(2);
    // USA won 2-1 -> 3 points, Mexico lost -> 0.
    expect(res.body[0].rows[0]).toMatchObject({
      teamName: "United States",
      points: 3,
    });
    expect(res.body[0].rows[1]).toMatchObject({
      teamName: "Mexico",
      points: 0,
    });
  });
});
