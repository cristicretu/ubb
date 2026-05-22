import type { InMemoryRepository } from "../repository/InMemoryRepository.js";
import type { Goal } from "../models/Match.js";

export interface GoalInput {
  playerId: number;
  minute: number;
}

export interface RecordResultInput {
  homeScore: number;
  awayScore: number;
  homeGoals: GoalInput[];
  awayGoals: GoalInput[];
}

export interface RecordResultOutcome {
  ok: boolean;
  errors: string[];
}

/**
 * "Record Match Result" — the functionality that touches all three
 * CRUD entities (Team, Player, Match) and writes a Match update with
 * its scorer list, validating that:
 *  - the match exists and is still Scheduled
 *  - scores match the number of provided goals
 *  - each scorer is a known Player belonging to one of the two Teams
 *  - each minute is in [1, 120]
 */
export function recordMatchResult(
  repo: InMemoryRepository,
  matchId: number,
  input: RecordResultInput,
): RecordResultOutcome {
  const errors: string[] = [];
  const match = repo.getMatch(matchId);
  if (!match) return { ok: false, errors: ["match not found"] };
  if (match.status === "Completed")
    return { ok: false, errors: ["match already completed"] };

  if (!Number.isInteger(input.homeScore) || input.homeScore < 0)
    errors.push("homeScore must be a non-negative integer");
  if (!Number.isInteger(input.awayScore) || input.awayScore < 0)
    errors.push("awayScore must be a non-negative integer");
  if (!Array.isArray(input.homeGoals) || !Array.isArray(input.awayGoals))
    errors.push("homeGoals and awayGoals must be arrays");
  if (errors.length) return { ok: false, errors };

  if (input.homeGoals.length !== input.homeScore)
    errors.push("homeGoals length must equal homeScore");
  if (input.awayGoals.length !== input.awayScore)
    errors.push("awayGoals length must equal awayScore");

  const homeTeam = repo.getTeam(match.homeTeamId);
  const awayTeam = repo.getTeam(match.awayTeamId);
  if (!homeTeam || !awayTeam) {
    errors.push("home/away team has been deleted; refresh match");
    return { ok: false, errors };
  }

  const goals: Goal[] = [];
  for (const g of input.homeGoals) {
    const p = repo.getPlayer(g.playerId);
    if (!p) {
      errors.push(`player ${g.playerId} not found`);
      continue;
    }
    if (p.teamId !== homeTeam.id) {
      errors.push(`player ${p.name} does not play for ${homeTeam.name}`);
      continue;
    }
    if (!Number.isInteger(g.minute) || g.minute < 1 || g.minute > 120) {
      errors.push(`invalid minute ${g.minute} for ${p.name}`);
      continue;
    }
    goals.push({ playerId: p.id, teamId: homeTeam.id, minute: g.minute });
  }
  for (const g of input.awayGoals) {
    const p = repo.getPlayer(g.playerId);
    if (!p) {
      errors.push(`player ${g.playerId} not found`);
      continue;
    }
    if (p.teamId !== awayTeam.id) {
      errors.push(`player ${p.name} does not play for ${awayTeam.name}`);
      continue;
    }
    if (!Number.isInteger(g.minute) || g.minute < 1 || g.minute > 120) {
      errors.push(`invalid minute ${g.minute} for ${p.name}`);
      continue;
    }
    goals.push({ playerId: p.id, teamId: awayTeam.id, minute: g.minute });
  }
  if (errors.length) return { ok: false, errors };

  repo.recordMatchResult(matchId, input.homeScore, input.awayScore, goals);
  return { ok: true, errors: [] };
}
