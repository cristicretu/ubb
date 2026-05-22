import type { Team } from "../models/Team.js";
import type { Player } from "../models/Player.js";
import type { Match } from "../models/Match.js";
import { computeStandingsRow, type StandingsRow } from "./standingsService.js";

export interface TopScorerEntry {
  playerId: number;
  playerName: string;
  teamId: number;
  teamName: string;
  country: string;
  position: string;
  goals: number;
}

/**
 * Cross-entity report — Top Scorers.
 *
 * Aggregates Match.goals across all completed matches and joins with
 * Player and Team data. Returns the leaderboard sorted by goal count
 * descending, then by player name ascending.
 *
 * This is the "1 functionality to generate a report on specific
 * information" required by Task 1; it touches all three CRUD entities
 * (Team, Player, Match) and is the read side of the
 * "Record Match Result" feature.
 */
export function topScorersReport(
  matches: Match[],
  players: Player[],
  teams: Team[],
  options?: { limit?: number; minGoals?: number },
): TopScorerEntry[] {
  const minGoals = options?.minGoals ?? 1;
  const limit = options?.limit ?? Number.POSITIVE_INFINITY;
  const playerById = new Map<number, Player>(players.map((p) => [p.id, p]));
  const teamById = new Map<number, Team>(teams.map((t) => [t.id, t]));
  const counts = new Map<number, number>();
  for (const m of matches) {
    if (m.status !== "Completed") continue;
    for (const g of m.goals) {
      counts.set(g.playerId, (counts.get(g.playerId) ?? 0) + 1);
    }
  }
  const entries: TopScorerEntry[] = [];
  for (const [playerId, goals] of counts) {
    if (goals < minGoals) continue;
    const player = playerById.get(playerId);
    if (!player) continue;
    const team = teamById.get(player.teamId);
    if (!team) continue;
    entries.push({
      playerId,
      playerName: player.name,
      teamId: team.id,
      teamName: team.name,
      country: team.country,
      position: player.position,
      goals,
    });
  }
  entries.sort((a, b) =>
    b.goals !== a.goals
      ? b.goals - a.goals
      : a.playerName.localeCompare(b.playerName),
  );
  return entries.slice(0, limit);
}

export interface GroupStanding {
  group: string;
  rows: Array<{
    teamId: number;
    teamName: string;
    country: string;
  } & StandingsRow>;
}

/**
 * Secondary cross-entity report — Group Standings.
 * Demonstrates an alternative way to combine all three entities.
 */
export function groupStandings(
  matches: Match[],
  teams: Team[],
): GroupStanding[] {
  const groups = new Map<string, GroupStanding>();
  for (const t of teams) {
    if (!groups.has(t.group)) groups.set(t.group, { group: t.group, rows: [] });
    const row = computeStandingsRow(t.id, matches);
    groups.get(t.group)!.rows.push({
      teamId: t.id,
      teamName: t.name,
      country: t.country,
      ...row,
    });
  }
  for (const g of groups.values()) {
    g.rows.sort((a, b) =>
      b.points !== a.points
        ? b.points - a.points
        : b.goalDifference !== a.goalDifference
          ? b.goalDifference - a.goalDifference
          : b.goalsFor - a.goalsFor,
    );
  }
  return Array.from(groups.values()).sort((a, b) =>
    a.group.localeCompare(b.group),
  );
}
