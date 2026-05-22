import type { Match } from "../models/Match.js";

/**
 * Compute the group-stage points and goal statistics for a team across
 * a set of matches. SUT for the White-Box Testing exercise: contains
 * a loop, a guard, four boolean decisions and 1..7 explicit predicates,
 * mirroring the GiveNextDate example from Laboratory 3.
 *
 * Rules (FIFA group-stage scoring):
 *   - Only matches with status === "Completed" count.
 *   - Only matches at stage === "Group" count for the group-stage table.
 *   - A team gets 3 points for a win, 1 for a draw, 0 for a loss.
 *   - goalsFor/goalsAgainst accumulate over all counted matches.
 *
 * Inputs:
 *   teamId  : number > 0
 *   matches : Match[] (possibly empty)
 * Output:
 *   { played, won, drawn, lost, goalsFor, goalsAgainst, goalDifference, points }
 *
 * Error codes (returned via the .errorCode field):
 *   0 = OK
 *   1 = teamId is not a positive integer
 *   2 = matches is not an array
 */
export interface StandingsRow {
  errorCode: 0 | 1 | 2;
  played: number;
  won: number;
  drawn: number;
  lost: number;
  goalsFor: number;
  goalsAgainst: number;
  goalDifference: number;
  points: number;
}

export function emptyRow(errorCode: 0 | 1 | 2 = 0): StandingsRow {
  return {
    errorCode,
    played: 0,
    won: 0,
    drawn: 0,
    lost: 0,
    goalsFor: 0,
    goalsAgainst: 0,
    goalDifference: 0,
    points: 0,
  };
}

export function computeStandingsRow(
  teamId: number,
  matches: Match[],
): StandingsRow {
  if (!Number.isInteger(teamId) || teamId <= 0) {
    return emptyRow(1);
  }
  if (!Array.isArray(matches)) {
    return emptyRow(2);
  }
  const row = emptyRow(0);
  for (const m of matches) {
    if (m.status !== "Completed") {
      continue;
    }
    if (m.stage !== "Group") {
      continue;
    }
    if (m.homeTeamId !== teamId && m.awayTeamId !== teamId) {
      continue;
    }
    if (m.homeScore === null || m.awayScore === null) {
      continue;
    }
    const isHome = m.homeTeamId === teamId;
    const own = isHome ? m.homeScore : m.awayScore;
    const opp = isHome ? m.awayScore : m.homeScore;
    row.played += 1;
    row.goalsFor += own;
    row.goalsAgainst += opp;
    if (own > opp) {
      row.won += 1;
      row.points += 3;
    } else if (own === opp) {
      row.drawn += 1;
      row.points += 1;
    } else {
      row.lost += 1;
    }
  }
  row.goalDifference = row.goalsFor - row.goalsAgainst;
  return row;
}
