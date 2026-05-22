import type { Team, TeamInput } from "../models/Team.js";
import type { Player, PlayerInput } from "../models/Player.js";
import type { Match, MatchInput, Goal } from "../models/Match.js";

export class InMemoryRepository {
  private teams = new Map<number, Team>();
  private players = new Map<number, Player>();
  private matches = new Map<number, Match>();
  private nextTeamId = 1;
  private nextPlayerId = 1;
  private nextMatchId = 1;

  reset(): void {
    this.teams.clear();
    this.players.clear();
    this.matches.clear();
    this.nextTeamId = 1;
    this.nextPlayerId = 1;
    this.nextMatchId = 1;
  }

  // Teams
  listTeams(): Team[] {
    return Array.from(this.teams.values()).sort((a, b) => a.id - b.id);
  }
  getTeam(id: number): Team | undefined {
    return this.teams.get(id);
  }
  createTeam(input: TeamInput): Team {
    const team: Team = { id: this.nextTeamId++, ...input };
    this.teams.set(team.id, team);
    return team;
  }
  updateTeam(id: number, input: TeamInput): Team | undefined {
    if (!this.teams.has(id)) return undefined;
    const team: Team = { id, ...input };
    this.teams.set(id, team);
    return team;
  }
  deleteTeam(id: number): boolean {
    if (!this.teams.has(id)) return false;
    // cascade-block: cannot delete team with players or matches
    for (const p of this.players.values()) if (p.teamId === id) return false;
    for (const m of this.matches.values())
      if (m.homeTeamId === id || m.awayTeamId === id) return false;
    return this.teams.delete(id);
  }

  // Players
  listPlayers(): Player[] {
    return Array.from(this.players.values()).sort((a, b) => a.id - b.id);
  }
  listPlayersByTeam(teamId: number): Player[] {
    return this.listPlayers().filter((p) => p.teamId === teamId);
  }
  getPlayer(id: number): Player | undefined {
    return this.players.get(id);
  }
  createPlayer(input: PlayerInput): Player {
    const player: Player = { id: this.nextPlayerId++, ...input };
    this.players.set(player.id, player);
    return player;
  }
  updatePlayer(id: number, input: PlayerInput): Player | undefined {
    if (!this.players.has(id)) return undefined;
    const player: Player = { id, ...input };
    this.players.set(id, player);
    return player;
  }
  deletePlayer(id: number): boolean {
    if (!this.players.has(id)) return false;
    for (const m of this.matches.values())
      if (m.goals.some((g) => g.playerId === id)) return false;
    return this.players.delete(id);
  }

  // Matches
  listMatches(): Match[] {
    return Array.from(this.matches.values()).sort((a, b) => a.id - b.id);
  }
  getMatch(id: number): Match | undefined {
    return this.matches.get(id);
  }
  createMatch(input: MatchInput): Match {
    const match: Match = {
      id: this.nextMatchId++,
      ...input,
      homeScore: null,
      awayScore: null,
      status: "Scheduled",
      goals: [],
    };
    this.matches.set(match.id, match);
    return match;
  }
  updateMatch(id: number, input: MatchInput): Match | undefined {
    const existing = this.matches.get(id);
    if (!existing) return undefined;
    const updated: Match = { ...existing, ...input };
    this.matches.set(id, updated);
    return updated;
  }
  recordMatchResult(
    id: number,
    homeScore: number,
    awayScore: number,
    goals: Goal[],
  ): Match | undefined {
    const existing = this.matches.get(id);
    if (!existing) return undefined;
    const updated: Match = {
      ...existing,
      homeScore,
      awayScore,
      goals,
      status: "Completed",
    };
    this.matches.set(id, updated);
    return updated;
  }
  deleteMatch(id: number): boolean {
    return this.matches.delete(id);
  }
}

export const repository = new InMemoryRepository();
