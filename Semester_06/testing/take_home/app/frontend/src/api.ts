export type GroupLetter =
  | "A" | "B" | "C" | "D" | "E" | "F" | "G" | "H"
  | "I" | "J" | "K" | "L";

export interface Team {
  id: number;
  name: string;
  country: string;
  group: GroupLetter;
  coach: string;
  fifaRanking: number;
}

export type Position = "GK" | "DEF" | "MID" | "FWD";

export interface Player {
  id: number;
  teamId: number;
  name: string;
  position: Position;
  jerseyNumber: number;
  dateOfBirth: string;
  marketValueMillions: number;
}

export type Stage = "Group" | "R16" | "QF" | "SF" | "Final";
export type MatchStatus = "Scheduled" | "Completed";

export interface Goal {
  playerId: number;
  teamId: number;
  minute: number;
}

export interface Match {
  id: number;
  homeTeamId: number;
  awayTeamId: number;
  stage: Stage;
  date: string;
  venue: string;
  homeScore: number | null;
  awayScore: number | null;
  status: MatchStatus;
  goals: Goal[];
}

export interface TopScorerEntry {
  playerId: number;
  playerName: string;
  teamId: number;
  teamName: string;
  country: string;
  position: Position;
  goals: number;
}

async function request<T>(path: string, init?: RequestInit): Promise<T> {
  const res = await fetch(path, {
    headers: { "content-type": "application/json" },
    ...init,
  });
  if (!res.ok) {
    const body = await res.json().catch(() => ({}));
    const msg =
      body.errors?.join("; ") ?? body.error ?? `request failed (${res.status})`;
    throw new Error(msg);
  }
  if (res.status === 204) return undefined as T;
  return (await res.json()) as T;
}

export const api = {
  // teams
  listTeams: () => request<Team[]>("/api/teams"),
  createTeam: (t: Omit<Team, "id">) =>
    request<Team>("/api/teams", { method: "POST", body: JSON.stringify(t) }),
  updateTeam: (id: number, t: Omit<Team, "id">) =>
    request<Team>(`/api/teams/${id}`, { method: "PUT", body: JSON.stringify(t) }),
  deleteTeam: (id: number) =>
    request<void>(`/api/teams/${id}`, { method: "DELETE" }),

  // players
  listPlayers: () => request<Player[]>("/api/players"),
  listPlayersByTeam: (teamId: number) =>
    request<Player[]>(`/api/players?teamId=${teamId}`),
  createPlayer: (p: Omit<Player, "id">) =>
    request<Player>("/api/players", { method: "POST", body: JSON.stringify(p) }),
  updatePlayer: (id: number, p: Omit<Player, "id">) =>
    request<Player>(`/api/players/${id}`, { method: "PUT", body: JSON.stringify(p) }),
  deletePlayer: (id: number) =>
    request<void>(`/api/players/${id}`, { method: "DELETE" }),

  // matches
  listMatches: () => request<Match[]>("/api/matches"),
  createMatch: (m: {
    homeTeamId: number;
    awayTeamId: number;
    stage: Stage;
    date: string;
    venue: string;
  }) =>
    request<Match>("/api/matches", { method: "POST", body: JSON.stringify(m) }),
  updateMatch: (
    id: number,
    m: {
      homeTeamId: number;
      awayTeamId: number;
      stage: Stage;
      date: string;
      venue: string;
    },
  ) =>
    request<Match>(`/api/matches/${id}`, { method: "PUT", body: JSON.stringify(m) }),
  deleteMatch: (id: number) =>
    request<void>(`/api/matches/${id}`, { method: "DELETE" }),
  recordResult: (
    id: number,
    body: {
      homeScore: number;
      awayScore: number;
      homeGoals: { playerId: number; minute: number }[];
      awayGoals: { playerId: number; minute: number }[];
    },
  ) =>
    request<Match>(`/api/matches/${id}/result`, {
      method: "POST",
      body: JSON.stringify(body),
    }),

  // reports
  topScorers: (limit?: number) =>
    request<TopScorerEntry[]>(
      "/api/reports/top-scorers" + (limit ? `?limit=${limit}` : ""),
    ),

  reset: () => request<{ ok: boolean }>("/api/reset", { method: "POST" }),
};
