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
  date: string; // ISO YYYY-MM-DD
  venue: string;
  homeScore: number | null;
  awayScore: number | null;
  status: MatchStatus;
  goals: Goal[];
}

export interface MatchInput {
  homeTeamId: number;
  awayTeamId: number;
  stage: Stage;
  date: string;
  venue: string;
}

const VALID_STAGES: Stage[] = ["Group", "R16", "QF", "SF", "Final"];
const ISO_DATE = /^\d{4}-\d{2}-\d{2}$/;

export function validateMatchInput(input: unknown): string[] {
  const errors: string[] = [];
  if (typeof input !== "object" || input === null) {
    return ["payload must be a JSON object"];
  }
  const m = input as Record<string, unknown>;
  if (typeof m.homeTeamId !== "number" || !Number.isInteger(m.homeTeamId) || m.homeTeamId <= 0) {
    errors.push("homeTeamId must be a positive integer");
  }
  if (typeof m.awayTeamId !== "number" || !Number.isInteger(m.awayTeamId) || m.awayTeamId <= 0) {
    errors.push("awayTeamId must be a positive integer");
  }
  if (
    typeof m.homeTeamId === "number" &&
    typeof m.awayTeamId === "number" &&
    m.homeTeamId === m.awayTeamId
  ) {
    errors.push("homeTeamId and awayTeamId must differ");
  }
  if (typeof m.stage !== "string" || !VALID_STAGES.includes(m.stage as Stage)) {
    errors.push("stage must be Group, R16, QF, SF or Final");
  }
  if (typeof m.date !== "string" || !ISO_DATE.test(m.date)) {
    errors.push("date must be in YYYY-MM-DD format");
  }
  if (typeof m.venue !== "string" || m.venue.trim().length === 0) {
    errors.push("venue is required");
  }
  return errors;
}
