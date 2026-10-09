export type Position = "GK" | "DEF" | "MID" | "FWD";

export interface Player {
  id: number;
  teamId: number;
  name: string;
  position: Position;
  jerseyNumber: number;
  dateOfBirth: string; // ISO YYYY-MM-DD
  marketValueMillions: number;
}

export interface PlayerInput {
  teamId: number;
  name: string;
  position: Position;
  jerseyNumber: number;
  dateOfBirth: string;
  marketValueMillions: number;
}

const VALID_POSITIONS: Position[] = ["GK", "DEF", "MID", "FWD"];
const ISO_DATE = /^\d{4}-\d{2}-\d{2}$/;

export function validatePlayerInput(input: unknown): string[] {
  const errors: string[] = [];
  if (typeof input !== "object" || input === null) {
    return ["payload must be a JSON object"];
  }
  const p = input as Record<string, unknown>;
  if (typeof p.teamId !== "number" || !Number.isInteger(p.teamId) || p.teamId <= 0) {
    errors.push("teamId must be a positive integer");
  }
  if (typeof p.name !== "string" || p.name.trim().length === 0) {
    errors.push("name is required");
  } else if (p.name.length > 60) {
    errors.push("name must be 60 characters or fewer");
  }
  if (typeof p.position !== "string" || !VALID_POSITIONS.includes(p.position as Position)) {
    errors.push("position must be GK, DEF, MID or FWD");
  }
  if (
    typeof p.jerseyNumber !== "number" ||
    !Number.isInteger(p.jerseyNumber) ||
    p.jerseyNumber < 1 ||
    p.jerseyNumber > 99
  ) {
    errors.push("jerseyNumber must be an integer between 1 and 99");
  }
  if (typeof p.dateOfBirth !== "string" || !ISO_DATE.test(p.dateOfBirth)) {
    errors.push("dateOfBirth must be in YYYY-MM-DD format");
  } else {
    const [y, m, d] = p.dateOfBirth.split("-").map(Number);
    const dob = new Date(Date.UTC(y, m - 1, d));
    if (
      dob.getUTCFullYear() !== y ||
      dob.getUTCMonth() !== m - 1 ||
      dob.getUTCDate() !== d
    ) {
      errors.push("dateOfBirth is not a valid calendar date");
    } else {
      const today = new Date();
      const age =
        today.getUTCFullYear() - y -
        (today.getUTCMonth() < m - 1 ||
        (today.getUTCMonth() === m - 1 && today.getUTCDate() < d)
          ? 1
          : 0);
      if (age < 15 || age > 50) {
        errors.push("player age must be between 15 and 50 years");
      }
    }
  }
  if (
    typeof p.marketValueMillions !== "number" ||
    Number.isNaN(p.marketValueMillions) ||
    p.marketValueMillions < 0 ||
    p.marketValueMillions > 500
  ) {
    errors.push("marketValueMillions must be between 0 and 500");
  }
  return errors;
}
