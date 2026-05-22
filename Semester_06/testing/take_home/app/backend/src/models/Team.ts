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

export interface TeamInput {
  name: string;
  country: string;
  group: GroupLetter;
  coach: string;
  fifaRanking: number;
}

const VALID_GROUPS: GroupLetter[] = [
  "A", "B", "C", "D", "E", "F", "G", "H", "I", "J", "K", "L",
];

export function validateTeamInput(input: unknown): string[] {
  const errors: string[] = [];
  if (typeof input !== "object" || input === null) {
    return ["payload must be a JSON object"];
  }
  const t = input as Record<string, unknown>;
  if (typeof t.name !== "string" || t.name.trim().length === 0) {
    errors.push("name is required");
  } else if (t.name.length > 60) {
    errors.push("name must be 60 characters or fewer");
  }
  if (typeof t.country !== "string" || t.country.trim().length === 0) {
    errors.push("country is required");
  } else if (t.country.length > 60) {
    errors.push("country must be 60 characters or fewer");
  }
  if (typeof t.group !== "string" || !VALID_GROUPS.includes(t.group as GroupLetter)) {
    errors.push("group must be one of A..L");
  }
  if (typeof t.coach !== "string" || t.coach.trim().length === 0) {
    errors.push("coach is required");
  }
  if (
    typeof t.fifaRanking !== "number" ||
    !Number.isInteger(t.fifaRanking) ||
    t.fifaRanking < 1 ||
    t.fifaRanking > 210
  ) {
    errors.push("fifaRanking must be an integer between 1 and 210");
  }
  return errors;
}
