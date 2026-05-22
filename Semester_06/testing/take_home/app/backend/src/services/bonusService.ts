import type { Player, Position } from "../models/Player.js";

/**
 * Compute a player's tournament-performance bonus (in EUR).
 *
 * SUT for the Black-Box Testing exercise: rich input domain (position,
 * goals, market value) with well-defined equivalence classes and
 * boundaries, plus an error return code, mirroring the GiveBonus example
 * from Laboratory 2.
 *
 * Contract:
 *   Inputs:
 *     position           : "GK" | "DEF" | "MID" | "FWD"
 *     goals              : integer in [0, 10]   (>10 is treated as cheating)
 *     marketValueMillions: number  in [0, 500]
 *   Output:
 *     { code: 0, bonus: number }  on success
 *     { code: 1, bonus: 0 }       if position is invalid
 *     { code: 2, bonus: 0 }       if goals < 0 or goals > 10
 *     { code: 3, bonus: 0 }       if marketValue < 0 or marketValue > 500
 *   Special:
 *     goals == 0 yields { code: 0, bonus: 0 }
 *
 * Base rate per goal (EUR), reflecting positional rarity:
 *   FWD: 1000   MID: 1500   DEF: 2000   GK: 5000
 *
 * Multipliers (applied to base*goals, in order):
 *   marketValue <= 10  → multiplied by 1.5  (cheap player overperforming)
 *   marketValue >  100 → multiplied by 0.5  (expensive player, expected to score)
 */
export interface BonusResult {
  code: 0 | 1 | 2 | 3;
  bonus: number;
}

const BASE_RATE: Record<Position, number> = {
  FWD: 1000,
  MID: 1500,
  DEF: 2000,
  GK: 5000,
};

export function computeGoalBonus(
  position: string,
  goals: number,
  marketValueMillions: number,
): BonusResult {
  // 1) position validation
  if (
    position !== "GK" &&
    position !== "DEF" &&
    position !== "MID" &&
    position !== "FWD"
  ) {
    return { code: 1, bonus: 0 };
  }
  // 2) goals validation
  if (!Number.isInteger(goals) || goals < 0 || goals > 10) {
    return { code: 2, bonus: 0 };
  }
  // 3) market value validation
  if (
    typeof marketValueMillions !== "number" ||
    Number.isNaN(marketValueMillions) ||
    marketValueMillions < 0 ||
    marketValueMillions > 500
  ) {
    return { code: 3, bonus: 0 };
  }
  if (goals === 0) {
    return { code: 0, bonus: 0 };
  }
  let bonus = BASE_RATE[position as Position] * goals;
  if (marketValueMillions <= 10) {
    bonus = bonus * 1.5;
  } else if (marketValueMillions > 100) {
    bonus = bonus * 0.5;
  }
  return { code: 0, bonus };
}

/** Convenience wrapper that derives position and market value from a Player. */
export function computePlayerBonus(player: Player, goals: number): BonusResult {
  return computeGoalBonus(player.position, goals, player.marketValueMillions);
}
