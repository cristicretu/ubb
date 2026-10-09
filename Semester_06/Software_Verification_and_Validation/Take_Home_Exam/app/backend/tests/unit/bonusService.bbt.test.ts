import { describe, expect, test } from "vitest";
import { computeGoalBonus } from "../../src/services/bonusService.js";

/**
 * Black-Box Testing for computeGoalBonus.
 *
 * Cases mirror docs/Task2_BBT_ComputeGoalBonus.xlsx:
 *   - "TC based on EP"  -> describe("Equivalence Partitioning ...")
 *   - "TC based on BVA" -> describe("Boundary Value Analysis ...")
 *
 * Test ids match the "No TC" column in the spreadsheet.
 */

interface BBTCase {
  no: number;
  description: string;
  position: string;
  // typed as `any` so we can exercise invalid inputs (non-integer goals, etc.)
  goals: any;
  marketValueMillions: any;
  expectedCode: 0 | 1 | 2 | 3;
  expectedBonus: number;
}

const epCases: BBTCase[] = [
  { no:  1, description: "EC 1,7,12: FWD, 3 goals, mv=50 -> base*goals*1",   position: "FWD", goals: 3,   marketValueMillions: 50,    expectedCode: 0, expectedBonus: 3000 },
  { no:  2, description: "EC 2,7,11: MID, 4 goals, mv=5  -> base*goals*1.5", position: "MID", goals: 4,   marketValueMillions: 5,     expectedCode: 0, expectedBonus: 9000 },
  { no:  3, description: "EC 3,7,13: DEF, 2 goals, mv=150 -> base*goals*0.5",position: "DEF", goals: 2,   marketValueMillions: 150,   expectedCode: 0, expectedBonus: 2000 },
  { no:  4, description: "EC 4,7,11: GK, 5 goals, mv=8 -> base*goals*1.5",   position: "GK",  goals: 5,   marketValueMillions: 8,     expectedCode: 0, expectedBonus: 37500 },
  { no:  5, description: "EC 1,6,12: FWD, goals=0 -> bonus 0",               position: "FWD", goals: 0,   marketValueMillions: 50,    expectedCode: 0, expectedBonus: 0 },
  { no:  6, description: "EC 2,6,13: MID, goals=0 with large mv -> bonus 0", position: "MID", goals: 0,   marketValueMillions: 200,   expectedCode: 0, expectedBonus: 0 },
  { no:  7, description: "EC 1,7,12: FWD, 7 goals, mv=11 (just outside x1.5)",position: "FWD",goals: 7,   marketValueMillions: 11,    expectedCode: 0, expectedBonus: 7000 },
  { no:  8, description: "EC 3,7,12: DEF, 6 goals, mv=99 (just inside x1)",  position: "DEF", goals: 6,   marketValueMillions: 99,    expectedCode: 0, expectedBonus: 12000 },
  { no:  9, description: "EC 4,7,13: GK, 10 goals, mv=500 -> base*goals*0.5",position: "GK",  goals: 10,  marketValueMillions: 500,   expectedCode: 0, expectedBonus: 25000 },
  { no: 10, description: "EC 5: invalid position 'ST'",                      position: "ST",  goals: 3,   marketValueMillions: 50,    expectedCode: 1, expectedBonus: 0 },
  { no: 11, description: "EC 8: goals=-1 (below valid range)",               position: "FWD", goals: -1,  marketValueMillions: 50,    expectedCode: 2, expectedBonus: 0 },
  { no: 12, description: "EC 9: goals=11 (above valid range)",               position: "MID", goals: 11,  marketValueMillions: 50,    expectedCode: 2, expectedBonus: 0 },
  { no: 13, description: "EC 10: goals=3.5 (non-integer)",                   position: "DEF", goals: 3.5, marketValueMillions: 50,    expectedCode: 2, expectedBonus: 0 },
  { no: 14, description: "EC 14: marketValueMillions=-1 (below range)",      position: "GK",  goals: 4,   marketValueMillions: -1,    expectedCode: 3, expectedBonus: 0 },
  { no: 15, description: "EC 15: marketValueMillions=501 (above range)",     position: "FWD", goals: 4,   marketValueMillions: 501,   expectedCode: 3, expectedBonus: 0 },
];

const bvaCases: BBTCase[] = [
  { no:  1, description: "position='GK' (valid)",                            position: "GK",  goals: 3,   marketValueMillions: 50,      expectedCode: 0, expectedBonus: 15000 },
  { no:  2, description: "position='FWD' (valid)",                           position: "FWD", goals: 3,   marketValueMillions: 50,      expectedCode: 0, expectedBonus: 3000 },
  { no:  3, description: "position='' (empty string, invalid)",              position: "",    goals: 3,   marketValueMillions: 50,      expectedCode: 1, expectedBonus: 0 },
  { no:  4, description: "position='gk' (case-sensitive miss, invalid)",     position: "gk",  goals: 3,   marketValueMillions: 50,      expectedCode: 1, expectedBonus: 0 },
  { no:  5, description: "goals=-1 (below lower boundary)",                  position: "MID", goals: -1,  marketValueMillions: 50,      expectedCode: 2, expectedBonus: 0 },
  { no:  6, description: "goals=0 (lower boundary, special-case bonus 0)",   position: "MID", goals: 0,   marketValueMillions: 50,      expectedCode: 0, expectedBonus: 0 },
  { no:  7, description: "goals=1 (just inside valid range)",                position: "MID", goals: 1,   marketValueMillions: 50,      expectedCode: 0, expectedBonus: 1500 },
  { no:  8, description: "goals=10 (upper boundary, inclusive)",             position: "MID", goals: 10,  marketValueMillions: 50,      expectedCode: 0, expectedBonus: 15000 },
  { no:  9, description: "goals=11 (just above upper boundary)",             position: "MID", goals: 11,  marketValueMillions: 50,      expectedCode: 2, expectedBonus: 0 },
  { no: 10, description: "marketValueMillions=-1 (below range)",             position: "FWD", goals: 3,   marketValueMillions: -1,      expectedCode: 3, expectedBonus: 0 },
  { no: 11, description: "marketValueMillions=0 (lower boundary, x1.5)",     position: "FWD", goals: 3,   marketValueMillions: 0,       expectedCode: 0, expectedBonus: 4500 },
  { no: 12, description: "marketValueMillions=10 (boundary, still x1.5)",    position: "FWD", goals: 3,   marketValueMillions: 10,      expectedCode: 0, expectedBonus: 4500 },
  { no: 13, description: "marketValueMillions=10.001 (just above x1.5)",     position: "FWD", goals: 3,   marketValueMillions: 10.001,  expectedCode: 0, expectedBonus: 3000 },
  { no: 14, description: "marketValueMillions=100 (boundary, still x1)",     position: "FWD", goals: 3,   marketValueMillions: 100,     expectedCode: 0, expectedBonus: 3000 },
  { no: 15, description: "marketValueMillions=100.001 (just above x1)",      position: "FWD", goals: 3,   marketValueMillions: 100.001, expectedCode: 0, expectedBonus: 1500 },
  { no: 16, description: "marketValueMillions=500 (upper boundary)",         position: "FWD", goals: 3,   marketValueMillions: 500,     expectedCode: 0, expectedBonus: 1500 },
  { no: 17, description: "marketValueMillions=500.001 (just above range)",   position: "FWD", goals: 3,   marketValueMillions: 500.001, expectedCode: 3, expectedBonus: 0 },
];

describe("Equivalence Partitioning - computeGoalBonus", () => {
  test.each(epCases)(
    "EP TC $no: $description",
    ({ position, goals, marketValueMillions, expectedCode, expectedBonus }) => {
      const result = computeGoalBonus(position, goals, marketValueMillions);
      expect(result.code).toBe(expectedCode);
      expect(result.bonus).toBe(expectedBonus);
    },
  );
});

describe("Boundary Value Analysis - computeGoalBonus", () => {
  test.each(bvaCases)(
    "BVA TC $no: $description",
    ({ position, goals, marketValueMillions, expectedCode, expectedBonus }) => {
      const result = computeGoalBonus(position, goals, marketValueMillions);
      expect(result.code).toBe(expectedCode);
      expect(result.bonus).toBe(expectedBonus);
    },
  );
});
