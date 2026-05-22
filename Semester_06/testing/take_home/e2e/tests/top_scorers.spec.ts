import { expect, test } from "@playwright/test";
import { resetData } from "./support/reset.js";

test.describe("Top Scorers report", () => {
  test.beforeEach(async ({ request, page }) => {
    await resetData(request);
    await page.goto("/top-scorers");
    await expect(page.getByTestId("top-scorers-table")).toBeVisible();
  });

  test("seed shows Pulisic and Vinicius Junior with 2 goals each", async ({
    page,
  }) => {
    const table = page.getByTestId("top-scorers-table");

    const pulisicRow = table.locator("tbody tr", { hasText: "Christian Pulisic" });
    await expect(pulisicRow).toBeVisible();
    await expect(pulisicRow).toContainText("United States");
    await expect(pulisicRow.locator("td").last()).toHaveText("2");

    const viniRow = table.locator("tbody tr", { hasText: "Vinicius Junior" });
    await expect(viniRow).toBeVisible();
    await expect(viniRow).toContainText("Brazil");
    await expect(viniRow.locator("td").last()).toHaveText("2");
  });

  test("changing limit to 3 restricts the table to the top three rows", async ({
    page,
  }) => {
    const table = page.getByTestId("top-scorers-table");

    // Default limit shows all 7 scorers from the seed.
    await expect(table.locator("tbody tr")).toHaveCount(7);

    const limit = page.getByTestId("top-scorers-limit");
    await limit.fill("3");

    // Wait for the table to settle at exactly three rows.
    await expect(table.locator("tbody tr")).toHaveCount(3);

    // Every row left must have at least 2 goals (the top 3 are tied at 2).
    const goalCells = table.locator("tbody tr td:last-child");
    const values = await goalCells.allTextContents();
    for (const v of values) {
      expect(Number(v)).toBeGreaterThanOrEqual(2);
    }
  });
});
