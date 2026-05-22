import { expect, test } from "@playwright/test";
import { resetData } from "./support/reset.js";
import { fieldByLabel } from "./support/fields.js";

test.describe("Players page CRUD", () => {
  test.beforeEach(async ({ request, page }) => {
    await resetData(request);
    await page.goto("/players");
    await expect(page.getByTestId("players-table")).toBeVisible();
  });

  test("creates a player, edits position, filters by team, then deletes", async ({
    page,
  }) => {
    const form = page.getByTestId("player-form");

    // CREATE: add Enzo Fernandez to Argentina
    await fieldByLabel(form, "Name").fill("Enzo Fernandez");
    await fieldByLabel(form, "Team").selectOption({ label: "Argentina" });
    await fieldByLabel(form, "Position").selectOption("MID");
    await fieldByLabel(form, "Jersey #").fill("24");
    await fieldByLabel(form, "Date of birth").fill("2001-01-17");
    await fieldByLabel(form, "Market value (€M)").fill("100");
    await form.getByRole("button", { name: "Create" }).click();

    const newRow = page
      .getByTestId("players-table")
      .locator("tbody tr", { hasText: "Enzo Fernandez" });
    await expect(newRow).toBeVisible();
    await expect(newRow).toContainText("Argentina");
    await expect(newRow).toContainText("MID");
    await expect(newRow).toContainText("24");

    // EDIT: switch his position to DEF
    await newRow.getByRole("button", { name: "Edit" }).click();
    await expect(form.getByRole("heading")).toContainText("Edit player");
    await fieldByLabel(form, "Position").selectOption("DEF");
    await form.getByRole("button", { name: "Save" }).click();

    await expect(
      page
        .getByTestId("players-table")
        .locator("tbody tr", { hasText: "Enzo Fernandez" }),
    ).toContainText("DEF");

    // FILTER: pick Argentina; verify table is restricted to its 4 players.
    await page.getByTestId("player-filter").selectOption({ label: "Argentina" });
    const rows = page.getByTestId("players-table").locator("tbody tr");
    await expect(rows).toHaveCount(4); // Messi, Dybala, Martinez, Enzo
    for (const row of await rows.all()) {
      await expect(row).toContainText("Argentina");
    }

    // DELETE: remove Enzo Fernandez
    await page
      .getByTestId("players-table")
      .locator("tbody tr", { hasText: "Enzo Fernandez" })
      .getByRole("button", { name: "Delete" })
      .click();

    await expect(
      page
        .getByTestId("players-table")
        .locator("tbody tr", { hasText: "Enzo Fernandez" }),
    ).toHaveCount(0);

    // Argentina filter should now show the original 3 seeded players.
    await expect(
      page.getByTestId("players-table").locator("tbody tr"),
    ).toHaveCount(3);
  });
});
