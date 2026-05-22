import { expect, test } from "@playwright/test";
import { resetData } from "./support/reset.js";
import { fieldByLabel } from "./support/fields.js";

test.describe("Matches page CRUD", () => {
  test.beforeEach(async ({ request, page }) => {
    await resetData(request);
    await page.goto("/matches");
    await expect(page.getByTestId("matches-table")).toBeVisible();
  });

  test("schedules a new match, edits its venue, and verifies the table", async ({
    page,
  }) => {
    const form = page.getByTestId("match-form");

    // SCHEDULE: Mexico vs Canada, R16, 2026-07-05, Estadio Azteca.
    await fieldByLabel(form, "Home team").selectOption({ label: "Mexico" });
    await fieldByLabel(form, "Away team").selectOption({ label: "Canada" });
    await fieldByLabel(form, "Stage").selectOption("R16");
    await fieldByLabel(form, "Date").fill("2026-07-05");
    await fieldByLabel(form, "Venue").fill("Estadio Azteca");
    await form.getByRole("button", { name: "Schedule" }).click();

    // The new match shows in the table as Mexico vs Canada at Estadio Azteca.
    const row = page
      .getByTestId("matches-table")
      .locator("tbody tr", { hasText: "Estadio Azteca" });
    await expect(row).toBeVisible();
    await expect(row).toContainText("Mexico");
    await expect(row).toContainText("Canada");
    await expect(row).toContainText("R16");
    await expect(row).toContainText("2026-07-05");
    await expect(row).toContainText(/scheduled/i);

    // EDIT: move the match to a different venue.
    await row.getByRole("button", { name: "Edit" }).click();
    await expect(form.getByRole("heading")).toContainText("Edit match");
    await fieldByLabel(form, "Venue").fill("BC Place");
    await form.getByRole("button", { name: "Save" }).click();

    await expect(form.getByRole("heading")).toContainText("Schedule new match");

    // The original venue row is gone; new venue appears for that same pairing.
    await expect(
      page
        .getByTestId("matches-table")
        .locator("tbody tr", { hasText: "Estadio Azteca" }),
    ).toHaveCount(0);

    const updated = page
      .getByTestId("matches-table")
      .locator("tbody tr", { hasText: "BC Place" })
      .filter({ hasText: "Mexico" })
      .filter({ hasText: "Canada" });
    await expect(updated).toBeVisible();
  });
});
