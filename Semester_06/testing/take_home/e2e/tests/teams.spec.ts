import { expect, test } from "@playwright/test";
import { resetData } from "./support/reset.js";
import { fieldByLabel } from "./support/fields.js";

test.describe("Teams page CRUD", () => {
  test.beforeEach(async ({ request, page }) => {
    await resetData(request);
    await page.goto("/teams");
    await expect(page.getByTestId("teams-table")).toBeVisible();
  });

  test("creates a new team and shows it in the table", async ({ page }) => {
    const form = page.getByTestId("team-form");

    await fieldByLabel(form, "Name").fill("Romania");
    await fieldByLabel(form, "Country").fill("Romania");
    await fieldByLabel(form, "Group").selectOption("E");
    await fieldByLabel(form, "FIFA rank").fill("47");
    await fieldByLabel(form, "Coach").fill("Mircea Lucescu");
    await form.getByRole("button", { name: "Create" }).click();

    const row = page
      .getByTestId("teams-table")
      .locator("tbody tr", { hasText: "Romania" })
      .first();
    await expect(row).toBeVisible();
    await expect(row).toContainText("Romania");
    await expect(row).toContainText("Mircea Lucescu");
    await expect(row).toContainText("E");
  });

  test("edits an existing team and persists the new coach", async ({ page }) => {
    const usaRow = page
      .getByTestId("teams-table")
      .locator("tbody tr", { hasText: "United States" });
    await usaRow.getByRole("button", { name: "Edit" }).click();

    const form = page.getByTestId("team-form");
    await expect(form.getByRole("heading")).toContainText("Edit team");
    await fieldByLabel(form, "Coach").fill("Gregg Berhalter");
    await form.getByRole("button", { name: "Save" }).click();

    await expect(form.getByRole("heading")).toContainText("Add new team");
    await expect(
      page
        .getByTestId("teams-table")
        .locator("tbody tr", { hasText: "United States" }),
    ).toContainText("Gregg Berhalter");
  });

  test("deletes a team that has no dependencies", async ({ page }) => {
    // Create an unconstrained team so we don't trip the cascade rule.
    const form = page.getByTestId("team-form");
    await fieldByLabel(form, "Name").fill("Disposable FC");
    await fieldByLabel(form, "Country").fill("Nowhere");
    await fieldByLabel(form, "Group").selectOption("L");
    await fieldByLabel(form, "FIFA rank").fill("200");
    await fieldByLabel(form, "Coach").fill("Coach Anon");
    await form.getByRole("button", { name: "Create" }).click();

    const row = page
      .getByTestId("teams-table")
      .locator("tbody tr", { hasText: "Disposable FC" });
    await expect(row).toBeVisible();

    await row.getByRole("button", { name: "Delete" }).click();
    await expect(
      page
        .getByTestId("teams-table")
        .locator("tbody tr", { hasText: "Disposable FC" }),
    ).toHaveCount(0);
  });

  test("rejects creating a team with an empty name", async ({ page }) => {
    const form = page.getByTestId("team-form");
    // Country/Group/Coach/Rank are populated, but leave Name empty.
    await fieldByLabel(form, "Country").fill("Atlantis");
    await fieldByLabel(form, "Coach").fill("Aquaman");
    await fieldByLabel(form, "FIFA rank").fill("180");
    await form.getByRole("button", { name: "Create" }).click();

    const error = form.getByTestId("team-error");
    await expect(error).toBeVisible();
    await expect(error).toContainText(/name is required/i);
  });

  test("blocks deleting a team that has players (Argentina cascade guard)", async ({
    page,
  }) => {
    const argRow = page
      .getByTestId("teams-table")
      .locator("tbody tr", { hasText: "Argentina" });
    await argRow.getByRole("button", { name: "Delete" }).click();

    const error = page.getByTestId("team-error");
    await expect(error).toBeVisible();
    await expect(error).toContainText(/players or matches/i);
    // The team must still be present.
    await expect(
      page
        .getByTestId("teams-table")
        .locator("tbody tr", { hasText: "Argentina" }),
    ).toBeVisible();
  });
});
