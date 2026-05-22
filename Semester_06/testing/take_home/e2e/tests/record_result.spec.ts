import { expect, test } from "@playwright/test";
import { resetData } from "./support/reset.js";

test.describe("Record match result (cross-entity flow)", () => {
  test.beforeEach(async ({ request }) => {
    await resetData(request);
  });

  test("records a 2-1 Argentina win in the Final and acknowledges success", async ({
    page,
  }) => {
    await page.goto("/record");

    // Pick the only scheduled match (the Final: Argentina vs France).
    const select = page.getByTestId("record-match-select");
    await expect(select).toBeVisible();
    const finalOption = select.locator("option", {
      hasText: "Final",
    });
    await expect(finalOption).toHaveCount(1);
    const finalValue = await finalOption.getAttribute("value");
    if (!finalValue) throw new Error("could not resolve Final match option value");
    await select.selectOption(finalValue);

    const homePanel = page.getByTestId("record-home");
    const awayPanel = page.getByTestId("record-away");
    await expect(homePanel).toContainText("Argentina");
    await expect(awayPanel).toContainText("France");

    // Final score: 2 - 1
    await page.getByTestId("record-home-score").fill("2");
    await page.getByTestId("record-away-score").fill("1");

    // Wait for goal editor rows to appear and react state to settle.
    await expect(homePanel.locator("tbody tr")).toHaveCount(2);
    await expect(awayPanel.locator("tbody tr")).toHaveCount(1);

    // Assign Argentina goals to Messi (min 12) and Dybala (min 78).
    const homeRows = homePanel.locator("tbody tr");
    await homeRows.nth(0).locator("select").selectOption({ label: "Lionel Messi (FWD)" });
    await homeRows.nth(0).locator("input[type='number']").fill("12");
    await homeRows.nth(1).locator("select").selectOption({ label: "Paulo Dybala (FWD)" });
    await homeRows.nth(1).locator("input[type='number']").fill("78");

    // Assign France goal to Mbappe (min 55).
    const awayRow = awayPanel.locator("tbody tr").first();
    await awayRow.locator("select").selectOption({ label: "Kylian Mbappe (FWD)" });
    await awayRow.locator("input[type='number']").fill("55");

    // SUBMIT
    await page.getByRole("button", { name: "Save result" }).click();

    // ACK: success banner appears.
    const ok = page.getByTestId("record-ok");
    await expect(ok).toBeVisible();
    await expect(ok).toContainText(/recorded/i);

    // Cross-entity verification: the match is no longer in the scheduled
    // select, and the matches table shows the 2 – 1 score.
    await expect(
      select.locator("option", { hasText: /Argentina vs France/ }),
    ).toHaveCount(0);

    await page.goto("/matches");
    const finalRow = page
      .getByTestId("matches-table")
      .locator("tbody tr", { hasText: "Final" });
    await expect(finalRow).toContainText("2 – 1");
  });
});
