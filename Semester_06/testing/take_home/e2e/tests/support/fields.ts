import type { Locator } from "@playwright/test";

/**
 * Returns the first input/select/textarea inside the same field wrapper as a
 * <label> whose visible text equals `label` (case-sensitive, exact match).
 *
 * The app's pages render fields as `<div><label>Name</label><input/></div>` —
 * without `for`/`id` associations — so Playwright's `getByLabel` cannot find
 * them. This helper bridges that gap with a deterministic CSS query.
 */
export function fieldByLabel(scope: Locator, label: string): Locator {
  const escaped = label.replace(/"/g, '\\"');
  return scope
    .locator(`div:has(> label:text-is("${escaped}"))`)
    .locator("input, select, textarea")
    .first();
}
