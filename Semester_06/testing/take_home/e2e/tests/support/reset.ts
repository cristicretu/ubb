import type { APIRequestContext } from "@playwright/test";

/**
 * Re-seeds the backend with deterministic test data before each test so that
 * specs do not contaminate each other. Mirrors the @Before-style setup used in
 * the Lab 5 Serenity tests.
 */
export async function resetData(request: APIRequestContext): Promise<void> {
  const response = await request.post("http://localhost:2070/api/reset");
  if (!response.ok()) {
    throw new Error(`failed to reset backend: ${response.status()}`);
  }
}
