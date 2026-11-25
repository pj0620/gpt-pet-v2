import { defineConfig, devices } from "@playwright/test";

/**
 * Two tiers, no mocks:
 * - default (@smoke): Vite dev server only; the pet is offline and the portal must say so.
 * - PORTAL_LIVE=1 (@live): also starts `gpt-pet serve` (real simulator, real Gemini).
 * PORTAL_URL=http://localhost:8080 targets the production build served by the pet server.
 */
const live = process.env.PORTAL_LIVE === "1";

export default defineConfig({
  testDir: "./e2e",
  timeout: live ? 300_000 : 60_000,
  expect: { timeout: live ? 30_000 : 5_000 },
  workers: 1,
  fullyParallel: false,
  retries: 0,
  reporter: "list",
  grep: live ? undefined : /@smoke/,
  use: {
    baseURL: process.env.PORTAL_URL ?? "http://localhost:5173",
    trace: "retain-on-failure",
  },
  projects: [{ name: "chromium", use: { ...devices["Desktop Chrome"] } }],
  webServer: [
    ...(live
      ? [
          {
            command: "uv run gpt-pet serve --profile sim --port 8080",
            cwd: "../python",
            url: "http://localhost:8080/api/status",
            timeout: 240_000,
            reuseExistingServer: true,
            stdout: "pipe" as const,
            stderr: "pipe" as const,
          },
        ]
      : []),
    {
      command: "bun run dev",
      url: "http://localhost:5173",
      timeout: 60_000,
      reuseExistingServer: true,
    },
  ],
});
