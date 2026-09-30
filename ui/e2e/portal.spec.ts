import { expect, test } from "@playwright/test";

const PANELS = ["Goals Queue", "Camera View", "Top View", "Events Log"];

test.describe("portal shell @smoke", () => {
  test("renders the four panels in dark mode", async ({ page }) => {
    await page.goto("/");
    await expect(page).toHaveTitle(/GPTPet Management Portal/);
    await expect(page.getByRole("heading", { name: "GPTPet Management Portal" })).toBeVisible();
    for (const name of PANELS) {
      await expect(page.getByRole("heading", { name })).toBeVisible();
    }
    // The theme uses oklch colours; let the browser resolve the body background to sRGB.
    const luminance = await page.evaluate(() => {
      const context = document.createElement("canvas").getContext("2d");
      if (!context) return 1;
      context.fillStyle = getComputedStyle(document.body).backgroundColor;
      context.fillRect(0, 0, 1, 1);
      const [r = 255, g = 255, b = 255] = context.getImageData(0, 0, 1, 1).data;
      return (0.2126 * r + 0.7152 * g + 0.0722 * b) / 255;
    });
    expect(luminance).toBeLessThan(0.25);
    await expect(page.locator("html")).toHaveClass(/dark/);
  });

  test("reports the pet offline and disables the goal form without a server", async ({ page }) => {
    test.skip(process.env.PORTAL_LIVE === "1", "the pet server is running in the live tier");
    await page.goto("/");
    await expect(page.getByTestId("connection-state")).toHaveText(/offline|connecting/i);
    await expect(page.getByRole("button", { name: "Submit" })).toBeDisabled();
  });

  test("opens the settings sheet from the menu", async ({ page }) => {
    await page.goto("/");
    await page.getByRole("button", { name: "Open menu" }).click();
    await expect(page.getByRole("heading", { name: "Settings", exact: true })).toBeVisible();
  });
});

test.describe("portal live @live", () => {
  test.skip(process.env.PORTAL_LIVE !== "1", "needs `gpt-pet serve` and the simulator (PORTAL_LIVE=1)");

  test("shows the pet's goal, camera frame, and expandable tool calls", async ({ page }) => {
    await page.goto("/");
    await expect(page.getByTestId("connection-state")).toHaveText(/live/i, { timeout: 60_000 });
    await expect(page.getByTestId("goal-active")).toHaveCount(1, { timeout: 150_000 });
    const camera = page.getByTestId("camera-image");
    await expect
      .poll(async () => camera.evaluate((img: HTMLImageElement) => img.naturalWidth), { timeout: 150_000 })
      .toBeGreaterThan(0);
    const row = page.getByTestId("event-row").filter({ hasText: "get_current_view" }).first();
    await row.click();
    await expect(page.getByTestId("event-detail")).toContainText("get_current_view");
    await page.getByLabel("Custom goal").fill("find the owner and say hi");
    await page.getByRole("button", { name: "Submit" }).click();
    await expect(page.getByTestId("goal-pending").filter({ hasText: "find the owner" })).toBeVisible();

    const budget = page.getByTestId("goal-budget");
    await expect(budget).toHaveText(/goals \d+\/\d+/);
    const before = Number((await budget.textContent())?.match(/\/(\d+)/)?.[1]);
    await page.getByRole("button", { name: "Add 2 more goals" }).click();
    await expect(budget).toHaveText(new RegExp(`/${before + 2}$`));
  });

  test("flies the simulator's free camera", async ({ page }) => {
    await page.goto("/");
    await expect(page.getByTestId("connection-state")).toHaveText(/live/i, { timeout: 60_000 });
    await page.getByRole("button", { name: "Free" }).click();
    const image = page.getByTestId("free-camera-image");
    await expect
      .poll(async () => image.evaluate((img: HTMLImageElement) => img.naturalWidth).catch(() => 0), {
        timeout: 120_000,
      })
      .toBeGreaterThan(0);
    const view = page.getByTestId("free-camera");
    const before = Number(await view.getAttribute("data-version"));
    await page.getByRole("button", { name: "Look left" }).click();
    await expect
      .poll(async () => Number(await view.getAttribute("data-version")), { timeout: 30_000 })
      .toBeGreaterThan(before);
    await page.getByRole("button", { name: "top" }).click();
    await expect(view).toContainText(/pitch 89°/);
    await page.getByRole("button", { name: "Robot" }).click();
    await expect(page.getByTestId("camera-image")).toBeVisible();
  });
});
