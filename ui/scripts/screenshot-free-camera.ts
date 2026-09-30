/** Dev utility: open the portal, switch the camera panel to Free, and save a screenshot.
 *  Usage: bun run scripts/screenshot-free-camera.ts [url] [out.png] */
import { chromium } from "playwright";

const url = process.argv[2] ?? "http://localhost:5173";
const out = process.argv[3] ?? "free-camera.png";
const browser = await chromium.launch();
const page = await browser.newPage({ viewport: { width: 1440, height: 900 } });
await page.goto(url);
await page.getByRole("button", { name: "Free" }).click({ timeout: 30_000 });
await page.getByTestId("free-camera-image").waitFor({ timeout: 60_000 });
await page.waitForTimeout(1500);
await page.screenshot({ path: out });
await browser.close();
console.log(`saved ${out}`);
