import { createRequire } from 'node:module';
import { readFile, writeFile } from 'node:fs/promises';
import { createHash } from 'node:crypto';
import assert from 'node:assert/strict';

// Supply an existing, local Playwright installation; no global install required.
const require = createRequire(process.env.ARCHIFY_PLAYWRIGHT_PACKAGE);
const { chromium } = require('playwright-core');
const root = new URL('./', import.meta.url);
const bytes = await readFile(new URL('index.html', root));
const browser = await chromium.launch({ executablePath: process.env.ARCHIFY_CHROME, headless: true });
const checks = [];
try {
  const page = await browser.newPage({ viewport: { width: 1440, height: 900 } });
  const errors = [];
  page.on('pageerror', error => errors.push(error.message));
  await page.goto(new URL('index.html', root).href);
  await page.evaluate(() => document.fonts.ready);
  assert.equal(await page.locator('html').getAttribute('lang'), 'zh-CN');
  assert.equal(await page.locator('svg g[data-node-id]').count(), 11);
  checks.push('zh-CN locale; 11 live SVG components');
  const theme = await page.locator('#btn-theme').innerText();
  await page.locator('#btn-theme').click();
  assert.notEqual(await page.locator('#btn-theme').innerText(), theme);
  await page.locator('#btn-theme').click();
  assert.equal(await page.locator('#btn-theme').innerText(), theme);
  checks.push('Theme toggles and restores');
  await page.locator('#node-tracker').click();
  assert.equal(await page.locator('#node-tracker').getAttribute('aria-pressed'), 'true');
  assert.match(await page.locator('#focus-label').innerText(), /跟踪目标/);
  await page.screenshot({ path: new URL('interaction-focus.png', root).pathname });
  await page.locator('#btn-focus-clear').click();
  assert.equal(await page.locator('#node-tracker').getAttribute('aria-pressed'), 'false');
  checks.push('Tracker focus opens and closes');
  await page.locator('#btn-node-finder').click();
  await page.locator('#node-finder-input').fill('跟踪');
  await page.screenshot({ path: new URL('interaction-search.png', root).pathname });
  await page.locator('#node-finder-input').press('Enter');
  assert.equal(await page.locator('#node-tracker').getAttribute('aria-pressed'), 'true');
  await page.locator('#btn-focus-clear').click();
  await page.keyboard.press('Escape');
  checks.push('Chinese search selects Tracker');
  const before = await page.locator('svg[role="img"]').getAttribute('data-view-scale');
  const viewBox = await page.locator('svg[role="img"]').getAttribute('viewBox');
  await page.getByRole('button', { name: '放大', exact: true }).click();
  await page.waitForTimeout(400);
  assert.notEqual(await page.locator('svg[role="img"]').getAttribute('data-view-scale'), before);
  assert.equal(await page.locator('svg[role="img"]').getAttribute('viewBox'), viewBox);
  checks.push('Zoom changes viewer scale while preserving authored viewBox');
  const responsive = [];
  for (const width of [375, 768, 1280]) {
    await page.setViewportSize({ width, height: 900 });
    await page.reload();
    await page.evaluate(() => document.fonts.ready);
    await page.waitForTimeout(1500);
    responsive.push(await page.evaluate(() => ({ width: innerWidth, scrollWidth: document.documentElement.scrollWidth })));
    await page.screenshot({ path: new URL(`responsive-${width}.png`, root).pathname, fullPage: true });
  }
  assert(responsive.every(item => item.scrollWidth <= item.width));
  assert.deepEqual(errors, []);
  const receipt = { status: 'pass', artifactSha256: createHash('sha256').update(bytes).digest('hex'), artifactBytes: bytes.length, checks, responsive, pageErrors: errors };
  await writeFile(new URL('browser-check.json', root), JSON.stringify(receipt, null, 2) + '\n');
  console.log(JSON.stringify(receipt, null, 2));
} finally {
  await browser.close();
}
