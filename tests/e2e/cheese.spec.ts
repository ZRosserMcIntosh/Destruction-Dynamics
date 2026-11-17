import { expect, test } from '@playwright/test';

// Viewer request, Day 1: "A giant block of cheese in the center forever (make it indestructible pls)".
test('cheese scenario: the tower breaks over an indestructible block', async ({ page }) => {
  const errors: string[] = [];
  page.on('pageerror', (e) => errors.push(e.message));
  page.on('console', (m) => {
    if (m.type() === 'error') errors.push(m.text());
  });
  await page.goto('/?scenario=cheese');
  const state = () =>
    page.evaluate(() => (window as unknown as { __DD: { state(): Record<string, unknown> } }).__DD.state());
  await page.waitForFunction(
    () => ((window as unknown as { __DD?: { frames: number } }).__DD?.frames ?? 0) > 5,
    null,
    {
      timeout: 120_000,
    },
  );
  await expect(page.locator('#scenario')).toHaveValue('cheese');
  await expect(page.locator('.request')).toContainText('cheese');
  // Play stays disabled until the floors are solved, then the event runs.
  await expect(page.locator('#btn-trigger')).toBeEnabled({ timeout: 60_000 });
  const s0 = (await state()) as { variant: string; releaseTime: number; layers: { attached: boolean } };
  expect(s0.variant).toBe('cheese');
  expect(s0.layers.attached).toBe(true);
  await page.selectOption('#speed', '8');
  await page.click('#btn-trigger');
  await page.waitForFunction(
    (t) => (window as unknown as { __DD: { state(): { time: number } } }).__DD.state().time > t + 1.5,
    s0.releaseTime,
    { timeout: 120_000, polling: 250 },
  );
  await page.screenshot({ path: 'test-results/screenshots/cheese-impact.png' });
  await page.waitForFunction(
    () => (window as unknown as { __DD: { state(): { state: string } } }).__DD.state().state === 'AFTERMATH',
    null,
    { timeout: 180_000, polling: 250 },
  );
  await page.screenshot({ path: 'test-results/screenshots/cheese-aftermath.png' });
  expect(errors).toEqual([]);
});
