import { type Page, expect, test } from '@playwright/test';

// The Impact Lab: a crash-test car into breakable walls, and a projectile
// gallery. SwiftShader is slow, so these run at a small viewport in real time.
const LAB = 'http://localhost:4174/';

interface ImpactState {
  frames: number;
  time: number;
  loose: number;
  jointsBroken: number;
  car: { impact: boolean; front: number; crush: number; peakG: number } | null;
}
type Hooks = {
  __IMPACT: {
    state(): ImpactState;
    crash(wall: string, kmh: number): Promise<void>;
    fire(p: string, v: number): void;
    slowmo(v: string): void;
  };
};
const state = (page: Page) => page.evaluate(() => (window as unknown as Hooks).__IMPACT.state());

test.use({ viewport: { width: 640, height: 360 } });

test.describe('impact lab', () => {
  const errors: string[] = [];
  test.beforeEach(async ({ page }) => {
    errors.length = 0;
    page.on('pageerror', (e) => errors.push(e.message));
    page.on('console', (m) => {
      if (m.type() === 'error') errors.push(m.text());
    });
  });

  test('a car at 100 km/h goes through a brick wall', async ({ page }) => {
    await page.goto(LAB);
    await page.waitForFunction(
      () => ((window as unknown as Partial<Hooks>).__IMPACT?.state().frames ?? 0) > 3,
      null,
      {
        timeout: 120_000,
      },
    );
    await page.evaluate(async () => {
      const w = window as unknown as Hooks;
      w.__IMPACT.slowmo('1');
      await w.__IMPACT.crash('brick', 100);
    });
    await page.waitForFunction(() => (window as unknown as Hooks).__IMPACT.state().car?.impact, null, {
      timeout: 120_000,
      polling: 250,
    });
    const t = (await state(page)).time;
    await page.waitForFunction((t0) => (window as unknown as Hooks).__IMPACT.state().time > t0 + 1.5, t, {
      timeout: 170_000,
      polling: 250,
    });
    await page.screenshot({ path: 'test-results/screenshots/impact-crash.png' });
    const s = await state(page);
    expect(s.car!.front).toBeGreaterThan(0.5);
    expect(s.car!.crush).toBeGreaterThan(0.3);
    expect(s.loose).toBeGreaterThan(50);
    expect(errors).toEqual([]);
  });

  test('the gallery breaks window glass with a bowling ball', async ({ page }) => {
    await page.goto(`${LAB}?mode=gallery&target=glass`);
    await page.waitForFunction(
      () => ((window as unknown as Partial<Hooks>).__IMPACT?.state().frames ?? 0) > 3,
      null,
      {
        timeout: 120_000,
      },
    );
    await expect(page.locator('#gallery-controls')).toBeVisible();
    await expect(page.locator('#crash-controls')).toBeHidden();
    await page.evaluate(() => {
      const w = window as unknown as Hooks;
      w.__IMPACT.slowmo('1');
      w.__IMPACT.fire('bowling', 30);
    });
    await page.waitForFunction(() => (window as unknown as Hooks).__IMPACT.state().loose > 3, null, {
      timeout: 120_000,
      polling: 250,
    });
    await page.screenshot({ path: 'test-results/screenshots/impact-gallery.png' });
    expect(errors).toEqual([]);
  });
});
