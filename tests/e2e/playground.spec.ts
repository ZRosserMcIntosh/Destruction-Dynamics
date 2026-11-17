import { type Page, expect, test } from '@playwright/test';

// Drives the real UI in Chromium (SwiftShader in CI, so frame rates are low and
// the event runs at 8× time scale). Reads state through window.__DD.

type DDState = {
  state: string;
  tiltDeg: number;
  particles: number;
  debris: { active: number; spawned: number; limit: number };
  fluid: { grid: number; peak: number; maxDisturbance: number };
  collisionMode: string;
  layers: { count: number; status: string; attached: boolean };
  quality: {
    id: string;
    fluidGridSize: number;
    maxParticles: number;
    maxDebrisBodies: number;
    shadows: boolean;
  };
  render: {
    calls: number;
    triangles: number;
    pixelRatio: number;
    shadows: boolean;
    oceanGrid: number;
    panelsDetached: number;
  };
  camera: { mode: string; position: number[]; underwater: boolean; depth: number };
  frames: number;
};

const state = (page: Page) =>
  page.evaluate(() => (window as unknown as { __DD: { state(): DDState } }).__DD.state());
const waitFor = (page: Page, fn: string, timeout = 120_000) =>
  page.waitForFunction(`(() => { const s = window.__DD.state(); return ${fn}; })()`, null, {
    timeout,
    polling: 250,
  });

test.describe.serial('playground: The Deep End', () => {
  let page: Page;
  const pageErrors: string[] = [];
  const consoleErrors: string[] = [];

  test.beforeAll(async ({ browser }) => {
    page = await browser.newPage();
    page.on('pageerror', (e) => pageErrors.push(e.message));
    page.on('console', (m) => {
      if (m.type() === 'error') consoleErrors.push(m.text());
    });
    await page.goto('/');
  });

  test.afterAll(async () => {
    await page.close();
  });

  test('1. the scene loads and renders', async () => {
    await page.waitForFunction(
      () => ((window as unknown as { __DD?: { frames: number } }).__DD?.frames ?? 0) > 5,
      null,
      {
        timeout: 120_000,
      },
    );
    const s = await state(page);
    expect(s.state).toBe('INTACT');
    expect(s.render.calls).toBeGreaterThan(10);
    expect(s.render.triangles).toBeGreaterThan(50_000);
    await page.screenshot({ path: 'test-results/screenshots/01-loaded.png' });
  });

  test('2. the camera moves', async () => {
    await page.selectOption('#camera', 'free');
    const before = (await state(page)).camera.position;
    const canvas = page.locator('#viewport canvas');
    const box = (await canvas.boundingBox())!;
    await page.mouse.move(box.x + box.width * 0.3, box.y + box.height * 0.5);
    await page.mouse.down();
    await page.mouse.move(box.x + box.width * 0.55, box.y + box.height * 0.4, { steps: 8 });
    await page.mouse.up();
    await page.mouse.wheel(0, -600);
    await page.waitForTimeout(1500);
    const after = (await state(page)).camera.position;
    const moved = Math.hypot(after[0]! - before[0]!, after[1]! - before[1]!, after[2]! - before[2]!);
    expect(moved).toBeGreaterThan(1);
    await page.selectOption('#camera', 'cinematic');
  });

  test('3. the event starts', async () => {
    await page.selectOption('#speed', '8');
    await page.click('#btn-trigger');
    await waitFor(page, "s.state !== 'INTACT'", 30_000);
    expect((await state(page)).state).not.toBe('INTACT');
  });

  test('4. the tower visually collapses', async () => {
    await waitFor(page, 's.tiltDeg > 45');
    await page.screenshot({ path: 'test-results/screenshots/04-collapsing.png' });
    const s = await state(page);
    expect(s.render.panelsDetached).toBeGreaterThan(50);
    expect(s.debris.spawned).toBeGreaterThan(10);
    expect(['kinematic', 'wreck']).toContain(s.collisionMode);
  });

  test('5. the water responds', async () => {
    await waitFor(page, 's.fluid.peak > 2');
    await page.screenshot({ path: 'test-results/screenshots/05-impact.png' });
    const s = await state(page);
    expect(s.fluid.peak).toBeGreaterThan(2);
  });

  test('6. the event reaches aftermath', async () => {
    await waitFor(page, "s.state === 'AFTERMATH'", 180_000);
    const s = await state(page);
    expect(s.collisionMode).toBe('wreck');
    expect(s.tiltDeg).toBeGreaterThan(80);
    await page.screenshot({ path: 'test-results/screenshots/06-aftermath.png' });
  });

  test('8. the underwater camera functions', async () => {
    await page.selectOption('#camera', 'underwater');
    await waitFor(page, 's.camera.underwater === true', 30_000);
    const s = await state(page);
    expect(s.camera.mode).toBe('underwater');
    expect(s.camera.depth).toBeGreaterThan(0);
    // Swim forward and confirm the diver camera moves. Software rendering runs at
    // a few FPS with a clamped frame step, so wait on movement, not on a duration.
    const [bx, , bz] = s.camera.position;
    await page.keyboard.down('w');
    await waitFor(page, `Math.hypot(s.camera.position[0] - ${bx}, s.camera.position[2] - ${bz}) > 1`, 30_000);
    await page.keyboard.up('w');
    await page.screenshot({ path: 'test-results/screenshots/08-underwater.png' });
    await page.selectOption('#camera', 'cinematic');
  });

  test('7. the scene resets', async () => {
    await page.click('#btn-reset');
    await waitFor(page, "s.state === 'INTACT' && s.tiltDeg === 0 && s.debris.active === 0", 30_000);
    const s = await state(page);
    expect(s.render.panelsDetached).toBe(0);
    expect(s.collisionMode).toBe('intact');
  });

  test('9. quality presets change actual settings', async () => {
    const balanced = await state(page);
    await page.selectOption('#quality', 'dorito');
    await waitFor(page, "s.quality.id === 'dorito'", 30_000);
    const dorito = await state(page);
    expect(dorito.fluid.grid).toBe(96);
    expect(dorito.render.oceanGrid).toBe(48);
    expect(dorito.render.shadows).toBe(false);
    expect(dorito.render.pixelRatio).toBeLessThan(balanced.render.pixelRatio);
    expect(dorito.debris.limit).toBe(40);
    expect(dorito.layers).toMatchObject({ count: 5, status: 'authored' });
    await page.selectOption('#quality', 'high');
    await waitFor(page, "s.quality.id === 'high'", 30_000);
    const high = await state(page);
    expect(high.fluid.grid).toBe(256);
    expect(high.render.shadows).toBe(true);
    expect(high.render.oceanGrid).toBeGreaterThan(balanced.render.oceanGrid);
    expect(high.quality.maxParticles).toBeGreaterThan(balanced.quality.maxParticles);
    // High splits the tower into 26 layers, solved in a Web Worker.
    expect(high.layers.count).toBe(26);
    await waitFor(page, 's.layers.attached === true', 60_000);
  });

  test('10. no critical runtime errors occurred', async () => {
    const hookErrors = await page.evaluate(
      () => (window as unknown as { __DD: { errors: string[] } }).__DD.errors,
    );
    expect(hookErrors).toEqual([]);
    expect(pageErrors).toEqual([]);
    expect(consoleErrors).toEqual([]);
  });
});
