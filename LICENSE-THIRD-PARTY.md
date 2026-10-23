# Third-party notices

Destruction Dynamics' original code is MIT licensed (see `LICENSE`). It depends on:

| Package | License | Used for |
|---|---|---|
| [three](https://github.com/mrdoob/three.js) | MIT | Rendering in `@destruction-dynamics/rendering` and the playground |
| [@dimforge/rapier3d-compat](https://github.com/dimforge/rapier.js) | Apache-2.0 | Rigid-body physics backend (`RapierPhysicsAdapter`) |

Development-only tools (TypeScript, Vite, Vitest, Playwright, ESLint, Prettier) are not shipped.

## Assets

The repository contains **no** third-party art, audio or game content. All geometry is generated
procedurally at runtime; all sound is synthesised at runtime by `ProceduralAudioEngine` and is a
labelled placeholder for licensed recordings. Real places (Balneário Camboriú, São Paulo) only
inspire fictional layouts; no building is modelled on a real structure.
