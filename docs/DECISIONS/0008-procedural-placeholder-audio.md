# ADR 0008: Procedural placeholder audio behind `AudioEngine`

Status: accepted (Milestone 1)

## Context

Sound is critical to the spectacle, but the repository must not contain
unlicensed recordings, and the final game will probably use audio
middleware.

## Decision

- Director cues on the `audio` channel go through `AudioCueRouter` to an
  `AudioEngine`. Physics and destruction code never play sound.
- `ProceduralAudioEngine` synthesises every layer at runtime (groan, metal,
  glass, rumble, water impact, echoes, bubbles, cable snap and hum) with Web
  Audio. It uses HRTF panning, inverse-distance roll-off, propagation delay
  at the speed of sound and an underwater low-pass. It is labelled a
  placeholder.

## Consequences

- No asset licensing questions, and no download size for sound.
- The sound is synthetic and lower quality than recordings. Replacing it
  means implementing `AudioEngine` with licensed samples or middleware, with
  no change to cue routing.
- Automated tests check routing only. Nobody has listened to the result
  under test.
