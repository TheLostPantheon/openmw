# Frame pipeline redesign: cull in parallel with simulation

Status: proposal (2026-10-05). Measurements from device runs on build
`d52b674b4b` (walking in Balmora, DXT kept compressed, cores pinned).

## Summary

The Vita frame is one dependency chain that the current pipeline runs almost
entirely in series. Cull (≈10 ms) and simulation (≈10 ms) run back to back on
the sim worker, and the main thread waits on both every frame. Running cull on
the GL thread **in parallel with** simulation removes about one cull's worth
of time from every frame:

| | Today | After redesign | With planned work cuts too |
|---|---|---|---|
| Town frame (median) | ~40 ms (25 fps) | **~30 ms (33 fps)** | ~20–22 ms (45–50 fps) |

The redesign alone is a ~1.3× gain, not 2×: main's own work (~18 ms) depends
on the simulation results, so it cannot overlap them. The larger total comes
from combining it with the work reductions in "Independent work cuts" below,
which are lower risk and can land first.

## Measured baseline (per frame, medians)

| Thread | Work | ms |
|---|---|---|
| main | world update 11.1 (streaming hydrate 8.5), Lua 3.2, Lua sync 2.9, GUI/input/sound ~1.3 | ~18 |
| main | **waiting on the sim worker** | **22–23** |
| sim worker (the job main waits on) | update traversal 1.6, **cull 10.2**, scripts 2.6–8.0, mechanics 1.6, physics 0.3 | ~21 |
| GL worker | draw 6.6–11.7, swap 0.2 | ~7–12 |
| GPU | display queue depth 1.0, swap blocked 0.0–0.1 ms | not a limit |

Cull by subtree ([CullTree]): sky/other 3.2, statics 2.3, terrain/water/fx 2.2,
actors 0.6 ms.

## Current pipeline

```
main  |=== main work N+1 (18) ===|kick|.... wait ....................|=== main N+2
sim   |                          |upd N|== cull N (10) ==|= sim N+1 (11) =|
GL    |                          |== draw N-1 (7) ==|
                                  <------------- job ≈ 21 ms -------------->
frame ≈ main (18) + upd (1.6) + cull (10) + sim (11) ≈ 40 ms
```

The sim job is kicked at the end of main's frame, and main's next frame starts
by joining it (`engine.cpp`, "simjoin"): every frame pays main + cull + sim.

## Proposed pipeline

```
main  |=== main work N+1 (18) ===|kick|... wait ...|=== main N+2
sim   |                          |upd N|= sim N+1 (11) =|
GL    |                          |     |= cull N (10) =|= draw N (7) =|
                                  <-- ≈ 13 ms -->
frame ≈ main (18) + upd (1.6) + max(cull, sim) (11) ≈ 30 ms
```

1. **Sim worker job:** non-hazard update traversal N, then signal "update N
   done", then simulation N+1 (scripts, mechanics, physics).
2. **GL worker job:** wait for "update N done", cull N, draw N, swap. Cull
   moves off the sim worker.
3. **Main:** at frame start, join the sim job **and** a "cull N done" fence
   before any phase that touches the scene graph (input actions, Lua sync,
   world update, GUI, hazard update traversal). Main never mutates the scene
   while cull runs, as today.
4. **Setting** `vita cull on gl thread` (default off at first) selects the old
   or new pipeline, so we can A/B and fall back without a rebuild.

Draw keeps overlapping main exactly as today, so nothing new there.

### Why not overlap more?

- **Main's work with the sim job:** main's world update consumes the
  simulation results (positions, cell changes), so they stay in series.
- **Game logic one frame ahead of rendering** (deeper pipelining): main and sim
  would mutate the same game state at once (cell stores, scripts iterating
  cells). Not feasible without a much larger rewrite.
- **Cull with main's world update (full copy-on-write of the scene):** gains
  nothing in the common case, because main is waiting for the sim job anyway
  and cull ≤ sim. The copy-on-write work below makes it possible later.

## Safety: what runs concurrently with cull N

Only **simulation N+1** on the sim worker (plus draw N−1 finishing, as today).
Every sim-thread write that cull can observe:

| Hazard | Source on the sim thread | Handling |
|---|---|---|
| **Structural changes** (add/remove children) | scripts (Enable/Disable/PlaceItem/PlaceAtPC), spell effects attaching to actors, projectiles, summons, equipment rebuilding NPC parts | **Copy-on-write child lists in OSG** (below). Cull iterates the list it started with; the change is immediate for the thread that made it. No call-site changes. |
| Removed nodes freed during cull | same | The copy-on-write snapshot holds references, so nodes live until cull drops the snapshot. |
| Transform writes (`setPosition`/`setAttitude`) | physics (`updatePhysics`), mechanics | **Accepted:** a moving object may render one frame with a half-updated matrix. Monitor; if visible, apply physics transforms on main after the join. |
| Bounding-sphere recompute race | dirtying from moves while cull calls `getBound()` | **Accepted:** at worst a one-frame cull error on a moving object; it self-heals on the next move. |
| Node masks (visibility) | mechanics | Single word writes; benign. |
| Light lists | update traversal | Already double-buffered by frame (`LightManager`, `frame % 2`). |
| Animation state maps | mechanics start/stop animations | The update traversal that reads them stays **before** simulation, in series on the sim worker. |
| StateSet edits (fades, glow) | mechanics/effects | Cull only reads render-bin details; accepted risk, monitored. |

## Copy-on-write child lists (OSG patch)

- `Group` stores its children as an immutable, reference-counted list. A
  traversal takes a reference to the current list and iterates it. Mutators
  (add, insert, remove, replace, set) copy the list, modify the copy, and swap
  it in under a writer mutex. The public API is unchanged.
- The same applies to subclasses with parallel per-child arrays (`Switch`
  values, `LOD`/`PagedLOD` ranges, `Sequence` timings, `Billboard`
  positions), which swap together with the child list.
- Cost: one atomic reference-count pair per group traversal (≈1–2k per frame),
  and one copy per structural change (a few per frame while streaming). Both
  small next to the 10 ms saved.
- A debug mode logs structural changes made during cull from any thread other
  than main, to confirm coverage on device before the setting defaults on.

## Independent work cuts (lower risk, can land first)

| Target | Now | Approach |
|---|---|---|
| Streaming on main (hydrate) | 8.5 ms/frame while walking in town | The frame-time budget is overrun; check how adds are charged against it and cap the scans. |
| Sky/other cull | 3.2 ms | Split by child to find whether it's the sky's node structure or a render-to-texture camera (local map) re-culling the scene. |
| Scripts (sim) | 2.6–8.0 ms | Profile MW script cost in Balmora; per-actor throttling at distance if semantics allow. |
| Lua + Lua sync (main) | ~6 ms | Builtin actor scripts per actor; reduce frequency for distant actors. |
| Cull replay for merged statics | never fires (`crep=0`) | Find why the existing feature doesn't engage. |

After these cuts, cull and simulation both shrink, so the redesign saves
min(cull, sim) per frame rather than a fixed 10 ms. The two efforts add up but
are not fully independent.

## Plan

1. **Work cuts** (above), one measured device run each. Low risk.
2. **Copy-on-write child lists** in the OSG patch, with the debug coverage
   mode. About a day.
3. **Pipeline change** behind the setting: split the sim job, move cull into
   the GL job, add the main fence and fence-wait telemetry. About a day.
4. **Soak and A/B** on the fixed route (Balmora main street, then wilderness),
   using `vita_session.py` and the crash tooling. Default the setting on once
   it holds over several long sessions.

Rollback at every step is the setting, or the previous build.
