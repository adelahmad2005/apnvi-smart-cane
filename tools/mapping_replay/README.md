# Lap replay: tune the map settings on a recorded lap (15 Nov)

Why: on the device the LiDAR sits at desk height (77 cm) and the map drifts (a lap of ESB117 ended
2 to 3.8 m and 20 to 35 degrees off). Instead of pushing a lap for every setting, one lap was
recorded and is replayed on the Jetson with different settings.

The recorded lap: `~/lap_bags/lap_134450` on the Jetson (15 Nov, 306 s, `/scan_raw` and
`/tf_static`). It starts and ends on the START tape, facing the same way, so the right answer at the
end is "0 m and 0 degrees from the start".

Copy the files to `/tmp` on the Jetson first (`sed -i 's/\r$//'` on each). Runs can go side by side
with different domain numbers (each takes the length of the lap, about 5 minutes). Three at once on
battery power took the Jetson offline once (15 Nov): run two at most.

| File | What it does |
|---|---|
| `bag_start.sh` | Start recording a lap (`/scan_raw`, `/tf_static`, and since 16 Nov the tracker's `/odom` and `/tf`) into `~/lap_bags/` |
| `collect_drift_data.sh` | 16 Nov, for the cloud fixing chat: packs the recorded laps and runs seven tracker replays (today's settings and six one-change variants from `cfgs.py`) into `~/drift_export/`. See `../../results/map-drift/drift-data-request-16nov.md` |
| `cfgs.py` | Writes the movement-tracker (icp_odometry) settings to try: `/tmp/cfg_<name>.yaml` |
| `replay.sh NAME DOMAIN on|off CFG` | Replays the lap through the scan filter (on/off) and icp_odometry; prints the end error |
| `odom_eval.py` | Used by `replay.sh`: end error, path length, a track every 5 s |
| `track_replay.sh CFG DOMAIN` | 16 Nov: replays a recording through the scan filter and icp_odometry and keeps the whole path (`BAG=` picks the recording) |
| `track_rec.py` | Used by `track_replay.sh`: saves `track.npz` (`poses`: t, x, y, yaw; `info`: the tracker's own account of each sweep: match score, memory points, added or not, seconds of work, matched points, lost) |
| `runpair.sh A [B]` | 16 Nov: runs two named settings from `cfgs.py` side by side (domains 61 and 62) into `~/drift_exp/track_<name>.npz`. Never start a second one while one runs |
| `queue.sh "A,B" "C,D" ...` | Runs `runpair.sh` pairs one after the other |
| `rcfgs.py` | Writes the map (rtabmap) settings to try: `/tmp/rcfg_<name>.yaml` |
| `rtab_replay.sh NAME DOMAIN RCFG` | Replays the lap through filter, icp_odometry and rtabmap; prints the map's end error and the loop links |
| `rtab_eval.py` | Used by `rtab_replay.sh`: the map's end position (joins map->odom and odom->base_link) |
| `db_links.py` | Counts the links in a map file: steps, loop closures, "near-space" corrections |
| `db_odom_end.py` | 16 Nov: where the movement tracker alone ended, read from a map file (to see what the map added) |
| `offline/` | 16 Nov, for looking at a recorded lap away from the Jetson (plain Python, no ROS): `bagread.py` reads the sweeps from a bag folder, `lidar_odom.py` is a movement tracker built to RTAB-Map's recipe plus a sturdier one as yardstick, `sim.py` and `run_sim.py` try both on a made-up lab |
| `pos.py` | Prints where the device is now (odom and map), for the push test. Needs the device running |
| `scan_flicker.py` | Reads the lap itself (no replay): flicker (beams jumping away and back), the direction each sweep pins down worst, things carried along with the device. Read only, about 30 s |

**16 Nov: the cause and the fix** (all in `../../results/map-drift/drift-findings-16nov.md`). The
tracker added every sweep to its memory, so it remembered only the last second. Fix in
`stage3.launch.py`: `Odom/ScanKeyFrameThr` 0.45 and `OdomF2M/ScanMaxSize` 10000 (first 20000; 10000 is as accurate at half the time per sweep).

| Settings (16 Nov, tracker with the fix unless said) | End error | Notes |
|---|---|---|
| Tracker, today's settings (`now`) | 2.24 m, 35 deg | Before the fix |
| Tracker, the fix (`kf045_big`, `kf045_big_r2`, `fix`, `fix_r2`, `fix` alone) | 0.48 m / 1.0 deg; 1.17 / 3.5; 1.19 / 3.4; 1.19 / 3.4; 1.21 / 4.9 | 35 to 42 ms per sweep, no lost sweeps. About 0.5 m when the tracker starts on the lap's first sweep, about 1.2 m when it starts 0.2 to 0.4 s later (findings, section 2) |
| Tracker, memory 10000 (`kf045_10k`, `fix_10k_r2` alone) | 1.21 m / 4.2 deg; 0.52 / 2.0 | About 21 ms per sweep: the fallback if the tracker can't keep up live |
| Push recording, `now` against the fix | total turn +2.0 against +1.1 deg (walls: +1.6) | No worse |
| Fix + map, `r_now` | 0.48 m, 1 deg, 0 corrections | Tracker alone ended 0.44 m, 1.4 deg |
| Fix + map, `r_02_loop` (twice) | 1.13 m, 3 deg, 22 corrections | Worse than the tracker alone |
| Fix + map, `r_02_pfr3_mcd06` | 0.48 m, 0 deg, 18 corrections | No better than `r_now`: map settings unchanged |

Results so far (15 Nov):

| Settings | End error | Notes |
|---|---|---|
| Movement tracker, today's settings, no filter | 2.32 m, 32 deg | |
| Movement tracker, today's settings, with the rear filter | 2.01 m, 22 deg | The filter helps a little |
| Movement tracker, point-to-plane, bigger memory | 3.31 m, 32 deg | Worse |
| Map, today's `Icp/CorrespondenceRatio` 0.5 | 0 corrections | RTAB-Map doubles it for "been here before" checks: 100%, so never |
| Map, ratio 0.35 or 0.25 | 0 corrections | Doubled: 70%, 50% |
| Map, ratio 0.2 + loop settings (`rcfgs.py` LOOP), `r_02_loop` | **1.79 m, 9 deg**, 24 corrections | Heading much better; position barely. The end never snaps to START |
| Map, ratio 0.15 + loop settings, `r_015_loop` | 2.12 m, 22 deg, 14 corrections | No better than no map |
| Map, today's settings, `r_now` | 2.01 m, 22 deg, 0 corrections | Same as the movement tracker alone: today the map adds nothing |
| `r_02_loop` + match distance 1.0 m, 30 rounds, `r_02_mcd1` | 3.12 m, 34 deg, 0 corrections | Worse. The wide distance also spoils the step-to-step refining |

Why the end doesn't snap (15 Nov, second chat): the lap has no camera images, so the only correction is
"near-space" (proximity). RTAB-Map only tries an old place if its current guess is within
`RGBD/ProximityPathFilteringRadius` of it (default 1 m), and ICP only matches points within
`Icp/MaxCorrespondenceDistance` (0.3 m). With the guess ~1.8 m off at the end, START is never tried.
libpointmatcher is already in use (`Icp/Strategy` 1 is the default on the Jetson).

| `r_02_loop` + search radius 3 m, `r_02_pfr3` | 2.22 m, 21 deg, 0 corrections | Lost all corrections: odd (see below) |
| `r_02_pfr3` + match distance 0.6 m, step refining off, `r_02_pfr3_mcd06` | **1.54 m, 9 deg**, 16 corrections | Best so far; still far from the target (0.3 m, 5 deg) |

What the numbers say (15 Nov):
- When the heading is right (9 deg), the error is almost all along one line: the map ends 1.5 to 1.8 m
  short in x, with y near 0. That looks like the sweeps sliding along the lane between the desks: at
  desk height the LiDAR sees long straight edges that don't fix the position along them.
- Repeat check (15 Nov): `r_02_loop` and `r_02_pfr3_mcd06` run a second time (side by side again)
  gave exactly the same numbers and corrections. **The replays repeat exactly**, so every difference in
  the table is real, including `r_02_pfr3` losing its corrections.
- Even the best setting never snaps the end to START. The corrections fix the heading (22 to 9 deg)
  but not the 1.5 to 1.8 m along the lane: matching sweeps against old sweeps has the same blind spot
  along the desk edges as the movement tracker.

- Encoders: none on the motors (15 Nov), so the LiDAR is the only sense of movement.

## What the lap's sweeps show (`scan_flicker.py`, 15 Nov)

| Check | Result | Meaning |
|---|---|---|
| Flicker: a beam jumps > 0.5 m and comes back within 0.3 s | 0.6% of points per sweep while moving, 0.3% standing still. Jumps are mostly 3 m to 5 m away, front a bit more than its share | **Low.** No sign of desk edges flickering in and out at close range |
| How well a sweep pins the position (weak/strong ratio, 1 = equal both ways) | Median 0.63; badly pinned (< 0.15) in only 2% of sweeps. Weakest stretches: 150-160 s, 180-190 s, 240-280 s (0.21 to 0.38) | Most sweeps have plenty of shape |
| Which way is weakest | Along forward/back in 71% of sweeps (by chance: about 28%), and 93% of the badly pinned ones | A steady lean: the lane direction is always the softer one |
| Things carried along (close, unchanged for 5 s while moving) | None | The rear filter works; the user and handle are not in the sweeps |

**So the "LiDAR at desk height" idea is not well supported by this lap.** The view is mostly stable and
well shaped. What it does show is a small, steady weakness along the direction of travel, which adds up
to the 1.5 to 1.8 m shortfall over 68 m (about 2.5%). Raising the LiDAR might help, but this data
doesn't prove it would.

Placing on a saved map (16 Nov night, `../../results/map-drift/placing-findings-16nov.md`):
`place_bag.py` (recorded sweeps scored on a saved map), `place_sim.py` (the map manager's placing run
on a recording, also with a changed `relocalizer.py`), `place_live.py` (the live sweep on the map where
the device is shown). Older checks from the lab chat: `place_look.py`, `place_try.py`, `place_variants.py`.

Next (Adel to choose):
- A measured straight push (tape measure, about 5 m, once in the desk lane and once in open space):
  compares the tracked distance with the real one and shows whether the lane is the problem.
  Needs someone at the device; wheels pushed by hand, no motors.
- Same push with the LiDAR temporarily held higher (above the desks): a direct test of the height
  idea before straightening the tube.
- Offline: replay with sweep de-skewing (each sweep corrected for the device's movement while the
  LiDAR spins), a movement-tracker setting not tried yet.
