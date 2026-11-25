# How it works

This page describes every model and algorithm in the simulator, with the
parameters it uses by default and the places where it deliberately simplifies.
Source files are named in each heading.

Conventions: metres and radians, world frame with x to the right and y up,
headings counter-clockwise from +x. One simulation step is 0.1 s, and the
LiDAR produces one scan per step.

## Data flow per step

```
velocity command (keyboard, touch or autopilot)
        |
        v
true robot (unicycle) --collision check--> true pose
        |                                     |
        | true motion                         | ray casting
        v                                     v
odometry error model                  LiDAR scan (noise, dropouts)
        |                                     |
        +--> dead reckoning (red)             |
        |                                     |
        +--> odometry guess --scan matching--> estimate (blue) --> occupancy grid
        |                    (against the grid built so far)
        |
        +--> particle filter: predict ----weight with scan----> MCL estimate (purple)
                              (against the true floor plan)
```

The estimators never see the true pose, except that all of them start at the
known start pose (the map frame is defined by it). With "global localisation"
the particle filter does not get the start pose either.

## Robot and collisions (`motion.js`, `world.js`)

The robot is a unicycle: forward speed `v` and turn rate `w`, integrated
exactly over each step (a circular arc, or a straight line when `w` is 0). It
is a disc with radius 0.2 m. A step is blocked if the path crosses a wall or
ends within the radius of a wall it is approaching; the rotation part of the
step still happens. A blocked step produces no odometry translation.

## Odometry errors (`motion.js`)

Relative motion between two poses is split into rotate (rot1), translate
(trans), rotate (rot2), as in the odometry motion model of Thrun, Burgard and
Fox, _Probabilistic Robotics_ (2005), section 5.4. The odometry reading is the
true motion plus Gaussian noise whose standard deviation grows linearly with
the motion:

| term             | default      | meaning                                    |
| ---------------- | ------------ | ------------------------------------------ |
| `rotFromRot`     | 0.05 rad/rad | rotation error per radian turned           |
| `rotFromTrans`   | 0.02 rad/m   | rotation error per metre driven            |
| `transFromTrans` | 0.03 m/m     | distance error per metre driven            |
| `transFromRot`   | 0.005 m/rad  | distance error per radian turned           |
| `headingBias`    | 0.02 rad/m   | systematic turn per metre (unequal wheels) |

The book parameterises variances; here they are standard deviations, which
are easier to reason about. The bias is what makes dead reckoning curve away
over a long run; the noise terms alone mostly cancel out. The "Odometry error"
slider and `--odom-noise` scale all five terms together.

These values are illustrative. They were not measured on any robot.

## LiDAR (`lidar.js`, `geometry.js`)

Each beam is a ray cast against every wall segment (ray-segment intersection
by 2D cross products); the nearest hit within the maximum range wins. Defaults:
180 beams over 360 degrees, 8 m range, range noise with 0.02 m standard
deviation, 3 % of beams dropped at random. A dropped beam and a beam that hits
nothing are both reported as "no return" (NaN).

Since 0.2.0 a scan may carry an `origin`: the LiDAR's mount pose (offset and
yaw) in the robot frame. `simulateScan(walls, pose, config, rng, mount)` casts
from the mounted sensor and records the mount; the grid, scan matching, MCL
and `scanEndpoints` all place the beams at robot pose (+) origin, so the
estimators keep tracking the robot frame while the rays start where the
sensor is. Without an origin every result is the same as in 0.1.0. A
`minRange` in the LiDAR settings reports closer returns as no return (real 2D
LiDARs see nothing within roughly 0.1 m), and `angleMin` gives an asymmetric
fan, as a LaserScan's `angle_min` does.

Not modelled: beam divergence, mixed pixels at edges, reflectivity and motion
distortion within a scan.

## Occupancy grid (`grid.js`)

A log-odds grid with 0.05 m cells and the classic inverse sensor model
(_Probabilistic Robotics_, chapter 9). For each beam with a return, the cells
on the Bresenham line from the robot to the endpoint get `miss = -0.4` added,
and the endpoint cell gets `hit = +0.85`. Values are clamped to [-4, 4] so
that cells can still change their mind. Beams without a return are skipped:
treating them as "free up to max range" would erase walls every time a beam
drops out.

Display and export use p = 1 - 1 / (1 + e^l). The PGM export follows the ROS
`map_server` trinary convention (occupied above 0.65, free below 0.196).

## Scan matching (`scanmatch.js`, `distance.js`)

This is what turns the odometry-only estimate into a much better one. It is a
simplified version of Olson's correlative scan matching (E. Olson, "Real-Time
Correlative Scan Matching", ICRA 2009), matching each new scan against the
map built so far:

1. Cells with log-odds above 0.5 count as obstacles. An exact Euclidean
   distance transform (P. Felzenszwalb and D. Huttenlocher, "Distance
   Transforms of Sampled Functions", Theory of Computing, 2012) gives every
   cell its distance d to the nearest obstacle.
2. The likelihood field is exp(-d^2 / (2 sigma^2)) with sigma = 0.1 m, sampled
   with bilinear interpolation. A scan scores the mean field value at its
   endpoints.
3. The search tries every pose in a window around the odometry guess:
   +/-0.2 m in steps of 0.05 m in x and y, and +/-0.1 rad in steps of
   0.02 rad (891 poses), on up to 120 evenly spaced beams.
4. The best pose is refined by trying +/- a step on each axis and halving the
   step six times.
5. A small penalty for distance from the guess (5 % of the score at the edge
   of the window) breaks ties in corridors, where sliding along the corridor
   does not change the score.
6. The correction is rejected, and the odometry guess kept, if fewer than 20
   beams are valid or the best mean score is below 0.25.

The new scan is then added to the grid at the corrected pose.

What this is not: there is no loop closure and no pose graph. Errors that
slip past the matcher stay in the map, and over a long run the estimate can
still drift slowly. The search window bounds how much odometry error per step
can be corrected (0.2 m and 0.1 rad); faster drift than that breaks it. The
pose is quantised by the grid, so expect errors of about one cell (0.05 m)
even when everything works.

## Monte Carlo localisation (`mcl.js`)

A particle filter that answers "where am I on this map?" given the true floor
plan (not the map the robot builds). It follows _Probabilistic Robotics_,
chapter 8:

- **Prediction.** Each particle moves by the measured odometry with its own
  noise sample, using the model above with larger terms (0.1 rad/rad,
  0.05 rad/m, 0.06 m/m, 0.01 m/rad) and no bias. The filter does not know the
  robot's systematic drift; it has to observe it.
- **Motion models** (`motionModel`, since 0.2.0). `"odometry"` (the default)
  samples rot1-trans-rot2 as above, which suits a robot that drives forward
  and turns. For backward or sideways motion it is a poor model: driving
  backward is rot1 = pi, rot2 = -pi and a mecanum strafe is two 90 degree
  turns, so the rotation-dependent terms produce noise far larger than a
  small step and clipping the noisy distance at zero shortens it. Predicting
  1 m of backward driving in 2 cm steps with 2000 particles and no
  measurements leaves the cloud's mean at x = -0.37 m instead of -1 m, with a
  1.0 m spread and a 1.8 rad heading spread (test `sensor-pose.test.js`).
  `"omni"` samples the motion in the body frame instead: forward
  `dx` with sd transFromTrans |dx| + transFromRot |dtheta|, sideways `dy`
  likewise, and the turn with sd rotFromRot |dtheta| + rotFromTrans |(dx, dy)|.
  Forward driving gets the same distance noise as before; backward and
  sideways steps get noise proportional to the step (the same prediction ends
  at -1.00 m with a 0.01 m spread). Use it for holonomic bases and for
  frames that move sideways when the robot turns.
- **Weighting.** The true walls are rasterised into the grid and turned into
  a likelihood field with sigma = 0.2 m (wider than for scan matching, so that
  particles a little off still get a gradient). For 36 evenly spaced beams,
  p(z | x) = product of (0.9 * field(endpoint) + 0.1). The 0.1 term keeps one
  unexpected reading from zeroing a good particle. Computed in log space.
  Particles outside the map rectangle get weight 0.
- **Resampling.** Low-variance (systematic) resampling, when the effective
  sample size 1 / sum(w^2) drops below half the particle count.
- **Recovery.** Augmented MCL (book table 8.3): the filter tracks a slow
  (alpha 0.01) and a fast (alpha 0.2) average of the per-beam fit. When the
  fast average drops more than 5 % below the slow one, that fraction minus 5 %
  of the particles (at most 10 %) are replaced by random poses in free space.
  The 5 % dead band was added after testing showed that the fit ratio wobbles
  by a few per cent during normal tracking, which without it kept injecting
  stray particles.
- **Estimate.** Weighted mean position, circular mean heading. The reported
  spread is the weighted RMS distance of the particles from the mean. It is
  taken after the measurement update and before resampling (since 0.2.1):
  after resampling the weights are uniform, and particles just injected by
  recovery would pull the mean.

Default: 300 particles started around the known start pose (tracking). With
"global localisation" they start uniformly over free space; use 2000 or more.

Limits seen in testing: global localisation needs the robot to move before it
converges, and in the nearly symmetric corridor loop it can settle on the
mirror-image pose (about 10 m away) and stay there, because that pose explains
almost every scan as well as the true one. That is a genuine property of the
map, not a bug, and it is a good illustration of why symmetric environments
are hard.

## Autopilot (`autopilot.js`)

The demo route is a list of waypoints. The controller turns towards the next
waypoint with a gain of 2.5 (rad/s per rad, limited to 1.5 rad/s) and drives
at up to 0.8 m/s scaled by cos^2 of the heading error, so it turns on the
spot when the waypoint is beside or behind it. A waypoint counts as reached
within 0.25 m.

The autopilot steers with the **true** pose. That keeps the driven path
identical whether scan matching is on or off, so the two can be compared on
the same path. A real robot would steer with its estimate.

## Metrics (`metrics.js`, `sim.js`)

- **Position error**: distance between the estimated and the true position,
  now and as an RMS over all steps (an absolute trajectory error without
  alignment, which is valid here because all estimates start at the true pose).
- **Map precision**: the share of cells the estimate believes are occupied
  (log-odds above 1) that lie within 0.1 m of a true wall. Smeared or doubled
  walls lower it.
- **Matched**: the share of scan matches that were accepted.

## Randomness

All noise comes from seeded mulberry32 generators, with separate streams for
the LiDAR, the odometry and the particle filter. The same seed and settings
give identical runs in the browser and the CLI, and changing a LiDAR setting
does not change the odometry noise sequence.

## Recorded scans (`scanlog.js`, `lidar-slam-lab localize`)

Since 0.2.0 the estimators also run on recorded data: a scan log, one JSON
object per line, with the fields of a `sensor_msgs/LaserScan` and the
odometry pose of the robot at that scan:

```json
{
  "t": 0.2,
  "odom": [0.1, 0.0, 0.01],
  "angle_min": -3.14159,
  "angle_increment": 0.01745,
  "range_min": 0.12,
  "range_max": 3.5,
  "ranges": [1.02, null, 0.98, "Infinity"]
}
```

- `t` (s, not decreasing), `odom` ([x, y, theta] in the odometry frame) and
  `ranges` are required on every line; `angle_min`, `angle_increment`,
  `range_min` and `range_max` on the first line and whenever they change
  (later lines inherit them).
- A range that is `null`, not a finite number (including the strings
  `"Infinity"`, `"-Infinity"` and `"NaN"` that some exporters write) or
  outside `[range_min, range_max]` is "no return", as REP 117 specifies.
- An optional `truth` ([x, y, theta] in the map frame) on every line makes
  `localize` report RMS errors; `run --scan-log` writes it.
- Lines that are blank or start with `#` are skipped; at most 200000 scans
  and 10000 beams per scan.

`localize` takes the odometry motion between consecutive scans as the
measured delta and runs exactly the per-step sequence of the simulation:
dead reckoning, odometry + scan matching against the grid built from that
estimate, and, with `--map`, MCL on the given floor plan. `--mount x,y,yaw`
sets the LiDAR's pose in the robot frame (the scans' origin), `--start` the
robot's pose at the first scan in the map frame. Re-running a simulated log
reproduces the simulation's errors to within a few millimetres (the log
rounds ranges to 0.1 mm); a test checks this.
