# Changelog

All notable changes to this project are documented here. The format follows
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/) and the project uses
[Semantic Versioning](https://semver.org/).

## [0.2.0] - 2026-11-25

Requested by Robot Studio, which vendors these modules (ecosystem audit
findings 20, 27 and 35). With default settings every result is the same as in
0.1.0: `benchmark` prints identical numbers.

### Added

- LiDAR mount pose: `simulateScan(walls, pose, config, rng, mount)` casts from
  the mounted sensor and stores the mount as `scan.origin`; `integrateScan`,
  `matchScan`, `scoreScan`, MCL's `updateWeights` and `scanEndpoints` place
  the beams at robot pose (+) origin, so the estimators track the robot frame.
  New helpers `composePose` and `sensorPose`.
- `minRange` (returns closer than it are no return) and `angleMin` (an
  asymmetric fan, as LaserScan's `angle_min`) in the LiDAR settings.
- MCL `motionModel: "omni"`: motion noise sampled in the body frame (forward,
  sideways, turn) instead of rot1-trans-rot2, which reads backward driving and
  mecanum strafes as half or full turns. Predicting 1 m of backward driving in
  2 cm steps: mean -0.37 m and 1.0 m spread with "odometry", -1.00 m and
  0.01 m with "omni" (test). Motion helpers `bodyDelta`, `applyBodyDelta`,
  `bodyFromOdometry`, `corruptBodyDelta`.
- Recorded scans as input: `src/scanlog.js` reads a JSON Lines scan log
  (LaserScan fields plus the odometry pose; `null`, non-finite and
  out-of-range values are no return) and runs the same estimators;
  `lidar-slam-lab localize <log>` with `--map` (MCL), `--mount`, `--start`,
  `--motion`, `--out`, `--json`; `run --scan-log <file>` writes a log of a
  simulated run with its true poses.
- `exports` map in `package.json` naming the public modules.

## [0.1.0] - 2026-11-19

First release.

### Added

- Browser app with side-by-side ground truth and estimate views, keyboard and
  touch driving, demo-route autopilot, wall drawing and erasing, map JSON
  load/save, robot kidnapping, noise sliders, PGM and CSV downloads and URL
  parameters.
- Simulated 2D LiDAR (ray casting, range noise, dropouts) and a wheel odometry
  error model with systematic heading bias.
- Log-odds occupancy grid mapping.
- Correlative scan-to-map matching on a likelihood field built with an exact
  Euclidean distance transform.
- Monte Carlo localisation with low-variance resampling and augmented-MCL
  random particle injection, for tracking and global localisation.
- Three built-in floor plans with demo routes, and a validated JSON map format.
- Headless CLI (`run`, `benchmark`, `maps`, `export-map`) writing ROS
  `map_server` PGM/YAML maps and trajectory CSV.
- Tests with `node:test`, JSDoc type checking with TypeScript, CI on Ubuntu
  and Windows with Node 22 and 24, a headless Chrome smoke test, and
  deployment to GitHub Pages.

[0.2.0]: https://github.com/projectbin07-stack/lidar-slam-lab/releases/tag/v0.2.0
[0.1.0]: https://github.com/projectbin07-stack/lidar-slam-lab/releases/tag/v0.1.0
