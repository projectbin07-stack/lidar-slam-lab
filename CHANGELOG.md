# Changelog

All notable changes to this project are documented here. The format follows
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/) and the project uses
[Semantic Versioning](https://semver.org/).

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

[0.1.0]: https://github.com/projectbin07-stack/lidar-slam-lab/releases/tag/v0.1.0
