// The simulation loop: true robot, noisy wheel odometry, simulated LiDAR and
// an online estimator that predicts with odometry, optionally corrects with
// scan-to-map matching, and builds the occupancy grid from its own estimate.
// It has no knowledge of the true pose except at the start, where the map
// frame is defined to coincide with the start pose.

import { createRng } from "./rng.js";
import { DEFAULT_LIDAR, simulateScan } from "./lidar.js";
import {
  DEFAULT_ODOMETRY_NOISE,
  applyOdometryDelta,
  corruptOdometry,
  integrateUnicycle,
  odometryDelta,
} from "./motion.js";
import { createGrid, integrateScan, occupiedMask } from "./grid.js";
import { likelihoodField } from "./distance.js";
import { DEFAULT_MATCH, matchScan } from "./scanmatch.js";
import { collides } from "./world.js";
import { positionError } from "./metrics.js";

/** @typedef {import("./geometry.js").Pose} Pose */
/** @typedef {import("./world.js").World} World */
/** @typedef {import("./lidar.js").LidarConfig} LidarConfig */
/** @typedef {import("./lidar.js").Scan} Scan */
/** @typedef {import("./motion.js").OdometryNoise} OdometryNoise */
/** @typedef {import("./grid.js").OccupancyGrid} OccupancyGrid */
/** @typedef {import("./distance.js").LikelihoodField} LikelihoodField */
/** @typedef {import("./scanmatch.js").MatchOptions} MatchOptions */
/** @typedef {import("./scanmatch.js").MatchResult} MatchResult */
/** @typedef {import("./rng.js").Rng} Rng */

/**
 * @typedef {object} SimSettings
 * @property {number} seed
 * @property {number} dt Time step in seconds; one scan per step.
 * @property {number} robotRadius Metres, for collision checks.
 * @property {number} resolution Grid cell size in metres.
 * @property {boolean} scanMatching Correct the odometry guess with scan matching.
 * @property {number} fieldSigma Likelihood-field fall-off in metres.
 * @property {number} fieldOccupiedThreshold Log-odds above which a cell is an obstacle for matching.
 * @property {LidarConfig} lidar
 * @property {OdometryNoise} odometry
 * @property {MatchOptions} match
 */

/** @returns {SimSettings} */
export function defaultSettings() {
  return {
    seed: 1,
    dt: 0.1,
    robotRadius: 0.2,
    resolution: 0.05,
    scanMatching: true,
    fieldSigma: 0.1,
    fieldOccupiedThreshold: 0.5,
    lidar: { ...DEFAULT_LIDAR },
    odometry: { ...DEFAULT_ODOMETRY_NOISE },
    match: { ...DEFAULT_MATCH },
  };
}

/** Trajectories are thinned to every other point once they reach this length. */
export const MAX_TRAJECTORY_POINTS = 20000;

/**
 * @typedef {object} Simulation
 * @property {World} world
 * @property {SimSettings} settings
 * @property {number} time Seconds since reset.
 * @property {number} steps
 * @property {number} scansIntegrated Scans already added to the map.
 * @property {number} errorSamples Steps included in the RMS sums.
 * @property {Pose} truePose
 * @property {Pose} odomPose Dead reckoning only.
 * @property {Pose} estPose Odometry plus scan matching (equals odomPose's
 *   behaviour when matching is off).
 * @property {OccupancyGrid} grid Map built from estPose.
 * @property {LikelihoodField | null} field
 * @property {Scan | null} lastScan
 * @property {MatchResult | null} lastMatch
 * @property {Pose[]} trueTrail
 * @property {Pose[]} odomTrail
 * @property {Pose[]} estTrail
 * @property {number} distance Metres actually driven.
 * @property {number} collisions Steps in which a wall blocked the motion.
 * @property {number} matchAttempts
 * @property {number} matchAccepted
 * @property {number} sumSqEst Running sum of squared estimate position error.
 * @property {number} sumSqOdom Running sum of squared odometry position error.
 * @property {Rng} lidarRng
 * @property {Rng} odomRng
 */

/**
 * @param {World} world
 * @param {SimSettings} settings
 * @returns {Simulation}
 */
export function createSimulation(world, settings) {
  const margin = 2;
  const grid = createGrid(
    world.width + 2 * margin,
    world.height + 2 * margin,
    settings.resolution,
    -margin,
    -margin,
  );
  const start = { ...world.start };
  return {
    world,
    settings,
    time: 0,
    steps: 0,
    scansIntegrated: 0,
    errorSamples: 0,
    truePose: { ...start },
    odomPose: { ...start },
    estPose: { ...start },
    grid,
    field: null,
    lastScan: null,
    lastMatch: null,
    trueTrail: [{ ...start }],
    odomTrail: [{ ...start }],
    estTrail: [{ ...start }],
    distance: 0,
    collisions: 0,
    matchAttempts: 0,
    matchAccepted: 0,
    sumSqEst: 0,
    sumSqOdom: 0,
    // Independent streams, so changing the LiDAR settings does not change
    // the odometry noise sequence and the other way round.
    lidarRng: createRng(settings.seed),
    odomRng: createRng(settings.seed ^ 0x9e3779b9),
  };
}

/**
 * Advance the simulation by one time step with a velocity command.
 * @param {Simulation} sim
 * @param {number} v Forward speed in m/s.
 * @param {number} w Turn rate in rad/s.
 */
export function step(sim, v, w) {
  const { settings, world } = sim;
  const prev = sim.truePose;
  let next = integrateUnicycle(prev, v, w, settings.dt);
  if (collides(world.walls, prev.x, prev.y, next.x, next.y, settings.robotRadius)) {
    // Blocked: the wheels do not turn, only the in-place rotation happens.
    next = { x: prev.x, y: prev.y, theta: integrateUnicycle(prev, 0, w, settings.dt).theta };
    sim.collisions++;
  }
  sim.truePose = next;
  sim.distance += Math.hypot(next.x - prev.x, next.y - prev.y);

  const measured = corruptOdometry(odometryDelta(prev, next), settings.odometry, sim.odomRng);
  sim.odomPose = applyOdometryDelta(sim.odomPose, measured);
  const guess = applyOdometryDelta(sim.estPose, measured);

  const scan = simulateScan(world.walls, next, settings.lidar, sim.lidarRng);
  sim.lastScan = scan;

  if (settings.scanMatching && sim.scansIntegrated > 0) {
    if (sim.field === null) sim.field = buildField(sim);
    const result = matchScan(sim.field, scan, guess, settings.match);
    sim.lastMatch = result;
    sim.matchAttempts++;
    if (result.accepted) sim.matchAccepted++;
    sim.estPose = result.pose;
  } else {
    sim.lastMatch = null;
    sim.estPose = guess;
  }

  integrateScan(sim.grid, sim.estPose, scan);
  sim.scansIntegrated++;
  sim.field = null; // the map changed; rebuild lazily before the next match

  sim.time += settings.dt;
  sim.steps++;
  const eEst = positionError(sim.truePose, sim.estPose);
  const eOdom = positionError(sim.truePose, sim.odomPose);
  sim.sumSqEst += eEst * eEst;
  sim.sumSqOdom += eOdom * eOdom;
  sim.errorSamples++;
  pushTrail(sim.trueTrail, sim.truePose);
  pushTrail(sim.odomTrail, sim.odomPose);
  pushTrail(sim.estTrail, sim.estPose);
}

/**
 * Integrate the very first scan at the start pose. Call once after creating
 * the simulation so the map is not empty before the robot first moves.
 * @param {Simulation} sim
 */
export function initialScan(sim) {
  const scan = simulateScan(sim.world.walls, sim.truePose, sim.settings.lidar, sim.lidarRng);
  sim.lastScan = scan;
  integrateScan(sim.grid, sim.estPose, scan);
  sim.scansIntegrated++;
  sim.field = null;
}

/**
 * @param {Simulation} sim
 * @returns {LikelihoodField}
 */
export function buildField(sim) {
  const mask = occupiedMask(sim.grid, sim.settings.fieldOccupiedThreshold);
  return likelihoodField(mask, sim.grid, sim.settings.fieldSigma);
}

/**
 * Append a pose, thinning the trail when it grows too long.
 * @param {Pose[]} trail
 * @param {Pose} pose
 */
function pushTrail(trail, pose) {
  trail.push({ ...pose });
  if (trail.length >= MAX_TRAJECTORY_POINTS) {
    let k = 0;
    for (let i = 0; i < trail.length; i += 2) trail[k++] = trail[i];
    trail.length = k;
  }
}

/**
 * @typedef {object} SimSummary
 * @property {number} time
 * @property {number} distance
 * @property {number} odomError Current position error of dead reckoning (m).
 * @property {number} estError Current position error of the estimate (m).
 * @property {number} odomRms RMS position error of dead reckoning over all steps (m).
 * @property {number} estRms RMS position error of the estimate over all steps (m).
 * @property {number} acceptRate Fraction of scan matches accepted (NaN if none attempted).
 * @property {number} collisions
 */

/**
 * @param {Simulation} sim
 * @returns {SimSummary}
 */
export function summarize(sim) {
  const n = Math.max(1, sim.errorSamples);
  return {
    time: sim.time,
    distance: sim.distance,
    odomError: positionError(sim.truePose, sim.odomPose),
    estError: positionError(sim.truePose, sim.estPose),
    odomRms: Math.sqrt(sim.sumSqOdom / n),
    estRms: Math.sqrt(sim.sumSqEst / n),
    acceptRate: sim.matchAttempts === 0 ? NaN : sim.matchAccepted / sim.matchAttempts,
    collisions: sim.collisions,
  };
}
