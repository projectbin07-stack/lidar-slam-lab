// LiDAR mount pose (scan origin), minimum range, asymmetric fans and the
// omnidirectional MCL motion model (0.2.0).
import test from "node:test";
import assert from "node:assert/strict";
import { close } from "./helpers.js";
import { composePose } from "../src/geometry.js";
import { beamAngles, scanEndpoints, sensorPose, simulateScan } from "../src/lidar.js";
import { createGrid, integrateScan, occupiedMask } from "../src/grid.js";
import { likelihoodField } from "../src/distance.js";
import { matchScan, scoreScan } from "../src/scanmatch.js";
import {
  applyBodyDelta,
  applyOdometryDelta,
  bodyDelta,
  bodyFromOdometry,
  corruptBodyDelta,
  odometryDelta,
} from "../src/motion.js";
import {
  createParticleFilter,
  estimate,
  initializeAround,
  predict,
  updateWeights,
} from "../src/mcl.js";
import { createRng } from "../src/rng.js";
import { boundaryWalls } from "../src/world.js";
import { BUILTIN_MAPS } from "../src/maps.js";
import { rasterizeWalls } from "../src/metrics.js";

const quiet = { beams: 4, fov: 2 * Math.PI, maxRange: 10, rangeNoise: 0, dropout: 0 };
const ZERO = { rotFromRot: 0, rotFromTrans: 0, transFromTrans: 0, transFromRot: 0, headingBias: 0 };
const mount = { x: 0.2, y: -0.1, theta: Math.PI / 2 };

test("composePose: a sensor 0.2 m ahead of a robot facing +y is 0.2 m further up", () => {
  const p = composePose({ x: 1, y: 2, theta: Math.PI / 2 }, { x: 0.2, y: 0, theta: 0.1 });
  close(p.x, 1);
  close(p.y, 2.2);
  close(p.theta, Math.PI / 2 + 0.1);
});

test("beam angles: an asymmetric fan starts at angleMin", () => {
  const a = beamAngles(3, Math.PI / 2, -0.5);
  close(a[0], -0.5);
  close(a[2], -0.5 + Math.PI / 2);
  assert.throws(() => beamAngles(3, 1, NaN), RangeError);
});

test("a mounted LiDAR casts from its mount: room ranges by hand", () => {
  // 4 x 2 room, robot at (1, 0.5) facing +x, LiDAR 0.2 m ahead, 0.1 m right, turned +90 deg:
  // it sits at (1.2, 0.4) facing +y; beams at 0, 90, 180, 270 deg in its frame
  // point north, west, south and east.
  const scan = simulateScan(
    boundaryWalls(4, 2),
    { x: 1, y: 0.5, theta: 0 },
    quiet,
    createRng(1),
    mount,
  );
  close(scan.ranges[0], 1.6);
  close(scan.ranges[1], 1.2);
  close(scan.ranges[2], 0.4);
  close(scan.ranges[3], 2.8);
  assert.deepEqual(scan.origin, mount);
  // The same as a LiDAR without a mount placed at the sensor pose.
  const direct = simulateScan(
    boundaryWalls(4, 2),
    sensorPose(scan, { x: 1, y: 0.5, theta: 0 }),
    quiet,
    createRng(1),
  );
  assert.deepEqual(Array.from(scan.ranges), Array.from(direct.ranges));
  assert.equal(direct.origin, undefined);
  // End points land on the walls.
  const pts = scanEndpoints(scan, { x: 1, y: 0.5, theta: 0 });
  close(pts[0], 1.2);
  close(pts[1], 2);
  close(pts[6], 4);
  close(pts[7], 0.4);
});

test("minimum range: closer returns become no return", () => {
  const cfg = { ...quiet, minRange: 0.5 };
  const scan = simulateScan(boundaryWalls(4, 2), { x: 1, y: 0.4, theta: 0 }, cfg, createRng(1));
  close(scan.ranges[0], 3);
  assert.ok(Number.isNaN(scan.ranges[3]), "the south wall 0.4 m away is below the minimum");
});

test("the grid traces beams from the sensor, as an unmounted scan at the sensor pose", () => {
  const walls = boundaryWalls(4, 3);
  const lidar = { beams: 90, fov: 2 * Math.PI, maxRange: 8, rangeNoise: 0, dropout: 0 };
  const robot = { x: 1.5, y: 1.2, theta: 0.4 };
  const a = createGrid(5, 4, 0.05, -0.5, -0.5);
  const b = createGrid(5, 4, 0.05, -0.5, -0.5);
  const mounted = simulateScan(walls, robot, lidar, createRng(2), mount);
  integrateScan(a, robot, mounted);
  integrateScan(b, sensorPose(mounted, robot), { ...mounted, origin: undefined });
  assert.deepEqual(Array.from(a.logOdds), Array.from(b.logOdds));
});

/** A reference map of the office from three noise-free scans. */
function officeField() {
  const world = BUILTIN_MAPS.office();
  const lidar = { beams: 180, fov: 2 * Math.PI, maxRange: 8, rangeNoise: 0, dropout: 0 };
  const grid = createGrid(world.width + 2, world.height + 2, 0.05, -1, -1);
  const rng = createRng(1);
  for (const p of [
    { x: 2, y: 7, theta: 0 },
    { x: 6, y: 7, theta: 0.3 },
    { x: 9, y: 6.5, theta: 1 },
  ])
    integrateScan(grid, p, simulateScan(world.walls, p, lidar, rng));
  return { world, lidar, grid, field: likelihoodField(occupiedMask(grid, 0.5), grid, 0.1) };
}

test("scan matching with a mounted LiDAR recovers the robot pose, not the sensor pose", () => {
  const { world, lidar, field } = officeField();
  const truth = { x: 5, y: 7.2, theta: 0.2 };
  const m = { x: 0.25, y: 0.05, theta: 0.3 };
  const scan = simulateScan(world.walls, truth, { ...lidar, rangeNoise: 0.01 }, createRng(9), m);
  const guess = { x: truth.x + 0.12, y: truth.y - 0.08, theta: truth.theta + 0.04 };
  const res = matchScan(field, scan, guess);
  assert.equal(res.accepted, true, res.reason);
  assert.ok(
    Math.hypot(res.pose.x - truth.x, res.pose.y - truth.y) < 0.05,
    JSON.stringify(res.pose),
  );
  assert.ok(Math.abs(res.pose.theta - truth.theta) < 0.01);
  // The score of a mounted scan at the robot pose equals that of an
  // unmounted scan at the sensor pose.
  close(
    scoreScan(field, scan, truth),
    scoreScan(field, { ...scan, origin: undefined }, sensorPose(scan, truth)),
    1e-12,
  );
});

test("MCL weights a mounted scan as an unmounted scan from each particle's sensor pose", () => {
  const { world, lidar, grid } = officeField();
  const known = likelihoodField(rasterizeWalls(grid, world.walls), grid, 0.2);
  const m = { x: 0.3, y: 0, theta: 0 };
  const truth = { x: 4, y: 6.8, theta: -0.4 };
  const scan = simulateScan(world.walls, truth, lidar, createRng(3), m);
  const a = createParticleFilter(50, known, createRng(5));
  initializeAround(a, truth, 0.2, 0.1);
  const b = createParticleFilter(50, known, createRng(5));
  for (let i = 0; i < 50; i++) {
    const s = composePose({ x: a.x[i], y: a.y[i], theta: a.theta[i] }, m);
    b.x[i] = s.x;
    b.y[i] = s.y;
    b.theta[i] = a.theta[i] + m.theta;
  }
  updateWeights(a, scan);
  updateWeights(b, { ...scan, origin: undefined });
  for (let i = 0; i < 50; i++) close(a.weight[i], b.weight[i], 1e-9);
});

test("body deltas: inverse of each other and the same end pose as rot1-trans-rot2", () => {
  const a = { x: 1, y: -2, theta: 2.5 };
  const b = { x: 1.3, y: -1.6, theta: -2.9 };
  const d = bodyDelta(a, b);
  const back = applyBodyDelta(a, d);
  close(back.x, b.x);
  close(back.y, b.y);
  close(back.theta, b.theta);
  const viaOdom = applyBodyDelta(a, bodyFromOdometry(odometryDelta(a, b)));
  const ref = applyOdometryDelta(a, odometryDelta(a, b));
  close(viaOdom.x, ref.x);
  close(viaOdom.y, ref.y);
  close(viaOdom.theta, ref.theta);
  // A strafe to the left is a sideways move, not two turns.
  const s = bodyDelta({ x: 0, y: 0, theta: 0 }, { x: 0, y: 0.1, theta: 0 });
  close(s.dx, 0);
  close(s.dy, 0.1);
  close(s.dtheta, 0);
});

test("body-delta noise: zero noise is the identity, bias adds rotation per metre, three draws", () => {
  const d = { dx: 0.3, dy: 0.4, dtheta: 0.1 };
  const z = corruptBodyDelta(d, ZERO, createRng(1));
  assert.deepEqual(z, d);
  const biased = corruptBodyDelta(d, { ...ZERO, headingBias: 0.02 }, createRng(1));
  close(biased.dtheta, 0.1 + 0.02 * 0.5);
  const r1 = createRng(4);
  corruptBodyDelta(d, { ...ZERO, transFromTrans: 0.1 }, r1);
  const r2 = createRng(4);
  r2.gaussian();
  r2.gaussian();
  r2.gaussian();
  assert.equal(r1.next(), r2.next());
  // Sideways noise scales with the sideways motion only.
  const n = { ...ZERO, transFromTrans: 0.1 };
  const rng = createRng(7);
  let sx = 0;
  let sy = 0;
  const N = 4000;
  for (let i = 0; i < N; i++) {
    const c = corruptBodyDelta({ dx: 0, dy: 1, dtheta: 0 }, n, rng);
    sx += c.dx ** 2;
    sy += (c.dy - 1) ** 2;
  }
  close(Math.sqrt(sx / N), 0, 1e-12);
  close(Math.sqrt(sy / N), 0.1, 0.005);
});

test("MCL motion models: omni with zero noise moves particles exactly; unknown models throw", () => {
  const field = likelihoodField(
    new Uint8Array(1),
    { resolution: 1, width: 1, height: 1, originX: 0, originY: 0 },
    1,
  );
  const pf = createParticleFilter(3, field, createRng(1), {
    motionModel: "omni",
    motionNoise: ZERO,
  });
  pf.theta.set([0, Math.PI / 2, Math.PI]);
  predict(pf, odometryDelta({ x: 0, y: 0, theta: 0 }, { x: 0, y: 0.5, theta: 0.2 }));
  close(pf.x[0], 0);
  close(pf.y[0], 0.5);
  close(pf.x[1], -0.5);
  close(pf.y[1], 0);
  close(pf.theta[2], -Math.PI + 0.2);
  const bad = createParticleFilter(
    1,
    field,
    createRng(1),
    /** @type {any} */ ({ motionModel: "warp" }),
  );
  assert.throws(() => predict(bad, { rot1: 0, trans: 0.1, rot2: 0 }), RangeError);
});

/**
 * Predict only (no measurements): 50 steps of 2 cm in the body direction
 * (dx, dy), 2000 particles, default MCL noise.
 * @param {"odometry" | "omni"} motionModel
 * @param {number} dx
 * @param {number} dy
 */
function predictOnly(motionModel, dx, dy) {
  const field = likelihoodField(
    new Uint8Array(1),
    { resolution: 1, width: 1, height: 1, originX: 0, originY: 0 },
    1,
  );
  const pf = createParticleFilter(2000, field, createRng(1), { motionModel });
  const step = odometryDelta({ x: 0, y: 0, theta: 0 }, { x: dx, y: dy, theta: 0 });
  for (let k = 0; k < 50; k++) predict(pf, step);
  let th = 0;
  for (let i = 0; i < pf.count; i++) th += pf.theta[i] ** 2;
  return { ...estimate(pf), headingSd: Math.sqrt(th / pf.count) };
}

test("backward and sideways motion: rot1-trans-rot2 smears and biases the cloud, omni does not", () => {
  // Driving backward is rot1 = pi, trans, rot2 = -pi; a strafe is +-pi/2 twice.
  // The rotation-dependent noise then dwarfs the 2 cm steps, and clipping the
  // noisy distance at 0 shortens the path. After 1 m:
  const back = predictOnly("odometry", -0.02, 0);
  assert.ok(back.pose.x > -0.6, `odometry model mean x ${back.pose.x} (truth -1)`);
  assert.ok(back.spread > 0.5 && back.headingSd > 1, JSON.stringify(back));
  const left = predictOnly("odometry", 0, 0.02);
  assert.ok(left.pose.y < 0.85 && left.spread > 0.5, JSON.stringify(left));
  for (const [dx, dy] of [
    [-0.02, 0],
    [0, 0.02],
    [0.02, 0],
  ]) {
    const o = predictOnly("omni", dx, dy);
    assert.ok(Math.hypot(o.pose.x - 50 * dx, o.pose.y - 50 * dy) < 0.01, JSON.stringify(o));
    assert.ok(o.spread < 0.02 && o.headingSd < 0.02, JSON.stringify(o));
  }
  // Forward driving is the same for both models (within sampling noise).
  const fwd = predictOnly("odometry", 0.02, 0);
  assert.ok(Math.abs(fwd.pose.x - 1) < 0.01 && fwd.spread < 0.02, JSON.stringify(fwd));
});
