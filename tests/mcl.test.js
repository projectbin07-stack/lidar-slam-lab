import test from "node:test";
import assert from "node:assert/strict";
import { close } from "./helpers.js";
import {
  createParticleFilter,
  effectiveSampleSize,
  estimate,
  initializeAround,
  initializeUniform,
  injectionRate,
  lowVarianceIndices,
  predict,
  randomFreePose,
  resampleIfNeeded,
  updateWeights,
} from "../src/mcl.js";
import { likelihoodField } from "../src/distance.js";
import { createRng } from "../src/rng.js";
import { BUILTIN_MAPS } from "../src/maps.js";
import { boundaryWalls } from "../src/world.js";
import { pointSegmentDistance } from "../src/geometry.js";
import { simulateScan } from "../src/lidar.js";
import { createGrid } from "../src/grid.js";
import { rasterizeWalls } from "../src/metrics.js";
import { followRoute } from "../src/autopilot.js";
import { createSimulation, defaultSettings, initialScan, step, summarize } from "../src/sim.js";

const ZERO_NOISE = {
  rotFromRot: 0,
  rotFromTrans: 0,
  transFromTrans: 0,
  transFromRot: 0,
  headingBias: 0,
};

/** A 1 x 1 field that is zero everywhere; enough for tests that never sample it. */
const dummyField = likelihoodField(
  new Uint8Array(1),
  { resolution: 1, width: 1, height: 1, originX: 0, originY: 0 },
  1,
);

test("effective sample size: uniform weights give N, one-hot gives 1", () => {
  close(effectiveSampleSize(new Float64Array([0.25, 0.25, 0.25, 0.25])), 4);
  close(effectiveSampleSize(new Float64Array([0, 1, 0, 0])), 1);
  close(effectiveSampleSize(new Float64Array([0.5, 0.5, 0, 0])), 2);
});

test("low-variance resampling: hand-worked cases", () => {
  // All weight on particle 1: every pointer lands there.
  assert.deepEqual(Array.from(lowVarianceIndices(new Float64Array([0, 1, 0]), 0.1)), [1, 1, 1]);
  // Equal weights with offset 0.1 / N reproduce the set exactly.
  const eq = new Float64Array([0.25, 0.25, 0.25, 0.25]);
  assert.deepEqual(Array.from(lowVarianceIndices(eq, 0.025)), [0, 1, 2, 3]);
  // Weights 0.5, 0.25, 0.25 with r = 0.1: pointers 0.1, 0.35, 0.6, 0.85 over
  // cumulative 0.5, 0.75, 1.0 select 0, 0, 1, 2.
  const w = new Float64Array([0.5, 0.25, 0.25, 0]);
  assert.deepEqual(Array.from(lowVarianceIndices(w, 0.1)), [0, 0, 1, 2]);
});

test("predict without noise moves every particle by the delta in its own frame", () => {
  const pf = createParticleFilter(2, dummyField, createRng(1), { motionNoise: ZERO_NOISE });
  pf.x.set([0, 5]);
  pf.y.set([0, 5]);
  pf.theta.set([0, Math.PI / 2]);
  predict(pf, { rot1: 0, trans: 1, rot2: 0.5 });
  close(pf.x[0], 1);
  close(pf.y[0], 0);
  close(pf.theta[0], 0.5);
  close(pf.x[1], 5);
  close(pf.y[1], 6);
  close(pf.theta[1], Math.PI / 2 + 0.5);
});

test("estimate uses a circular mean for the heading", () => {
  const pf = createParticleFilter(2, dummyField, createRng(1));
  pf.x.set([1, 3]);
  pf.y.set([2, 2]);
  pf.theta.set([Math.PI - 0.1, -Math.PI + 0.1]);
  const e = estimate(pf);
  close(e.pose.x, 2);
  close(e.pose.y, 2);
  close(Math.abs(e.pose.theta), Math.PI, 1e-9);
  close(e.spread, 1);
});

test("initialisation around a pose and over free space", () => {
  const pf = createParticleFilter(2000, dummyField, createRng(3));
  initializeAround(pf, { x: 2, y: 3, theta: 0.5 }, 0.1, 0.05);
  const e = estimate(pf);
  close(e.pose.x, 2, 0.02);
  close(e.pose.y, 3, 0.02);
  close(e.pose.theta, 0.5, 0.01);
  // Spread of a 2D Gaussian with sigma 0.1 per axis: sqrt(2) * 0.1.
  close(e.spread, Math.SQRT2 * 0.1, 0.01);

  assert.throws(() => initializeUniform(pf), /free space/);
  const walls = boundaryWalls(4, 2);
  const space = { width: 4, height: 2, walls, clearance: 0.3 };
  const g = createParticleFilter(500, dummyField, createRng(4), {}, space);
  initializeUniform(g);
  for (let i = 0; i < g.count; i++) {
    const d = Math.min(...walls.map((w) => pointSegmentDistance(g.x[i], g.y[i], w)));
    assert.ok(d >= 0.3);
  }
  // A map with no free space returns a pose rather than looping forever.
  const p = randomFreePose(createRng(1), { width: 1, height: 1, walls, clearance: 10 });
  assert.ok(Number.isFinite(p.x));
});

/**
 * Known-map field for the office, as the simulator builds it.
 * @param {number} count
 * @param {number} seed
 */
function officeFilter(count, seed) {
  const world = BUILTIN_MAPS.office();
  const grid = createGrid(world.width + 4, world.height + 4, 0.05, -2, -2);
  const field = likelihoodField(rasterizeWalls(grid, world.walls), grid, 0.2);
  const space = { width: world.width, height: world.height, walls: world.walls, clearance: 0.2 };
  return { world, pf: createParticleFilter(count, field, createRng(seed), {}, space) };
}

test("a scan favours particles at the true pose", () => {
  const { world, pf } = officeFilter(3, 1);
  const truth = { x: 3, y: 7, theta: 0.3 };
  pf.x.set([truth.x, truth.x + 0.5, 9]);
  pf.y.set([truth.y, truth.y, 2]);
  pf.theta.set([truth.theta, truth.theta, 0]);
  const lidar = { beams: 180, fov: 2 * Math.PI, maxRange: 8, rangeNoise: 0.02, dropout: 0 };
  const used = updateWeights(pf, simulateScan(world.walls, truth, lidar, createRng(2)));
  assert.equal(used, 36);
  assert.ok(pf.weight[0] > 0.99, `weights ${Array.from(pf.weight)}`);
  close(pf.weight[0] + pf.weight[1] + pf.weight[2], 1);
  assert.equal(resampleIfNeeded(pf), true);
  assert.deepEqual(Array.from(pf.x), [3, 3, 3]);
});

test("particles outside the map get zero weight; all impossible restarts globally", () => {
  const { world, pf } = officeFilter(2, 1);
  pf.x.set([3, -5]);
  pf.y.set([7, 7]);
  const lidar = { beams: 90, fov: 2 * Math.PI, maxRange: 8, rangeNoise: 0, dropout: 0 };
  const scan = simulateScan(world.walls, { x: 3, y: 7, theta: 0 }, lidar, createRng(1));
  updateWeights(pf, scan);
  assert.equal(pf.weight[1], 0);
  pf.x.set([-5, 50]);
  pf.weight.fill(0.5);
  updateWeights(pf, scan);
  assert.ok(pf.x.every((x) => x >= 0 && x <= world.width));
  const empty = { ...scan, ranges: scan.ranges.map(() => NaN) };
  assert.equal(updateWeights(pf, empty), 0);
});

test("injection starts when the short-term fit drops below the long-term fit", () => {
  const { pf } = officeFilter(10, 1);
  assert.equal(injectionRate(pf), 0);
  pf.wSlow = 0.5;
  pf.wFast = 0.5;
  assert.equal(injectionRate(pf), 0);
  // A 2% drop is inside the 5% dead band.
  pf.wFast = 0.49;
  assert.equal(injectionRate(pf), 0);
  // A 10% drop: 0.10 - 0.05 = 0.05.
  pf.wFast = 0.45;
  close(injectionRate(pf), 0.05);
  // A 40% drop is capped at maxInjection = 0.1.
  pf.wFast = 0.3;
  close(injectionRate(pf), 0.1);
  pf.space = null;
  assert.equal(injectionRate(pf), 0);
});

/**
 * @param {Partial<import("../src/sim.js").SimSettings>} overrides
 * @param {number} steps
 */
function runOffice(overrides, steps) {
  const world = BUILTIN_MAPS.office();
  const sim = createSimulation(world, { ...defaultSettings(), scanMatching: false, ...overrides });
  initialScan(sim);
  let idx = 0;
  for (let k = 0; k < steps; k++) {
    const cmd = followRoute(sim.truePose, world.route, idx);
    idx = cmd.index;
    step(sim, cmd.v, cmd.w);
  }
  return sim;
}

test("MCL tracking stays within 0.1 m while odometry drifts by metres", () => {
  const s = summarize(runOffice({ seed: 2, mclParticles: 300 }, 600));
  assert.ok(s.odomError > 1, `odometry error ${s.odomError}`);
  assert.ok(s.mclRms < 0.1, `MCL RMS ${s.mclRms}`);
  assert.ok(s.mclSpread < 0.2);
});

test("MCL localises globally in the office from a uniform start", () => {
  const sim = runOffice({ seed: 1, mclParticles: 2000, mclGlobal: true }, 500);
  const s = summarize(sim);
  assert.ok(s.mclError < 0.15, `MCL error ${s.mclError}`);
  assert.ok(sim.pf !== null && sim.pf.resamples > 0);
});

test("with MCL disabled the summary reports NaN", () => {
  const s = summarize(runOffice({ mclParticles: 0 }, 5));
  assert.ok(Number.isNaN(s.mclError));
  assert.ok(Number.isNaN(s.mclRms));
});

test("the reported MCL pose is taken before resampling, so injected random particles do not pull it", () => {
  const world = BUILTIN_MAPS.office();
  const sim = createSimulation(world, { ...defaultSettings(), seed: 4 });
  initialScan(sim);
  let idx = 0;
  for (let k = 0; k < 30; k++) {
    const cmd = followRoute(sim.truePose, world.route, idx);
    idx = cmd.index;
    step(sim, cmd.v, cmd.w);
  }
  const pf = /** @type {import("../src/mcl.js").ParticleFilter} */ (sim.pf);
  // Make the long-term fit look far better than the recent one: recovery injects
  // the maximum fraction (10 %) of random particles at the next resampling.
  pf.wSlow = 1000;
  const injected = pf.injected;
  const cmd = followRoute(sim.truePose, world.route, idx);
  step(sim, cmd.v, cmd.w);
  assert.ok(pf.injected - injected > 10, `injected ${pf.injected - injected}`);
  const reported = Math.hypot(
    /** @type {any} */ (sim.mclPose).x - sim.truePose.x,
    /** @type {any} */ (sim.mclPose).y - sim.truePose.y,
  );
  // The resampled set (uniform weights, random particles included) is pulled away.
  const after = estimate(pf).pose;
  const pulled = Math.hypot(after.x - sim.truePose.x, after.y - sim.truePose.y);
  assert.ok(reported < 0.1, `reported ${reported}`);
  assert.ok(pulled > 3 * reported, `after resampling ${pulled}, reported ${reported}`);
});
