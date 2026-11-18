import test from "node:test";
import assert from "node:assert/strict";
import { close } from "./helpers.js";
import { distanceTransform, likelihoodField, sampleField } from "../src/distance.js";
import { createGrid, integrateScan, occupiedMask } from "../src/grid.js";
import { simulateScan } from "../src/lidar.js";
import { matchScan, scoreScan } from "../src/scanmatch.js";
import { createRng } from "../src/rng.js";
import { BUILTIN_MAPS } from "../src/maps.js";
import { rasterizeWalls } from "../src/metrics.js";

/**
 * Brute-force reference: distance to the nearest set cell.
 * @param {Uint8Array} mask
 * @param {number} w
 * @param {number} h
 */
function bruteForce(mask, w, h) {
  const out = new Float64Array(w * h).fill(Infinity);
  for (let j = 0; j < h; j++)
    for (let i = 0; i < w; i++)
      for (let q = 0; q < h; q++)
        for (let p = 0; p < w; p++)
          if (mask[q * w + p]) out[j * w + i] = Math.min(out[j * w + i], Math.hypot(i - p, j - q));
  return out;
}

test("distance transform of one centre cell in a 5 x 5 grid", () => {
  const mask = new Uint8Array(25);
  mask[12] = 1;
  const d = distanceTransform(mask, 5, 5);
  close(d[12], 0);
  close(d[13], 1);
  close(d[0], Math.sqrt(8));
  close(d[4], Math.sqrt(8));
  close(d[2], 2);
});

test("distance transform matches brute force on random masks", () => {
  const rng = createRng(5);
  for (let trial = 0; trial < 20; trial++) {
    const w = 3 + Math.floor(rng.next() * 15);
    const h = 3 + Math.floor(rng.next() * 15);
    const mask = new Uint8Array(w * h);
    for (let k = 0; k < mask.length; k++) mask[k] = rng.next() < 0.08 ? 1 : 0;
    mask[Math.floor(rng.next() * mask.length)] = 1;
    const fast = distanceTransform(mask, w, h);
    const ref = bruteForce(mask, w, h);
    for (let k = 0; k < fast.length; k++) close(fast[k], ref[k], 1e-9);
  }
});

test("empty mask gives infinite distances and a zero field", () => {
  const d = distanceTransform(new Uint8Array(6), 3, 2);
  assert.ok(Array.from(d).every((v) => v === Infinity));
  const f = likelihoodField(
    new Uint8Array(6),
    { resolution: 1, width: 3, height: 2, originX: 0, originY: 0 },
    1,
  );
  assert.equal(f.occupiedCells, 0);
  assert.ok(Array.from(f.values).every((v) => v === 0));
});

test("likelihood field is a Gaussian of distance and is bilinearly sampled", () => {
  // 1 x 3 grid with 0.1 m cells, obstacle in the middle; sigma = 0.1 m.
  const mask = new Uint8Array([0, 1, 0, 0, 1, 0]);
  const f = likelihoodField(
    mask,
    { resolution: 0.1, width: 3, height: 2, originX: 0, originY: 0 },
    0.1,
  );
  close(f.values[1], 1, 1e-7);
  close(f.values[0], Math.exp(-0.5), 1e-7); // one cell = 0.1 m = one sigma
  // Halfway between the centres of cells 0 and 1 on the bottom row.
  close(sampleField(f, 0.1, 0.05 + 1e-12), (Math.exp(-0.5) + 1) / 2, 1e-6);
  assert.equal(sampleField(f, -1, 0), 0);
});

/**
 * Map a built-in world from a few known poses, as a reference map.
 */
function referenceSetup() {
  const world = BUILTIN_MAPS.office();
  const lidar = { beams: 180, fov: 2 * Math.PI, maxRange: 8, rangeNoise: 0, dropout: 0 };
  const grid = createGrid(world.width + 2, world.height + 2, 0.05, -1, -1);
  const rng = createRng(1);
  for (const p of [
    { x: 2, y: 7, theta: 0 },
    { x: 6, y: 7, theta: 0.3 },
    { x: 9, y: 6.5, theta: 1 },
  ]) {
    integrateScan(grid, p, simulateScan(world.walls, p, lidar, rng));
  }
  const field = likelihoodField(occupiedMask(grid, 0.5), grid, 0.1);
  return { world, lidar, field };
}

test("scan matching recovers a pose offset by 0.15 m, -0.1 m and 0.05 rad", () => {
  const { world, field } = referenceSetup();
  const truth = { x: 5, y: 7.2, theta: 0.2 };
  const lidar = { beams: 180, fov: 2 * Math.PI, maxRange: 8, rangeNoise: 0.01, dropout: 0.02 };
  const scan = simulateScan(world.walls, truth, lidar, createRng(9));
  const guess = { x: truth.x + 0.15, y: truth.y - 0.1, theta: truth.theta + 0.05 };
  const res = matchScan(field, scan, guess);
  assert.equal(res.accepted, true, res.reason);
  assert.ok(res.score > res.guessScore);
  // Within one 0.05 m grid cell (walls are quantised to cells, so a bias of
  // up to half a cell is expected) and about half a degree.
  assert.ok(
    Math.hypot(res.pose.x - truth.x, res.pose.y - truth.y) < 0.05,
    JSON.stringify(res.pose),
  );
  assert.ok(Math.abs(res.pose.theta - truth.theta) < 0.01, `theta ${res.pose.theta}`);
  assert.ok(res.evaluations > 100);
});

test("a correct guess stays put and scores higher than a wrong one", () => {
  const { world, lidar, field } = referenceSetup();
  const truth = { x: 4, y: 6.8, theta: -0.4 };
  const scan = simulateScan(world.walls, truth, lidar, createRng(4));
  const res = matchScan(field, scan, truth);
  assert.ok(
    Math.hypot(res.pose.x - truth.x, res.pose.y - truth.y) < 0.05,
    JSON.stringify(res.pose),
  );
  assert.ok(scoreScan(field, scan, truth) > scoreScan(field, scan, { ...truth, x: truth.x + 0.3 }));
});

test("matching declines with too few beams, an empty map, or a poor fit", () => {
  const { world, field } = referenceSetup();
  const pose = { x: 4, y: 7, theta: 0 };
  const sparse = simulateScan(
    world.walls,
    pose,
    { beams: 10, fov: 2 * Math.PI, maxRange: 8, rangeNoise: 0, dropout: 0 },
    createRng(1),
  );
  assert.equal(matchScan(field, sparse, pose).reason, "too few valid beams");
  const full = simulateScan(
    world.walls,
    pose,
    { beams: 90, fov: 2 * Math.PI, maxRange: 8, rangeNoise: 0, dropout: 0 },
    createRng(1),
  );
  const empty = likelihoodField(new Uint8Array(field.values.length), field, 0.1);
  assert.equal(matchScan(empty, full, pose).reason, "map is empty");
  // A pose in the far corner of the map, against a map seen from elsewhere.
  const res = matchScan(field, full, { x: 14, y: 1, theta: 2 }, { minScore: 0.99 });
  assert.equal(res.accepted, false);
  assert.equal(res.reason, "best score below threshold");
  assert.deepEqual(res.pose, { x: 14, y: 1, theta: 2 });
  assert.equal(scoreScan(field, { ...full, ranges: full.ranges.map(() => NaN) }, pose), 0);
});

test("rasterized walls feed the distance transform", () => {
  const geom = { resolution: 0.5, width: 4, height: 4, originX: 0, originY: 0 };
  const m = rasterizeWalls(geom, [[0.25, 0.25, 1.75, 0.25]]);
  assert.deepEqual(Array.from(m.slice(0, 4)), [1, 1, 1, 1]);
  assert.equal(
    m.slice(4).reduce((a, b) => a + b, 0),
    0,
  );
});
