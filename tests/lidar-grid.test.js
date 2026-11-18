import test from "node:test";
import assert from "node:assert/strict";
import { close } from "./helpers.js";
import { beamAngles, scanEndpoints, simulateScan } from "../src/lidar.js";
import {
  createGrid,
  inGrid,
  integrateScan,
  logOddsToProbability,
  occupiedMask,
  traceLine,
  worldToCell,
} from "../src/grid.js";
import { createRng } from "../src/rng.js";
import { boundaryWalls } from "../src/world.js";

const quiet = { beams: 4, fov: 2 * Math.PI, maxRange: 10, rangeNoise: 0, dropout: 0 };

test("beam angles: full circle without a duplicate, partial fan symmetric", () => {
  assert.deepEqual(Array.from(beamAngles(4, 2 * Math.PI)), [
    0,
    Math.PI / 2,
    Math.PI,
    (3 * Math.PI) / 2,
  ]);
  const fan = beamAngles(3, Math.PI);
  close(fan[0], -Math.PI / 2);
  close(fan[1], 0);
  close(fan[2], Math.PI / 2);
  assert.throws(() => beamAngles(1, Math.PI), RangeError);
  assert.throws(() => beamAngles(10, 7), RangeError);
});

test("noise-free scan in a 4 x 2 room from (1, 0.5)", () => {
  const walls = boundaryWalls(4, 2);
  const scan = simulateScan(walls, { x: 1, y: 0.5, theta: 0 }, quiet, createRng(1));
  // East wall 3 m away, north 1.5 m, west 1 m, south 0.5 m.
  close(scan.ranges[0], 3);
  close(scan.ranges[1], 1.5);
  close(scan.ranges[2], 1);
  close(scan.ranges[3], 0.5);
  const pts = scanEndpoints(scan, { x: 1, y: 0.5, theta: 0 });
  close(pts[0], 4);
  close(pts[1], 0.5);
});

test("out of range and dropout give NaN; endpoints skip them", () => {
  const walls = boundaryWalls(4, 2);
  const short = simulateScan(
    walls,
    { x: 1, y: 0.5, theta: 0 },
    { ...quiet, maxRange: 1.2 },
    createRng(1),
  );
  assert.ok(Number.isNaN(short.ranges[0]));
  assert.ok(Number.isNaN(short.ranges[1]));
  close(short.ranges[2], 1);
  assert.equal(scanEndpoints(short, { x: 1, y: 0.5, theta: 0 }).length, 4);
  const allDropped = simulateScan(
    walls,
    { x: 1, y: 0.5, theta: 0 },
    { ...quiet, dropout: 1 },
    createRng(1),
  );
  assert.ok(Array.from(allDropped.ranges).every(Number.isNaN));
});

test("range noise has the configured standard deviation", () => {
  const walls = boundaryWalls(10, 10);
  const cfg = { beams: 360, fov: 2 * Math.PI, maxRange: 20, rangeNoise: 0.05, dropout: 0 };
  const pose = { x: 5, y: 5, theta: 0 };
  const exact = simulateScan(walls, pose, { ...cfg, rangeNoise: 0 }, createRng(1));
  const rng = createRng(2);
  let sumSq = 0;
  let n = 0;
  for (let k = 0; k < 20; k++) {
    const s = simulateScan(walls, pose, cfg, rng);
    for (let i = 0; i < s.ranges.length; i++) {
      sumSq += (s.ranges[i] - exact.ranges[i]) ** 2;
      n++;
    }
  }
  close(Math.sqrt(sumSq / n), 0.05, 0.002);
});

test("Bresenham line (0,0) -> (3,1) visits the four textbook cells", () => {
  /** @type {Array<[number, number]>} */
  const cells = [];
  traceLine(0, 0, 3, 1, (i, j) => {
    cells.push([i, j]);
  });
  assert.deepEqual(cells, [
    [0, 0],
    [1, 0],
    [2, 1],
    [3, 1],
  ]);
  /** @type {Array<[number, number]>} */
  const back = [];
  traceLine(2, 2, 2, -1, (i, j) => {
    back.push([i, j]);
  });
  assert.deepEqual(back, [
    [2, 2],
    [2, 1],
    [2, 0],
    [2, -1],
  ]);
  let visits = 0;
  traceLine(0, 0, 10, 0, () => ++visits < 3);
  assert.equal(visits, 3);
});

test("grid geometry and probability conversion", () => {
  const g = createGrid(2, 1, 0.1, -1, 0);
  assert.equal(g.width, 20);
  assert.equal(g.height, 10);
  assert.deepEqual(worldToCell(g, -1, 0), [0, 0]);
  assert.deepEqual(worldToCell(g, 0.05, 0.95), [10, 9]);
  assert.equal(inGrid(g, 20, 0), false);
  assert.equal(inGrid(g, -1, 0), false);
  close(logOddsToProbability(0), 0.5);
  close(logOddsToProbability(Math.log(3)), 0.75);
  assert.throws(() => createGrid(1, 1, 0), RangeError);
  assert.throws(() => createGrid(0, 1, 0.1), RangeError);
});

test("one beam: free cells along the ray, one hit at the end, clamped on repeat", () => {
  const g = createGrid(1, 0.1, 0.1);
  const scan = {
    angles: new Float64Array([0]),
    ranges: new Float64Array([0.5]),
    maxRange: 1,
  };
  // From the centre of cell 0 a 0.5 m beam ends in the centre of cell 5.
  integrateScan(g, { x: 0.05, y: 0.05, theta: 0 }, scan);
  for (let i = 0; i < 5; i++) close(g.logOdds[i], -0.4, 1e-6);
  close(g.logOdds[5], 0.85, 1e-6);
  assert.equal(g.logOdds[6], 0);
  for (let k = 0; k < 20; k++) integrateScan(g, { x: 0.05, y: 0.05, theta: 0 }, scan);
  close(g.logOdds[5], 4, 1e-6);
  close(g.logOdds[0], -4, 1e-6);
  assert.deepEqual(Array.from(occupiedMask(g)).slice(0, 7), [0, 0, 0, 0, 0, 1, 0]);
});

test("beams leaving the grid and dropped beams are ignored safely", () => {
  const g = createGrid(1, 1, 0.1);
  const scan = {
    angles: new Float64Array([0, Math.PI]),
    ranges: new Float64Array([5, NaN]),
    maxRange: 10,
  };
  integrateScan(g, { x: 0.55, y: 0.55, theta: 0 }, scan);
  const row = Array.from(g.logOdds.slice(50, 60));
  assert.deepEqual(row.slice(0, 5), [0, 0, 0, 0, 0]);
  for (const v of row.slice(5)) close(v, -0.4, 1e-6);
});
