import test from "node:test";
import assert from "node:assert/strict";
import { close } from "./helpers.js";
import {
  castRay,
  localToWorld,
  nearestWallIndex,
  normalizeAngle,
  pointSegmentDistance,
  raySegmentDistance,
  segmentsIntersect,
} from "../src/geometry.js";
import { createRng } from "../src/rng.js";

test("normalizeAngle wraps into (-pi, pi]", () => {
  close(normalizeAngle(0), 0);
  close(normalizeAngle(3 * Math.PI), Math.PI);
  close(normalizeAngle(-Math.PI), Math.PI);
  close(normalizeAngle(Math.PI / 2 + 4 * Math.PI), Math.PI / 2);
  close(normalizeAngle((-3 * Math.PI) / 2), Math.PI / 2);
});

test("ray hits a vertical wall at the expected distance", () => {
  // Ray from origin along +x, wall x = 3 from y = -1 to 1: distance 3.
  close(raySegmentDistance(0, 0, 1, 0, [3, -1, 3, 1]), 3);
  // At 45 degrees towards wall x = 2 (y from 0 to 5): hit at (2, 2), distance 2*sqrt(2).
  close(castRay([[2, 0, 2, 5]], 0, 0, Math.PI / 4, 10), 2 * Math.SQRT2);
});

test("ray misses: behind, past the end, parallel, beyond max range", () => {
  assert.equal(raySegmentDistance(0, 0, 1, 0, [-3, -1, -3, 1]), Infinity);
  assert.equal(raySegmentDistance(0, 0, 1, 0, [3, 0.5, 3, 1]), Infinity);
  assert.equal(raySegmentDistance(0, 0, 1, 0, [0, 1, 5, 1]), Infinity);
  assert.equal(castRay([[3, -1, 3, 1]], 0, 0, 0, 2.5), Infinity);
});

test("castRay returns the nearest of several walls", () => {
  const walls = /** @type {import("../src/geometry.js").Segment[]} */ ([
    [5, -1, 5, 1],
    [2, -1, 2, 1],
    [8, -1, 8, 1],
  ]);
  close(castRay(walls, 0, 0, 0, 10), 2);
});

test("pointSegmentDistance: perpendicular foot, and clamped to the ends", () => {
  close(pointSegmentDistance(1, 2, [0, 0, 4, 0]), 2);
  close(pointSegmentDistance(7, 4, [0, 0, 4, 0]), 5); // 3-4-5 from the end (4, 0)
  close(pointSegmentDistance(1, 1, [0, 0, 0, 0]), Math.SQRT2); // degenerate segment
});

test("segmentsIntersect: crossing, touching, collinear overlap, disjoint", () => {
  assert.equal(segmentsIntersect([0, 0, 2, 2], [0, 2, 2, 0]), true);
  assert.equal(segmentsIntersect([0, 0, 1, 0], [1, 0, 1, 1]), true);
  assert.equal(segmentsIntersect([0, 0, 2, 0], [1, 0, 3, 0]), true);
  assert.equal(segmentsIntersect([0, 0, 1, 0], [2, 0, 3, 0]), false);
  assert.equal(segmentsIntersect([0, 0, 1, 1], [0, 1, 0.4, 0.6]), false);
  assert.equal(segmentsIntersect([0, 1, 0.5, 0.5], [0, 0, 1, 1]), true);
  // Touching at the second segment's end point, and at the first one's.
  assert.equal(segmentsIntersect([0, 0, 2, 0], [3, 3, 1, 0]), true);
  assert.equal(segmentsIntersect([1, 0, 1, 2], [0, 0, 2, 0]), true);
});

test("nearestWallIndex picks the closest wall within reach", () => {
  const walls = /** @type {import("../src/geometry.js").Segment[]} */ ([
    [0, 0, 10, 0],
    [0, 1, 10, 1],
  ]);
  assert.equal(nearestWallIndex(walls, 5, 0.8, 0.5), 1);
  assert.equal(nearestWallIndex(walls, 5, 0.3, 0.5), 0);
  assert.equal(nearestWallIndex(walls, 5, 3, 0.5), -1);
});

test("localToWorld rotates then translates", () => {
  const [x, y] = localToWorld({ x: 1, y: 2, theta: Math.PI / 2 }, 1, 0);
  close(x, 1);
  close(y, 3);
});

test("rng is deterministic and gaussian has roughly unit variance", () => {
  const a = createRng(42);
  const b = createRng(42);
  for (let i = 0; i < 10; i++) assert.equal(a.next(), b.next());
  const r = createRng(7);
  let sum = 0;
  let sumSq = 0;
  const n = 20000;
  for (let i = 0; i < n; i++) {
    const g = r.gaussian();
    sum += g;
    sumSq += g * g;
  }
  const mean = sum / n;
  const variance = sumSq / n - mean * mean;
  // Standard error of the mean is 1/sqrt(20000) ~= 0.007; allow 5 sigma.
  assert.ok(Math.abs(mean) < 0.035, `mean ${mean}`);
  assert.ok(Math.abs(variance - 1) < 0.05, `variance ${variance}`);
  for (let i = 0; i < 1000; i++) {
    const u = r.next();
    assert.ok(u >= 0 && u < 1);
  }
});
