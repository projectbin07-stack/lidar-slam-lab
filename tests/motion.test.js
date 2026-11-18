import test from "node:test";
import assert from "node:assert/strict";
import { close } from "./helpers.js";
import {
  applyOdometryDelta,
  corruptOdometry,
  integrateUnicycle,
  odometryDelta,
} from "../src/motion.js";
import { createRng } from "../src/rng.js";

const ZERO_NOISE = {
  rotFromRot: 0,
  rotFromTrans: 0,
  transFromTrans: 0,
  transFromRot: 0,
  headingBias: 0,
};

test("straight-line motion", () => {
  const p = integrateUnicycle({ x: 1, y: 1, theta: Math.PI / 2 }, 2, 0, 0.5);
  close(p.x, 1);
  close(p.y, 2);
  close(p.theta, Math.PI / 2);
});

test("quarter circle: v = 1, w = pi/2, dt = 1 ends at (2/pi, 2/pi)", () => {
  // Radius r = v / w = 2/pi; a quarter turn about the centre (0, r).
  const p = integrateUnicycle({ x: 0, y: 0, theta: 0 }, 1, Math.PI / 2, 1);
  close(p.x, 2 / Math.PI);
  close(p.y, 2 / Math.PI);
  close(p.theta, Math.PI / 2);
});

test("odometryDelta decomposes (0,0,0) -> (1,1,pi/2) as pi/4, sqrt2, pi/4", () => {
  const d = odometryDelta({ x: 0, y: 0, theta: 0 }, { x: 1, y: 1, theta: Math.PI / 2 });
  close(d.rot1, Math.PI / 4);
  close(d.trans, Math.SQRT2);
  close(d.rot2, Math.PI / 4);
});

test("pure rotation has zero rot1 and trans", () => {
  const d = odometryDelta({ x: 2, y: 3, theta: 0.5 }, { x: 2, y: 3, theta: -0.2 });
  assert.equal(d.rot1, 0);
  assert.equal(d.trans, 0);
  close(d.rot2, -0.7);
});

test("applyOdometryDelta inverts odometryDelta from any start pose", () => {
  const a = { x: -1.5, y: 4, theta: 2.8 };
  const b = { x: 0.3, y: 2.2, theta: -2.9 };
  const d = odometryDelta(a, b);
  const c = applyOdometryDelta(a, d);
  close(c.x, b.x);
  close(c.y, b.y);
  close(c.theta, b.theta);
});

test("the same relative motion can be replayed from a different frame", () => {
  // Driving 1 m forward from a pose facing +y moves along +y.
  const d = odometryDelta({ x: 0, y: 0, theta: 0 }, { x: 1, y: 0, theta: 0 });
  const p = applyOdometryDelta({ x: 5, y: 5, theta: Math.PI / 2 }, d);
  close(p.x, 5);
  close(p.y, 6);
});

test("zero noise leaves the motion unchanged, the bias adds rotation per metre", () => {
  const truth = { rot1: 0.1, trans: 2, rot2: -0.05 };
  const exact = corruptOdometry(truth, ZERO_NOISE, createRng(1));
  assert.deepEqual(exact, truth);
  const biased = corruptOdometry(truth, { ...ZERO_NOISE, headingBias: 0.01 }, createRng(1));
  close(biased.rot2, -0.05 + 0.02);
});

test("no motion produces no odometry noise", () => {
  const d = corruptOdometry(
    { rot1: 0, trans: 0, rot2: 0 },
    { rotFromRot: 1, rotFromTrans: 1, transFromTrans: 1, transFromRot: 1, headingBias: 1 },
    createRng(3),
  );
  assert.deepEqual(d, { rot1: 0, trans: 0, rot2: 0 });
});

test("translation noise has the configured standard deviation", () => {
  const rng = createRng(11);
  const noise = { ...ZERO_NOISE, transFromTrans: 0.1 };
  const n = 5000;
  let sumSq = 0;
  for (let i = 0; i < n; i++) {
    const d = corruptOdometry({ rot1: 0, trans: 1, rot2: 0 }, noise, rng);
    sumSq += (d.trans - 1) ** 2;
  }
  // Expected std 0.1 * 1 m; the estimate's relative standard error is ~1%.
  close(Math.sqrt(sumSq / n), 0.1, 0.005);
});
