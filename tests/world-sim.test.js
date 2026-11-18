import test from "node:test";
import assert from "node:assert/strict";
import { close } from "./helpers.js";
import { collides, parseWorld, serializeWorld, boundaryWalls } from "../src/world.js";
import { BUILTIN_MAPS, isBuiltinMap } from "../src/maps.js";
import { followRoute, steerTo } from "../src/autopilot.js";
import { headingError, mapPrecision, positionError, trajectoryRmse } from "../src/metrics.js";
import { createGrid } from "../src/grid.js";
import { pointSegmentDistance } from "../src/geometry.js";
import {
  MAX_TRAJECTORY_POINTS,
  createSimulation,
  defaultSettings,
  initialScan,
  step,
  summarize,
} from "../src/sim.js";

const minimal = { version: 1, width: 4, height: 3, walls: [[0, 0, 4, 0]], start: { x: 1, y: 1 } };

test("parseWorld accepts a minimal map and fills defaults", () => {
  const w = parseWorld(JSON.stringify(minimal));
  assert.equal(w.name, "Untitled");
  assert.equal(w.start.theta, 0);
  assert.deepEqual(w.route, []);
  assert.deepEqual(w.walls, [[0, 0, 4, 0]]);
});

test("parseWorld rejects malformed maps with a message naming the field", () => {
  /** @type {Array<[unknown, RegExp]>} */
  const cases = [
    ["{", /not valid JSON/],
    ["[]", /JSON object/],
    [{ ...minimal, version: 2 }, /version/],
    [{ ...minimal, width: "4" }, /width must be a number/],
    [{ ...minimal, width: 1000 }, /width and height/],
    [{ ...minimal, walls: {} }, /walls must be an array/],
    [{ ...minimal, walls: [[0, 0, 1]] }, /walls\[0\]/],
    [{ ...minimal, walls: [[0, 0, 1, Infinity]] }, /walls\[0\]\[3\]/],
    [{ ...minimal, walls: [[0, 0, 9, 0]] }, /outside the map/],
    [{ ...minimal, walls: [[1, 1, 1, 1]] }, /zero length/],
    [{ ...minimal, walls: new Array(2001).fill([0, 0, 1, 1]) }, /at most/],
    [{ ...minimal, start: null }, /start is required/],
    [{ ...minimal, start: { x: 5, y: 1 } }, /inside the map/],
    [{ ...minimal, route: 3 }, /route must be/],
    [{ ...minimal, route: [[1]] }, /route\[0\]/],
    [{ ...minimal, route: new Array(501).fill([1, 1]) }, /too long/],
  ];
  for (const [input, re] of cases) assert.throws(() => parseWorld(input), re);
});

test("every built-in map round-trips through JSON", () => {
  for (const id of Object.keys(BUILTIN_MAPS)) {
    assert.ok(isBuiltinMap(id));
    const world = BUILTIN_MAPS[/** @type {keyof typeof BUILTIN_MAPS} */ (id)]();
    assert.deepEqual(parseWorld(serializeWorld(world)), world);
  }
  assert.equal(isBuiltinMap("toString"), false);
});

test("built-in routes keep the robot clear of walls at every waypoint", () => {
  for (const make of Object.values(BUILTIN_MAPS)) {
    const world = make();
    for (const [x, y] of [[world.start.x, world.start.y], ...world.route]) {
      const d = Math.min(...world.walls.map((w) => pointSegmentDistance(x, y, w)));
      assert.ok(d > 0.45, `${world.name}: waypoint (${x}, ${y}) only ${d} m from a wall`);
    }
  }
});

test("the autopilot completes every built-in route without touching a wall", () => {
  for (const make of Object.values(BUILTIN_MAPS)) {
    const world = make();
    const sim = createSimulation(world, {
      ...defaultSettings(),
      scanMatching: false,
      lidar: { beams: 8, fov: 2 * Math.PI, maxRange: 8, rangeNoise: 0, dropout: 0 },
    });
    let idx = 0;
    let done = false;
    for (let k = 0; k < 5000 && !done; k++) {
      const cmd = followRoute(sim.truePose, world.route, idx);
      idx = cmd.index;
      done = cmd.done;
      if (!done) step(sim, cmd.v, cmd.w);
    }
    assert.ok(done, `${world.name}: route not finished`);
    assert.equal(sim.collisions, 0, world.name);
  }
});

test("collision: crossing a wall, ending too close, backing away is allowed", () => {
  const walls = boundaryWalls(4, 4);
  assert.equal(collides(walls, 2, 2, 5, 2, 0.2), true);
  assert.equal(collides(walls, 2, 2, 3.9, 2, 0.2), true);
  assert.equal(collides(walls, 2, 2, 3, 2, 0.2), false);
  assert.equal(collides(walls, 3.9, 2, 3.85, 2, 0.2), false);
});

test("steerTo: straight ahead drives, sideways turns in place", () => {
  const ahead = steerTo({ x: 0, y: 0, theta: 0 }, 5, 0);
  close(ahead.v, 0.8);
  close(ahead.w, 0);
  const left = steerTo({ x: 0, y: 0, theta: 0 }, 0, 5);
  close(left.v, 0, 1e-12);
  close(left.w, 1.5);
  const nearby = steerTo({ x: 0, y: 0, theta: 0 }, 0.3, 0);
  close(nearby.v, 0.3);
});

test("followRoute skips reached waypoints and reports completion", () => {
  const route = /** @type {Array<[number, number]>} */ ([
    [0.1, 0],
    [3, 0],
  ]);
  const r = followRoute({ x: 0, y: 0, theta: 0 }, route, 0);
  assert.equal(r.index, 1);
  assert.equal(r.done, false);
  const done = followRoute({ x: 3, y: 0.1, theta: 0 }, route, 1);
  assert.deepEqual(done, { v: 0, w: 0, index: 2, done: true });
});

test("metrics: position, heading and RMS errors", () => {
  close(positionError({ x: 0, y: 0, theta: 0 }, { x: 3, y: 4, theta: 0 }), 5);
  close(headingError({ x: 0, y: 0, theta: 3 }, { x: 0, y: 0, theta: -3 }), 2 * Math.PI - 6);
  const t = [
    { x: 0, y: 0, theta: 0 },
    { x: 1, y: 0, theta: 0 },
  ];
  const e = [
    { x: 0, y: 3, theta: 0 },
    { x: 1, y: 4, theta: 0 },
  ];
  close(trajectoryRmse(t, e), Math.sqrt((9 + 16) / 2));
  assert.equal(trajectoryRmse([], []), 0);
  assert.throws(() => trajectoryRmse(t, []), /length/);
});

test("mapPrecision counts occupied cells near true walls", () => {
  const g = createGrid(2, 1, 0.1);
  // Wall along y = 0.05 (row 0). Mark two cells on it and one 0.5 m away.
  g.logOdds[3] = 3;
  g.logOdds[4] = 3;
  g.logOdds[5 * g.width + 3] = 3;
  const s = mapPrecision(g, [[0, 0.05, 2, 0.05]], 0.1);
  assert.equal(s.occupiedCells, 3);
  close(s.precision, 2 / 3);
  assert.ok(Number.isNaN(mapPrecision(createGrid(1, 1, 0.1), [[0, 0.05, 1, 0.05]], 0.1).precision));
});

/**
 * Drive the office demo route for a fixed number of steps.
 * @param {Partial<import("../src/sim.js").SimSettings>} overrides
 * @param {number} steps
 */
function runOffice(overrides, steps) {
  const world = BUILTIN_MAPS.office();
  const sim = createSimulation(world, { ...defaultSettings(), ...overrides });
  initialScan(sim);
  let idx = 0;
  for (let k = 0; k < steps; k++) {
    const cmd = followRoute(sim.truePose, world.route, idx);
    idx = cmd.index;
    step(sim, cmd.v, cmd.w);
  }
  return sim;
}

test("without noise the estimate equals the truth and the map is sharp", () => {
  const sim = runOffice(
    {
      scanMatching: false,
      odometry: {
        rotFromRot: 0,
        rotFromTrans: 0,
        transFromTrans: 0,
        transFromRot: 0,
        headingBias: 0,
      },
      lidar: { beams: 180, fov: 2 * Math.PI, maxRange: 8, rangeNoise: 0, dropout: 0 },
    },
    300,
  );
  const s = summarize(sim);
  assert.ok(s.estError < 1e-9);
  assert.ok(s.odomRms < 1e-9);
  assert.ok(s.distance > 10, `drove ${s.distance} m`);
  assert.ok(Number.isNaN(s.acceptRate));
  assert.equal(s.collisions, 0);
  assert.ok(mapPrecision(sim.grid, sim.world.walls, 0.1).precision > 0.99);
});

test("scan matching keeps the estimate far closer to the truth than odometry", () => {
  // Same seed, so both runs see identical odometry and LiDAR noise.
  const off = summarize(runOffice({ scanMatching: false, seed: 3 }, 600));
  const on = summarize(runOffice({ scanMatching: true, seed: 3 }, 600));
  close(on.odomRms, off.odomRms); // the dead-reckoning path is identical
  assert.ok(off.estRms > 0.2, `odometry RMS ${off.estRms}`);
  assert.ok(on.estRms < off.estRms / 3, `matched ${on.estRms} vs odometry ${off.estRms}`);
  assert.ok(on.acceptRate > 0.9);
});

test("runs are reproducible for a given seed", () => {
  const a = runOffice({ seed: 8 }, 120);
  const b = runOffice({ seed: 8 }, 120);
  assert.deepEqual(a.estPose, b.estPose);
  assert.deepEqual(Array.from(a.grid.logOdds), Array.from(b.grid.logOdds));
});

test("driving into a wall is blocked and counted", () => {
  const world = parseWorld({ ...minimal, walls: boundaryWalls(4, 3) });
  const sim = createSimulation(world, defaultSettings());
  for (let k = 0; k < 60; k++) step(sim, 1, 0);
  assert.ok(sim.truePose.x <= 4 - 0.2 + 1e-9);
  assert.ok(sim.collisions > 0);
});

test("trails are thinned instead of growing without bound", () => {
  const world = parseWorld({ ...minimal, walls: boundaryWalls(4, 3) });
  const sim = createSimulation(world, {
    ...defaultSettings(),
    lidar: { beams: 8, fov: 2 * Math.PI, maxRange: 8, rangeNoise: 0, dropout: 0 },
    scanMatching: false,
  });
  for (let k = 0; k < MAX_TRAJECTORY_POINTS + 10; k++) step(sim, 0, 0.1);
  assert.ok(sim.trueTrail.length < MAX_TRAJECTORY_POINTS);
  assert.equal(sim.errorSamples, MAX_TRAJECTORY_POINTS + 10);
});
