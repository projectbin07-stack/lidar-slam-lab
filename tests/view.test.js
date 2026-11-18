import test from "node:test";
import assert from "node:assert/strict";
import { close } from "./helpers.js";
import { fitViewport, snap, toCanvas, toWorld } from "../src/view.js";

test("a 10 x 5 m world fits a 220 x 220 canvas with 10 px padding", () => {
  // Width limits: (220 - 20) / 10 = 20 px per metre; the 100 px tall image is
  // centred vertically, so y = 0 sits at 110 + 50 = 160.
  const v = fitViewport(0, 0, 10, 5, 220, 220, 10);
  close(v.scale, 20);
  assert.deepEqual(toCanvas(v, 0, 0), [10, 160]);
  assert.deepEqual(toCanvas(v, 10, 5), [210, 60]);
  const [x, y] = toWorld(v, 110, 110);
  close(x, 5);
  close(y, 2.5);
});

test("a shifted origin maps to the same place", () => {
  const v = fitViewport(-2, -1, 4, 2, 400, 200, 0);
  const [px, py] = toCanvas(v, -2, -1);
  close(px, 0);
  close(py, 200);
});

test("snap rounds to the step and clamps to the map", () => {
  assert.equal(snap(1.234, 0.1, 10), 1.2);
  assert.equal(snap(-0.3, 0.1, 10), 0);
  assert.equal(snap(12, 0.1, 10), 10);
  assert.equal(snap(0.35, 0.5, 10), 0.5);
});
