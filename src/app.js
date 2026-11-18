// Browser front end. All simulation logic lives in the other modules; this
// file wires them to the DOM, draws the two views and handles input.

import { BUILTIN_MAPS, isBuiltinMap } from "./maps.js";
import { boundaryWalls, parseWorld, serializeWorld } from "./world.js";
import { createSimulation, defaultSettings, initialScan, step, summarize } from "./sim.js";
import { DEFAULT_ODOMETRY_NOISE } from "./motion.js";
import { followRoute } from "./autopilot.js";
import { scanEndpoints } from "./lidar.js";
import { logOddsToProbability } from "./grid.js";
import { mapPrecision } from "./metrics.js";
import { gridToPgm, trajectoriesCsv } from "./export.js";
import { randomFreePose } from "./mcl.js";
import { createRng } from "./rng.js";
import { nearestWallIndex } from "./geometry.js";
import { fitViewport, snap, toCanvas, toWorld } from "./view.js";

/** @typedef {import("./world.js").World} World */
/** @typedef {import("./sim.js").Simulation} Simulation */
/** @typedef {import("./geometry.js").Pose} Pose */
/** @typedef {import("./view.js").Viewport} Viewport */
/** @typedef {"drive" | "draw" | "erase" | "start"} Mode */

/**
 * @template {Element} T
 * @param {string} id
 * @param {{ new (): T, prototype: T }} type
 * @returns {T}
 */
function el(id, type) {
  const e = document.getElementById(id);
  if (!(e instanceof type)) throw new Error(`#${id} is missing or not a ${type.name}`);
  return e;
}

const ui = {
  truth: el("truth", HTMLCanvasElement),
  estimate: el("estimate", HTMLCanvasElement),
  play: el("play", HTMLButtonElement),
  reset: el("reset", HTMLButtonElement),
  autopilot: el("autopilot", HTMLButtonElement),
  kidnap: el("kidnap", HTMLButtonElement),
  speed: el("speed", HTMLSelectElement),
  seed: el("seed", HTMLInputElement),
  scanmatch: el("scanmatch", HTMLInputElement),
  particles: el("particles", HTMLSelectElement),
  global: el("global", HTMLInputElement),
  showParticles: el("show-particles", HTMLInputElement),
  showBeams: el("show-beams", HTMLInputElement),
  odom: el("odom", HTMLInputElement),
  rnoise: el("rnoise", HTMLInputElement),
  dropout: el("dropout", HTMLInputElement),
  beams: el("beams", HTMLInputElement),
  range: el("range", HTMLInputElement),
  map: el("map", HTMLSelectElement),
  saveMap: el("save-map", HTMLButtonElement),
  loadMap: el("load-map", HTMLInputElement),
  savePgm: el("save-pgm", HTMLButtonElement),
  saveCsv: el("save-csv", HTMLButtonElement),
  status: el("status", HTMLParagraphElement),
  hint: el("truth-hint", HTMLSpanElement),
};

const MAX_STEPS_PER_FRAME = 8;

const state = {
  /** @type {World} */
  world: BUILTIN_MAPS.office(),
  /** @type {Simulation} */
  sim: /** @type {Simulation} */ (/** @type {unknown} */ (null)),
  running: true,
  autopilot: false,
  routeIndex: 0,
  /** @type {Mode} */
  mode: "drive",
  /** @type {Set<string>} */
  keys: new Set(),
  /** @type {Set<string>} */
  pad: new Set(),
  /** @type {[number, number] | null} */
  draftStart: null,
  /** @type {[number, number] | null} */
  draftEnd: null,
  accumulator: 0,
  lastFrame: 0,
  stepCostMs: NaN,
  precision: NaN,
  precisionAt: -1,
  kidnaps: 0,
};

// ---------------------------------------------------------------- settings

function readSettings() {
  const s = defaultSettings();
  s.seed = clampInt(Number(ui.seed.value), 0, 4294967295, 1);
  s.scanMatching = ui.scanmatch.checked;
  s.mclParticles = Number(ui.particles.value);
  s.mclGlobal = ui.global.checked;
  applyLiveSettings(s);
  return s;
}

/**
 * Settings that can change while the simulation runs.
 * @param {import("./sim.js").SimSettings} s
 */
function applyLiveSettings(s) {
  const k = Number(ui.odom.value);
  for (const key of /** @type {Array<keyof typeof DEFAULT_ODOMETRY_NOISE>} */ (
    Object.keys(DEFAULT_ODOMETRY_NOISE)
  )) {
    s.odometry[key] = DEFAULT_ODOMETRY_NOISE[key] * k;
  }
  s.lidar.rangeNoise = Number(ui.rnoise.value);
  s.lidar.dropout = Number(ui.dropout.value);
  s.lidar.beams = Number(ui.beams.value);
  s.lidar.maxRange = Number(ui.range.value);
  s.scanMatching = ui.scanmatch.checked;
  el("odom-out", HTMLOutputElement).value = `${k.toFixed(2)}x`;
  el("rnoise-out", HTMLOutputElement).value = `${s.lidar.rangeNoise.toFixed(2)} m`;
  el("dropout-out", HTMLOutputElement).value = `${Math.round(s.lidar.dropout * 100)} %`;
  el("beams-out", HTMLOutputElement).value = String(s.lidar.beams);
  el("range-out", HTMLOutputElement).value = `${s.lidar.maxRange} m`;
}

/**
 * @param {number} v
 * @param {number} min
 * @param {number} max
 * @param {number} fallback
 */
function clampInt(v, min, max, fallback) {
  return Number.isInteger(v) && v >= min && v <= max ? v : fallback;
}

function resetSimulation() {
  state.sim = createSimulation(state.world, readSettings());
  initialScan(state.sim);
  state.routeIndex = 0;
  state.accumulator = 0;
  state.precision = NaN;
  state.precisionAt = -1;
  state.kidnaps = 0;
  ui.autopilot.disabled = state.world.route.length === 0;
  if (state.world.route.length === 0) setAutopilot(false);
}

/** @param {World} world */
function setWorld(world) {
  state.world = world;
  resetSimulation();
}

/** @param {boolean} on */
function setAutopilot(on) {
  state.autopilot = on;
  ui.autopilot.setAttribute("aria-pressed", String(on));
}

/** @param {boolean} on */
function setRunning(on) {
  state.running = on;
  ui.play.textContent = on ? "Pause" : "Run";
}

/**
 * @param {string} text
 * @param {boolean} [isError]
 */
function setStatus(text, isError = false) {
  ui.status.textContent = text;
  ui.status.classList.toggle("error", isError);
}

function emptyWorld() {
  return {
    name: "Empty room",
    width: 12,
    height: 9,
    walls: boundaryWalls(12, 9),
    start: { x: 1.5, y: 1.5, theta: 0 },
    route: /** @type {Array<[number, number]>} */ ([]),
  };
}

// ---------------------------------------------------------------- stepping

/** @returns {{ v: number, w: number }} */
function manualCommand() {
  const has = (/** @type {string[]} */ ...names) =>
    names.some((n) => state.keys.has(n) || state.pad.has(n));
  let v = 0;
  let w = 0;
  if (has("ArrowUp", "w", "fwd")) v += 0.8;
  if (has("ArrowDown", "s", "back")) v -= 0.4;
  if (has("ArrowLeft", "a", "left")) w += 1.5;
  if (has("ArrowRight", "d", "right")) w -= 1.5;
  return { v, w };
}

function stepOnce() {
  const sim = state.sim;
  let cmd = manualCommand();
  if (state.autopilot && cmd.v === 0 && cmd.w === 0) {
    const r = followRoute(sim.truePose, state.world.route, state.routeIndex);
    state.routeIndex = r.index;
    if (r.done) {
      setAutopilot(false);
      setStatus("Demo route finished.");
    }
    cmd = r;
  }
  const t0 = performance.now();
  step(sim, cmd.v, cmd.w);
  const cost = performance.now() - t0;
  state.stepCostMs = Number.isNaN(state.stepCostMs) ? cost : 0.9 * state.stepCostMs + 0.1 * cost;
}

/** @param {number} now */
function frame(now) {
  const dt = Math.min(0.25, (now - (state.lastFrame || now)) / 1000);
  state.lastFrame = now;
  if (state.running) {
    state.accumulator += dt * Number(ui.speed.value);
    let n = 0;
    while (state.accumulator >= state.sim.settings.dt && n < MAX_STEPS_PER_FRAME) {
      stepOnce();
      state.accumulator -= state.sim.settings.dt;
      n++;
    }
    if (n === MAX_STEPS_PER_FRAME) state.accumulator = 0; // too slow: drop time rather than spiral
  }
  draw();
  requestAnimationFrame(frame);
}

// ---------------------------------------------------------------- drawing

/** @param {string} name */
function css(name) {
  return getComputedStyle(document.documentElement).getPropertyValue(name).trim();
}

/**
 * @param {string} hex
 * @returns {[number, number, number]}
 */
function hexToRgb(hex) {
  const m = /^#?([0-9a-f]{6})$/i.exec(hex);
  if (m === null) return [128, 128, 128];
  const n = parseInt(m[1], 16);
  return [(n >> 16) & 255, (n >> 8) & 255, n & 255];
}

/**
 * Size the canvas backing store to its CSS size times the device pixel ratio.
 * @param {HTMLCanvasElement} canvas
 */
function prepare(canvas) {
  const dpr = window.devicePixelRatio || 1;
  const w = Math.max(1, Math.round(canvas.clientWidth * dpr));
  const h = Math.max(1, Math.round(canvas.clientHeight * dpr));
  if (canvas.width !== w || canvas.height !== h) {
    canvas.width = w;
    canvas.height = h;
  }
  const ctx = canvas.getContext("2d");
  if (ctx === null) throw new Error("2D canvas not supported");
  return { ctx, dpr, w, h };
}

/**
 * @param {HTMLCanvasElement} canvas
 * @returns {Viewport}
 */
function viewportFor(canvas) {
  const dpr = window.devicePixelRatio || 1;
  return fitViewport(
    0,
    0,
    state.world.width,
    state.world.height,
    canvas.width,
    canvas.height,
    12 * dpr,
  );
}

/**
 * @param {CanvasRenderingContext2D} ctx
 * @param {Viewport} v
 * @param {readonly Pose[]} trail
 * @param {string} color
 * @param {number} width
 * @param {number[]} [dash]
 */
function drawTrail(ctx, v, trail, color, width, dash = []) {
  if (trail.length < 2) return;
  ctx.strokeStyle = color;
  ctx.lineWidth = width;
  ctx.setLineDash(dash);
  ctx.beginPath();
  trail.forEach((p, i) => {
    const [x, y] = toCanvas(v, p.x, p.y);
    if (i === 0) ctx.moveTo(x, y);
    else ctx.lineTo(x, y);
  });
  ctx.stroke();
  ctx.setLineDash([]);
}

/**
 * @param {CanvasRenderingContext2D} ctx
 * @param {Viewport} v
 * @param {Pose} p
 * @param {string} color
 * @param {number} radiusM
 * @param {boolean} filled
 */
function drawRobot(ctx, v, p, color, radiusM, filled) {
  const [x, y] = toCanvas(v, p.x, p.y);
  const r = Math.max(3, radiusM * v.scale);
  ctx.strokeStyle = color;
  ctx.fillStyle = color;
  ctx.lineWidth = 2;
  ctx.beginPath();
  ctx.arc(x, y, r, 0, 2 * Math.PI);
  if (filled) {
    ctx.globalAlpha = 0.25;
    ctx.fill();
    ctx.globalAlpha = 1;
  }
  ctx.stroke();
  ctx.beginPath();
  ctx.moveTo(x, y);
  ctx.lineTo(x + Math.cos(p.theta) * r * 1.6, y - Math.sin(p.theta) * r * 1.6);
  ctx.stroke();
}

function drawTruth() {
  const { ctx, dpr, w, h } = prepare(ui.truth);
  const v = viewportFor(ui.truth);
  const sim = state.sim;
  ctx.fillStyle = css("--surface");
  ctx.fillRect(0, 0, w, h);

  if (ui.showBeams.checked && sim.lastScan !== null) {
    const pts = scanEndpoints(sim.lastScan, sim.truePose);
    const [rx, ry] = toCanvas(v, sim.truePose.x, sim.truePose.y);
    ctx.strokeStyle = css("--beam");
    ctx.lineWidth = 1 * dpr;
    ctx.beginPath();
    for (let k = 0; k < pts.length; k += 2) {
      const [x, y] = toCanvas(v, pts[k], pts[k + 1]);
      ctx.moveTo(rx, ry);
      ctx.lineTo(x, y);
    }
    ctx.stroke();
    ctx.fillStyle = css("--hit");
    for (let k = 0; k < pts.length; k += 2) {
      const [x, y] = toCanvas(v, pts[k], pts[k + 1]);
      ctx.fillRect(x - 1.5 * dpr, y - 1.5 * dpr, 3 * dpr, 3 * dpr);
    }
  }

  ctx.strokeStyle = css("--wall");
  ctx.lineWidth = 2.5 * dpr;
  ctx.lineCap = "round";
  ctx.beginPath();
  for (const [x1, y1, x2, y2] of state.world.walls) {
    const [a, b] = toCanvas(v, x1, y1);
    const [c, d] = toCanvas(v, x2, y2);
    ctx.moveTo(a, b);
    ctx.lineTo(c, d);
  }
  ctx.stroke();

  if (state.draftStart !== null && state.draftEnd !== null) {
    ctx.strokeStyle = css("--draft");
    ctx.setLineDash([6 * dpr, 4 * dpr]);
    const [a, b] = toCanvas(v, ...state.draftStart);
    const [c, d] = toCanvas(v, ...state.draftEnd);
    ctx.beginPath();
    ctx.moveTo(a, b);
    ctx.lineTo(c, d);
    ctx.stroke();
    ctx.setLineDash([]);
  }

  if (state.autopilot && state.world.route.length > 0) {
    ctx.fillStyle = css("--muted");
    for (const [x, y] of state.world.route.slice(state.routeIndex)) {
      const [px, py] = toCanvas(v, x, y);
      ctx.fillRect(px - 2 * dpr, py - 2 * dpr, 4 * dpr, 4 * dpr);
    }
  }

  if (sim.pf !== null && ui.showParticles.checked) {
    ctx.fillStyle = css("--particle");
    for (let i = 0; i < sim.pf.count; i++) {
      const [x, y] = toCanvas(v, sim.pf.x[i], sim.pf.y[i]);
      ctx.fillRect(x - 1 * dpr, y - 1 * dpr, 2 * dpr, 2 * dpr);
    }
  }
  drawTrail(ctx, v, sim.trueTrail, css("--truth"), 1.5 * dpr);
  drawRobot(ctx, v, sim.truePose, css("--truth"), sim.settings.robotRadius, true);
  if (sim.mclPose !== null)
    drawRobot(ctx, v, sim.mclPose, css("--mcl"), sim.settings.robotRadius, false);
}

/** Offscreen canvas holding one pixel per grid cell. */
const gridCanvas = document.createElement("canvas");

function drawEstimate() {
  const { ctx, dpr, w, h } = prepare(ui.estimate);
  const v = viewportFor(ui.estimate);
  const sim = state.sim;
  const grid = sim.grid;
  ctx.fillStyle = css("--surface");
  ctx.fillRect(0, 0, w, h);

  if (gridCanvas.width !== grid.width || gridCanvas.height !== grid.height) {
    gridCanvas.width = grid.width;
    gridCanvas.height = grid.height;
  }
  const gctx = gridCanvas.getContext("2d");
  if (gctx !== null) {
    const img = gctx.createImageData(grid.width, grid.height);
    const free = hexToRgb(css("--map-free"));
    const occ = hexToRgb(css("--map-occupied"));
    const unknown = hexToRgb(css("--map-unknown"));
    for (let j = 0; j < grid.height; j++) {
      const row = (grid.height - 1 - j) * grid.width; // image rows run top-down
      for (let i = 0; i < grid.width; i++) {
        const l = grid.logOdds[j * grid.width + i];
        const o = 4 * (row + i);
        if (l === 0) {
          img.data[o] = unknown[0]; // never observed
          img.data[o + 1] = unknown[1];
          img.data[o + 2] = unknown[2];
        } else {
          const p = logOddsToProbability(l);
          img.data[o] = free[0] + (occ[0] - free[0]) * p;
          img.data[o + 1] = free[1] + (occ[1] - free[1]) * p;
          img.data[o + 2] = free[2] + (occ[2] - free[2]) * p;
        }
        img.data[o + 3] = 255;
      }
    }
    gctx.putImageData(img, 0, 0);
    const [x0, y1] = toCanvas(v, grid.originX, grid.originY + grid.height * grid.resolution);
    ctx.imageSmoothingEnabled = false;
    ctx.drawImage(
      gridCanvas,
      x0,
      y1,
      grid.width * grid.resolution * v.scale,
      grid.height * grid.resolution * v.scale,
    );
  }

  drawTrail(ctx, v, sim.trueTrail, css("--truth"), 1.5 * dpr);
  drawTrail(ctx, v, sim.odomTrail, css("--odom"), 1.5 * dpr, [5 * dpr, 4 * dpr]);
  if (sim.pf !== null) drawTrail(ctx, v, sim.mclTrail, css("--mcl"), 1.5 * dpr, [2 * dpr, 3 * dpr]);
  drawTrail(ctx, v, sim.estTrail, css("--est"), 2 * dpr);
  drawRobot(ctx, v, sim.odomPose, css("--odom"), sim.settings.robotRadius, false);
  drawRobot(ctx, v, sim.estPose, css("--est"), sim.settings.robotRadius, true);
}

/**
 * @param {number} v
 * @param {string} unit
 * @param {number} [digits]
 */
function fmt(v, unit, digits = 2) {
  return Number.isNaN(v) ? "-" : `${v.toFixed(digits)} ${unit}`;
}

function drawMetrics() {
  const sim = state.sim;
  const s = summarize(sim);
  // Map precision costs a distance transform; refresh it every 2 s of sim time.
  if (sim.steps !== state.precisionAt && sim.steps % 20 === 0) {
    state.precision = mapPrecision(sim.grid, state.world.walls, 0.1).precision;
    state.precisionAt = sim.steps;
  }
  const set = (/** @type {string} */ id, /** @type {string} */ text) => {
    const node = document.getElementById(id);
    if (node !== null && node.textContent !== text) node.textContent = text;
  };
  set("m-odom", fmt(s.odomError, "m"));
  set("m-odom-rms", fmt(s.odomRms, "m"));
  set("m-est", sim.settings.scanMatching || sim.matchAttempts > 0 ? fmt(s.estError, "m") : "off");
  set("m-est-rms", fmt(s.estRms, "m"));
  set("m-mcl", sim.pf === null ? "off" : fmt(s.mclError, "m"));
  set("m-mcl-rms", fmt(s.mclRms, "m"));
  set("m-time", fmt(s.time, "s", 1));
  set("m-dist", fmt(s.distance, "m", 1));
  set("m-accept", Number.isNaN(s.acceptRate) ? "-" : `${(100 * s.acceptRate).toFixed(0)} %`);
  set("m-cost", fmt(state.stepCostMs, "ms", 1));
  set("m-spread", sim.pf === null ? "-" : fmt(s.mclSpread, "m"));
  set(
    "m-precision",
    Number.isNaN(state.precision) ? "-" : `${(100 * state.precision).toFixed(0)} %`,
  );
}

function draw() {
  drawTruth();
  drawEstimate();
  drawMetrics();
}

// ---------------------------------------------------------------- input

const DRIVE_KEYS = new Set(["ArrowUp", "ArrowDown", "ArrowLeft", "ArrowRight", "w", "a", "s", "d"]);

window.addEventListener("keydown", (e) => {
  const target = e.target;
  if (target instanceof HTMLInputElement || target instanceof HTMLSelectElement) {
    if (target.type !== "checkbox" && target.type !== "range") return;
  }
  const key = e.key.length === 1 ? e.key.toLowerCase() : e.key;
  if (DRIVE_KEYS.has(key)) {
    state.keys.add(key);
    if (!state.running) setRunning(true);
    e.preventDefault();
  } else if (key === " " && !(target instanceof HTMLButtonElement)) {
    setRunning(!state.running);
    e.preventDefault();
  }
});
window.addEventListener("keyup", (e) => {
  state.keys.delete(e.key.length === 1 ? e.key.toLowerCase() : e.key);
});
window.addEventListener("blur", () => {
  state.keys.clear();
  state.pad.clear();
});

for (const button of document.querySelectorAll("[data-drive]")) {
  const name = button.getAttribute("data-drive") ?? "";
  button.addEventListener("pointerdown", () => {
    state.pad.add(name);
    if (!state.running) setRunning(true);
  });
  for (const ev of ["pointerup", "pointerleave", "pointercancel"]) {
    button.addEventListener(ev, () => state.pad.delete(name));
  }
}

for (const button of document.querySelectorAll("[data-mode]")) {
  button.addEventListener("click", () => {
    const mode = /** @type {Mode} */ (button.getAttribute("data-mode"));
    state.mode = mode;
    for (const b of document.querySelectorAll("[data-mode]")) {
      b.setAttribute("aria-pressed", String(b === button));
    }
    ui.hint.textContent = {
      drive: "Arrow keys or WASD to drive",
      draw: "Drag to draw a wall",
      erase: "Click a wall to remove it",
      start: "Click to place the robot",
    }[mode];
  });
}

/**
 * @param {PointerEvent} e
 * @returns {[number, number]}
 */
function pointerWorld(e) {
  const rect = ui.truth.getBoundingClientRect();
  const dpr = window.devicePixelRatio || 1;
  return toWorld(
    viewportFor(ui.truth),
    (e.clientX - rect.left) * dpr,
    (e.clientY - rect.top) * dpr,
  );
}

ui.truth.addEventListener("pointerdown", (e) => {
  const [x, y] = pointerWorld(e);
  const W = state.world.width;
  const H = state.world.height;
  if (state.mode === "draw") {
    state.draftStart = [snap(x, 0.1, W), snap(y, 0.1, H)];
    state.draftEnd = state.draftStart;
    ui.truth.setPointerCapture(e.pointerId);
  } else if (state.mode === "erase") {
    const i = nearestWallIndex(state.world.walls, x, y, 0.3);
    if (i >= 0) {
      const walls = state.world.walls.filter((_, k) => k !== i);
      setWorld({ ...state.world, walls, route: [] });
      setStatus("Wall removed; the simulation restarted on the edited map.");
    }
  } else if (state.mode === "start") {
    if (x > 0 && y > 0 && x < W && y < H) {
      setWorld({ ...state.world, start: { x, y, theta: state.world.start.theta }, route: [] });
      setStatus("Start moved; the simulation restarted.");
    }
  }
});
ui.truth.addEventListener("pointermove", (e) => {
  if (state.draftStart === null) return;
  const [x, y] = pointerWorld(e);
  state.draftEnd = [snap(x, 0.1, state.world.width), snap(y, 0.1, state.world.height)];
});
ui.truth.addEventListener("pointerup", () => {
  const a = state.draftStart;
  const b = state.draftEnd;
  state.draftStart = null;
  state.draftEnd = null;
  if (a === null || b === null || Math.hypot(b[0] - a[0], b[1] - a[1]) < 0.1) return;
  /** @type {import("./geometry.js").Segment} */
  const wall = [a[0], a[1], b[0], b[1]];
  // A drawn wall may cut the demo route, so the route is dropped.
  setWorld({ ...state.world, walls: [...state.world.walls, wall], route: [] });
  setStatus("Wall added; the simulation restarted on the edited map.");
});

ui.play.addEventListener("click", () => setRunning(!state.running));
ui.reset.addEventListener("click", () => {
  resetSimulation();
  setStatus("Reset.");
});
ui.autopilot.addEventListener("click", () => {
  if (!state.autopilot && state.routeIndex >= state.world.route.length) resetSimulation();
  setAutopilot(!state.autopilot);
  if (state.autopilot) setRunning(true);
});
ui.kidnap.addEventListener("click", () => {
  const sim = state.sim;
  const rng = createRng(sim.settings.seed + 7919 * ++state.kidnaps);
  const space = {
    width: state.world.width,
    height: state.world.height,
    walls: state.world.walls,
    clearance: 0.5,
  };
  sim.truePose = randomFreePose(rng, space);
  setAutopilot(false);
  setStatus(
    "Robot moved without telling the estimators. Drive around: MCL injects random particles when its fit drops; scan matching cannot recover.",
  );
});
for (const input of [ui.seed, ui.particles, ui.global]) {
  input.addEventListener("change", resetSimulation);
}
for (const input of [ui.odom, ui.rnoise, ui.dropout, ui.beams, ui.range, ui.scanmatch]) {
  input.addEventListener("input", () => applyLiveSettings(state.sim.settings));
}
ui.map.addEventListener("change", () => {
  const id = ui.map.value;
  setWorld(isBuiltinMap(id) ? BUILTIN_MAPS[id]() : emptyWorld());
  setStatus(id === "empty" ? "Choose Draw walls, then drag on the left view." : "");
});

/**
 * @param {BlobPart} data
 * @param {string} type
 * @param {string} name
 */
function download(data, type, name) {
  const url = URL.createObjectURL(new Blob([data], { type }));
  const a = document.createElement("a");
  a.href = url;
  a.download = name;
  a.click();
  setTimeout(() => URL.revokeObjectURL(url), 1000);
}

ui.saveMap.addEventListener("click", () => {
  download(serializeWorld(state.world), "application/json", "map.json");
});
ui.savePgm.addEventListener("click", () => {
  download(gridToPgm(state.sim.grid), "image/x-portable-graymap", "built-map.pgm");
});
ui.saveCsv.addEventListener("click", () => {
  const sim = state.sim;
  /** @type {Record<string, Pose[]>} */
  const trails = { truth: sim.trueTrail, odometry: sim.odomTrail, estimate: sim.estTrail };
  if (sim.pf !== null) trails.mcl = sim.mclTrail;
  download(trajectoriesCsv(trails, sim.settings.dt), "text/csv", "trajectories.csv");
});
ui.loadMap.addEventListener("change", async () => {
  const file = ui.loadMap.files?.[0];
  ui.loadMap.value = "";
  if (file === undefined) return;
  if (file.size > 1_000_000) {
    setStatus("That file is larger than 1 MB; maps are small JSON files.", true);
    return;
  }
  try {
    setWorld(parseWorld(await file.text()));
    setStatus(`Loaded "${state.world.name}".`);
  } catch (err) {
    setStatus(`Could not load map: ${err instanceof Error ? err.message : String(err)}`, true);
  }
});

// ---------------------------------------------------------------- start

/**
 * URL parameters, for links and for headless screenshots:
 * ?map=warehouse&seed=3&run=600 drives the demo route for 600 steps before
 * the first frame; &paused=1 stops there.
 */
function applyUrlParams() {
  const params = new URLSearchParams(location.search);
  const map = params.get("map");
  if (map !== null && (isBuiltinMap(map) || map === "empty")) ui.map.value = map;
  const seed = params.get("seed");
  if (seed !== null) ui.seed.value = seed;
  const particles = params.get("particles");
  if (particles !== null && [...ui.particles.options].some((o) => o.value === particles)) {
    ui.particles.value = particles;
  }
  if (params.get("global") === "1") ui.global.checked = true;
  if (params.get("scanmatch") === "0") ui.scanmatch.checked = false;
  const id = ui.map.value;
  state.world = isBuiltinMap(id) ? BUILTIN_MAPS[id]() : emptyWorld();
  resetSimulation();
  const run = clampInt(Number(params.get("run")), 1, 5000, 0);
  if (run > 0 && state.world.route.length > 0) {
    setAutopilot(true);
    for (let k = 0; k < run && state.autopilot; k++) stepOnce();
  }
  if (params.get("paused") === "1") setRunning(false);
}

applyUrlParams();
requestAnimationFrame(frame);
