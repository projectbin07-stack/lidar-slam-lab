// Recorded scans as input: a scan log (one JSON object per line, the fields
// of a sensor_msgs/LaserScan plus the odometry pose at that scan) is read,
// checked and run through the same estimators the simulation uses: wheel
// odometry, odometry corrected by scan-to-map matching with the grid built
// from that estimate and, when a floor plan is given, MCL on it.
//
// Line format (JSON Lines; keys after the first scan line may be omitted and
// are then taken from the previous line, except t, odom and ranges):
//   {"t": 0.2, "odom": [x, y, theta],
//    "angle_min": -3.14159, "angle_increment": 0.01745,
//    "range_min": 0.12, "range_max": 3.5,
//    "ranges": [1.02, null, 0.98, "Infinity", ...]}
// A range that is null, not a finite number (including the strings
// "Infinity", "-Infinity" and "NaN" that some exporters write) or outside
// [range_min, range_max] is "no return", as REP 117 specifies for LaserScan.
// Blank lines and lines starting with # are skipped.

import { createGrid, integrateScan, occupiedMask } from "./grid.js";
import { likelihoodField } from "./distance.js";
import { DEFAULT_MATCH, matchScan } from "./scanmatch.js";
import { applyOdometryDelta, odometryDelta } from "./motion.js";
import { rasterizeWalls } from "./metrics.js";
import {
  createParticleFilter,
  estimate,
  initializeAround,
  initializeUniform,
  predict,
  resampleIfNeeded,
  updateWeights,
} from "./mcl.js";
import { createRng } from "./rng.js";
import { MCL_FIELD_SIGMA } from "./sim.js";

/** @typedef {import("./geometry.js").Pose} Pose */
/** @typedef {import("./lidar.js").Scan} Scan */
/** @typedef {import("./world.js").World} World */
/** @typedef {import("./grid.js").OccupancyGrid} OccupancyGrid */
/** @typedef {import("./mcl.js").MotionModel} MotionModel */
/** @typedef {import("./scanmatch.js").MatchOptions} MatchOptions */

/**
 * @typedef {object} LoggedScan
 * @property {number} t seconds
 * @property {Pose} odom odometry pose of the robot at the scan
 * @property {Scan} scan angles in the sensor frame, NaN = no return
 * @property {Pose} [truth] true map-frame pose, when known (simulated logs)
 */

/** At most this many scans and beams per scan, so a bad file cannot exhaust memory. */
export const MAX_SCANS = 200000;
export const MAX_BEAMS = 10000;

/**
 * @param {unknown} v
 * @returns {number}
 */
function rangeValue(v) {
  if (typeof v === "number") return v;
  return NaN; // null, "Infinity", "NaN", ...: no return
}

/**
 * Parse a scan log.
 * @param {string} text
 * @param {Pose} [mount] LiDAR pose in the robot frame, stored as each scan's origin
 * @returns {LoggedScan[]}
 */
export function parseScanLog(text, mount) {
  /** @type {LoggedScan[]} */
  const out = [];
  /** @type {{ angle_min?: number, angle_increment?: number, range_min?: number, range_max?: number }} */
  const prev = {};
  const lines = text.split(/\r?\n/);
  for (let n = 0; n < lines.length; n++) {
    const line = lines[n].trim();
    if (line === "" || line.startsWith("#")) continue;
    const where = `line ${n + 1}`;
    /** @type {any} */
    let o;
    try {
      o = JSON.parse(line);
    } catch {
      throw new Error(`${where}: not valid JSON`);
    }
    if (o === null || typeof o !== "object" || Array.isArray(o))
      throw new Error(`${where}: expected an object`);
    const t = o.t;
    if (typeof t !== "number" || !Number.isFinite(t))
      throw new Error(`${where}: t must be a number`);
    if (out.length > 0 && t < out[out.length - 1].t)
      throw new Error(`${where}: t goes backwards (${t} after ${out[out.length - 1].t})`);
    const odom = o.odom;
    if (
      !Array.isArray(odom) ||
      odom.length !== 3 ||
      !odom.every((v) => typeof v === "number" && Number.isFinite(v))
    )
      throw new Error(`${where}: odom must be [x, y, theta] with finite numbers`);
    for (const k of /** @type {const} */ ([
      "angle_min",
      "angle_increment",
      "range_min",
      "range_max",
    ])) {
      if (o[k] !== undefined) {
        if (typeof o[k] !== "number" || !Number.isFinite(o[k]))
          throw new Error(`${where}: ${k} must be a finite number`);
        prev[k] = o[k];
      } else if (prev[k] === undefined) throw new Error(`${where}: ${k} is missing`);
    }
    const angleMin = /** @type {number} */ (prev.angle_min);
    const inc = /** @type {number} */ (prev.angle_increment);
    const rMin = /** @type {number} */ (prev.range_min);
    const rMax = /** @type {number} */ (prev.range_max);
    if (!(rMax > 0) || rMin < 0 || rMin >= rMax)
      throw new Error(`${where}: need 0 <= range_min < range_max`);
    if (inc === 0) throw new Error(`${where}: angle_increment must not be 0`);
    if (!Array.isArray(o.ranges) || o.ranges.length < 2 || o.ranges.length > MAX_BEAMS)
      throw new Error(`${where}: ranges must be an array of 2 to ${MAX_BEAMS} values`);
    if (out.length >= MAX_SCANS) throw new Error(`${where}: more than ${MAX_SCANS} scans`);
    const count = o.ranges.length;
    const angles = new Float64Array(count);
    const ranges = new Float64Array(count);
    for (let i = 0; i < count; i++) {
      angles[i] = angleMin + i * inc;
      const r = rangeValue(o.ranges[i]);
      ranges[i] = Number.isFinite(r) && r >= rMin && r <= rMax ? r : NaN;
    }
    /** @type {Scan} */
    const scan = { angles, ranges, maxRange: rMax };
    if (mount !== undefined) scan.origin = { ...mount };
    /** @type {LoggedScan} */
    const rec = { t, odom: { x: odom[0], y: odom[1], theta: odom[2] }, scan };
    if (o.truth !== undefined) {
      const tr = o.truth;
      if (
        !Array.isArray(tr) ||
        tr.length !== 3 ||
        !tr.every((v) => typeof v === "number" && Number.isFinite(v))
      )
        throw new Error(`${where}: truth must be [x, y, theta] with finite numbers`);
      rec.truth = { x: tr[0], y: tr[1], theta: tr[2] };
    }
    out.push(rec);
  }
  if (out.length === 0) throw new Error("the scan log has no scans");
  return out;
}

/**
 * @typedef {object} ScanLogOptions
 * @property {Pose} [start] Map-frame pose of the robot at the first scan
 *   (default: the first odometry pose, so the map frame is the odometry frame).
 * @property {boolean} [scanMatching] default true
 * @property {number} [resolution] grid cell size, m (default 0.05)
 * @property {number} [fieldSigma] likelihood-field fall-off for matching, m (default 0.1)
 * @property {number} [fieldOccupiedThreshold] default 0.5
 * @property {Partial<MatchOptions>} [match]
 * @property {World} [map] known floor plan: runs MCL on it from `start`
 * @property {number} [particles] MCL particle count (default 300)
 * @property {boolean} [global] start MCL spread over the whole map
 * @property {MotionModel} [motionModel] MCL motion model (default "odometry")
 * @property {number} [seed] MCL random seed (default 1)
 * @property {number} [clearance] m, for MCL's free space (default 0.2)
 */

/**
 * @typedef {object} ScanLogResult
 * @property {number[]} t
 * @property {Pose[]} odometry odometry in the map frame
 * @property {Pose[]} estimate odometry + scan matching
 * @property {Pose[] | null} mcl
 * @property {OccupancyGrid} grid built from the estimate
 * @property {number} matchAttempts
 * @property {number} matchAccepted
 * @property {number} distance metres, from odometry
 * @property {{ odometry: number, estimate: number, mcl: number } | null} rms RMS
 *   position errors against the log's truth poses (null unless every line has
 *   one; mcl is NaN without MCL)
 */

/**
 * Run the estimators over a scan log.
 * @param {LoggedScan[]} log
 * @param {ScanLogOptions} [options]
 * @returns {ScanLogResult}
 */
export function runScanLog(log, options = {}) {
  if (log.length === 0) throw new Error("the scan log has no scans");
  const start = options.start ?? log[0].odom;
  const resolution = options.resolution ?? 0.05;
  const scanMatching = options.scanMatching ?? true;
  const fieldSigma = options.fieldSigma ?? 0.1;
  const threshold = options.fieldOccupiedThreshold ?? 0.5;
  const match = { ...DEFAULT_MATCH, ...options.match };
  // Grid: the map's rectangle with a margin, or a square around the start
  // large enough for the odometry path plus the LiDAR's range.
  const margin = 2;
  let x0;
  let y0;
  let w;
  let h;
  if (options.map) {
    x0 = -margin;
    y0 = -margin;
    w = options.map.width + 2 * margin;
    h = options.map.height + 2 * margin;
  } else {
    let minX = Infinity;
    let minY = Infinity;
    let maxX = -Infinity;
    let maxY = -Infinity;
    let pose = start;
    let reach = 0;
    for (let i = 0; i < log.length; i++) {
      if (i > 0) pose = applyOdometryDelta(pose, odometryDelta(log[i - 1].odom, log[i].odom));
      minX = Math.min(minX, pose.x);
      maxX = Math.max(maxX, pose.x);
      minY = Math.min(minY, pose.y);
      maxY = Math.max(maxY, pose.y);
      reach = Math.max(reach, log[i].scan.maxRange);
    }
    const pad = reach + margin;
    x0 = minX - pad;
    y0 = minY - pad;
    w = maxX - minX + 2 * pad;
    h = maxY - minY + 2 * pad;
  }
  const grid = createGrid(w, h, resolution, x0, y0);

  /** @type {import("./mcl.js").ParticleFilter | null} */
  let pf = null;
  if (options.map && (options.particles ?? 300) > 0) {
    const known = likelihoodField(rasterizeWalls(grid, options.map.walls), grid, MCL_FIELD_SIGMA);
    pf = createParticleFilter(
      options.particles ?? 300,
      known,
      createRng((options.seed ?? 1) ^ 0x85ebca6b),
      { motionModel: options.motionModel ?? "odometry" },
      {
        width: options.map.width,
        height: options.map.height,
        walls: options.map.walls,
        clearance: options.clearance ?? 0.2,
      },
    );
    if (options.global) initializeUniform(pf);
    else initializeAround(pf, start, 0.1, 0.05);
  }

  let odom = { ...start };
  let est = { ...start };
  /** @type {ScanLogResult} */
  const res = {
    t: [log[0].t],
    odometry: [{ ...odom }],
    estimate: [{ ...est }],
    mcl: pf ? [estimate(pf).pose] : null,
    grid,
    matchAttempts: 0,
    matchAccepted: 0,
    distance: 0,
    rms: null,
  };
  integrateScan(grid, est, log[0].scan);
  for (let i = 1; i < log.length; i++) {
    const measured = odometryDelta(log[i - 1].odom, log[i].odom);
    res.distance += measured.trans;
    const scan = log[i].scan;
    odom = applyOdometryDelta(odom, measured);
    const guess = applyOdometryDelta(est, measured);
    if (pf) {
      predict(pf, measured);
      updateWeights(pf, scan);
      resampleIfNeeded(pf);
      /** @type {Pose[]} */ (res.mcl).push(estimate(pf).pose);
    }
    if (scanMatching) {
      const field = likelihoodField(occupiedMask(grid, threshold), grid, fieldSigma);
      const r = matchScan(field, scan, guess, match);
      res.matchAttempts++;
      if (r.accepted) res.matchAccepted++;
      est = r.pose;
    } else est = guess;
    integrateScan(grid, est, scan);
    res.t.push(log[i].t);
    res.odometry.push({ ...odom });
    res.estimate.push({ ...est });
  }
  if (log.every((l) => l.truth !== undefined)) {
    const rms = (/** @type {Pose[]} */ ps) =>
      Math.sqrt(
        ps.reduce((acc, p, i) => {
          const tr = /** @type {Pose} */ (log[i].truth);
          return acc + (p.x - tr.x) ** 2 + (p.y - tr.y) ** 2;
        }, 0) / ps.length,
      );
    res.rms = {
      odometry: rms(res.odometry),
      estimate: rms(res.estimate),
      mcl: res.mcl ? rms(res.mcl) : NaN,
    };
  }
  return res;
}

/**
 * The poses of a run as CSV with the log's timestamps.
 * @param {ScanLogResult} r
 * @returns {string}
 */
export function scanLogCsv(r) {
  /** @type {Record<string, Pose[]>} */
  const trails = { odometry: r.odometry, estimate: r.estimate };
  if (r.mcl) trails.mcl = r.mcl;
  const names = Object.keys(trails);
  const rows = [["t", ...names.flatMap((m) => [`${m}_x`, `${m}_y`, `${m}_theta`])].join(",")];
  for (let i = 0; i < r.t.length; i++) {
    const cells = [r.t[i].toFixed(3)];
    for (const n of names) {
      const p = trails[n][i];
      cells.push(p.x.toFixed(4), p.y.toFixed(4), p.theta.toFixed(5));
    }
    rows.push(cells.join(","));
  }
  return rows.join("\n") + "\n";
}

/**
 * One scan log line (the inverse of parseScanLog for one scan). Angle and
 * range limits are written only when `full` is true (the first line, or
 * after they change). Ranges are rounded to 0.1 mm; no return is null.
 * @param {number} t
 * @param {Pose} odom
 * @param {Scan} scan
 * @param {{ truth?: Pose, full?: boolean, rangeMin?: number }} [extra]
 * @returns {string}
 */
export function scanLogLine(t, odom, scan, extra = {}) {
  const r6 = (/** @type {number} */ v) => Number(v.toFixed(6));
  /** @type {Record<string, unknown>} */
  const o = { t: Number(t.toFixed(6)), odom: [r6(odom.x), r6(odom.y), r6(odom.theta)] };
  if (extra.truth) o.truth = [r6(extra.truth.x), r6(extra.truth.y), r6(extra.truth.theta)];
  if (extra.full) {
    o.angle_min = scan.angles[0];
    o.angle_increment = scan.angles[1] - scan.angles[0];
    o.range_min = extra.rangeMin ?? 0;
    o.range_max = scan.maxRange;
  }
  o.ranges = Array.from(scan.ranges, (r) => (Number.isNaN(r) ? null : Number(r.toFixed(4))));
  return JSON.stringify(o);
}
