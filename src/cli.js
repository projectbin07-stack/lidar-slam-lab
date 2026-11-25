// Headless command-line interface. Runs the same simulation code as the
// browser app, without a DOM, for scripting and reproducible numbers.

import { parseArgs } from "node:util";
import { BUILTIN_MAPS, isBuiltinMap } from "./maps.js";
import { parseWorld, serializeWorld } from "./world.js";
import { followRoute } from "./autopilot.js";
import {
  MAX_TRAJECTORY_POINTS,
  createSimulation,
  defaultSettings,
  initialScan,
  step,
  summarize,
} from "./sim.js";
import { gridToPgm, mapYaml, trajectoriesCsv } from "./export.js";
import { mapPrecision } from "./metrics.js";
import { parseScanLog, runScanLog, scanLogCsv, scanLogLine } from "./scanlog.js";

/** @typedef {import("./world.js").World} World */
/** @typedef {import("./sim.js").SimSettings} SimSettings */
/** @typedef {import("./sim.js").SimSummary} SimSummary */

export const VERSION = "0.2.1";

/**
 * Everything the CLI touches outside itself, so tests can run it in memory.
 * @typedef {object} CliIo
 * @property {(text: string) => void} out
 * @property {(text: string) => void} err
 * @property {(path: string) => string} readText
 * @property {(path: string, data: string | Uint8Array) => void} writeFile
 * @property {(path: string) => void} mkdir
 */

const HELP = `lidar-slam-lab ${VERSION}: 2D LiDAR SLAM and localisation simulator

Usage:
  lidar-slam-lab maps
  lidar-slam-lab export-map <map-id>
  lidar-slam-lab run [options]
  lidar-slam-lab localize <scans.jsonl> [options]
  lidar-slam-lab benchmark [--maps office,warehouse,loop] [--seeds 5] [--steps N]
  lidar-slam-lab --help | --version

run drives the map's demo route with the autopilot and prints the errors of
dead reckoning, scan matching and MCL against the ground truth.

Options for run:
  --map <id|file.json>    built-in map id or a map JSON file (default: office)
  --seed <n>              random seed (default: 1)
  --steps <n>             stop after n steps of 0.1 s even if the route
                          is not finished (default: 3000, max ${MAX_TRAJECTORY_POINTS / 2})
  --no-scan-matching      dead reckoning only for the mapping estimate
  --particles <n>         MCL particle count, 0 disables (default: 300)
  --global                start MCL with no idea where the robot is
  --odom-noise <k>        scale every odometry error term by k (default: 1)
  --range-noise <m>       LiDAR range noise std in metres (default: 0.02)
  --dropout <p>           probability a beam returns nothing (default: 0.03)
  --beams <n>             beams per scan (default: 180)
  --out <dir>             write map.pgm, map.yaml and trajectory.csv there
  --scan-log <file>       also write every scan with the odometry and true
                          pose as a scan log (the input format of localize)
  --json                  print the summary as JSON

localize runs the same estimators on recorded scans: a JSON Lines file with
one sensor_msgs/LaserScan-like object per line plus the odometry pose at that
scan (format in docs/algorithms.md). Without --map it builds a map with
odometry + scan matching; with --map it also runs MCL on that floor plan.

Options for localize:
  --mount <x,y,yaw>       LiDAR pose in the robot frame (m, m, rad; default 0,0,0)
  --start <x,y,theta>     robot pose at the first scan in the map frame
                          (default: the map's start with --map, else the first
                          odometry pose)
  --map <id|file.json>    known floor plan for MCL
  --particles <n>         MCL particle count, 0 disables (default: 300)
  --global                start MCL with no idea where the robot is
  --motion <model>        MCL motion model: odometry (rot1-trans-rot2) or omni
                          (forward, sideways, turn; for mecanum bases)
  --no-scan-matching      dead reckoning only for the mapping estimate
  --seed <n>              MCL random seed (default: 1)
  --out <dir>             write map.pgm, map.yaml and trajectory.csv there
  --json                  print the summary as JSON
`;

/**
 * @param {string} value
 * @param {string} name
 * @param {number} min
 * @param {number} max
 * @param {boolean} [integer]
 * @returns {number}
 */
function numberOption(value, name, min, max, integer = false) {
  const v = Number(value);
  if (!Number.isFinite(v) || v < min || v > max || (integer && !Number.isInteger(v))) {
    throw new UsageError(
      `--${name} must be ${integer ? "an integer" : "a number"} in [${min}, ${max}]`,
    );
  }
  return v;
}

class UsageError extends Error {}

/**
 * @param {string} spec
 * @param {CliIo} io
 * @returns {World}
 */
function loadWorld(spec, io) {
  if (isBuiltinMap(spec)) return BUILTIN_MAPS[spec]();
  if (!spec.endsWith(".json")) {
    throw new UsageError(
      `unknown map "${spec}"; use one of ${Object.keys(BUILTIN_MAPS).join(", ")} or a .json file`,
    );
  }
  try {
    return parseWorld(io.readText(spec));
  } catch (e) {
    throw new UsageError(`${spec}: ${e instanceof Error ? e.message : String(e)}`);
  }
}

/**
 * Drive the world's route (or stand still if it has none) for up to maxSteps.
 * @param {World} world
 * @param {SimSettings} settings
 * @param {number} maxSteps
 * @param {(sim: import("./sim.js").Simulation) => void} [onScan] called after the
 *   first scan and after every step
 */
export function runRoute(world, settings, maxSteps, onScan) {
  const sim = createSimulation(world, settings);
  initialScan(sim);
  onScan?.(sim);
  let idx = 0;
  let finished = world.route.length === 0;
  for (let k = 0; k < maxSteps && !finished; k++) {
    const cmd = followRoute(sim.truePose, world.route, idx);
    idx = cmd.index;
    finished = cmd.done;
    if (!finished) {
      step(sim, cmd.v, cmd.w);
      onScan?.(sim);
    }
  }
  return { sim, finished };
}

/**
 * @param {number} v
 * @param {number} [digits]
 */
const fmt = (v, digits = 3) => (Number.isNaN(v) ? "-" : v.toFixed(digits));

/**
 * @param {string[]} argv Arguments after the program name.
 * @param {CliIo} io
 * @returns {number} Exit code.
 */
export function main(argv, io) {
  try {
    return dispatch(argv, io);
  } catch (e) {
    if (e instanceof UsageError || (e instanceof TypeError && "code" in e)) {
      io.err(`error: ${e.message}\n`);
      return 2;
    }
    throw e;
  }
}

/**
 * @param {string[]} argv
 * @param {CliIo} io
 * @returns {number}
 */
function dispatch(argv, io) {
  const [command, ...rest] = argv;
  if (command === undefined || command === "--help" || command === "-h" || command === "help") {
    io.out(HELP);
    return command === undefined ? 2 : 0;
  }
  if (command === "--version" || command === "-V") {
    io.out(`${VERSION}\n`);
    return 0;
  }
  if (command === "maps") {
    for (const [id, make] of Object.entries(BUILTIN_MAPS)) {
      const w = make();
      io.out(
        `${id.padEnd(10)} ${w.name.padEnd(18)} ${w.width} x ${w.height} m, ${w.walls.length} walls, ${w.route.length} waypoints\n`,
      );
    }
    return 0;
  }
  if (command === "export-map") {
    const id = rest[0];
    if (id === undefined || !isBuiltinMap(id))
      throw new UsageError("export-map needs a built-in map id");
    io.out(serializeWorld(BUILTIN_MAPS[id]()) + "\n");
    return 0;
  }
  if (command === "run") return runCommand(rest, io);
  if (command === "benchmark") return benchmarkCommand(rest, io);
  if (command === "localize") return localizeCommand(rest, io);
  throw new UsageError(`unknown command "${command}" (try --help)`);
}

/**
 * @param {string[]} args
 * @param {CliIo} io
 * @returns {number}
 */
function runCommand(args, io) {
  const { values } = parseArgs({
    args,
    strict: true,
    options: {
      map: { type: "string", default: "office" },
      seed: { type: "string", default: "1" },
      steps: { type: "string", default: "3000" },
      "no-scan-matching": { type: "boolean", default: false },
      particles: { type: "string", default: "300" },
      global: { type: "boolean", default: false },
      "odom-noise": { type: "string", default: "1" },
      "range-noise": { type: "string", default: "0.02" },
      dropout: { type: "string", default: "0.03" },
      beams: { type: "string", default: "180" },
      out: { type: "string" },
      "scan-log": { type: "string" },
      json: { type: "boolean", default: false },
    },
  });
  const world = loadWorld(values.map, io);
  const settings = defaultSettings();
  settings.seed = numberOption(values.seed, "seed", 0, 2 ** 32 - 1, true);
  settings.scanMatching = !values["no-scan-matching"];
  settings.mclParticles = numberOption(values.particles, "particles", 0, 20000, true);
  settings.mclGlobal = values.global;
  const k = numberOption(values["odom-noise"], "odom-noise", 0, 10);
  for (const key of /** @type {Array<keyof typeof settings.odometry>} */ (
    Object.keys(settings.odometry)
  )) {
    settings.odometry[key] *= k;
  }
  settings.lidar.rangeNoise = numberOption(values["range-noise"], "range-noise", 0, 1);
  settings.lidar.dropout = numberOption(values.dropout, "dropout", 0, 1);
  settings.lidar.beams = numberOption(values.beams, "beams", 8, 2000, true);
  const maxSteps = numberOption(values.steps, "steps", 1, MAX_TRAJECTORY_POINTS / 2, true);

  /** @type {string[]} */
  const logLines = [];
  const onScan =
    values["scan-log"] === undefined
      ? undefined
      : (/** @type {import("./sim.js").Simulation} */ sim) => {
          logLines.push(
            scanLogLine(
              sim.time,
              sim.odomPose,
              /** @type {import("./lidar.js").Scan} */ (sim.lastScan),
              {
                truth: sim.truePose,
                full: logLines.length === 0,
              },
            ),
          );
        };
  const { sim, finished } = runRoute(world, settings, maxSteps, onScan);
  if (values["scan-log"] !== undefined)
    io.writeFile(values["scan-log"], logLines.join("\n") + "\n");
  const s = summarize(sim);
  const map = mapPrecision(sim.grid, world.walls, 0.1);

  if (values.out !== undefined) {
    const dir = values.out.replace(/[\\/]+$/, "");
    io.mkdir(dir);
    io.writeFile(`${dir}/map.pgm`, gridToPgm(sim.grid));
    io.writeFile(`${dir}/map.yaml`, mapYaml(sim.grid, "map.pgm"));
    /** @type {Record<string, import("./geometry.js").Pose[]>} */
    const trails = { truth: sim.trueTrail, odometry: sim.odomTrail, estimate: sim.estTrail };
    if (sim.pf !== null) trails.mcl = sim.mclTrail;
    io.writeFile(`${dir}/trajectory.csv`, trajectoriesCsv(trails, settings.dt));
  }

  const result = {
    map: world.name,
    seed: settings.seed,
    scanMatching: settings.scanMatching,
    routeFinished: finished,
    ...s,
    mapPrecision: map.precision,
    mapOccupiedCells: map.occupiedCells,
  };
  if (values.json) {
    // NaN is not valid JSON; report "not applicable" as null.
    io.out(
      JSON.stringify(result, (_k, v) => (typeof v === "number" && Number.isNaN(v) ? null : v), 2) +
        "\n",
    );
    return 0;
  }
  io.out(
    [
      `map              ${world.name} (seed ${settings.seed})`,
      `route            ${finished ? "finished" : "stopped at step limit"} after ${fmt(s.time, 1)} s, ${fmt(s.distance, 1)} m`,
      `                 final error   RMS error`,
      `dead reckoning   ${fmt(s.odomError).padStart(8)} m  ${fmt(s.odomRms).padStart(8)} m`,
      `${settings.scanMatching ? "scan matching   " : "mapping pose    "} ${fmt(s.estError).padStart(8)} m  ${fmt(s.estRms).padStart(8)} m`,
      `MCL${settings.mclGlobal ? " (global)    " : "             "} ${fmt(s.mclError).padStart(8)} m  ${fmt(s.mclRms).padStart(8)} m`,
      `scan matches     ${Number.isNaN(s.acceptRate) ? "-" : `${fmt(100 * s.acceptRate, 1)} % accepted`}`,
      `map precision    ${Number.isNaN(map.precision) ? "-" : `${fmt(100 * map.precision, 1)} %`} of ${map.occupiedCells} occupied cells within 0.1 m of a true wall`,
      `collisions       ${s.collisions}`,
      "",
    ].join("\n"),
  );
  if (values.out !== undefined)
    io.out(`wrote map.pgm, map.yaml, trajectory.csv to ${values.out}\n`);
  if (values["scan-log"] !== undefined)
    io.out(`wrote ${logLines.length} scans to ${values["scan-log"]}\n`);
  return 0;
}

/**
 * @param {string | undefined} value
 * @param {string} name
 * @returns {import("./geometry.js").Pose | undefined}
 */
function poseOption(value, name) {
  if (value === undefined) return undefined;
  const v = value.split(",").map((x) => Number(x.trim()));
  if (v.length !== 3 || !v.every(Number.isFinite))
    throw new UsageError(`--${name} must be three numbers x,y,theta`);
  return { x: v[0], y: v[1], theta: v[2] };
}

/**
 * @param {string[]} args
 * @param {CliIo} io
 * @returns {number}
 */
function localizeCommand(args, io) {
  const { values, positionals } = parseArgs({
    args,
    strict: true,
    allowPositionals: true,
    options: {
      mount: { type: "string" },
      start: { type: "string" },
      map: { type: "string" },
      particles: { type: "string", default: "300" },
      global: { type: "boolean", default: false },
      motion: { type: "string", default: "odometry" },
      "no-scan-matching": { type: "boolean", default: false },
      seed: { type: "string", default: "1" },
      out: { type: "string" },
      json: { type: "boolean", default: false },
    },
  });
  if (positionals.length !== 1) throw new UsageError("localize needs one scan log file");
  const file = positionals[0];
  if (values.motion !== "odometry" && values.motion !== "omni")
    throw new UsageError("--motion must be odometry or omni");
  const mount = poseOption(values.mount, "mount");
  const map = values.map === undefined ? undefined : loadWorld(values.map, io);
  let log;
  try {
    log = parseScanLog(io.readText(file), mount);
  } catch (e) {
    if (e instanceof TypeError && "code" in e) throw e;
    throw new UsageError(`${file}: ${e instanceof Error ? e.message : String(e)}`);
  }
  const r = runScanLog(log, {
    start: poseOption(values.start, "start") ?? map?.start,
    map,
    particles: numberOption(values.particles, "particles", 0, 20000, true),
    global: values.global,
    motionModel: values.motion,
    scanMatching: !values["no-scan-matching"],
    seed: numberOption(values.seed, "seed", 0, 2 ** 32 - 1, true),
  });
  if (values.out !== undefined) {
    const dir = values.out.replace(/[\\/]+$/, "");
    io.mkdir(dir);
    io.writeFile(`${dir}/map.pgm`, gridToPgm(r.grid));
    io.writeFile(`${dir}/map.yaml`, mapYaml(r.grid, "map.pgm"));
    io.writeFile(`${dir}/trajectory.csv`, scanLogCsv(r));
  }
  const last = r.odometry.length - 1;
  const summary = {
    scans: log.length,
    duration: log[last].t - log[0].t,
    distance: r.distance,
    acceptRate: r.matchAttempts === 0 ? NaN : r.matchAccepted / r.matchAttempts,
    final: { odometry: r.odometry[last], estimate: r.estimate[last], mcl: r.mcl?.[last] ?? null },
    rms: r.rms,
  };
  if (values.json) {
    io.out(
      JSON.stringify(summary, (_k, v) => (typeof v === "number" && Number.isNaN(v) ? null : v), 2) +
        "\n",
    );
    return 0;
  }
  const pose = (/** @type {import("./geometry.js").Pose | null} */ p) =>
    p === null ? "-" : `${fmt(p.x)}, ${fmt(p.y)}, ${fmt(p.theta)}`;
  const lines = [
    `scans            ${log.length} over ${fmt(summary.duration, 1)} s, ${fmt(r.distance, 1)} m by odometry`,
    `scan matches     ${Number.isNaN(summary.acceptRate) ? "-" : `${fmt(100 * summary.acceptRate, 1)} % accepted`}`,
    `final pose       x, y, theta`,
    `  odometry       ${pose(summary.final.odometry)}`,
    `  scan matching  ${pose(summary.final.estimate)}`,
    `  MCL            ${pose(summary.final.mcl)}`,
  ];
  if (r.rms)
    lines.push(
      `RMS error against the log's truth poses: odometry ${fmt(r.rms.odometry)} m, scan matching ${fmt(r.rms.estimate)} m, MCL ${Number.isNaN(r.rms.mcl) ? "-" : `${fmt(r.rms.mcl)} m`}`,
    );
  io.out(lines.join("\n") + "\n");
  if (values.out !== undefined)
    io.out(`wrote map.pgm, map.yaml, trajectory.csv to ${values.out}\n`);
  return 0;
}

/**
 * @param {string[]} args
 * @param {CliIo} io
 * @returns {number}
 */
function benchmarkCommand(args, io) {
  const { values } = parseArgs({
    args,
    strict: true,
    options: {
      maps: { type: "string", default: Object.keys(BUILTIN_MAPS).join(",") },
      seeds: { type: "string", default: "5" },
      steps: { type: "string", default: "3000" },
    },
  });
  const seeds = numberOption(values.seeds, "seeds", 1, 100, true);
  const maxSteps = numberOption(values.steps, "steps", 1, MAX_TRAJECTORY_POINTS / 2, true);
  const ids = values.maps.split(",").map((m) => m.trim());
  for (const id of ids) if (!isBuiltinMap(id)) throw new UsageError(`unknown map "${id}"`);

  io.out("RMS position error over the demo route, metres (lower is better)\n\n");
  io.out("map         seed  distance  dead-reckoning  scan-matching  MCL-tracking\n");
  for (const id of ids) {
    /** @type {number[][]} */
    const cols = [[], [], []];
    for (let seed = 1; seed <= seeds; seed++) {
      const world = BUILTIN_MAPS[/** @type {keyof typeof BUILTIN_MAPS} */ (id)]();
      // Two runs with the same seed see identical odometry and LiDAR noise.
      const base = { ...defaultSettings(), seed };
      const a = runRoute(world, { ...base, scanMatching: false }, maxSteps).sim;
      const b = runRoute(world, { ...base, mclParticles: 0 }, maxSteps).sim;
      const sa = summarize(a);
      const sb = summarize(b);
      cols[0].push(sa.odomRms);
      cols[1].push(sb.estRms);
      cols[2].push(sa.mclRms);
      io.out(
        `${id.padEnd(10)} ${String(seed).padStart(5)} ${fmt(sa.distance, 1).padStart(8)} m ${fmt(sa.odomRms).padStart(15)} ${fmt(sb.estRms).padStart(14)} ${fmt(sa.mclRms).padStart(13)}\n`,
      );
    }
    const mean = (/** @type {number[]} */ xs) => xs.reduce((p, q) => p + q, 0) / xs.length;
    io.out(
      `${id.padEnd(10)}  mean ${"".padStart(10)} ${fmt(mean(cols[0])).padStart(15)} ${fmt(mean(cols[1])).padStart(14)} ${fmt(mean(cols[2])).padStart(13)}\n`,
    );
  }
  return 0;
}
