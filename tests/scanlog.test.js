// Recorded scans as input: the scan log format, its checks, and the
// estimators run over a log written by the simulator.
import test from "node:test";
import assert from "node:assert/strict";
import { close } from "./helpers.js";
import { parseScanLog, runScanLog, scanLogCsv, scanLogLine } from "../src/scanlog.js";
import { main, runRoute } from "../src/cli.js";
import { defaultSettings, summarize } from "../src/sim.js";
import { BUILTIN_MAPS } from "../src/maps.js";

const head = '"angle_min":-1.5,"angle_increment":1,"range_min":0.1,"range_max":3';

test("parse: LaserScan fields, inherited limits, no-return values, mount as origin", () => {
  const text = [
    "# a comment",
    `{"t":0,"odom":[0,0,0],${head},"ranges":[1,null,"Infinity",0.05]}`,
    "",
    '{"t":0.1,"odom":[0.1,0,0.01],"ranges":[2.5,3.5,"NaN",1]}',
  ].join("\n");
  const log = parseScanLog(text, { x: 0.1, y: 0, theta: 0 });
  assert.equal(log.length, 2);
  assert.deepEqual(Array.from(log[0].scan.angles), [-1.5, -0.5, 0.5, 1.5]);
  assert.equal(log[0].scan.ranges[0], 1);
  for (const i of [1, 2, 3]) assert.ok(Number.isNaN(log[0].scan.ranges[i]), `beam ${i}`);
  assert.equal(log[1].scan.ranges[0], 2.5);
  assert.ok(Number.isNaN(log[1].scan.ranges[1]), "above range_max");
  assert.equal(log[1].scan.maxRange, 3);
  assert.deepEqual(log[1].scan.origin, { x: 0.1, y: 0, theta: 0 });
  assert.deepEqual(log[1].odom, { x: 0.1, y: 0, theta: 0.01 });
  assert.equal(log[1].truth, undefined);
});

test("parse: every malformed line is reported with its line number", () => {
  const ok = `{"t":0,"odom":[0,0,0],${head},"ranges":[1,1]}`;
  const cases = [
    ["{", /line 1: not valid JSON/],
    ["[1]", /line 1: expected an object/],
    ['{"odom":[0,0,0]}', /t must be a number/],
    [`{"t":0,"odom":[0,0],${head},"ranges":[1,1]}`, /odom must be/],
    ['{"t":0,"odom":[0,0,0],"ranges":[1,1]}', /angle_min is missing/],
    [`{"t":0,"odom":[0,0,0],${head.replace("-1.5", '"x"')},"ranges":[1,1]}`, /angle_min must be/],
    [
      `{"t":0,"odom":[0,0,0],${head.replace('"range_min":0.1', '"range_min":5')},"ranges":[1,1]}`,
      /range_min < range_max/,
    ],
    [
      `{"t":0,"odom":[0,0,0],${head.replace('"angle_increment":1', '"angle_increment":0')},"ranges":[1,1]}`,
      /angle_increment/,
    ],
    [`{"t":0,"odom":[0,0,0],${head},"ranges":[1]}`, /ranges must be an array/],
    [`{"t":0,"odom":[0,0,0],"truth":[1],${head},"ranges":[1,1]}`, /truth must be/],
    [`${ok}\n{"t":-1,"odom":[0,0,0],"ranges":[1,1]}`, /line 2: t goes backwards/],
    ["# only a comment", /no scans/],
  ];
  for (const [text, re] of /** @type {[string, RegExp][]} */ (cases))
    assert.throws(() => parseScanLog(text), re, text);
  assert.throws(() => runScanLog([]), /no scans/);
});

test("a scan log line round-trips through the parser", () => {
  const scan = {
    angles: Float64Array.from([0, 0.5, 1]),
    ranges: Float64Array.from([1.23456, NaN, 2]),
    maxRange: 8,
  };
  const line = scanLogLine(0.1, { x: 1, y: 2, theta: 0.3 }, scan, {
    truth: { x: 1.01, y: 2, theta: 0.3 },
    full: true,
  });
  const [r] = parseScanLog(line);
  assert.deepEqual(Array.from(r.scan.angles), [0, 0.5, 1]);
  close(r.scan.ranges[0], 1.2346);
  assert.ok(Number.isNaN(r.scan.ranges[1]));
  assert.deepEqual(r.truth, { x: 1.01, y: 2, theta: 0.3 });
  assert.ok(!scanLogLine(0, r.odom, scan).includes("angle_min"));
});

test("the estimators over a simulated log reproduce the simulation's own errors", () => {
  const world = BUILTIN_MAPS.loop();
  const settings = { ...defaultSettings(), seed: 3 };
  /** @type {string[]} */
  const lines = [];
  const { sim } = runRoute(world, settings, 400, (s) =>
    lines.push(
      scanLogLine(s.time, s.odomPose, /** @type {any} */ (s.lastScan), {
        truth: s.truePose,
        full: lines.length === 0,
      }),
    ),
  );
  const ref = summarize(sim);
  const log = parseScanLog(lines.join("\n"));
  assert.equal(log.length, 401);
  const r = runScanLog(log, { map: world, start: world.start });
  const rms = /** @type {NonNullable<typeof r.rms>} */ (r.rms);
  // The log rounds poses to 1e-6 and ranges to 0.1 mm, and the simulation's
  // RMS skips the start pose, so agreement is close, not exact.
  close(rms.odometry, ref.odomRms, 0.01);
  close(rms.estimate, ref.estRms, 0.02);
  close(rms.mcl, ref.mclRms, 0.03);
  assert.ok(rms.estimate < rms.odometry);
  assert.equal(r.matchAttempts, 400);
  const csv = scanLogCsv(r).trim().split("\n");
  assert.equal(csv.length, 402);
  assert.equal(
    csv[0],
    "t,odometry_x,odometry_y,odometry_theta,estimate_x,estimate_y,estimate_theta,mcl_x,mcl_y,mcl_theta",
  );
  // Without a map: no MCL, the grid follows the path.
  const noMap = runScanLog(log, { scanMatching: false });
  assert.equal(noMap.mcl, null);
  assert.equal(noMap.matchAttempts, 0);
  assert.ok(Number.isNaN(/** @type {any} */ (noMap.rms).mcl));
  assert.ok(!scanLogCsv(noMap).includes("mcl_x"));
});

/** In-memory CLI io. */
function memoryIo(files = /** @type {Record<string, string>} */ ({})) {
  /** @type {Record<string, string | Uint8Array>} */
  const written = {};
  let out = "";
  let err = "";
  return {
    io: {
      out: (/** @type {string} */ t) => (out += t),
      err: (/** @type {string} */ t) => (err += t),
      readText: (/** @type {string} */ p) => {
        if (!(p in files)) {
          const e = new TypeError(`ENOENT: no such file, open '${p}'`);
          /** @type {any} */ (e).code = "ENOENT";
          throw e;
        }
        return files[p];
      },
      writeFile: (/** @type {string} */ p, /** @type {string | Uint8Array} */ d) => {
        written[p] = d;
      },
      mkdir: () => {},
    },
    written,
    get out() {
      return out;
    },
    get err() {
      return err;
    },
  };
}

test("CLI: run --scan-log, then localize it with and without the map", () => {
  const a = memoryIo();
  assert.equal(main(["run", "--map", "loop", "--steps", "150", "--scan-log", "s.jsonl"], a.io), 0);
  assert.match(a.out, /wrote 151 scans to s\.jsonl/);
  const files = { "s.jsonl": String(a.written["s.jsonl"]) };
  const b = memoryIo(files);
  assert.equal(
    main(["localize", "s.jsonl", "--map", "loop", "--motion", "omni", "--out", "o"], b.io),
    0,
  );
  assert.match(b.out, /scans {12}151 over 15\.0 s/);
  assert.match(
    b.out,
    /RMS error against the log's truth poses: odometry [\d.]+ m, scan matching [\d.]+ m, MCL [\d.]+ m/,
  );
  assert.ok(b.written["o/map.pgm"] instanceof Uint8Array);
  assert.match(String(b.written["o/trajectory.csv"]), /^t,odometry_x/);
  const c = memoryIo(files);
  assert.equal(main(["localize", "s.jsonl", "--json", "--mount", "0,0,0"], c.io), 0);
  const j = JSON.parse(c.out);
  assert.equal(j.scans, 151);
  assert.equal(j.final.mcl, null);
  assert.equal(j.rms.mcl, null);
  const d = memoryIo(files);
  assert.equal(main(["localize", "s.jsonl"], d.io), 0);
  assert.match(d.out, /MCL {12}-/);
  assert.match(d.out, /MCL -$/m);
});

test("CLI: localize usage errors exit 2", () => {
  const files = { "bad.jsonl": "{", "s.jsonl": `{"t":0,"odom":[0,0,0],${head},"ranges":[1,1]}` };
  for (const [args, re] of /** @type {[string[], RegExp][]} */ ([
    [["localize"], /needs one scan log file/],
    [["localize", "s.jsonl", "--motion", "warp"], /--motion must be/],
    [["localize", "s.jsonl", "--mount", "1,2"], /--mount must be three numbers/],
    [["localize", "bad.jsonl"], /bad\.jsonl: line 1: not valid JSON/],
    [["localize", "missing.jsonl"], /ENOENT/],
  ])) {
    const m = memoryIo(files);
    assert.equal(main(args, m.io), 2, args.join(" "));
    assert.match(m.err, re);
  }
});
