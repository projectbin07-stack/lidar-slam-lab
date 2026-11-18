import test from "node:test";
import assert from "node:assert/strict";
import { cellToGrey, gridToPgm, mapYaml, trajectoriesCsv } from "../src/export.js";
import { createGrid } from "../src/grid.js";
import { main, VERSION } from "../src/cli.js";
import { serializeWorld } from "../src/world.js";
import { BUILTIN_MAPS } from "../src/maps.js";

test("grey levels follow the map_server trinary convention", () => {
  assert.equal(cellToGrey(0, 0.196, 0.65), 205); // p = 0.5, unknown
  assert.equal(cellToGrey(2, 0.196, 0.65), 0); // p = 0.88, occupied
  assert.equal(cellToGrey(-2, 0.196, 0.65), 254); // p = 0.12, free
});

test("PGM has a P5 header and writes the top row first", () => {
  const g = createGrid(0.3, 0.2, 0.1); // 3 x 2 cells
  g.logOdds[0] = 3; // bottom-left occupied
  g.logOdds[5] = -3; // top-right free
  const pgm = gridToPgm(g);
  const header = "P5\n3 2\n255\n";
  assert.equal(new TextDecoder().decode(pgm.slice(0, header.length)), header);
  assert.deepEqual(Array.from(pgm.slice(header.length)), [205, 205, 254, 0, 205, 205]);
});

test("map YAML names the image, resolution and origin", () => {
  const yaml = mapYaml(createGrid(1, 1, 0.05, -2, -1.5), "map.pgm");
  assert.match(yaml, /^image: map\.pgm$/m);
  assert.match(yaml, /^resolution: 0\.05$/m);
  assert.match(yaml, /^origin: \[-2, -1\.5, 0\.0\]$/m);
});

test("trajectory CSV has one column triple per trail", () => {
  const a = [
    { x: 0, y: 0, theta: 0 },
    { x: 1, y: 0.5, theta: 0.25 },
  ];
  const csv = trajectoriesCsv({ truth: a, est: a }, 0.1);
  const lines = csv.trim().split("\n");
  assert.equal(lines[0], "t,truth_x,truth_y,truth_theta,est_x,est_y,est_theta");
  assert.equal(lines[2], "0.100,1.0000,0.5000,0.25000,1.0000,0.5000,0.25000");
  assert.throws(() => trajectoriesCsv({ a, b: [a[0]] }, 0.1), /different length/);
  assert.equal(trajectoriesCsv({}, 0.1), "t\n");
});

/**
 * Run the CLI against an in-memory file system.
 * @param {string[]} argv
 * @param {Record<string, string>} [files]
 */
function run(argv, files = {}) {
  let out = "";
  let err = "";
  /** @type {Map<string, string | Uint8Array>} */
  const written = new Map();
  /** @type {string[]} */
  const dirs = [];
  const code = main(argv, {
    out: (t) => {
      out += t;
    },
    err: (t) => {
      err += t;
    },
    readText: (p) => {
      if (!(p in files)) throw new Error(`ENOENT: ${p}`);
      return files[p];
    },
    writeFile: (p, d) => {
      written.set(p, d);
    },
    mkdir: (p) => {
      dirs.push(p);
    },
  });
  return { code, out, err, written, dirs };
}

test("help, version and usage errors", () => {
  assert.equal(run(["--help"]).code, 0);
  assert.match(run(["--help"]).out, /Usage:/);
  assert.equal(run([]).code, 2);
  assert.equal(run(["--version"]).out, `${VERSION}\n`);
  const bad = run(["fly"]);
  assert.equal(bad.code, 2);
  assert.match(bad.err, /unknown command "fly"/);
  assert.match(run(["run", "--steps", "0"]).err, /--steps must be an integer/);
  assert.match(run(["run", "--odom-noise", "x"]).err, /--odom-noise must be a number/);
  assert.match(run(["run", "--unknown"]).err, /error:/);
  assert.match(run(["run", "--map", "moon"]).err, /unknown map "moon"/);
  assert.match(run(["export-map"]).err, /built-in map id/);
  assert.match(run(["benchmark", "--maps", "moon"]).err, /unknown map/);
});

test("maps lists the built-ins and export-map prints loadable JSON", () => {
  const list = run(["maps"]);
  assert.equal(list.code, 0);
  assert.match(list.out, /^office\s+Office\s+16 x 12 m/m);
  const exported = run(["export-map", "loop"]);
  assert.equal(exported.out.trim(), serializeWorld(BUILTIN_MAPS.loop()));
});

test("run prints a summary table and writes the three output files", () => {
  const r = run(["run", "--steps", "40", "--particles", "50", "--out", "results/"]);
  assert.equal(r.code, 0, r.err);
  assert.match(r.out, /stopped at step limit after 4\.0 s/);
  assert.match(r.out, /dead reckoning\s+\d+\.\d{3} m\s+\d+\.\d{3} m/);
  assert.match(r.out, /scan matches\s+100\.0 % accepted/);
  assert.deepEqual(r.dirs, ["results"]);
  assert.deepEqual(
    [...r.written.keys()],
    ["results/map.pgm", "results/map.yaml", "results/trajectory.csv"],
  );
  const csv = /** @type {string} */ (r.written.get("results/trajectory.csv"));
  assert.equal(csv.trim().split("\n").length, 1 + 41); // header, start pose, 40 steps
  assert.match(csv.split("\n")[0], /mcl_theta$/);
});

test("run --json reports NaN metrics as null and reads map files", () => {
  const files = {
    "room.json": JSON.stringify({
      version: 1,
      width: 6,
      height: 4,
      walls: [
        [0, 0, 6, 0],
        [6, 0, 6, 4],
        [6, 4, 0, 4],
        [0, 4, 0, 0],
        [3, 0, 3, 2],
      ],
      start: { x: 1, y: 1 },
      route: [
        [1, 3],
        [5, 3],
        [5, 1],
      ],
    }),
  };
  const r = run(
    ["run", "--map", "room.json", "--no-scan-matching", "--particles", "0", "--json"],
    files,
  );
  assert.equal(r.code, 0, r.err);
  const j = JSON.parse(r.out);
  assert.equal(j.map, "Untitled");
  assert.equal(j.routeFinished, true);
  assert.equal(j.scanMatching, false);
  assert.equal(j.mclRms, null);
  assert.equal(j.acceptRate, null);
  // 2 + 4 + 2 m of route, minus the 0.25 m arrival radius at the waypoints.
  assert.ok(j.distance > 7, `drove ${j.distance} m`);
  assert.match(run(["run", "--map", "missing.json"], files).err, /missing\.json: ENOENT/);
  assert.match(
    run(["run", "--map", "room.json"], { "room.json": "{}" }).err,
    /room\.json: unsupported map version/,
  );
});

test("global MCL run labels its row", () => {
  const r = run(["run", "--steps", "5", "--particles", "100", "--global", "--map", "loop"]);
  assert.match(r.out, /MCL \(global\)/);
});

test("benchmark prints one row per seed and a mean", () => {
  const r = run(["benchmark", "--maps", "loop", "--seeds", "2", "--steps", "30"]);
  assert.equal(r.code, 0, r.err);
  const rows = r.out.trim().split("\n");
  assert.match(rows.at(-3) ?? "", /^loop\s+1\s/);
  assert.match(rows.at(-2) ?? "", /^loop\s+2\s/);
  assert.match(rows.at(-1) ?? "", /^loop\s+mean\s/);
});
