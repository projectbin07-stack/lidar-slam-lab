# Security policy

## Supported versions

Only the latest release and the version deployed on GitHub Pages receive fixes.

## Reporting a vulnerability

Please do not open a public issue for a security problem. Use the private
vulnerability reporting of this repository on GitHub (Security tab, "Report a
vulnerability"):
https://github.com/projectbin07-stack/lidar-slam-lab/security/advisories/new

Include what you found, how to reproduce it and the version or commit. This is
a spare-time project maintained by one person, so there is no guaranteed
response time.

## Scope

The web page is static. It makes no network requests after loading its own
files, sends no data anywhere, uses no cookies and stores nothing. Map files
you load are read locally in the browser, parsed with `JSON.parse` and
validated (size, field types, coordinate ranges, wall count); they are never
evaluated as code or inserted as HTML. Files you download are generated in the
browser.

The CLI reads the map file you name and writes `map.pgm`, `map.yaml` and
`trajectory.csv` into the directory you give with `--out`. It opens no network
connections. `scripts/serve.js` is a development server bound to 127.0.0.1;
do not expose it to a network.

## Not a safety component

This is a teaching and experimentation tool. Nothing in it has been validated
on a real robot or sensor, and it must not be used to localise or steer real
hardware.
