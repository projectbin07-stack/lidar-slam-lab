# Contributing

Bug reports, corrections to the algorithms and pull requests are welcome.

## Setup

```bash
git clone https://github.com/projectbin07-stack/lidar-slam-lab.git
cd lidar-slam-lab
npm ci            # dev tools only: TypeScript (type checking), Prettier, Node types
npm run serve     # http://127.0.0.1:8000/
```

The site itself has no runtime dependencies and no build step: `index.html`
loads the ES modules in `src/` directly. Any static file server works.

## Checks

All of these must pass; CI runs them on Ubuntu and Windows with Node 22 and 24,
plus a headless Chrome smoke test of the page.

```bash
npm run format:check
npm run typecheck
npm run coverage      # node:test with line, branch and function thresholds
```

`npm run check` runs all three.

## Guidelines

- Keep the shipped page free of runtime dependencies. Dev tools go in
  `devDependencies` and must not be imported from `src/`.
- Keep simulation logic out of `src/app.js`. Everything that computes something
  lives in a module that runs under Node and has tests.
- Every algorithm needs a test against a value you can derive by hand or a
  brute-force reference (see the distance transform test). A test that only
  checks that code runs is not enough.
- If you change an algorithm or a default parameter, update
  `docs/algorithms.md` and rerun `npm run benchmark`; put the before and after
  numbers in the pull request.
- Types are JSDoc comments checked by `tsc --checkJs` in strict mode.
- Small, focused commits with imperative messages.
