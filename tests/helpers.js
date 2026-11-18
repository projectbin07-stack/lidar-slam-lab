import assert from "node:assert/strict";

/**
 * Assert that two numbers agree within an absolute tolerance.
 * @param {number} actual
 * @param {number} expected
 * @param {number} [eps]
 */
export function close(actual, expected, eps = 1e-9) {
  assert.ok(
    Math.abs(actual - expected) <= eps,
    `expected ${actual} to be within ${eps} of ${expected}`,
  );
}
