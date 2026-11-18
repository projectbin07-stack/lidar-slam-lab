// Mapping between world metres (y up) and canvas pixels (y down).

/**
 * @typedef {object} Viewport
 * @property {number} scale Pixels per metre.
 * @property {number} offsetX Canvas x of world x = 0.
 * @property {number} offsetY Canvas y of world y = 0.
 */

/**
 * Fit a world rectangle [x0, x0 + w] x [y0, y0 + h] into a canvas, centred,
 * keeping the aspect ratio and leaving `pad` pixels on every side.
 * @param {number} x0
 * @param {number} y0
 * @param {number} w
 * @param {number} h
 * @param {number} canvasW
 * @param {number} canvasH
 * @param {number} pad
 * @returns {Viewport}
 */
export function fitViewport(x0, y0, w, h, canvasW, canvasH, pad) {
  const scale = Math.max(1e-6, Math.min((canvasW - 2 * pad) / w, (canvasH - 2 * pad) / h));
  const offsetX = (canvasW - w * scale) / 2 - x0 * scale;
  const offsetY = (canvasH + h * scale) / 2 + y0 * scale;
  return { scale, offsetX, offsetY };
}

/**
 * @param {Viewport} v
 * @param {number} x
 * @param {number} y
 * @returns {[number, number]}
 */
export function toCanvas(v, x, y) {
  return [v.offsetX + x * v.scale, v.offsetY - y * v.scale];
}

/**
 * @param {Viewport} v
 * @param {number} px
 * @param {number} py
 * @returns {[number, number]}
 */
export function toWorld(v, px, py) {
  return [(px - v.offsetX) / v.scale, (v.offsetY - py) / v.scale];
}

/**
 * Round to the nearest multiple of `step` and clamp to [0, max].
 * @param {number} value
 * @param {number} step
 * @param {number} max
 * @returns {number}
 */
export function snap(value, step, max) {
  const r = Math.round(value / step) * step;
  return Math.min(max, Math.max(0, Math.round(r * 1e6) / 1e6));
}
