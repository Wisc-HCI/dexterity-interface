/**
 * Clamp a numeric value to an inclusive range.
 *
 * Args:
 *   v (number): Input value.
 *   lo (number): Lower bound.
 *   hi (number): Upper bound.
 *
 * Returns:
 *   (number): Clamped value.
 */
export function clamp(v, lo, hi) {
  return Math.max(lo, Math.min(hi, v));
}

/**
 * Convert degrees to radians.
 *
 * Args:
 *   d (number): Degrees.
 *
 * Returns:
 *   (number): Radians.
 */
export function degToRad(d) {
  return (d * Math.PI) / 180.0;
}

/**
 * Convert radians to degrees.
 *
 * Args:
 *   r (number): Radians.
 *
 * Returns:
 *   (number): Degrees.
 */
export function radToDeg(r) {
  return (r * 180.0) / Math.PI;
}

/**
 * Format a numeric value for display in inputs / cards.
 *
 * - Rounds to a fixed number of decimals.
 * - Handles null/undefined/NaN gracefully.
 *
 * Args:
 *   value (any): Value to format.
 *   decimals (number): Number of decimal places.
 *
 * Returns:
 *   (string): Formatted string.
 */
export function format_number(value, decimals) {
  const v = Number(value);
  if (!Number.isFinite(v)) return "";
  return v.toFixed(decimals);
}

/**
 * Infer decimals from an HTML input step string.
 *
 * Args:
 *   step (string): e.g. "0.001", "1"
 *
 * Returns:
 *   (number): decimals
 */
export function decimals_from_step(step) {
  const s = String(step ?? "0.001");
  const dot = s.indexOf(".");
  return dot === -1 ? 0 : s.length - dot - 1;
}

/**
 * Convert quaternion to Euler angles (roll, pitch, yaw) in degrees.
 * Sequence: roll (X), pitch (Y), yaw (Z).
 *
 * Args:
 *   q (Array<number>): (4,) [qx,qy,qz,qw]
 *
 * Returns:
 *   (Array<number>): (3,) [roll_deg, pitch_deg, yaw_deg]
 */
export function quatToEulerDeg(q) {
  const qx = q[0],
    qy = q[1],
    qz = q[2],
    qw = q[3];

  const sinr_cosp = 2 * (qw * qx + qy * qz);
  const cosr_cosp = 1 - 2 * (qx * qx + qy * qy);
  const roll = Math.atan2(sinr_cosp, cosr_cosp);

  const sinp = 2 * (qw * qy - qz * qx);
  const pitch = Math.asin(clamp(sinp, -1, 1));

  const siny_cosp = 2 * (qw * qz + qx * qy);
  const cosy_cosp = 1 - 2 * (qy * qy + qz * qz);
  const yaw = Math.atan2(siny_cosp, cosy_cosp);

  return [radToDeg(roll), radToDeg(pitch), radToDeg(yaw)];
}

/**
 * Convert Euler angles (degrees) to quaternion [qx,qy,qz,qw].
 *
 * Args:
 *   e (Array<number>): (3,) [roll_deg,pitch_deg,yaw_deg]
 *
 * Returns:
 *   (Array<number>): (4,) [qx,qy,qz,qw]
 */
export function eulerDegToQuat(e) {
  const roll = degToRad(e[0]);
  const pitch = degToRad(e[1]);
  const yaw = degToRad(e[2]);

  const cy = Math.cos(yaw * 0.5);
  const sy = Math.sin(yaw * 0.5);
  const cp = Math.cos(pitch * 0.5);
  const sp = Math.sin(pitch * 0.5);
  const cr = Math.cos(roll * 0.5);
  const sr = Math.sin(roll * 0.5);

  const qw = cr * cp * cy + sr * sp * sy;
  const qx = sr * cp * cy - cr * sp * sy;
  const qy = cr * sp * cy + sr * cp * sy;
  const qz = cr * cp * sy - sr * sp * cy;

  return [qx, qy, qz, qw];
}
