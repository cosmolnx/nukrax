/* NUKRAX prototype V4 — 6-axis industrial arm.
   Geometry: rebuilt against the reference sketch (squat drum base on a foot plate, wedge shoulder casing with big bearing-housing caps,
   slender blade upper arm with rims + recessed carbon panel, thick banded forearm with a roll ring, knuckle wrist hanging vertical,
   three-finger claw with pivots and linkage). Motion: jerk-smoothed joint plans (heavy joints ramp longer) tracked by a
   torque-limited servo with load/reach-dependent inertia, viscous friction and a brake/stiction hold. */
import * as THREE from 'three';
import { M, carbon } from './materials.js';

export const K = { BX: 1470, ZB: 170, HS: 330, L1: 540, L2: 640, LT: 186, PLATE: 18 };
const PI = Math.PI, D = PI / 180;
const clamp = (v, a, b) => Math.max(a, Math.min(b, v));
const wrap = a => { while (a > PI) a -= 2 * PI; while (a < -PI) a += 2 * PI; return a; };

// ---------- claw geometry (shared with the contact solver) ----------
const R0 = 30, LP = 66, LD = 36, KAP = 28 * D, FRONT = 28 * D;
function padRadial(rho) { const p1 = -rho, p2 = p1 + KAP; const pz = -LP * Math.sin(p1) - 26 * Math.sin(p2); return R0 + pz - 5.5 * Math.cos(p2); }
export function contactRho(half, ang = 0) {
  const rc = half / Math.cos(ang); let lo = -60 * D, hi = 70 * D;
  for (let i = 0; i < 40; i++) { const m = (lo + hi) / 2; if (padRadial(m) > rc) hi = m; else lo = m; }
  return (lo + hi) / 2;
}

// ---------- geometry helpers (also used by the work cell) ----------
export const Mh = (g, m, x = 0, y = 0, z = 0) => { const o = new THREE.Mesh(g, m); o.position.set(x, y, z); o.castShadow = o.receiveShadow = true; return o; };
export const cylX = (r, len, m, x, y = 0, z = 0, rt = r) => { const g = new THREE.CylinderGeometry(rt, r, len, 40); g.rotateZ(-PI / 2); return Mh(g, m, x, y, z); };
export const cylZ = (r, len, m, x = 0, y = 0, z = 0) => { const g = new THREE.CylinderGeometry(r, r, len, 48); g.rotateX(PI / 2); return Mh(g, m, x, y, z); };
export const cylY = (r, len, m, x = 0, y = 0, z = 0, rt = r) => Mh(new THREE.CylinderGeometry(rt, r, len, 56), m, x, y, z);
export const box = (w, h, d, m, x, y, z) => Mh(new THREE.BoxGeometry(w, h, d), m, x, y, z);
function rrect(w, h, r) { const s = new THREE.Shape(), x = -w / 2, y = -h / 2; s.moveTo(x + r, y); s.lineTo(x + w - r, y); s.quadraticCurveTo(x + w, y, x + w, y + r); s.lineTo(x + w, y + h - r); s.quadraticCurveTo(x + w, y + h, x + w - r, y + h); s.lineTo(x + r, y + h); s.quadraticCurveTo(x, y + h, x, y + h - r); s.lineTo(x, y + r); s.quadraticCurveTo(x, y, x + r, y); return s; }
function ext(shape, d, bev, m, x = 0, y = 0, z = 0, seg = 14) { const g = new THREE.ExtrudeGeometry(shape, { depth: Math.max(0.1, d - 2 * bev), bevelEnabled: true, bevelThickness: bev, bevelSize: bev, bevelSegments: 3, curveSegments: seg }); g.translate(0, 0, -(d - 2 * bev) / 2); return Mh(g, m, x, y, z); }
export const rbox = (w, h, d, r, m, x, y, z) => ext(rrect(w, h, r), d, Math.min(2.5, r / 2), m, x, y, z, 6);
function blade(len, r0, r1) { const s = new THREE.Shape(); s.moveTo(0, -r0); s.quadraticCurveTo(len * 0.5, -(r0 + r1) / 2 - 3, len, -r1); s.absarc(len, 0, r1, -PI / 2, PI / 2, false); s.quadraticCurveTo(len * 0.5, (r0 + r1) / 2 + 3, 0, r0); s.absarc(0, 0, r0, PI / 2, PI * 1.5, false); return s; }
function lathe(pts, axis, m, x = 0, y = 0, z = 0) { const g = new THREE.LatheGeometry(pts.map(([r, t]) => new THREE.Vector2(r, t)), 64); if (axis === 'x') g.rotateZ(-PI / 2); return Mh(g, m, x, y, z); }
const _bz = {}, _bx = {}, _by = {};
export function boltsZ(parent, cx, cy, z, R, n, r = 4.2, m = M.machined) { const g = _bz[r] || (_bz[r] = new THREE.CylinderGeometry(r, r, 5, 6).rotateX(PI / 2)); for (let i = 0; i < n; i++) { const a = i / n * 2 * PI + 0.3, b = new THREE.Mesh(g, m); b.position.set(cx + Math.cos(a) * R, cy + Math.sin(a) * R, z); b.castShadow = true; parent.add(b); } }
function boltsX(parent, x, R, n, r = 3.4, m = M.machined) { const g = _bx[r] || (_bx[r] = new THREE.CylinderGeometry(r, r, 5, 6).rotateZ(-PI / 2)); for (let i = 0; i < n; i++) { const a = i / n * 2 * PI + 0.3, b = new THREE.Mesh(g, m); b.position.set(x, Math.sin(a) * R, Math.cos(a) * R); parent.add(b); } }
export function boltsY(parent, y, R, n, r = 6, m = M.machined) { const g = _by[r] || (_by[r] = new THREE.CylinderGeometry(r, r, 7, 6)); for (let i = 0; i < n; i++) { const a = i / n * 2 * PI + 0.2, b = new THREE.Mesh(g, m); b.position.set(Math.cos(a) * R, y, Math.sin(a) * R); b.castShadow = true; parent.add(b); } }
export function tube(parent, pts, r, m = M.rubber) { const c = new THREE.CatmullRomCurve3(pts.map(p => new THREE.Vector3(...p))); parent.add(Mh(new THREE.TubeGeometry(c, 36, r, 10), m)); return c; }
export const clampBlock = (parent, p, w = 14) => parent.add(rbox(w, 11, 18, 3, M.anodized, p.x, p.y, p.z));
// bearing housing seen from the side: flange, machined retaining ring + bolt circle, stepped cover, hub, shadow gap
function bearingCap(parent, z, R, s = 1, cover = M.satin) {
  parent.add(cylZ(R, 9, M.anodized, 0, 0, z)); parent.add(cylZ(R * 0.86, 5, M.machined, 0, 0, z + s * 5)); boltsZ(parent, 0, 0, z + s * 8, R * 0.72, 10, 3.6);
  parent.add(cylZ(R * 0.6, 7, cover, 0, 0, z + s * 9)); parent.add(cylZ(R * 0.36, 6, M.graphite, 0, 0, z + s * 14)); parent.add(cylZ(R * 0.14, 4, M.machined, 0, 0, z + s * 18));
  parent.add(cylZ(R * 0.62, 1.4, M.rubber, 0, 0, z + s * 12.8));
}

export function buildArm() {
  const TY = 186, sh = K.HS - TY;
  const root = new THREE.Group(); root.position.set(K.BX, 0, K.ZB);
  const cfA = carbon(1 / 58), cfB = carbon(1 / 30), cfC = carbon(1 / 22);
  // ================= base: foot plate with corner bolts, squat drum, flange (as the reference) =================
  root.add(rbox(352, 24, 352, 20, M.anodized, 0, 12, 0));
  for (const sx of [-1, 1]) for (const sz of [-1, 1]) { root.add(cylY(14, 14, M.graphite, sx * 146, 31, sz * 146)); root.add(cylY(8, 8, M.machined, sx * 146, 40, sz * 146)); }
  root.add(lathe([[0, 24], [176, 24], [176, 32], [150, 38], [128, 52], [122, 66], [118, 150], [124, 158], [124, 176], [0, 176]], 'y', M.graphite));
  root.add(cylY(172, 5, M.machined, 0, 27, 0)); for (const y of [74, 108, 140]) root.add(cylY(119.5, 3, M.rubber, 0, y, 0));
  root.add(cylY(129, 12, M.satin, 0, 182, 0)); boltsY(root, 189, 118, 18, 5);
  root.add(rbox(116, 100, 92, 9, M.charcoal, -184, 74, 40)); for (let i = 0; i < 6; i++) root.add(box(84, 3, 2, M.rubber, -184, 46 + i * 9, 87)); root.add(box(26, 2.6, 2, M.accent, -184, 106, 87));
  root.add(rbox(64, 44, 54, 6, M.anodized, -184, 150, 40)); boltsZ(root, -184, 150, 68, 19, 4, 3);
  const cab = tube(root, [[-152, 156, 40], [-138, 214, 60], [-98, 238, 30], [-60, 224, 6]], 6.5); tube(root, [[-162, 156, 52], [-148, 210, 74], [-102, 246, 44], [-64, 230, 14]], 4.6);
  for (const t of [0.12, 0.5, 0.88]) clampBlock(root, cab.getPoint(t), 12);
  // ================= turret + shoulder casing =================
  const turret = new THREE.Group(); turret.position.y = TY; root.add(turret);
  turret.add(lathe([[0, 0], [118, 0], [122, 6], [122, 30], [108, 38], [0, 38]], 'y', M.charcoal)); turret.add(cylY(122.5, 3.4, M.accent, 0, 33, 0));
  turret.add(ext(rrect(206, 222, 54), 148, 12, M.graphite, 4, 146, 0));
  turret.add(ext(rrect(214, 14, 30), 120, 4, M.satin, 0, 252, 0));
  for (const s of [1, -1]) {
    turret.add(ext(rrect(156, 176, 34), 8, 2, M.charcoal, 6, 138, s * 78));
    turret.add(ext(rrect(112, 130, 22), 5, 1.5, cfA, 6, 140, s * 82.5)); boltsZ(turret, 6, 140, s * 85, 66, 4, 3.4);
    turret.add(cylZ(96, 12, M.satin, 0, sh, s * 74));
    turret.add(cylZ(82, 96, M.anodized, 0, sh, s * 128)); turret.add(cylZ(64, 14, M.machined, 0, sh, s * 180)); boltsZ(turret, 0, sh, s * 188, 72, 12, 4.4);
    for (let i = 0; i < 6; i++) turret.add(cylZ(85, 2.4, M.graphite, 0, sh, s * (100 + i * 11)));
    turret.add(cylZ(46, 14, M.charcoal, 0, sh, s * 192)); turret.add(cylZ(30, 8, M.machined, 0, sh, s * 200));
    turret.add(box(30, 34, 46, M.anodized, -78, sh + 78, s * 118));                              // hard-stop lug
  }
  tube(turret, [[70, sh + 90, 128], [78, sh + 120, 80], [40, sh + 100, 30]], 4.6);
  // ================= shoulder joint + upper arm (slender blade) =================
  const shoulder = new THREE.Group(); shoulder.position.y = sh; turret.add(shoulder);
  shoulder.add(ext(blade(K.L1, 78, 58), 92, 9, M.graphite));
  for (const z of [47, -47]) {
    shoulder.add(ext(blade(K.L1 - 120, 54, 38), 6, 1.5, M.anodized, 62, 0, z));
    shoulder.add(ext(blade(K.L1 - 150, 46, 30), 5, 1.2, cfB, 70, 0, z * 1.05));
    for (const f of [0.3, 0.55, 0.8]) boltsZ(shoulder, K.L1 * f, 0, z * 1.1, 0, 0);
  }
  shoulder.add(rbox(14, 110, 100, 4, M.anodized, K.L1 * 0.9 - 20, 0, 0));
  const slope = Math.atan2(-20, K.L1);
  const rim = box(K.L1 - 150, 8, 70, M.charcoal, K.L1 / 2 + 20, 66, 0); rim.rotation.z = slope; shoulder.add(rim);
  const rim2 = box(K.L1 - 150, 8, 70, M.charcoal, K.L1 / 2 + 20, -66, 0); rim2.rotation.z = -slope; shoulder.add(rim2);
  for (const s of [1, -1]) bearingCap(shoulder, s * 50, 68, s);
  const duct = tube(shoulder, [[80, 76, 34], [K.L1 * 0.4, 84, 36], [K.L1 * 0.74, 66, 36], [K.L1 - 56, 56, 34]], 6.2);
  for (const t of [0.14, 0.42, 0.72, 0.94]) clampBlock(shoulder, duct.getPoint(t), 14);
  shoulder.add(rbox(66, 18, 66, 4, M.anodized, K.L1 - 140, 80, 0));
  shoulder.add(cylX(38, 124, M.anodized, K.L1 - 140, 112, 0)); shoulder.add(cylX(28, 10, M.machined, K.L1 - 74, 112, 0)); boltsX(shoulder, K.L1 - 68, 20, 6, 3);
  for (let i = 0; i < 6; i++) shoulder.add(cylX(41, 3, M.graphite, K.L1 - 190 + i * 11, 112, 0));
  shoulder.add(rbox(30, 22, 36, 4, M.charcoal, K.L1 - 190, 150, 0));
  // ================= elbow + forearm (thick banded tube with roll ring) =================
  const elbow = new THREE.Group(); elbow.position.x = K.L1; shoulder.add(elbow);
  elbow.add(cylZ(66, 112, M.anodized, 0, 0, 0)); for (const s of [1, -1]) bearingCap(elbow, s * 56, 56, s);
  elbow.add(box(26, 30, 40, M.anodized, 4, 76, 0));
  const bellEnd = 360;
  elbow.add(lathe([[0, 0], [64, 0], [74, 22], [78, 56], [76, 120], [66, 200], [58, 270], [53, 330], [51, bellEnd], [0, bellEnd]], 'x', M.graphite));
  for (const x of [112, 256]) elbow.add(cylX(79 - x * 0.08, 6, M.rubber, x));
  elbow.add(cylX(61, 16, M.satin, 205)); boltsX(elbow, 205, 60, 12, 3.2);
  elbow.add(cylX(53.5, 9, M.accent, 300));                                                      // accent: one thin datum ring
  elbow.add(rbox(150, 12, 62, 5, M.charcoal, 170, 70, 0)); elbow.add(box(40, 2, 18, M.machined, 150, 76.5, 0));
  const roll = new THREE.Group(); roll.position.x = bellEnd; elbow.add(roll);
  roll.add(cylX(56, 18, M.machined, 9)); roll.add(cylX(58, 8, M.rubber, 22)); boltsX(roll, 3, 54, 14, 3.2);
  const fa = K.L2 - 80;
  roll.add(cylX(46, fa - bellEnd - 28, M.graphite, 28 + (fa - bellEnd - 28) / 2, 0, 0, 40));
  const sleeve = new THREE.Mesh(new THREE.CylinderGeometry(46.8, 46.8, 120, 48, 1, true).rotateZ(-PI / 2), cfC); sleeve.position.x = 110; sleeve.castShadow = sleeve.receiveShadow = true; roll.add(sleeve);
  roll.add(cylX(48, 5, M.machined, 50)); roll.add(cylX(48, 5, M.machined, 170));
  const cb = tube(elbow, [[80, -66, 26], [bellEnd * 0.6, -60, 28], [bellEnd + 90, -46, 22], [K.L2 - 40, -36, 16]], 5); tube(elbow, [[84, -70, 10], [bellEnd * 0.6, -64, 12], [bellEnd + 90, -50, 8], [K.L2 - 44, -40, 4]], 4);
  for (const t of [0.18, 0.46, 0.74]) clampBlock(elbow, cb.getPoint(t), 14);
  // ================= wrist knuckle (roll housing -> pitch joint -> neck) =================
  const wr4 = new THREE.Group(); wr4.position.x = K.L2 - 80; elbow.add(wr4);
  wr4.add(lathe([[0, 0], [42, 0], [46, 10], [47, 56], [43, 80], [0, 80]], 'x', M.anodized)); wr4.add(cylX(48, 6, M.machined, 6)); boltsX(wr4, 9, 44, 10, 3);
  wr4.add(cylX(44.8, 26, cfC, 44));
  const wr5 = new THREE.Group(); wr5.position.x = 80; wr4.add(wr5);
  wr5.add(cylZ(52, 100, M.anodized, 0, 0, 0)); wr5.add(rbox(48, 70, 92, 8, M.graphite, 20, -10, 0));
  for (const s of [1, -1]) { bearingCap(wr5, s * 52, 42, s); wr5.add(cylZ(30, 3, cfB, 0, 0, s * 70)); }
  wr5.add(box(20, 18, 30, M.anodized, -34, 46, 0));
  wr5.add(cylX(31, 34, M.graphite, 24)); wr5.add(cylX(39, 7, M.machined, 40)); wr5.add(cylX(38, 3, M.rubber, 45)); boltsX(wr5, 49, 31, 8, 3);
  tube(wr5, [[-50, 20, 20], [-20, 60, 34], [26, 44, 30], [38, 22, 22]], 3.8);
  const wr6 = new THREE.Group(); wr6.position.x = 46; wr5.add(wr6);
  wr6.add(cylX(46, 46, M.graphite, 23, 0, 0, 44)); wr6.add(cylX(49, 5, M.machined, 46)); wr6.add(cylX(47.4, 3, M.accent, 38)); boltsX(wr6, 42, 36, 6, 3.4);
  for (const s of [1, -1]) wr6.add(box(26, 3, 30, cfB, 24, 0, s * 45.5));
  const gmark = new THREE.Object3D(); gmark.position.x = 46 + 96; wr6.add(gmark);
  // ================= claw: back finger + two front fingers; actuator pods, clevis, visible linkage =================
  const angles = [PI, FRONT, -FRONT], hinges = [], rods = [], rodGeo = new THREE.CylinderGeometry(2.2, 2.2, 1, 10).rotateZ(-PI / 2);
  for (const a of angles) {
    const f = new THREE.Group(); f.position.set(46, R0 * Math.sin(a), R0 * Math.cos(a)); f.rotation.x = -a; wr6.add(f);
    f.add(rbox(22, 16, 20, 3, M.anodized, 6, 0, -2)); f.add(cylX(8.4, 26, M.satin, 2, 0, 24)); f.add(rbox(10, 12, 12, 2, M.machined, 14, 0, 24));
    const rod = new THREE.Mesh(rodGeo, M.machined); rod.castShadow = true; f.add(rod); rods.push(rod);
    const h = new THREE.Group(); f.add(h); hinges.push(h);
    h.add(cylY(7.2, 16, M.machined)); h.add(Mh(new THREE.BoxGeometry(LP, 11, 8), M.graphite, LP / 2, 0, 0)); h.add(Mh(new THREE.BoxGeometry(LP * 0.62, 4.5, 10), M.anodized, LP * 0.55, 0, 0));
    h.add(cylY(2.8, 14, M.machined, 24, 0, 5.5));
    const d = new THREE.Group(); d.position.x = LP; d.rotation.y = KAP; h.add(d);
    d.add(cylY(5.6, 12, M.machined)); d.add(Mh(new THREE.BoxGeometry(LD + 4, 9, 7), M.graphite, (LD + 4) / 2 - 2, 0, 0));
    d.add(Mh(new THREE.BoxGeometry(14, 9, 4), M.pad, 26, 0, -5.5));
  }
  return { root, turret, shoulder, elbow, wr4, wr5, wr6, gmark, hinges, rods };
}

// ============================ motion ============================
const J = ['yaw', 'a', 'b', 'j4', 'j5', 'j6', 'grip'];
//          v max rad/s   a max rad/s^2   ramp fraction (heavier joints ramp longer)   tracker w   start delay (propagation)
const JL = { yaw: { v: 1.4, a: 1.25, r: 0.42, w: 6.5, d: 0 }, a: { v: 1.0, a: 1.1, r: 0.4, w: 7, d: 0.05 }, b: { v: 1.3, a: 1.65, r: 0.36, w: 8.5, d: 0.09 }, j4: { v: 2.2, a: 4.5, r: 0.3, w: 14, d: 0.13 }, j5: { v: 2, a: 4.2, r: 0.3, w: 15, d: 0.15 }, j6: { v: 3, a: 7, r: 0.28, w: 17, d: 0.17 }, grip: { v: 2.6, a: 10, r: 0.25, w: 24, d: 0.08 } };
const LIMITS = { yaw: [-1.2, 4.2], j4: [-3.0, 3.0], j5: [-2.25, 2.25], j6: [-3.3, 3.3] };
// smooth-ramp trapezoid (sinusoidal accel ramps = no jerk spikes): returns [p, v, a] for total distance d over window T
function ramp(t, T, r, d) {
  const ta = r * T, vp = d / (T - ta); t = clamp(t, 0, T);
  const up = tau => [vp * (tau / 2 - ta / (2 * PI) * Math.sin(PI * tau / ta)), vp * 0.5 * (1 - Math.cos(PI * tau / ta)), vp * PI / (2 * ta) * Math.sin(PI * tau / ta)];
  if (t < ta) return up(t);
  if (t <= T - ta) return [vp * (ta / 2 + (t - ta)), vp, 0];
  const [p, v, a] = up(T - t); return [d - p, v, -a];
}
const minWindow = (d, { v, a, r }) => d < 1e-6 ? 0 : Math.max(Math.sqrt(d * PI / (2 * a * r * (1 - r))), d / (v * (1 - r)));

export class Arm3D {
  constructor(parts) {
    this.p = parts; this.q = {}; this.qd = {}; this.cmd = {}; for (const k of J) { this.q[k] = 0; this.qd[k] = 0; this.cmd[k] = { p: 0, v: 0, a: 0 }; }
    this.dF = contactRho(9, FRONT) - contactRho(9, 0); this.spec = null; this.payload = 0; this.reach = 0.3; this._v = new THREE.Vector3();
    this.q.grip = this.cmd.grip.p = 30 * D;
  }
  get qT() { const o = {}; for (const k of J) o[k] = this.cmd[k].p; return o; }
  // ----- kinematics -----
  planar(r, h) {
    const dx = r, dy = h - K.HS, d = clamp(Math.hypot(dx, dy), Math.abs(K.L1 - K.L2) + 10, K.L1 + K.L2 - 6);
    const A = Math.acos(clamp((K.L1 * K.L1 + d * d - K.L2 * K.L2) / (2 * K.L1 * d), -1, 1));
    const a = Math.atan2(dy, dx) + A, er = K.L1 * Math.cos(a), eh = K.HS + K.L1 * Math.sin(a);
    return { a, b: Math.atan2(h - eh, r - er) };
  }
  solveZ(G, phi, roll = 0, lk = 0, ref = null) {
    const cl = Math.cos(lk), t = new THREE.Vector3(Math.sin(phi) * cl, -Math.cos(phi) * cl, Math.sin(lk)), W = new THREE.Vector3(G[0], G[1], G[2]).addScaledVector(t, -K.LT);
    const dx = W.x - K.BX, dz = W.z - K.ZB, yaw = Math.atan2(-dz, dx), { a, b } = this.planar(Math.hypot(dx, dz), W.y);
    const Rf = new THREE.Matrix4().makeRotationY(yaw).multiply(new THREE.Matrix4().makeRotationZ(b));
    const Z = new THREE.Vector3(0, 0, 1); Z.addScaledVector(t, -Z.dot(t)).normalize(); if (roll) Z.applyAxisAngle(t, roll); const Y = new THREE.Vector3().crossVectors(Z, t);
    const Rt = new THREE.Matrix4().makeBasis(t, Y, Z), m = Rf.clone().transpose().multiply(Rt).elements;
    let b5 = Math.atan2(Math.hypot(m[1], m[2]), m[0]), a4, c6;
    if (Math.sin(b5) > 1e-4) { a4 = Math.atan2(m[2], m[1]); c6 = Math.atan2(m[8], -m[4]); } else { a4 = 0; c6 = Math.atan2(-m[6], m[5]); }
    const s1 = [wrap(a4), b5, wrap(c6)], s2 = [wrap(a4 - PI), -b5, wrap(c6 + PI)];       // two equivalent wrist solutions: keep the one continuous with the current pose
    const r = ref || { j4: 0, j5: 0, j6: 0 }, cost = s => Math.abs(wrap(s[0] - r.j4)) + Math.abs(s[1] - r.j5) * 1.2 + Math.abs(wrap(s[2] - r.j6));
    const s = cost(s1) <= cost(s2) ? s1 : s2;
    return { yaw, a, b, j4: s[0], j5: s[1], j6: s[2] };
  }
  solve(sp) { const q = this.solveZ(sp.G, sp.phi, sp.roll || 0, sp.lk || 0, this.qT); for (const k in LIMITS) q[k] = clamp(q[k], LIMITS[k][0], LIMITS[k][1]); q.grip = sp.grip !== undefined ? sp.grip * D : this.cmd.grip.p; return q; }
  apply() {
    const q = this.q, p = this.p;
    p.turret.rotation.y = q.yaw; p.shoulder.rotation.z = q.a; p.elbow.rotation.z = q.b - q.a;
    p.wr4.rotation.x = q.j4; p.wr5.rotation.z = q.j5; p.wr6.rotation.x = q.j6;
    p.hinges[0].rotation.y = -q.grip; p.hinges[1].rotation.y = p.hinges[2].rotation.y = -(q.grip + this.dF);
    for (let i = 0; i < 3; i++) {   // actuator linkage follows the finger angle
      const th = -(q.grip + (i ? this.dF : 0)), c = Math.cos(th), sn = Math.sin(th), bx = 24 * c + 5.5 * sn, bz = -24 * sn + 5.5 * c, ax = 14, az = 24, dx = bx - ax, dz = bz - az, L = Math.hypot(dx, dz), r = p.rods[i];
      r.position.set((ax + bx) / 2, 0, (az + bz) / 2); r.rotation.y = Math.atan2(-dz, dx); r.scale.x = L;
    }
    p.root.updateMatrixWorld(true);
    const g = p.gmark.getWorldPosition(this._v); this.reach = clamp(Math.hypot(g.x - K.BX, g.z - K.ZB) / 1100, 0.1, 1);
  }
  gWorld() { return this.p.gmark.getWorldPosition(new THREE.Vector3()); }
  toolMatrix() { return this.p.wr6.matrixWorld; }
  // ----- motion commands (closures for a script runner) -----
  move(spec, mode = 'j', Tmin = 0.5) {
    let st = null; const self = this;
    return dt => {
      if (!st) st = self._begin(spec, mode, Tmin);
      st.t += dt;
      if (st.mode === 'l') {
        const [s] = ramp(st.t, st.T, 0.36, 1), q = self.solve(self._lerp(st.s0, spec, s));
        for (const k of J) { const c = self.cmd[k], np = (k === 'j6' || k === 'j4') ? c.p + wrap(q[k] - c.p) : q[k]; if (dt > 1e-6) c.v = (np - c.p) / dt * 0.7 + c.v * 0.3; c.a = 0; c.p = np; }
      } else for (const k of J) {
        const L = JL[k], d = st.q1[k] - st.q0[k], tj = st.t - L.d * st.T, W = st.T * (1 - L.d), c = self.cmd[k];
        if (Math.abs(d) < 1e-7) { c.v = 0; c.a = 0; continue; }
        const [p, v, a] = ramp(tj, W, L.r, Math.abs(d)), sg = Math.sign(d); c.p = st.q0[k] + sg * p; c.v = sg * v; c.a = sg * a;
      }
      const done = st.t >= st.T;
      if (done) { for (const k of J) { const c = self.cmd[k]; if (st.mode === 'j') c.p = st.q1[k]; c.v = 0; c.a = 0; } self.spec = Object.assign({}, spec, spec.grip === undefined && self.spec ? { grip: self.spec.grip } : {}); }
      return done && self.settled() || st.t > st.T + 1.2;
    };
  }
  gripTo(deg, Tmin = 0.4) {
    let st = null; const self = this;
    return dt => {
      if (!st) { const q0 = self.cmd.grip.p, q1 = deg * D, d = Math.abs(q1 - q0); st = { t: 0, q0, q1, d, T: Math.max(Tmin, minWindow(d, JL.grip)) }; }
      st.t += dt; const c = self.cmd.grip, [p, v, a] = ramp(st.t, st.T, JL.grip.r, st.d), sg = Math.sign(st.q1 - st.q0) || 1;
      c.p = st.q0 + sg * p; c.v = sg * v; c.a = sg * a;
      if (st.t >= st.T) { c.p = st.q1; c.v = 0; c.a = 0; if (self.spec) self.spec.grip = deg; }
      return st.t >= st.T && Math.abs(self.q.grip - c.p) < 0.02 || st.t > st.T + 0.8;
    };
  }
  _begin(spec, mode, Tmin) {
    if (mode === 'l' && !this.spec) mode = 'j';
    const st = { t: 0, mode, q0: this.qT };
    if (mode === 'l') {
      const s0 = this.spec; st.s0 = Object.assign({}, s0, { grip: s0.grip !== undefined ? s0.grip : this.cmd.grip.p / D }); if (spec.grip === undefined) spec.grip = st.s0.grip;
      const dist = Math.hypot(spec.G[0] - s0.G[0], spec.G[1] - s0.G[1], spec.G[2] - s0.G[2]);
      const dang = Math.max(Math.abs(spec.phi - s0.phi), Math.abs((spec.roll || 0) - (s0.roll || 0)), Math.abs((spec.lk || 0) - (s0.lk || 0)));
      st.T = Math.max(Tmin, minWindow(dist, { v: 430, a: 700, r: 0.36 }), minWindow(dang, { v: 1.2, a: 3, r: 0.36 }));
    } else {
      st.q1 = this.solve(spec); for (const k of ['j4', 'j6']) st.q1[k] = st.q0[k] + wrap(st.q1[k] - st.q0[k]);
      let T = Tmin; for (const k of J) T = Math.max(T, minWindow(Math.abs(st.q1[k] - st.q0[k]), JL[k]) / (1 - JL[k].d)); st.T = T;
    }
    return st;
  }
  _lerp(a, b, s) { const o = { grip: a.grip + (b.grip - a.grip) * s }; o.G = a.G.map((v, i) => v + (b.G[i] - v) * s); o.phi = a.phi + (b.phi - a.phi) * s; o.roll = (a.roll || 0) + ((b.roll || 0) - (a.roll || 0)) * s; o.lk = (a.lk || 0) + ((b.lk || 0) - (a.lk || 0)) * s; return o; }
  abort() {   // brakes: target the stopping point implied by the current velocity; the servo's torque limit decides the real stopping distance
    for (const k of J) { const c = this.cmd[k], L = JL[k], ds = this.qd[k] * Math.abs(this.qd[k]) / (2 * L.a * 1.4); c.p = this.q[k] + ds; c.v = 0; c.a = 0; }
    this.spec = null;
  }
  settled() { for (const k of J) if (Math.abs(this.cmd[k].p - this.q[k]) > 0.012 || Math.abs(this.qd[k]) > 0.05) return false; return true; }
  init(spec) { this.spec = Object.assign({}, spec); const q = this.solve(spec); for (const k of J) { this.q[k] = this.cmd[k].p = q[k]; this.qd[k] = 0; this.cmd[k].v = this.cmd[k].a = 0; } this.apply(); }
  inertia(k) { const e = this.reach * this.reach, pl = this.payload; return k === 'yaw' ? 0.75 + 1.0 * e + 0.5 * pl * e : k === 'a' ? 0.8 + 0.8 * e + 0.45 * pl : k === 'b' ? 0.75 + 0.5 * e + 0.4 * pl : k === 'grip' ? 1 : 1 + 0.7 * pl; }
  step(dt) {   // torque-limited servo: feed-forward accel + PD, saturated by motor torque, viscous friction, brake hold at rest
    for (let s = 0; s < 2; s++) {
      const h = dt / 2;
      for (const k of J) {
        const L = JL[k], I = this.inertia(k), c = this.cmd[k], w = L.w / Math.sqrt(I), z = 0.85, e = c.p - this.q[k], ed = c.v - this.qd[k];
        let acc = c.a + w * w * e + 2 * z * w * ed; const am = L.a * 1.6 / I; acc = clamp(acc, -am, am) - 0.55 * this.qd[k];
        this.qd[k] += acc * h; this.q[k] += this.qd[k] * h;
        if (Math.abs(c.v) < 1e-5 && Math.abs(e) < 0.0007 && Math.abs(this.qd[k]) < 0.004) { this.q[k] = c.p; this.qd[k] = 0; }
      }
    }
    this.apply();
  }
}
