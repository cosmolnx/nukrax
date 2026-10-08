/* NUKRAX prototype V2 — 6-axis industrial arm (Three.js).
   Geometry is procedural, modelled on the reference sketch: slew base + turret, flat-plate upper arm with
   inset panel, tapered tubular forearm with bands, big wrist joint, and a hinged claw gripper.
   Motion: per-joint velocity/acceleration limits, staggered coordination, and a second-order (mass/spring)
   tracker per joint so every move accelerates, cruises, decelerates and settles with a slight overshoot. */
import * as THREE from 'three';

export const K = { BX: 1470, ZB: 170, HS: 330, L1: 600, L2: 570, LT: 186, PLATE: 18 };
const PI = Math.PI, D = PI / 180;
const clamp = (v, a, b) => Math.max(a, Math.min(b, v));
const wrap = a => { while (a > PI) a -= 2 * PI; while (a < -PI) a += 2 * PI; return a; };
const smoother = t => { t = clamp(t, 0, 1); return t * t * t * (t * (t * 6 - 15) + 10); };
function trap(u, f = 0.28) { u = clamp(u, 0, 1); const v = 1 / (1 - f); if (u < f) return v * u * u / (2 * f); if (u < 1 - f) return v * (f / 2 + (u - f)); const w = 1 - u; return 1 - v * w * w / (2 * f); }
const prof = u => 0.55 * trap(u) + 0.45 * smoother(u);           // accel -> cruise -> decel, jerk-softened
const trapT = (d, v, a) => d < v * v / a ? 2 * Math.sqrt(d / a) : d / v + v / a;

// ---------- claw geometry (shared by builder + contact solver) ----------
const R0 = 30, LP = 66, LD = 36, KAP = 28 * D, FRONT = 28 * D;
function padRadial(rho) { const p1 = -rho, p2 = p1 + KAP; const pz = -LP * Math.sin(p1) - 26 * Math.sin(p2); return R0 + pz - 5.5 * Math.cos(p2); }
export function contactRho(half, ang = 0) { // finger angle (rad) at which the pad's inner face touches a plate of half-thickness `half`
  const rc = half / Math.cos(ang); let lo = -60 * D, hi = 70 * D;
  for (let i = 0; i < 40; i++) { const m = (lo + hi) / 2; if (padRadial(m) > rc) hi = m; else lo = m; }
  return (lo + hi) / 2;
}

// ---------- materials: NUKRAX palette (Color_System_Specification) ----------
// primary paint = text-muted #898781 | secondary panels = border-strong #4D4D4C | dark structure = surface-control #2D2D2D / overlay #20201F
// machined metal = text-secondary #C3C2B7 | rubber = bg-recessed #111111 | accent = #ABBED3 (used twice, thinly)
const MM = (c, r, m) => new THREE.MeshStandardMaterial({ color: c, roughness: r, metalness: m });
const mats = {
  paint: MM(0x898781, 0.5, 0.42), panel: MM(0x4d4d4c, 0.55, 0.5), joint: MM(0x2d2d2d, 0.42, 0.85), deep: MM(0x20201f, 0.5, 0.8),
  steel: MM(0xc3c2b7, 0.3, 1), rubber: MM(0x111111, 0.92, 0), accent: MM(0xabbed3, 0.5, 0.25), pad: MM(0x1a1a19, 0.85, 0.1),
};
export { mats };
const M = (g, m, x = 0, y = 0, z = 0) => { const o = new THREE.Mesh(g, m); o.position.set(x, y, z); o.castShadow = o.receiveShadow = true; return o; };
const cylX = (r, len, m, x, y = 0, z = 0, rt = r) => { const g = new THREE.CylinderGeometry(rt, r, len, 40); g.rotateZ(-PI / 2); return M(g, m, x, y, z); };
const cylZ = (r, len, m, x = 0, y = 0, z = 0) => { const g = new THREE.CylinderGeometry(r, r, len, 44); g.rotateX(PI / 2); return M(g, m, x, y, z); };
const cylY = (r, len, m, x = 0, y = 0, z = 0, rt = r) => M(new THREE.CylinderGeometry(rt, r, len, 56), m, x, y, z);
function rrect(w, h, r) { const s = new THREE.Shape(), x = -w / 2, y = -h / 2; s.moveTo(x + r, y); s.lineTo(x + w - r, y); s.quadraticCurveTo(x + w, y, x + w, y + r); s.lineTo(x + w, y + h - r); s.quadraticCurveTo(x + w, y + h, x + w - r, y + h); s.lineTo(x + r, y + h); s.quadraticCurveTo(x, y + h, x, y + h - r); s.lineTo(x, y + r); s.quadraticCurveTo(x, y, x + r, y); return s; }
function ext(shape, d, bev, m, x = 0, y = 0, z = 0) { const g = new THREE.ExtrudeGeometry(shape, { depth: d - 2 * bev, bevelEnabled: true, bevelThickness: bev, bevelSize: bev, bevelSegments: 2, curveSegments: 14 }); g.translate(0, 0, -(d - 2 * bev) / 2); return M(g, m, x, y, z); }
const rbox = (w, h, d, r, m, x, y, z) => ext(rrect(w, h, r), d, 2.5, m, x, y, z);
const box = (w, h, d, m, x, y, z) => M(new THREE.BoxGeometry(w, h, d), m, x, y, z);
function plateShape(len, r0, r1) { const s = new THREE.Shape(); s.moveTo(0, -r0); s.lineTo(len, -r1); s.absarc(len, 0, r1, -PI / 2, PI / 2, false); s.lineTo(0, r0); s.absarc(0, 0, r0, PI / 2, PI * 1.5, false); return s; }
const _bz = {}, _bx = {};
function boltsZ(parent, cx, cy, z, R, n, r = 4.5, m = mats.steel) { const g = _bz[r] || (_bz[r] = new THREE.CylinderGeometry(r, r, 6, 6).rotateX(PI / 2)); for (let i = 0; i < n; i++) { const a = i / n * 2 * PI + 0.3, b = new THREE.Mesh(g, m); b.position.set(cx + Math.cos(a) * R, cy + Math.sin(a) * R, z); b.castShadow = true; parent.add(b); } }
function boltsX(parent, x, R, n, r = 3.5, m = mats.steel) { const g = _bx[r] || (_bx[r] = new THREE.CylinderGeometry(r, r, 5, 6).rotateZ(-PI / 2)); for (let i = 0; i < n; i++) { const a = i / n * 2 * PI + 0.3, b = new THREE.Mesh(g, m); b.position.set(x, Math.sin(a) * R, Math.cos(a) * R); parent.add(b); } }
const seam = (parent, x, R, w = 3) => parent.add(cylX(R, w, mats.rubber, x));
function finsZ(parent, n, r, z0, dz, x = 0, y = 0) { for (let i = 0; i < n; i++) parent.add(cylZ(r, 2.6, mats.deep, x, y, z0 + i * dz)); }
function tube(parent, pts, r, m = mats.rubber) { const c = new THREE.CatmullRomCurve3(pts.map(p => new THREE.Vector3(...p))); const t = M(new THREE.TubeGeometry(c, 32, r, 10), m); parent.add(t); return c; }
function clamp3(parent, p, w = 14) { parent.add(rbox(w, 12, 20, 3, mats.joint, p.x, p.y, p.z)); }
// joint cap stack: layered disc, machined ring, bolt circle, hub (reads as a real cast cover at landing-page scale)
function jointCap(parent, z, R, sgn = 1) { parent.add(cylZ(R, 7, mats.deep, 0, 0, z)); parent.add(cylZ(R * 0.82, 4, mats.steel, 0, 0, z + sgn * 4)); boltsZ(parent, 0, 0, z + sgn * 7, R * 0.68, 8, 3.8); parent.add(cylZ(R * 0.36, 6, mats.joint, 0, 0, z + sgn * 8)); parent.add(cylZ(R * 0.16, 4, mats.steel, 0, 0, z + sgn * 11)); }

export function buildArm() {
  const root = new THREE.Group(); root.position.set(K.BX, 0, K.ZB);
  // ---- fixed base: chamfered plate, bolt circle, ribbed pedestal, slew bearing ----
  root.add(cylY(214, 22, mats.joint, 0, 11, 0, 208)); root.add(cylY(172, 5, mats.steel, 0, 24, 0, 166));
  for (let i = 0; i < 8; i++) { const a = i / 8 * 2 * PI + 0.2; root.add(cylY(11, 12, mats.deep, Math.cos(a) * 190, 28, Math.sin(a) * 190)); root.add(cylY(7, 5, mats.steel, Math.cos(a) * 190, 35, Math.sin(a) * 190)); }
  root.add(cylY(114, 128, mats.paint, 0, 88, 0, 106));
  for (let i = 0; i < 8; i++) { const a = i / 8 * 2 * PI + PI / 8; const r = box(12, 84, 30, mats.panel, Math.cos(a) * 112, 70, Math.sin(a) * 112); r.rotation.y = -a; r.castShadow = true; root.add(r); }
  for (const y of [50, 120]) root.add(cylY(116, 4, mats.rubber, 0, y, 0));
  root.add(cylY(126, 26, mats.deep, 0, 162, 0)); root.add(cylY(128, 5, mats.steel, 0, 177, 0)); boltsZ(root, 0, 0, 0, 0, 0);
  for (let i = 0; i < 24; i++) { const a = i / 24 * 2 * PI; root.add(cylY(4, 8, mats.joint, Math.cos(a) * 119, 162, Math.sin(a) * 119)); }
  root.add(rbox(116, 104, 90, 9, mats.panel, -178, 64, 56)); for (let i = 0; i < 6; i++) root.add(box(86, 3, 2, mats.rubber, -178, 36 + i * 9, 102)); root.add(box(30, 3, 2, mats.accent, -178, 100, 102));
  root.add(rbox(66, 46, 56, 6, mats.joint, -178, 142, 56)); boltsZ(root, -178, 142, 87, 20, 4, 3);
  const cab = tube(root, [[-150, 150, 56], [-136, 200, 70], [-96, 222, 40], [-52, 208, 8]], 6); tube(root, [[-160, 150, 66], [-146, 196, 82], [-100, 232, 50], [-58, 214, 16]], 4.5);
  for (const t of [0.15, 0.5, 0.85]) clamp3(root, cab.getPoint(t), 12);
  // ---- turret (yaw about Y): housing, side motor pods with cooling fins ----
  const turret = new THREE.Group(); turret.position.y = 163; root.add(turret);
  turret.add(cylY(104, 30, mats.panel, 0, 22, 0, 100)); turret.add(cylY(106, 4, mats.accent, 0, 40, 0)); // accent #1: a single thin band
  turret.add(rbox(184, 180, 152, 26, mats.paint, 0, 112, 0)); turret.add(rbox(152, 10, 130, 6, mats.panel, 0, 204, 0));
  for (const x of [-60, 60]) turret.add(box(12, 150, 156, mats.panel, x * 1.25, 112, 0));
  boltsZ(turret, 0, 0, 78, 0, 0);
  const sh = K.HS - 163;
  for (const s of [1, -1]) {
    turret.add(cylZ(76, 54, mats.joint, 0, sh, s * 100)); turret.add(cylZ(56, 16, mats.steel, 0, sh, s * 134)); boltsZ(turret, 0, sh, s * 143, 64, 8, 4.2);
    finsZ(turret, 5, 79, s * 78, s * 6, 0, sh); turret.add(rbox(36, 30, 34, 4, mats.deep, 62, sh + 70, s * 100)); turret.add(cylZ(26, 8, mats.deep, 0, sh, s * 146));
  }
  tube(turret, [[62, sh + 84, 100], [70, sh + 110, 60], [40, sh + 90, 20]], 4);
  // ---- shoulder / upper arm: layered structural plate ----
  const shoulder = new THREE.Group(); shoulder.position.y = sh; turret.add(shoulder);
  shoulder.add(ext(plateShape(K.L1, 86, 64), 96, 6, mats.paint));
  for (const z of [51, -51]) { shoulder.add(ext(plateShape(K.L1 - 110, 66, 48), 9, 2, mats.panel, 46, 0, z)); }
  const slope = Math.atan2(-22, K.L1);
  const spine = box(K.L1 - 130, 10, 66, mats.panel, K.L1 * 0.5 + 10, 78, 0); spine.rotation.z = slope; shoulder.add(spine);
  const trayB = box(K.L1 - 130, 10, 66, mats.panel, K.L1 * 0.5 + 10, -78, 0); trayB.rotation.z = -slope; shoulder.add(trayB);
  for (const f of [0.28, 0.5, 0.72]) shoulder.add(rbox(14, 150 - f * 36, 108, 4, mats.deep, K.L1 * f, 0, 0));
  for (const z of [56, -56]) { for (const f of [0.28, 0.5, 0.72]) boltsZ(shoulder, K.L1 * f, 40, z, 0, 0); }
  for (const s of [1, -1]) jointCap(shoulder, s * 51.5, 70, s);
  const duct = tube(shoulder, [[70, 84, 38], [K.L1 * 0.35, 90, 40], [K.L1 * 0.7, 74, 40], [K.L1 - 60, 62, 38]], 6.5); tube(shoulder, [[74, 84, 28], [K.L1 * 0.35, 94, 30], [K.L1 * 0.7, 78, 30], [K.L1 - 64, 66, 30]], 4.5);
  for (const t of [0.12, 0.38, 0.64, 0.9]) { const p = duct.getPoint(t); clamp3(shoulder, p, 14); }
  // elbow motor pod riding on the upper arm
  shoulder.add(rbox(60, 20, 70, 4, mats.panel, K.L1 - 120, 92, 0));
  const pod = cylX(40, 128, mats.joint, K.L1 - 120, 126, 0); shoulder.add(pod); shoulder.add(cylX(30, 10, mats.steel, K.L1 - 54, 126, 0)); boltsX(shoulder, K.L1 - 49, 22, 6, 3);
  for (let i = 0; i < 7; i++) shoulder.add(cylX(43, 3, mats.deep, K.L1 - 170 + i * 11, 126, 0));
  shoulder.add(rbox(34, 26, 40, 4, mats.deep, K.L1 - 150, 166, 0));
  // ---- elbow ----
  const elbow = new THREE.Group(); elbow.position.x = K.L1; shoulder.add(elbow);
  elbow.add(cylZ(70, 130, mats.joint, 0, 0, 0)); for (const s of [1, -1]) jointCap(elbow, s * 66, 56, s);
  elbow.add(cylX(72, 36, mats.paint, 40, 0, 0, 64));
  // ---- forearm: housing -> split-shell tube -> wrist motor, with service cover + cable bundle ----
  const fl = K.L2 - 60;
  const tubeM = cylX(60, fl - 60, mats.paint, 60 + (fl - 60) / 2, 0, 0, 38); elbow.add(tubeM);
  for (const t of [0.1, 0.3, 0.52, 0.76]) { const x = 60 + (fl - 60) * t, r = 60 - 22 * t; elbow.add(cylX(r + 4, 12, mats.deep, x)); seam(elbow, x + 15, r + 1); }
  elbow.add(cylX(46, 7, mats.accent, 60 + (fl - 60) * 0.2, 0, 0, 47));                       // accent #2
  const cov = rbox(190, 12, 70, 4, mats.panel, 60 + (fl - 60) * 0.5, 50, 0); cov.rotation.z = 0.045; elbow.add(cov); boltsZ(elbow, 60 + (fl - 60) * 0.5 - 80, 54, 33, 0, 0);
  for (const dx of [-72, 72]) for (const dz of [-27, 27]) { const b = cylY(4, 5, mats.steel, 60 + (fl - 60) * 0.5 + dx, 58.5 + dx * 0.045, dz); elbow.add(b); }
  elbow.add(rbox(52, 24, 38, 4, mats.deep, 60 + (fl - 60) * 0.5, 60, 0));
  boltsX(elbow, 58, 62, 12);
  const cb = tube(elbow, [[70, -58, 22], [fl * 0.35, -54, 26], [fl * 0.7, -42, 22], [fl + 6, -30, 16]], 5); tube(elbow, [[70, -62, 8], [fl * 0.35, -58, 12], [fl * 0.7, -46, 8], [fl + 6, -34, 3]], 4);
  for (const t of [0.2, 0.5, 0.8]) clamp3(elbow, cb.getPoint(t), 14);
  elbow.add(rbox(120, 38, 50, 8, mats.joint, fl - 90, 0, 0, 0)); elbow.children[elbow.children.length - 1].scale.set(1, 1, 1);
  // ---- wrist: roll housing, pitch joint with layered caps, tool flange ----
  const wr4 = new THREE.Group(); wr4.position.x = K.L2 - 62; elbow.add(wr4);
  wr4.add(cylX(42, 64, mats.paint, 32, 0, 0, 38)); wr4.add(cylX(46, 8, mats.steel, 2)); boltsX(wr4, 6, 40, 10, 3); wr4.add(cylX(40, 4, mats.rubber, 36));
  const wr5 = new THREE.Group(); wr5.position.x = 62; wr4.add(wr5);
  wr5.add(cylZ(50, 98, mats.joint, 0, 0, 0)); for (const s of [1, -1]) jointCap(wr5, s * 54, 40, s);
  wr5.add(cylX(31, 34, mats.paint, 22)); wr5.add(cylX(38, 9, mats.steel, 40)); boltsX(wr5, 46, 30, 6, 3);
  const wr6 = new THREE.Group(); wr6.position.x = 44; wr5.add(wr6);
  wr6.add(cylX(44, 44, mats.joint, 22, 0, 0, 42)); wr6.add(cylX(48, 6, mats.steel, 45)); wr6.add(cylX(47, 3, mats.accent, 39)); boltsX(wr6, 41, 36, 6, 3.5);
  const gmark = new THREE.Object3D(); gmark.position.x = 44 + 96; wr6.add(gmark);
  // ---- claw: back finger + two front fingers (front = tool +Z); actuator pods + visible linkage rods ----
  const angles = [PI, FRONT, -FRONT], hinges = [], rods = [], rodGeo = new THREE.CylinderGeometry(2.2, 2.2, 1, 10).rotateZ(-PI / 2);
  for (const a of angles) {
    const f = new THREE.Group(); f.position.set(44, R0 * Math.sin(a), R0 * Math.cos(a)); f.rotation.x = -a; wr6.add(f);
    f.add(rbox(20, 15, 18, 3, mats.steel, 6, 0, -2));
    f.add(cylX(8, 24, mats.deep, 2, 0, 24)); f.add(rbox(10, 12, 12, 2, mats.steel, 14, 0, 24));    // actuator pod + clevis
    const rod = new THREE.Mesh(rodGeo, mats.steel); rod.castShadow = true; f.add(rod); rods.push(rod);
    const h = new THREE.Group(); f.add(h); hinges.push(h);
    h.add(cylY(7, 15, mats.deep));
    h.add(M(new THREE.BoxGeometry(LP, 11, 8), mats.paint, LP / 2, 0, 0));
    h.add(M(new THREE.BoxGeometry(LP * 0.6, 4, 10), mats.deep, LP * 0.55, 0, 0));
    h.add(cylY(2.8, 14, mats.steel, 24, 0, 5.5));
    const d = new THREE.Group(); d.position.x = LP; d.rotation.y = KAP; h.add(d);
    d.add(cylY(5.5, 12, mats.steel));
    d.add(M(new THREE.BoxGeometry(LD + 4, 9, 7), mats.paint, (LD + 4) / 2 - 2, 0, 0));
    d.add(M(new THREE.BoxGeometry(14, 9, 4), mats.pad, 26, 0, -5.5));
  }
  return { root, turret, shoulder, elbow, wr4, wr5, wr6, gmark, hinges, rods };
}

// ---------- joint limits [vmax rad/s, amax rad/s^2], tracker natural frequency, coordination delay ----------
const J = ['yaw', 'a', 'b', 'j4', 'j5', 'j6', 'grip'];
const LIM = { yaw: [1.25, 2.2], a: [0.9, 1.6], b: [1.1, 2.0], j4: [2.6, 7], j5: [2.6, 7], j6: [3.4, 9], grip: [2.4, 11] };
const WN = { yaw: 7.5, a: 8.5, b: 10.5, j4: 15, j5: 17, j6: 19, grip: 24 };
const DELAY = { yaw: 0, a: 0.06, b: 0.1, j4: 0.14, j5: 0.15, j6: 0.17, grip: 0.1 };
const ZETA = 0.74;

export class Arm3D {
  constructor(parts) {
    this.p = parts; this.q = {}; this.qd = {}; this.qT = {}; for (const k of J) { this.q[k] = 0; this.qd[k] = 0; this.qT[k] = 0; }
    this.dF = contactRho(9, FRONT) - contactRho(9, 0); this.spec = null; this.mv = null;
    this.q.grip = this.qT.grip = 30 * D;
  }
  // ----- kinematics -----
  planar(r, h) {
    const dx = r, dy = h - K.HS, d = clamp(Math.hypot(dx, dy), Math.abs(K.L1 - K.L2) + 10, K.L1 + K.L2 - 6);
    const A = Math.acos(clamp((K.L1 * K.L1 + d * d - K.L2 * K.L2) / (2 * K.L1 * d), -1, 1));
    const a = Math.atan2(dy, dx) + A, er = K.L1 * Math.cos(a), eh = K.HS + K.L1 * Math.sin(a);
    return { a, b: Math.atan2(h - eh, r - er) };
  }
  solvePlane(yaw, r, h, g) {  // tool in the arm plane; g = tilt from straight down toward +r
    const wr = r - K.LT * Math.sin(g), wh = h + K.LT * Math.cos(g), { a, b } = this.planar(wr, wh);
    return { yaw, a, b, j4: 0, j5: wrap(Math.atan2(-Math.cos(g), Math.sin(g)) - b), j6: 0 };
  }
  solveZ(G, phi, roll = 0, lk = 0) {  // tool axis: tilt phi (ccw about +Z) then leaning toward +Z by lk; jaw axis = world +Z projected, rolled by `roll`
    const cl = Math.cos(lk), t = new THREE.Vector3(Math.sin(phi) * cl, -Math.cos(phi) * cl, Math.sin(lk)), W = new THREE.Vector3(G[0], G[1], G[2]).addScaledVector(t, -K.LT);
    const dx = W.x - K.BX, dz = W.z - K.ZB, yaw = Math.atan2(-dz, dx), { a, b } = this.planar(Math.hypot(dx, dz), W.y);
    const Rf = new THREE.Matrix4().makeRotationY(yaw).multiply(new THREE.Matrix4().makeRotationZ(b));
    const Z = new THREE.Vector3(0, 0, 1); Z.addScaledVector(t, -Z.dot(t)).normalize(); if (roll) Z.applyAxisAngle(t, roll); const Y = new THREE.Vector3().crossVectors(Z, t);
    const Rt = new THREE.Matrix4().makeBasis(t, Y, Z), m = Rf.clone().transpose().multiply(Rt).elements;
    let b5 = Math.atan2(Math.hypot(m[1], m[2]), m[0]), a4, c6;
    if (Math.sin(b5) > 1e-4) { a4 = Math.atan2(m[2], m[1]); c6 = Math.atan2(m[8], -m[4]); } else { a4 = 0; c6 = Math.atan2(-m[6], m[5]); }
    if (Math.abs(wrap(a4)) > PI / 2) { a4 = wrap(a4 - PI); b5 = -b5; c6 = wrap(c6 + PI); }
    return { yaw, a, b, j4: wrap(a4), j5: b5, j6: wrap(c6) };
  }
  solve(s) { const q = s.m === 'p' ? this.solvePlane(s.yaw, s.r, s.h, s.g) : this.solveZ(s.G, s.phi, s.roll || 0, s.lk || 0); q.grip = (s.grip !== undefined ? s.grip * D : this.qT.grip); return q; }
  apply() {
    const q = this.q, p = this.p;
    p.turret.rotation.y = q.yaw; p.shoulder.rotation.z = q.a; p.elbow.rotation.z = q.b - q.a;
    p.wr4.rotation.x = q.j4; p.wr5.rotation.z = q.j5; p.wr6.rotation.x = q.j6;
    p.hinges[0].rotation.y = -q.grip; p.hinges[1].rotation.y = p.hinges[2].rotation.y = -(q.grip + this.dF);
    for (let i = 0; i < 3; i++) {   // actuator linkage: rod from the fixed clevis to the pin on the proximal link
      const th = -(q.grip + (i ? this.dF : 0)), c = Math.cos(th), sn = Math.sin(th), bx = 24 * c + 5.5 * sn, bz = -24 * sn + 5.5 * c, ax = 14, az = 24, dx = bx - ax, dz = bz - az, L = Math.hypot(dx, dz), r = p.rods[i];
      r.position.set((ax + bx) / 2, 0, (az + bz) / 2); r.rotation.y = Math.atan2(-dz, dx); r.scale.x = L;
    }
    p.root.updateMatrixWorld(true);
  }
  gWorld() { return this.p.gmark.getWorldPosition(new THREE.Vector3()); }
  toolMatrix() { return this.p.wr6.matrixWorld; }
  // ----- motion commands (return per-step closures; use with a script runner) -----
  move(spec, mode = 'j', Tmin = 0.5) {
    let st = null; const self = this;
    return dt => {
      if (!st) st = self._begin(spec, mode, Tmin);
      st.t += dt; const u = st.t / st.T;
      if (mode === 'l') { self.qT = self.solve(self._lerp(st.s0, spec, prof(u))); }
      else for (const k of J) { const d = DELAY[k], w = clamp((st.t / st.T - d) / (1 - d), 0, 1); self.qT[k] = st.q0[k] + (st.q1[k] - st.q0[k]) * prof(w); }
      if (u >= 1) { self.qT = mode === 'l' ? self.solve(spec) : Object.assign({}, st.q1); self.spec = Object.assign({}, spec, spec.grip === undefined && self.spec ? { grip: self.spec.grip } : {}); }
      return u >= 1 && self.settled() || st.t > st.T + 0.9;
    };
  }
  gripTo(deg, Tmin = 0.4) {
    let st = null; const self = this;
    return dt => {
      if (!st) { const q0 = self.qT.grip, q1 = deg * D; st = { t: 0, q0, q1, T: Math.max(Tmin, trapT(Math.abs(q1 - q0), LIM.grip[0], LIM.grip[1])) }; }
      st.t += dt; const u = st.t / st.T; self.qT.grip = st.q0 + (st.q1 - st.q0) * prof(u);
      if (u >= 1 && self.spec) self.spec.grip = deg;
      return u >= 1 && Math.abs(self.q.grip - self.qT.grip) < 0.02 || st.t > st.T + 0.8;
    };
  }
  _begin(spec, mode, Tmin) {
    const st = { t: 0, q0: Object.assign({}, this.qT) };
    if (mode === 'l') {
      const s0 = this.spec; st.s0 = Object.assign({}, s0, { grip: s0.grip !== undefined ? s0.grip : this.qT.grip / D });
      if (spec.grip === undefined) spec.grip = st.s0.grip;
      const dist = spec.m === 'p' ? Math.hypot(spec.r - s0.r, spec.h - s0.h) : Math.hypot(spec.G[0] - s0.G[0], spec.G[1] - s0.G[1], spec.G[2] - s0.G[2]);
      const dang = spec.m === 'p' ? Math.abs(spec.g - s0.g) : Math.max(Math.abs(spec.phi - s0.phi), Math.abs((spec.roll || 0) - (s0.roll || 0)), Math.abs((spec.lk || 0) - (s0.lk || 0)));
      st.T = Math.max(Tmin, trapT(dist, 480, 1100), trapT(dang, 1.1, 3.5));
    } else {
      st.q1 = this.solve(spec); let T = Tmin;
      for (const k of J) { const dq = Math.abs(st.q1[k] - st.q0[k]); T = Math.max(T, trapT(dq, LIM[k][0], LIM[k][1]) / (1 - DELAY[k])); }
      st.T = T;
    }
    return st;
  }
  _lerp(a, b, s) {
    const o = { m: b.m, grip: a.grip + (b.grip - a.grip) * s };
    if (b.m === 'p') { o.yaw = b.yaw; o.r = a.r + (b.r - a.r) * s; o.h = a.h + (b.h - a.h) * s; o.g = a.g + (b.g - a.g) * s; }
    else { o.G = a.G.map((v, i) => v + (b.G[i] - v) * s); o.phi = a.phi + (b.phi - a.phi) * s; o.roll = (a.roll || 0) + ((b.roll || 0) - (a.roll || 0)) * s; o.lk = (a.lk || 0) + ((b.lk || 0) - (a.lk || 0)) * s; }
    return o;
  }
  settled() { for (const k of J) if (Math.abs(this.qT[k] - this.q[k]) > 0.012 || Math.abs(this.qd[k]) > 0.05) return false; return true; }
  init(spec) { this.spec = Object.assign({}, spec); const q = this.solve(spec); for (const k of J) { this.q[k] = this.qT[k] = q[k]; this.qd[k] = 0; } this.apply(); }
  step(dt) { // mass/spring tracking of the planned targets: gives inertia, lag, small overshoot + settling
    for (let s = 0; s < 2; s++) { const h = dt / 2; for (const k of J) { const w = WN[k], acc = w * w * (this.qT[k] - this.q[k]) - 2 * ZETA * w * this.qd[k]; this.qd[k] += acc * h; this.q[k] += this.qd[k] * h; } }
    this.apply();
  }
}
