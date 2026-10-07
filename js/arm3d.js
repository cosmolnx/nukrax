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

// ---------- materials ----------
const mats = {
  paint: new THREE.MeshStandardMaterial({ color: 0x8f999f, roughness: 0.5, metalness: 0.42 }),
  panel: new THREE.MeshStandardMaterial({ color: 0x778288, roughness: 0.58, metalness: 0.45 }),
  joint: new THREE.MeshStandardMaterial({ color: 0x2b3238, roughness: 0.42, metalness: 0.85 }),
  steel: new THREE.MeshStandardMaterial({ color: 0xc9d0d4, roughness: 0.3, metalness: 1 }),
  rubber: new THREE.MeshStandardMaterial({ color: 0x14181b, roughness: 0.92, metalness: 0 }),
  accent: new THREE.MeshStandardMaterial({ color: 0xabbed3, roughness: 0.5, metalness: 0.25 }),
  pad: new THREE.MeshStandardMaterial({ color: 0x1b2023, roughness: 0.85, metalness: 0.1 }),
};
export { mats };
const M = (g, m, x = 0, y = 0, z = 0) => { const o = new THREE.Mesh(g, m); o.position.set(x, y, z); o.castShadow = o.receiveShadow = true; return o; };
const cylX = (r, len, m, x, y = 0, z = 0, rt = r) => { const g = new THREE.CylinderGeometry(rt, r, len, 44); g.rotateZ(-PI / 2); return M(g, m, x, y, z); };
const cylZ = (r, len, m, x = 0, y = 0, z = 0) => { const g = new THREE.CylinderGeometry(r, r, len, 44); g.rotateX(PI / 2); return M(g, m, x, y, z); };
const cylY = (r, len, m, x = 0, y = 0, z = 0, rt = r) => M(new THREE.CylinderGeometry(rt, r, len, 56), m, x, y, z);
function rrect(w, h, r) { const s = new THREE.Shape(), x = -w / 2, y = -h / 2; s.moveTo(x + r, y); s.lineTo(x + w - r, y); s.quadraticCurveTo(x + w, y, x + w, y + r); s.lineTo(x + w, y + h - r); s.quadraticCurveTo(x + w, y + h, x + w - r, y + h); s.lineTo(x + r, y + h); s.quadraticCurveTo(x, y + h, x, y + h - r); s.lineTo(x, y + r); s.quadraticCurveTo(x, y, x + r, y); return s; }
function ext(shape, d, bev, m, x = 0, y = 0, z = 0) { const g = new THREE.ExtrudeGeometry(shape, { depth: d - 2 * bev, bevelEnabled: true, bevelThickness: bev, bevelSize: bev, bevelSegments: 2, curveSegments: 14 }); g.translate(0, 0, -(d - 2 * bev) / 2); return M(g, m, x, y, z); }
const rbox = (w, h, d, r, m, x, y, z) => ext(rrect(w, h, r), d, 2.5, m, x, y, z);
function plateShape(len, r0, r1) { const s = new THREE.Shape(); s.moveTo(0, -r0); s.lineTo(len, -r1); s.absarc(len, 0, r1, -PI / 2, PI / 2, false); s.lineTo(0, r0); s.absarc(0, 0, r0, PI / 2, PI * 1.5, false); return s; }
function boltsZ(parent, cx, cy, z, R, n, r = 4.5, m = mats.joint) { for (let i = 0; i < n; i++) { const a = i / n * 2 * PI; parent.add(cylZ(r, 6, m, cx + Math.cos(a) * R, cy + Math.sin(a) * R, z)); } }
function boltsX(parent, x, R, n, r = 3.5) { for (let i = 0; i < n; i++) { const a = i / n * 2 * PI; parent.add(cylX(r, 5, mats.joint, x, Math.sin(a) * R, Math.cos(a) * R)); } }
function seam(parent, x, R, w = 3) { parent.add(cylX(R, w, mats.rubber, x)); }

export function buildArm() {
  const root = new THREE.Group(); root.position.set(K.BX, 0, K.ZB);
  // ---- fixed base ----
  root.add(cylY(212, 22, mats.steel, 0, 11, 0, 206));
  for (let i = 0; i < 8; i++) { const a = i / 8 * 2 * PI + 0.2; root.add(cylY(10, 14, mats.joint, Math.cos(a) * 172, 28, Math.sin(a) * 172)); }
  root.add(cylY(112, 128, mats.paint, 0, 86, 0, 108));
  for (const y of [52, 96, 130]) root.add(cylY(113, 5, mats.rubber, 0, y, 0));
  root.add(cylY(122, 26, mats.joint, 0, 163, 0));
  root.add(rbox(110, 96, 84, 8, mats.paint, -172, 62, 40)); root.add(rbox(60, 40, 50, 6, mats.joint, -172, 128, 40));  // cabinet + junction (cf. reference)
  // ---- turret (yaw about Y) ----
  const turret = new THREE.Group(); turret.position.y = 163; root.add(turret);
  turret.add(cylY(100, 30, mats.paint, 0, 15 + 7, 0, 96));
  turret.add(rbox(176, 178, 150, 22, mats.paint, 0, 108, 0));
  // motor housings left/right of the shoulder (visible while the base turns)
  const sh = K.HS - 163;
  turret.add(cylZ(74, 52, mats.joint, 0, sh, 100)); turret.add(cylZ(52, 16, mats.steel, 0, sh, 132));
  turret.add(cylZ(74, 52, mats.joint, 0, sh, -100)); turret.add(cylZ(52, 16, mats.steel, 0, sh, -132));
  boltsZ(turret, 0, sh, 134, 62, 6, 4); boltsZ(turret, 0, sh, -134, 62, 6, 4);
  // ---- shoulder / upper arm ----
  const shoulder = new THREE.Group(); shoulder.position.y = sh; turret.add(shoulder);
  shoulder.add(ext(plateShape(K.L1, 86, 64), 96, 6, mats.paint));
  const panel = ext(plateShape(K.L1 - 40, 62, 42), 102, 2, mats.panel, 30, 0, 0); shoulder.add(panel);
  for (const z of [51.5, -51.5]) { shoulder.add(cylZ(66, 7, mats.joint, 0, 0, z)); boltsZ(shoulder, 0, 0, z * 1.02, 52, 8); shoulder.add(cylZ(30, 8, mats.steel, 0, 0, z * 1.06)); }
  const duct = new THREE.CatmullRomCurve3([new THREE.Vector3(60, 74, 38), new THREE.Vector3(K.L1 * 0.35, 76, 40), new THREE.Vector3(K.L1 * 0.7, 62, 40), new THREE.Vector3(K.L1 - 40, 54, 38)]);
  shoulder.add(M(new THREE.TubeGeometry(duct, 30, 6, 10), mats.rubber)); for (const t of [0.18, 0.42, 0.7]) { const p = duct.getPoint(t); shoulder.add(rbox(14, 12, 20, 3, mats.joint, p.x, p.y, 40)); }
  for (const f of [0.38, 0.62]) shoulder.add(rbox(16, 150 - f * 40, 108, 4, mats.joint, K.L1 * f, 0, 0));
  // ---- elbow ----
  const elbow = new THREE.Group(); elbow.position.x = K.L1; shoulder.add(elbow);
  elbow.add(cylZ(70, 130, mats.joint, 0, 0, 0));
  for (const z of [66, -66]) { elbow.add(cylZ(54, 14, mats.steel, 0, 0, z)); boltsZ(elbow, 0, 0, z * 1.05, 44, 8, 4); elbow.add(cylZ(20, 8, mats.joint, 0, 0, z * 1.1)); }
  // ---- forearm (tube along +X) ----
  const fl = K.L2 - 60;
  elbow.add(cylX(66, 38, mats.paint, 56, 0, 0, 62));
  const tube = cylX(60, fl - 60, mats.paint, 60 + (fl - 60) / 2, 0, 0, 38); elbow.add(tube);
  for (const t of [0.12, 0.33, 0.55, 0.78]) { const x = 60 + (fl - 60) * t, r = 60 - 22 * t; elbow.add(cylX(r + 4, 12, mats.joint, x)); seam(elbow, x + 14, r + 1); }
  elbow.add(cylX(48, 8, mats.accent, 60 + (fl - 60) * 0.22, 0, 0, 48)); // single restrained brand stripe
  elbow.add(rbox(150, 12, 66, 4, mats.panel, 60 + (fl - 60) * 0.5, 48, 0));
  elbow.add(rbox(60, 8, 30, 2, mats.joint, 60 + (fl - 60) * 0.5, 56, 0));
  boltsX(elbow, 60, 62, 12);
  // ---- wrist ----
  const wr4 = new THREE.Group(); wr4.position.x = K.L2 - 62; elbow.add(wr4);
  wr4.add(cylX(40, 62, mats.paint, 31, 0, 0, 36)); wr4.add(cylX(44, 8, mats.joint, 2)); boltsX(wr4, 4, 38, 8, 3);
  const wr5 = new THREE.Group(); wr5.position.x = 62; wr4.add(wr5);
  wr5.add(cylZ(48, 98, mats.joint, 0, 0, 0));
  for (const z of [54, -54]) { wr5.add(cylZ(36, 12, mats.steel, 0, 0, z)); boltsZ(wr5, 0, 0, z * 1.06, 26, 6, 3.2); }
  wr5.add(cylX(30, 30, mats.paint, 22)); wr5.add(cylX(36, 8, mats.steel, 40));
  const wr6 = new THREE.Group(); wr6.position.x = 44; wr5.add(wr6);
  wr6.add(cylX(44, 44, mats.joint, 22, 0, 0, 42)); wr6.add(cylX(48, 6, mats.steel, 45));
  boltsX(wr6, 41, 36, 6, 3.5);
  const gmark = new THREE.Object3D(); gmark.position.x = 44 + 96; wr6.add(gmark);
  // ---- claw: back finger + two front fingers (front = tool +Z) ----
  const angles = [PI, FRONT, -FRONT], hinges = [];
  for (const a of angles) {
    const f = new THREE.Group(); f.position.set(44, R0 * Math.sin(a), R0 * Math.cos(a)); f.rotation.x = -a; wr6.add(f);
    f.add(rbox(18, 14, 16, 3, mats.steel, 6, 0, -2));
    const h = new THREE.Group(); f.add(h); hinges.push(h);
    h.add(cylY(7, 15, mats.joint));
    h.add(M(new THREE.BoxGeometry(LP, 11, 8), mats.paint, LP / 2, 0, 0));
    h.add(M(new THREE.BoxGeometry(LP * 0.6, 4, 10), mats.joint, LP * 0.55, 0, 0));
    const d = new THREE.Group(); d.position.x = LP; d.rotation.y = KAP; h.add(d);
    d.add(cylY(5.5, 12, mats.steel));
    d.add(M(new THREE.BoxGeometry(LD + 4, 9, 7), mats.paint, (LD + 4) / 2 - 2, 0, 0));
    d.add(M(new THREE.BoxGeometry(14, 9, 4), mats.pad, 26, 0, -5.5));
  }
  return { root, turret, shoulder, elbow, wr4, wr5, wr6, gmark, hinges };
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
  solveZ(G, phi) {  // tool axis in world XY plane (tilt phi, ccw about +Z), jaw axis = world +Z (parallel to the letters' normal)
    const t = new THREE.Vector3(Math.sin(phi), -Math.cos(phi), 0), W = new THREE.Vector3(G[0], G[1], G[2]).addScaledVector(t, -K.LT);
    const dx = W.x - K.BX, dz = W.z - K.ZB, yaw = Math.atan2(-dz, dx), { a, b } = this.planar(Math.hypot(dx, dz), W.y);
    const Rf = new THREE.Matrix4().makeRotationY(yaw).multiply(new THREE.Matrix4().makeRotationZ(b));
    const Z = new THREE.Vector3(0, 0, 1), Y = new THREE.Vector3().crossVectors(Z, t);
    const Rt = new THREE.Matrix4().makeBasis(t, Y, Z), m = Rf.clone().transpose().multiply(Rt).elements;
    let b5 = Math.atan2(Math.hypot(m[1], m[2]), m[0]), a4, c6;
    if (Math.sin(b5) > 1e-4) { a4 = Math.atan2(m[2], m[1]); c6 = Math.atan2(m[8], -m[4]); } else { a4 = 0; c6 = Math.atan2(-m[6], m[5]); }
    if (Math.abs(wrap(a4)) > PI / 2) { a4 = wrap(a4 - PI); b5 = -b5; c6 = wrap(c6 + PI); }
    return { yaw, a, b, j4: wrap(a4), j5: b5, j6: wrap(c6) };
  }
  solve(s) { const q = s.m === 'p' ? this.solvePlane(s.yaw, s.r, s.h, s.g) : this.solveZ(s.G, s.phi); q.grip = (s.grip !== undefined ? s.grip * D : this.qT.grip); return q; }
  apply() {
    const q = this.q, p = this.p;
    p.turret.rotation.y = q.yaw; p.shoulder.rotation.z = q.a; p.elbow.rotation.z = q.b - q.a;
    p.wr4.rotation.x = q.j4; p.wr5.rotation.z = q.j5; p.wr6.rotation.x = q.j6;
    p.hinges[0].rotation.y = -q.grip; p.hinges[1].rotation.y = p.hinges[2].rotation.y = -(q.grip + this.dF);
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
      const dang = spec.m === 'p' ? Math.abs(spec.g - s0.g) : Math.abs(spec.phi - s0.phi);
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
    else { o.G = a.G.map((v, i) => v + (b.G[i] - v) * s); o.phi = a.phi + (b.phi - a.phi) * s; }
    return o;
  }
  settled() { for (const k of J) if (Math.abs(this.qT[k] - this.q[k]) > 0.012 || Math.abs(this.qd[k]) > 0.05) return false; return true; }
  init(spec) { this.spec = Object.assign({}, spec); const q = this.solve(spec); for (const k of J) { this.q[k] = this.qT[k] = q[k]; this.qd[k] = 0; } this.apply(); }
  step(dt) { // mass/spring tracking of the planned targets: gives inertia, lag, small overshoot + settling
    for (let s = 0; s < 2; s++) { const h = dt / 2; for (const k of J) { const w = WN[k], acc = w * w * (this.qT[k] - this.q[k]) - 2 * ZETA * w * this.qd[k]; this.qd[k] += acc * h; this.q[k] += this.qd[k] * h; } }
    this.apply();
  }
}
