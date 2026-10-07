/* NUKRAX prototype V2 — scene, rigid-body letters (Matter.js, plane-constrained), and the robot behaviour script.
   Fixed 60 Hz simulation clock. The fall is NOT seeded to a convenient outcome: each page load is a new roll
   (use ?seed=N to replay one). The robot plans each grasp from the letters' actual resting poses. */
import * as THREE from 'three';
import { RoomEnvironment } from '../vendor/RoomEnvironment.js';
import { K, buildArm, Arm3D, contactRho, mats } from './arm3d.js';

const { Engine, Bodies, Body, Composite, Common } = Matter;
Common.setDecomp(window.decomp);
const qs = new URLSearchParams(location.search);
const SEED = qs.has('seed') ? +qs.get('seed') : (Math.random() * 1e9) | 0;
const DEBUG = qs.has('debug'), SPEED = +(qs.get('speed') || 1);
const reduce = matchMedia('(prefers-reduced-motion: reduce)').matches;
const W = 1920, H = 1080, FLOOR = 900, S = 0.056, BASE_Y = 470, X0 = 430, TH = K.PLATE, WALLZ = -90, DT = 1 / 60, D = Math.PI / 180;
let rs = SEED; const rnd = () => { rs |= 0; rs = rs + 0x6D2B79F5 | 0; let t = Math.imul(rs ^ rs >>> 15, 1 | rs); t = t + Math.imul(t ^ t >>> 7, 61 | t) ^ t; return ((t ^ t >>> 14) >>> 0) / 4294967296; };
const clamp = (v, a, b) => Math.max(a, Math.min(b, v));
const wrapA = a => { while (a > Math.PI) a -= 2 * Math.PI; while (a < -Math.PI) a += 2 * Math.PI; return a; };

// =============== renderer / scene ===============
const host = document.getElementById('stage');
let renderer;
try { renderer = new THREE.WebGLRenderer({ antialias: true, powerPreference: 'high-performance' }); } catch (e) { document.body.classList.add('nogl'); throw e; }
renderer.setPixelRatio(qs.has('manual') ? 1 : Math.min(devicePixelRatio || 1, 2)); renderer.shadowMap.enabled = true; renderer.shadowMap.type = THREE.PCFSoftShadowMap;
renderer.toneMapping = THREE.ACESFilmicToneMapping; renderer.toneMappingExposure = 0.8; renderer.outputColorSpace = THREE.SRGBColorSpace;
host.replaceWith(renderer.domElement); renderer.domElement.id = 'stage';
const scene = new THREE.Scene(); scene.background = new THREE.Color(0x07090b); scene.fog = new THREE.Fog(0x07090b, 3800, 7500);
const pm = new THREE.PMREMGenerator(renderer); scene.environment = pm.fromScene(new RoomEnvironment(), 0.04).texture; scene.environmentIntensity = 0.22;
const camera = new THREE.PerspectiveCamera(26, 16 / 9, 200, 12000); camera.position.set(960, 840, 2950); camera.lookAt(960, 470, 0);
scene.add(new THREE.HemisphereLight(0x8da0b0, 0x050607, 0.32));
const key = new THREE.DirectionalLight(0xf0efec, 2.1); key.position.set(300, 1500, 1300); key.target.position.set(980, 200, 0); key.castShadow = true;
Object.assign(key.shadow.camera, { left: -1500, right: 1500, top: 1100, bottom: -700, near: 200, far: 4000 }); key.shadow.mapSize.set(qs.has('manual') ? 2048 : 3072, qs.has('manual') ? 2048 : 3072); key.shadow.bias = -0.0004; key.shadow.normalBias = 1.2; key.shadow.radius = 4;
scene.add(key, key.target);
const rim = new THREE.DirectionalLight(0xabbed3, 1.5); rim.position.set(2200, 800, -900); rim.target.position.set(1400, 400, 0); scene.add(rim, rim.target);
const fill = new THREE.DirectionalLight(0x9fb0c0, 0.5); fill.position.set(-1500, 400, 1500); scene.add(fill);

function gridTex() { const c = document.createElement('canvas'); c.width = c.height = 256; const g = c.getContext('2d'); g.fillStyle = '#0b0f12'; g.fillRect(0, 0, 256, 256); for (let i = 0; i < 2600; i++) { g.fillStyle = `rgba(255,255,255,${Math.random() * 0.025})`; g.fillRect(Math.random() * 256, Math.random() * 256, 1.5, 1.5); } g.strokeStyle = 'rgba(207,222,234,.07)'; g.lineWidth = 2; g.strokeRect(0, 0, 256, 256); const t = new THREE.CanvasTexture(c); t.wrapS = t.wrapT = THREE.RepeatWrapping; t.repeat.set(60, 40); t.colorSpace = THREE.SRGBColorSpace; t.anisotropy = 8; return t; }
const floor = new THREE.Mesh(new THREE.PlaneGeometry(6000, 4000), new THREE.MeshStandardMaterial({ map: gridTex(), roughness: 0.82, metalness: 0.1 })); floor.rotation.x = -Math.PI / 2; floor.position.set(980, 0, 700); floor.receiveShadow = true; scene.add(floor);
const wall = new THREE.Mesh(new THREE.PlaneGeometry(7000, 3200), new THREE.MeshStandardMaterial({ color: 0x070a0c, roughness: 0.95, metalness: 0 })); wall.position.set(980, 1500, WALLZ); wall.receiveShadow = true; scene.add(wall);
const skirt = new THREE.Mesh(new THREE.BoxGeometry(7000, 16, 8), new THREE.MeshStandardMaterial({ color: 0x1b2328, roughness: 0.6, metalness: 0.4 })); skirt.position.set(980, 8, WALLZ + 4); skirt.receiveShadow = true; scene.add(skirt);

// work cell on the right (the robot's original task)
const PA = 1700, PB = 1830, PH = 60, BLK = 26;
for (const x of [PA, PB]) { const b = new THREE.Mesh(new THREE.BoxGeometry(84, PH, 84), mats.joint); b.position.set(x, PH / 2, K.ZB); b.castShadow = b.receiveShadow = true; scene.add(b); const top = new THREE.Mesh(new THREE.BoxGeometry(86, 3, 86), mats.steel); top.position.set(x, PH + 1.5, K.ZB); scene.add(top); }
const block = new THREE.Mesh(new THREE.BoxGeometry(BLK, BLK, BLK), mats.accent); block.castShadow = true; scene.add(block); let blockHeld = false;

// =============== robot ===============
const parts = buildArm(); scene.add(parts.root);
const arm = new Arm3D(parts);
const plateRho0 = contactRho(TH / 2, 0), blockRho0 = contactRho(BLK / 2, 0);
const OPEN = 34, HOVER = 210;
const pSpec = (x, y, g = 0, grip) => ({ m: 'p', yaw: 0, r: x - K.BX, h: y, g, grip });
arm.init(pSpec(PA, PH + BLK / 2 + 120, 0, OPEN));

// =============== letters ===============
const engine = Engine.create({ positionIterations: 12, velocityIterations: 10 });
engine.gravity.y = 1; engine.gravity.scale = 0.0017;
const letters = []; let ox = X0;
const inside = (pts, holes, x, y) => { let c = false; for (const poly of [pts, ...holes]) for (let i = 0, j = poly.length - 1; i < poly.length; j = i++) { const [xi, yi] = poly[i], [xj, yj] = poly[j]; if ((yi > y) !== (yj > y) && x < (xj - xi) * (y - yi) / (yj - yi) + xi) c = !c; } return c; };
const segDist = (px, py, a, b) => { const dx = b[0] - a[0], dy = b[1] - a[1], t = clamp(((px - a[0]) * dx + (py - a[1]) * dy) / (dx * dx + dy * dy || 1), 0, 1); return Math.hypot(px - a[0] - t * dx, py - a[1] - t * dy); };
const letterMat = new THREE.MeshStandardMaterial({ color: 0xf0efec, roughness: 0.4, metalness: 0.06 });
for (const ch of 'NUKRAX') {
  const g = NKX_GLYPHS.g[ch], pts = g.c.map(([x, y]) => [x * S, -y * S]);
  let A = 0, cx = 0, cy = 0; for (let i = 0; i < pts.length; i++) { const p = pts[i], q = pts[(i + 1) % pts.length], f = p[0] * q[1] - q[0] * p[1]; A += f; cx += (p[0] + q[0]) * f; cy += (p[1] + q[1]) * f; } cx /= 3 * A; cy /= 3 * A;
  const loc = pts.map(p => ({ x: p[0] - cx, y: p[1] - cy })), xs = loc.map(p => p.x);
  // 3D local frame: x right, y up, origin at centroid
  const o3 = g.c.map(([x, y]) => [x * S - cx, y * S + cy]), h3 = (g.holes || []).map(h => h.map(([x, y]) => [x * S - cx, y * S + cy]));
  const shape = new THREE.Shape(o3.map(p => new THREE.Vector2(...p))); for (const h of h3) shape.holes.push(new THREE.Path(h.map(p => new THREE.Vector2(...p))));
  const geo = new THREE.ExtrudeGeometry(shape, { depth: TH - 3, bevelEnabled: true, bevelThickness: 1.5, bevelSize: 1.1, bevelSegments: 2, curveSegments: 8 }); geo.translate(0, 0, -(TH - 3) / 2);
  const mesh = new THREE.Mesh(geo, letterMat); mesh.castShadow = mesh.receiveShadow = true; scene.add(mesh);
  // candidate grasp points: interior points with enough stroke clearance for a pad
  const cands = [], bx0 = Math.min(...o3.map(p => p[0])), bx1 = Math.max(...o3.map(p => p[0])), by0 = Math.min(...o3.map(p => p[1])), by1 = Math.max(...o3.map(p => p[1]));
  const edges = []; for (const poly of [o3, ...h3]) for (let i = 0; i < poly.length; i++) edges.push([poly[i], poly[(i + 1) % poly.length]]);
  for (let x = bx0; x <= bx1; x += 2.5) for (let y = by0; y <= by1; y += 2.5) if (inside(o3, h3, x, y)) { let d = 1e9; for (const [a, b] of edges) d = Math.min(d, segDist(x, y, a, b)); if (d >= 4.6) cands.push({ x, y, r: d }); }
  const slot = { x: ox + cx, y: FLOOR - BASE_Y - cy };               // world (y up) centroid when mounted
  const body = Bodies.fromVertices(ox + cx, BASE_Y + cy, [loc], { friction: 0.6, frictionStatic: 0.9, restitution: 0.1, density: 0.004, frictionAir: 0.002 }, true);
  Body.setStatic(body, true);
  // two wall studs (stay on the wall when the letter falls)
  const sorted = cands.slice().sort((a, b) => (b.r + b.y * 0.02) - (a.r + a.y * 0.02)), s1 = sorted[0], s2 = sorted.find(c => Math.hypot(c.x - s1.x, c.y - s1.y) > (bx1 - bx0) * 0.45) || sorted[1];
  for (const s of [s1, s2]) { const st = new THREE.Mesh(new THREE.CylinderGeometry(3.4, 3.4, -WALLZ - TH / 2 - 2, 20), mats.steel); st.rotation.x = Math.PI / 2; st.position.set(slot.x + s.x, slot.y + s.y, WALLZ / 2 - TH / 4 - 1); st.castShadow = true; scene.add(st); const w = new THREE.Mesh(new THREE.CylinderGeometry(8, 8, 3, 24), mats.joint); w.rotation.x = Math.PI / 2; w.position.set(slot.x + s.x, slot.y + s.y, WALLZ + 1.5); scene.add(w); }
  letters.push({ ch, body, mesh, slot, cx, cy, cands, loc, ex: 0, state: 'wall', release: 0.78 + rnd() * 0.22, minY: Math.min(...o3.map(p => p[1])), seat: null, rel: null });
  mesh.position.set(slot.x, slot.y, 0); ox += g.adv * S;
}
const floorB = Bodies.rectangle(W / 2, FLOOR + 60, 3000, 120, { isStatic: true, friction: 0.8, restitution: 0.05 });
const wallL = Bodies.rectangle(330 - 20, 600, 40, 900, { isStatic: true }), wallR = Bodies.rectangle(1240 + 20, 600, 40, 900, { isStatic: true });
Composite.add(engine.world, [floorB, wallL, wallR, ...letters.map(l => l.body)]);

// =============== state ===============
let T = 0, settled = false, calm = 0, released = false, phase = 'WORK';
const v3 = (x, y, z) => new THREE.Vector3(x, y, z);
const poseOf = L => ({ x: L.body.position.x, y: FLOOR - L.body.position.y, rot: -L.body.angle });
function syncMesh(L) { const p = poseOf(L); L.mesh.position.set(p.x, p.y, 0); L.mesh.rotation.set(0, 0, p.rot); }
function syncBody(L) { Body.setPosition(L.body, { x: L.mesh.position.x, y: FLOOR - L.mesh.position.y }); Body.setAngle(L.body, -new THREE.Euler().setFromQuaternion(L.mesh.quaternion).z); }

// ---- grasp planning from the letter's ACTUAL pose ----
function planGrasp(L) {
  const p = poseOf(L), rot = wrapA(p.rot), c = Math.cos(rot), s = Math.sin(rot);
  const phiG = clamp(rot / 2, -70 * D, 70 * D), t = [Math.sin(phiG), -Math.cos(phiG)];
  const tl = [t[0] * c + t[1] * s, -t[0] * s + t[1] * c];            // tool axis in letter-local coordinates
  let best = null, bs = -1e9;
  for (const q of L.cands) {
    const wx = p.x + q.x * c - q.y * s, wy = p.y + q.x * s + q.y * c;
    let back = 0; for (const v of L.cornerPts) back = Math.max(back, -((v[0] - q.x) * tl[0] + (v[1] - q.y) * tl[1]));   // material behind the pads (palm side)
    const score = Math.min(q.r, 6.5) * 6 + wy * 0.35 - Math.max(0, back - 62) * 3 - Math.hypot(q.x, q.y) * 0.25 - (wy < 30 ? (30 - wy) * 2 : 0);
    if (score > bs) { bs = score; best = { q, wx, wy }; }
  }
  return { G: [best.wx, best.wy, 0], q: best.q, rot, phiG, phiP: phiG - rot };
}
for (const L of letters) { const pts = []; for (let i = 0; i < L.loc.length; i++) pts.push([L.loc[i].x, -L.loc[i].y]); L.cornerPts = pts; }
const zSpec = (G, phi, grip) => ({ m: 'z', G, phi, grip });
const up = (G, d) => [G[0], G[1] + d, G[2]];
const blockers = L => { const out = []; for (const pr of engine.pairs.list) { if (!pr.isActive) continue; const a = letters.find(l => l.body === pr.bodyA.parent), b = letters.find(l => l.body === pr.bodyB.parent); if (!a || !b || (a !== L && b !== L)) continue; const o = a === L ? b : a; if (o.state === 'fall' && o.body.position.y < L.body.position.y - 14) out.push(o); } return out; };

// =============== script ===============
const TP = +(qs.get('tempo') || 0.72);   // tempo: scales commanded minimum durations and pauses (joint limits still apply)
const wait = s => { let t = 0; return dt => (t += dt) >= s * TP; };
const gripTo = (d, T) => arm.gripTo(d, T * TP);
const act = f => () => { f(); return true; };
const mvJ = (s, T = 0.6) => arm.move(s, 'j', T * TP), mvL = (s, T = 0.4) => arm.move(s, 'l', T * TP);

function* taskCycle(dir) {
  const a = dir ? PB : PA, b = dir ? PA : PB, top = PH + BLK / 2; phase = 'WORK';
  if (!blockHeld && Math.abs(block.position.x - a) > 1) block.position.set(a, top, K.ZB);
  const bp = x => pSpec(x, top + 110, 0, OPEN);
  yield mvJ(bp(a), 0.9); yield mvL(pSpec(a, top + 4, 0, OPEN), 0.5); yield wait(0.12);
  yield gripTo(contactRho(BLK / 2, 0) / D, 0.4); blockHeld = true; yield wait(0.1);
  yield mvL(bp(a), 0.45); yield mvJ(bp(b), 0.9); yield mvL(pSpec(b, top + 4, 0, blockRho0 / D), 0.5); yield wait(0.1);
  blockHeld = false; block.position.set(b, top, K.ZB); yield gripTo(OPEN, 0.35); yield mvL(bp(b), 0.4);
}

function holdStart(L) {   // jaws are closed on the plate: capture the letter in the tool frame
  L.mesh.updateMatrixWorld(); const tm = arm.toolMatrix().clone().invert();
  L.rel = tm.multiply(L.mesh.matrixWorld.clone()); L.relCur = L.rel.clone();
  const gl = arm.p.gmark.position.clone(), pg = v3(L.grasp.q.x, L.grasp.q.y, 0).applyMatrix4(L.rel);
  L.snap = gl.sub(pg); L.snap.z = 0; L.pull = 0;               // residual misalignment the closing jaws will correct
  L.state = 'held'; Body.setStatic(L.body, true); L.body.isSensor = true;
}
function holdUpdate(L, dt) {
  L.pull = Math.min(1, L.pull + dt / 0.22); const e = 1 - Math.pow(1 - L.pull, 3);
  const m = L.rel.clone(); m.setPosition(v3().setFromMatrixPosition(L.rel).addScaledVector(L.snap, e));
  const w = arm.toolMatrix().clone().multiply(m); w.decompose(L.mesh.position, L.mesh.quaternion, L.mesh.scale); syncBody(L);
}
function* carry(L, dest) {      // dest: {kind:'slot'} | {kind:'floor', x}
  const tag = L.ch, spec = G => zSpec(G, 0, OPEN + 0);
  phase = 'APPROACH ' + tag; let g = planGrasp(L); L.grasp = g;
  yield mvJ(zSpec(up(g.G, HOVER), g.phiG, OPEN), 1.5);
  phase = 'SLOW/ALIGN ' + tag; g = planGrasp(L); L.grasp = g;
  yield mvL(zSpec(up(g.G, 38), g.phiG, OPEN), 0.9); yield wait(0.3);
  g = planGrasp(L); L.grasp = g; const err = [1.8 * (rnd() < 0.5 ? -1 : 1), -1.3, 0];
  yield mvL(zSpec([g.G[0] + err[0], g.G[1] + err[1], 0], g.phiG + 0.012, OPEN), 0.85); yield wait(0.2);
  phase = 'ALIGN ' + tag; yield mvL(zSpec(g.G, g.phiG, OPEN), 0.35); yield wait(0.32);
  phase = 'GRIP ' + tag; const cr = contactRho(TH / 2, 0) / D;
  yield gripTo(cr + 9, 0.4); yield gripTo(cr, 0.5);
  yield act(() => holdStart(L)); yield wait(0.28); phase = 'CONFIRM ' + tag;
  yield mvL(zSpec(up(g.G, 9), g.phiG, cr), 0.3); yield wait(0.22);
  phase = 'LIFT ' + tag; yield mvL(zSpec(up(g.G, 150), g.phiG, cr), 0.8);
  if (dest.kind === 'slot') {
    phase = 'INSPECT ' + tag;
    const ins = [840 + (rnd() - 0.5) * 60, 600, 150]; yield mvJ(zSpec(ins, g.phiG, cr), 1.5); yield wait(0.2);
    const sgn = rnd() < 0.5 ? 1 : -1; yield mvL(zSpec(ins, g.phiG + sgn * 0.38, cr), 0.75); yield wait(0.25); yield mvL(zSpec(ins, g.phiG - sgn * 0.22, cr), 0.7); yield wait(0.45);
    phase = 'RETURN ' + tag;
    const gp = [L.slot.x + g.q.x, L.slot.y + g.q.y, 0];
    yield mvJ(zSpec(up(gp, 190), g.phiP, cr), 1.9);
    phase = 'PLACE ' + tag; yield mvL(zSpec(up(gp, 36), g.phiP, cr), 0.9); yield wait(0.25);
    yield mvL(zSpec([gp[0] + 1.6, gp[1] + 1.4, 0], g.phiP, cr), 0.7); yield wait(0.18);
    yield mvL(zSpec(gp, g.phiP, cr), 0.35); yield wait(0.3);
    yield act(() => { const m = L.mesh; L.seat = { p0: m.position.clone(), q0: m.quaternion.clone(), t: 0 }; L.state = 'seating'; Body.setStatic(L.body, true); });
    yield gripTo(cr + 26, 0.5); yield wait(0.15);
    yield mvL(zSpec(up(gp, 9), g.phiP, cr + 26), 0.3); yield mvL(zSpec(up(gp, 170), 0, OPEN), 0.7);
  } else {
    phase = 'CLEAR ' + tag;
    const gx = dest.x, gy = -L.minY + 5, loc = g.q, gp = [gx + loc.x, gy + loc.y, 0];
    yield mvJ(zSpec(up(gp, 160), g.phiP, cr), 1.6); yield mvL(zSpec(gp, g.phiP, cr), 0.8); yield wait(0.2);
    yield act(() => { L.state = 'fall'; Body.setStatic(L.body, false); L.body.isSensor = false; Body.setVelocity(L.body, { x: 0, y: 0 }); Body.setAngularVelocity(L.body, 0); });
    yield gripTo(cr + 28, 0.5); yield wait(0.2); yield mvL(zSpec(up(gp, 160), 0, OPEN), 0.8);
  }
}
function* serve(L) {
  let guard = 0, b;
  while ((b = blockers(L))[0] && guard++ < 3) { phase = 'CLEAR BLOCKER'; yield* carry(b[0], { kind: 'floor', x: 360 + rnd() * 40 }); yield wait(0.5); }
  yield* carry(L, { kind: 'slot' });
}
function* script() {
  let k = 0; while (!settled) yield* taskCycle(k++ & 1);
  phase = 'NOTICE'; yield wait(0.3);
  yield mvJ(pSpec(1640, 560, 0, OPEN), 1.1); yield wait(0.55);
  phase = 'TURN'; yield mvJ(zSpec([900, 420, 120], 0, OPEN), 2.4); yield wait(0.4);
  for (const L of letters) yield* serve(L);
  phase = 'CHECK'; yield wait(0.7); yield mvL(zSpec(up([letters[5].slot.x, letters[5].slot.y, 0], 170), 0.05, OPEN), 0.3); yield mvL(zSpec(up([letters[5].slot.x, letters[5].slot.y, 0], 170), -0.03, OPEN), 0.3); yield wait(0.8);
  phase = 'RETURN TO WORK'; yield mvJ(pSpec(PA, PH + BLK / 2 + 110, 0, OPEN), 2.6);
  for (;;) yield* taskCycle(k++ & 1);
}
const gen = script(); let cur = null;
function runScript(dt) { for (let n = 0; n < 8; n++) { if (!cur) { const r = gen.next(); if (r.done) return; cur = r.value; } if (cur(dt)) { cur = null; dt = 0; } else return; } }

// =============== simulation step ===============
function step() {
  T += DT;
  for (const L of letters) {
    if (L.state !== 'wall') continue;
    if (T >= 0.5 && T < L.release) Body.setAngle(L.body, 0.012 * Math.sin((T - 0.5) * 52 + L.slot.x) * clamp((T - 0.5) * 4, 0, 1));
    else if (T >= L.release) { L.state = 'fall'; Body.setStatic(L.body, false); Body.setVelocity(L.body, { x: (rnd() - 0.5) * 1.6, y: 0 }); Body.setAngularVelocity(L.body, (rnd() - 0.5) * 0.09); }
  }
  Engine.update(engine, 1000 / 120); Engine.update(engine, 1000 / 120);
  if (!released && letters.every(l => l.state !== 'wall')) released = true;
  if (released && !settled) { const mov = letters.some(l => l.state === 'fall' && (l.body.speed > 0.1 || Math.abs(l.body.angularSpeed) > 0.003)); calm = mov ? 0 : calm + DT; if (calm > 0.8) settled = true; }
  runScript(DT); arm.step(DT);
  for (const L of letters) {
    if (L.state === 'fall' || L.state === 'wall') syncMesh(L);
    else if (L.state === 'held') holdUpdate(L, DT);
    else if (L.state === 'seating') { const s = L.seat; s.t = Math.min(1, s.t + DT / 0.3); const e = 1 - Math.pow(1 - s.t, 3); L.mesh.position.lerpVectors(s.p0, v3(L.slot.x, L.slot.y, 0), e); L.mesh.quaternion.slerpQuaternions(s.q0, new THREE.Quaternion(), e); if (s.t >= 1) { L.state = 'placed'; Body.setPosition(L.body, { x: L.slot.x, y: FLOOR - L.slot.y }); Body.setAngle(L.body, 0); L.body.isSensor = false; } }
  }
  if (blockHeld) block.position.copy(arm.gWorld());
}

// =============== loop / sizing ===============
function fit() { const w = Math.min(innerWidth, innerHeight * 16 / 9), h = w * 9 / 16; renderer.setSize(w, h); camera.aspect = 16 / 9; camera.updateProjectionMatrix(); }
let last = 0, acc = 0;
function frame(ts) { const dt = Math.min(0.1, (ts - last) / 1000 || 0); last = ts; acc += dt * SPEED; for (let n = 0; acc >= DT && n < 8; n++) { step(); acc -= DT; } renderer.render(scene, camera); if (hud) hud.textContent = `${phase}  t=${T.toFixed(1)}`; requestAnimationFrame(frame); }
const hud = DEBUG ? Object.assign(document.createElement('div'), { className: 'dbg' }) : null; if (hud) document.body.append(hud);
window.__nkx = {
  skip(sec, onStep) { for (let i = 0; i < sec / DT; i++) { step(); if (onStep) onStep(); } renderer.render(scene, camera); },
  get t() { return T; }, get phase() { return phase; }, get settled() { return settled; }, seed: SEED, arm,
  letters: () => letters.map(l => ({ ch: l.ch, state: l.state, x: +l.mesh.position.x.toFixed(1), y: +l.mesh.position.y.toFixed(1), deg: +(-l.mesh.rotation.z * 57.2958).toFixed(1) })),
  until(ph) { let g = 0; while (!phase.startsWith(ph) && g++ < 60000) step(); },
  q: () => ({ ...arm.q }), qd: () => ({ ...arm.qd }), render: () => renderer.render(scene, camera),
};
fit(); addEventListener('resize', () => { fit(); renderer.render(scene, camera); });
if (reduce) { for (const L of letters) { L.state = 'placed'; L.mesh.position.set(L.slot.x, L.slot.y, 0); L.mesh.rotation.set(0, 0, 0); } renderer.render(scene, camera); }
else if (!qs.has('manual')) requestAnimationFrame(frame);
(document.fonts ? document.fonts.ready : Promise.resolve()).then(() => document.body.classList.add('ready'));
