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
// NUKRAX web palette (Color_System_Specification.pdf, dark theme)
const PAL = { page: 0x151515, recess: 0x111111, surface: 0x1a1a19, overlay: 0x20201f, control: 0x2d2d2d, hover: 0x373736, strong: 0x4d4d4c, text: 0xf0efec, text2: 0xc3c2b7, muted: 0x898781, accent: 0xabbed3 };
const clamp = (v, a, b) => Math.max(a, Math.min(b, v));
const wrapA = a => { while (a > Math.PI) a -= 2 * Math.PI; while (a < -Math.PI) a += 2 * Math.PI; return a; };

// =============== renderer / scene ===============
const host = document.getElementById('stage');
let renderer;
try { renderer = new THREE.WebGLRenderer({ antialias: true, powerPreference: 'high-performance' }); } catch (e) { document.body.classList.add('nogl'); throw e; }
renderer.setPixelRatio(qs.has('manual') ? 1 : Math.min(devicePixelRatio || 1, 2)); renderer.shadowMap.enabled = true; renderer.shadowMap.type = THREE.PCFSoftShadowMap;
renderer.toneMapping = THREE.NeutralToneMapping; renderer.toneMappingExposure = 1.25; renderer.outputColorSpace = THREE.SRGBColorSpace;
host.replaceWith(renderer.domElement); renderer.domElement.id = 'stage';
const scene = new THREE.Scene(); scene.background = new THREE.Color(PAL.page); scene.fog = new THREE.Fog(PAL.page, 3800, 7500);
const pm = new THREE.PMREMGenerator(renderer); scene.environment = pm.fromScene(new RoomEnvironment(), 0.04).texture; scene.environmentIntensity = 0.55;
const camera = new THREE.PerspectiveCamera(26, 16 / 9, 200, 12000); camera.position.set(960, 840, 2950); camera.lookAt(960, 470, 0);
scene.add(new THREE.HemisphereLight(PAL.text2, PAL.surface, 0.7));
const key = new THREE.DirectionalLight(PAL.text, 3.2); key.position.set(300, 1500, 1300); key.target.position.set(980, 200, 0); key.castShadow = true;
Object.assign(key.shadow.camera, { left: -1500, right: 1500, top: 1100, bottom: -700, near: 200, far: 4000 }); key.shadow.mapSize.set(qs.has('manual') ? 2048 : 3072, qs.has('manual') ? 2048 : 3072); key.shadow.bias = -0.0004; key.shadow.normalBias = 1.2; key.shadow.radius = 4;
scene.add(key, key.target);
const rim = new THREE.DirectionalLight(PAL.accent, 1.7); rim.position.set(2200, 800, -900); rim.target.position.set(1400, 400, 0); scene.add(rim, rim.target);
const kick = new THREE.DirectionalLight(PAL.accent, 0.9); kick.position.set(-1300, 900, -900); kick.target.position.set(1100, 400, 0); scene.add(kick, kick.target);
const fill = new THREE.DirectionalLight(PAL.text2, 0.9); fill.position.set(-1500, 400, 1500); scene.add(fill);

function gridTex() { const c = document.createElement('canvas'); c.width = c.height = 256; const g = c.getContext('2d'); g.fillStyle = '#1a1a19'; g.fillRect(0, 0, 256, 256); for (let i = 0; i < 2600; i++) { g.fillStyle = `rgba(255,255,255,${Math.random() * 0.025})`; g.fillRect(Math.random() * 256, Math.random() * 256, 1.5, 1.5); } g.strokeStyle = '#2d2d2d'; g.lineWidth = 2; g.strokeRect(0, 0, 256, 256); const t = new THREE.CanvasTexture(c); t.wrapS = t.wrapT = THREE.RepeatWrapping; t.repeat.set(60, 40); t.colorSpace = THREE.SRGBColorSpace; t.anisotropy = 8; return t; }
const floor = new THREE.Mesh(new THREE.PlaneGeometry(6000, 4000), new THREE.MeshStandardMaterial({ map: gridTex(), roughness: 0.8, metalness: 0.12 })); floor.rotation.x = -Math.PI / 2; floor.position.set(980, 0, 700); floor.receiveShadow = true; scene.add(floor);
const wall = new THREE.Mesh(new THREE.PlaneGeometry(7000, 3200), new THREE.MeshStandardMaterial({ color: PAL.recess, roughness: 0.95, metalness: 0 })); wall.position.set(980, 1500, WALLZ); wall.receiveShadow = true; scene.add(wall);
const skirt = new THREE.Mesh(new THREE.BoxGeometry(7000, 16, 8), new THREE.MeshStandardMaterial({ color: PAL.control, roughness: 0.6, metalness: 0.4 })); skirt.position.set(980, 8, WALLZ + 4); skirt.receiveShadow = true; scene.add(skirt);

// ---- work cell (right): precision insertion with in-process gauging ----
// magazine feeder -> dial-indicator inspection -> press-fit into a 3-pocket fixture; finished parts are pulled and dropped down a chute.
const TOP = 44, PART_R = 13, PART_H = 26, CX = [1726, 1760, 1794];
const Z_FEED = 40, Z_INS = 170, Z_FIX = 330, INS = [1888, 154, Z_INS], CHUTE = [1946, 84, 332];
const mk = (g, m, x, y, z) => { const o = new THREE.Mesh(g, m); o.position.set(x, y, z); o.castShadow = o.receiveShadow = true; scene.add(o); return o; };
mk(new THREE.BoxGeometry(305, TOP - 4, 380), mats.deep, 1842, (TOP - 4) / 2, 200); mk(new THREE.BoxGeometry(305, 4, 380), mats.joint, 1842, TOP - 2, 200);
for (const [w, d, x, z] of [[305, 5, 1842, 12], [305, 5, 1842, 388], [5, 380, 1692, 200], [5, 380, 1992, 200]]) mk(new THREE.BoxGeometry(w, 4, d), mats.panel, x, TOP + 0.5, z);
mk(new THREE.BoxGeometry(124, 16, 46), mats.panel, 1760, TOP + 8, Z_FEED); mk(new THREE.BoxGeometry(124, 22, 52), mats.joint, 1760, TOP + 11, Z_FIX);
for (const x of CX) { mk(new THREE.CylinderGeometry(15.5, 15.5, 0.6, 28), mats.rubber, x, TOP + 16.3, Z_FEED); const r = mk(new THREE.CylinderGeometry(17, 17, 1.2, 28), mats.steel, x, TOP + 22.2, Z_FIX); mk(new THREE.CylinderGeometry(14.6, 14.6, 1.5, 28), mats.rubber, x, TOP + 22.6, Z_FIX); }
// dial-indicator station: post, gauge body, face, plunger (touches the rotating part)
mk(new THREE.BoxGeometry(70, 12, 70), mats.panel, 1930, TOP + 6, Z_INS); mk(new THREE.CylinderGeometry(9, 9, 150, 20), mats.steel, 1976, TOP + 75, Z_INS);
mk(new THREE.BoxGeometry(34, 22, 22), mats.joint, 1960, INS[1], Z_INS); { const f = mk(new THREE.CylinderGeometry(17, 17, 6, 36), mats.deep, 1962, INS[1] + 22, Z_INS + 12); f.rotation.x = Math.PI / 2; const ring = mk(new THREE.TorusGeometry(16, 1.5, 8, 36), mats.accent, 1962, INS[1] + 22, Z_INS + 16); const tick = mk(new THREE.BoxGeometry(1.5, 10, 1), mats.steel, 1962, INS[1] + 26, Z_INS + 16); }
const plunger = mk(new THREE.CylinderGeometry(2.4, 2.4, 60, 12), mats.steel, 1930, INS[1], Z_INS); plunger.rotation.z = Math.PI / 2; let plungerX = 0;
// chute: opaque housing + dark opening (parts disappear below it)
mk(new THREE.BoxGeometry(66, 44, 58), mats.panel, CHUTE[0], TOP + 22, CHUTE[2]); mk(new THREE.BoxGeometry(38, 0.8, 30), mats.rubber, CHUTE[0], TOP + 44.5, CHUTE[2]);
// machined bushings (pool): hollow cylinder with one accent datum mark so wrist roll is readable
const partGeo = new THREE.LatheGeometry([new THREE.Vector2(7, -PART_H / 2), new THREE.Vector2(PART_R - 1.5, -PART_H / 2), new THREE.Vector2(PART_R, -PART_H / 2 + 1.5), new THREE.Vector2(PART_R, PART_H / 2 - 1.5), new THREE.Vector2(PART_R - 1.5, PART_H / 2), new THREE.Vector2(7, PART_H / 2)], 28);
const bushings = []; for (let i = 0; i < 6; i++) { const p = new THREE.Mesh(partGeo, mats.steel); p.castShadow = p.receiveShadow = true; const mark = new THREE.Mesh(new THREE.BoxGeometry(2, 8, 5), mats.accent); mark.position.set(PART_R + 0.4, 0, 0); p.add(mark); p.visible = false; scene.add(p); bushings.push(p); }
const wr = (() => { let s = (SEED ^ 0x9e3779b9) | 0; return () => { s = s + 0x6D2B79F5 | 0; let t = Math.imul(s ^ s >>> 15, 1 | s); t = t + Math.imul(t ^ t >>> 7, 61 | t) ^ t; return ((t ^ t >>> 14) >>> 0) / 4294967296; }; })();
const feeder = [null, null, null], fixture = [null, null, null]; let held = null, lastPocket = -1, drop = null;
const feedPos = i => [CX[i], TOP + 16 - 6 + PART_H / 2, Z_FEED], fixPos = j => [CX[j], TOP + 22 - 8 + PART_H / 2, Z_FIX];
const getPart = () => bushings.find(p => !p.visible);
function spawn(i, rise = false) { const p = getPart(); const [x, y, z] = feedPos(i); p.position.set(x, y - (rise ? 40 : 0), z); p.rotation.set(0, 0, 0); p.visible = true; p.rise = rise ? { y, t: 0 } : null; feeder[i] = p; }
for (let i = 0; i < 3; i++) spawn(i);

// =============== robot ===============
const parts = buildArm(); scene.add(parts.root);
const arm = new Arm3D(parts);
const OPEN = 34, HOVER = 210;
const zSpec = (G, phi, grip, o) => Object.assign({ m: 'z', G, phi, grip }, o);
const partRho = contactRho(PART_R, 0) / D;
arm.init(zSpec([CX[0], feedPos(0)[1] + 110, Z_FEED], 0, OPEN));

// =============== letters ===============
const engine = Engine.create({ positionIterations: 12, velocityIterations: 10 });
engine.gravity.y = 1; engine.gravity.scale = 0.0017;
const letters = []; let ox = X0;
const inside = (pts, holes, x, y) => { let c = false; for (const poly of [pts, ...holes]) for (let i = 0, j = poly.length - 1; i < poly.length; j = i++) { const [xi, yi] = poly[i], [xj, yj] = poly[j]; if ((yi > y) !== (yj > y) && x < (xj - xi) * (y - yi) / (yj - yi) + xi) c = !c; } return c; };
const segDist = (px, py, a, b) => { const dx = b[0] - a[0], dy = b[1] - a[1], t = clamp(((px - a[0]) * dx + (py - a[1]) * dy) / (dx * dx + dy * dy || 1), 0, 1); return Math.hypot(px - a[0] - t * dx, py - a[1] - t * dy); };
const letterMat = new THREE.MeshStandardMaterial({ color: PAL.text, roughness: 0.4, metalness: 0.06 });
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
  letters.push({ ch, body, mesh, slot, cx, cy, cands, loc, ex: 0, ext: bx1 - bx0, state: 'wall', release: 0.78 + rnd() * 0.22, minY: Math.min(...o3.map(p => p[1])), seat: null, rel: null });
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
const up = (G, d) => [G[0], G[1] + d, G[2]];
const blockers = L => { const out = []; for (const pr of engine.pairs.list) { if (!pr.isActive) continue; const a = letters.find(l => l.body === pr.bodyA.parent), b = letters.find(l => l.body === pr.bodyB.parent); if (!a || !b || (a !== L && b !== L)) continue; const o = a === L ? b : a; if (o.state === 'fall' && o.body.position.y < L.body.position.y - 14) out.push(o); } return out; };

// =============== script ===============
const TP = +(qs.get('tempo') || 0.72);   // tempo: scales commanded minimum durations and pauses (joint limits still apply)
let opened = false, interactive = false, intervene = false, knockQ = null, interventions = 0, plungerTarget = 0, refills = [];
const stopSoon = () => (!opened && settled) || intervene;
const wantStop = () => !held && ((!opened && settled) || intervene);
const tp = () => TP * (held && ((!opened && settled) || intervene) ? 0.62 : 1);   // finish a part in hand a bit faster when interrupted
const wait = s => { let t = 0; return dt => (t += dt) >= s * tp(); };
const until = f => () => f();
const gripTo = (d, T) => arm.gripTo(d, T * tp());
const act = f => () => { f(); return true; };
const mvJ = (s, T = 0.6) => arm.move(s, 'j', T * tp()), mvL = (s, T = 0.4) => arm.move(s, 'l', T * tp());
const withS = (o, T) => { const s0 = arm.spec; return Object.assign({}, s0, o); };

// ---------- normal work: feeder -> dial-gauge inspection -> press into fixture; pull + chute finished parts ----------
function* take(i) {
  const p = feeder[i], [x, y, z] = feedPos(i), hv = 96 + wr() * 44; phase = 'WORK · FEED';
  yield mvJ(zSpec([x, y + hv, z], 0, OPEN), 0.9 + wr() * 0.3); if (wantStop()) return false;
  yield mvL(zSpec([x, y + 26, z], 0, OPEN), 0.5); yield wait(0.06 + wr() * 0.16); yield mvL(zSpec([x, y, z], 0, OPEN), 0.32); yield wait(0.1);
  yield gripTo(partRho, 0.35); yield act(() => { feeder[i] = null; held = p; refills.push({ i, at: T + 0.9 + wr() * 0.6 }); }); yield wait(0.1);
  yield mvL(zSpec([x, y + hv * 0.8, z], 0, partRho), 0.45); return true;
}
function* gauge() {
  const [x, y, z] = INS, sw = (62 + wr() * 30) * D; phase = 'WORK · GAUGE';
  yield mvJ(zSpec([x, y + 76, z], 0, partRho), 1.0 + wr() * 0.3); if (stopSoon()) return; yield mvL(zSpec([x, y, z], 0, partRho), 0.6); yield wait(0.2);
  yield act(() => { plungerTarget = -5; }); yield wait(0.25);
  yield mvL(zSpec([x, y, z], 0, partRho, { roll: -sw }), 0.45); yield mvL(zSpec([x, y, z], 0, partRho, { roll: sw }), 1.0 + wr() * 0.4);
  if (wr() < 0.35 && !stopSoon()) { yield wait(0.15); yield mvL(zSpec([x, y, z], 0, partRho, { roll: -sw * 0.5 }), 0.9); }
  yield mvL(zSpec([x, y, z], 0, partRho, { roll: 0 }), 0.6); yield act(() => { plungerTarget = 0; }); yield wait(0.25 + wr() * 0.3);
  yield mvL(zSpec([x, y + 64, z], 0, partRho), 0.5);
}
function* press(j) {
  const [x, y, z] = fixPos(j); phase = 'WORK · INSERT';
  yield mvJ(zSpec([x, y + 92, z], 0, partRho), 1.1 + wr() * 0.3); yield mvL(zSpec([x, y + 22, z], 0, partRho), 0.55); if (!stopSoon()) yield wait(0.15 + wr() * 0.2);
  yield mvL(zSpec([x, y + 5, z], 0, partRho), 0.65);
  if (wr() < 0.5 && !stopSoon()) { const e = (2 + wr() * 2) * D; yield mvL(zSpec([x, y + 5, z], 0, partRho, { roll: e }), 0.22); yield mvL(zSpec([x, y + 5, z], 0, partRho, { roll: -e }), 0.28); yield mvL(zSpec([x, y + 5, z], 0, partRho, { roll: 0 }), 0.22); }
  yield mvL(zSpec([x, y, z], 0, partRho), 0.4); yield wait(0.15);
  yield act(() => { fixture[j] = held; held.position.set(...fixPos(j)); held.rotation.set(0, 0, 0); held = null; lastPocket = j; });
  yield gripTo(OPEN, 0.35); yield mvL(zSpec([x, y + 92, z], 0, OPEN), 0.5);
}
function* extract(j) {
  const p = fixture[j], [x, y, z] = fixPos(j); phase = 'WORK · UNLOAD';
  yield mvJ(zSpec([x, y + 92, z], 0, OPEN), 0.9 + wr() * 0.3); if (wantStop()) return; yield mvL(zSpec([x, y + 22, z], 0, OPEN), 0.5); yield mvL(zSpec([x, y, z], 0, OPEN), 0.35); yield wait(0.12);
  yield gripTo(partRho, 0.35); yield act(() => { fixture[j] = null; held = p; }); yield wait(0.12);
  yield mvL(zSpec([x, y + 28, z], 0, partRho), 0.4); yield mvL(zSpec([x, y + 96, z], 0, partRho), 0.45);
  const cy = TOP + 44 + PART_R + 34;
  yield mvJ(zSpec([CHUTE[0], cy, CHUTE[2]], 0, partRho), 1.0 + wr() * 0.3); yield wait(0.12);
  yield act(() => { drop = { p: held, vy: 0 }; held = null; }); yield gripTo(OPEN, 0.3); yield wait(0.2); yield mvL(zSpec([CHUTE[0], cy + 70, CHUTE[2]], 0, OPEN), 0.4);
}
function* taskCycle() {
  phase = 'WORK';
  const j = (lastPocket + 1 + (wr() < 0.3 ? 1 : 0)) % 3;
  if (fixture[j]) { yield* extract(j); if (wantStop()) return; }
  const free = [0, 1, 2].filter(i => feeder[i] && !feeder[i].rise); if (!free.length) { yield wait(0.3); return; }
  const i = free[(wr() * free.length) | 0];
  if (!(yield* take(i))) return;
  if (!stopSoon()) yield* gauge();   // skipped when something needs attention
  yield* press(j);
  if (wr() < 0.28) { const s0 = arm.spec; yield mvL(Object.assign({}, s0, { phi: (wr() - 0.5) * 0.12 }), 0.35); yield wait(0.2); yield mvL(Object.assign({}, s0, { phi: 0 }), 0.35); }   // occasional wrist re-check
  yield wait(0.1 + wr() * 0.5);
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
const calmFor = (L, sec) => { let t = 0; return dt => { const q = L.body.speed < 0.1 && Math.abs(L.body.angularSpeed) < 0.003; t = q ? t + dt : 0; return t >= sec; }; };
function* reaction(L) {   // the user knocked a letter down: stop, take it in, look at it, look at the user, fix it, look again, back to work
  interventions++; const k = 1 + 0.25 * Math.min(interventions - 1, 3);
  phase = 'NOTICED'; yield wait(0.12);
  const s0 = arm.spec; yield mvL(Object.assign({}, s0, { G: [s0.G[0], s0.G[1] + 14, s0.G[2]] }), 0.35);        // work stops; small lift of the tool
  yield calmFor(L, 0.55);
  const p = poseOf(L);
  phase = 'TURN TO LETTER'; yield mvJ(zSpec([p.x, p.y + 270, 0], 0, OPEN), 1.9); yield wait(0.8 * k);           // look at the letter
  phase = 'LOOK AT USER'; yield mvJ(zSpec([p.x + 60, 500, 330], 0, OPEN, { lk: 1.15 }), 1.35); yield wait(1.25 * k);   // ...really?
  yield mvL(zSpec([p.x + 60, 492, 330], 0, OPEN, { lk: 1.05 }), 0.5); yield wait(0.3);
  yield* carry(L, { kind: 'slot' });
  phase = 'PAUSE'; yield wait(0.7);
  phase = 'DONT DO THAT'; yield mvJ(zSpec([L.slot.x + 40, 480, 330], 0, OPEN, { lk: 1.15 }), 1.5); yield wait(0.9 * k);
  yield mvL(zSpec([L.slot.x + 40, 480, 330], 0, OPEN, { lk: 0.98 }), 0.45); yield mvL(zSpec([L.slot.x + 40, 480, 330], 0, OPEN, { lk: 1.17 }), 0.5); yield wait(0.55);
}
function* script() {
  while (!settled) yield* taskCycle();
  phase = 'NOTICE'; yield wait(0.3);
  yield mvJ(zSpec([1720, 540, Z_INS], 0, OPEN), 1.1); yield wait(0.55);
  phase = 'TURN'; yield mvJ(zSpec([900, 420, 120], 0, OPEN), 2.4); yield wait(0.4);
  for (const L of letters) yield* serve(L);
  phase = 'CHECK'; yield wait(0.7); yield mvL(zSpec(up([letters[5].slot.x, letters[5].slot.y, 0], 170), 0.05, OPEN), 0.3); yield mvL(zSpec(up([letters[5].slot.x, letters[5].slot.y, 0], 170), -0.03, OPEN), 0.3); yield wait(0.8);
  phase = 'RETURN TO WORK'; yield mvJ(zSpec([CX[1], feedPos(1)[1] + 110, Z_FEED], 0, OPEN), 2.6);
  opened = true; interactive = !reduce;
  for (;;) {
    if (knockQ) { const L = knockQ; yield* reaction(L); knockQ = null; intervene = false; interactive = true; continue; }
    yield* taskCycle();
  }
}
const gen = script(); let cur = null;
function runScript(dt) { for (let n = 0; n < 8; n++) { if (!cur) { const r = gen.next(); if (r.done) return; cur = r.value; } if (cur(dt)) { cur = null; dt = 0; } else return; } }

const _p = new THREE.Vector3(), _q = new THREE.Quaternion(), _s = new THREE.Vector3(), qFix = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0, 0, 1), -Math.PI / 2);
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
  // work-cell bookkeeping: held part follows the tool; magazine refills; chute drop; gauge plunger
  if (held) { const m = arm.toolMatrix(); m.decompose(_p, _q, _s); _q.multiply(qFix); held.quaternion.copy(_q); held.position.copy(arm.gWorld()); }
  for (const r of refills.slice()) if (T >= r.at) { refills.splice(refills.indexOf(r), 1); if (!feeder[r.i]) spawn(r.i, true); }
  for (const p of feeder) if (p && p.rise) { p.position.y = Math.min(p.rise.y, p.position.y + 70 * DT); if (p.position.y >= p.rise.y) p.rise = null; }
  if (drop) { drop.vy -= 1700 * DT; drop.p.position.y += drop.vy * DT; if (drop.p.position.y < TOP + 44 - 40) { drop.p.visible = false; drop = null; } }
  plungerX += (plungerTarget - plungerX) * Math.min(1, DT * 10); plunger.position.x = 1936 + plungerX;
}

// =============== user interaction: click a letter to knock it off ===============
const ray = new THREE.Raycaster(), ndc = new THREE.Vector2(), TOL = [[0, 0]];
for (const r of [5, 10]) for (let k = 0; k < 8; k++) TOL.push([Math.cos(k * Math.PI / 4) * r, Math.sin(k * Math.PI / 4) * r]);
function pickLetter(cx, cy) {      // the visible letter is the target; a ~10px forgiving ring, nearest-to-pointer wins
  const r = renderer.domElement.getBoundingClientRect(), live = letters.filter(l => l.state === 'placed'); if (!live.length) return null;
  const meshes = live.map(l => l.mesh); let best = null;
  for (const [ox, oy] of TOL) {
    ndc.set((cx - r.left + ox) / r.width * 2 - 1, -((cy - r.top + oy) / r.height * 2 - 1)); ray.setFromCamera(ndc, camera);
    const h = ray.intersectObjects(meshes, false)[0]; if (h) { const d = Math.hypot(ox, oy); if (!best || d < best.d) best = { d, L: live[meshes.indexOf(h.object)], pt: h.point }; }
  }
  return best;
}
function knock(L, pt) {
  Body.setPosition(L.body, { x: L.slot.x, y: FLOOR - L.slot.y }); Body.setAngle(L.body, 0);
  Body.setStatic(L.body, false); L.body.isSensor = false; L.state = 'fall';
  const dx = clamp((pt.x - L.slot.x) / (L.ext / 2), -1, 1);           // off-centre hit -> rotation + sideways drift, like a real tap
  Body.setVelocity(L.body, { x: -dx * 0.5 + (rnd() - 0.5) * 0.5, y: -0.25 }); Body.setAngularVelocity(L.body, dx * 0.05 * (0.6 + rnd() * 0.8) + (rnd() - 0.5) * 0.03);
  knockQ = L; interactive = false; intervene = true;
}
function onPointer(e) { if (!interactive || knockQ) return; const h = pickLetter(e.clientX, e.clientY); if (h) knock(h.L, h.pt); }
renderer.domElement.addEventListener('pointerdown', onPointer);
renderer.domElement.addEventListener('pointermove', e => { renderer.domElement.style.cursor = interactive && !knockQ && pickLetter(e.clientX, e.clientY) ? 'pointer' : ''; });

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
  click: (cx, cy) => onPointer({ clientX: cx, clientY: cy }), get interactive() { return interactive; }, get opened() { return opened; }, get knocked() { return !!knockQ; },
  screenOf(i) { const v = letters[i].mesh.position.clone().project(camera), r = renderer.domElement.getBoundingClientRect(); return [r.left + (v.x + 1) / 2 * r.width, r.top + (1 - v.y) / 2 * r.height]; },
  q: () => ({ ...arm.q }), qd: () => ({ ...arm.qd }), render: () => renderer.render(scene, camera),
};
fit(); addEventListener('resize', () => { fit(); renderer.render(scene, camera); });
if (reduce) { for (const L of letters) { L.state = 'placed'; L.mesh.position.set(L.slot.x, L.slot.y, 0); L.mesh.rotation.set(0, 0, 0); } renderer.render(scene, camera); }
else if (!qs.has('manual')) requestAnimationFrame(frame);
(document.fonts ? document.fonts.ready : Promise.resolve()).then(() => document.body.classList.add('ready'));
