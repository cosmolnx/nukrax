/* NUKRAX prototype V4 — precision inspection cell (right of the robot).
   Rail feeder with escapement + hopper  ->  pneumatic-clamp inspection fixture with dial-gauge probe  ->  finished-part cassette
   (swaps out when full) / reject bin.  Everything animated here is mechanical: slide, clamp, probe, cassette, status LEDs. */
import * as THREE from 'three';
import { M, carbon } from './materials.js';
import { Mh, cylX, cylY, cylZ, box, rbox, boltsY, boltsZ, tube, clampBlock } from './arm3d.js';

export const TOP = 44, PART_R = 13, PART_H = 26;
const FZ = 40, XZ = 200, TZ = 330, RZ = 110;                       // feeder / fixture / tray / reject-bin z
const FIX_X = 1860, CAS_X = 1925, REJ_X = 1938;

export function buildCell(scene) {
  const g = new THREE.Group(); scene.add(g); const add = o => (g.add(o), o);
  const cf = carbon(1 / 30);
  const boltsAt = (x, z, y, R, n, r) => { const b = new THREE.Group(); b.position.set(x, 0, z); boltsY(b, y, R, n, r); g.add(b); };
  // ---- machined table: body, ground top plate with T-slots, corner fasteners ----
  add(box(305, TOP - 8, 380, M.bed, 1842, (TOP - 8) / 2, 200));
  add(rbox(305, 8, 380, 3, M.anodized, 1842, TOP - 4, 200));
  for (let i = 0; i < 9; i++) add(box(300, 0.8, 2.6, M.rubber, 1842, TOP + 0.1, 28 + i * 40));
  boltsAt(1962, XZ, TOP + 13, 20, 4, 3); boltsAt(FIX_X, XZ, TOP + 26, 38, 4, 3.4);
  for (const [x, z] of [[1700, 20], [1985, 20], [1700, 380], [1985, 380]]) { add(cylY(7, 3, M.machined, x, TOP + 0.8, z)); }
  add(box(2, 6, 380, M.machined, 1693, TOP + 3, 200)); add(box(2, 6, 380, M.machined, 1991, TOP + 3, 200));
  // ---- feeder: hopper + rail + escapement stop + pusher cylinder ----
  add(rbox(120, 12, 54, 4, M.anodized, 1762, TOP + 6, FZ));
  for (const s of [-1, 1]) add(box(104, 5, 4, M.machined, 1764, TOP + 14.5, FZ + s * 15));
  add(box(104, 1, 22, M.rubber, 1764, TOP + 12.6, FZ));
  add(rbox(46, 62, 56, 7, M.graphite, 1700, TOP + 43, FZ)); add(box(34, 2, 44, cf, 1700, TOP + 75, FZ)); add(box(30, 12, 3, M.rubber, 1724, TOP + 36, FZ));
  boltsZ(g, 1700, TOP + 38, FZ + 29, 14, 4, 2.6);
  add(rbox(10, 22, 40, 2, M.anodized, 1810, TOP + 23, FZ));                                    // escapement stop
  add(cylX(7, 48, M.satin, 1742, TOP + 26, FZ + 24)); add(cylX(2.6, 30, M.machined, 1778, TOP + 26, FZ + 24)); add(rbox(8, 14, 12, 2, M.anodized, 1798, TOP + 26, FZ + 24));
  // ---- inspection fixture: base, ground plate, nest ring, locators, clamp jaws on pneumatic slides ----
  add(rbox(104, 22, 104, 6, M.anodized, FIX_X, TOP + 11, XZ)); add(box(84, 3, 84, M.satin, FIX_X, TOP + 23.5, XZ));
  for (let i = 0; i < 4; i++) { const a = i * Math.PI / 2 + 0.78; add(cylY(3.2, 6, M.machined, FIX_X + Math.cos(a) * 33, TOP + 28, XZ + Math.sin(a) * 33)); }
  add(cylY(19.5, 3, M.machined, FIX_X, TOP + 26.5, XZ)); add(cylY(15, 3.2, M.rubber, FIX_X, TOP + 26.8, XZ));
  const jaws = []; for (const s of [-1, 1]) {
    const j = new THREE.Group(); j.position.set(FIX_X, TOP + 36, XZ); g.add(j);
    j.add(rbox(18, 22, 20, 3, M.anodized, 0, 0, 0)); j.add(box(16, 8, 5, M.pad, 0, -4, -s * 8)); j.add(cylZ(3.5, 40, M.machined, 0, 0, s * 26));
    add(cylZ(10, 44, M.satin, FIX_X, TOP + 36, XZ + s * 66)); add(rbox(18, 28, 8, 2, M.anodized, FIX_X, TOP + 36, XZ + s * 90)); add(box(14, 4, 52, M.machined, FIX_X, TOP + 24, XZ + s * 62));
    jaws.push({ g: j, s });
  }
  // ---- dial-gauge probe: column, electronics box with glass readout, carriage + plunger ----
  add(rbox(34, 112, 34, 6, M.graphite, 1962, TOP + 56, XZ)); add(rbox(52, 12, 52, 4, M.anodized, 1962, TOP + 6, XZ)); 
  add(rbox(40, 30, 28, 4, M.charcoal, 1962, TOP + 94, XZ + 22)); add(box(30, 15, 1.4, M.glass, 1962, TOP + 96, XZ + 36.5)); add(box(15, 1.4, 1.6, M.accent, 1958, TOP + 98, XZ + 36.1));
  const gauge = new THREE.Group(); gauge.position.set(1930, TOP + 29 + 14, XZ); g.add(gauge);
  gauge.add(cylX(10, 52, M.anodized, 16, 0, 0)); gauge.add(cylX(12, 8, M.machined, 38, 0, 0)); gauge.add(cylX(2.6, 44, M.machined, -22, 0, 0)); gauge.add(cylX(3.6, 4, M.accent, -44, 0, 0));
  gauge.add(box(24, 6, 10, M.satin, 20, 12, 0)); boltsX_(gauge);
  tube(g, [[1962, TOP + 10, XZ - 14], [1976, TOP + 8, XZ - 40], [1980, TOP + 6, XZ - 80]], 3.4);
  // ---- finished-part cassette on a guide rail (swaps out when full) + reject bin ----
  add(box(250, 5, 10, M.machined, 1990, TOP + 2.5, TZ + 56)); add(box(250, 5, 10, M.machined, 1990, TOP + 2.5, TZ - 56));
  const cas = new THREE.Group(); cas.position.set(CAS_X, 0, TZ); g.add(cas);
  cas.add(rbox(112, 10, 88, 5, M.anodized, 0, TOP + 5, 0)); cas.add(rbox(102, 6, 78, 3, M.graphite, 0, TOP + 13, 0));
  const pockets = [[-32, -17], [0, -17], [32, -17], [-32, 17], [0, 17], [32, 17]];
  for (const [x, z] of pockets) { cas.add(cylY(16.5, 1, M.machined, x, TOP + 16.2, z)); cas.add(cylY(14.4, 1.2, M.rubber, x, TOP + 16.7, z)); }
  cas.add(box(108, 3, 2, M.machined, 0, TOP + 17, 42)); cas.add(box(20, 2, 1.6, M.accent, -34, TOP + 17.2, 42.2));
  add(rbox(72, 44, 64, 6, M.charcoal, REJ_X, TOP + 22, RZ)); add(box(38, 0.8, 30, M.rubber, REJ_X, TOP + 44.4, RZ)); add(box(70, 3, 4, M.machined, REJ_X, TOP + 40, RZ + 33));
  // ---- manifold + pneumatic hoses (to clamp cylinders and feeder pusher), cable duct ----
  add(rbox(40, 30, 36, 4, M.anodized, 1990, TOP + 17, 270)); boltsZ(g, 1990, TOP + 17, 289, 9, 3, 2.6);
  tube(g, [[1990, TOP + 26, 262], [1980, TOP + 12, 248], [FIX_X + 20, TOP + 8, XZ + 90]], 3.2); tube(g, [[1990, TOP + 26, 250], [1975, TOP + 14, 230], [FIX_X + 30, TOP + 8, XZ - 90]], 3.2);
  tube(g, [[1990, TOP + 30, 262], [1960, TOP + 44, 150], [1796, TOP + 40, FZ + 40], [1746, TOP + 28, FZ + 30]], 3);
  // ---- status LEDs (tiny, matte until active) ----
  const led = (x, y, z) => { const m = new THREE.Mesh(new THREE.CylinderGeometry(2.6, 2.6, 2.4, 14).rotateX(Math.PI / 2), M.ledOff); m.position.set(x, y, z); g.add(m); return m; };
  const leds = { fix: led(FIX_X - 36, TOP + 12, XZ + 53), probe: led(1962, TOP + 84, XZ + 17.5), feed: led(1700, TOP + 58, FZ + 28.6) };
  // ---- machined bushings (pool): hollow cylinder with one accent datum mark so rotation is readable ----
  const partGeo = new THREE.LatheGeometry([[7, -PART_H / 2], [PART_R - 1.5, -PART_H / 2], [PART_R, -PART_H / 2 + 1.5], [PART_R, PART_H / 2 - 1.5], [PART_R - 1.5, PART_H / 2], [7, PART_H / 2]].map(([x, y]) => new THREE.Vector2(x, y)), 32);
  const pool = []; for (let i = 0; i < 9; i++) { const p = new THREE.Mesh(partGeo, M.machined); p.castShadow = p.receiveShadow = true; const mk = new THREE.Mesh(new THREE.BoxGeometry(1.6, 8, 5), M.accent); mk.position.set(PART_R + 0.3, 0, 0); p.add(mk); p.visible = false; scene.add(p); pool.push(p); }

  const cell = {
    group: g, pool, leds, TOP,
    feedPos: k => [1792 - 32 * k, TOP + 12 + PART_H / 2, FZ], hopper: [1702, TOP + 12 + PART_H / 2, FZ],
    fixSeat: [FIX_X, TOP + 26 + 3 - 8 + PART_H / 2 - 2, XZ],
    trayPos: i => [CAS_X + pockets[i][0], TOP + 16 + 0.5 - 4 + PART_H / 2, TZ + pockets[i][1]],
    rejectTop: [REJ_X, TOP + 44, RZ],
    clamp: 0, clampT: 0, probe: -2.6, probeT: -2.6, cassette: { x: 0, state: 'idle', t: 0 }, feedQ: [], tray: [], drops: [],
    getPart() { return pool.find(p => !p.visible && !p.busy); },
    show(p, pos) { p.visible = true; p.position.set(...pos); p.rotation.set(0, 0, 0); p.tw = null; },
    slide(p, pos, speed = 90) { p.tw = { to: pos, v: speed }; },
    setLed(n, on) { leds[n].material = on ? M.ledOn : M.ledOff; },
    swapCassette() { if (this.cassette.state === 'idle') { this.cassette.state = 'out'; this.cassette.t = 0; } },
    step(dt) {
      this.clamp += (this.clampT - this.clamp) * Math.min(1, dt * (this.clampT > this.clamp ? 7 : 6));
      for (const j of jaws) j.g.position.z = XZ + j.s * (PART_R + 9 + (1 - this.clamp) * 24);
      this.probe += (this.probeT - this.probe) * Math.min(1, dt * 8); gauge.position.x = 1930 - this.probe * 15;
      for (const p of pool) if (p.tw) { const t = p.tw, dx = t.to[0] - p.position.x, dy = t.to[1] - p.position.y, dz = t.to[2] - p.position.z, L = Math.hypot(dx, dy, dz), s = Math.min(L, t.v * dt); if (L < 0.2) { p.position.set(...t.to); p.tw = null; } else p.position.add(new THREE.Vector3(dx, dy, dz).multiplyScalar(s / L)); }
      for (const d of this.drops.slice()) { d.vy -= 1700 * dt; d.p.position.y += d.vy * dt; if (d.p.position.y < TOP + 44 - 42) { d.p.visible = false; d.p.busy = false; this.drops.splice(this.drops.indexOf(d), 1); } }
      const c = this.cassette;
      if (c.state !== 'idle') {
        c.t += dt; const dur = 1.0;
        if (c.state === 'out') { c.x = 205 * Math.min(1, c.t / dur) ** 1.6 * 0 + 205 * smooth(c.t / dur); if (c.t >= dur) { for (const p of this.tray) { p.visible = false; p.busy = false; } this.tray.length = 0; c.state = 'in'; c.t = 0; } }
        else if (c.state === 'in') { c.x = 205 * (1 - smooth(c.t / (dur * 1.2))); if (c.t >= dur * 1.2) { c.x = 0; c.state = 'idle'; } }
        cas.position.x = CAS_X + c.x; for (const p of this.tray) p.position.x = p.home[0] + c.x;
      }
    },
  };
  return cell;
}
const smooth = t => { t = Math.max(0, Math.min(1, t)); return t * t * (3 - 2 * t); };
function boltsX_(parent) { for (let i = 0; i < 4; i++) { const a = i / 4 * Math.PI * 2 + 0.78; parent.add(cylX(2.4, 3, M.machined, 40, Math.sin(a) * 8, Math.cos(a) * 8)); } }
