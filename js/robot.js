/* NUKRAX prototype — articulated industrial arm.
   Kinematics: base yaw + planar 3-link chain (shoulder, elbow, wrist pitch) + parallel-jaw gripper.
   The arm plane is rotated by `yaw` about the vertical base axis and projected obliquely, so the
   base turn reads as real depth (arm swings toward the viewer) while yaw = PI / 0 gives a pure side profile. */
(function () {
  const C = { bx: 1470, floor: 900, hS: 310, L1: 540, L2: 510, Lt: 110, E: 0.26 };
  const clamp = (v, a, b) => Math.max(a, Math.min(b, v));
  const ease = t => { t = clamp(t, 0, 1); return t * t * t * (t * (t * 6 - 15) + 10); }; // min-jerk
  const lerp = (a, b, t) => a + (b - a) * t;

  function ik(r, h, g) {
    const wr = r - C.Lt * Math.sin(g), wh = h + C.Lt * Math.cos(g);
    const dx = wr, dy = wh - C.hS;
    const d = clamp(Math.hypot(dx, dy), Math.abs(C.L1 - C.L2) + 8, C.L1 + C.L2 - 4);
    const A = Math.acos(clamp((C.L1 * C.L1 + d * d - C.L2 * C.L2) / (2 * C.L1 * d), -1, 1));
    const a = Math.atan2(dy, dx) + A;
    const er = C.L1 * Math.cos(a), eh = C.hS + C.L1 * Math.sin(a);
    return { a, b: Math.atan2(wh - eh, wr - er), g };
  }

  class Arm {
    constructor() { this.q = { yaw: 0, a: 0.5, b: -0.9, g: 0, grip: 70 }; }
    // ---- kinematics (plane coords: r along heading, h up) ----
    fk(q = this.q) {
      const S = [0, C.hS];
      const E = [S[0] + C.L1 * Math.cos(q.a), S[1] + C.L1 * Math.sin(q.a)];
      const W = [E[0] + C.L2 * Math.cos(q.b), E[1] + C.L2 * Math.sin(q.b)];
      const G = [W[0] + C.Lt * Math.sin(q.g), W[1] - C.Lt * Math.cos(q.g)];
      return { S, E, W, G };
    }
    P(pt, yaw = this.q.yaw) { return [C.bx + pt[0] * Math.cos(yaw), C.floor - pt[1] + pt[0] * Math.sin(yaw) * Math.sin(C.E)]; }
    V(dr, dh, yaw = this.q.yaw) { return [dr * Math.cos(yaw), -dh + dr * Math.sin(yaw) * Math.sin(C.E)]; }
    pose() { const f = this.fk(); return { yaw: this.q.yaw, r: f.G[0], h: f.G[1], g: this.q.g, grip: this.q.grip }; }
    set(p) { Object.assign(this.q, ik(p.r, p.h, p.g), { yaw: p.yaw, grip: p.grip }); }
    // tool frame in screen space (G = grasp centre, a2 = tool axis, c2 = closing axis)
    tool() {
      const f = this.fk(), g = this.q.g, y = this.q.yaw;
      return { G: this.P(f.G), W: this.P(f.W), a2: this.V(Math.sin(g), -Math.cos(g)), c2: this.V(Math.cos(g), Math.sin(g)) };
    }
    // ---- motion ----
    begin(p, dur, mode) {
      const cur = this.pose(); p = Object.assign({}, cur, p);
      return { dur, mode, u: 0, p0: cur, p1: p, q0: Object.assign({}, this.q), q1: Object.assign({ yaw: p.yaw, grip: p.grip }, ik(p.r, p.h, p.g)) };
    }
    advance(m, dt) {
      m.u = Math.min(1, m.u + dt / m.dur);
      if (m.mode === 'cart') {
        const e = ease(m.u), a = m.p0, b = m.p1;
        this.set({ yaw: lerp(a.yaw, b.yaw, e), r: lerp(a.r, b.r, e), h: lerp(a.h, b.h, e), g: lerp(a.g, b.g, e), grip: lerp(a.grip, b.grip, e) });
      } else { // joint space, staggered: base -> shoulder -> elbow -> wrist
        const D = { yaw: 0, a: 0.05, b: 0.12, g: 0.2, grip: 0.3 };
        for (const k in D) this.q[k] = lerp(m.q0[k], m.q1[k], ease((m.u - D[k]) / (1 - D[k])));
      }
      return m.u >= 1;
    }
    // ---- rendering ----
    draw(ctx, held) {
      const q = this.q, f = this.fk(), P = pt => this.P(pt);
      const S = P(f.S), E = P(f.E), W = P(f.W), cy = Math.cos(q.yaw), sy = Math.sin(q.yaw);
      this.base(ctx, sy);
      // shadow of tool on floor
      const T = this.tool(), hh = clamp((C.floor - T.G[1]) / 500, 0, 1);
      ctx.fillStyle = `rgba(0,0,0,${0.38 * (1 - hh * 0.7)})`;
      ctx.beginPath(); ctx.ellipse(T.G[0], C.floor + 12, 80 + hh * 60, 9 + hh * 8, 0, 0, 7); ctx.fill();
      // upper arm: tapered plate with recessed panel
      capsule(ctx, S, E, 134, 92, 0); capsule(ctx, lerpP(S, E, 0.08), lerpP(S, E, 0.9), 70, 44, 1);
      band(ctx, S, E, 0.5, 100); band(ctx, S, E, 0.54, 100);
      this.disc(ctx, S, 82, cy, 1); this.disc(ctx, S, 44, cy, 2);
      // forearm: cylinder with rings
      capsule(ctx, E, W, 92, 62, 0);
      for (const t of [0.12, 0.3, 0.5, 0.72]) band(ctx, E, W, t, lerp(88, 62, t));
      plate(ctx, E, W, 0.4);
      this.disc(ctx, E, 66, cy, 1); this.disc(ctx, E, 34, cy, 2);
      this.cable(ctx, S, E, W, T);
      // wrist
      const a2 = T.a2, c2 = T.c2, wt = (u, v) => [T.W[0] + c2[0] * u + a2[0] * v, T.W[1] + c2[1] * u + a2[1] * v];
      this.disc(ctx, W, 44, cy, 1); this.disc(ctx, W, 22, cy, 2);
      if (held) held();
      this.gripper(ctx, wt, q.grip);
    }
    gripper(ctx, wt, grip) {
      const quad = (u0, v0, u1, v1, pal) => {
        const p = [wt(u0, v0), wt(u1, v0), wt(u1, v1), wt(u0, v1)];
        ctx.beginPath(); ctx.moveTo(...p[0]); p.slice(1).forEach(x => ctx.lineTo(...x)); ctx.closePath();
        const g = ctx.createLinearGradient(...wt(u0, 0), ...wt(u1, 0));
        pal.forEach((c, i) => g.addColorStop(i / (pal.length - 1), c));
        ctx.fillStyle = g; ctx.fill(); ctx.strokeStyle = 'rgba(207,222,234,.2)'; ctx.lineWidth = 1; ctx.stroke();
      };
      const M = ['#222d33', '#6c7f88', '#b9cad2', '#7a8d96', '#242f35'];
      quad(-34, -4, 34, 34, M);               // wrist flange
      quad(-124, 34, 124, 70, M);             // gripper rail body
      const a = wt(-124, 52), b = wt(124, 52); ctx.strokeStyle = 'rgba(143,184,196,.55)'; ctx.lineWidth = 2; ctx.beginPath(); ctx.moveTo(...a); ctx.lineTo(...b); ctx.stroke();
      for (const s of [-1, 1]) {
        const iu = s * grip / 2;
        quad(s > 0 ? iu : iu - 18, 70, s > 0 ? iu + 18 : iu, 150, M);     // finger
        quad(s > 0 ? iu - 4 : iu, 96, s > 0 ? iu : iu + 4, 146, ['#8FB8C4', '#8FB8C4']); // pad
        quad(s * (grip / 2 + 9) - 14, 60, s * (grip / 2 + 9) + 14, 72, ['#2a353c', '#58696f', '#2a353c']); // slider
      }
    }
    base(ctx, sy) {
      const bx = C.bx, fl = C.floor, y0 = fl - C.hS;
      ctx.fillStyle = 'rgba(0,0,0,.5)'; ctx.beginPath(); ctx.ellipse(bx, fl + 16, 200, 30, 0, 0, 7); ctx.fill();
      const hg = (x0, x1, pal) => { const g = ctx.createLinearGradient(x0, 0, x1, 0); pal.forEach((c, i) => g.addColorStop(i / (pal.length - 1), c)); return g; };
      const steel = ['#1c262b', '#56676f', '#a9bbc4', '#6c7f88', '#1f2a30'];
      ctx.fillStyle = hg(bx - 190, bx + 190, steel); ctx.beginPath(); ctx.ellipse(bx, fl + 6, 176, 26, 0, 0, 7); ctx.fill(); // plate
      ctx.fillStyle = '#10171b'; ctx.fillRect(bx - 176, fl - 10, 352, 16); ctx.fillStyle = hg(bx - 176, bx + 176, steel); ctx.fillRect(bx - 176, fl - 14, 352, 10);
      for (let i = -3; i <= 3; i += 6) { ctx.fillStyle = '#0d1317'; ctx.beginPath(); ctx.arc(bx + i * 52, fl - 6, 9, 0, 7); ctx.fill(); }
      ctx.fillStyle = hg(bx - 104, bx + 104, steel); ctx.fillRect(bx - 104, fl - 150, 208, 140);          // pedestal
      ctx.fillStyle = 'rgba(8,12,15,.55)'; for (const y of [fl - 120, fl - 70]) ctx.fillRect(bx - 104, y, 208, 3);
      ctx.fillStyle = hg(bx - 118, bx + 118, steel); ctx.fillRect(bx - 118, fl - 164, 236, 20);           // slew ring
      const off = -Math.sin(Math.PI - Math.PI + (this.q.yaw)) * 0; // (kept for clarity: turret centred)
      ctx.fillStyle = hg(bx - 96, bx + 96, steel); ctx.fillRect(bx - 96, y0 + 40, 192, fl - 164 - y0 - 40); // turret
      const mx = bx - Math.sin(this.q.yaw) * 100;                                                           // side motor housing rotates into view
      ctx.fillStyle = hg(mx - 56, mx + 56, steel); ctx.fillRect(mx - 56, y0 - 48, 112, 150);
      ctx.fillStyle = '#10171b'; ctx.fillRect(mx - 56, y0 + 40, 112, 4);
    }
    disc(ctx, c, R, cy, kind) {
      const rx = Math.max(R * Math.abs(cy), kind === 1 ? 16 : 7);
      const g = ctx.createRadialGradient(c[0] - R * 0.25, c[1] - R * 0.3, R * 0.1, c[0], c[1], R);
      if (kind === 1) { g.addColorStop(0, '#c7d6dd'); g.addColorStop(0.7, '#6a7c85'); g.addColorStop(1, '#232e34'); }
      else { g.addColorStop(0, '#3a474e'); g.addColorStop(1, '#161e23'); }
      ctx.fillStyle = g; ctx.beginPath(); ctx.ellipse(c[0], c[1], rx, R, 0, 0, 7); ctx.fill();
      ctx.strokeStyle = 'rgba(207,222,234,.25)'; ctx.lineWidth = 1.2; ctx.stroke();
      if (kind === 1 && rx > 30) { ctx.fillStyle = '#10171b'; for (let i = 0; i < 6; i++) { const a = i * 1.047; ctx.beginPath(); ctx.arc(c[0] + Math.cos(a) * rx * 0.82, c[1] + Math.sin(a) * R * 0.82, 3.2, 0, 7); ctx.fill(); } }
    }
    cable(ctx, S, E, W, T) {
      const a = lerpP(E, W, 0.55), b = [T.W[0] - 18, T.W[1] + 8], sag = 70 + 0.12 * Math.hypot(b[0] - a[0], b[1] - a[1]);
      ctx.lineCap = 'round'; ctx.strokeStyle = '#0b1013'; ctx.lineWidth = 7;
      ctx.beginPath(); ctx.moveTo(...a); ctx.quadraticCurveTo((a[0] + b[0]) / 2, Math.max(a[1], b[1]) + sag, ...b); ctx.stroke();
      ctx.strokeStyle = 'rgba(143,184,196,.35)'; ctx.lineWidth = 1.5; ctx.stroke();
    }
  }
  const lerpP = (a, b, t) => [lerp(a[0], b[0], t), lerp(a[1], b[1], t)];
  function capsule(ctx, a, b, wa, wb, inset) {
    const dx = b[0] - a[0], dy = b[1] - a[1], L = Math.hypot(dx, dy) || 1; let nx = -dy / L, ny = dx / L, ang = Math.atan2(dy, dx);
    ctx.beginPath(); ctx.moveTo(a[0] + nx * wa / 2, a[1] + ny * wa / 2); ctx.lineTo(b[0] + nx * wb / 2, b[1] + ny * wb / 2);
    ctx.arc(b[0], b[1], wb / 2, ang + Math.PI / 2, ang - Math.PI / 2, true); ctx.lineTo(a[0] - nx * wa / 2, a[1] - ny * wa / 2);
    ctx.arc(a[0], a[1], wa / 2, ang - Math.PI / 2, ang + Math.PI / 2, true); ctx.closePath();
    if (nx * -0.6 + ny * -0.8 < 0) { nx = -nx; ny = -ny; }
    const m = lerpP(a, b, 0.5), w = Math.max(wa, wb) / 2, g = ctx.createLinearGradient(m[0] + nx * w, m[1] + ny * w, m[0] - nx * w, m[1] - ny * w);
    (inset ? ['#0f161a', '#2b373d', '#46565e', '#2b373d', '#10171b'] : ['#cfdde4', '#9fb1ba', '#667a84', '#33424a', '#1a2328']).forEach((c, i, A) => g.addColorStop(i / (A.length - 1), c));
    ctx.fillStyle = g; ctx.fill(); ctx.strokeStyle = 'rgba(207,222,234,.22)'; ctx.lineWidth = 1.2; ctx.stroke();
  }
  function band(ctx, a, b, t, w) {
    const dx = b[0] - a[0], dy = b[1] - a[1], L = Math.hypot(dx, dy) || 1, nx = -dy / L, ny = dx / L, c = lerpP(a, b, t);
    ctx.strokeStyle = 'rgba(8,12,15,.7)'; ctx.lineWidth = 3; ctx.beginPath(); ctx.moveTo(c[0] + nx * w / 2, c[1] + ny * w / 2); ctx.lineTo(c[0] - nx * w / 2, c[1] - ny * w / 2); ctx.stroke();
    ctx.strokeStyle = 'rgba(207,222,234,.25)'; ctx.lineWidth = 1; ctx.beginPath(); ctx.moveTo(c[0] + nx * w / 2 + dx / L * 4, c[1] + ny * w / 2 + dy / L * 4); ctx.lineTo(c[0] - nx * w / 2 + dx / L * 4, c[1] - ny * w / 2 + dy / L * 4); ctx.stroke();
  }
  function plate(ctx, a, b, t) {
    const dx = b[0] - a[0], dy = b[1] - a[1], L = Math.hypot(dx, dy) || 1, c = lerpP(a, b, t);
    ctx.save(); ctx.translate(c[0], c[1]); ctx.rotate(Math.atan2(dy, dx)); ctx.fillStyle = '#10171b'; ctx.fillRect(-26, -10, 52, 20);
    ctx.fillStyle = '#8FB8C4'; ctx.fillRect(-20, -4, 22, 2); ctx.fillStyle = '#46626B'; ctx.fillRect(-20, 1, 34, 2); ctx.restore();
  }
  window.NKXRobot = { Arm, C, ease, clamp };
})();
