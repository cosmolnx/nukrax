/* NUKRAX prototype — scene: Matter.js rigid-body letters + choreography script for the arm.
   Everything runs on a fixed 60 Hz simulation clock (deterministic for a given ?seed). */
(function () {
  const { Engine, Bodies, Body, Composite, Common } = Matter;
  const { Arm, C, clamp } = NKXRobot;
  Common.setDecomp(window.decomp);
  const W = 1920, H = 1080, FLOOR = C.floor, S = 0.056, BASE_Y = 470, X0 = 430, BX = C.bx;
  const qs = new URLSearchParams(location.search);
  const SEED = +(qs.get('seed') || 24), DEBUG = qs.has('debug'), SPEED = +(qs.get('speed') || 1);
  const reduce = matchMedia('(prefers-reduced-motion: reduce)').matches;
  const DT = 1 / 60, SPIN = +(qs.get('spin') || 1);
  let rs = SEED; const rnd = () => { rs |= 0; rs = rs + 0x6D2B79F5 | 0; let t = Math.imul(rs ^ rs >>> 15, 1 | rs); t = t + Math.imul(t ^ t >>> 7, 61 | t) ^ t; return ((t ^ t >>> 14) >>> 0) / 4294967296; };

  // ---------- letters ----------
  const engine = Engine.create({ positionIterations: 12, velocityIterations: 10 });
  engine.gravity.y = 1; engine.gravity.scale = 0.0017;
  const letters = []; let ox = X0;
  for (const ch of 'NUKRAX') {
    const g = NKX_GLYPHS.g[ch], pts = g.c.map(([x, y]) => [x * S, -y * S]);
    let A = 0, cx = 0, cy = 0;
    for (let i = 0; i < pts.length; i++) { const p = pts[i], q = pts[(i + 1) % pts.length], f = p[0] * q[1] - q[0] * p[1]; A += f; cx += (p[0] + q[0]) * f; cy += (p[1] + q[1]) * f; }
    cx /= 3 * A; cy /= 3 * A;
    const loc = pts.map(p => ({ x: p[0] - cx, y: p[1] - cy })), xs = loc.map(p => p.x);
    const slot = { x: ox + cx, y: BASE_Y + cy };
    const body = Bodies.fromVertices(slot.x, slot.y, [loc], { friction: 0.6, frictionStatic: 0.9, restitution: 0.28, density: 0.004, frictionAir: 0.002 }, true);
    Body.setStatic(body, true);
    letters.push({ ch, body, slot, cx, cy, ox, ex: (Math.min(...xs) + Math.max(...xs)) / 2, ext: Math.max(...xs) - Math.min(...xs), state: 'wall', release: 0.78 + rnd() * 0.2, loc });
    ox += g.adv * S;
  }
  const floor = Bodies.rectangle(W / 2, FLOOR + 60, 3000, 120, { isStatic: true, friction: 0.8, restitution: 0.1 });
  const wallL = Bodies.rectangle(150, 600, 40, 900, { isStatic: true }), wallR = Bodies.rectangle(1330, 600, 40, 900, { isStatic: true });
  Composite.add(engine.world, [floor, wallL, wallR, ...letters.map(l => l.body)]);

  // ---------- state ----------
  const arm = new Arm(); let T = 0, settled = false, calm = 0, phase = 'WORK', released = false;
  const Lp = (x, y, th, grip) => ({ yaw: Math.PI, r: BX - x, h: FLOOR - y, g: th || 0, grip });
  const Rp = (x, y, grip) => ({ yaw: 0, r: x - BX, h: FLOOR - y, g: 0, grip });
  const PA = 1700, PB = 1830, PH = 60, BLK = 26;
  const block = { x: PA, held: false };
  arm.set(Rp(PA, FLOOR - PH - BLK / 2 - 90, 70));

  // ---------- script runner ----------
  const mv = (p, dur, mode = 'joint') => { let m = null; return dt => { if (!m) m = arm.begin(typeof p === 'function' ? p() : p, dur, mode); return arm.advance(m, dt); }; };
  const wait = s => { let t = 0; return dt => (t += dt) >= s; };
  const until = f => () => f();
  const act = f => () => { f(); return true; };
  function grasp(L) {
    const b = L.body, th = Math.atan2(Math.sin(b.angle), Math.cos(b.angle));
    return { x: b.position.x + Math.cos(th) * L.ex, y: b.position.y + Math.sin(th) * L.ex, th };
  }
  function* taskCycle(dir) {
    const a = dir ? PB : PA, b = dir ? PA : PB, top = FLOOR - PH - BLK / 2;
    phase = 'WORK';
    yield mv(Rp(a, top - 90, 70), 0.8); yield mv(Rp(a, top, 70), 0.4, 'cart'); yield mv({ grip: BLK }, 0.3, 'cart');
    block.held = true; yield mv(Rp(a, top - 90, BLK), 0.4, 'cart'); yield mv(Rp(b, top - 90, BLK), 0.8);
    yield mv(Rp(b, top, BLK), 0.4, 'cart'); block.held = false; block.x = b; yield mv({ grip: 70 }, 0.25, 'cart'); yield mv(Rp(b, top - 90, 70), 0.35, 'cart');
  }
  function* handle(i) {
    const L = letters[i], tag = L.ch;
    phase = 'APPROACH ' + tag;
    yield mv(() => { const g = grasp(L); return Lp(g.x, g.y - 190, g.th, L.ext + 70); }, 1.5);
    phase = 'SELECT ' + tag; yield wait(0.3);
    yield mv(() => { const g = grasp(L); return Lp(g.x, g.y, g.th, L.ext + 70); }, 0.6, 'cart'); yield wait(0.12);
    phase = 'GRAB ' + tag; yield mv({ grip: L.ext }, 0.5, 'cart');
    L.state = 'held'; Body.setStatic(L.body, true); L.body.isSensor = true; yield wait(0.15);
    phase = 'LIFT ' + tag; const g0 = arm.pose(); const up = { r: g0.r, h: g0.h + 200 };
    yield mv(up, 0.7, 'cart');
    phase = 'INSPECT ' + tag; const th = arm.pose().g;
    yield mv({ g: th + 0.4 }, 0.55, 'cart'); yield wait(0.2); yield mv({ g: th - 0.35 }, 0.7, 'cart'); yield wait(0.2); yield mv({ g: th }, 0.45, 'cart');
    phase = 'RETURN ' + tag; yield mv(Lp(L.slot.x + L.ex, L.slot.y - 190, 0, L.ext), 1.9);
    phase = 'PLACE ' + tag; yield mv(Lp(L.slot.x + L.ex, L.slot.y, 0, L.ext), 0.65, 'cart'); yield wait(0.15);
    Body.setPosition(L.body, L.slot); Body.setAngle(L.body, 0); L.state = 'placed'; L.body.isSensor = false;
    yield mv({ grip: L.ext + 70 }, 0.45, 'cart'); yield mv(Lp(L.slot.x + L.ex, L.slot.y - 150, 0, L.ext + 70), 0.55, 'cart');
  }
  function* script() {
    let d = 0; while (!settled) { yield* taskCycle(d++ & 1); }
    phase = 'NOTICE'; yield wait(0.2);
    yield mv(Rp(1700, 560, 70), 0.7, 'cart'); yield wait(0.3); yield mv({ grip: 120 }, 0.3, 'cart'); yield mv({ grip: 70 }, 0.3, 'cart'); yield wait(0.35);
    phase = 'TURN'; yield mv(Lp(900, 560, 0, 80), 2.6); yield wait(0.35);
    for (let i = 0; i < letters.length; i++) yield* handle(i);
    phase = 'CHECK'; yield wait(0.3); yield mv({ g: 0.05 }, 0.25, 'cart'); yield mv({ g: -0.03 }, 0.3, 'cart'); yield mv({ g: 0 }, 0.25, 'cart'); yield wait(0.7);
    phase = 'RETURN TO WORK'; yield mv(Rp(PA, FLOOR - PH - BLK / 2 - 90, 70), 2.8);
    for (;;) yield* taskCycle(d++ & 1);
  }
  const gen = script(); let cur = null;
  function runScript(dt) {
    for (let n = 0; n < 8; n++) {
      if (!cur) { const r = gen.next(); if (r.done) return; cur = r.value; }
      if (cur(dt)) { cur = null; dt = 0; } else return;
    }
  }

  // ---------- fixed step ----------
  function step() {
    T += DT;
    for (const L of letters) {
      if (L.state !== 'wall') continue;
      if (T >= 0.5 && T < L.release) Body.setAngle(L.body, 0.014 * Math.sin((T - 0.5) * 55 + L.slot.x) * clamp((T - 0.5) * 4, 0, 1));
      else if (T >= L.release) {
        L.state = 'fall'; Body.setStatic(L.body, false);
        Body.setVelocity(L.body, { x: (rnd() - 0.5) * 1.2 * SPIN, y: 0 }); Body.setAngularVelocity(L.body, (rnd() - 0.5) * 0.07 * SPIN);
      }
    }
    Engine.update(engine, 1000 / 60);
    if (!released && letters.every(l => l.state !== 'wall')) released = true;
    if (released && !settled) {
      const mov = letters.some(l => l.state === 'fall' && (l.body.speed > 0.12 || Math.abs(l.body.angularSpeed) > 0.003));
      calm = mov ? 0 : calm + DT; if (calm > 0.7) settled = true;
    }
    runScript(DT);
    // held letters follow the gripper rigidly
    const tl = arm.tool();
    for (const L of letters) if (L.state === 'held') {
      const th = Math.atan2(-tl.a2[0], tl.a2[1]);
      Body.setAngle(L.body, th); Body.setPosition(L.body, { x: tl.G[0] - Math.cos(th) * L.ex, y: tl.G[1] - Math.sin(th) * L.ex });
    }
  }

  // ---------- rendering ----------
  const cv = document.getElementById('stage'), ctx = cv.getContext('2d');
  let bg = null, dpr = 1, running = false;
  function makeBg() {
    bg = document.createElement('canvas'); bg.width = W; bg.height = H; const c = bg.getContext('2d');
    c.fillStyle = '#07090B'; c.fillRect(0, 0, W, H);
    let g = c.createRadialGradient(W * 0.45, 380, 50, W * 0.45, 380, 1000); g.addColorStop(0, 'rgba(143,184,196,.07)'); g.addColorStop(1, 'rgba(0,0,0,0)'); c.fillStyle = g; c.fillRect(0, 0, W, FLOOR);
    g = c.createLinearGradient(0, FLOOR, 0, H); g.addColorStop(0, '#0d1418'); g.addColorStop(1, '#07090B'); c.fillStyle = g; c.fillRect(0, FLOOR, W, H - FLOOR);
    c.strokeStyle = 'rgba(207,222,234,.05)'; c.lineWidth = 1; const vx = W * 0.5, vy = FLOOR - 1100;
    for (let i = -14; i <= 14; i++) { const x = vx + i * 230; c.beginPath(); c.moveTo(vx + (x - vx) * (FLOOR - vy) / (FLOOR - vy + 1), FLOOR); c.lineTo(vx + (x - vx) * (H - vy) / (FLOOR - vy) , H); c.stroke(); }
    for (const d of [30, 72, 130, 200]) { c.beginPath(); c.moveTo(0, FLOOR + d); c.lineTo(W, FLOOR + d); c.stroke(); }
    c.strokeStyle = '#2A353C'; c.lineWidth = 1.5; c.beginPath(); c.moveTo(0, FLOOR); c.lineTo(W, FLOOR); c.stroke();
    c.strokeStyle = 'rgba(207,222,234,.22)'; c.lineWidth = 1;
    for (const L of letters) for (const sx of [L.slot.x - L.cx + 4, L.slot.x - L.cx + L.ext - 4]) { const y = BASE_Y; c.beginPath(); c.moveTo(sx - 5, y); c.lineTo(sx + 5, y); c.moveTo(sx, y - 5); c.lineTo(sx, y + 5); c.stroke(); }
    // plinths for the work cell
    for (const x of [PA, PB]) { c.fillStyle = '#131a1f'; c.fillRect(x - 46, FLOOR - PH, 92, PH); c.fillStyle = '#2A353C'; c.fillRect(x - 46, FLOOR - PH, 92, 3); }
  }
  function fit() {
    const cw = Math.min(innerWidth, innerHeight * 16 / 9), ch = cw * 9 / 16; dpr = Math.min(devicePixelRatio || 1, 2);
    cv.style.width = cw + 'px'; cv.style.height = ch + 'px'; cv.width = Math.round(cw * dpr); cv.height = Math.round(ch * dpr);
  }
  function render() {
    ctx.setTransform(cv.width / W, 0, 0, cv.height / H, 0, 0); ctx.drawImage(bg, 0, 0);
    ctx.font = `${2048 * S}px "NKX Display"`; ctx.textBaseline = 'alphabetic'; ctx.textAlign = 'left';
    for (const L of letters) {
      const b = L.body;
      if (L.state === 'fall') { const low = Math.max(...b.vertices.map(v => v.y)), a = clamp(1 - (FLOOR - low) / 260, 0, 1) * 0.55; ctx.fillStyle = `rgba(0,0,0,${a})`; ctx.beginPath(); ctx.ellipse(b.position.x, FLOOR + 5, 78, 7, 0, 0, 7); ctx.fill(); }
      if (L.state === 'held') continue;
      drawLetter(L);
    }
    // block of the idle task
    const tl = arm.tool(); const bx = block.held ? tl.G[0] : block.x, by = block.held ? tl.G[1] : FLOOR - PH - BLK / 2;
    ctx.fillStyle = '#8FB8C4'; ctx.fillRect(bx - BLK / 2, by - BLK / 2, BLK, BLK); ctx.strokeStyle = '#46626B'; ctx.strokeRect(bx - BLK / 2, by - BLK / 2, BLK, BLK);
    arm.draw(ctx, () => { for (const L of letters) if (L.state === 'held') drawLetter(L); });
    if (DEBUG) { ctx.strokeStyle = '#0ff'; ctx.lineWidth = 1; for (const L of letters) for (const p of L.body.parts.slice(1)) { ctx.beginPath(); p.vertices.forEach((v, i) => i ? ctx.lineTo(v.x, v.y) : ctx.moveTo(v.x, v.y)); ctx.closePath(); ctx.stroke(); } ctx.fillStyle = '#fff'; ctx.font = '22px monospace'; ctx.fillText(phase + '  t=' + T.toFixed(2), 24, 40); }
  }
  function drawLetter(L) {
    const b = L.body; ctx.save(); ctx.translate(b.position.x, b.position.y); ctx.rotate(b.angle); ctx.translate(-L.cx, -L.cy);
    ctx.fillStyle = '#F2F5F5'; ctx.fillText(L.ch, 0, 0); ctx.restore();
  }

  // ---------- main ----------
  let last = 0, acc = 0;
  function frame(ts) {
    const dt = Math.min(0.1, (ts - last) / 1000 || 0); last = ts; acc += dt * SPEED;
    for (let n = 0; acc >= DT && n < 8; n++) { step(); acc -= DT; }
    render(); requestAnimationFrame(frame);
  }
  window.__nkx = {
    skip(sec) { for (let i = 0; i < sec / DT; i++) step(); render(); },
    get t() { return T; }, get phase() { return phase; }, get settled() { return settled; },
    letters: () => letters.map(l => ({ ch: l.ch, state: l.state, x: +l.body.position.x.toFixed(1), y: +l.body.position.y.toFixed(1), deg: +(l.body.angle * 57.2958).toFixed(1), slot: l.slot })),
    arm
  };
  function start() {
    if (running) return; running = true; fit(); makeBg();
    addEventListener('resize', () => { fit(); render(); });
    if (reduce) { for (const L of letters) { Body.setPosition(L.body, L.slot); L.state = 'placed'; L.body.isSensor = false; } render(); return; }
    requestAnimationFrame(frame);
  }
  (document.fonts ? document.fonts.load(`${2048 * S}px "NKX Display"`, 'NUKRAX') : Promise.resolve()).then(start, start);
})();
