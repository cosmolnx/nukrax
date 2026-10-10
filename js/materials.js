/* NUKRAX prototype V4 — physically differentiated materials + studio environment.
   Differences are carried by roughness / metalness / clearcoat / texture response, not just colour.
   Palette comes from Color_System_Specification.pdf (dark theme). */
import * as THREE from 'three';

export const PAL = { page: 0x151515, recess: 0x111111, surface: 0x1a1a19, overlay: 0x20201f, control: 0x2d2d2d, hover: 0x373736, strong: 0x4d4d4c, text: 0xf0efec, text2: 0xc3c2b7, muted: 0x898781, accent: 0xabbed3 };

// ---- procedural 2x2 twill carbon fibre (colour + bump share one weave) ----
function weave() {
  const S = 512, n = 16, s = S / n, c = document.createElement('canvas'); c.width = c.height = S; const g = c.getContext('2d');
  g.fillStyle = '#060606'; g.fillRect(0, 0, S, S);
  for (let i = 0; i < n; i++) for (let j = 0; j < n; j++) {
    const horiz = ((i + j) % 4) < 2, x = i * s, y = j * s;
    const gr = horiz ? g.createLinearGradient(0, y, 0, y + s) : g.createLinearGradient(x, 0, x + s, 0);
    gr.addColorStop(0, '#0b0b0b'); gr.addColorStop(0.5, horiz ? '#3a3a3b' : '#2c2c2d'); gr.addColorStop(1, '#0b0b0b');
    g.fillStyle = gr; g.fillRect(x + 0.6, y + 0.6, s - 1.2, s - 1.2);
    g.strokeStyle = 'rgba(255,255,255,.055)'; g.lineWidth = 0.7;
    for (let k = 1; k < 6; k++) { g.beginPath(); if (horiz) { g.moveTo(x, y + k * s / 6); g.lineTo(x + s, y + k * s / 6); } else { g.moveTo(x + k * s / 6, y); g.lineTo(x + k * s / 6, y + s); } g.stroke(); }
  }
  return c;
}
let _w = null;
function weaveTex(rx, ry) { _w = _w || weave(); const t = new THREE.CanvasTexture(_w); t.wrapS = t.wrapT = THREE.RepeatWrapping; t.repeat.set(rx, ry); t.anisotropy = 8; t.colorSpace = THREE.SRGBColorSpace; return t; }
export function carbon(rx = 1 / 64, ry = rx) {
  const map = weaveTex(rx, ry), bump = weaveTex(rx, ry); bump.colorSpace = THREE.NoColorSpace;
  return new THREE.MeshPhysicalMaterial({ map, bumpMap: bump, bumpScale: 1.6, color: 0xffffff, roughness: 0.38, metalness: 0.25, clearcoat: 0.6, clearcoatRoughness: 0.2 });
}

const P = (o) => new THREE.MeshPhysicalMaterial(o);
export const M = {
  graphite: P({ color: 0x5a5a58, roughness: 0.5, metalness: 0.35, clearcoat: 0.35, clearcoatRoughness: 0.35 }),     // main body: coated industrial metal
  charcoal: P({ color: 0x353534, roughness: 0.8, metalness: 0.04 }),                                                  // matte dark polymer covers
  satin: P({ color: 0x8b8a84, roughness: 0.4, metalness: 1 }),                                                       // satin metal caps / rings
  machined: P({ color: 0xc3c2b7, roughness: 0.2, metalness: 1 }),                                                    // exposed machined metal
  anodized: P({ color: 0x6b6b69, roughness: 0.4, metalness: 0.8 }),                                                // dark anodised aluminium
  rubber: P({ color: 0x111111, roughness: 0.95, metalness: 0 }),                                                     // cables / hoses / seals
  pad: P({ color: 0x161616, roughness: 0.9, metalness: 0 }),                                                         // contact pads
  accent: P({ color: 0xabbed3, roughness: 0.5, metalness: 0.2 }),                                                    // NUKRAX accent (used sparingly)
  glass: P({ color: 0x0b0d0e, roughness: 0.04, metalness: 0, clearcoat: 1, clearcoatRoughness: 0.03, transparent: true, opacity: 0.55 }),
  ledOff: P({ color: 0x22262a, roughness: 0.4, metalness: 0 }),
  ledOn: P({ color: 0xabbed3, emissive: 0xabbed3, emissiveIntensity: 0.7, roughness: 0.4 }),
  bed: P({ color: 0x3a3a39, roughness: 0.55, metalness: 0.5 }),                                                      // machined table
};

// ---- studio environment: dark room with softbox strips (gives metals their specular character) ----
export function makeEnv(renderer) {
  const sc = new THREE.Scene(); sc.add(new THREE.Mesh(new THREE.BoxGeometry(80, 40, 80), new THREE.MeshBasicMaterial({ color: 0x0d0d0d, side: THREE.BackSide })));
  const strip = (w, h, p, col, k) => { const m = new THREE.Mesh(new THREE.PlaneGeometry(w, h), new THREE.MeshBasicMaterial({ color: new THREE.Color(col).multiplyScalar(k), side: THREE.DoubleSide })); m.position.set(...p); m.lookAt(0, 0, 0); sc.add(m); };
  strip(34, 12, [0, 18, 8], PAL.text, 7);          // overhead softbox
  strip(5, 26, [-22, 2, 6], PAL.text2, 4.5);       // left strip
  strip(4, 24, [22, 3, -8], PAL.accent, 4);        // cool rim strip, back-right
  strip(40, 3, [0, -9, 18], PAL.text2, 1.4);       // low front fill
  strip(6, 20, [14, 4, 18], PAL.text, 1.6);        // soft right-front kicker
  const pm = new THREE.PMREMGenerator(renderer); const t = pm.fromScene(sc, 0.02).texture; pm.dispose(); return t;
}
