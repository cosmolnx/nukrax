# NUKRAX — Robotic Arm Physics Prototype (V2: realism pass)
Standalone experiment; the production repo is never touched. V1 (Canvas 2D) is tagged `v1-prototype`.

**Run:** serve the folder over HTTP (ES modules need it; GitHub Pages works): `python3 -m http.server 8000` → http://localhost:8000/
**Query flags:** `?debug` (phase HUD) · `?seed=N` replay one fall (default: a fresh random fall every load) · `?tempo=1` slower/`0.5` faster · `?speed=N`

- `js/main.js` — Three.js scene, Matter.js rigid-body letters (plane-constrained), grasp planning, robot behaviour script
- `js/arm3d.js` — procedural 6-axis arm, IK (planar + 6-axis wrist decomposition), claw gripper, joint-limited motion + mass/spring tracking
- `assets/glyphs.js` / `tools/glyphs.py` — NUKRAX display-font outlines (collision shapes + extruded meshes)
- `vendor/` — three.js r170, RoomEnvironment (lighting), Matter.js, poly-decomp
- `docs/validation/` — screenshots of the whole sequence + joint motion trace
