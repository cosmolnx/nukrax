# NUKRAX — Robotic Arm Physics Prototype
Standalone experiment. Does not touch the production repo.

Run: `python3 -m http.server 8000` in this folder, open http://localhost:8000/
Debug: `?debug` (physics hulls + phase), `?seed=N`, `?spin=N`, `?speed=N`.
Regenerate glyph physics shapes: `python3 tools/glyphs.py` (needs fontTools).

- `js/scene.js` Matter.js rigid-body letters, floor, fixed-step sim, choreography script
- `js/robot.js` base-yaw + 3-link arm kinematics (IK), parallel-jaw gripper, canvas renderer
- `assets/fonts` real NKX Display font copied from production; `assets/glyphs.js` outlines for collision shapes
