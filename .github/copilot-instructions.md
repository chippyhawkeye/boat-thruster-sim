# Copilot Instructions for AI Agents

## Project Overview
This is a browser-based boat maneuvering simulator using p5.js. The simulation models a boat with three bidirectional thrusters, wind effects, and a dock for collision testing. The main logic is in `sketch.js`, loaded via `index.html`.

## Key Files
- `index.html`: Loads p5.js and the main simulation script.
- `sketch.js`: Contains all simulation logic, UI, and physics. No build step; edit and reload in browser.

## Architecture & Patterns
- **Single-file simulation**: All logic (physics, UI, rendering, controls) is in `sketch.js`.
- **p5.js lifecycle**: Uses `preload`, `setup`, and `draw` for initialization and animation.
- **Boat and dock**: Boat and dock are modeled as objects with physical properties. Collision uses OBB (oriented bounding box) and SAT (separating axis theorem).
- **Thruster allocation**: Joystick input is mapped to force/torque, then distributed to thrusters.
- **Digital anchor**: When joystick is neutral, a PI controller drives velocities to zero.
- **Power-limited thrusters**: Thruster force is limited by available power and a max value.
- **Drag modeling**: Uses ITTC-57 for surge, crossflow strip for sway/yaw.
- **Canvas coordinates**: +x is right, +y is down. Boat body frame: x=forward, y=starboard.

## Developer Workflow
- **No build step**: Open `index.html` in a browser to run. Edit `sketch.js` and reload.
- **Add assets**: Place images (e.g., `boat_topdown.png`) in the project root. The sprite is auto-keyed for transparency.
- **Debugging**: Use browser dev tools (console, breakpoints) for debugging.
- **No tests**: There are no automated tests or test framework.

## Project Conventions
- **Units**: Meters for physical properties, pixels for rendering (conversion via `PX_PER_M`).
- **Angles**: Degrees for UI, radians for internal math.
- **Variable naming**: Uppercase for constants/config, camelCase for variables/functions.
- **Physics**: All physics and control logic is in `sketch.js`.

## Integration Points
- **p5.js**: Loaded from CDN in `index.html`.
- **No external build tools or frameworks**: Pure JS and HTML.

## Examples
- To add a new control, define it in `sketch.js` and update the UI in the `draw` loop.
- To change boat physics, edit the relevant section in `sketch.js` (e.g., drag, thruster logic).

---

For more details, see code comments in `sketch.js`.
