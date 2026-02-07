/**
 * BOAT MANEUVERING SIMULATION - 3 THRUSTER DESIGN (FULL)
 * - dt-stable integration
 * - Wind UI + wind arrow
 * - Bidirectional thrusters
 * - Joystick -> [Fx,Fy,Tau] -> thruster allocation
 * - Digital Anchor toggle: when stick neutral, drives u,v,r -> 0 (PI control)
 * - Power-limited thrusters: T(V) = min(eta*P/max(V,Vmin), T_bollard_max)
 * - Geometry-informed hull drag:
 *    * Surge friction (ITTC-57) using Swet
 *    * Crossflow strip integration using A_lat for sway + yaw moment
 *
 * Conventions:
 * - Canvas/world coords: +x right, +y down, angles deg: 0=→, 90=↓
 * - Boat BODY frame: x=forward, y=starboard
 * - Thruster angle is FORCE direction in BODY frame (deg)
 */

const SIM_VERSION = "v0.1.6";
const PX_PER_M = 20;

// ---------------- DOCK ----------------
// 10' x 40' => 3.048 m x 12.192 m
const DOCK = {
  width_m: 4,
  length_m: 14,
  pos: { x: 0, y: 0 },  // px (world)
  angleDeg: 0          // 0=along +x
};

const DOCK_COLLISION = {
  // Spring/damper "bumper" parameters (tune)
  k: 2200,   // N per pixel of penetration (since our geometry is in px)
  c: 900,    // N per (m/s) damping along normal
  mu: 0.35   // tangential friction coefficient
};

function initDock() {
  // Place dock to the right-middle of the screen by default
  DOCK.pos.x = width * 0.70;
  DOCK.pos.y = height * 0.55;
}

function drawDock() {
  const L = DOCK.length_m * PX_PER_M;
  const W = DOCK.width_m * PX_PER_M;

  push();
  translate(DOCK.pos.x, DOCK.pos.y);
  rotate(radians(DOCK.angleDeg));

  // Dock body
  noStroke();
  fill(130, 110, 90);
  rectMode(CENTER);
  rect(0, 0, L, W, 6);

  // Plank lines
  stroke(0, 0, 0, 45);
  strokeWeight(1);
  for (let x = -L / 2; x <= L / 2; x += 18) {
    line(x, -W / 2, x, W / 2);
  }

  // Edge highlight
  noFill();
  stroke(255, 255, 255, 60);
  strokeWeight(2);
  rect(0, 0, L, W, 6);

  pop();
}

// ---------------- OBB / COLLISION HELPERS ----------------
function getBoatOBB() {
  // Boat OBB in WORLD pixels
  const L = BOAT_CONFIG.length_m * PX_PER_M * BOAT_SPRITE.scale;
  const B = BOAT_CONFIG.beam_m * PX_PER_M * BOAT_SPRITE.scale;
  return {
    c: createVector(boatState.pos.x, boatState.pos.y),
    hx: L / 2,
    hy: B / 2,
    angle: boatState.heading
  };
}

function getDockOBB() {
  const L = DOCK.length_m * PX_PER_M;
  const W = DOCK.width_m * PX_PER_M;
  return {
    c: createVector(DOCK.pos.x, DOCK.pos.y),
    hx: L / 2,
    hy: W / 2,
    angle: radians(DOCK.angleDeg)
  };
}

function obbAxes(obb) {
  const c = Math.cos(obb.angle);
  const s = Math.sin(obb.angle);
  // local x axis (length direction), local y axis (width direction)
  const ax = createVector(c, s);
  const ay = createVector(-s, c);
  return [ax, ay];
}

function obbCorners(obb) {
  const [ax, ay] = obbAxes(obb);
  const hx = obb.hx, hy = obb.hy;
  return [
    p5.Vector.add(obb.c, p5.Vector.add(p5.Vector.mult(ax,  hx), p5.Vector.mult(ay,  hy))),
    p5.Vector.add(obb.c, p5.Vector.add(p5.Vector.mult(ax,  hx), p5.Vector.mult(ay, -hy))),
    p5.Vector.add(obb.c, p5.Vector.add(p5.Vector.mult(ax, -hx), p5.Vector.mult(ay,  hy))),
    p5.Vector.add(obb.c, p5.Vector.add(p5.Vector.mult(ax, -hx), p5.Vector.mult(ay, -hy)))
  ];
}

function projectPointsOnAxis(points, axis) {
  let min = Infinity, max = -Infinity;
  for (const p of points) {
    const d = p.x * axis.x + p.y * axis.y;
    if (d < min) min = d;
    if (d > max) max = d;
  }
  return { min, max };
}

function intervalOverlap(a, b) {
  return Math.min(a.max, b.max) - Math.max(a.min, b.min);
}

function obbSAT(a, b) {
  // SAT between two OBBs in 2D
  const axesA = obbAxes(a);
  const axesB = obbAxes(b);
  const axes = [axesA[0], axesA[1], axesB[0], axesB[1]];

  const ptsA = obbCorners(a);
  const ptsB = obbCorners(b);

  let minPen = Infinity;
  let bestAxis = null;

  let minSep = Infinity;
  let sepAxis = null;

  const centerDelta = p5.Vector.sub(a.c, b.c);

  for (const axis0 of axes) {
    // Normalize axis
    const axis = axis0.copy();
    const mag = axis.mag();
    if (mag < 1e-9) continue;
    axis.div(mag);

    const pa = projectPointsOnAxis(ptsA, axis);
    const pb = projectPointsOnAxis(ptsB, axis);
    const o = intervalOverlap(pa, pb);

    if (o < 0) {
      const sep = -o;
      if (sep < minSep) {
        minSep = sep;
        sepAxis = axis.copy();
        // orient sep axis from dock->boat
        if (centerDelta.dot(sepAxis) < 0) sepAxis.mult(-1);
      }
    } else {
      if (o < minPen) {
        minPen = o;
        bestAxis = axis.copy();
        if (centerDelta.dot(bestAxis) < 0) bestAxis.mult(-1);
      }
    }
  }

  const collided = (minSep === Infinity); // no separating axis found
  if (collided) {
    return { collided: true, penetration: minPen, normal: bestAxis };
  }
  return { collided: false, separation: minSep, normal: sepAxis };
}

function clampMag(vec, maxMag) {
  const m = vec.mag();
  if (m > maxMag && m > 1e-9) vec.mult(maxMag / m);
  return vec;
}

// ---- Boat top-down sprite ----
// Put your boat image in the same p5.js project and name it exactly like this.
// If the image has a flat background, we'll auto-key out the corner color.
const BOAT_SPRITE_FILE = "boat_topdown.png";
let boatImg = null;
let boatImgReady = false;
const BOAT_SPRITE = {
  key_out_background: true,
  key_tolerance: 18,   // 0..255 (higher removes more)
  scale: 1.0           // tweak if needed
};

function preload() {
  // If the file isn't present, we'll fall back to the rectangle.
  try {
    boatImg = loadImage(BOAT_SPRITE_FILE, () => {
      boatImgReady = true;
      if (BOAT_SPRITE.key_out_background) {
        boatImg = keyOutBackground(boatImg, BOAT_SPRITE.key_tolerance);
      }
    }, () => {
      boatImgReady = false;
      boatImg = null;
    });
  } catch (e) {
    boatImgReady = false;
    boatImg = null;
  }
}

function keyOutBackground(img, tol) {
  // Make the corner color transparent (simple chroma-key)
  // Works well if the background is flat/near-flat.
  const out = img.get();
  out.loadPixels();

  // sample top-left corner
  const idx0 = 0;
  const r0 = out.pixels[idx0 + 0];
  const g0 = out.pixels[idx0 + 1];
  const b0 = out.pixels[idx0 + 2];

  for (let i = 0; i < out.pixels.length; i += 4) {
    const r = out.pixels[i + 0];
    const g = out.pixels[i + 1];
    const b = out.pixels[i + 2];

    // Euclidean distance in RGB space
    const dr = r - r0;
    const dg = g - g0;
    const db = b - b0;
    const d = Math.sqrt(dr*dr + dg*dg + db*db);

    if (d <= tol) {
      out.pixels[i + 3] = 0;
    }
  }

  out.updatePixels();
  return out;
}

// ---------------- CONFIG ----------------
const BOAT_CONFIG = {
  mass: 2340,            // kg
  length_m: 7.0,         // m (for drawing / inertia approx)
  beam_m: 2.5,           // m
  profile_area_m2: 6.0,  // m^2 for wind

  wind_Cd: 1.0, // (legacy; no longer used directly)

  // Wind model (anisotropic, body-frame)
  // This fixes the issue where headwind vs beam wind required the same holding power.
  wind: {
    rho: 1.225,
    Cd_front: 0.9,     // head/tail wind drag coefficient
    Cd_side: 1.2,      // beam wind drag coefficient
    A_front: 2.0,      // m^2 projected area for headwind
    A_side: 6.0,       // m^2 projected area for beam wind
    cp_x: 0.0          // m, center of pressure ahead of CG (+forward)
  },

  // Geometry-informed water model inputs (from Chip)
  hydro: {
    rho: 1025,         // kg/m^3 (seawater). Use 1000 for freshwater.
    nu: 1.19e-6,       // m^2/s kinematic viscosity (rough)
    LWL: 6.4,          // m
    Swet: 10.0,        // m^2 wetted surface area
    A_lat: 2.4,        // m^2 lateral projected underwater area
    formFactor: 1.2,   // (1+k) form factor; tune 1.1–1.3
    Cd_lat: 1.1,       // crossflow Cd; tune 1.0–1.6
    nStrips: 21        // strip count (odd recommended)
  },

  // Thrusters (power-limited)
  thrusters: [
    { name:"Bow",       x:  1.387, y:  0.000, angle:  90,
      P_cont: 3000, P_peak: 3700, eta: 0.08,
      T_forward_max: 400, T_reverse_max: 300, forwardSign: 1 },

    { name:"Port",      x: -2.555, y:  0.753, angle:  211.5,
      P_cont: 3000, P_peak: 3700, eta: 0.08,
      T_forward_max: 400, T_reverse_max: 300, forwardSign: -1 },

    { name:"Starboard", x: -2.555, y: -0.753, angle: 148.5,
      P_cont: 3000, P_peak: 3700, eta: 0.08,
      T_forward_max: 400, T_reverse_max: 300, forwardSign: -1 },
  ]
};

// Keyboard input state
const keyboardState = {
  w: false,
  a: false,
  s: false,
  d: false,
  q: false,
  e: false,
  j: false,
  k: false,
  space: false
};

// Joystick authority (desired body forces/moment)
const CONTROL_LIMITS = {
  Fx_max: 2500,  // N
  Fy_max: 2500,  // N
  Tau_max: 7000  // N*m
};

// Digital Anchor gains (PI on body velocities u,v and yaw rate r)
const ANCHOR_CTRL = {
  // Proportional: N per (m/s), and N*m per (rad/s)
  Kp_u: 4500,
  Kp_v: 6500,
  Kp_r: 9000,

  // Integral: N per (m), and N*m per (rad)
  Ki_u: 1200,
  Ki_v: 1600,
  Ki_r: 2500,

  // Anti-windup clamps on the integral terms (units: N, N, N*m)
  Iu_max: 1200,
  Iv_max: 1200,
  Ir_max: 2500,

  // Stick considered "neutral" below this magnitude
  neutral_db: 0.08
};

// ---------------- STATE ----------------
let boatState = {
  pos: { x: 0, y: 0 },   // px
  vel: { x: 0, y: 0 },   // m/s (world)
  heading: 0,            // rad
  angularVel: 0,         // rad/s
  thrusterCmd: [0, 0, 0] // -1..+1
};

// Digital anchor integrators (body frame)
let anchorInt = { iu: 0, iv: 0, ir: 0 };

// Wind state (editable)
let windSpeed = 5.0;          // m/s
let windDirectionDeg = 90;    // deg, 0=→, 90=↓ (default)    // deg, 0=→, 90=↓

// UI elements
let windSpeedSlider, windDirSlider;
let digitalAnchorToggle;
let settingsButton;
let vesselSettingsButton;
let modeText = "";

// Settings panel state
let settingsPanelOpen = false;
let thrusterInputs = [];
let vesselSettingsPanelOpen = false;
let vesselInputs = {};
let vesselSettingsTab = 'HYDRO'; // 'HYDRO' or 'WIND'

// Dock metrics for UI
let lastDockMetrics = {
  distance_m: 0,
  distance_ft: 0,
  approach_deg: 0,
  collided: false
};

function computeDockMetrics(sat) {
  // Distance estimate: use SAT separation axis (in px) as a good lower bound
  const sep_px = sat && !sat.collided ? sat.separation : 0;
  const dist_m = Math.max(0, sep_px / PX_PER_M);
  const dist_ft = dist_m * 3.28084;

  // Approach angle: boat forward axis vs dock normal
  const dockAng = radians(DOCK.angleDeg);
  const dockNormal = createVector(-Math.sin(dockAng), Math.cos(dockAng));
  const boatFwd = createVector(Math.cos(boatState.heading), Math.sin(boatState.heading));
  let a = degrees(Math.acos(clamp(Math.abs(boatFwd.dot(dockNormal)), 0, 1)));
  // a=0 means bow aimed into dock (perpendicular). a=90 means parallel to dock.

  return {
    distance_m: dist_m,
    distance_ft: dist_ft,
    approach_deg: a,
    collided: !!(sat && sat.collided)
  };
}

// Allocator cache
let ALLOC = null;

// ---------------- HELPERS ----------------
function boatYawInertia() {
  // Rectangle-ish approx about CG
  return (1 / 12) * BOAT_CONFIG.mass * (BOAT_CONFIG.length_m ** 2 + BOAT_CONFIG.beam_m ** 2);
}

function applyDeadband(x, db = 0.06) {
  if (Math.abs(x) < db) return 0;
  const s = Math.sign(x);
  return s * (Math.abs(x) - db) / (1 - db);
}

function expo(x, k = 0.35) {
  return (1 - k) * x + k * x * x * x;
}

function clamp(x, lo, hi) {
  return Math.max(lo, Math.min(hi, x));
}

function worldToBody(vx, vy, heading) {
  const c = Math.cos(heading);
  const s = Math.sin(heading);
  const u =  c * vx + s * vy;
  const v = -s * vx + c * vy;
  return { u, v };
}

function bodyToWorld(fx_body, fy_body, heading) {
  const c = Math.cos(heading);
  const s = Math.sin(heading);
  const fx = c * fx_body - s * fy_body;
  const fy = s * fx_body + c * fy_body;
  return { fx, fy };
}

function ittc57Cf(Re) {
  if (Re < 1e5) Re = 1e5;
  const logRe = Math.log10(Re);
  const denom = (logRe - 2);
  return 0.075 / (denom * denom);
}

// Compute hull water forces in BODY frame: returns {X, Y, N, Cf, Re}
function hullHydroForces(u, v, r) {
  const H = BOAT_CONFIG.hydro;
  const L = H.LWL;
  const rho = H.rho;

  // (A) Surge friction drag (ITTC-57)
  const Re = Math.abs(u) * L / H.nu;
  const Cf = ittc57Cf(Re);
  const X = -0.5 * rho * H.Swet * Cf * H.formFactor * u * Math.abs(u);

  // (B) Crossflow drag (strip integration) for sway + yaw
  const n = H.nStrips;
  const dx = L / (n - 1);
  const a_per_m = H.A_lat / L;
  const Cd = H.Cd_lat;

  let Y = 0;
  let N = 0;

  for (let i = 0; i < n; i++) {
    const x = -L/2 + i * dx;
    const v_loc = v + r * x;

    const dA = a_per_m * dx;
    const dY = -0.5 * rho * Cd * dA * v_loc * Math.abs(v_loc);

    Y += dY;
    N += x * dY;
  }

  return { X, Y, N, Cf, Re };
}

// ---------------- THRUSTER THRUST MODEL ----------------
function thrustLimitForThruster(t, speed_mps, usePeak = false) {
  const P = usePeak ? t.P_peak : t.P_cont; // W
  const Vmin = 0.6;                        // m/s slip floor
  
  // Power-limited thrust for forward (using base eta)
  const T_power_fwd = (t.eta * P) / Math.max(speed_mps, Vmin);
  
  // Power-limited thrust for reverse (assume efficiency scales with max thrust ratio)
  // eta_rev = eta * (T_rev_max / T_fwd_max)
  const ratio = t.T_reverse_max / t.T_forward_max;
  const T_power_rev = (T_power_fwd * ratio);

  return { T_power_fwd, T_power_rev, T_forward_max: t.T_forward_max, T_reverse_max: t.T_reverse_max, forwardSign: t.forwardSign };
}

// ---------------- CONTROL ALLOCATION ----------------
function buildAllocator() {
  const T = BOAT_CONFIG.thrusters;

  let A = [
    [0, 0, 0],
    [0, 0, 0],
    [0, 0, 0]
  ];

  for (let i = 0; i < 3; i++) {
    const th = radians(T[i].angle);
    const c = Math.cos(th);
    const s = Math.sin(th);
    const rx = T[i].x;
    const ry = T[i].y;

    A[0][i] = c;
    A[1][i] = s;
    A[2][i] = (rx * s - ry * c);
  }

  const Ainv = inv3x3(A);
  if (!Ainv) console.warn("Allocator matrix singular. Check thruster geometry/angles.");
  ALLOC = { A, Ainv };
}

function inv3x3(m) {
  const a=m[0][0], b=m[0][1], c=m[0][2];
  const d=m[1][0], e=m[1][1], f=m[1][2];
  const g=m[2][0], h=m[2][1], i=m[2][2];

  const A =  (e*i - f*h);
  const B = -(d*i - f*g);
  const C =  (d*h - e*g);
  const D = -(b*i - c*h);
  const E =  (a*i - c*g);
  const F = -(a*h - b*g);
  const G =  (b*f - c*e);
  const H = -(a*f - c*d);
  const I =  (a*e - b*d);

  const det = a*A + b*B + c*C;
  if (Math.abs(det) < 1e-9) return null;

  const invDet = 1.0 / det;
  return [
    [A*invDet, D*invDet, G*invDet],
    [B*invDet, E*invDet, H*invDet],
    [C*invDet, F*invDet, I*invDet]
  ];
}

function mul3x3_vec3(m, v) {
  return [
    m[0][0]*v[0] + m[0][1]*v[1] + m[0][2]*v[2],
    m[1][0]*v[0] + m[1][1]*v[1] + m[1][2]*v[2],
    m[2][0]*v[0] + m[2][1]*v[1] + m[2][2]*v[2],
  ];
}

function allocateThrusters(Fx_cmd, Fy_cmd, Tau_cmd) {
  if (!ALLOC || !ALLOC.Ainv || !boatState) return [0, 0, 0];

  const Fi = mul3x3_vec3(ALLOC.Ainv, [Fx_cmd, Fy_cmd, Tau_cmd]);

  const speed = Math.hypot(boatState.vel.x, boatState.vel.y);
  let cmd = Fi.map((f, idx) => {
    const t = BOAT_CONFIG.thrusters[idx];
    const thrustInfo = thrustLimitForThruster(t, speed, false);
    
    // Determine max thrust available in the requested direction
    // If f and forwardSign have same sign -> Forward mode (efficient)
    // If f and forwardSign have diff sign -> Reverse mode (inefficient)
    const isForward = (f * t.forwardSign) >= 0;
    
    const limit_struct = isForward ? thrustInfo.T_forward_max : thrustInfo.T_reverse_max;
    const limit_power = isForward ? thrustInfo.T_power_fwd : thrustInfo.T_power_rev;
    
    // Available thrust is min of power limit and structural limit
    const available = Math.min(limit_struct, limit_power);
    
    return f / Math.max(available, 1e-6);
  });

  const maxAbs = Math.max(...cmd.map(c => Math.abs(c)));
  if (maxAbs > 1) cmd = cmd.map(c => c / maxAbs);

  return cmd.map(c => clamp(c, -1, 1));
}

// ---------------- P5 INPUT HANDLERS ----------------
function keyPressed() {
  const k = key.toLowerCase();
  if (k === 'w') keyboardState.w = true;
  if (k === 'a') keyboardState.a = true;
  if (k === 's') keyboardState.s = true;
  if (k === 'd') keyboardState.d = true;
  if (k === 'j') keyboardState.j = true;
  if (k === 'k') keyboardState.k = true;
  if (k === 'q') keyboardState.q = true; // Added Q/E for sway (left/right) if needed, though A/D is mapping to fy currently? 
  // Wait, readKeyboard maps A/D to fy (sway) and W/S to fx (surge). 
  // The help text says:
  // '  W/S: Forward/Reverse thrust',
  // '  A/D: Rotate left/right (yaw)',
  // '  Q/E: Sway left/right (sideways thrust)',
  
  // Let's check readKeyboard again.
  // 589:  if (keyboardState.d) fy += 1;
  // 590:  if (keyboardState.a) fy -= 1;
  // This maps A/D to Sway (fy). 
  // Usual boat controls: A/D is Yaw. Q/E is Sway (or vice versa).
  
  if (k === 'e') keyboardState.e = true;
  if (k === 'q') keyboardState.q = true;

  if (key === ' ') keyboardState.space = true;
  // Prevent default scrolling for game keys
  if ([' ', 'ArrowUp', 'ArrowDown', 'ArrowLeft', 'ArrowRight'].includes(key)) {
    return false;
  }
}

function keyReleased() {
  const k = key.toLowerCase();
  if (k === 'w') keyboardState.w = false;
  if (k === 'a') keyboardState.a = false;
  if (k === 's') keyboardState.s = false;
  if (k === 'd') keyboardState.d = false;
  if (k === 'j') keyboardState.j = false;
  if (k === 'k') keyboardState.k = false;
  if (k === 'q') keyboardState.q = false;
  if (k === 'e') keyboardState.e = false;
  if (key === ' ') keyboardState.space = false;
}

// ---------------- KEYBOARD INPUT ----------------
function readKeyboard() {
  let fx = 0, fy = 0, yaw = 0;
  
  if (keyboardState.w) fx += 1;
  if (keyboardState.s) fx -= 1;

  // Sway (Sideways)
  if (keyboardState.e) fy += 1;
  if (keyboardState.q) fy -= 1;
  
  // Yaw (Rotation)
  if (keyboardState.d) yaw += 1;
  if (keyboardState.a) yaw -= 1;
  if (keyboardState.k) yaw += 1;
  if (keyboardState.j) yaw -= 1;
  
  return { fx, fy, yaw };
}

// Merge keyboard, touch, and gamepad inputs (priority: Touch > Keyboard > Gamepad)
function readCombinedInput() {
  const touchInput = readTouch();
  if (touchInput.active) {
    // Touch overrides everything if active
    return { ...touchInput, connected: true, axesCount: 3, source: 'touch', anchor: false };
  }

  const kbInput = readKeyboard();
  const hasKeyboardInput = kbInput.fx !== 0 || kbInput.fy !== 0 || kbInput.yaw !== 0 || keyboardState.space;
  if (hasKeyboardInput) {
    return { ...kbInput, connected: true, axesCount: 3, source: 'keyboard', anchor: keyboardState.space };
  }
  return { ...readJoystick(), source: 'gamepad' };
}

// ---------------- JOYSTICK (GAMEPAD) ----------------
// NOTE (per Chip): yaw axis is 5 (index 5)
function readJoystick() {
  const pads = navigator.getGamepads ? navigator.getGamepads() : [];
  const gp = pads && pads[0] ? pads[0] : null;
  if (!gp) return { fx:0, fy:0, yaw:0, connected:false, axesCount:0 };

  let lr = gp.axes[0] ?? 0;
  let ud = gp.axes[1] ?? 0;
  ud = -ud; // up=+forward

  let yaw = 0;
  if (gp.axes.length > 5) yaw = gp.axes[5] ?? 0;
  else if (gp.axes.length > 4) yaw = gp.axes[4] ?? 0;
  else if (gp.axes.length > 2) yaw = gp.axes[2] ?? 0;
  else if (gp.axes.length > 3) yaw = gp.axes[3] ?? 0;

  lr = expo(applyDeadband(lr), 0.35);
  ud = expo(applyDeadband(ud), 0.35);
  yaw = expo(applyDeadband(yaw), 0.35);

  return { fx:ud, fy:lr, yaw:yaw, connected:true, axesCount: gp.axes.length };
}

function stickIsNeutral(joy) {
  const m = Math.max(Math.abs(joy.fx), Math.abs(joy.fy), Math.abs(joy.yaw));
  return m < ANCHOR_CTRL.neutral_db;
}

function resetAnchorIntegrators() {
  anchorInt.iu = 0;
  anchorInt.iv = 0;
  anchorInt.ir = 0;
}

// ---------------- DIGITAL ANCHOR CONTROL ----------------
function digitalAnchorController(dt) {
  const bodyVel = worldToBody(boatState.vel.x, boatState.vel.y, boatState.heading);
  const u = bodyVel.u;
  const v = bodyVel.v;
  const r = boatState.angularVel;

  const eu = -u;
  const ev = -v;
  const er = -r;

  anchorInt.iu = clamp(anchorInt.iu + eu * dt * ANCHOR_CTRL.Ki_u, -ANCHOR_CTRL.Iu_max, ANCHOR_CTRL.Iu_max);
  anchorInt.iv = clamp(anchorInt.iv + ev * dt * ANCHOR_CTRL.Ki_v, -ANCHOR_CTRL.Iv_max, ANCHOR_CTRL.Iv_max);
  anchorInt.ir = clamp(anchorInt.ir + er * dt * ANCHOR_CTRL.Ki_r, -ANCHOR_CTRL.Ir_max, ANCHOR_CTRL.Ir_max);

  let Fx_cmd = ANCHOR_CTRL.Kp_u * eu + anchorInt.iu;
  let Fy_cmd = ANCHOR_CTRL.Kp_v * ev + anchorInt.iv;
  let Tau_cmd = ANCHOR_CTRL.Kp_r * er + anchorInt.ir;

  Fx_cmd = clamp(Fx_cmd, -CONTROL_LIMITS.Fx_max, CONTROL_LIMITS.Fx_max);
  Fy_cmd = clamp(Fy_cmd, -CONTROL_LIMITS.Fy_max, CONTROL_LIMITS.Fy_max);
  Tau_cmd = clamp(Tau_cmd, -CONTROL_LIMITS.Tau_max, CONTROL_LIMITS.Tau_max);

  return { Fx_cmd, Fy_cmd, Tau_cmd, u, v, r };
}

// ---------------- UI / P5 ----------------
function setup() {
  createCanvas(windowWidth, windowHeight);
  initDock();
  boatState.pos = { x: width / 2, y: height / 2 };
  buildAllocator();

  // ---- Wind UI (top-right panel) ----
  windSpeedSlider = createSlider(0, 20, windSpeed, 0.1);
  windDirSlider = createSlider(0, 359, windDirectionDeg, 1);

  // We'll position these in a helper so resize also works
  updateInterfaceLayout();

  // Digital anchor toggle
  digitalAnchorToggle = createCheckbox("Digital Anchor (hold u,v,r = 0 when stick neutral)", false);
  digitalAnchorToggle.style("color", "#fff");

  // Settings button
  settingsButton = createButton("⚙️ Thruster Settings");
  settingsButton.mousePressed(toggleSettingsPanel);
  settingsButton.style("padding", "8px 16px");
  settingsButton.style("font-size", "14px");
  settingsButton.style("cursor", "pointer");

  // Vessel settings button
  vesselSettingsButton = createButton("🚤 Vessel Settings");
  vesselSettingsButton.mousePressed(toggleVesselSettingsPanel);
  vesselSettingsButton.style("padding", "8px 16px");
  vesselSettingsButton.style("font-size", "14px");
  vesselSettingsButton.style("cursor", "pointer");
  
  // Call layout again once buttons exist
  updateInterfaceLayout();

  // Create thruster input fields (initially hidden)
  createThrusterInputs();
  createVesselInputs();
}

function updateInterfaceLayout() {
  if (!windSpeedSlider) return; // Not ready
  
  const isNarrow = width < 720;
  
  if (isNarrow) {
     // --- MOBILE LAYOUT ---
     
     // 1. Wind Panel (Moves to top-left, stacked under status panel)
     // Status panel (drawn in drawUI) is approx 200px tall. We'll simplify it in drawUI later.
     // Let's assume on Mobile we want the wind sliders somewhat accessible.
     // Status Panel: y=10. h=120 (reduced height).
     // Wind Panel: y=140.
     
     const wx = 20;
     const wy = 140; // under status
     
     windSpeedSlider.position(wx + 20, wy + 28);
     windSpeedSlider.style("width", Math.min(220, width - 60) + "px");
     
     windDirSlider.position(wx + 20, wy + 62);
     windDirSlider.style("width", Math.min(220, width - 60) + "px");
     
     // 2. Buttons (Stacked below Wind)
     // Wind panel height approx 90px (compact).
     let by = wy + 100;
     
     if (digitalAnchorToggle) {
        digitalAnchorToggle.position(wx, by);
        // Make tap area larger
        digitalAnchorToggle.style("transform", "scale(1.2)");
        digitalAnchorToggle.style("transform-origin", "left center");
     }
     by += 40;
     
     if (settingsButton) {
        settingsButton.position(wx, by);
     }
     by += 45;
     
     if (vesselSettingsButton) {
        vesselSettingsButton.position(wx, by);
     }
     
  } else {
     // --- DESKTOP LAYOUT ---
     
     // Wind Top Right
     const panelW = 260;
     const x0 = Math.max(20, width - panelW - 20);
     const y0 = 20;
     
     windSpeedSlider.position(x0 + 20, y0 + 108);
     windSpeedSlider.style("width", "220px");

     windDirSlider.position(x0 + 20, y0 + 142);
     windDirSlider.style("width", "220px");
     
     // Buttons Top Left (below status panel which is ~178h)
     const btnX = 20;
     const btnY = 215;
     
     if (digitalAnchorToggle) {
        digitalAnchorToggle.position(btnX, btnY);
        digitalAnchorToggle.style("transform", "scale(1.0)");
     }
     if (settingsButton) settingsButton.position(btnX, 250);
     if (vesselSettingsButton) vesselSettingsButton.position(btnX, 290);
  }
}

// Deprecated old function, kept just in case but we use updateInterfaceLayout now
function positionWindUI() { 
  updateInterfaceLayout();
}

let prevAnchorEnabled = false;

function draw() {

  background(20, 40, 60);

  // --- Keyboard Controls Help ---
  drawKeyboardHelp();
// Draws keyboard control instructions on the screen
function drawKeyboardHelp() {
  const lines = [
    'Keyboard Controls:',
    '  W/S: Forward/Reverse thrust',
    '  A/D: Rotate left/right (yaw)',
    '  Q/E: Sway left/right (sideways thrust)',
    '  SPACE: Digital anchor (hold to stop)',
    '',
    'Tip: Combine keys for complex maneuvers.'
  ];
  const lh = 18;
  const blockHeight = lines.length * lh;
  // Place block vertically centered, left side, but below top info panel (offset ~180px)
  const y0 = Math.max(180, (height - blockHeight) / 2);
  const x = 24;
  push();
  textAlign(LEFT, TOP);
  textSize(15);
  fill(255, 255, 220, 210);
  noStroke();
  for (let i = 0; i < lines.length; ++i) {
    text(lines[i], x, y0 + i * lh);
  }
  pop();
}

  // Draw dock first so boat overlays it
  drawDock();
  const dt = clamp(deltaTime / 1000, 0, 0.05);

  // Keep DOM controls pinned (some embeds don't reliably keep positions)
  positionWindUI();

  windSpeed = windSpeedSlider.value();
  windDirectionDeg = windDirSlider.value();

  const joy = readCombinedInput();
  // Keyboard: SPACE bar acts as digital anchor
  const anchorEnabled = joy.anchor !== undefined ? joy.anchor : digitalAnchorToggle.checked();

  if (anchorEnabled !== prevAnchorEnabled) resetAnchorIntegrators();
  prevAnchorEnabled = anchorEnabled;

  if (joy.connected) {
    const neutral = stickIsNeutral(joy);

    if (anchorEnabled && neutral) {
      modeText = "Mode: Digital Anchor (PI velocity hold) — stick neutral";
      const ctrl = digitalAnchorController(dt);
      boatState.thrusterCmd = allocateThrusters(ctrl.Fx_cmd, ctrl.Fy_cmd, ctrl.Tau_cmd);
    } else {
      if (anchorEnabled) resetAnchorIntegrators();

      modeText = anchorEnabled
        ? "Mode: Joystick (manual) — anchor armed"
        : "Mode: Joystick (manual)";

      const Fx_cmd  = joy.fx  * CONTROL_LIMITS.Fx_max;
      const Fy_cmd  = joy.fy  * CONTROL_LIMITS.Fy_max;
      const Tau_cmd = joy.yaw * CONTROL_LIMITS.Tau_max;
      boatState.thrusterCmd = allocateThrusters(Fx_cmd, Fy_cmd, Tau_cmd);
    }
  } else {
    modeText = anchorEnabled
      ? "Mode: Manual thruster control disabled (no gamepad) — anchor idle"
      : "Mode: Manual thruster control disabled (no gamepad)";
    
    boatState.thrusterCmd = [0, 0, 0];
  }

  const debug = updatePhysics(dt);

  drawWindArrow();
  drawBoat();
  drawUI(debug, joy);
  drawWindPanelLabel(); // Helper to draw just the non-DOM parts (background/text)
  
  // Draw settings panel if open
  if (settingsPanelOpen) {
    drawSettingsPanel();
  }
  
  // Draw vessel settings panel if open
  if (vesselSettingsPanelOpen) {
    drawVesselSettingsPanel();
  }
}

// ---------------- PHYSICS ----------------
function updatePhysics(dt) {
  let netFx = 0, netFy = 0;
  let netTorque = 0;

  const speed = Math.hypot(boatState.vel.x, boatState.vel.y);

  // (1) Thrusters
  BOAT_CONFIG.thrusters.forEach((t, i) => {
    const cmd = clamp(boatState.thrusterCmd[i], -1, 1);
    const thrustInfo = thrustLimitForThruster(t, speed, false);
    // Determine if command is forward or reverse
    const isForward = (cmd * t.forwardSign) > 0;
    
    const Tmax_struct = isForward ? thrustInfo.T_forward_max : thrustInfo.T_reverse_max;
    const Tmax_power = isForward ? thrustInfo.T_power_fwd : thrustInfo.T_power_rev;
    
    // Effective max thrust is min of structural and power limit
    const Tmax = Math.min(Tmax_struct, Tmax_power);
    
    const F = Tmax * Math.abs(cmd); // apply magnitude scaling

    // Direction depends on command sign relative to forwardSign
    // if cmd > 0 and forwardSign=1 -> Fwd (+ thrust).  
    // if cmd < 0 and forwardSign=1 -> Rev (- thrust).
    // so raw singed thrust F_signed = F * sign(cmd) * forwardSign? No.
    // Thruster angle is defined for Positive Thrust. 
    // If cmd is positive, we thrust in direction 'angle'.
    // If cmd is negative, we thrust in direction 'angle + 180'.
    
    // Let's just use cmd as the sign flipper.
    // F_scalar = F_mag * sign(cmd)
    
    const F_signed = F * Math.sign(cmd); 

    const worldAngle = boatState.heading + radians(t.angle);
    const fx = Math.cos(worldAngle) * F_signed;
    const fy = Math.sin(worldAngle) * F_signed;

    netFx += fx; netFy += fy;

    // torque cross in body frame
    const c = Math.cos(-boatState.heading);
    const s = Math.sin(-boatState.heading);
    const fbx = c * fx - s * fy;
    const fby = s * fx + c * fy;
    netTorque += (t.x * fby - t.y * fbx);
  });

  // (2) Wind (quadratic, anisotropic in body frame)
  // Compute relative wind in WORLD
  const windAngle = radians(windDirectionDeg);
  const windVx = Math.cos(windAngle) * windSpeed;
  const windVy = Math.sin(windAngle) * windSpeed;

  const relVx = windVx - boatState.vel.x;
  const relVy = windVy - boatState.vel.y;

  // Convert relative wind to BODY frame
  const relBody = worldToBody(relVx, relVy, boatState.heading);
  const u_w = relBody.u; // +forward component
  const v_w = relBody.v; // +starboard component

  const W = BOAT_CONFIG.wind;
  const rhoAir = W.rho;

  // Force on hull points WITH relative wind (downwind)
  const Xw =  0.5 * rhoAir * W.Cd_front * W.A_front * u_w * Math.abs(u_w);
  const Yw =  0.5 * rhoAir * W.Cd_side  * W.A_side  * v_w * Math.abs(v_w);

  const FwindWorld = bodyToWorld(Xw, Yw, boatState.heading);
  netFx += FwindWorld.fx;
  netFy += FwindWorld.fy;

  // Wind yawing moment (side-force acting at CP ahead of CG)
  netTorque += W.cp_x * Yw;

  // (2.5) Dock bumper collision
  const boatOBB = getBoatOBB();
  const dockOBB = getDockOBB();
  const sat = obbSAT(boatOBB, dockOBB);

  // Save for UI
  lastDockMetrics = computeDockMetrics(sat);

  if (sat.collided && sat.normal) {
    // Spring force is proportional to penetration (px) along normal
    const n = sat.normal.copy();

    // Convert penetration px to force: k*(px)
    let Fn_mag = DOCK_COLLISION.k * sat.penetration;

    // Damping along normal (m/s)
    const v = createVector(boatState.vel.x, boatState.vel.y);
    const vn = v.dot(n);
    Fn_mag += -DOCK_COLLISION.c * vn;

    // Don't let damping reverse the bumper force
    Fn_mag = Math.max(0, Fn_mag);

    const Fn = p5.Vector.mult(n, Fn_mag);

    // Tangential (friction) force: oppose tangential velocity, limited by mu*Fn
    const t = createVector(-n.y, n.x);
    const vt = v.dot(t);
    let Ft = p5.Vector.mult(t, -vt * DOCK_COLLISION.c * 0.35);
    Ft = clampMag(Ft, DOCK_COLLISION.mu * Fn_mag);

    const Fdock = p5.Vector.add(Fn, Ft);

    netFx += Fdock.x;
    netFy += Fdock.y;

    // Approximate contact point at boat center shifted back along normal
    const contact = p5.Vector.sub(boatOBB.c, p5.Vector.mult(n, boatOBB.hx * 0.2));
    const r = p5.Vector.sub(contact, boatOBB.c);
    // Torque (2D): r x F = r.x*F.y - r.y*F.x
    netTorque += (r.x * Fdock.y - r.y * Fdock.x);
  }

  // (3) Water / Hull hydro / Hull hydro
  const bodyVel = worldToBody(boatState.vel.x, boatState.vel.y, boatState.heading);
  const u = bodyVel.u;
  const v = bodyVel.v;
  const r = boatState.angularVel;

  const hydro = hullHydroForces(u, v, r);
  const Fw = bodyToWorld(hydro.X, hydro.Y, boatState.heading);
  netFx += Fw.fx;
  netFy += Fw.fy;
  netTorque += hydro.N;

  // integrate translation
  boatState.vel.x += (netFx / BOAT_CONFIG.mass) * dt;
  boatState.vel.y += (netFy / BOAT_CONFIG.mass) * dt;

  boatState.pos.x += boatState.vel.x * PX_PER_M * dt;
  boatState.pos.y += boatState.vel.y * PX_PER_M * dt;

  // integrate rotation
  const Izz = boatYawInertia();
  boatState.angularVel += (netTorque / Izz) * dt;
  boatState.heading = (boatState.heading + boatState.angularVel * dt + TWO_PI) % TWO_PI;

  // wrap screen
  if (boatState.pos.x < -100) boatState.pos.x = width + 100;
  if (boatState.pos.x > width + 100) boatState.pos.x = -100;
  if (boatState.pos.y < -100) boatState.pos.y = height + 100;
  if (boatState.pos.y > height + 100) boatState.pos.y = -100;

  const netF = Math.hypot(netFx, netFy);
  const TmaxList = BOAT_CONFIG.thrusters.map(t => thrustLimitForThruster(t, speed, false));
  return { netFx, netFy, netF, netTorque, TmaxList, hydroCf: hydro.Cf, hydroRe: hydro.Re };
}

// ---------------- DRAWING ----------------
function drawBoat() {
  push();
  translate(boatState.pos.x, boatState.pos.y);
  rotate(boatState.heading);

  const Lpx = BOAT_CONFIG.length_m * PX_PER_M * BOAT_SPRITE.scale;

  // --- Hull (sprite if available; else pretty procedural boat) ---
  if (boatImg && boatImgReady) {
    const aspect = boatImg.height / Math.max(1, boatImg.width);
    const Wpx = Lpx;
    const Hpx = Wpx * aspect;
    imageMode(CENTER);
    image(boatImg, 0, 0, Wpx, Hpx);
  } else {
    drawPrettyBoatTopDown(Lpx, BOAT_CONFIG.beam_m * PX_PER_M);
  }

  // --- Thruster diagram overlay ---
  BOAT_CONFIG.thrusters.forEach((t, i) => {
    const px = t.x * PX_PER_M;
    const py = t.y * PX_PER_M;

    noStroke();
    fill(255, 80, 80);
    circle(px, py, 10);

    const cmd = boatState.thrusterCmd[i];
    if (Math.abs(cmd) > 1e-3) {
      const dir = radians(t.angle);
      const len = 55 * Math.abs(cmd);

      stroke(cmd > 0 ? color(255, 220, 80) : color(80, 200, 255));
      strokeWeight(3);

      // Reverse the direction: arrow points opposite to force
      const sgn = cmd >= 0 ? -1 : 1;
      const x2 = px + Math.cos(dir) * len * sgn;
      const y2 = py + Math.sin(dir) * len * sgn;
      line(px, py, x2, y2);

      const ah = 8;
      const ang = Math.atan2(y2 - py, x2 - px);
      line(x2, y2, x2 - ah * Math.cos(ang - 0.5), y2 - ah * Math.sin(ang - 0.5));
      line(x2, y2, x2 - ah * Math.cos(ang + 0.5), y2 - ah * Math.sin(ang + 0.5));
      strokeWeight(1);
    }
  });

  pop();
}

function drawPrettyBoatTopDown(Lpx, Bpx) {
  // A stylized center-console / bowrider-ish top view.
  // Coordinate system: +x forward (bow), +y starboard.

  const L = Lpx;
  const B = Math.max(40, Bpx);

  // Proportions
  const bowSharp = 0.55;          // how pointy the bow is
  const chineInset = 0.10 * B;    // hull inset for interior
  const deckPadInset = 0.18 * B;

  // Hull outline
  noStroke();
  fill(235);
  beginShape();
  // Bow tip
  vertex(L * 0.52, 0);
  // Starboard sheer to stern
  bezierVertex(L * 0.44,  B * bowSharp,
               L * 0.20,  B * 0.58,
              -L * 0.46,  B * 0.48);
  // Stern corner starboard
  bezierVertex(-L * 0.52,  B * 0.47,
               -L * 0.56,  B * 0.32,
               -L * 0.56,  B * 0.18);
  // Transom
  bezierVertex(-L * 0.56,  B * 0.06,
               -L * 0.56, -B * 0.06,
               -L * 0.56, -B * 0.18);
  // Stern corner port
  bezierVertex(-L * 0.56, -B * 0.32,
               -L * 0.52, -B * 0.47,
              -L * 0.46, -B * 0.48);
  // Port sheer back to bow
  bezierVertex(L * 0.20, -B * 0.58,
               L * 0.44, -B * bowSharp,
               L * 0.52, 0);
  endShape(CLOSE);

  // Rubrail accent
  stroke(30, 30, 30, 70);
  strokeWeight(2);
  noFill();
  beginShape();
  vertex(L * 0.50, 0);
  bezierVertex(L * 0.42,  B * bowSharp,
               L * 0.18,  B * 0.55,
              -L * 0.47,  B * 0.45);
  bezierVertex(-L * 0.55,  B * 0.40,
               -L * 0.58,  B * 0.22,
               -L * 0.58,  B * 0.12);
  endShape();
  beginShape();
  vertex(L * 0.50, 0);
  bezierVertex(L * 0.42, -B * bowSharp,
               L * 0.18, -B * 0.55,
              -L * 0.47, -B * 0.45);
  bezierVertex(-L * 0.55, -B * 0.40,
               -L * 0.58, -B * 0.22,
               -L * 0.58, -B * 0.12);
  endShape();
  noStroke();

  // Inner cockpit/deck cutout
  fill(210);
  beginShape();
  vertex(L * 0.44, 0);
  bezierVertex(L * 0.36,  B * (bowSharp - 0.10),
               L * 0.16,  B * 0.48,
              -L * 0.40,  B * 0.40);
  bezierVertex(-L * 0.48,  B * 0.35,
               -L * 0.50,  B * 0.20,
               -L * 0.50,  B * 0.12);
  bezierVertex(-L * 0.50,  B * 0.04,
               -L * 0.50, -B * 0.04,
               -L * 0.50, -B * 0.12);
  bezierVertex(-L * 0.50, -B * 0.20,
               -L * 0.48, -B * 0.35,
              -L * 0.40, -B * 0.40);
  bezierVertex(L * 0.16, -B * 0.48,
               L * 0.36, -B * (bowSharp - 0.10),
               L * 0.44, 0);
  endShape(CLOSE);

  // Bow seating pad
  fill(70, 70, 70, 210);
  const bowX = L * 0.28;
  const bowW = L * 0.28;
  const bowH = B * 0.62;
  rectMode(CENTER);
  rect(bowX, 0, bowW, bowH, 22);

  // Center console
  fill(235);
  const cx = -L * 0.05;
  const cw = L * 0.22;
  const ch = B * 0.36;
  rect(cx, 0, cw, ch, 18);

  // Console windshield
  fill(30, 30, 30, 170);
  rect(cx + cw * 0.10, 0, cw * 0.55, ch * 0.55, 14);

  // T-top / roof
  fill(50, 50, 50, 220);
  const tx = cx - cw * 0.05;
  rect(tx, 0, cw * 1.10, ch * 0.80, 22);

  // T-top legs
  stroke(40, 40, 40, 180);
  strokeWeight(6);
  line(cx - cw * 0.25, -ch * 0.35, cx - cw * 0.40, -ch * 0.10);
  line(cx - cw * 0.25,  ch * 0.35, cx - cw * 0.40,  ch * 0.10);
  line(cx + cw * 0.25, -ch * 0.35, cx + cw * 0.40, -ch * 0.10);
  line(cx + cw * 0.25,  ch * 0.35, cx + cw * 0.40,  ch * 0.10);
  noStroke();

  // Stern lounge / bench
  fill(70, 70, 70, 220);
  rect(-L * 0.42, 0, L * 0.22, B * 0.55, 22);

  // Simple centerline
  stroke(0, 0, 0, 35);
  strokeWeight(2);
  line(-L * 0.50, 0, L * 0.48, 0);
  noStroke();
}

function drawWindArrow() {
  // Wind panel (top-right)
  const panelW = 260;
  const panelH = 170;
  const x0 = Math.max(20, width - panelW - 20);
  const y0 = 20;

  // Panel background
  noStroke();
  fill(0, 0, 0, 80);
  rect(x0, y0, panelW, panelH, 10);

  // Big arrow in panel
  const cx = x0 + 70;
  const cy = y0 + 75;
  const ang = radians(windDirectionDeg);
  const len = map(windSpeed, 0, 20, 10, 65);

  const x2 = cx + Math.cos(ang) * len;
  const y2 = cy + Math.sin(ang) * len;

  stroke(220);
  strokeWeight(6);
  line(cx, cy, x2, y2);

  // Arrowhead
  const ah = 14;
  const a = Math.atan2(y2 - cy, x2 - cx);
  line(x2, y2, x2 - ah * Math.cos(a - 0.55), y2 - ah * Math.sin(a - 0.55));
  line(x2, y2, x2 - ah * Math.cos(a + 0.55), y2 - ah * Math.sin(a + 0.55));
  strokeWeight(1);

  // Labels (kept above sliders)
  noStroke();
  fill(255);
  textSize(13);
  text("Wind", x0 + 140, y0 + 24);
  text(`Speed: ${(windSpeed * 1.94384).toFixed(1)} kt`, x0 + 140, y0 + 44);
  text(`Dir: ${windDirectionDeg.toFixed(0)}°`, x0 + 140, y0 + 64);

  // Decorative "field" arrows around the window edges
  drawWindFieldArrows(ang);

  // Also show a subtle center wind indicator
  drawWindCenterArrow(ang);
}

function drawWindCenterArrow(ang) {
  // Middle-of-screen wind arrow so direction is obvious while driving
  const cx = width * 0.5;
  const cy = height * 0.5;
  const len = map(windSpeed, 0, 20, 20, 90);

  // faint ring
  noFill();
  stroke(200, 230, 255, 90);
  strokeWeight(2);
  circle(cx, cy, 70);

  // arrow
  const x2 = cx + Math.cos(ang) * len;
  const y2 = cy + Math.sin(ang) * len;
  stroke(200, 230, 255, 150);
  strokeWeight(5);
  line(cx, cy, x2, y2);

  const ah = 16;
  const a = Math.atan2(y2 - cy, x2 - cx);
  line(x2, y2, x2 - ah * Math.cos(a - 0.55), y2 - ah * Math.sin(a - 0.55));
  line(x2, y2, x2 - ah * Math.cos(a + 0.55), y2 - ah * Math.sin(a + 0.55));

  // small label
  noStroke();
  fill(255, 255, 255, 140);
  textSize(12);
  text("wind", cx - 14, cy - 45);
}

function drawWindFieldArrows(ang) {
  // Small arrows along the border + an interior grid so direction is obvious everywhere
  const n = 7;                // same frequency as before
  const margin = 14;
  const len = 18;

  stroke(200, 230, 255, 170);
  strokeWeight(2);

  // top edge
  for (let i = 0; i < n; i++) {
    const x = map(i, 0, n - 1, margin, width - margin);
    const y = margin;
    drawArrow(x, y, ang, len);
  }
  // bottom edge
  for (let i = 0; i < n; i++) {
    const x = map(i, 0, n - 1, margin, width - margin);
    const y = height - margin;
    drawArrow(x, y, ang, len);
  }
  // left edge
  for (let i = 0; i < n; i++) {
    const x = margin;
    const y = map(i, 0, n - 1, margin, height - margin);
    drawArrow(x, y, ang, len);
  }
  // right edge
  for (let i = 0; i < n; i++) {
    const x = width - margin;
    const y = map(i, 0, n - 1, margin, height - margin);
    drawArrow(x, y, ang, len);
  }

  // interior grid (same frequency)
  const gx0 = margin + 30;
  const gx1 = width - margin - 30;
  const gy0 = margin + 30;
  const gy1 = height - margin - 30;

  // Slightly fainter than border
  stroke(200, 230, 255, 95);
  strokeWeight(2);

  for (let iy = 0; iy < n; iy++) {
    const y = map(iy, 0, n - 1, gy0, gy1);
    for (let ix = 0; ix < n; ix++) {
      const x = map(ix, 0, n - 1, gx0, gx1);
      drawArrow(x, y, ang, len);
    }
  }

  strokeWeight(1);
}

function drawArrow(x, y, ang, len) {
  const x2 = x + Math.cos(ang) * len;
  const y2 = y + Math.sin(ang) * len;
  line(x, y, x2, y2);

  const ah = 6;
  const a = Math.atan2(y2 - y, x2 - x);
  line(x2, y2, x2 - ah * Math.cos(a - 0.6), y2 - ah * Math.sin(a - 0.6));
  line(x2, y2, x2 - ah * Math.cos(a + 0.6), y2 - ah * Math.sin(a + 0.6));
}

function drawUI(debug, joy) {
    // Only show if this looks like a touch device
    const isTouch = (typeof window !== 'undefined') && ('ontouchstart' in window || navigator.maxTouchPoints > 0);

  // Draw virtual joysticks (if touch supported/active)
  drawTouchJoysticks();

  // Top-left status panel
  const isNarrow = width < 720;
  
  const x0 = 20;
  const y0 = 20;
  // Responsive width for mobile
  const w = Math.min(360, width - 40);
  // Compact height for mobile to fit wind panel below
  const h = isNarrow ? 110 : 178;

  noStroke();
  fill(0, 0, 0, 80);
  rect(x0, y0, w, h, 10);


  fill(255);
  textSize(14);
  textAlign(LEFT, BASELINE);

  const speed = Math.hypot(boatState.vel.x, boatState.vel.y);
  const knots = speed * 1.94384;

  let y = y0 + 24;
  text(modeText, x0 + 14, y); y += 22;
  text(`Speed: ${speed.toFixed(2)} m/s  (${knots.toFixed(2)} kt)`, x0 + 14, y); y += 20;
  text(`Heading: ${degrees(boatState.heading).toFixed(1)}°   r: ${boatState.angularVel.toFixed(2)} rad/s`, x0 + 14, y); y += 20;
  
  // Hide debug details on mobile to save space
  if (!isNarrow) {
     text(`Net |F|: ${debug.netF.toFixed(0)} N   Net τ: ${debug.netTorque.toFixed(0)} N·m`, x0 + 14, y); y += 20;
     text(`Hydro: Cf=${debug.hydroCf.toExponential(2)}  Re=${debug.hydroRe.toExponential(2)}`, x0 + 14, y);
  }

  // Thruster panel (bottom-left) -- Hide on small touch screens to avoid overlap
  if (!isTouch || width > 600) {
      drawThrusterPanel();
  } else {
     // On mobile, show a simplified text line above bottom status?
     // Or just nothing to keep clean.
  }

  // Bottom-right docking metrics (always on) -- Hide on small touch screens to avoid overlap
  if (!isTouch || width > 600) {
      drawDockMetricsPanel();
  }

  // Bottom status line with input info
  fill(255);
  textSize(Math.min(13, width / 40)); // Scale text for very narrow screens
  
  // Show version center-bottom
  push();
  textAlign(CENTER, BASELINE);
  fill(150, 150, 150);
  text(SIM_VERSION, width / 2, height - 18);
  pop();

  if (joy && joy.connected) {
    if (joy.source === 'keyboard') {
      text(`Keyboard: W/S surge, Q/E sway, A/D yaw | fx ${joy.fx.toFixed(2)}  fy ${joy.fy.toFixed(2)}  yaw ${joy.yaw.toFixed(2)}`, 20, height - 18);
    } else if (joy.source === 'touch') {
       text(`Touch Input Active | fx ${joy.fx.toFixed(2)}  fy ${joy.fy.toFixed(2)}  yaw ${joy.yaw.toFixed(2)}`, 20, height - 18);
    } else {
      text(`Gamepad: fx ${joy.fx.toFixed(2)}  fy ${joy.fy.toFixed(2)}  yaw(axis5) ${joy.yaw.toFixed(2)}  axes:${joy.axesCount}`, 20, height - 18);
    }
  } else {
     if (isTouch && width < 600) {
         text("Use onscreen joysticks to move and turn", 20, height - 18);
     } else {
        text("Keyboard: W/S surge | Q/E sway | A/D yaw | or connect gamepad", 20, height - 18);
     }
  }
}

function drawDockMetricsPanel() {
  const pad = 14;
  const w = Math.min(280, width - 40);
  const h = 86;
  const x0 = width - w - pad;
  const y0 = height - h - pad;

  noStroke();
  fill(0, 0, 0, 80);
  rect(x0, y0, w, h, 10);

  fill(255);
  textSize(14);
  text("Docking", x0 + 14, y0 + 24);

  textSize(13);
  const dft = lastDockMetrics.distance_ft;
  const dm = lastDockMetrics.distance_m;
  text(`Distance: ${dft.toFixed(1)} ft (${dm.toFixed(2)} m)`, x0 + 14, y0 + 46);
  text(`Approach angle: ${lastDockMetrics.approach_deg.toFixed(1)}°`, x0 + 14, y0 + 66);

  if (lastDockMetrics.collided) {
    fill(255, 120, 120);
    text("CONTACT", x0 + w - 90, y0 + 24);
  }
}

function drawThrusterPanel() {
  const x0 = 20;
  const w = Math.min(360, width - 40);
  const h = 170;
  // Pin near bottom-left, keep above bottom status line
  const y0 = Math.max(230, height - h - 40);

  noStroke();
  fill(0, 0, 0, 80);
  rect(x0, y0, w, h, 10);

  fill(255);
  textSize(14);
  text("Thrusters", x0 + 14, y0 + 24);

  const speed = Math.hypot(boatState.vel.x, boatState.vel.y);
  const Veff = Math.max(speed, 0.6); // must match thrust model Vmin

  let totalP_W = 0;
  let y = y0 + 48;

  for (let i = 0; i < 3; i++) {
    const t = BOAT_CONFIG.thrusters[i];
    const cmd = clamp(boatState.thrusterCmd[i], -1, 1);
    
    const thrustInfo = thrustLimitForThruster(t, speed, false);
    
    // Determine limit based on direction
    const isForward = (cmd * t.forwardSign) >= 0;
    const limit_struct = isForward ? thrustInfo.T_forward_max : thrustInfo.T_reverse_max;
    const limit_power = isForward ? thrustInfo.T_power_fwd : thrustInfo.T_power_rev;
    const available = Math.min(limit_struct, limit_power);
    
    const Tact = Math.abs(cmd) * available;

    // Estimate input power (W): P ≈ T * Veff / (eta_eff)
    let P_est = 0;
    // Effective efficiency scales with the max thrust ratio if in reverse
    // We assume T_power_rev was derived such that P is constant.
    // P = T_power_fwd * V / eta
    // P = T_power_rev * V / eta_rev
    // => eta_rev = eta * (T_rev / T_fwd)
    
    const eta_eff = isForward ? t.eta : (t.eta * t.T_reverse_max / t.T_forward_max);

    if (eta_eff > 1e-6) {
      P_est = (Tact * Veff) / eta_eff;
      P_est = Math.min(P_est, t.P_cont);
    }

    totalP_W += P_est;

    const cmdPct = (cmd * 100).toFixed(0);
    textSize(13);
    text(
      `${t.name}: cmd ${cmdPct}%   Thrust ${Tact.toFixed(0)} N   P≈ ${(P_est / 1000).toFixed(2)} kW`,
      x0 + 14,
      y
    );
    y += 22;
  }

  textSize(13);
  text(`Total power ≈ ${(totalP_W / 1000).toFixed(2)} kW`, x0 + 14, y0 + h - 18);
}

function windowResized() {
  resizeCanvas(windowWidth, windowHeight);
  updateInterfaceLayout();
  initDock();
}

function toggleSettingsPanel() {
  settingsPanelOpen = !settingsPanelOpen;
  
  // Show/hide input fields and update values
  thrusterInputs.forEach((inputs, idx) => {
    if (settingsPanelOpen) {
      // Update input values to reflect current config
      inputs.T_forward_max.value(BOAT_CONFIG.thrusters[idx].T_forward_max.toString());
      inputs.T_reverse_max.value(BOAT_CONFIG.thrusters[idx].T_reverse_max.toString());
      inputs.eta.value(BOAT_CONFIG.thrusters[idx].eta.toString());
      inputs.forwardSign.value(BOAT_CONFIG.thrusters[idx].forwardSign.toString());
      inputs.T_forward_max.show();
      inputs.T_reverse_max.show();
      inputs.eta.show();
      inputs.forwardSign.show();
    } else {
      inputs.T_forward_max.hide();
      inputs.T_reverse_max.hide();
      inputs.eta.hide();
      inputs.forwardSign.hide();
    }
  });

}

function drawSettingsPanel() {
  push(); // Save drawing state
  
  // Semi-transparent overlay
  fill(0, 0, 0, 150);
  noStroke();
  rect(0, 0, width, height);
  
  // Settings panel
  const panelW = 640;
  const panelH = 480;
  // Ensure panel fits on screen even if window is narrow
  const panelX = Math.max(10, (width - panelW) / 2);
  const panelY = Math.max(10, (height - panelH) / 2);
  
  fill(40, 50, 65);
  stroke(100, 120, 150);
  strokeWeight(2);
  rect(panelX, panelY, panelW, panelH, 12);
  
  // Title
  fill(255);
  noStroke();
  textSize(20);
  textAlign(CENTER, TOP);
  text("Thruster Settings", panelX + panelW / 2, panelY + 20);
  
  // Instructions
  textSize(12);
  fill(200, 220, 255);
  text("Edit thruster parameters below. Changes apply immediately.", panelX + panelW / 2, panelY + 50);
  
  // (Removed duplicate column headers and thruster rows)
  
  // Column headers
  textAlign(LEFT, TOP);
  textSize(13);
  fill(180, 200, 220);
  const headerY = panelY + 85;
  text("Thruster", panelX + 30, headerY);
  text("Forward Max (N)", panelX + 150, headerY);
  text("Reverse Max (N)", panelX + 270, headerY);
  text("Efficiency (η)", panelX + 390, headerY);
  text("Forward Dir", panelX + 510, headerY);

  // Thruster rows
  BOAT_CONFIG.thrusters.forEach((t, idx) => {
    const rowY = panelY + 120 + idx * 100;

    // Thruster name
    textSize(15);
    fill(255, 255, 255);
    textAlign(LEFT, TOP);
    text(t.name, panelX + 30, rowY + 8);

    // Labels for inputs
    textSize(11);
    fill(180, 200, 220);
    text("Forward:", panelX + 150, rowY - 5);
    text("Reverse:", panelX + 270, rowY - 5);
    text("Motor efficiency:", panelX + 390, rowY - 5);
    text("Direction:", panelX + 510, rowY - 5);

    // Position input fields
    thrusterInputs[idx].T_forward_max.position(panelX + 150, rowY + 15);
    thrusterInputs[idx].T_reverse_max.position(panelX + 270, rowY + 15);
    thrusterInputs[idx].eta.position(panelX + 390, rowY + 15);
    thrusterInputs[idx].forwardSign.position(panelX + 510, rowY + 15);

    // Current values hint
    textSize(10);
    fill(150, 170, 190);
    text(`Range: 100-5000 N`, panelX + 150, rowY + 55);
    text(`Range: 100-5000 N`, panelX + 270, rowY + 55);
    text(`Range: 0.1-1.0`, panelX + 390, rowY + 55);
    text(`+1: Forward = +cmd`, panelX + 510, rowY + 55);
    text(`-1: Forward = -cmd`, panelX + 510, rowY + 70);
  });

  // Close instruction
  textSize(13);
  textAlign(CENTER, TOP);
  fill(200, 220, 255);
  text("Click the button again or press ESC to close", panelX + panelW / 2, panelY + panelH - 35);
  
  pop(); // Restore drawing state
}

function createThrusterInputs() {
  BOAT_CONFIG.thrusters.forEach((t, i) => {
    let inputs = {};
    
    // T_forward_max
    inputs.T_forward_max = createInput(t.T_forward_max.toString());
    inputs.T_forward_max.size(60);
    inputs.T_forward_max.input(() => {
      let val = parseFloat(inputs.T_forward_max.value());
      if (!isNaN(val)) t.T_forward_max = val;
    });
    inputs.T_forward_max.hide();

    // T_reverse_max
    inputs.T_reverse_max = createInput(t.T_reverse_max.toString());
    inputs.T_reverse_max.size(60);
    inputs.T_reverse_max.input(() => {
      let val = parseFloat(inputs.T_reverse_max.value());
      if (!isNaN(val)) t.T_reverse_max = val;
    });
    inputs.T_reverse_max.hide();

    // eta
    inputs.eta = createInput(t.eta.toString());
    inputs.eta.size(60);
    inputs.eta.input(() => {
      let val = parseFloat(inputs.eta.value());
      if (!isNaN(val)) t.eta = val;
    });
    inputs.eta.hide();

    // forwardSign
    inputs.forwardSign = createInput(t.forwardSign.toString());
    inputs.forwardSign.size(60);
    inputs.forwardSign.input(() => {
      let val = parseFloat(inputs.forwardSign.value());
      if (!isNaN(val)) t.forwardSign = val;
    });
    inputs.forwardSign.hide();

    thrusterInputs.push(inputs);
  });
}

function createVesselInputs() {
  // Helper to create branded inputs
  const mkInput = (obj, key) => {
    const inp = createInput(obj[key].toString());
    inp.size(60);
    inp.input(() => {
       const v = parseFloat(inp.value());
       if (!isNaN(v)) obj[key] = v;
    });
    inp.hide();
    return inp;
  };

  vesselInputs.mass = mkInput(BOAT_CONFIG, 'mass');
  vesselInputs.Cd_lat = mkInput(BOAT_CONFIG.hydro, 'Cd_lat');
  vesselInputs.A_lat = mkInput(BOAT_CONFIG.hydro, 'A_lat');
  
  vesselInputs.Cd_front = mkInput(BOAT_CONFIG.wind, 'Cd_front');
  vesselInputs.Cd_side = mkInput(BOAT_CONFIG.wind, 'Cd_side');
  vesselInputs.A_front = mkInput(BOAT_CONFIG.wind, 'A_front');
  vesselInputs.A_side = mkInput(BOAT_CONFIG.wind, 'A_side');
}

function toggleVesselSettingsPanel() {
  vesselSettingsPanelOpen = !vesselSettingsPanelOpen;
  updateVesselInputsVisibility();
}

function updateVesselInputsVisibility() {
  const show = vesselSettingsPanelOpen;
  const isNarrow = width < 720;
  const tab = vesselSettingsTab; // 'HYDRO' or 'WIND'

  const setVis = (inp, visible) => {
    if (visible) inp.show(); else inp.hide();
  };

  if(!show) {
    if (vesselInputs.mass) Object.values(vesselInputs).forEach(inp => inp.hide());
    return;
  }

  // If showing, determines which ones based on layout
  if (isNarrow) {
     const showHydro = (tab === 'HYDRO');
     const showWind  = (tab === 'WIND');
     
     setVis(vesselInputs.mass, showHydro);
     setVis(vesselInputs.Cd_lat, showHydro);
     setVis(vesselInputs.A_lat, showHydro);
     
     setVis(vesselInputs.Cd_front, showWind);
     setVis(vesselInputs.Cd_side, showWind);
     setVis(vesselInputs.A_front, showWind);
     setVis(vesselInputs.A_side, showWind);
  } else {
     // Desktop: show all
     if (vesselInputs.mass) Object.values(vesselInputs).forEach(inp => inp.show());
  }
}

function drawVesselSettingsPanel() {
  push(); 
  fill(0, 0, 0, 150);
  noStroke();
  rect(0, 0, width, height);
  
  const panelW = width < 720 ? width - 20 : 600;
  const panelH = width < 720 ? 600 : 400; 
  const panelX = Math.max(10, (width - panelW) / 2);
  const panelY = Math.max(10, (height - panelH) / 2);
  
  fill(40, 50, 65);
  stroke(100, 120, 150);
  strokeWeight(2);
  rect(panelX, panelY, panelW, panelH, 12);
  
  fill(255);
  noStroke();
  textSize(20);
  textAlign(CENTER, TOP);
  text("Vessel Settings", panelX + panelW / 2, panelY + 20);
  
  textSize(12);
  fill(200, 220, 255);
  text("Adjust Mass, Drag, and Wind properties", panelX + panelW / 2, panelY + 50);

  const isNarrow = width < 720;
  
  if (isNarrow) {
     const tabY = panelY + 80;
     const tabW = panelW / 2;
     const tabH = 40;
     
     // Hydro Tab
     fill(vesselSettingsTab === 'HYDRO' ? 100 : 60, vesselSettingsTab === 'HYDRO' ? 120 : 70, vesselSettingsTab === 'HYDRO' ? 150 : 80);
     rect(panelX, tabY, tabW, tabH);
     fill(255);
     textAlign(CENTER, CENTER);
     text("HYDRO", panelX + tabW/2, tabY + tabH/2);
     
     // Wind Tab
     fill(vesselSettingsTab === 'WIND' ? 100 : 60, vesselSettingsTab === 'WIND' ? 120 : 70, vesselSettingsTab === 'WIND' ? 150 : 80);
     rect(panelX + tabW, tabY, tabW, tabH);
     fill(255);
     text("WIND", panelX + tabW + tabW/2, tabY + tabH/2);
     
     textAlign(LEFT, CENTER);
     const contentY = tabY + tabH + 20;
     const rowH = 50;
     
     if (vesselSettingsTab === 'HYDRO') {
        text("Mass (kg)", panelX + 20, contentY);
        vesselInputs.mass.position(panelX + 150, contentY - 10);
        
        text("Cd Lateral", panelX + 20, contentY + rowH);
        vesselInputs.Cd_lat.position(panelX + 150, contentY + rowH - 10);
        
        text("Area Lat (m²)", panelX + 20, contentY + rowH*2);
        vesselInputs.A_lat.position(panelX + 150, contentY + rowH*2 - 10);
     } else {
        text("Cd Front", panelX + 20, contentY);
        vesselInputs.Cd_front.position(panelX + 150, contentY - 10);
        
        text("Cd Side", panelX + 20, contentY + rowH);
        vesselInputs.Cd_side.position(panelX + 150, contentY + rowH - 10);
        
        text("Area Front", panelX + 20, contentY + rowH*2);
        vesselInputs.A_front.position(panelX + 150, contentY + rowH*2 - 10);
        
        text("Area Side", panelX + 20, contentY + rowH*3);
        vesselInputs.A_side.position(panelX + 150, contentY + rowH*3 - 10);
     }
  } else {
     textAlign(LEFT, TOP);
     let col1 = panelX + 40;
     let col2 = panelX + 320;
     let yVal = panelY + 100;
     
     text("HYDRODYNAMICS", col1, yVal);
     yVal += 30;
     
     text("Mass (kg):", col1, yVal);
     vesselInputs.mass.position(col1 + 100, yVal);
     
     text("Cd Lateral:", col1, yVal + 40);
     vesselInputs.Cd_lat.position(col1 + 100, yVal + 40);
     
     text("Area Lat (m²):", col1, yVal + 80);
     vesselInputs.A_lat.position(col1 + 100, yVal + 80);
     
     yVal = panelY + 100;
     text("WIND AERODYNAMICS", col2, yVal);
     yVal += 30;
     
     text("Cd Front:", col2, yVal);
     vesselInputs.Cd_front.position(col2 + 100, yVal);
     
     text("Cd Side:", col2, yVal + 40);
     vesselInputs.Cd_side.position(col2 + 100, yVal + 40);

     text("Area Front:", col2, yVal + 80);
     vesselInputs.A_front.position(col2 + 100, yVal + 80);
     
     text("Area Side:", col2, yVal + 120);
     vesselInputs.A_side.position(col2 + 100, yVal + 120);
  }
  
  textAlign(CENTER, TOP);
  fill(200, 220, 255);
  text("Click the button again or press ESC to close", panelX + panelW / 2, panelY + panelH - 35);
  
  pop(); // Restore drawing state
}

function mousePressed() {
  // Handle clicks on Vessel Settings Tabs (Mobile only)
  if (vesselSettingsPanelOpen && width < 720) {
     const panelW = width - 20;
     const panelH = 600;
     const panelX = Math.max(10, (width - panelW) / 2);
     const panelY = Math.max(10, (height - panelH) / 2);
     const tabY = panelY + 80;
     const tabW = panelW / 2;
     const tabH = 40;

     if (mouseY >= tabY && mouseY <= tabY + tabH) {
       if (mouseX >= panelX && mouseX < panelX + tabW) {
         vesselSettingsTab = 'HYDRO';
         updateVesselInputsVisibility();
         return false; // Consume event
       } else if (mouseX >= panelX + tabW && mouseX < panelX + 2*tabW) {
         vesselSettingsTab = 'WIND';
         updateVesselInputsVisibility();
         return false; // Consume event
       }
     }
  }
  // Allow default behavior for other clicks
  return true;
}

// ---------------- TOUCH CONTROLS ----------------
const TOUCH_joysticks = {
  left: { 
    active: false, 
    id: -1, 
    originX: 0, originY: 0, 
    currX: 0, currY: 0, 
    valX: 0, valY: 0 // -1..1 output (surge/sway)
  },
  right: { 
    active: false, 
    id: -1, 
    originX: 0, originY: 0, 
    currX: 0, currY: 0, 
    valX: 0 // -1..1 output (yaw)
  },
  maxDist: 60 // px radius
};

function readTouch() {
  // Returns object compatible with our input format
  // fx: surge (up/down on left stick) -> -1..1
  // fy: sway (left/right on left stick) -> -1..1
  // yaw: (right stick x) -> -1..1
  
  // NOTE: In screen coords, Y is down. So stick UP is negative Y.
  // We want forward thrust (+fx) when stick is UP (-Y).
  // So fx = -valY
  
  // Sway: Stick right (+X) is starboard sway (+fy).
  // fy = valX
  
  // Yaw: Right stick right (+X) is CW turn (+yaw).
  
  let fx = 0, fy = 0, yaw = 0;
  
  if (TOUCH_joysticks.left.active) {
    fx = -TOUCH_joysticks.left.valY;
    fy = TOUCH_joysticks.left.valX; // Right is starboard (+)
  }
  
  if (TOUCH_joysticks.right.active) {
    yaw = TOUCH_joysticks.right.valX; 
  }

  // Deadband is handled nicely by the touch logic, but let's be sure
  if (Math.abs(fx) < 0.05) fx = 0;
  if (Math.abs(fy) < 0.05) fy = 0;
  if (Math.abs(yaw) < 0.05) yaw = 0;

  return { fx, fy, yaw, active: (TOUCH_joysticks.left.active || TOUCH_joysticks.right.active) };
}

function drawTouchJoysticks() {
    // Only show if this looks like a touch device, or if touch is currently active
    const isTouch = (typeof window !== 'undefined') && ('ontouchstart' in window || navigator.maxTouchPoints > 0);
    const anyActive = (TOUCH_joysticks.left.active || TOUCH_joysticks.right.active);
    
    if (!isTouch && !anyActive) return;

  push();
  // Only draw if we detect touch capability or are active
  // Simple heuristic: just always check specific active states
  
  noStroke();
  
  // Draw Left Stick
  if (TOUCH_joysticks.left.active) {
    fill(255, 255, 255, 50);
    circle(TOUCH_joysticks.left.originX, TOUCH_joysticks.left.originY, TOUCH_joysticks.maxDist * 2);
    fill(255, 200, 100, 200);
    circle(TOUCH_joysticks.left.currX, TOUCH_joysticks.left.currY, 50);
  } else {
    // Hint circle
    fill(255, 255, 255, 20);
    circle(100, height - 100, 80);
    fill(255);
    textAlign(CENTER);
    textSize(10);
    text("MOVE", 100, height - 100);
  }

  // Draw Right Stick
  if (TOUCH_joysticks.right.active) {
    fill(255, 255, 255, 50);
    circle(TOUCH_joysticks.right.originX, TOUCH_joysticks.right.originY, TOUCH_joysticks.maxDist * 2);
    fill(100, 200, 255, 200);
    circle(TOUCH_joysticks.right.currX, TOUCH_joysticks.right.currY, 50);
  } else {
    // Hint circle
    fill(255, 255, 255, 20);
    circle(width - 100, height - 100, 80);
    fill(255);
    textAlign(CENTER);
    textSize(10);
    text("TURN", width - 100, height - 100);
  }
  pop();
}

// Helper to detect if a touch is on a known UI element interaction zone.
// Used to prevent joystick activation when using UI.
function isTouchOnUI(x, y) {
   const isNarrow = width < 720;
   
   if (isNarrow) {
       // Mobile Layout Zones
       
       // 1. Wind + Toggle + Buttons area (Top-Left / Mid-Left)
       // Wind panel starts y=140, h=90 => y=[140..230]
       // Buttons start y=230 down to ~400
       // Let's protect the whole left strip for safety?
       // width < 300px?
       // Or simpler: define a rectangle for the UI block
       
       // Wind Panel zone
       if (y >= 140 && y < 240 && x < width) return true;
       
       // Buttons zone (toggle, settings, vessel)
       // They are at x=20..
       // Wind panel is y=140. Mobile buttons start at wy + 100 => 240.
       // However, DOM elements might have padding or the touch area might extend upwards.
       // Let's protect form y=220 to safely include the top of the checkbox.
       if (x < 360 && y >= 220 && y < 500) return true;
       
       // Vessel Settings Panel (if open)
       if (vesselSettingsPanelOpen) {
           const panelW = width - 20;
           const panelH = 600;
           const panelX = Math.max(10, (width - panelW) / 2);
           const panelY = Math.max(10, (height - panelH) / 2);
           if (x >= panelX && x <= panelX + panelW && y >= panelY && y <= panelY + panelH) return true;
       }
       
       // Thruster Settings Panel (if open)
       if (settingsPanelOpen) {
          // It's a modal overlay, so technically whole screen is UI
          return true; 
       }
   }
   
   return false;
}

function touchStarted() {
  // Process all new touches
  let consumed = false;
  
  // joystick config
  const joystickTopLimit = height - 250; // Only allow joysticks in bottom ~250px

  // p5 stores touches[] array
  for (let t of touches) {
    // strict check: ignore touches that are not in the joystick zone
    // AND ignore touches that are specifically on UI elements
    if (t.y < joystickTopLimit || isTouchOnUI(t.x, t.y)) continue; 

    // Determine if this touch is left half or right half
    if (t.x < width / 2) {
      // Left stick (Translation)
      if (!TOUCH_joysticks.left.active) {
        TOUCH_joysticks.left.active = true;
        TOUCH_joysticks.left.id = t.id;
        TOUCH_joysticks.left.originX = t.x;
        TOUCH_joysticks.left.originY = t.y;
        TOUCH_joysticks.left.currX = t.x;
        TOUCH_joysticks.left.currY = t.y;
        TOUCH_joysticks.left.valX = 0;
        TOUCH_joysticks.left.valY = 0;
        consumed = true;
      }
    } else {
      // Right stick (Rotation)
      if (!TOUCH_joysticks.right.active) {
        TOUCH_joysticks.right.active = true;
        TOUCH_joysticks.right.id = t.id;
        TOUCH_joysticks.right.originX = t.x;
        TOUCH_joysticks.right.originY = t.y;
        TOUCH_joysticks.right.currX = t.x;
        TOUCH_joysticks.right.currY = t.y;
        TOUCH_joysticks.right.valX = 0; // Only care about X for yaw
        consumed = true;
      }
    }
  }
  
  // Only prevent default if we actually started a joystick interaction
  // Otherwise allow clicks on buttons etc.
  if (consumed) {
    return false;
  }
  // return true lets the event pass through (e.g. to DOM elements)
  return true;
}

function touchMoved() {
  let joystickMoved = false;

  for (let t of touches) {
    if (TOUCH_joysticks.left.active && t.id === TOUCH_joysticks.left.id) {
      const dx = t.x - TOUCH_joysticks.left.originX;
      const dy = t.y - TOUCH_joysticks.left.originY;
      const dist = Math.sqrt(dx*dx + dy*dy);
      const clampedDist = Math.min(dist, TOUCH_joysticks.maxDist);
      const angle = Math.atan2(dy, dx);
      
      TOUCH_joysticks.left.currX = TOUCH_joysticks.left.originX + Math.cos(angle) * clampedDist;
      TOUCH_joysticks.left.currY = TOUCH_joysticks.left.originY + Math.sin(angle) * clampedDist;
      
      // Normalize output -1..1
      TOUCH_joysticks.left.valX = (TOUCH_joysticks.left.currX - TOUCH_joysticks.left.originX) / TOUCH_joysticks.maxDist;
      TOUCH_joysticks.left.valY = (TOUCH_joysticks.left.currY - TOUCH_joysticks.left.originY) / TOUCH_joysticks.maxDist;
      joystickMoved = true;
    }
    
    if (TOUCH_joysticks.right.active && t.id === TOUCH_joysticks.right.id) {
       const dx = t.x - TOUCH_joysticks.right.originX;
       const dy = t.y - TOUCH_joysticks.right.originY;
       // Only care about X for yaw really, but visualise 2d
       const dist = Math.sqrt(dx*dx + dy*dy);
       const clampedDist = Math.min(dist, TOUCH_joysticks.maxDist);
       const angle = Math.atan2(dy, dx);

       TOUCH_joysticks.right.currX = TOUCH_joysticks.right.originX + Math.cos(angle) * clampedDist;
       TOUCH_joysticks.right.currY = TOUCH_joysticks.right.originY + Math.sin(angle) * clampedDist;

       TOUCH_joysticks.right.valX = (TOUCH_joysticks.right.currX - TOUCH_joysticks.right.originX) / TOUCH_joysticks.maxDist;
       joystickMoved = true;
    }
  }
  
  // NOTE: If we return `false`, it prevents default scrolling behavior on mobile.
  // We want to prevent scroll ONLY if we are actively dragging a joystick.
  // If we are dragging a slider, we must return true (or let it pass) for it to update.
  if (joystickMoved) {
      return false; 
  }
  return true;
}

function touchEnded() {
  // Check which touches remain
  const remainingIds = touches.map(t => t.id);
  
  if (TOUCH_joysticks.left.active && !remainingIds.includes(TOUCH_joysticks.left.id)) {
    TOUCH_joysticks.left.active = false;
    TOUCH_joysticks.left.valX = 0;
    TOUCH_joysticks.left.valY = 0;
  }
  
  if (TOUCH_joysticks.right.active && !remainingIds.includes(TOUCH_joysticks.right.id)) {
    TOUCH_joysticks.right.active = false;
    TOUCH_joysticks.right.valX = 0;
  }
  return false;
}

function drawWindPanelLabel() {
  const isNarrow = width < 720;
  
  // Calculate position same as updateInterfaceLayout
  let x0, y0, panelW, panelH;
  
  if (isNarrow) {
     x0 = 20;
     y0 = 140; // under status
     panelW = Math.min(260, width - 40);
     panelH = 90;
  } else {
     panelW = 260;
     panelH = 170;
     x0 = Math.max(20, width - panelW - 20);
     y0 = 20;
  }

  noStroke();
  fill(0, 0, 0, 80);
  rect(x0, y0, panelW, panelH, 10);
  
  fill(255);
  textSize(14);
  textAlign(LEFT, TOP);
  text("Wind Control", x0 + 14, y0 + 10);
  
  textSize(12);
  let speed = windSpeedSlider ? windSpeedSlider.value() : windSpeed;
  let dir = windDirSlider ? windDirSlider.value() : windDirectionDeg;
  
  if (isNarrow) {
     text(`Speed: ${speed} m/s`, x0 + 14, y0 + 32);
     text(`Dir: ${dir}°`, x0 + 14, y0 + 66);
  } else {
    // Desktop layout with description
    text(`Speed: ${speed} m/s`, x0 + 14, y0 + 90);
    text(`Dir: ${dir}°`, x0 + 14, y0 + 124);
    
    // Wind vector viz inside panel
    const cx = x0 + panelW / 2;
    const cy = y0 + 60;
    push();
    translate(cx, cy);
    noFill();
    stroke(255, 255, 255, 50);
    circle(0, 0, 40);
    
    // Arrow pointing FROM wind direction
    rotate(radians(dir + 180));
    stroke(100, 200, 255);
    strokeWeight(2);
    line(0, -15, 0, 15);
    line(0, 15, -4, 10);
    line(0, 15, 4, 10);
    pop();
  }
}
