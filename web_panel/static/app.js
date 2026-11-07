/**
 * TELBot Operator Console - Frontend Application
 * - WebHID Controller Reader (Logitech Gamepad F310 PoC compatible)
 * - Configurable Axis & Button Mapping (persisted in localStorage)
 * - Real-time WebSocket Control Stream (/ws/control)
 * - Live Camera Stream & Telemetry Monitor
 */

const F310_PRODUCT_ID = 49693; // 0xC21D (Logitech F310)

// Exact F310 raw range definitions from web_panel/example/index.ts
const F310_RAW_RANGES = [
  null,
  { range: 256, offset: 128, reverse: false }, // Byte 1: Left Stick X
  null,
  { range: 256, offset: 128, reverse: true },  // Byte 3: Left Stick Y
  null,
  { range: 256, offset: 128, reverse: false }, // Byte 5: Right Stick X
  null,
  { range: 256, offset: 128, reverse: true },  // Byte 7: Right Stick Y
  { range: 128, offset: 0, reverse: false },   // Byte 8: Trigger 8 (0..100%)
  { range: 256, offset: 128, reverse: false }, // Byte 9: Trigger 9 (-50..+50%)
  null,
  null,
  null,
  null,
];

const F310_BUTTONS = ["A", "B", "X", "Y", "LB", "RB", "Back", "Start"];

// Available Axis Sources for Mapping
const AXIS_SOURCES = [
  { id: "byte3", label: "Left Stick Y (Byte 3)" },
  { id: "byte1", label: "Left Stick X (Byte 1)" },
  { id: "byte7", label: "Right Stick Y (Byte 7)" },
  { id: "byte5", label: "Right Stick X (Byte 5)" },
  { id: "byte8", label: "Trigger Axis (Byte 8)" },
  { id: "byte9", label: "Trigger Axis (Byte 9)" },
  { id: "none", label: "Disabled (0)" },
];

// Configurable Robot Axes
const ROBOT_AXES = [
  { key: "vx", label: "Chassis Fwd/Back (VX)" },
  { key: "vy", label: "Chassis Strafe (VY)" },
  { key: "omega", label: "Chassis Rotate (Omega)" },
  { key: "angle", label: "Shooter Pitch (Angle)" },
];

// Available Button Actions
const BUTTON_ACTIONS = [
  { id: "emergency_stop", label: "STOP ALL (Emergency)", short: "STOP ALL" },
  { id: "shooter_hold", label: "Shooter Flywheel (Hold)", short: "Shoot (Hold)" },
  { id: "shooter_toggle", label: "Shooter Flywheel (Toggle)", short: "Shoot (Tgl)" },
  { id: "conveyor_trigger_hold", label: "Conveyor Trigger / Feed (Hold)", short: "Feed (Hold)" },
  { id: "conveyor_spinner_toggle", label: "Conveyor Spinner (Toggle)", short: "Spinner (Tgl)" },
  { id: "conveyor_spinner_rev_hold", label: "Conveyor Spinner Reverse (Hold)", short: "Spin Rev" },
  { id: "angle_up_hold", label: "Shooter Angle Up (Hold)", short: "Pitch +" },
  { id: "angle_down_hold", label: "Shooter Angle Down (Hold)", short: "Pitch -" },
  { id: "angle_disable", label: "Shooter Angle Disable (Release)", short: "Pitch Off" },
  { id: "zero_all", label: "Zero All Latch States", short: "Reset States" },
  { id: "none", label: "Unassigned (None)", short: "---" },
];

const STORAGE_KEY = "telbot_controller_mapping_v1";

function getDefaultMapping() {
  return {
    deadzonePct: 8,
    speedScalePct: 100,
    shooterSpeed: 800,
    conveyorSpeed: 700,
    axes: {
      vx: { source: "byte3", invert: false },
      vy: { source: "byte1", invert: false },
      omega: { source: "byte5", invert: false },
      angle: { source: "byte7", invert: false },
    },
    buttons: {
      A: "conveyor_trigger_hold",
      B: "emergency_stop",
      X: "conveyor_spinner_toggle",
      Y: "shooter_toggle",
      LB: "conveyor_spinner_rev_hold",
      RB: "shooter_hold",
      Back: "emergency_stop",
      Start: "zero_all",
    },
  };
}

let mappingConfig = loadMapping();
let currentDevice = null;
let reportTableEl = null;

// Live decoded controller state
const controllerState = {
  connected: false,
  // Normalized axes in [-1.0, +1.0]
  axesNorm: {
    byte1: 0,
    byte3: 0,
    byte5: 0,
    byte7: 0,
    byte8: 0,
    byte9: 0,
  },
  // Percentage values matching PoC (-50..+50 or 0..100)
  axesPct: {
    byte1: 0,
    byte3: 0,
    byte5: 0,
    byte7: 0,
    byte8: 0,
    byte9: 0,
  },
  buttons: {
    A: false,
    B: false,
    X: false,
    Y: false,
    LB: false,
    RB: false,
    Back: false,
    Start: false,
  },
  prevButtons: {
    A: false,
    B: false,
    X: false,
    Y: false,
    LB: false,
    RB: false,
    Back: false,
    Start: false,
  },
};

// Toggle states controlled by toggle-type button actions
const toggleState = {
  shooterActive: false,
  spinnerActive: false,
};

// Outgoing command snapshot
const lastSentCmd = {
  vx: 0,
  vy: 0,
  omega: 0,
  flywheel: 0,
  angle_run: false,
  angle_reversed: false,
  trigger: 0,
  spinner_run: false,
  spinner_reversed: false,
};

let ws = null;
let wsConnected = false;
let controlArmed = true;

// ============================================================================
// Initialization
// ============================================================================
document.addEventListener("DOMContentLoaded", async () => {
  reportTableEl = document.getElementById("report-table");
  document.getElementById("host-origin-label").textContent = window.location.origin;

  initThemeToggle();
  initMatchControls();
  initButtonGridUI();
  initMappingUI();
  initVideoControls();
  initControlSwitches();
  initFieldPoseMap();
  initWebSocket();
  initStatusPolling();
  await initWebHID();

  // 25 Hz (40 ms) real-time control loop
  setInterval(controlTick, 40);
});

// ============================================================================
// Persistence & Mapping UI
// ============================================================================
function loadMapping() {
  try {
    const raw = localStorage.getItem(STORAGE_KEY);
    if (!raw) return getDefaultMapping();
    const parsed = JSON.parse(raw);
    const def = getDefaultMapping();
    return {
      ...def,
      ...parsed,
      axes: { ...def.axes, ...(parsed.axes || {}) },
      buttons: { ...def.buttons, ...(parsed.buttons || {}) },
    };
  } catch (e) {
    return getDefaultMapping();
  }
}

function saveMapping() {
  try {
    localStorage.setItem(STORAGE_KEY, JSON.stringify(mappingConfig));
  } catch (e) {
    console.warn("Failed to save mapping to localStorage:", e);
  }
  updateButtonGridLabels();
}

function initButtonGridUI() {
  const grid = document.getElementById("f310-btn-grid");
  grid.innerHTML = "";
  for (const btnName of F310_BUTTONS) {
    const pill = document.createElement("div");
    pill.className = "controller-btn-pill";
    pill.id = `btn-pill-${btnName}`;
    pill.innerHTML = `
      <span class="btn-name mono">${btnName}</span>
      <span class="btn-action-label" id="btn-pill-action-${btnName}">---</span>
    `;
    grid.appendChild(pill);
  }
  updateButtonGridLabels();
}

function updateButtonGridLabels() {
  for (const btnName of F310_BUTTONS) {
    const actionId = mappingConfig.buttons[btnName] || "none";
    const found = BUTTON_ACTIONS.find((a) => a.id === actionId);
    const el = document.getElementById(`btn-pill-action-${btnName}`);
    if (el) el.textContent = found ? found.short : "---";
  }
}

function initMappingUI() {
  // Sliders
  const bindSlider = (sliderId, labelId, key, suffix = "") => {
    const slider = document.getElementById(sliderId);
    const label = document.getElementById(labelId);
    slider.value = mappingConfig[key];
    label.textContent = `${mappingConfig[key]}${suffix}`;
    slider.addEventListener("input", () => {
      mappingConfig[key] = Number(slider.value);
      label.textContent = `${mappingConfig[key]}${suffix}`;
      saveMapping();
    });
  };

  bindSlider("deadzone-slider", "deadzone-val-label", "deadzonePct", "%");
  bindSlider("speed-scale-slider", "speed-scale-val-label", "speedScalePct", "%");
  bindSlider("shooter-speed-slider", "shooter-speed-val-label", "shooterSpeed", "");
  bindSlider("conveyor-speed-slider", "conveyor-speed-val-label", "conveyorSpeed", "");

  // Axis Rows
  const axisContainer = document.getElementById("axis-mapping-container");
  axisContainer.innerHTML = "";
  for (const rAxis of ROBOT_AXES) {
    const cfg = mappingConfig.axes[rAxis.key];
    const row = document.createElement("div");
    row.className = "mapping-row";

    const optionsHtml = AXIS_SOURCES.map(
      (s) => `<option value="${s.id}" ${cfg.source === s.id ? "selected" : ""}>${s.label}</option>`
    ).join("");

    row.innerHTML = `
      <span class="small-text">${rAxis.label}</span>
      <div class="row middle-align">
        <div class="field small border round no-margin" style="width: 175px;">
          <select id="map-axis-src-${rAxis.key}">${optionsHtml}</select>
          <i>arrow_drop_down</i>
        </div>
        <label class="checkbox small-text">
          <input type="checkbox" id="map-axis-inv-${rAxis.key}" ${cfg.invert ? "checked" : ""} />
          <span>Inv</span>
        </label>
      </div>
    `;
    axisContainer.appendChild(row);

    document.getElementById(`map-axis-src-${rAxis.key}`).addEventListener("change", (e) => {
      mappingConfig.axes[rAxis.key].source = e.target.value;
      saveMapping();
    });
    document.getElementById(`map-axis-inv-${rAxis.key}`).addEventListener("change", (e) => {
      mappingConfig.axes[rAxis.key].invert = e.target.checked;
      saveMapping();
    });
  }

  // Button Rows
  const btnContainer = document.getElementById("button-mapping-container");
  btnContainer.innerHTML = "";
  for (const btnName of F310_BUTTONS) {
    const currentAction = mappingConfig.buttons[btnName] || "none";
    const row = document.createElement("div");
    row.className = "mapping-row";

    const optionsHtml = BUTTON_ACTIONS.map(
      (a) => `<option value="${a.id}" ${currentAction === a.id ? "selected" : ""}>${a.label}</option>`
    ).join("");

    row.innerHTML = `
      <span class="mono bold small-text">Button ${btnName}</span>
      <div class="field small border round no-margin" style="width: 235px;">
        <select id="map-btn-${btnName}">${optionsHtml}</select>
        <i>arrow_drop_down</i>
      </div>
    `;
    btnContainer.appendChild(row);

    document.getElementById(`map-btn-${btnName}`).addEventListener("change", (e) => {
      mappingConfig.buttons[btnName] = e.target.value;
      saveMapping();
    });
  }

  // Reset Button
  document.getElementById("reset-mapping-btn").addEventListener("click", () => {
    mappingConfig = getDefaultMapping();
    saveMapping();
    initMappingUI();
  });
}

// ============================================================================
// WebHID Controller Reader (Ported from web_panel/example/index.ts)
// ============================================================================
async function initWebHID() {
  if (!("hid" in navigator)) {
    const warnCard = document.getElementById("webhid-warning-card");
    warnCard.style.display = "block";
    const httpsLink = document.getElementById("https-redirect-link");
    httpsLink.href = `https://${window.location.host}${window.location.pathname}`;
    httpsLink.textContent = `https://${window.location.host}`;
    document.getElementById("connect-btn").setAttribute("disabled", "true");
    return;
  }

  document.getElementById("connect-btn").addEventListener("click", async () => {
    try {
      if (!(await checkHIDPerm())) {
        await navigator.hid.requestDevice({ filters: [] });
      }
      await setHIDDevices();
    } catch (err) {
      console.error("WebHID connect error:", err);
    }
  });

  document.getElementById("forget-btn").addEventListener("click", async () => {
    if (currentDevice) {
      try {
        await currentDevice.close();
        if (typeof currentDevice.forget === "function") {
          await currentDevice.forget();
        }
      } catch (err) {
        console.error(err);
      }
      clearCurrentDevice();
    }
  });

  navigator.hid.addEventListener("disconnect", (event) => {
    if (currentDevice && event.device === currentDevice) {
      console.warn(`Lost connection to HID device: ${event.device.productName}`);
      clearCurrentDevice();
      sendEmergencyStop();
    }
  });

  // Auto-connect if permission was already granted in browser
  if (await checkHIDPerm()) {
    await setHIDDevices();
  }
}

async function checkHIDPerm() {
  const devices = await navigator.hid.getDevices();
  return devices.length !== 0;
}

async function setHIDDevices() {
  const devices = await navigator.hid.getDevices();
  if (!devices || devices.length === 0) return;

  // Prefer Logitech F310 (productId 49693) if multiple HID devices exist
  const device = devices.find((d) => d.productId === F310_PRODUCT_ID) || devices[0];
  currentDevice = device;
  controllerState.connected = true;

  document.getElementById("connected-device-text").textContent =
    `${device.productName || "HID Controller"} (0x${device.productId.toString(16)})`;
  document.getElementById("forget-btn").removeAttribute("disabled");

  device.oninputreport = (event) => {
    const data = new Uint8Array(event.data.buffer);
    if (!reportTableEl.hasAttribute("data-initialized")) {
      initiateTable(data.length);
    }
    for (let i = 0; i < data.length; i++) {
      const cell = document.getElementById(`report-raw-${i}`);
      if (cell) cell.textContent = data[i].toString();
    }
    rawF310ToDisplay(data);
  };

  if (!device.opened) {
    await device.open();
  }
}

function initiateTable(length) {
  clearTable();
  const idxRow = document.getElementById("report-table-idx");
  const rawRow = document.getElementById("report-table-raw");
  const percentRow = document.getElementById("report-table-percnt");
  const btnRow = document.getElementById("report-table-btn");

  for (let i = 0; i < length; i++) {
    const idxCell = document.createElement("td");
    idxCell.textContent = i.toString();
    idxCell.id = `report-idx-${i}`;
    idxRow.appendChild(idxCell);

    const rawCell = document.createElement("td");
    rawCell.textContent = "---";
    rawCell.id = `report-raw-${i}`;
    rawRow.appendChild(rawCell);

    const percentCell = document.createElement("td");
    percentCell.textContent = "---";
    percentCell.id = `report-percnt-${i}`;
    percentRow.appendChild(percentCell);
  }

  const btnCell = document.createElement("td");
  btnCell.textContent = "---";
  btnCell.colSpan = length;
  btnCell.id = "report-btn";
  btnRow.appendChild(btnCell);

  reportTableEl.setAttribute("data-initialized", "true");
}

function clearTable() {
  const idxRow = document.getElementById("report-table-idx");
  const rawRow = document.getElementById("report-table-raw");
  const percentRow = document.getElementById("report-table-percnt");
  const btnRow = document.getElementById("report-table-btn");

  idxRow.innerHTML = "<th>#</th>";
  rawRow.innerHTML = "<td>RAW</td>";
  percentRow.innerHTML = "<td>%</td>";
  btnRow.innerHTML = "<td>BTN</td>";

  reportTableEl.removeAttribute("data-initialized");
}

function rawF310ToDisplay(rawArray) {
  for (let i = 0; i < rawArray.length; i++) {
    if (F310_RAW_RANGES[i]) {
      const { range, offset, reverse } = F310_RAW_RANGES[i];
      let percent = ((rawArray[i] - offset) / range) * 100;
      if (reverse) percent *= -1;

      const cell = document.getElementById(`report-percnt-${i}`);
      if (cell) cell.textContent = percent.toFixed(1);

      const key = `byte${i}`;
      controllerState.axesPct[key] = percent;

      // Normalize into [-1.0, +1.0] for robot control
      if (i === 8) {
        // Byte 8 has range 128, offset 0 (0..100%)
        controllerState.axesNorm[key] = Math.max(-1, Math.min(1, percent / 100));
      } else {
        // Centered sticks (offset 128, range 256 -> percent is -50%..+50%)
        controllerState.axesNorm[key] = Math.max(-1, Math.min(1, percent / 50));
      }
    } else if (i === 10) {
      const rawBtnState = rawArray[i].toString(2).padStart(F310_BUTTONS.length, "0");
      const btnStates = [];

      // Save previous button states for rising-edge toggle detection
      for (const bName of F310_BUTTONS) {
        controllerState.prevButtons[bName] = controllerState.buttons[bName];
      }

      for (let j = 0; j < F310_BUTTONS.length; j++) {
        const bName = F310_BUTTONS[F310_BUTTONS.length - j - 1];
        const isPressed = rawBtnState[j] === "1";
        controllerState.buttons[bName] = isPressed;
        if (isPressed) btnStates.push(bName);
      }

      const reportBtnEl = document.getElementById("report-btn");
      if (reportBtnEl) {
        if (btnStates.length === 0) {
          reportBtnEl.textContent = "---";
        } else {
          btnStates.reverse();
          reportBtnEl.textContent = btnStates.join(", ");
        }
      }

      processButtonEdges();
    }
  }

  updateControllerVisuals();
}

function processButtonEdges() {
  for (const bName of F310_BUTTONS) {
    const pressedNow = controllerState.buttons[bName];
    const pressedBefore = controllerState.prevButtons[bName];

    // Rising edge (button just pressed)
    if (pressedNow && !pressedBefore) {
      const action = mappingConfig.buttons[bName] || "none";
      if (action === "emergency_stop") {
        toggleState.shooterActive = false;
        toggleState.spinnerActive = false;
        sendEmergencyStop();
      } else if (action === "shooter_toggle") {
        toggleState.shooterActive = !toggleState.shooterActive;
      } else if (action === "conveyor_spinner_toggle") {
        toggleState.spinnerActive = !toggleState.spinnerActive;
      } else if (action === "angle_disable") {
        sendWsJson({ type: "shooter_disable_angle" });
      } else if (action === "zero_all") {
        toggleState.shooterActive = false;
        toggleState.spinnerActive = false;
      }
    }
  }
}

function updateControllerVisuals() {
  // Left Stick (Byte 1 X, Byte 3 Y)
  const lxPct = controllerState.axesPct.byte1 || 0;
  const lyPct = controllerState.axesPct.byte3 || 0;
  const leftDot = document.getElementById("left-stick-dot");
  leftDot.style.left = `${50 + lxPct}%`;
  leftDot.style.top = `${50 - lyPct}%`;
  document.getElementById("left-stick-text").textContent =
    `X: ${lxPct.toFixed(1)}% | Y: ${lyPct.toFixed(1)}%`;

  // Right Stick (Byte 5 X, Byte 7 Y)
  const rxPct = controllerState.axesPct.byte5 || 0;
  const ryPct = controllerState.axesPct.byte7 || 0;
  const rightDot = document.getElementById("right-stick-dot");
  rightDot.style.left = `${50 + rxPct}%`;
  rightDot.style.top = `${50 - ryPct}%`;
  document.getElementById("right-stick-text").textContent =
    `X: ${rxPct.toFixed(1)}% | Y: ${ryPct.toFixed(1)}%`;

  // Triggers (Byte 8 & Byte 9)
  const b8Pct = controllerState.axesPct.byte8 || 0;
  const b9Pct = controllerState.axesPct.byte9 || 0;
  document.getElementById("axis-b8-text").textContent = `${b8Pct.toFixed(1)}%`;
  document.getElementById("axis-b8-bar").value = Math.max(0, Math.min(100, b8Pct));
  document.getElementById("axis-b9-text").textContent = `${b9Pct.toFixed(1)}%`;
  document.getElementById("axis-b9-bar").value = Math.max(0, Math.min(100, 50 + b9Pct));

  // Button pills
  for (const bName of F310_BUTTONS) {
    const pill = document.getElementById(`btn-pill-${bName}`);
    if (pill) {
      pill.classList.toggle("active", Boolean(controllerState.buttons[bName]));
    }
  }
}

function clearCurrentDevice() {
  document.getElementById("connected-device-text").textContent = "Disconnected (Logitech F310)";
  currentDevice = null;
  controllerState.connected = false;
  for (const k of Object.keys(controllerState.axesNorm)) {
    controllerState.axesNorm[k] = 0;
    controllerState.axesPct[k] = 0;
  }
  for (const b of F310_BUTTONS) {
    controllerState.buttons[b] = false;
    controllerState.prevButtons[b] = false;
  }
  toggleState.shooterActive = false;
  toggleState.spinnerActive = false;
  document.getElementById("forget-btn").setAttribute("disabled", "true");
  clearTable();
  updateControllerVisuals();
}

// ============================================================================
// WebSocket Control Client (/ws/control) & Mapping Evaluation
// ============================================================================
function initWebSocket() {
  const proto = window.location.protocol === "https:" ? "wss:" : "ws:";
  const wsUrl = `${proto}//${window.location.host}/ws/control`;

  ws = new WebSocket(wsUrl);

  ws.onopen = () => {
    wsConnected = true;
    updateWsBadge(true);
  };

  ws.onmessage = (event) => {
    try {
      const msg = JSON.parse(event.data);
      updateStatusFromWs(msg);
    } catch (e) {
      // Ignore malformed messages
    }
  };

  ws.onclose = () => {
    wsConnected = false;
    updateWsBadge(false);
    setTimeout(initWebSocket, 1500);
  };

  ws.onerror = () => {
    ws.close();
  };
}

function sendWsJson(payload) {
  if (ws && wsConnected && ws.readyState === WebSocket.OPEN) {
    ws.send(JSON.stringify(payload));
  }
}

function sendEmergencyStop() {
  sendWsJson({ type: "stop", part: 0 });
  // Also fire REST fallback in case WS is reconnecting
  fetch("/api/stop", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ part: 0, clear_pending_drive: true }),
  }).catch(() => {});
}

function evaluateMappedAxis(axisKey, applySpeedScale = true) {
  const cfg = mappingConfig.axes[axisKey];
  if (!cfg || cfg.source === "none") return 0;

  let norm = controllerState.axesNorm[cfg.source] || 0;
  if (cfg.invert) norm = -norm;

  const dz = (mappingConfig.deadzonePct || 0) / 100;
  if (Math.abs(norm) < dz) return 0;

  // Rescale outside deadzone smoothly to [-1, +1]
  const sign = Math.sign(norm);
  const rescaled = sign * ((Math.abs(norm) - dz) / (1 - dz));
  const scale = applySpeedScale ? (mappingConfig.speedScalePct / 100) : 1.0;

  return Math.round(Math.max(-1000, Math.min(1000, rescaled * scale * 1000)));
}

function isActionHeld(actionId) {
  for (const bName of F310_BUTTONS) {
    if (controllerState.buttons[bName] && mappingConfig.buttons[bName] === actionId) {
      return true;
    }
  }
  return false;
}

function controlTick() {
  // 1. Evaluate Chassis Drive
  const vx = controllerState.connected ? evaluateMappedAxis("vx", true) : 0;
  const vy = controllerState.connected ? evaluateMappedAxis("vy", true) : 0;
  const omega = controllerState.connected ? evaluateMappedAxis("omega", true) : 0;

  // 2. Evaluate Shooter (Flywheel + Pitch Angle Stepper)
  const shootActive =
    controllerState.connected &&
    (toggleState.shooterActive || isActionHeld("shooter_hold"));
  const flywheel = shootActive ? mappingConfig.shooterSpeed : 0;

  let angleRun = false;
  let angleReversed = false;
  if (controllerState.connected) {
    if (isActionHeld("angle_up_hold")) {
      angleRun = true;
      angleReversed = false;
    } else if (isActionHeld("angle_down_hold")) {
      angleRun = true;
      angleReversed = true;
    } else {
      const angleAxis = evaluateMappedAxis("angle", false);
      if (Math.abs(angleAxis) >= 300) {
        angleRun = true;
        angleReversed = angleAxis < 0;
      }
    }
  }

  // 3. Evaluate Conveyor (Trigger + Spinner)
  const triggerHeld = controllerState.connected && isActionHeld("conveyor_trigger_hold");
  const trigger = triggerHeld ? mappingConfig.conveyorSpeed : 0;

  const spinRevHeld = controllerState.connected && isActionHeld("conveyor_spinner_rev_hold");
  const spinnerRun =
    controllerState.connected && (toggleState.spinnerActive || spinRevHeld);
  const spinnerReversed = Boolean(spinRevHeld);

  // Update UI Telemetry Cards
  document.getElementById("cmd-vx").textContent = vx;
  document.getElementById("cmd-vy").textContent = vy;
  document.getElementById("cmd-omega").textContent = omega;
  document.getElementById("cmd-flywheel").textContent = flywheel;
  document.getElementById("cmd-angle").textContent =
    angleRun ? (angleReversed ? "REV" : "FWD") : "OFF";
  document.getElementById("cmd-conveyor").textContent =
    `${trigger} | ${spinnerRun ? (spinnerReversed ? "REV" : "FWD") : "OFF"}`;

  const armedLabel = document.getElementById("control-armed-label");
  if (armedLabel) {
    if (!controlArmed) {
      armedLabel.textContent = "Monitor Only (Disarmed)";
    } else if (!currentMatchState.teleop_allowed) {
      armedLabel.textContent = `Locked (${currentMatchState.mode.toUpperCase()})`;
    } else {
      armedLabel.textContent = "Control Armed (25 Hz)";
    }
  }

  if (!controlArmed || !wsConnected) return;

  // Send drive continuously while connected or when transitioning back to 0
  if (
    controllerState.connected ||
    lastSentCmd.vx !== 0 ||
    lastSentCmd.vy !== 0 ||
    lastSentCmd.omega !== 0
  ) {
    sendWsJson({ type: "drive", vx, vy, omega });
    lastSentCmd.vx = vx;
    lastSentCmd.vy = vy;
    lastSentCmd.omega = omega;
  }

  // Send shooter command when changed or active
  if (
    flywheel !== lastSentCmd.flywheel ||
    angleRun !== lastSentCmd.angle_run ||
    angleReversed !== lastSentCmd.angle_reversed ||
    flywheel !== 0 ||
    angleRun
  ) {
    sendWsJson({
      type: "shooter",
      flywheel,
      angle_run: angleRun,
      angle_reversed: angleReversed,
    });
    lastSentCmd.flywheel = flywheel;
    lastSentCmd.angle_run = angleRun;
    lastSentCmd.angle_reversed = angleReversed;
  }

  // Send conveyor command when changed or active
  if (
    trigger !== lastSentCmd.trigger ||
    spinnerRun !== lastSentCmd.spinner_run ||
    spinnerReversed !== lastSentCmd.spinner_reversed ||
    trigger !== 0 ||
    spinnerRun
  ) {
    sendWsJson({
      type: "conveyor",
      trigger,
      spinner_run: spinnerRun,
      spinner_reversed: spinnerReversed,
    });
    lastSentCmd.trigger = trigger;
    lastSentCmd.spinner_run = spinnerRun;
    lastSentCmd.spinner_reversed = spinnerReversed;
  }
}

// ============================================================================
// Video Stream & Status Polling
// ============================================================================
function initVideoControls() {
  const img = document.getElementById("video-stream-img");
  const overlay = document.getElementById("video-offline-overlay");
  const crosshair = document.getElementById("video-crosshair");
  const crosshairToggle = document.getElementById("crosshair-toggle");
  const sourceSelect = document.getElementById("video-source-mode");

  const getStreamUrl = () => {
    if (sourceSelect.value === "direct") {
      return `http://${window.location.hostname}:8001/stream?t=${Date.now()}`;
    }
    return `/api/video/stream?t=${Date.now()}`;
  };

  const loadStream = () => {
    img.src = getStreamUrl();
  };

  img.addEventListener("load", () => {
    overlay.style.display = "none";
  });

  img.addEventListener("error", () => {
    overlay.style.display = "flex";
  });

  crosshairToggle.addEventListener("change", () => {
    crosshair.style.display = crosshairToggle.checked ? "flex" : "none";
  });

  sourceSelect.addEventListener("change", loadStream);
  document.getElementById("video-reload-btn").addEventListener("click", loadStream);

  loadStream();
}

function initControlSwitches() {
  const armedToggle = document.getElementById("control-armed-toggle");
  const armedLabel = document.getElementById("control-armed-label");

  armedToggle.addEventListener("change", () => {
    controlArmed = armedToggle.checked;
    armedLabel.textContent = controlArmed ? "Control Armed (25 Hz)" : "Monitor Only (Disarmed)";
    if (!controlArmed) {
      sendEmergencyStop();
    }
  });

  document.getElementById("emergency-stop-btn").addEventListener("click", () => {
    toggleState.shooterActive = false;
    toggleState.spinnerActive = false;
    sendEmergencyStop();
  });
}

function initThemeToggle() {
  const btn = document.getElementById("theme-toggle-btn");
  const icon = document.getElementById("theme-toggle-icon");
  btn.addEventListener("click", () => {
    const isDark = document.body.classList.toggle("dark");
    document.body.classList.toggle("light", !isDark);
    icon.textContent = isDark ? "light_mode" : "dark_mode";
  });
}

function updateWsBadge(connected) {
  const chip = document.getElementById("ws-status-chip");
  const icon = document.getElementById("ws-status-icon");
  const text = document.getElementById("ws-status-text");

  chip.classList.toggle("primary", connected);
  icon.textContent = connected ? "wifi" : "sync_problem";
  text.textContent = connected ? "WS: Connected" : "WS: Reconnecting";
}

function updateEstopUI(estopActive) {
  const chip = document.getElementById("estop-status-chip");
  const icon = document.getElementById("estop-status-icon");
  const text = document.getElementById("estop-status-text");
  const banner = document.getElementById("estop-alert-banner");

  chip.classList.toggle("error", Boolean(estopActive));
  icon.textContent = estopActive ? "gpp_bad" : "verified_user";
  text.textContent = estopActive ? "E-STOP LATCHED" : "E-Stop: Normal";
  banner.style.display = estopActive ? "flex" : "none";
}

function updateStatusFromWs(msg) {
  updateEstopUI(msg.estop_active);
  if (typeof msg.tx_packets === "number" && typeof msg.rx_packets === "number") {
    document.getElementById("uart-stats-text").textContent =
      `TX: ${msg.tx_packets} | RX: ${msg.rx_packets}`;
  }
  if (msg.telemetry && typeof msg.telemetry.battery_v === "number") {
    document.getElementById("battery-text").textContent =
      `${msg.telemetry.battery_v.toFixed(2)} V`;
  }
  if (msg.match) {
    updateMatchUI(msg.match);
  }
  if (msg.vision) {
    updateVisionUI(msg.vision);
  }
  if (msg.pose) {
    updatePoseUI(msg.pose);
  }
}

function initStatusPolling() {
  const poll = async () => {
    try {
      const res = await fetch("/api/status");
      if (res.ok) {
        const data = await res.json();
        updateEstopUI(data.estop_active);
        if (data.stats) {
          document.getElementById("uart-stats-text").textContent =
            `TX: ${data.stats.tx_packets} | RX: ${data.stats.rx_packets}`;
        }
        if (data.latest_telemetry && typeof data.latest_telemetry.battery_v === "number") {
          document.getElementById("battery-text").textContent =
            `${data.latest_telemetry.battery_v.toFixed(2)} V`;
        }
        if (data.match) {
          updateMatchUI(data.match);
        }
        if (data.vision) {
          updateVisionUI(data.vision);
        }
        if (data.pose) {
          updatePoseUI(data.pose);
        }
      }
    } catch (e) {
      // Ignore transient poll errors
    }

    try {
      const vres = await fetch("/api/video/state");
      if (vres.ok) {
        const vdata = await vres.json();
        const online = Boolean(vdata.ok && vdata.result && vdata.result.source && vdata.result.source.online);
        const onlineChip = document.getElementById("cam-online-chip");
        const onlineIcon = document.getElementById("cam-online-icon");
        const onlineText = document.getElementById("cam-online-text");

        onlineChip.classList.toggle("primary", online);
        onlineIcon.textContent = online ? "sensors" : "sensors_off";
        onlineText.textContent = online ? "Online" : "Offline";

        if (vdata.result && vdata.result.source) {
          const src = vdata.result.source;
          if (src.resolution) {
            document.getElementById("cam-res-chip").textContent =
              `${src.resolution.width}x${src.resolution.height}`;
          }
          if (typeof src.captured_fps === "number") {
            document.getElementById("cam-fps-chip").textContent = `${src.captured_fps} fps`;
          }
        }
      }
    } catch (e) {
      // Ignore video state poll errors
    }
  };

  poll();
  setInterval(poll, 1500);
}

// ============================================================================
// Match Phase & Transition Management (Auto 1:00 -> AUTO_WAIT -> Tele-Op 2:00)
// ============================================================================
let currentMatchState = {
  mode: "disarmed",
  phase_time_remaining: 0.0,
  phase_time_elapsed: 0.0,
  phase_duration: 0.0,
  auto_duration: 60.0,
  teleop_duration: 120.0,
  teleop_allowed: false,
  status_message: "Robot is disarmed. Enable Tele-Op to drive",
  last_updated: Date.now(),
};

function formatTime(seconds) {
  const s = Math.max(0, Math.ceil(seconds));
  const m = Math.floor(s / 60);
  const remSec = s % 60;
  return `${String(m).padStart(2, "0")}:${String(remSec).padStart(2, "0")}`;
}

function updateMatchUI(match) {
  if (!match) return;
  currentMatchState = {
    ...currentMatchState,
    ...match,
    last_updated: Date.now(),
  };

  const chip = document.getElementById("match-mode-chip");
  const icon = document.getElementById("match-mode-icon");
  const text = document.getElementById("match-mode-text");
  const timerDisplay = document.getElementById("match-timer-display");
  const phaseLabel = document.getElementById("match-phase-label");
  const statusMsg = document.getElementById("match-status-msg");
  const enableTeleopBtn = document.getElementById("match-enable-teleop-btn");

  if (!chip || !icon || !text || !timerDisplay || !phaseLabel || !statusMsg) return;

  // Clear mode specific styling and animations
  chip.className = "match-mode-chip";
  if (enableTeleopBtn) {
    enableTeleopBtn.classList.remove("btn-pulse-teleop");
  }

  switch (match.mode) {
    case "auto":
      chip.classList.add("match-mode-auto");
      icon.textContent = "smart_toy";
      text.textContent = "AUTO (1:00)";
      timerDisplay.textContent = formatTime(match.phase_time_remaining);
      phaseLabel.textContent = "AUTONOMOUS TIME REMAINING";
      statusMsg.textContent = "Autonomous mode active (60s)";
      break;

    case "auto_wait":
      chip.classList.add("match-mode-auto_wait");
      icon.textContent = "warning";
      text.textContent = "AUTO DONE - WAITING FOR DRIVER";
      timerDisplay.textContent = "00:00";
      phaseLabel.textContent = "AWAITING OPERATOR TRIGGER";
      statusMsg.textContent = "Click \"Enable Tele-Op\" to start driver control.";
      if (enableTeleopBtn) {
        enableTeleopBtn.classList.add("btn-pulse-teleop");
      }
      break;

    case "teleop":
      chip.classList.add("match-mode-teleop");
      icon.textContent = "sports_esports";
      text.textContent = "TELE-OP (2:00)";
      timerDisplay.textContent = formatTime(match.phase_time_remaining);
      phaseLabel.textContent = "TELE-OP TIME REMAINING";
      statusMsg.textContent = "Tele-Op active (120s)";
      break;

    case "match_over":
      chip.classList.add("match-mode-match_over");
      icon.textContent = "sports_score";
      text.textContent = "MATCH OVER";
      timerDisplay.textContent = "00:00";
      phaseLabel.textContent = "MATCH CONCLUDED";
      statusMsg.textContent = "Match concluded (3:00 total). Reset match to drive.";
      break;

    case "practice":
      chip.classList.add("match-mode-practice");
      icon.textContent = "model_training";
      text.textContent = "PRACTICE (UNLIMITED)";
      timerDisplay.textContent = formatTime(match.phase_time_elapsed);
      phaseLabel.textContent = "PRACTICE TIME ELAPSED";
      statusMsg.textContent = "Practice mode active (Unlimited Tele-Op without match lockout).";
      break;

    case "disarmed":
    default:
      chip.classList.add("match-mode-disarmed");
      icon.textContent = "lock";
      text.textContent = "DISARMED";
      timerDisplay.textContent = "00:00";
      phaseLabel.textContent = "STANDBY";
      statusMsg.textContent = match.status_message || "Robot is disarmed. Enable Tele-Op or Start Auto to begin.";
      break;
  }
}

function localMatchTimerTick() {
  if (!currentMatchState || !currentMatchState.last_updated) return;
  const elapsedSinceUpdate = (Date.now() - currentMatchState.last_updated) / 1000;
  const timerDisplay = document.getElementById("match-timer-display");
  if (!timerDisplay) return;

  if (currentMatchState.mode === "auto" || currentMatchState.mode === "teleop") {
    const rem = Math.max(0, currentMatchState.phase_time_remaining - elapsedSinceUpdate);
    timerDisplay.textContent = formatTime(rem);
  } else if (currentMatchState.mode === "practice") {
    const el = currentMatchState.phase_time_elapsed + elapsedSinceUpdate;
    timerDisplay.textContent = formatTime(el);
  }
}

function initMatchControls() {
  const postMatchAction = async (endpoint) => {
    try {
      const res = await fetch(endpoint, { method: "POST" });
      if (res.ok) {
        const data = await res.json();
        updateMatchUI(data);
      }
    } catch (e) {
      console.error(`Failed match action ${endpoint}:`, e);
    }
  };

  const autoBtn = document.getElementById("match-start-auto-btn");
  if (autoBtn) {
    autoBtn.addEventListener("click", () => postMatchAction("/api/match/auto/start"));
  }

  const teleopBtn = document.getElementById("match-enable-teleop-btn");
  if (teleopBtn) {
    teleopBtn.addEventListener("click", () => postMatchAction("/api/match/teleop/enable"));
  }

  const practiceBtn = document.getElementById("match-start-practice-btn");
  if (practiceBtn) {
    practiceBtn.addEventListener("click", () => postMatchAction("/api/match/practice/start"));
  }

  const resetBtn = document.getElementById("match-reset-btn");
  if (resetBtn) {
    resetBtn.addEventListener("click", () => postMatchAction("/api/match/reset"));
  }

  // Smooth local timer countdown tick
  setInterval(localMatchTimerTick, 200);
}

// ============================================================================
// Vision Target Tracking & Latching (Special 1x3 Grid Row)
// ============================================================================
let currentVisionState = {
  active_target: "none",
  target_detected: false,
  latched: false,
  timestamp: 0,
  target_coords: null,
};

function updateVisionUI(vision) {
  if (!vision) return;
  currentVisionState = { ...currentVisionState, ...vision };

  const chip = document.getElementById("vision-target-chip");
  const icon = document.getElementById("vision-target-icon");

  if (chip && icon && text) {
    chip.classList.remove("vision-chip-active", "vision-chip-searching");
    if (currentVisionState.latched && currentVisionState.active_target !== "none") {
      chip.classList.add("vision-chip-active");
      icon.textContent = "verified";
    } else {
      chip.classList.add("vision-chip-searching");
      icon.textContent = "center_focus_weak";
    }
  }

  // Redraw field map to reflect target highlighting
  if (typeof drawFieldMap === "function") {
    drawFieldMap(currentPoseState);
  }
}

// ============================================================================
// Live 2D Field Pose Map (400 cm x 500 cm, Origin at Bottom-Left)
// ============================================================================
const FIELD_WIDTH_CM = 400.0;
const FIELD_LENGTH_CM = 500.0;
const poseTrail = []; // Recent (x_cm, y_cm) history
let currentPoseState = null;

function initFieldPoseMap() {
  const canvas = document.getElementById("field-map-canvas");
  if (!canvas) return;

  // Bind Zero Heading & Reset History buttons
  document.getElementById("pose-zero-heading-btn").addEventListener("click", async () => {
    try {
      await fetch("/api/pose/calibrate", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ target_field_deg: 0.0, reset_history: false }),
      });
    } catch (e) {
      console.error("Failed to calibrate heading:", e);
    }
  });

  document.getElementById("pose-reset-btn").addEventListener("click", async () => {
    poseTrail.length = 0;
    currentPoseState = null;
    drawFieldMap(null);
    try {
      await fetch("/api/pose/calibrate", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ reset_history: true }),
      });
    } catch (e) {
      console.error("Failed to reset pose history:", e);
    }
  });

  // Bind Simulator Sliders
  const fSlider = document.getElementById("sim-f-slider");
  const lSlider = document.getElementById("sim-l-slider");
  const rSlider = document.getElementById("sim-r-slider");
  const hSlider = document.getElementById("sim-h-slider");

  const updateSimLabels = () => {
    document.getElementById("sim-f-label").textContent = `${fSlider.value} cm`;
    document.getElementById("sim-l-label").textContent = `${lSlider.value} cm`;
    document.getElementById("sim-r-label").textContent = `${rSlider.value} cm`;
    document.getElementById("sim-h-label").textContent = `${hSlider.value}\u00B0`;
  };

  const sendSimPose = async (f, l, r, h) => {
    try {
      const res = await fetch("/api/pose/update", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({
          dist_front_cm: Number(f),
          dist_left_cm: Number(l),
          dist_right_cm: Number(r),
          raw_heading_deg: Number(h),
        }),
      });
      if (res.ok) {
        const data = await res.json();
        if (data.pose) updatePoseUI(data.pose);
      }
    } catch (e) {
      console.error("Simulator update error:", e);
    }
  };

  for (const s of [fSlider, lSlider, rSlider, hSlider]) {
    s.addEventListener("input", () => {
      updateSimLabels();
      sendSimPose(fSlider.value, lSlider.value, rSlider.value, hSlider.value);
    });
  }

  document.getElementById("sim-send-btn").addEventListener("click", () => {
    sendSimPose(fSlider.value, lSlider.value, rSlider.value, hSlider.value);
  });

  document.getElementById("sim-obstacle-r-btn").addEventListener("click", () => {
    rSlider.value = 45;
    updateSimLabels();
    sendSimPose(fSlider.value, lSlider.value, 45, hSlider.value);
  });

  document.getElementById("sim-obstacle-l-btn").addEventListener("click", () => {
    lSlider.value = 45;
    updateSimLabels();
    sendSimPose(fSlider.value, 45, rSlider.value, hSlider.value);
  });

  // Vision 1x3 active goal simulator buttons
  const sendSimVision = async (target, detected) => {
    try {
      const res = await fetch("/api/vision/target", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({
          active_target: target,
          target_detected: detected,
          timestamp: Date.now() / 1000,
          force: true,
        }),
      });
      if (res.ok) {
        const data = await res.json();
        if (data.vision) updateVisionUI(data.vision);
      }
    } catch (e) {
      console.error("Failed to send sim vision:", e);
    }
  };

  const simTargetLeftBtn = document.getElementById("sim-target-left-btn");
  if (simTargetLeftBtn) simTargetLeftBtn.addEventListener("click", () => sendSimVision("left", true));
  const simTargetCenterBtn = document.getElementById("sim-target-center-btn");
  if (simTargetCenterBtn) simTargetCenterBtn.addEventListener("click", () => sendSimVision("center", true));
  const simTargetRightBtn = document.getElementById("sim-target-right-btn");
  if (simTargetRightBtn) simTargetRightBtn.addEventListener("click", () => sendSimVision("right", true));
  const simTargetResetBtn = document.getElementById("sim-target-reset-btn");
  if (simTargetResetBtn) {
    simTargetResetBtn.addEventListener("click", async () => {
      try {
        const res = await fetch("/api/vision/reset", { method: "POST" });
        if (res.ok) {
          const data = await res.json();
          if (data.vision) updateVisionUI(data.vision);
        }
      } catch (e) {
        console.error("Failed to reset sim vision:", e);
      }
    });
  }

  // Clicking on the 2D field canvas places the simulated robot at that (X, Y)
  canvas.addEventListener("click", (evt) => {
    const rect = canvas.getBoundingClientRect();
    const scaleX = canvas.width / rect.width;
    const scaleY = canvas.height / rect.height;
    const px = (evt.clientX - rect.left) * scaleX;
    const py = (evt.clientY - rect.top) * scaleY;

    const padL = 34;
    const padR = 18;
    const padT = 30;
    const padB = 28;
    const fieldW = canvas.width - padL - padR;
    const fieldH = canvas.height - padT - padB;

    const xCm = Math.round(Math.max(10, Math.min(390, ((px - padL) / fieldW) * FIELD_WIDTH_CM)));
    const yCm = Math.round(Math.max(10, Math.min(490, (1 - (py - padT) / fieldH) * FIELD_LENGTH_CM)));

    fSlider.value = Math.round(FIELD_LENGTH_CM - yCm);
    lSlider.value = xCm;
    rSlider.value = Math.round(FIELD_WIDTH_CM - xCm);
    hSlider.value = 0;
    updateSimLabels();
    sendSimPose(fSlider.value, lSlider.value, rSlider.value, 0);
  });

  drawFieldMap(null);
}

function updatePoseUI(pose) {
  currentPoseState = pose;

  const xText = typeof pose.x_cm === "number" ? `${pose.x_cm.toFixed(1)} cm` : "---.- cm";
  const yText = typeof pose.y_cm === "number" ? `${pose.y_cm.toFixed(1)} cm` : "---.- cm";
  const hText = typeof pose.heading_deg === "number" ? `${pose.heading_deg.toFixed(1)}\u00B0` : "---.-\u00B0";

  document.getElementById("pose-x-val").textContent = xText;
  document.getElementById("pose-y-val").textContent = yText;
  document.getElementById("pose-heading-val").textContent = hText;

  document.getElementById("lidar-front-val").textContent =
    typeof pose.dist_front_cm === "number" ? `${pose.dist_front_cm.toFixed(0)} cm` : "--- cm";
  document.getElementById("lidar-left-val").textContent =
    typeof pose.dist_left_cm === "number" ? `${pose.dist_left_cm.toFixed(0)} cm` : "--- cm";
  document.getElementById("lidar-right-val").textContent =
    typeof pose.dist_right_cm === "number" ? `${pose.dist_right_cm.toFixed(0)} cm` : "--- cm";

  const validL = typeof pose.last_valid_left_cm === "number" ? `${pose.last_valid_left_cm.toFixed(0)}` : "---";
  const validR = typeof pose.last_valid_right_cm === "number" ? `${pose.last_valid_right_cm.toFixed(0)}` : "---";
  document.getElementById("pose-validation-detail").textContent =
    `L/R: ${pose.lr_status} | Front: ${pose.front_status} | Cached L=${validL}cm, R=${validR}cm`;

  const statusChip = document.getElementById("pose-status-chip");
  const statusText = document.getElementById("pose-status-text");
  const isRejected =
    pose.lr_status.includes("rejected") || pose.front_status.includes("rejected");

  statusChip.classList.toggle("primary", Boolean(pose.valid && !isRejected));
  statusChip.classList.toggle("error", Boolean(isRejected));

  if (pose.lr_status === "left_only_rejected_right") {
    statusText.textContent = "Obstacle on Right (Using L)";
  } else if (pose.lr_status === "right_only_rejected_left") {
    statusText.textContent = "Obstacle on Left (Using R)";
  } else if (pose.valid) {
    statusText.textContent = "Pose Locked";
  } else {
    statusText.textContent = "Partial / Uncalibrated";
  }

  if (typeof pose.x_cm === "number" && typeof pose.y_cm === "number") {
    const prev = poseTrail[poseTrail.length - 1];
    if (!prev || Math.hypot(pose.x_cm - prev.x, pose.y_cm - prev.y) >= 1.5) {
      poseTrail.push({ x: pose.x_cm, y: pose.y_cm });
      if (poseTrail.length > 45) poseTrail.shift();
    }
  }

  drawFieldMap(pose);
}

function drawFieldMap(pose) {
  const canvas = document.getElementById("field-map-canvas");
  if (!canvas) return;
  const ctx = canvas.getContext("2d");

  const W = canvas.width;
  const H = canvas.height;
  const padL = 34;
  const padR = 18;
  const padT = 30;
  const padB = 28;
  const fieldW = W - padL - padR;
  const fieldH = H - padT - padB;

  const toPxX = (xCm) => padL + (xCm / FIELD_WIDTH_CM) * fieldW;
  const toPxY = (yCm) => padT + ((FIELD_LENGTH_CM - yCm) / FIELD_LENGTH_CM) * fieldH;

  ctx.clearRect(0, 0, W, H);

  // 1. Field surface background
  ctx.fillStyle = "#111827";
  ctx.fillRect(padL, padT, fieldW, fieldH);

  // 2. Grid lines (every 50 cm, major every 100 cm)
  ctx.textAlign = "center";
  ctx.textBaseline = "middle";
  ctx.font = "10px monospace";

  for (let x = 0; x <= FIELD_WIDTH_CM; x += 50) {
    const px = toPxX(x);
    ctx.beginPath();
    ctx.moveTo(px, padT);
    ctx.lineTo(px, padT + fieldH);
    ctx.strokeStyle = x % 100 === 0 ? "rgba(148, 163, 184, 0.25)" : "rgba(148, 163, 184, 0.1)";
    ctx.lineWidth = 1;
    ctx.stroke();

    if (x % 100 === 0) {
      ctx.fillStyle = "#94a3b8";
      ctx.fillText(`${x}`, px, padT + fieldH + 14);
    }
  }

  ctx.textAlign = "right";
  for (let y = 0; y <= FIELD_LENGTH_CM; y += 50) {
    const py = toPxY(y);
    ctx.beginPath();
    ctx.moveTo(padL, py);
    ctx.lineTo(padL + fieldW, py);
    ctx.strokeStyle = y % 100 === 0 ? "rgba(148, 163, 184, 0.25)" : "rgba(148, 163, 184, 0.1)";
    ctx.lineWidth = 1;
    ctx.stroke();

    if (y % 100 === 0) {
      ctx.fillStyle = "#94a3b8";
      ctx.fillText(`${y}`, padL - 6, py);
    }
  }

  // 3. Top Wall 3x3 Target Grid Banner (y = 500 cm)
  const gridLeftX = toPxX(110);
  const gridRightX = toPxX(290);
  const gridW = gridRightX - gridLeftX;
  ctx.fillStyle = "rgba(245, 158, 11, 0.22)";
  ctx.fillRect(gridLeftX, padT - 18, gridW, 18);
  ctx.strokeStyle = "#f59e0b";
  ctx.lineWidth = 2;
  ctx.strokeRect(gridLeftX, padT - 18, gridW, 18);

  // 3 columns inside the 3x3 target zone
  for (let c = 1; c <= 2; c++) {
    const cx = gridLeftX + (gridW * c) / 3;
    ctx.beginPath();
    ctx.moveTo(cx, padT - 18);
    ctx.lineTo(cx, padT);
    ctx.stroke();
  }
  ctx.fillStyle = "#fbbf24";
  ctx.textAlign = "center";
  ctx.font = "bold 10px monospace";
  ctx.fillText("3x3 TARGET GRID (Y=500)", padL + fieldW / 2, padT - 9);

  // 3b. Special 1x3 Grid Row (y = 500 cm, 60 cm column spacing)
  const specialCols = [
    { key: "left", xCm: 140.0, label: "L (140)" },
    { key: "center", xCm: 200.0, label: "C (200)" },
    { key: "right", xCm: 260.0, label: "R (260)" },
  ];
  const colBoxWCm = 44.0;
  const colBoxHCm = 26.0;
  const halfWCmPx = (colBoxWCm / 2 / FIELD_WIDTH_CM) * fieldW;
  const colHPx = (colBoxHCm / FIELD_LENGTH_CM) * fieldH;

  for (const sc of specialCols) {
    const scx = toPxX(sc.xCm);
    const scy = padT;
    const isActive =
      currentVisionState.latched &&
      currentVisionState.active_target.toLowerCase() === sc.key;

    ctx.save();
    if (isActive) {
      // Glowing highlight for active 4-point goal
      ctx.fillStyle = "rgba(250, 204, 21, 0.4)";
      ctx.fillRect(scx - halfWCmPx, scy, halfWCmPx * 2, colHPx);
      ctx.strokeStyle = "#facc15";
      ctx.lineWidth = 2.5;
      ctx.strokeRect(scx - halfWCmPx, scy, halfWCmPx * 2, colHPx);

      ctx.fillStyle = "#facc15";
      ctx.font = "bold 9px monospace";
      ctx.textAlign = "center";
      ctx.fillText("4 PTS*", scx, scy + colHPx / 2 + 1);
    } else {
      ctx.fillStyle = "rgba(148, 163, 184, 0.08)";
      ctx.fillRect(scx - halfWCmPx, scy, halfWCmPx * 2, colHPx);
      ctx.strokeStyle = "rgba(148, 163, 184, 0.35)";
      ctx.lineWidth = 1;
      ctx.strokeRect(scx - halfWCmPx, scy, halfWCmPx * 2, colHPx);

      ctx.fillStyle = "rgba(148, 163, 184, 0.6)";
      ctx.font = "8px monospace";
      ctx.textAlign = "center";
      ctx.fillText(sc.label, scx, scy + colHPx / 2 + 1);
    }
    ctx.restore();
  }

  // 4. Outer Field Boundary Walls
  ctx.strokeStyle = "#64748b";
  ctx.lineWidth = 2;
  ctx.strokeRect(padL, padT, fieldW, fieldH);

  // 5. Draw Breadcrumb Trail
  if (poseTrail.length > 1) {
    ctx.beginPath();
    for (let i = 0; i < poseTrail.length; i++) {
      const pt = poseTrail[i];
      const tx = toPxX(pt.x);
      const ty = toPxY(pt.y);
      if (i === 0) ctx.moveTo(tx, ty);
      else ctx.lineTo(tx, ty);
    }
    ctx.strokeStyle = "rgba(56, 189, 248, 0.45)";
    ctx.lineWidth = 2;
    ctx.stroke();
  }

  if (!pose || typeof pose.x_cm !== "number" || typeof pose.y_cm !== "number") {
    ctx.fillStyle = "rgba(226, 232, 240, 0.7)";
    ctx.font = "12px monospace";
    ctx.textAlign = "center";
    ctx.fillText("Click map or send sensor data", padL + fieldW / 2, padT + fieldH / 2);
    return;
  }

  const rx = toPxX(pose.x_cm);
  const ry = toPxY(pose.y_cm);
  const headingRad = ((pose.heading_deg || 0) * Math.PI) / 180.0;

  // 6. Draw 3 LiDAR Beams (Front=0, Left=-90, Right=+90)
  const drawBeam = (distCm, relDeg, isRejected, label) => {
    if (typeof distCm !== "number" || distCm <= 0) return;
    const alphaRad = ((pose.heading_deg + relDeg) * Math.PI) / 180.0;
    const endXCm = pose.x_cm + distCm * Math.sin(alphaRad);
    const endYCm = pose.y_cm + distCm * Math.cos(alphaRad);
    const ex = toPxX(endXCm);
    const ey = toPxY(endYCm);

    ctx.save();
    ctx.beginPath();
    ctx.moveTo(rx, ry);
    ctx.lineTo(ex, ey);
    if (isRejected) {
      ctx.setLineDash([5, 4]);
      ctx.strokeStyle = "#ff5252";
      ctx.lineWidth = 2;
    } else {
      ctx.strokeStyle = "rgba(0, 230, 118, 0.85)";
      ctx.lineWidth = 1.8;
    }
    ctx.stroke();

    // Hit point marker
    ctx.beginPath();
    ctx.arc(ex, ey, 4, 0, Math.PI * 2);
    ctx.fillStyle = isRejected ? "#ff5252" : "#00e676";
    ctx.fill();

    if (isRejected) {
      ctx.fillStyle = "#ff8a80";
      ctx.font = "bold 9px monospace";
      ctx.textAlign = "center";
      ctx.fillText(`OBS (${label})`, ex, ey - 8);
    }
    ctx.restore();
  };

  const rejectLeft =
    pose.lr_status === "right_only_rejected_left" || pose.lr_status === "left_rejected_delta";
  const rejectRight =
    pose.lr_status === "left_only_rejected_right" || pose.lr_status === "right_rejected_delta";
  const rejectFront = pose.front_status === "rejected_delta";

  drawBeam(pose.dist_left_cm, -90.0, rejectLeft, "L");
  drawBeam(pose.dist_right_cm, 90.0, rejectRight, "R");
  drawBeam(pose.dist_front_cm, 0.0, rejectFront, "F");

  // 7. Draw Robot Chassis & Heading Arrow
  ctx.save();
  ctx.translate(rx, ry);
  ctx.rotate(headingRad);

  // Robot square chassis (~36x36 cm scaled)
  const halfBox = (18 / FIELD_WIDTH_CM) * fieldW;
  ctx.fillStyle = "rgba(56, 189, 248, 0.28)";
  ctx.strokeStyle = "#38bdf8";
  ctx.lineWidth = 2;
  ctx.beginPath();
  ctx.roundRect(-halfBox, -halfBox, halfBox * 2, halfBox * 2, 4);
  ctx.fill();
  ctx.stroke();

  // Heading arrow pointing toward robot Front (0 deg = Up)
  ctx.beginPath();
  ctx.moveTo(0, -halfBox - 8);
  ctx.lineTo(-6, -4);
  ctx.lineTo(6, -4);
  ctx.closePath();
  ctx.fillStyle = "#38bdf8";
  ctx.fill();
  ctx.restore();

  // Coordinate label next to robot
  ctx.fillStyle = "#f8fafc";
  ctx.font = "bold 10px monospace";
  ctx.textAlign = "left";
  ctx.fillText(
    `(${pose.x_cm.toFixed(0)}, ${pose.y_cm.toFixed(0)})`,
    Math.min(rx + 14, padL + fieldW - 62),
    Math.max(ry - 12, padT + 12)
  );
}


