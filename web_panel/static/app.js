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
  angle: 0,
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
  initButtonGridUI();
  initMappingUI();
  initVideoControls();
  initControlSwitches();
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

  // 2. Evaluate Shooter (Flywheel + Pitch Angle)
  const shootActive =
    controllerState.connected &&
    (toggleState.shooterActive || isActionHeld("shooter_hold"));
  const flywheel = shootActive ? mappingConfig.shooterSpeed : 0;

  let angle = controllerState.connected ? evaluateMappedAxis("angle", false) : 0;
  if (controllerState.connected) {
    if (isActionHeld("angle_up_hold")) angle = 600;
    else if (isActionHeld("angle_down_hold")) angle = -600;
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
  document.getElementById("cmd-angle").textContent = angle;
  document.getElementById("cmd-conveyor").textContent =
    `${trigger} | ${spinnerRun ? (spinnerReversed ? "REV" : "FWD") : "OFF"}`;

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
    angle !== lastSentCmd.angle ||
    flywheel !== 0 ||
    angle !== 0
  ) {
    sendWsJson({ type: "shooter", flywheel, angle });
    lastSentCmd.flywheel = flywheel;
    lastSentCmd.angle = angle;
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

