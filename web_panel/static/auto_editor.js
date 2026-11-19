/**
 * TELBot Dedicated Autonomous Routine Editor (/auto)
 * - JSON routine CRUD backed by /api/auto/routines
 * - Interactive 400x500cm 2D waypoint planner canvas
 * - Step sequence builder & Practice test execution
 */

const FIELD_WIDTH_CM = 400.0;
const FIELD_LENGTH_CM = 500.0;
const WALL_MARGIN_CM = 25.0;
const SPECIAL_GOALS = {
  left: { x: 140.0, y: 500.0, label: "L (140)" },
  center: { x: 200.0, y: 500.0, label: "C (200)" },
  right: { x: 260.0, y: 500.0, label: "R (260)" },
};

const STEP_LABELS = {
  wait_vision: "Wait Vision",
  drive_to: "Drive To",
  aim_at: "Aim At",
  shooter_set: "Shooter Set",
  pitch_step: "Pitch Step",
  feed_balls: "Feed Balls",
  wait: "Wait",
  stop_all: "Stop All",
};

let routinesSummary = [];
let activeRoutineId = "";
let currentRoutine = null;
let selectedStepIndex = -1;
let editorToolMode = "drag"; // "drag" | "add_drive" | "place_start"
let draggingTarget = null; // null | { kind: "start" } | { kind: "step", index: number }
let isDirty = false;
let latestStatusData = null;

function escapeHtml(val) {
  return String(val ?? "")
    .replace(/&/g, "&amp;")
    .replace(/</g, "&lt;")
    .replace(/>/g, "&gt;")
    .replace(/"/g, "&quot;");
}

function clampFieldXY(xCm, yCm, snap5cm = false) {
  let x = Math.max(WALL_MARGIN_CM, Math.min(FIELD_WIDTH_CM - WALL_MARGIN_CM, Number(xCm) || 200));
  let y = Math.max(WALL_MARGIN_CM, Math.min(FIELD_LENGTH_CM - WALL_MARGIN_CM, Number(yCm) || 250));
  if (snap5cm) {
    x = Math.round(x / 5.0) * 5.0;
    y = Math.round(y / 5.0) * 5.0;
  } else {
    x = Math.round(x);
    y = Math.round(y);
  }
  return { x, y };
}

function createDefaultStep(type, index) {
  const stepNum = index + 1;
  const id = `step_${Date.now().toString(36)}_${stepNum}`;
  switch (type) {
    case "wait_vision":
      return {
        id,
        type,
        label: "Wait for YOLO 1x3 Latch",
        timeout_s: 2.0,
        fallback_target: "center",
      };
    case "drive_to":
      return {
        id,
        type,
        label: `Drive Waypoint ${stepNum}`,
        x_cm: 200.0,
        y_cm: 260.0,
        aim_mode: "active_1x3",
        heading_deg: 0.0,
        goal_column: "center",
        stop_on_arrival: true,
        pre_spin_shooter: false,
        shooter_mode: "auto",
        shooter_rpm: 3900.0,
        timeout_s: 5.0,
      };
    case "aim_at":
      return {
        id,
        type,
        label: "Aim at Goal",
        aim_mode: "active_1x3",
        heading_deg: 0.0,
        goal_column: "center",
        stop_on_arrival: true,
        timeout_s: 2.5,
      };
    case "shooter_set":
      return {
        id,
        type,
        label: "Spin Up Flywheels",
        enabled: true,
        mode: "auto",
        target_rpm: 3900.0,
        wait_at_speed: true,
        timeout_s: 2.5,
      };
    case "pitch_step":
      return {
        id,
        type,
        label: "Adjust Pitch Stepper",
        action: "up",
        duration_s: 0.4,
      };
    case "feed_balls":
      return {
        id,
        type,
        label: "Feed Balls",
        trigger_speed: 700,
        spinner_run: true,
        require_at_speed: true,
        at_speed_timeout_s: 1.5,
        ball_count: 2,
        duration_s: 2.5,
      };
    case "wait":
      return {
        id,
        type,
        label: "Wait",
        duration_s: 0.5,
      };
    case "stop_all":
    default:
      return {
        id,
        type: "stop_all",
        label: "Safe Stop",
      };
  }
}

function estimateStepDuration(step) {
  if (!step) return 0.0;
  switch (step.type) {
    case "wait_vision":
    case "drive_to":
    case "aim_at":
      return Number(step.timeout_s) || 0.0;
    case "shooter_set":
      return step.enabled && step.wait_at_speed ? Number(step.timeout_s) || 0.0 : 0.0;
    case "pitch_step":
      return step.action === "disable" ? 0.0 : Number(step.duration_s) || 0.0;
    case "feed_balls":
      return (
        (Number(step.duration_s) || 0.0) +
        (step.require_at_speed ? Number(step.at_speed_timeout_s) || 0.0 : 0.0)
      );
    case "wait":
      return Number(step.duration_s) || 0.0;
    case "stop_all":
    default:
      return 0.0;
  }
}

function estimateRoutineDuration(routine) {
  if (!routine || !Array.isArray(routine.steps)) return 0.0;
  return routine.steps.reduce((acc, s) => acc + estimateStepDuration(s), 0.0);
}

function setDirty(flag) {
  isDirty = Boolean(flag);
  const badge = document.getElementById("editor-dirty-badge");
  const icon = document.getElementById("editor-dirty-icon");
  const text = document.getElementById("editor-dirty-text");
  if (!badge || !icon || !text) return;
  badge.classList.toggle("auto-chip-running", isDirty);
  icon.textContent = isDirty ? "edit" : "check";
  text.textContent = isDirty ? "Unsaved Changes" : "Saved";
}

function setSubtitle(msg) {
  const el = document.getElementById("editor-status-subtitle");
  if (el) el.textContent = msg;
}

function updateDurationBadge() {
  const chip = document.getElementById("routine-est-duration-chip");
  const countEl = document.getElementById("editor-step-count");
  const total = estimateRoutineDuration(currentRoutine);
  if (chip) {
    chip.textContent = `Est: ${total.toFixed(1)}s / 60.0s`;
    chip.classList.toggle("auto-chip-aborted", total > 60.0);
  }
  if (countEl && currentRoutine && Array.isArray(currentRoutine.steps)) {
    countEl.textContent = String(currentRoutine.steps.length);
  }
}

// ============================================================================
// Initialization
// ============================================================================
document.addEventListener("DOMContentLoaded", async () => {
  initThemeToggle();
  initToolbarButtons();
  initMetadataInputs();
  initCanvasInteraction();
  await fetchRoutinesList(true);
  startStatusPolling();
});

function initThemeToggle() {
  const btn = document.getElementById("editor-theme-toggle-btn");
  const icon = document.getElementById("editor-theme-toggle-icon");
  if (!btn || !icon) return;
  btn.addEventListener("click", () => {
    const isDark = document.body.classList.toggle("dark");
    document.body.classList.toggle("light", !isDark);
    icon.textContent = isDark ? "light_mode" : "dark_mode";
  });
}

function initToolbarButtons() {
  document.getElementById("editor-routine-select")?.addEventListener("change", async (e) => {
    const rid = e.target.value;
    if (!rid) return;
    if (isDirty && !confirm("Discard unsaved changes to current routine?")) {
      renderRoutineSelector();
      return;
    }
    await loadRoutine(rid);
  });

  document.getElementById("editor-new-btn")?.addEventListener("click", () => {
    if (isDirty && !confirm("Discard unsaved changes?")) return;
    const slug = `routine_${Date.now().toString(36).slice(-5)}`;
    currentRoutine = {
      id: slug,
      name: "New Auto Routine",
      description: "",
      start_pose: { x_cm: 200, y_cm: 80, heading_deg: 0 },
      steps: [
        createDefaultStep("wait_vision", 0),
        createDefaultStep("drive_to", 1),
        createDefaultStep("feed_balls", 2),
        createDefaultStep("stop_all", 3),
      ],
    };
    selectedStepIndex = 1;
    setDirty(true);
    renderAll();
    setSubtitle("Created new unsaved routine. Click Save to persist.");
  });

  document.getElementById("editor-dup-btn")?.addEventListener("click", () => {
    if (!currentRoutine) return;
    const clone = JSON.parse(JSON.stringify(currentRoutine));
    clone.id = `${currentRoutine.id}_copy`.slice(0, 64);
    clone.name = `${currentRoutine.name} (Copy)`.slice(0, 100);
    currentRoutine = clone;
    setDirty(true);
    renderAll();
    setSubtitle("Duplicated routine. Edit ID/name and click Save.");
  });

  document.getElementById("editor-save-btn")?.addEventListener("click", () => saveCurrentRoutine(false));

  document.getElementById("editor-set-active-btn")?.addEventListener("click", async () => {
    if (!currentRoutine) return;
    if (isDirty) {
      const ok = await saveCurrentRoutine(true);
      if (!ok) return;
    } else {
      await setActiveRoutineOnServer(currentRoutine.id);
    }
  });

  document.getElementById("editor-delete-btn")?.addEventListener("click", async () => {
    if (!currentRoutine) return;
    if (!confirm(`Delete routine '${currentRoutine.name}' (${currentRoutine.id})?`)) return;
    try {
      const res = await fetch(`/api/auto/routines/${encodeURIComponent(currentRoutine.id)}`, {
        method: "DELETE",
      });
      const data = await res.json();
      if (!res.ok) {
        alert(data.detail || `Delete failed (${res.status})`);
        return;
      }
      setDirty(false);
      setSubtitle(`Deleted routine '${currentRoutine.id}'.`);
      await fetchRoutinesList(true);
    } catch (e) {
      console.error("Failed to delete routine:", e);
    }
  });

  document.getElementById("editor-export-btn")?.addEventListener("click", () => {
    if (!currentRoutine) return;
    const blob = new Blob([JSON.stringify(currentRoutine, null, 2) + "\n"], {
      type: "application/json",
    });
    const url = URL.createObjectURL(blob);
    const a = document.createElement("a");
    a.href = url;
    a.download = `${currentRoutine.id || "routine"}.json`;
    a.click();
    URL.revokeObjectURL(url);
  });

  const importFileInput = document.getElementById("editor-import-file");
  document.getElementById("editor-import-btn")?.addEventListener("click", () => {
    importFileInput?.click();
  });
  importFileInput?.addEventListener("change", async (e) => {
    const file = e.target.files?.[0];
    if (!file) return;
    try {
      const text = await file.text();
      const parsed = JSON.parse(text);
      if (!parsed || typeof parsed !== "object" || !Array.isArray(parsed.steps)) {
        alert("Invalid routine JSON file (expected object with 'steps' array).");
        return;
      }
      currentRoutine = {
        id: String(parsed.id || file.name.replace(/\.json$/i, "")).replace(/[^a-zA-Z0-9_-]/g, "_"),
        name: String(parsed.name || "Imported Routine"),
        description: String(parsed.description || ""),
        start_pose: parsed.start_pose || { x_cm: 200, y_cm: 80, heading_deg: 0 },
        steps: parsed.steps,
      };
      selectedStepIndex = currentRoutine.steps.length > 0 ? 0 : -1;
      setDirty(true);
      renderAll();
      setSubtitle(`Imported '${file.name}'. Click Save to store on Host.`);
    } catch (err) {
      alert(`Failed to import JSON: ${err}`);
    } finally {
      e.target.value = "";
    }
  });

  document.getElementById("editor-test-run-btn")?.addEventListener("click", async () => {
    if (!currentRoutine) return;
    if (isDirty) {
      const saved = await saveCurrentRoutine(false);
      if (!saved) return;
    }
    try {
      const mode = latestStatusData?.match?.mode;
      if (mode !== "practice" && mode !== "auto") {
        await fetch("/api/match/practice/start", { method: "POST" });
      }
      const res = await fetch("/api/auto/run", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ routine_id: currentRoutine.id }),
      });
      const data = await res.json();
      if (!res.ok) {
        alert(data.detail || `Failed to run routine (${res.status})`);
        return;
      }
      if (data.auto_routine) {
        updateRunnerUI(data.auto_routine);
      }
      setSubtitle(`Running '${currentRoutine.name}'...`);
    } catch (e) {
      console.error("Test run failed:", e);
    }
  });

  document.getElementById("editor-abort-run-btn")?.addEventListener("click", async () => {
    try {
      const res = await fetch("/api/auto/abort", { method: "POST" });
      if (res.ok) {
        const data = await res.json();
        if (data.auto_routine) updateRunnerUI(data.auto_routine);
      }
    } catch (e) {
      console.error("Abort failed:", e);
    }
  });

  document.getElementById("editor-stop-all-btn")?.addEventListener("click", async () => {
    try {
      await fetch("/api/stop", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ part: 0, clear_pending_drive: true }),
      });
    } catch (e) {
      console.error("STOP ALL failed:", e);
    }
  });

  // Quick-add step buttons
  document.querySelectorAll("[data-add-step]").forEach((btn) => {
    btn.addEventListener("click", () => {
      if (!currentRoutine) return;
      const stepType = btn.getAttribute("data-add-step");
      const newStep = createDefaultStep(stepType, currentRoutine.steps.length);
      currentRoutine.steps.push(newStep);
      selectedStepIndex = currentRoutine.steps.length - 1;
      setDirty(true);
      renderStepsList();
      updateDurationBadge();
      drawEditorCanvas();
    });
  });

  // Tool mode buttons
  const setTool = (mode) => {
    editorToolMode = mode;
    const mapping = [
      ["tool-select-drag-btn", "drag"],
      ["tool-add-drive-btn", "add_drive"],
      ["tool-place-start-btn", "place_start"],
    ];
    for (const [id, m] of mapping) {
      const el = document.getElementById(id);
      if (!el) continue;
      el.classList.toggle("primary", editorToolMode === m);
      el.classList.toggle("border", editorToolMode !== m);
    }
  };
  document.getElementById("tool-select-drag-btn")?.addEventListener("click", () => setTool("drag"));
  document.getElementById("tool-add-drive-btn")?.addEventListener("click", () => setTool("add_drive"));
  document.getElementById("tool-place-start-btn")?.addEventListener("click", () => setTool("place_start"));
  document.getElementById("editor-show-live-robot-toggle")?.addEventListener("change", drawEditorCanvas);
}

function initMetadataInputs() {
  const bindText = (id, field) => {
    const el = document.getElementById(id);
    if (!el) return;
    el.addEventListener("input", () => {
      if (!currentRoutine) return;
      currentRoutine[field] = el.value;
      setDirty(true);
    });
  };
  bindText("routine-name-input", "name");
  bindText("routine-id-input", "id");
  bindText("routine-desc-input", "description");

  const bindStart = (id, key) => {
    const el = document.getElementById(id);
    if (!el) return;
    el.addEventListener("input", () => {
      if (!currentRoutine) return;
      currentRoutine.start_pose = currentRoutine.start_pose || { x_cm: 200, y_cm: 80, heading_deg: 0 };
      currentRoutine.start_pose[key] = Number(el.value);
      setDirty(true);
      drawEditorCanvas();
    });
  };
  bindStart("routine-start-x", "x_cm");
  bindStart("routine-start-y", "y_cm");
  bindStart("routine-start-heading", "heading_deg");
}

// ============================================================================
// REST Persistence
// ============================================================================
async function fetchRoutinesList(loadSelected = false) {
  try {
    const res = await fetch("/api/auto/routines");
    if (!res.ok) return;
    const data = await res.json();
    routinesSummary = Array.isArray(data.routines) ? data.routines : [];
    activeRoutineId = data.active_routine_id || "";
    renderRoutineSelector();
    if (data.runner) updateRunnerUI(data.runner);
    if (loadSelected && activeRoutineId) {
      await loadRoutine(activeRoutineId);
    }
  } catch (e) {
    console.error("Failed to list routines:", e);
  }
}

async function loadRoutine(routineId) {
  try {
    const res = await fetch(`/api/auto/routines/${encodeURIComponent(routineId)}`);
    if (!res.ok) return;
    const data = await res.json();
    currentRoutine = data.routine;
    selectedStepIndex = currentRoutine.steps.length > 0 ? 0 : -1;
    setDirty(false);
    renderAll();
    setSubtitle(`Loaded '${currentRoutine.name}' (${currentRoutine.id}.json)`);
  } catch (e) {
    console.error("Failed to load routine:", e);
  }
}

async function saveCurrentRoutine(alsoSetActive = false) {
  if (!currentRoutine) return false;
  try {
    const payload = { ...currentRoutine, set_active: Boolean(alsoSetActive) };
    const res = await fetch("/api/auto/routines", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(payload),
    });
    const data = await res.json();
    if (!res.ok) {
      alert(data.detail || `Failed to save routine (${res.status})`);
      return false;
    }
    currentRoutine = data.routine;
    activeRoutineId = data.active_routine_id || activeRoutineId;
    routinesSummary = Array.isArray(data.routines) ? data.routines : routinesSummary;
    setDirty(false);
    renderAll();
    setSubtitle(
      alsoSetActive
        ? `Saved and activated '${currentRoutine.name}' for 1:00 Auto.`
        : `Saved '${currentRoutine.id}.json'.`
    );
    return true;
  } catch (e) {
    console.error("Failed to save routine:", e);
    return false;
  }
}

async function setActiveRoutineOnServer(routineId) {
  try {
    const res = await fetch("/api/auto/active", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ routine_id: routineId }),
    });
    const data = await res.json();
    if (!res.ok) {
      alert(data.detail || `Failed to set active routine (${res.status})`);
      return;
    }
    activeRoutineId = data.active_routine_id;
    await fetchRoutinesList(false);
    setSubtitle(`Active Auto routine set to '${routineId}'.`);
  } catch (e) {
    console.error("Failed to set active routine:", e);
  }
}

// ============================================================================
// Rendering Metadata, Selector, and Steps
// ============================================================================
function renderAll() {
  renderRoutineSelector();
  renderMetadataInputs();
  renderStepsList();
  updateDurationBadge();
  drawEditorCanvas();
}

function renderRoutineSelector() {
  const select = document.getElementById("editor-routine-select");
  if (!select) return;
  select.innerHTML = "";
  const seen = new Set();
  for (const r of routinesSummary) {
    seen.add(r.id);
    const opt = document.createElement("option");
    opt.value = r.id;
    opt.textContent = r.id === activeRoutineId ? `★ ${r.name} (Active)` : r.name;
    if (currentRoutine && r.id === currentRoutine.id) opt.selected = true;
    select.appendChild(opt);
  }
  if (currentRoutine && !seen.has(currentRoutine.id)) {
    const opt = document.createElement("option");
    opt.value = currentRoutine.id;
    opt.textContent = `* ${currentRoutine.name} (Unsaved)`;
    opt.selected = true;
    select.appendChild(opt);
  }
}

function renderMetadataInputs() {
  if (!currentRoutine) return;
  const sp = currentRoutine.start_pose || { x_cm: 200, y_cm: 80, heading_deg: 0 };
  document.getElementById("routine-name-input").value = currentRoutine.name || "";
  document.getElementById("routine-id-input").value = currentRoutine.id || "";
  document.getElementById("routine-desc-input").value = currentRoutine.description || "";
  document.getElementById("routine-start-x").value = sp.x_cm ?? 200;
  document.getElementById("routine-start-y").value = sp.y_cm ?? 80;
  document.getElementById("routine-start-heading").value = sp.heading_deg ?? 0;
}

function buildStepFieldsHtml(step) {
  const t = step.type;
  if (t === "wait_vision") {
    return `
      <div class="grid" style="margin-top: 6px;">
        <div class="s6">
          <label class="small-text">Timeout (s)</label>
          <div class="field small border round no-margin">
            <input type="number" data-field="timeout_s" min="0.1" max="15" step="0.1" value="${Number(step.timeout_s ?? 2.0)}" />
          </div>
        </div>
        <div class="s6">
          <label class="small-text">Fallback 1x3 Column</label>
          <div class="field small border round no-margin">
            <select data-field="fallback_target">
              <option value="left" ${step.fallback_target === "left" ? "selected" : ""}>Left (140)</option>
              <option value="center" ${step.fallback_target === "center" ? "selected" : ""}>Center (200)</option>
              <option value="right" ${step.fallback_target === "right" ? "selected" : ""}>Right (260)</option>
            </select>
            <i>arrow_drop_down</i>
          </div>
        </div>
      </div>`;
  }
  if (t === "drive_to") {
    return `
      <div class="grid" style="margin-top: 6px;">
        <div class="s4">
          <label class="small-text">X (cm)</label>
          <div class="field small border round no-margin">
            <input type="number" data-field="x_cm" min="25" max="375" step="5" value="${Number(step.x_cm ?? 200)}" />
          </div>
        </div>
        <div class="s4">
          <label class="small-text">Y (cm)</label>
          <div class="field small border round no-margin">
            <input type="number" data-field="y_cm" min="25" max="475" step="5" value="${Number(step.y_cm ?? 260)}" />
          </div>
        </div>
        <div class="s4">
          <label class="small-text">Timeout (s)</label>
          <div class="field small border round no-margin">
            <input type="number" data-field="timeout_s" min="0.2" max="30" step="0.5" value="${Number(step.timeout_s ?? 5.0)}" />
          </div>
        </div>
        <div class="s4">
          <label class="small-text">Aim Mode</label>
          <div class="field small border round no-margin">
            <select data-field="aim_mode">
              <option value="active_1x3" ${step.aim_mode === "active_1x3" ? "selected" : ""}>Active 1x3 Goal</option>
              <option value="column" ${step.aim_mode === "column" ? "selected" : ""}>Fixed Column</option>
              <option value="explicit" ${step.aim_mode === "explicit" ? "selected" : ""}>Explicit Heading</option>
            </select>
            <i>arrow_drop_down</i>
          </div>
        </div>
        <div class="s4">
          <label class="small-text">Heading (&deg;)</label>
          <div class="field small border round no-margin">
            <input type="number" data-field="heading_deg" min="0" max="359" step="5" value="${Number(step.heading_deg ?? 0)}" ${step.aim_mode !== "explicit" ? "disabled" : ""} />
          </div>
        </div>
        <div class="s4">
          <label class="small-text">Goal Column</label>
          <div class="field small border round no-margin">
            <select data-field="goal_column" ${step.aim_mode !== "column" ? "disabled" : ""}>
              <option value="left" ${step.goal_column === "left" ? "selected" : ""}>Left (140)</option>
              <option value="center" ${step.goal_column === "center" ? "selected" : ""}>Center (200)</option>
              <option value="right" ${step.goal_column === "right" ? "selected" : ""}>Right (260)</option>
            </select>
            <i>arrow_drop_down</i>
          </div>
        </div>
        <div class="s6">
          <label class="checkbox small-text">
            <input type="checkbox" data-field="stop_on_arrival" ${step.stop_on_arrival !== false ? "checked" : ""} />
            <span>Stop on Arrival</span>
          </label>
        </div>
        <div class="s6">
          <label class="checkbox small-text">
            <input type="checkbox" data-field="pre_spin_shooter" ${step.pre_spin_shooter ? "checked" : ""} />
            <span>Pre-Spin Shooter</span>
          </label>
        </div>
        ${
          step.pre_spin_shooter
            ? `<div class="s6">
                <label class="small-text">Shooter Mode</label>
                <div class="field small border round no-margin">
                  <select data-field="shooter_mode">
                    <option value="auto" ${step.shooter_mode === "auto" ? "selected" : ""}>Auto (Distance)</option>
                    <option value="manual" ${step.shooter_mode === "manual" ? "selected" : ""}>Manual RPM</option>
                  </select>
                  <i>arrow_drop_down</i>
                </div>
              </div>
              <div class="s6">
                <label class="small-text">Manual/Fallback RPM</label>
                <div class="field small border round no-margin">
                  <input type="number" data-field="shooter_rpm" min="0" max="5500" step="50" value="${Number(step.shooter_rpm ?? 3900)}" />
                </div>
              </div>`
            : ""
        }
      </div>`;
  }
  if (t === "aim_at") {
    return `
      <div class="grid" style="margin-top: 6px;">
        <div class="s4">
          <label class="small-text">Aim Mode</label>
          <div class="field small border round no-margin">
            <select data-field="aim_mode">
              <option value="active_1x3" ${step.aim_mode === "active_1x3" ? "selected" : ""}>Active 1x3 Goal</option>
              <option value="column" ${step.aim_mode === "column" ? "selected" : ""}>Fixed Column</option>
              <option value="explicit" ${step.aim_mode === "explicit" ? "selected" : ""}>Explicit Heading</option>
            </select>
            <i>arrow_drop_down</i>
          </div>
        </div>
        <div class="s4">
          <label class="small-text">Heading (&deg;)</label>
          <div class="field small border round no-margin">
            <input type="number" data-field="heading_deg" min="0" max="359" step="5" value="${Number(step.heading_deg ?? 0)}" ${step.aim_mode !== "explicit" ? "disabled" : ""} />
          </div>
        </div>
        <div class="s4">
          <label class="small-text">Goal Column</label>
          <div class="field small border round no-margin">
            <select data-field="goal_column" ${step.aim_mode !== "column" ? "disabled" : ""}>
              <option value="left" ${step.goal_column === "left" ? "selected" : ""}>Left (140)</option>
              <option value="center" ${step.goal_column === "center" ? "selected" : ""}>Center (200)</option>
              <option value="right" ${step.goal_column === "right" ? "selected" : ""}>Right (260)</option>
            </select>
            <i>arrow_drop_down</i>
          </div>
        </div>
        <div class="s6">
          <label class="small-text">Timeout (s)</label>
          <div class="field small border round no-margin">
            <input type="number" data-field="timeout_s" min="0.2" max="15" step="0.1" value="${Number(step.timeout_s ?? 2.5)}" />
          </div>
        </div>
        <div class="s6 middle-align">
          <label class="checkbox small-text">
            <input type="checkbox" data-field="stop_on_arrival" ${step.stop_on_arrival !== false ? "checked" : ""} />
            <span>Stop on Arrival</span>
          </label>
        </div>
      </div>`;
  }
  if (t === "shooter_set") {
    return `
      <div class="grid" style="margin-top: 6px;">
        <div class="s4">
          <label class="checkbox small-text">
            <input type="checkbox" data-field="enabled" ${step.enabled !== false ? "checked" : ""} />
            <span>Enabled</span>
          </label>
        </div>
        <div class="s4">
          <label class="checkbox small-text">
            <input type="checkbox" data-field="wait_at_speed" ${step.wait_at_speed !== false ? "checked" : ""} />
            <span>Wait At Speed</span>
          </label>
        </div>
        <div class="s4">
          <label class="small-text">Timeout (s)</label>
          <div class="field small border round no-margin">
            <input type="number" data-field="timeout_s" min="0.1" max="15" step="0.1" value="${Number(step.timeout_s ?? 2.5)}" />
          </div>
        </div>
        <div class="s6">
          <label class="small-text">Speed Mode</label>
          <div class="field small border round no-margin">
            <select data-field="mode">
              <option value="auto" ${step.mode === "auto" ? "selected" : ""}>Auto (Distance)</option>
              <option value="manual" ${step.mode === "manual" ? "selected" : ""}>Manual RPM</option>
            </select>
            <i>arrow_drop_down</i>
          </div>
        </div>
        <div class="s6">
          <label class="small-text">Manual/Fallback RPM</label>
          <div class="field small border round no-margin">
            <input type="number" data-field="target_rpm" min="0" max="5500" step="50" value="${Number(step.target_rpm ?? 3900)}" />
          </div>
        </div>
      </div>`;
  }
  if (t === "pitch_step") {
    return `
      <div class="grid" style="margin-top: 6px;">
        <div class="s6">
          <label class="small-text">Action</label>
          <div class="field small border round no-margin">
            <select data-field="action">
              <option value="up" ${step.action === "up" ? "selected" : ""}>Pitch Up (FWD)</option>
              <option value="down" ${step.action === "down" ? "selected" : ""}>Pitch Down (REV)</option>
              <option value="disable" ${step.action === "disable" ? "selected" : ""}>Disable Stepper</option>
            </select>
            <i>arrow_drop_down</i>
          </div>
        </div>
        <div class="s6">
          <label class="small-text">Pulse Duration (s)</label>
          <div class="field small border round no-margin">
            <input type="number" data-field="duration_s" min="0.05" max="5" step="0.05" value="${Number(step.duration_s ?? 0.4)}" ${step.action === "disable" ? "disabled" : ""} />
          </div>
        </div>
      </div>`;
  }
  if (t === "feed_balls") {
    return `
      <div class="grid" style="margin-top: 6px;">
        <div class="s4">
          <label class="small-text">Trigger Speed</label>
          <div class="field small border round no-margin">
            <input type="number" data-field="trigger_speed" min="-1000" max="1000" step="50" value="${Number(step.trigger_speed ?? 700)}" />
          </div>
        </div>
        <div class="s4">
          <label class="small-text">Ball Count (0=Time)</label>
          <div class="field small border round no-margin">
            <input type="number" data-field="ball_count" min="0" max="10" step="1" value="${Number(step.ball_count ?? 2)}" />
          </div>
        </div>
        <div class="s4">
          <label class="small-text">Max Feed Dur (s)</label>
          <div class="field small border round no-margin">
            <input type="number" data-field="duration_s" min="0.1" max="15" step="0.1" value="${Number(step.duration_s ?? 2.5)}" />
          </div>
        </div>
        <div class="s4">
          <label class="checkbox small-text">
            <input type="checkbox" data-field="spinner_run" ${step.spinner_run !== false ? "checked" : ""} />
            <span>Run Spinner</span>
          </label>
        </div>
        <div class="s4">
          <label class="checkbox small-text">
            <input type="checkbox" data-field="require_at_speed" ${step.require_at_speed !== false ? "checked" : ""} />
            <span>Wait At Speed</span>
          </label>
        </div>
        <div class="s4">
          <label class="small-text">Speed Wait (s)</label>
          <div class="field small border round no-margin">
            <input type="number" data-field="at_speed_timeout_s" min="0" max="10" step="0.1" value="${Number(step.at_speed_timeout_s ?? 1.5)}" />
          </div>
        </div>
      </div>`;
  }
  if (t === "wait") {
    return `
      <div class="grid" style="margin-top: 6px;">
        <div class="s6">
          <label class="small-text">Wait Duration (s)</label>
          <div class="field small border round no-margin">
            <input type="number" data-field="duration_s" min="0.05" max="30" step="0.1" value="${Number(step.duration_s ?? 0.5)}" />
          </div>
        </div>
      </div>`;
  }
  return `<div class="small-text mono" style="opacity: 0.7; margin-top: 4px;">Halts chassis, flywheels, pitch stepper, and conveyor.</div>`;
}

function renderStepsList() {
  const container = document.getElementById("editor-steps-container");
  if (!container || !currentRoutine) return;
  container.innerHTML = "";

  const runner = latestStatusData?.auto_routine;
  const activeStepIdx = runner && runner.running ? Number(runner.current_step_index) : -1;

  currentRoutine.steps.forEach((step, idx) => {
    const card = document.createElement("div");
    card.className = "editor-step-card";
    if (idx === selectedStepIndex) card.classList.add("step-selected");
    if (idx === activeStepIdx) card.classList.add("step-running");

    const typeLabel = STEP_LABELS[step.type] || step.type;
    card.innerHTML = `
      <div class="row space-between middle-align" style="gap: 6px;">
        <div class="row middle-align" style="gap: 8px; flex: 1; min-width: 0;">
          <span class="mono bold">#${idx + 1}</span>
          <span class="step-type-badge">${escapeHtml(typeLabel)}</span>
          <div class="field small border round no-margin" style="flex: 1; min-width: 120px;">
            <input type="text" data-field="label" value="${escapeHtml(step.label || "")}" placeholder="Step label..." />
          </div>
        </div>
        <div class="row no-space">
          <button class="button small transparent circle" data-step-act="up" title="Move Up" ${idx === 0 ? "disabled" : ""}><i>arrow_upward</i></button>
          <button class="button small transparent circle" data-step-act="down" title="Move Down" ${idx === currentRoutine.steps.length - 1 ? "disabled" : ""}><i>arrow_downward</i></button>
          <button class="button small transparent circle" data-step-act="dup" title="Duplicate"><i>content_copy</i></button>
          <button class="button small transparent circle" data-step-act="del" title="Delete"><i>delete</i></button>
        </div>
      </div>
      ${buildStepFieldsHtml(step)}
    `;

    card.addEventListener("click", (e) => {
      if (e.target.closest("button")) return;
      if (selectedStepIndex !== idx) {
        selectedStepIndex = idx;
        renderStepsList();
        drawEditorCanvas();
      }
    });

    card.querySelectorAll("[data-step-act]").forEach((btn) => {
      btn.addEventListener("click", (e) => {
        e.stopPropagation();
        const act = btn.getAttribute("data-step-act");
        if (act === "up" && idx > 0) {
          const tmp = currentRoutine.steps[idx - 1];
          currentRoutine.steps[idx - 1] = currentRoutine.steps[idx];
          currentRoutine.steps[idx] = tmp;
          selectedStepIndex = idx - 1;
        } else if (act === "down" && idx < currentRoutine.steps.length - 1) {
          const tmp = currentRoutine.steps[idx + 1];
          currentRoutine.steps[idx + 1] = currentRoutine.steps[idx];
          currentRoutine.steps[idx] = tmp;
          selectedStepIndex = idx + 1;
        } else if (act === "dup") {
          const copy = JSON.parse(JSON.stringify(step));
          copy.id = `step_${Date.now().toString(36)}_${currentRoutine.steps.length + 1}`;
          currentRoutine.steps.splice(idx + 1, 0, copy);
          selectedStepIndex = idx + 1;
        } else if (act === "del") {
          currentRoutine.steps.splice(idx, 1);
          if (selectedStepIndex >= currentRoutine.steps.length) {
            selectedStepIndex = currentRoutine.steps.length - 1;
          }
        }
        setDirty(true);
        renderStepsList();
        updateDurationBadge();
        drawEditorCanvas();
      });
    });

    card.querySelectorAll("[data-field]").forEach((input) => {
      const evtName = input.tagName === "SELECT" || input.type === "checkbox" ? "change" : "input";
      input.addEventListener(evtName, () => {
        const field = input.getAttribute("data-field");
        if (input.type === "checkbox") {
          step[field] = Boolean(input.checked);
        } else if (input.type === "number") {
          step[field] = Number(input.value);
        } else {
          step[field] = input.value;
        }
        setDirty(true);
        updateDurationBadge();
        // Re-render card conditional inputs when mode/toggle fields change
        if (["aim_mode", "pre_spin_shooter", "action"].includes(field)) {
          renderStepsList();
        }
        drawEditorCanvas();
      });
    });

    container.appendChild(card);
  });
}

// ============================================================================
// Interactive 2D Canvas Field Planner
// ============================================================================
function getCanvasGeometry(canvas) {
  const padL = 34;
  const padR = 18;
  const padT = 30;
  const padB = 28;
  const fieldW = canvas.width - padL - padR;
  const fieldH = canvas.height - padT - padB;
  return {
    padL,
    padR,
    padT,
    padB,
    fieldW,
    fieldH,
    toPxX: (xCm) => padL + (xCm / FIELD_WIDTH_CM) * fieldW,
    toPxY: (yCm) => padT + ((FIELD_LENGTH_CM - yCm) / FIELD_LENGTH_CM) * fieldH,
    toFieldX: (px) => ((px - padL) / fieldW) * FIELD_WIDTH_CM,
    toFieldY: (py) => (1 - (py - padT) / fieldH) * FIELD_LENGTH_CM,
  };
}

function initCanvasInteraction() {
  const canvas = document.getElementById("auto-editor-canvas");
  if (!canvas) return;
  const coordsLabel = document.getElementById("editor-canvas-coords");

  const getPointerFieldPos = (evt) => {
    const rect = canvas.getBoundingClientRect();
    const px = ((evt.clientX - rect.left) * canvas.width) / rect.width;
    const py = ((evt.clientY - rect.top) * canvas.height) / rect.height;
    const g = getCanvasGeometry(canvas);
    const snap = Boolean(document.getElementById("editor-snap-5cm-toggle")?.checked);
    const clamped = clampFieldXY(g.toFieldX(px), g.toFieldY(py), snap);
    return { px, py, xCm: clamped.x, yCm: clamped.y, g };
  };

  canvas.addEventListener("pointerdown", (evt) => {
    if (!currentRoutine) return;
    const { px, py, xCm, yCm, g } = getPointerFieldPos(evt);

    if (editorToolMode === "place_start") {
      currentRoutine.start_pose = {
        ...(currentRoutine.start_pose || {}),
        x_cm: xCm,
        y_cm: yCm,
      };
      setDirty(true);
      renderMetadataInputs();
      drawEditorCanvas();
      return;
    }

    if (editorToolMode === "add_drive") {
      const step = createDefaultStep("drive_to", currentRoutine.steps.length);
      step.x_cm = xCm;
      step.y_cm = yCm;
      currentRoutine.steps.push(step);
      selectedStepIndex = currentRoutine.steps.length - 1;
      setDirty(true);
      renderStepsList();
      updateDurationBadge();
      drawEditorCanvas();
      return;
    }

    // Drag mode: hit-test waypoints (steps first, then start pose)
    for (let i = currentRoutine.steps.length - 1; i >= 0; i--) {
      const st = currentRoutine.steps[i];
      if (st.type !== "drive_to") continue;
      const distPx = Math.hypot(g.toPxX(st.x_cm) - px, g.toPxY(st.y_cm) - py);
      if (distPx <= 14) {
        draggingTarget = { kind: "step", index: i };
        selectedStepIndex = i;
        canvas.setPointerCapture(evt.pointerId);
        renderStepsList();
        drawEditorCanvas();
        return;
      }
    }

    const sp = currentRoutine.start_pose;
    if (sp && Math.hypot(g.toPxX(sp.x_cm) - px, g.toPxY(sp.y_cm) - py) <= 14) {
      draggingTarget = { kind: "start" };
      canvas.setPointerCapture(evt.pointerId);
      return;
    }

    // Clicking empty canvas while a drive_to step is selected moves that waypoint
    if (
      selectedStepIndex >= 0 &&
      currentRoutine.steps[selectedStepIndex] &&
      currentRoutine.steps[selectedStepIndex].type === "drive_to"
    ) {
      currentRoutine.steps[selectedStepIndex].x_cm = xCm;
      currentRoutine.steps[selectedStepIndex].y_cm = yCm;
      setDirty(true);
      renderStepsList();
      drawEditorCanvas();
    }
  });

  canvas.addEventListener("pointermove", (evt) => {
    const { xCm, yCm } = getPointerFieldPos(evt);
    if (coordsLabel) {
      coordsLabel.textContent = `X: ${xCm.toFixed(0)} cm, Y: ${yCm.toFixed(0)} cm`;
    }
    if (!draggingTarget || !currentRoutine) return;

    if (draggingTarget.kind === "start") {
      currentRoutine.start_pose.x_cm = xCm;
      currentRoutine.start_pose.y_cm = yCm;
      renderMetadataInputs();
    } else if (draggingTarget.kind === "step") {
      const st = currentRoutine.steps[draggingTarget.index];
      if (st) {
        st.x_cm = xCm;
        st.y_cm = yCm;
        renderStepsList();
      }
    }
    setDirty(true);
    drawEditorCanvas();
  });

  const endDrag = (evt) => {
    if (draggingTarget) {
      draggingTarget = null;
      try {
        canvas.releasePointerCapture(evt.pointerId);
      } catch (_) {}
    }
  };
  canvas.addEventListener("pointerup", endDrag);
  canvas.addEventListener("pointercancel", endDrag);
}

function drawEditorCanvas() {
  const canvas = document.getElementById("auto-editor-canvas");
  if (!canvas) return;
  const ctx = canvas.getContext("2d");
  const W = canvas.width;
  const H = canvas.height;
  const g = getCanvasGeometry(canvas);

  ctx.clearRect(0, 0, W, H);

  // Field surface
  ctx.fillStyle = "#111827";
  ctx.fillRect(g.padL, g.padT, g.fieldW, g.fieldH);

  // Grid lines
  ctx.textAlign = "center";
  ctx.textBaseline = "middle";
  ctx.font = "10px monospace";
  for (let x = 0; x <= FIELD_WIDTH_CM; x += 50) {
    const px = g.toPxX(x);
    ctx.beginPath();
    ctx.moveTo(px, g.padT);
    ctx.lineTo(px, g.padT + g.fieldH);
    ctx.strokeStyle = x % 100 === 0 ? "rgba(148, 163, 184, 0.25)" : "rgba(148, 163, 184, 0.1)";
    ctx.lineWidth = 1;
    ctx.stroke();
    if (x % 100 === 0) {
      ctx.fillStyle = "#94a3b8";
      ctx.fillText(`${x}`, px, g.padT + g.fieldH + 14);
    }
  }
  ctx.textAlign = "right";
  for (let y = 0; y <= FIELD_LENGTH_CM; y += 50) {
    const py = g.toPxY(y);
    ctx.beginPath();
    ctx.moveTo(g.padL, py);
    ctx.lineTo(g.padL + g.fieldW, py);
    ctx.strokeStyle = y % 100 === 0 ? "rgba(148, 163, 184, 0.25)" : "rgba(148, 163, 184, 0.1)";
    ctx.lineWidth = 1;
    ctx.stroke();
    if (y % 100 === 0) {
      ctx.fillStyle = "#94a3b8";
      ctx.fillText(`${y}`, g.padL - 6, py);
    }
  }

  // 25 cm safe wall margin box
  const mx1 = g.toPxX(WALL_MARGIN_CM);
  const mx2 = g.toPxX(FIELD_WIDTH_CM - WALL_MARGIN_CM);
  const myTop = g.toPxY(FIELD_LENGTH_CM - WALL_MARGIN_CM);
  const myBot = g.toPxY(WALL_MARGIN_CM);
  ctx.save();
  ctx.setLineDash([4, 4]);
  ctx.strokeStyle = "rgba(148, 163, 184, 0.35)";
  ctx.strokeRect(mx1, myTop, mx2 - mx1, myBot - myTop);
  ctx.restore();

  // Top 3x3 & 1x3 Goals
  const gridLeftX = g.toPxX(110);
  const gridRightX = g.toPxX(290);
  const gridW = gridRightX - gridLeftX;
  ctx.fillStyle = "rgba(245, 158, 11, 0.22)";
  ctx.fillRect(gridLeftX, g.padT - 18, gridW, 18);
  ctx.strokeStyle = "#f59e0b";
  ctx.lineWidth = 2;
  ctx.strokeRect(gridLeftX, g.padT - 18, gridW, 18);
  ctx.fillStyle = "#fbbf24";
  ctx.textAlign = "center";
  ctx.font = "bold 10px monospace";
  ctx.fillText("3x3 TARGET GRID (Y=500)", g.padL + g.fieldW / 2, g.padT - 9);

  const halfWCmPx = (44 / 2 / FIELD_WIDTH_CM) * g.fieldW;
  const colHPx = (24 / FIELD_LENGTH_CM) * g.fieldH;
  for (const colKey of ["left", "center", "right"]) {
    const sc = SPECIAL_GOALS[colKey];
    const scx = g.toPxX(sc.x);
    ctx.fillStyle = "rgba(148, 163, 184, 0.1)";
    ctx.fillRect(scx - halfWCmPx, g.padT, halfWCmPx * 2, colHPx);
    ctx.strokeStyle = "rgba(250, 204, 21, 0.45)";
    ctx.lineWidth = 1;
    ctx.strokeRect(scx - halfWCmPx, g.padT, halfWCmPx * 2, colHPx);
    ctx.fillStyle = "rgba(250, 204, 21, 0.85)";
    ctx.font = "8px monospace";
    ctx.fillText(sc.label, scx, g.padT + colHPx / 2 + 1);
  }

  // Outer Field Walls
  ctx.strokeStyle = "#64748b";
  ctx.lineWidth = 2;
  ctx.strokeRect(g.padL, g.padT, g.fieldW, g.fieldH);

  // Routine start pose & drive_to waypoints
  if (currentRoutine) {
    const sp = currentRoutine.start_pose || { x_cm: 200, y_cm: 80, heading_deg: 0 };
    const nodes = [{ kind: "start", x: Number(sp.x_cm), y: Number(sp.y_cm), heading: Number(sp.heading_deg || 0) }];
    currentRoutine.steps.forEach((st, idx) => {
      if (st.type === "drive_to") {
        nodes.push({
          kind: "step",
          index: idx,
          step: st,
          x: Number(st.x_cm),
          y: Number(st.y_cm),
        });
      }
    });

    // Connecting path polyline
    if (nodes.length > 1) {
      ctx.save();
      ctx.beginPath();
      nodes.forEach((n, i) => {
        const nx = g.toPxX(n.x);
        const ny = g.toPxY(n.y);
        if (i === 0) ctx.moveTo(nx, ny);
        else ctx.lineTo(nx, ny);
      });
      ctx.strokeStyle = "#facc15";
      ctx.lineWidth = 2.2;
      ctx.stroke();

      // Segment direction arrows
      for (let i = 0; i < nodes.length - 1; i++) {
        const ax = g.toPxX(nodes[i].x);
        const ay = g.toPxY(nodes[i].y);
        const bx = g.toPxX(nodes[i + 1].x);
        const by = g.toPxY(nodes[i + 1].y);
        const mx = (ax + bx) * 0.5;
        const my = (ay + by) * 0.5;
        const ang = Math.atan2(by - ay, bx - ax);
        ctx.save();
        ctx.translate(mx, my);
        ctx.rotate(ang);
        ctx.beginPath();
        ctx.moveTo(5, 0);
        ctx.lineTo(-4, -4);
        ctx.lineTo(-4, 4);
        ctx.closePath();
        ctx.fillStyle = "#facc15";
        ctx.fill();
        ctx.restore();
      }
      ctx.restore();
    }

    const runner = latestStatusData?.auto_routine;
    const runningStepIdx = runner && runner.running ? Number(runner.current_step_index) : -1;

    // Draw nodes & aim sight lines
    for (const n of nodes) {
      const nx = g.toPxX(n.x);
      const ny = g.toPxY(n.y);
      if (n.kind === "start") {
        ctx.save();
        ctx.beginPath();
        ctx.arc(nx, ny, 9, 0, Math.PI * 2);
        ctx.fillStyle = "#10b981";
        ctx.fill();
        ctx.strokeStyle = "#ecfdf5";
        ctx.lineWidth = 1.5;
        ctx.stroke();
        ctx.fillStyle = "#06281e";
        ctx.font = "bold 9px monospace";
        ctx.textAlign = "center";
        ctx.textBaseline = "middle";
        ctx.fillText("S", nx, ny + 0.5);
        ctx.restore();
      } else {
        const st = n.step;
        const isSel = n.index === selectedStepIndex;
        const isRun = n.index === runningStepIdx;

        // Aim sight line
        let aimX = null;
        let aimY = null;
        if (st.aim_mode === "column" && SPECIAL_GOALS[st.goal_column]) {
          aimX = SPECIAL_GOALS[st.goal_column].x;
          aimY = SPECIAL_GOALS[st.goal_column].y;
        } else if (st.aim_mode === "active_1x3") {
          aimX = SPECIAL_GOALS.center.x;
          aimY = SPECIAL_GOALS.center.y;
        } else {
          const rad = ((Number(st.heading_deg) || 0) * Math.PI) / 180.0;
          aimX = n.x + 45 * Math.sin(rad);
          aimY = n.y + 45 * Math.cos(rad);
        }
        if (aimX !== null && aimY !== null) {
          ctx.save();
          ctx.setLineDash([3, 3]);
          ctx.beginPath();
          ctx.moveTo(nx, ny);
          ctx.lineTo(g.toPxX(aimX), g.toPxY(aimY));
          ctx.strokeStyle = isSel ? "rgba(56, 189, 248, 0.8)" : "rgba(148, 163, 184, 0.35)";
          ctx.lineWidth = 1.3;
          ctx.stroke();
          ctx.restore();
        }

        ctx.save();
        if (isSel || isRun) {
          ctx.beginPath();
          ctx.arc(nx, ny, 13, 0, Math.PI * 2);
          ctx.strokeStyle = isRun ? "#f59e0b" : "#38bdf8";
          ctx.lineWidth = 2;
          ctx.stroke();
        }
        ctx.beginPath();
        ctx.arc(nx, ny, 9, 0, Math.PI * 2);
        ctx.fillStyle = isRun ? "#f59e0b" : isSel ? "#38bdf8" : "#facc15";
        ctx.fill();
        ctx.fillStyle = "#0b0f17";
        ctx.font = "bold 9px monospace";
        ctx.textAlign = "center";
        ctx.textBaseline = "middle";
        ctx.fillText(String(n.index + 1), nx, ny + 0.5);
        ctx.restore();
      }
    }
  }

  // Live robot pose overlay
  const showLive = Boolean(document.getElementById("editor-show-live-robot-toggle")?.checked);
  const pose = latestStatusData?.pose;
  if (showLive && pose && typeof pose.x_cm === "number" && typeof pose.y_cm === "number") {
    const rx = g.toPxX(pose.x_cm);
    const ry = g.toPxY(pose.y_cm);
    const rad = ((pose.heading_deg || 0) * Math.PI) / 180.0;
    const halfBox = (18 / FIELD_WIDTH_CM) * g.fieldW;
    ctx.save();
    ctx.translate(rx, ry);
    ctx.rotate(rad);
    ctx.fillStyle = "rgba(56, 189, 248, 0.25)";
    ctx.strokeStyle = "#38bdf8";
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.roundRect(-halfBox, -halfBox, halfBox * 2, halfBox * 2, 4);
    ctx.fill();
    ctx.stroke();
    ctx.beginPath();
    ctx.moveTo(0, -halfBox - 7);
    ctx.lineTo(-5, -3);
    ctx.lineTo(5, -3);
    ctx.closePath();
    ctx.fillStyle = "#38bdf8";
    ctx.fill();
    ctx.restore();
  }
}

// ============================================================================
// Live Status & Runner Synchronization
// ============================================================================
function updateRunnerUI(autoState) {
  if (!autoState) return;
  const chip = document.getElementById("editor-runner-chip");
  const icon = document.getElementById("editor-runner-icon");
  const text = document.getElementById("editor-runner-text");

  if (chip && icon && text) {
    chip.classList.remove("auto-chip-running", "auto-chip-completed", "auto-chip-aborted");
    if (autoState.running) {
      chip.classList.add("auto-chip-running");
      icon.textContent = "smart_toy";
      const stepNum = Number(autoState.current_step_index || 0) + 1;
      const total = Number(autoState.total_steps || 0);
      const label = autoState.current_step_label || autoState.current_step_type || "Running";
      text.textContent = `Step ${stepNum}/${total}: ${label} (${Number(autoState.step_elapsed_s || 0).toFixed(1)}s)`;
    } else if (autoState.status === "completed") {
      chip.classList.add("auto-chip-completed");
      icon.textContent = "check_circle";
      text.textContent = `Completed (${Number(autoState.routine_elapsed_s || 0).toFixed(1)}s)`;
    } else if (autoState.status === "aborted") {
      chip.classList.add("auto-chip-aborted");
      icon.textContent = "cancel";
      text.textContent = "Aborted";
    } else {
      icon.textContent = "playlist_play";
      text.textContent = "Auto: Idle";
    }
  }
}

function updateMatchModeBadge(match) {
  if (!match) return;
  const chip = document.getElementById("editor-match-mode-chip");
  const icon = document.getElementById("editor-match-mode-icon");
  const text = document.getElementById("editor-match-mode-text");
  if (!chip || !icon || !text) return;

  chip.className = `match-mode-chip match-mode-${match.mode || "disarmed"}`;
  const labels = {
    auto: ["smart_toy", "AUTO (1:00)"],
    auto_wait: ["warning", "AUTO WAIT"],
    teleop: ["sports_esports", "TELE-OP"],
    practice: ["model_training", "PRACTICE"],
    match_over: ["sports_score", "MATCH OVER"],
    disarmed: ["lock", "DISARMED"],
  };
  const [ic, lbl] = labels[match.mode] || labels.disarmed;
  icon.textContent = ic;
  text.textContent = lbl;
}

function startStatusPolling() {
  let prevRunning = false;
  const poll = async () => {
    try {
      const res = await fetch("/api/status");
      if (res.ok) {
        const data = await res.json();
        latestStatusData = data;
        if (data.match) updateMatchModeBadge(data.match);
        if (data.auto_routine) {
          updateRunnerUI(data.auto_routine);
          if (data.auto_routine.running || prevRunning) {
            renderStepsList();
          }
          prevRunning = Boolean(data.auto_routine.running);
        }
        drawEditorCanvas();
      }
    } catch (_) {
      // Ignore transient poll errors
    } finally {
      const intervalMs = prevRunning ? 250 : 1000;
      setTimeout(poll, intervalMs);
    }
  };
  poll();
}
