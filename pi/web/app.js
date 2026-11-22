"use strict";

const $ = (selector) => document.querySelector(selector);
const connection = $("#connection");
const connectionLabel = $("#connection-label");
const toast = $("#toast");
const speed = $("#speed");
const turnRate = $("#turn-rate");
let currentStatus = {};
let driveSession = null;
let toastTimer = null;
let settingsDirty = false;

const driveVectors = {
  forward: () => [Number(speed.value), 0],
  reverse: () => [-Number(speed.value), 0],
  left: () => [0, Number(turnRate.value)],
  right: () => [0, -Number(turnRate.value)],
};

async function api(payload) {
  const response = await fetch("/api/command", {
    method: "POST",
    headers: {"Content-Type": "application/json"},
    body: JSON.stringify(payload),
    cache: "no-store",
  });
  const data = await response.json();
  if (!response.ok || !data.ok) {
    throw new Error(data.error || "Robot command failed");
  }
  return data;
}

function showToast(message, error = false) {
  if (error && String(message).includes("BALANCE_TRIPPED")) {
    message = "Driving locked after balancing ended - press Clear E-stop to unlock";
  }
  clearTimeout(toastTimer);
  toast.textContent = message;
  toast.classList.toggle("error", error);
  toast.classList.add("visible");
  toastTimer = setTimeout(() => toast.classList.remove("visible"), 2400);
}

function setConnected(connected) {
  connection.classList.toggle("online", connected);
  connection.classList.toggle("offline", !connected);
  connectionLabel.textContent = connected ? "Robot online" : "Robot offline";
}

function telemetryLabel(source, age) {
  if (!source) return "—";
  if (source === "LIVE") return "Live now";
  if (source === "CACHED") {
    return age !== undefined ? `Background · ${age} ms old` : "Background";
  }
  return source;
}

function renderDriver(axis, status) {
  const key = axis.toUpperCase();
  const prefix = axis.toLowerCase();
  const health = status[`${key}_DRIVER`] || "—";
  const connected = health !== "NO_REPLY" && health !== "—";
  const healthy = health.startsWith("OK_");
  const card = $(`#${prefix}-driver-card`);

  card.classList.toggle("healthy", healthy);
  card.classList.toggle("fault", connected && !healthy);
  card.classList.toggle("offline", !connected);
  $(`#${prefix}-driver-health`).textContent = health;

  const currentScale = status[`${key}_CS`];
  $(`#${prefix}-current-scale`).textContent =
    currentScale && currentScale !== "NA" ? `${currentScale} / 31` : "—";
  $(`#${prefix}-tmc-mode`).textContent =
    status[`${key}_TMC_MODE`] || "—";

  const stepFrequency = status[`${key}_STEP_HZ`];
  $(`#${prefix}-step-frequency`).textContent =
    stepFrequency !== undefined ? `${stepFrequency} Hz` : "—";
  const fullStep = status[`${key}_FULLSTEP`];
  $(`#${prefix}-fullstep`).textContent =
    fullStep === "1" ? "Yes" : fullStep === "0" ? "No" : "—";

  $(`#${prefix}-telemetry`).textContent = telemetryLabel(
    status[`${key}_TELEMETRY`],
    status[`${key}_TELEMETRY_AGE_MS`],
  );
  const pollTime = status[`${key}_POLL_US`];
  $(`#${prefix}-poll-time`).textContent =
    pollTime !== undefined ? `${pollTime} µs` : "—";
}

function render(service) {
  const status = service.status || {};
  currentStatus = status;
  const connected = Boolean(service.connected);
  const estopped = status.ESTOP === "1";
  const controlsDisabled = !connected || estopped;
  const settingsAvailable = connected && !estopped && status.MODE === "IDLE";

  setConnected(connected);
  $("#mode").textContent = status.MODE || "—";
  $("#faults").textContent = status.FAULTS || "—";
  $("#x-driver").textContent = status.X_DRIVER || "—";
  $("#y-driver").textContent = status.Y_DRIVER || "—";
  renderDriver("x", status);
  renderDriver("y", status);
  if (!settingsDirty) {
    if (status.RUN_CURRENT_MA) $("#run-current").value = status.RUN_CURRENT_MA;
    if (status.MICROSTEPS) $("#microsteps").value = status.MICROSTEPS;
    if (status.ACCEL_MM_S2) $("#acceleration").value = status.ACCEL_MM_S2;
    if (status.REQUESTED_TMC_MODE) {
      $("#requested-mode").value = status.REQUESTED_TMC_MODE;
    }
  }
  $("#settings-state").textContent = settingsAvailable
    ? "Ready to apply"
    : estopped
      ? "Emergency stop latched"
      : status.MODE && status.MODE !== "IDLE"
        ? "Available when idle"
        : "Unavailable";
  $("#apply-settings").disabled = !settingsAvailable || !settingsDirty;
  // Also shown when driving is locked after balancing ended.
  $("#clear-estop").hidden = !(estopped || status.DRIVE_LOCK === "1");

  const job = status.JOB && status.JOB !== "0" ? `Job ${status.JOB}` : null;
  $("#job-label").textContent = job || "No active job";
  document
    .querySelectorAll(".drive-button, .action-button:not(.imu-control)")
    .forEach((button) => {
      button.disabled = controlsDisabled;
    });

  const stamp = new Date().toLocaleTimeString([], {
    hour: "2-digit",
    minute: "2-digit",
    second: "2-digit",
  });
  $("#last-update").textContent = `Telemetry ${stamp}`;
  if ("imu" in service) renderImu(service);
}

async function updateStatus() {
  try {
    const response = await fetch("/api/status", {cache: "no-store"});
    const data = await response.json();
    if (!response.ok || !data.ok) throw new Error(data.error);
    render(data.service);
  } catch (error) {
    setConnected(false);
    connectionLabel.textContent = "Control unavailable";
  }
}

async function renewDrive(session) {
  if (driveSession !== session) return;
  let [linear, angular] = driveVectors[session.direction]();
  if (currentStatus.MODE === "BALANCE") {
    // Balance-mode drive limits (firmware: 300 mm/s, 90 deg/s).
    linear = Math.max(-250, Math.min(250, linear));
    angular = Math.max(-60, Math.min(60, angular));
  }
  try {
    await api({
      action: "velocity",
      linear,
      angular,
      lease_ms: 1000,
      hold_ms: 600,
    });
    if (driveSession === session) {
      session.timer = setTimeout(() => renewDrive(session), 200);
    }
  } catch (error) {
    showToast(error.message, true);
    endDrive(false);
  }
}

function beginDrive(event) {
  if (event.button !== undefined && event.button !== 0) return;
  if (currentStatus.ESTOP === "1") return;
  event.preventDefault();
  endDrive(false);
  const button = event.currentTarget;
  if (button.setPointerCapture) button.setPointerCapture(event.pointerId);
  button.classList.add("active");
  const session = {
    direction: button.dataset.drive,
    button,
    pointerId: event.pointerId,
    timer: null,
  };
  driveSession = session;
  renewDrive(session);
}

function endDrive(sendStop = true) {
  const wasDriving = driveSession !== null;
  if (driveSession) {
    clearTimeout(driveSession.timer);
    driveSession.button.classList.remove("active");
    driveSession = null;
  }
  if (sendStop && wasDriving) {
    // While balancing, letting go means "speed 0", not STOP (which would end
    // balancing and drop the robot onto a training wheel).
    const release = currentStatus.MODE === "BALANCE"
      ? {action: "velocity", linear: 0, angular: 0, lease_ms: 500, hold_ms: 500}
      : {action: "stop"};
    api(release).catch((error) => showToast(error.message, true));
  }
}

document.querySelectorAll(".drive-button").forEach((button) => {
  button.addEventListener("pointerdown", beginDrive);
  button.addEventListener("pointerup", () => endDrive());
  button.addEventListener("pointercancel", () => endDrive());
  button.addEventListener("lostpointercapture", () => endDrive());
  button.addEventListener("contextmenu", (event) => event.preventDefault());
  button.addEventListener("selectstart", (event) => event.preventDefault());
  button.addEventListener("dragstart", (event) => event.preventDefault());
});
window.addEventListener("pointerup", () => endDrive());
window.addEventListener("pointercancel", () => endDrive());
window.addEventListener("pointermove", (event) => {
  if (
    driveSession &&
    event.pointerId === driveSession.pointerId &&
    event.pointerType !== "touch" &&
    event.buttons === 0
  ) {
    endDrive();
  }
});
window.addEventListener("blur", () => endDrive());
document.addEventListener("visibilitychange", () => {
  if (document.hidden) endDrive();
});

$("#stop").addEventListener("click", () => {
  endDrive(false);
  api({action: "stop"})
    .then(() => showToast("Robot stopped"))
    .catch((error) => showToast(error.message, true));
});

$("#estop").addEventListener("click", () => {
  endDrive(false);
  api({action: "estop"})
    .then(() => {
      showToast("Emergency stop latched");
      updateStatus();
    })
    .catch((error) => showToast(error.message, true));
});

$("#clear-estop").addEventListener("click", () => {
  if (!window.confirm("Clear the emergency-stop latch?")) return;
  api({action: "clear_estop"})
    .then(() => {
      showToast("Emergency stop cleared");
      updateStatus();
    })
    .catch((error) => showToast(error.message, true));
});

$("#shutdown").addEventListener("click", async () => {
  if (!window.confirm("Shut down the Raspberry Pi? The robot controls will go offline.")) {
    return;
  }
  try {
    const response = await fetch("/api/shutdown", {
      method: "POST",
      headers: {"Content-Type": "application/json"},
      body: JSON.stringify({confirm: "shutdown"}),
    });
    const data = await response.json();
    if (!response.ok || !data.ok) throw new Error(data.error || "Shutdown failed");
    showToast("Raspberry Pi is shutting down");
  } catch (error) {
    showToast(error.message, true);
  }
});

$("#move").addEventListener("click", () => {
  api({action: "move", distance: Number($("#distance").value)})
    .then((data) => showToast(data.reply))
    .catch((error) => showToast(error.message, true));
});

$("#turn").addEventListener("click", () => {
  api({action: "turn", angle: Number($("#angle").value)})
    .then((data) => showToast(data.reply))
    .catch((error) => showToast(error.message, true));
});

speed.addEventListener("input", () => {
  $("#speed-output").textContent = `${speed.value} mm/s`;
});
turnRate.addEventListener("input", () => {
  $("#turn-output").textContent = `${turnRate.value}°/s`;
});

document
  .querySelectorAll("#run-current, #microsteps, #acceleration, #requested-mode")
  .forEach((input) => {
    input.addEventListener("input", () => {
      settingsDirty = true;
      render({connected: true, status: currentStatus});
    });
  });

$("#apply-settings").addEventListener("click", () => {
  api({
    action: "configure",
    current_ma: Number($("#run-current").value),
    microsteps: Number($("#microsteps").value),
    acceleration_mm_s2: Number($("#acceleration").value),
    tmc_mode: $("#requested-mode").value,
  })
    .then(() => {
      settingsDirty = false;
      showToast("Motor settings applied");
      updateStatus();
    })
    .catch((error) => showToast(error.message, true));
});

// ================================================================== IMU
// Live IMU values come with /api/status (robotd polls the Pico at ~10 Hz).
// The chart uses robotd's rolling history. Tests use IMU test windows: the
// Pico's core 1 accumulates statistics over every 500 Hz sample between
// IMU_WINDOW_START and IMU_WINDOW.

const CHART_SECONDS = 30;
const RESULTS_KEY = "r2w-imu-results";
const MAX_RESULTS = 50;
const chartSeries = {
  angles: {keys: ["roll", "pitch"], labels: ["Roll", "Pitch"], unit: "°", span: 2},
  yaw: {keys: ["yaw"], labels: ["Heading"], unit: "°", span: 2},
  rates: {keys: ["p", "q", "r"], labels: ["p", "q", "r"], unit: "°/s", span: 2},
  accel: {keys: ["ax", "ay", "az"], labels: ["ax", "ay", "az"], unit: " g", span: 0.1},
};

const imu = {
  latest: {},
  supported: null,
  connected: false,
  status: {},
  points: [],
  lastPoint: 0,
  test: null,
  recording: null,
  results: loadResults(),
  settingsHoldUntil: 0,
};

function num(value) {
  if (value === undefined || value === null || value === "") return null;
  const n = Number(value);
  return Number.isFinite(n) ? n : null;
}

function fmt(value, digits = 1, unit = "") {
  const n = num(value);
  return n === null ? "—" : `${n.toFixed(digits)}${unit}`;
}

function signed(value, digits = 1, unit = "") {
  const n = num(value);
  if (n === null) return "—";
  return `${n >= 0 ? "+" : ""}${n.toFixed(digits)}${unit}`;
}

function sleep(ms) {
  return new Promise((resolve) => setTimeout(resolve, ms));
}

function loadResults() {
  try {
    const stored = JSON.parse(localStorage.getItem(RESULTS_KEY) || "[]");
    return Array.isArray(stored) ? stored : [];
  } catch (error) {
    return [];
  }
}

function saveResults() {
  try {
    localStorage.setItem(RESULTS_KEY, JSON.stringify(imu.results));
  } catch (error) {
    // Storage unavailable: results stay for this page only.
  }
}

function renderImu(service) {
  imu.supported = service.imu_supported;
  imu.connected = Boolean(service.connected);
  imu.status = service.status || {};
  const data = service.imu || {};
  imu.latest = data;

  const state = imu.supported === false
    ? "NOT IN FIRMWARE"
    : data.STATE || (imu.connected ? "WAITING" : "OFFLINE");
  const pill = $("#imu-state");
  pill.textContent = data.CAL === "FORCED" && state === "OK"
    ? "OK · calibration forced"
    : state.replace("_", " ");
  pill.classList.toggle("ok", state === "OK");
  pill.classList.toggle("busy", state === "CALIBRATING" || state === "STARTING" ||
    state === "WAITING");
  pill.classList.toggle("fault", ["NO_SENSOR", "STALE", "OFF", "NOT IN FIRMWARE",
    "OFFLINE"].includes(state));

  const live = state === "OK";
  $("#imu-roll").textContent = live ? signed(data.ROLL, 1, "°") : "—";
  $("#imu-pitch").textContent = live ? signed(data.PITCH, 1, "°") : "—";
  $("#imu-yaw").textContent = live ? signed(data.YAW, 1, "°") : "—";
  $("#imu-rates").textContent = live
    ? `${fmt(data.P, 1)} / ${fmt(data.Q, 1)} / ${fmt(data.R, 1)}`
    : "—";
  $("#imu-rate").textContent =
    data.SAMPLE_HZ !== undefined ? `${data.SAMPLE_HZ} Hz` : "—";
  $("#imu-temp").textContent = fmt(data.TEMP_C, 1, " °C");

  if (Date.now() > imu.settingsHoldUntil) {
    const tau = num(data.TAU);
    if (tau !== null) {
      let best = null;
      for (const option of $("#imu-tau").options) {
        if (best === null ||
            Math.abs(Number(option.value) - tau) <
            Math.abs(Number(best.value) - tau)) {
          best = option;
        }
      }
      if (best) $("#imu-tau").value = best.value;
    }
    $("#imu-auto-bias").checked = data.AUTO_BIAS === "1";
  }

  if (data.STATE) {
    $("#imu-health").textContent = [
      `Calibration ${data.CAL || "—"}` +
        (data.CAL_ATTEMPTS && data.CAL_ATTEMPTS !== "1"
          ? ` (attempt ${data.CAL_ATTEMPTS})` : ""),
      `bias ${fmt(data.BIAS_X, 2)} / ${fmt(data.BIAS_Y, 2)} / ${fmt(data.BIAS_Z, 2)} °/s`,
      `errors ${data.ERRORS ?? "—"}`,
      `stalls ${data.STALLS ?? "—"}`,
      `missed ${data.LATE ?? "—"}`,
      `accel skips ${data.ACC_REJECTS ?? "—"}`,
      `auto-bias updates ${data.AUTO_UPDATES ?? "—"}`,
      `sample age ${data.AGE_MS ?? "—"} ms`,
    ].join(" · ");
  } else {
    $("#imu-health").textContent = imu.supported === false
      ? "This firmware has no IMU commands. Flash the IMU build to use this panel."
      : "—";
  }
  renderImuControls();
}

function renderImuControls() {
  const state = imu.latest.STATE;
  const ok = imu.connected && state === "OK";
  const up = imu.connected && ["OK", "CALIBRATING", "STALE"].includes(state);
  const idle = imu.status.MODE === "IDLE";
  const estop = imu.status.ESTOP === "1";
  const busy = Boolean(imu.test || imu.recording);

  $("#imu-calibrate").disabled = !(up && idle) || busy;
  $("#imu-zero").disabled = !ok;
  $("#imu-tau").disabled = !up;
  $("#imu-auto-bias").disabled = !up;
  $("#record-start").disabled = !ok || busy;
  $("#record-stop").disabled = !imu.recording;
  const driveRecord = $("#drive-record");
  driveRecord.disabled = imu.recording ? false : !ok || busy;
  driveRecord.classList.toggle("recording", Boolean(imu.recording));
  $("#drive-record-label").textContent = imu.recording
    ? `Stop recording · ${Math.round((Date.now() - imu.recording.startedAt) / 1000)} s`
    : "Record IMU";
  document.querySelectorAll(".imu-test-run").forEach((button) => {
    button.disabled = !ok || !idle || estop || busy;
  });
}

// ---- rolling chart

async function updateImuHistory() {
  if (imu.supported === false || document.hidden) return;
  try {
    const data = await api({action: "imu_history", since: imu.lastPoint});
    if (data.latest < imu.lastPoint) {
      imu.points = [];          // robotd restarted
      imu.lastPoint = 0;
    }
    for (const point of data.points) {
      imu.points.push(point);
      imu.lastPoint = point.n;
    }
    if (imu.points.length) {
      const newest = imu.points[imu.points.length - 1].t;
      const firstKept = imu.points.findIndex((p) => p.t >= newest - CHART_SECONDS);
      if (firstKept > 0) imu.points.splice(0, firstKept);
    }
    drawChart();
  } catch (error) {
    // Status polling already reports connection problems.
  }
}

function chartColours() {
  const style = getComputedStyle(document.documentElement);
  return {
    series: [style.getPropertyValue("--green").trim(),
             style.getPropertyValue("--amber").trim(), "#7cc4ff"],
    grid: style.getPropertyValue("--border").trim(),
    text: style.getPropertyValue("--muted").trim(),
  };
}

function renderLegend() {
  const series = chartSeries[$("#imu-chart-mode").value];
  const legend = $("#imu-legend");
  legend.replaceChildren();
  series.labels.forEach((label, index) => {
    const item = document.createElement("span");
    const swatch = document.createElement("i");
    swatch.className = `c${index}`;
    item.append(swatch, label);
    legend.append(item);
  });
}

function drawChart() {
  const canvas = $("#imu-chart");
  const ratio = window.devicePixelRatio || 1;
  const width = canvas.clientWidth;
  const height = canvas.clientHeight;
  if (!width || !height) return;
  if (canvas.width !== Math.round(width * ratio) ||
      canvas.height !== Math.round(height * ratio)) {
    canvas.width = Math.round(width * ratio);
    canvas.height = Math.round(height * ratio);
  }
  const context = canvas.getContext("2d");
  context.setTransform(ratio, 0, 0, ratio, 0, 0);
  context.clearRect(0, 0, width, height);

  const colours = chartColours();
  const series = chartSeries[$("#imu-chart-mode").value];
  const points = imu.points;
  const left = 46;
  const right = width - 8;
  const top = 8;
  const bottom = height - 18;

  context.font = "10px system-ui, sans-serif";
  context.fillStyle = colours.text;
  if (points.length < 2) {
    context.fillText("Waiting for IMU data…", left, top + 20);
    return;
  }

  const newest = points[points.length - 1].t;
  const oldest = newest - CHART_SECONDS;
  let low = Infinity;
  let high = -Infinity;
  for (const point of points) {
    for (const key of series.keys) {
      low = Math.min(low, point[key]);
      high = Math.max(high, point[key]);
    }
  }
  const centre = (low + high) / 2;
  const span = Math.max(high - low, series.span) * 1.15;
  low = centre - span / 2;
  high = centre + span / 2;

  const x = (t) => left + ((t - oldest) / CHART_SECONDS) * (right - left);
  const y = (v) => bottom - ((v - low) / (high - low)) * (bottom - top);

  context.strokeStyle = colours.grid;
  context.lineWidth = 1;
  context.textAlign = "right";
  for (let i = 0; i <= 4; i += 1) {
    const value = low + ((high - low) * i) / 4;
    const yy = y(value);
    context.beginPath();
    context.moveTo(left, yy);
    context.lineTo(right, yy);
    context.stroke();
    const digits = series.span < 1 ? 2 : Math.abs(high - low) < 10 ? 1 : 0;
    context.fillText(`${value.toFixed(digits)}${series.unit}`, left - 5, yy + 3);
  }
  context.textAlign = "center";
  for (let s = 0; s <= CHART_SECONDS; s += 5) {
    context.fillText(s === CHART_SECONDS ? "now" : `-${CHART_SECONDS - s}s`,
                     x(oldest + s), height - 4);
  }

  series.keys.forEach((key, index) => {
    context.strokeStyle = colours.series[index];
    context.lineWidth = 1.6;
    context.beginPath();
    let started = false;
    for (const point of points) {
      if (point.t < oldest) continue;
      const xx = x(point.t);
      const yy = y(point[key]);
      if (!started) {
        context.moveTo(xx, yy);
        started = true;
      } else {
        context.lineTo(xx, yy);
      }
    }
    context.stroke();
  });
}

// ---- tests and recording

function setProgress(text) {
  $("#imu-progress-text").textContent = text;
}

function showProgress(visible) {
  $("#imu-progress").hidden = !visible;
}

function fieldFromReply(reply, key) {
  const match = new RegExp(`\\b${key}=(\\S+)`).exec(reply || "");
  return match ? match[1] : null;
}

async function startWindow() {
  const data = await api({action: "imu_window_start"});
  return Number(fieldFromReply(data.reply, "WINDOW") || 0);
}

async function readWindow() {
  const data = await api({action: "imu_window"});
  return data.window || {};
}

async function waitForJob(job, test, label) {
  const started = Date.now();
  while (true) {
    if (test.cancelled) throw new Error("Test cancelled");
    const data = await api({action: "status"});
    const status = data.service.status || {};
    if (status.JOB !== String(job)) {
      test.jobDone = true;
      return status.LAST_RESULT || "OK";
    }
    if (Date.now() - started > 60000) {
      throw new Error("Timed out waiting for the job to finish");
    }
    setProgress(`${label}… ${((Date.now() - started) / 1000).toFixed(0)} s`);
    await sleep(300);
  }
}

function makeResult(kind, label, startedAt, w, expected = null, note = "") {
  const n = (key) => num(w[key]);
  const duration = (n("DURATION_MS") ?? 0) / 1000;
  const yawChange = n("YAW_CHANGE");
  const result = {
    kind,
    label,
    time: startedAt.toLocaleTimeString([], {hour: "2-digit", minute: "2-digit",
                                            second: "2-digit"}),
    durationS: duration,
    samples: n("SAMPLES"),
    movingSamples: n("MOVING_SAMPLES"),
    yawChange,
    expected,
    error: expected !== null && yawChange !== null ? yawChange - expected : null,
    driftPerMin: kind === "still" && duration > 0 && yawChange !== null
      ? yawChange / (duration / 60) : null,
    rollRange: n("ROLL_MAX") !== null ? n("ROLL_MAX") - n("ROLL_MIN") : null,
    pitchRange: n("PITCH_MAX") !== null ? n("PITCH_MAX") - n("PITCH_MIN") : null,
    gyroSd: Math.max(n("GX_SD") ?? 0, n("GY_SD") ?? 0, n("GZ_SD") ?? 0),
    accelSdMg: 1000 * Math.max(n("AX_SD") ?? 0, n("AY_SD") ?? 0, n("AZ_SD") ?? 0),
    amagMin: n("AMAG_MIN"),
    amagMax: n("AMAG_MAX"),
    late: n("LATE") ?? 0,
    errors: n("ERRORS") ?? 0,
    stalls: n("STALLS") ?? 0,
    note,
    raw: w,
  };
  if (expected !== null && yawChange !== null && expected !== 0 &&
      Math.sign(yawChange) !== Math.sign(expected) && Math.abs(yawChange) > 5) {
    result.note = [note, "heading changed the opposite way: check IMU_AXIS_SIGN"]
      .filter(Boolean).join("; ");
  }
  return result;
}

function addResult(result) {
  imu.results.unshift(result);
  imu.results.length = Math.min(imu.results.length, MAX_RESULTS);
  saveResults();
  renderResults();
}

function cell(row, text, className = "") {
  const td = document.createElement("td");
  td.textContent = text;
  if (className) td.className = className;
  row.append(td);
}

function renderResults() {
  const body = $("#results-body");
  body.replaceChildren();
  if (!imu.results.length) {
    const row = document.createElement("tr");
    row.className = "imu-empty";
    const td = document.createElement("td");
    td.colSpan = 12;
    td.textContent = "No results yet";
    row.append(td);
    body.append(row);
    $("#results-raw").textContent = "—";
    return;
  }
  for (const r of imu.results) {
    const row = document.createElement("tr");
    row.title = r.note || "";
    cell(row, r.note ? `${r.label} ⚠` : r.label, r.note ? "warn" : "");
    cell(row, r.time);
    cell(row, `${fmt(r.durationS, 1)} s`);
    cell(row, signed(r.yawChange, 1, "°"));
    cell(row, r.expected === null ? "—"
      : `${signed(r.expected, 0, "°")} / ${signed(r.error, 1, "°")}` +
        ` (${fmt(Math.abs(100 * r.error / r.expected), 1)}%)`);
    cell(row, r.driftPerMin === null ? "—" : `${signed(r.driftPerMin, 2)}°/min`);
    cell(row, `${fmt(r.rollRange, 2)}°`);
    cell(row, `${fmt(r.pitchRange, 2)}°`);
    cell(row, `${fmt(r.gyroSd, 2)} °/s`);
    cell(row, `${fmt(r.accelSdMg, 1)} mg`);
    cell(row, `${fmt(r.amagMin, 2)}–${fmt(r.amagMax, 2)} g`);
    const problems = r.late + r.errors + r.stalls;
    cell(row, `${r.late} / ${r.errors} / ${r.stalls}`, problems ? "bad" : "");
    body.append(row);
  }
  const latest = imu.results[0];
  $("#results-raw").textContent =
    `${latest.label} at ${latest.time}\n` +
    Object.entries(latest.raw || {}).map(([k, v]) => `${k}=${v}`).join("  ");
}

async function runTest(kind) {
  if (imu.test || imu.recording) return;
  const test = {kind, cancelled: false, job: null, jobDone: false};
  imu.test = test;
  renderImuControls();
  showProgress(true);
  const startedAt = new Date();
  let label = kind;
  let expected = null;
  let note = "";
  try {
    if (kind === "still") {
      const seconds = Number($("#still-duration").value);
      label = `Still ${seconds >= 60 ? `${seconds / 60} min` : `${seconds} s`}`;
      await startWindow();
      const started = Date.now();
      while (Date.now() - started < seconds * 1000) {
        if (test.cancelled) throw new Error("Test cancelled");
        if (currentStatus.MODE && currentStatus.MODE !== "IDLE") {
          throw new Error("The robot moved during the still test");
        }
        const left = Math.ceil(seconds - (Date.now() - started) / 1000);
        setProgress(`Still test: ${left} s left. Keep the robot still.`);
        await sleep(250);
      }
    } else {
      const value = Number($(kind === "move" ? "#test-distance" : "#test-angle").value);
      if (!value) throw new Error("Enter a non-zero value");
      label = kind === "move" ? `Move ${value} mm` : `Turn ${value}°`;
      if (kind === "turn") expected = value;
      await startWindow();
      setProgress(`${label}: recording baseline…`);
      await sleep(400);
      const reply = await api(kind === "move"
        ? {action: "move", distance: value}
        : {action: "turn", angle: value});
      test.job = Number(fieldFromReply(reply.reply, "JOB") || 0);
      const outcome = await waitForJob(test.job, test, kind === "move" ? "Moving" : "Turning");
      if (outcome !== "OK") note = `job ended ${outcome}`;
      setProgress(`${label}: settling…`);
      await sleep(800);
    }
    const window = await readWindow();
    addResult(makeResult(kind, label, startedAt, window, expected, note));
    showToast(`${label}: result saved`);
  } catch (error) {
    if (test.job && !test.jobDone) {
      api({action: "stop"}).catch(() => {});
    }
    showToast(error.message, true);
  } finally {
    imu.test = null;
    showProgress(false);
    renderImuControls();
  }
}

async function startRecording() {
  if (imu.test || imu.recording) return;
  try {
    const label = $("#record-label").value.trim() ||
      `Drive recording (${speed.value} mm/s, ${turnRate.value}°/s)`;
    await startWindow();
    imu.recording = {label, startedAt: new Date(), timer: null};
    showProgress(true);
    setProgress(`Recording "${label}"…`);
    imu.recording.timer = setInterval(updateRecording, 1000);
    renderImuControls();
  } catch (error) {
    showToast(error.message, true);
  }
}

async function updateRecording() {
  const recording = imu.recording;
  if (!recording) return;
  renderImuControls();
  try {
    const w = await readWindow();
    if (imu.recording !== recording) return;
    setProgress(`Recording "${recording.label}": ` +
      `${fmt((num(w.DURATION_MS) ?? 0) / 1000, 0)} s, heading ${signed(w.YAW_CHANGE, 1, "°")}, ` +
      `roll range ${fmt((num(w.ROLL_MAX) ?? 0) - (num(w.ROLL_MIN) ?? 0), 1)}°, ` +
      `pitch range ${fmt((num(w.PITCH_MAX) ?? 0) - (num(w.PITCH_MIN) ?? 0), 1)}°`);
  } catch (error) {
    // Keep recording; the next update may succeed.
  }
}

async function stopRecording(save = true) {
  const recording = imu.recording;
  if (!recording) return;
  clearInterval(recording.timer);
  imu.recording = null;
  showProgress(false);
  renderImuControls();
  if (!save) {
    showToast("Recording discarded");
    return;
  }
  try {
    const w = await readWindow();
    addResult(makeResult("record", recording.label, recording.startedAt, w));
    showToast(`"${recording.label}" saved`);
  } catch (error) {
    showToast(error.message, true);
  }
}

function resultsCsv() {
  const columns = [
    ["label", "Test"], ["time", "Time"], ["durationS", "Duration s"],
    ["samples", "Samples"], ["movingSamples", "Moving samples"],
    ["yawChange", "Heading change deg"], ["expected", "Expected deg"],
    ["error", "Error deg"], ["driftPerMin", "Drift deg/min"],
    ["rollRange", "Roll range deg"], ["pitchRange", "Pitch range deg"],
    ["gyroSd", "Gyro sd max deg/s"], ["accelSdMg", "Accel sd max mg"],
    ["amagMin", "|a| min g"], ["amagMax", "|a| max g"], ["late", "Missed"],
    ["errors", "Errors"], ["stalls", "Stalls"], ["note", "Note"],
  ];
  const quote = (value) => {
    const text = value === null || value === undefined ? "" : String(value);
    return /[",\n]/.test(text) ? `"${text.replace(/"/g, '""')}"` : text;
  };
  const lines = [columns.map(([, title]) => quote(title)).join(",")];
  for (const r of [...imu.results].reverse()) {
    lines.push(columns.map(([key]) => quote(r[key])).join(","));
  }
  return lines.join("\n") + "\n";
}

$("#imu-chart-mode").addEventListener("change", () => {
  renderLegend();
  drawChart();
});
window.addEventListener("resize", drawChart);

$("#imu-calibrate").addEventListener("click", () => {
  api({action: "imu_cal"})
    .then(() => showToast("Calibrating: keep the robot still for about 2 s"))
    .catch((error) => showToast(error.message, true));
});

$("#imu-zero").addEventListener("click", () => {
  api({action: "imu_zero_yaw"})
    .then(() => showToast("Heading zeroed"))
    .catch((error) => showToast(error.message, true));
});

$("#imu-tau").addEventListener("change", () => {
  imu.settingsHoldUntil = Date.now() + 3000;
  api({action: "imu_set", tau: Number($("#imu-tau").value)})
    .then(() => showToast(`Filter time constant ${$("#imu-tau").value} s`))
    .catch((error) => showToast(error.message, true));
});

$("#imu-auto-bias").addEventListener("change", () => {
  imu.settingsHoldUntil = Date.now() + 3000;
  const on = $("#imu-auto-bias").checked;
  api({action: "imu_set", auto_bias: on})
    .then(() => showToast(on ? "Auto-bias on (only while stopped)" : "Auto-bias off"))
    .catch((error) => showToast(error.message, true));
});

$("#record-start").addEventListener("click", startRecording);
$("#drive-record").addEventListener("click", () => {
  if (imu.recording) {
    stopRecording(true);
  } else {
    startRecording();
  }
});
$("#record-stop").addEventListener("click", () => stopRecording(true));
document.querySelectorAll(".imu-test-run").forEach((button) => {
  button.addEventListener("click", () => runTest(button.dataset.test));
});

$("#imu-cancel").addEventListener("click", () => {
  if (imu.test) {
    imu.test.cancelled = true;
    if (imu.test.job && !imu.test.jobDone) {
      api({action: "stop"}).catch((error) => showToast(error.message, true));
    }
  } else if (imu.recording) {
    stopRecording(false);
  }
});

$("#results-csv").addEventListener("click", () => {
  if (!imu.results.length) {
    showToast("No results to download");
    return;
  }
  const blob = new Blob([resultsCsv()], {type: "text/csv"});
  const link = document.createElement("a");
  link.href = URL.createObjectURL(blob);
  link.download = `r2w-imu-results-${new Date().toISOString().slice(0, 19)
    .replace(/[:T]/g, "-")}.csv`;
  document.body.append(link);
  link.click();
  link.remove();
  setTimeout(() => URL.revokeObjectURL(link.href), 1000);
});

$("#results-clear").addEventListener("click", () => {
  if (!imu.results.length || !window.confirm("Clear all saved IMU results?")) return;
  imu.results = [];
  saveResults();
  renderResults();
});

renderLegend();
renderResults();
updateStatus();
setInterval(updateStatus, 750);
setInterval(updateImuHistory, 250);
