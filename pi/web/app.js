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
  $("#clear-estop").hidden = !estopped;

  const job = status.JOB && status.JOB !== "0" ? `Job ${status.JOB}` : null;
  $("#job-label").textContent = job || "No active job";
  document.querySelectorAll(".drive-button, .action-button").forEach((button) => {
    button.disabled = controlsDisabled;
  });

  const stamp = new Date().toLocaleTimeString([], {
    hour: "2-digit",
    minute: "2-digit",
    second: "2-digit",
  });
  $("#last-update").textContent = `Telemetry ${stamp}`;
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
  const [linear, angular] = driveVectors[session.direction]();
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
    api({action: "stop"}).catch((error) => showToast(error.message, true));
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

updateStatus();
setInterval(updateStatus, 750);
