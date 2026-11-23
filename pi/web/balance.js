"use strict";

// Balance page: status, run control with kicks, driving, tuning, runs + plots.

const $ = (selector) => document.querySelector(selector);
const POLL_MS = 300;
const KICK_U = 2.0;          // m/s^2
const KICK_MS = 40;
const KICK_FIRST_S = 8;      // after engaging
const KICK_EVERY_S = 3;
const GAIN_STEP_LIMIT = 0.15;  // fraction, for KP/KD/KV/KX
const TUNE_LABELS = {
  KP: "KP (angle)", KD: "KD (angle rate)", KV: "KV (speed)", KX: "KX (position)",
  TRIM: "Trim (°)", DRIVE_LEAN: "Drive lean (°)", DRIVE_ACCEL: "Drive accel (m/s²)",
  DRIVE_FF: "Drive feed-forward", TRIM_RATE: "Trim rate",
};

let bal = {};          // latest BAL_STATUS fields
let service = {};      // latest robotd snapshot
let tuning = {saved: {}, defaults: {}, ranges: {}};
let tuneFilled = false;
let drive = null;      // {direction, button, timer}
let toastTimer = null;
const kick = {wanted: 0, done: 0, engagedAt: null, next: 0, busy: false};
let runActive = false;

// ---------- helpers ----------
function showToast(message, error = false) {
  const toast = $("#toast");
  clearTimeout(toastTimer);
  if (error && String(message).includes("BALANCE_TRIPPED")) {
    message = "Driving locked after balancing ended - press Clear";
  }
  toast.textContent = message;
  toast.classList.toggle("error", error);
  toast.classList.add("visible");
  toastTimer = setTimeout(() => toast.classList.remove("visible"), 3500);
}

async function post(path, payload) {
  const response = await fetch(path, {
    method: "POST",
    headers: {"Content-Type": "application/json"},
    body: JSON.stringify(payload || {}),
    cache: "no-store",
  });
  const data = await response.json().catch(() => ({ok: false, error: "bad reply"}));
  if (!response.ok || !data.ok) throw new Error(data.error || `HTTP ${response.status}`);
  return data;
}

const command = (payload) => post("/api/command", payload);
const num = (value, digits = 2) =>
  (value === undefined || value === "" || isNaN(Number(value))) ? "—" : Number(value).toFixed(digits);

// ---------- status ----------
async function poll() {
  try {
    const response = await fetch("/api/balance", {cache: "no-store"});
    const data = await response.json();
    if (!response.ok || !data.ok) throw new Error(data.error || "no status");
    bal = data.balance || {};
    service = data.service || {};
    setConnection(Boolean(service.connected), service.connected ? "Online" : "Pico offline");
    render();
    if (!tuneFilled && bal.KP !== undefined) fillTuning(bal);
    kickTick();
  } catch (error) {
    setConnection(false, "Offline");
  } finally {
    setTimeout(poll, POLL_MS);
  }
}

function setConnection(online, label) {
  const element = $("#connection");
  element.classList.toggle("online", online);
  element.classList.toggle("offline", !online);
  $("#connection-label").textContent = label;
}

function render() {
  const state = bal.BAL || "—";
  const card = $(".metric-state");
  card.classList.toggle("running", state === "RUNNING");
  card.classList.toggle("arming", state === "ARMING");
  card.classList.toggle("tripped", state === "TRIPPED");
  $("#s-state").textContent = state;
  $("#s-reason").textContent =
    (state === "TRIPPED" || state === "OFF") && bal.REASON && bal.REASON !== "NONE"
      ? `last end: ${bal.REASON}` : (bal.DRY === "1" ? "dry run" : "");
  $("#s-pitch").textContent = `${num(bal.THETA)}°`;
  $("#s-err").textContent = `${num(bal.ERR)}°`;
  $("#s-v").textContent = `${num(Number(bal.V) * 1000, 0)} mm/s`;
  $("#s-x").textContent = `${num(Number(bal.X) * 1000, 0)} mm`;
  $("#s-trim").textContent = `${num(bal.TRIM_EST)}°`;
  $("#s-run").textContent = bal.RUN_MS ? `${(Number(bal.RUN_MS) / 1000).toFixed(1)} s` : "—";
  $("#s-max").textContent = `${num(bal.MAX_ERR, 1)}° / ${num(bal.MAX_U, 1)}`;

  const status = service.status || {};
  const locked = status.DRIVE_LOCK === "1" || status.ESTOP === "1";
  $("#lock-banner").hidden = !locked;
  $("#lock-text").textContent = status.ESTOP === "1"
    ? "Emergency stop is latched." : "Driving is locked after balancing ended.";

  const running = state === "RUNNING";
  if (bal.BAL) document.querySelectorAll(".drive-button").forEach((b) => { b.disabled = !running; });
  // Only a real state ends a drive; a status without BAL (a glitch) doesn't.
  if (!running && drive && bal.BAL) endDrive(false, `balancing is ${state}`);
  const balancing = state === "RUNNING" || state === "ARMING";
  $("#run-start").disabled = balancing || !service.connected;
  $("#run-stop").disabled = !balancing && !runActive;
  if (runActive && !balancing && kick.engagedAt !== null) {
    // Balancing ended by itself (trip): finish the run.
    finishRun("balancing ended");
  }
  renderTuningMarks();
}

// ---------- run control and kicks ----------
async function startRun() {
  const notes = $("#run-notes").value.trim();
  const kicks = Math.max(0, Math.min(200, parseInt($("#run-kicks").value || "0", 10)));
  const record = $("#run-record").checked;
  $("#run-start").disabled = true;
  try {
    const result = await post("/api/balance/start", {notes, record});
    runActive = true;
    Object.assign(kick, {wanted: kicks, done: 0, engagedAt: null, next: 0, busy: false});
    $("#run-summary").hidden = true;
    const reset = Object.keys(result.reset || {});
    showToast(reset.length ? `Armed (safety limits reset: ${reset.join(", ")})` : "Armed: lift it upright");
    setProgress(record ? `Armed, recording ${result.run}. Lift it upright…` : "Armed (not recording). Lift it upright…");
  } catch (error) {
    showToast(error.message, true);
  }
}

function kickTick() {
  if (!runActive) return;
  const now = performance.now() / 1000;
  if (bal.BAL === "RUNNING" && kick.engagedAt === null) {
    kick.engagedAt = now;
    kick.next = now + KICK_FIRST_S;
    setProgress(kick.wanted ? `Balancing. Kicks start in ${KICK_FIRST_S} s.` : "Balancing.");
  }
  if (bal.BAL !== "RUNNING" || kick.done >= kick.wanted || kick.busy) return;
  const calm = Math.abs(Number(bal.RATE)) < 10 && Math.abs(Number(bal.V)) < 0.1;
  if (now < kick.next || !calm) return;
  const sign = kick.done % 2 === 0 ? 1 : -1;
  kick.busy = true;
  command({action: "bal_kick", u: sign * KICK_U, ms: KICK_MS})
    .then(() => {
      kick.done += 1;
      kick.next = performance.now() / 1000 + KICK_EVERY_S;
      setProgress(kick.done >= kick.wanted
        ? `Kicks done (${kick.done}). End balancing when ready.`
        : `Kick ${kick.done}/${kick.wanted}`);
    })
    .catch((error) => showToast(error.message, true))
    .finally(() => { kick.busy = false; });
}

async function stopRun() {
  await finishRun("ended");
}

let finishing = false;
async function finishRun(why) {
  if (finishing) return;
  finishing = true;
  endDrive(false);
  setProgress(`Balancing ${why}. Saving and analysing the run…`);
  try {
    const result = await post("/api/balance/stop", {});
    runActive = false;
    kick.wanted = 0;
    if (result.summary) {
      showSummary(result.summary);
      setProgress(`Saved ${result.summary.run}.`);
      loadRuns();
    } else {
      setProgress("Balancing ended (no run was recorded).");
    }
  } catch (error) {
    // Don't retry automatically on every poll; the button still works.
    kick.engagedAt = null;
    showToast(error.message, true);
    setProgress(`Could not finish the run: ${error.message}`);
  } finally {
    finishing = false;
  }
}

function setProgress(text) {
  $("#run-progress").textContent = text;
}

function summaryText(s) {
  const parts = [`stood ${s.stood_s} s`, `ended ${s.end_reason || "?"}`,
                 `max |err| ${s.max_err_deg}°`, `max |u| ${s.max_u} m/s²`];
  const k = s.kick;
  if (k && k.l_eff_mm) {
    parts.push(`l_eff ${k.l_eff_mm} mm (${(k.kicks || []).length} kicks)`);
    if (k.ring_hz) parts.push(`ring ${k.ring_hz} Hz, damping ${k.ring_zeta}`);
  }
  return parts.join(" · ");
}

function showSummary(s) {
  const box = $("#run-summary");
  box.textContent = "";
  const title = document.createElement("strong");
  title.textContent = s.run;
  box.append(title, document.createElement("br"), summaryText(s));
  if (s.blackbox) box.append(document.createElement("br"), `black box: ${s.blackbox}`);
  box.hidden = false;
}

// ---------- driving ----------
const driveVectors = {
  forward: () => [Number($("#speed").value), 0],
  reverse: () => [-Number($("#speed").value), 0],
  left: () => [0, Number($("#turn").value)],
  right: () => [0, -Number($("#turn").value)],
};

// Drive renewal. robotd keeps re-sending the command to the robot until
// hold_ms after the last renewal from this page, then sends STOP (in balance
// mode: zero drive, keep balancing). hold_ms was 600 ms, so one slow phone
// request (Wi-Fi power saving, a busy Pi) stopped the drive: the iPhone
// "drives for 4-8 s then stops" problem (24 Nov). Now 1500 ms, and one failed
// renewal is retried before giving up.
const DRIVE_HOLD_MS = 1500;
const DRIVE_RENEW_MS = 200;

async function renewDrive(session) {
  if (drive !== session) return;
  const [linear, angular] = driveVectors[session.direction]();
  try {
    await command({action: "velocity", linear, angular, lease_ms: 1000, hold_ms: DRIVE_HOLD_MS});
    session.failures = 0;
  } catch (error) {
    session.failures = (session.failures || 0) + 1;
    if (session.failures >= 2) {
      endDrive(false, `request failed (${error.message})`);
      return;
    }
  }
  if (drive === session) session.timer = setTimeout(() => renewDrive(session), DRIVE_RENEW_MS);
}

function beginDrive(event) {
  if (event.button !== undefined && event.button !== 0) return;
  event.preventDefault();
  endDrive(false);
  const button = event.currentTarget;
  if (button.setPointerCapture) button.setPointerCapture(event.pointerId);
  button.classList.add("active");
  drive = {direction: button.dataset.drive, button, timer: null, failures: 0, started: Date.now()};
  renewDrive(drive);
}

// reason: why the drive ended, shown briefly when it wasn't a normal release.
function endDrive(sendZero = true, reason = "") {
  const was = drive !== null;
  if (was && reason) {
    const held = ((Date.now() - drive.started) / 1000).toFixed(1);
    showToast(`Drive ended after ${held} s: ${reason}`, true);
  }
  if (drive) {
    clearTimeout(drive.timer);
    drive.button.classList.remove("active");
    drive = null;
  }
  if (sendZero && was && bal.BAL === "RUNNING") {
    command({action: "velocity", linear: 0, angular: 0, lease_ms: 500, hold_ms: 500})
      .catch((error) => showToast(error.message, true));
  }
}

// ---------- tuning ----------
async function loadTuning() {
  try {
    const data = await command({action: "tuning"});
    tuning = {saved: data.saved || {}, defaults: data.defaults || {}, ranges: data.ranges || {}};
    buildTuning();
    if (bal.KP !== undefined) fillTuning(bal);
    const savedKeys = Object.keys(tuning.saved);
    $("#tune-info").textContent = savedKeys.length
      ? `Saved start-up values: ${savedKeys.map((k) => `${k} ${tuning.saved[k]}`).join(", ")}. Re-applied whenever the SKR starts.`
      : "No saved start-up values: the SKR starts with its firmware defaults. Changes apply live, even while balancing.";
  } catch (error) {
    showToast(`Tuning: ${error.message}`, true);
  }
}

function buildTuning() {
  const grid = $("#tune-grid");
  grid.textContent = "";
  for (const key of Object.keys(tuning.defaults)) {
    const [lo, hi] = tuning.ranges[key] || ["", ""];
    const item = document.createElement("label");
    item.className = "tune-item";
    item.dataset.key = key;
    const label = document.createElement("span");
    label.textContent = TUNE_LABELS[key] || key;
    const input = document.createElement("input");
    input.type = "number";
    input.step = "any";
    input.min = lo;
    input.max = hi;
    input.addEventListener("input", renderTuningMarks);
    const live = document.createElement("small");
    live.className = "live";
    const saved = document.createElement("small");
    saved.className = "saved";
    item.append(label, input, live, saved);
    grid.append(item);
  }
}

function fillTuning(source) {
  document.querySelectorAll(".tune-item").forEach((item) => {
    const key = item.dataset.key;
    if (source[key] !== undefined) item.querySelector("input").value = Number(source[key]);
  });
  tuneFilled = true;
  renderTuningMarks();
}

function renderTuningMarks() {
  document.querySelectorAll(".tune-item").forEach((item) => {
    const key = item.dataset.key;
    const input = item.querySelector("input");
    const live = bal[key];
    const saved = tuning.saved[key];
    item.querySelector(".live").textContent = live !== undefined ? `robot: ${Number(live)}` : "";
    item.querySelector(".saved").textContent = saved !== undefined ? `saved: ${saved}` : `default: ${tuning.defaults[key]}`;
    item.classList.toggle("changed", live !== undefined && input.value !== "" &&
      Math.abs(Number(input.value) - Number(live)) > 1e-6);
    item.classList.toggle("differs-saved", saved !== undefined && live !== undefined &&
      Math.abs(Number(saved) - Number(live)) > 1e-6);
  });
}

async function applyTuning() {
  const changes = {};
  const problems = [];
  document.querySelectorAll(".tune-item").forEach((item) => {
    const key = item.dataset.key;
    const raw = item.querySelector("input").value;
    if (raw === "") return;
    const value = Number(raw);
    const live = Number(bal[key]);
    if (!isFinite(value)) { problems.push(`${key} is not a number`); return; }
    const [lo, hi] = tuning.ranges[key] || [-Infinity, Infinity];
    if (value < lo || value > hi) { problems.push(`${key} must be ${lo}–${hi}`); return; }
    if (Math.abs(value - live) < 1e-6) return;
    if (["KP", "KD", "KV", "KX"].includes(key) && live > 0 && !$("#tune-large").checked &&
        Math.abs(value - live) / live > GAIN_STEP_LIMIT) {
      problems.push(`${key}: ${live} → ${value} is more than ${GAIN_STEP_LIMIT * 100} % in one step`);
      return;
    }
    changes[key] = value;
  });
  if (problems.length) { showToast(problems.join("; "), true); return; }
  if (!Object.keys(changes).length) { showToast("Nothing to change"); return; }
  try {
    await command({action: "bal_set", set: changes});
    showToast(`Applied ${Object.entries(changes).map(([k, v]) => `${k} ${v}`).join(", ")}`);
  } catch (error) {
    showToast(error.message, true);
  }
}

async function saveTuning() {
  const values = {};
  for (const key of Object.keys(tuning.defaults)) {
    if (bal[key] !== undefined) values[key] = Number(bal[key]);
  }
  if (document.querySelector(".tune-item.changed") &&
      !window.confirm("Some fields differ from the robot and haven't been applied. Save the robot's current values anyway?")) {
    return;
  }
  try {
    const data = await command({action: "tuning_save", values});
    tuning.saved = data.saved || {};
    showToast("Saved: re-applied whenever the SKR starts");
    loadTuning();
  } catch (error) {
    showToast(error.message, true);
  }
}

// ---------- runs and plots ----------
async function loadRuns() {
  try {
    const response = await fetch("/api/runs", {cache: "no-store"});
    const data = await response.json();
    if (!data.ok) throw new Error(data.error);
    const list = $("#runs-list");
    list.textContent = "";
    for (const item of (data.runs || []).slice(0, 30)) {
      const row = document.createElement("div");
      row.className = "run-row";
      const text = document.createElement("div");
      const name = document.createElement("div");
      name.className = "run-name";
      name.textContent = item.run;
      const detail = document.createElement("div");
      detail.className = "run-detail";
      const notes = (item.notes || "").replace(/^balance_run\.py\s*/, "").replace(/^web\s*/, "");
      detail.textContent = item.balance ? summaryText(item.balance) : (notes || item.kind || "");
      text.append(name, detail);
      if (item.balance && notes) {
        const n = document.createElement("div");
        n.className = "run-detail";
        n.textContent = notes;
        text.append(n);
      }
      const actions = document.createElement("div");
      actions.className = "run-actions";
      if ((item.files || []).includes("data.csv")) {
        const plot = document.createElement("button");
        plot.className = "small-button";
        plot.type = "button";
        plot.textContent = "Plot";
        plot.addEventListener("click", () => plotRun(item.run));
        const csv = document.createElement("a");
        csv.href = `/runs/${encodeURIComponent(item.run)}/data.csv`;
        csv.textContent = "CSV";
        actions.append(plot, csv);
      }
      row.append(text, actions);
      list.append(row);
    }
    if (!list.children.length) list.textContent = "No runs yet.";
  } catch (error) {
    showToast(`Runs: ${error.message}`, true);
  }
}

async function plotRun(run) {
  $("#plot-wrap").hidden = false;
  $("#plot-title").textContent = `${run} (loading…)`;
  try {
    const response = await fetch(`/runs/${encodeURIComponent(run)}/data.csv`, {cache: "no-store"});
    if (!response.ok) throw new Error(`HTTP ${response.status}`);
    const lines = (await response.text()).trim().split("\n");
    const head = lines[0].split(",");
    const col = (name) => head.indexOf(name);
    const it = col("t_s"), ip = col("pitch_deg"), iu = col("u_mm_s2"),
          il = col("v_left_mm_s"), ir = col("v_right_mm_s"), iflag = col("flags");
    const n = lines.length - 1;
    const stride = Math.max(1, Math.floor(n / 2500));
    const t = [], pitch = [], u = [], v = [], kicks = [];
    let prevKick = false;
    for (let i = 1; i < lines.length; i += 1) {
      const f = lines[i].split(",");
      const flags = Number(f[iflag]);
      const isKick = (flags & 0xa0) === 0xa0;
      if (isKick && !prevKick) kicks.push(Number(f[it]));
      prevKick = isKick;
      if ((i - 1) % stride) continue;
      t.push(Number(f[it]));
      pitch.push(Number(f[ip]));
      u.push(Number(f[iu]) / 1000);
      v.push((Number(f[il]) + Number(f[ir])) / 2);
    }
    $("#plot-title").textContent = `${run} · ${n} samples${kicks.length ? ` · ${kicks.length} kicks` : ""}`;
    drawPlot(t, [
      {label: "pitch °", data: pitch, color: "#62e39d"},
      {label: "u m/s²", data: u, color: "#7fb7ff"},
      {label: "speed mm/s", data: v, color: "#ffc56e"},
    ], kicks);
    $("#plot-wrap").scrollIntoView({behavior: "smooth", block: "nearest"});
  } catch (error) {
    $("#plot-title").textContent = `${run}: ${error.message}`;
  }
}

function drawPlot(t, series, kicks) {
  const canvas = $("#plot");
  const ratio = window.devicePixelRatio || 1;
  const width = canvas.clientWidth;
  const height = canvas.clientHeight;
  canvas.width = Math.round(width * ratio);
  canvas.height = Math.round(height * ratio);
  const ctx = canvas.getContext("2d");
  ctx.scale(ratio, ratio);
  ctx.clearRect(0, 0, width, height);
  if (!t.length) return;
  const left = 54, right = 10, gap = 10, top = 8, bottom = 22;
  const panelH = (height - top - bottom - gap * (series.length - 1)) / series.length;
  const t0 = t[0], t1 = t[t.length - 1] || t0 + 1;
  const xOf = (x) => left + (x - t0) / (t1 - t0) * (width - left - right);
  ctx.font = "11px system-ui, sans-serif";
  series.forEach((s, k) => {
    const y0 = top + k * (panelH + gap);
    let lo = Infinity, hi = -Infinity;
    for (const value of s.data) { if (isFinite(value)) { lo = Math.min(lo, value); hi = Math.max(hi, value); } }
    if (lo === hi) { lo -= 1; hi += 1; }
    const pad = (hi - lo) * 0.08;
    lo -= pad; hi += pad;
    const yOf = (value) => y0 + (hi - value) / (hi - lo) * panelH;
    ctx.strokeStyle = "#2b3733";
    ctx.strokeRect(left, y0, width - left - right, panelH);
    if (lo < 0 && hi > 0) {
      ctx.beginPath();
      ctx.moveTo(left, yOf(0));
      ctx.lineTo(width - right, yOf(0));
      ctx.stroke();
    }
    ctx.strokeStyle = "rgba(255, 150, 60, 0.55)";
    for (const kt of kicks) {
      ctx.beginPath();
      ctx.moveTo(xOf(kt), y0);
      ctx.lineTo(xOf(kt), y0 + panelH);
      ctx.stroke();
    }
    ctx.strokeStyle = s.color;
    ctx.lineWidth = 1.2;
    ctx.beginPath();
    s.data.forEach((value, i) => {
      const x = xOf(t[i]), y = yOf(value);
      if (i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
    });
    ctx.stroke();
    ctx.lineWidth = 1;
    ctx.fillStyle = "#8fa19a";
    ctx.fillText(hi.toFixed(1), 4, y0 + 10);
    ctx.fillText(lo.toFixed(1), 4, y0 + panelH - 2);
    ctx.fillStyle = s.color;
    ctx.fillText(s.label, left + 6, y0 + 13);
  });
  ctx.fillStyle = "#8fa19a";
  ctx.fillText(`${t0.toFixed(0)} s`, left, height - 6);
  const endLabel = `${t1.toFixed(0)} s`;
  ctx.fillText(endLabel, width - right - ctx.measureText(endLabel).width, height - 6);
}

// ---------- wiring ----------
document.querySelectorAll(".drive-button").forEach((button) => {
  button.addEventListener("pointerdown", beginDrive);
  button.addEventListener("pointerup", () => endDrive());
  button.addEventListener("pointercancel", () => endDrive(true, "touch cancelled by the browser"));
  button.addEventListener("lostpointercapture", () => endDrive(true, "touch capture lost"));
  button.addEventListener("contextmenu", (event) => event.preventDefault());
});
window.addEventListener("blur", () => endDrive(true, "page lost focus"));
document.addEventListener("visibilitychange", () => { if (document.hidden) endDrive(true, "page hidden"); });

$("#speed").addEventListener("input", () => { $("#speed-out").textContent = `${$("#speed").value} mm/s`; });
$("#turn").addEventListener("input", () => { $("#turn-out").textContent = `${$("#turn").value} °/s`; });
$("#stop").addEventListener("click", () => {
  endDrive(false);
  command({action: "stop"}).then(() => showToast("Stopped")).catch((e) => showToast(e.message, true));
});
$("#estop").addEventListener("click", () => {
  endDrive(false);
  command({action: "estop"}).then(() => showToast("Emergency stop")).catch((e) => showToast(e.message, true));
});
$("#clear-lock").addEventListener("click", () => {
  if (!window.confirm("Clear the drive lock / emergency stop? Only do this with the robot safe.")) return;
  command({action: "clear_estop"}).then(() => showToast("Cleared")).catch((e) => showToast(e.message, true));
});
$("#run-kicks").addEventListener("change", () => {
  // The kick count can be changed during a run: kicks start (or stop)
  // straight away once it is balancing.
  if (!runActive) return;
  const wanted = Math.max(0, Math.min(200, parseInt($("#run-kicks").value || "0", 10)));
  kick.wanted = wanted;
  if (kick.engagedAt !== null) {
    kick.next = Math.min(kick.next, performance.now() / 1000 + 1);
    setProgress(wanted > kick.done ? `Kicks: ${kick.done}/${wanted}` : `Kicks stopped at ${kick.done}.`);
  }
});
$("#run-start").addEventListener("click", startRun);
$("#run-stop").addEventListener("click", stopRun);
$("#tune-apply").addEventListener("click", applyTuning);
$("#tune-reload").addEventListener("click", () => fillTuning(bal));
$("#tune-save").addEventListener("click", saveTuning);
$("#tune-saved").addEventListener("click", () => {
  if (!Object.keys(tuning.saved).length) { showToast("Nothing saved yet"); return; }
  fillTuning(tuning.saved);
  showToast("Saved values loaded into the fields: press Apply");
});
$("#tune-defaults").addEventListener("click", () => {
  fillTuning(tuning.defaults);
  showToast("Firmware defaults loaded into the fields: press Apply");
});
$("#runs-refresh").addEventListener("click", loadRuns);

poll();
loadTuning();
loadRuns();
