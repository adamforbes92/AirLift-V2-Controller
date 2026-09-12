document.addEventListener("DOMContentLoaded", init);

let setupCache = null;
let useBar = false;

function fmt(psi) {
  if (psi == null) return "--";
  if (useBar) return (psi * 0.0689476).toFixed(2);
  return psi;
}

const CORNER_NAMES = ["FL", "FR", "RL", "RR", "ALL"];
const DIR_NAMES    = ["up", "down"];

async function saveSetting(key, value) {
  try {
    const res = await fetch("/api/settings", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ [key]: value }),
    });
    const r = await res.json().catch(() => ({}));
    if (!r.ok) showNotif("Failed to save " + key, true);
  } catch (e) {
    showNotif("Save error: " + e.message, true);
  }
}

function showNotif(msg, isErr) {
  const el = document.getElementById("settingsStatus");
  if (!el) return;
  el.textContent = msg;
  el.style.color = isErr ? "#e55" : "";
  clearTimeout(showNotif._t);
  showNotif._t = setTimeout(() => { el.textContent = ""; el.style.color = ""; }, 2000);
}

function init() {
  document.querySelectorAll(".tab").forEach((t) => {
    t.addEventListener("click", () => {
      document.querySelectorAll(".tab").forEach((x) => x.classList.remove("active"));
      t.classList.add("active");
      document.querySelectorAll(".panel").forEach((p) => p.classList.remove("active"));
      document.getElementById("tab-" + t.dataset.tab).classList.add("active");
    });
  });

  document.querySelectorAll("#tab-overview .corner-btns button").forEach((b) => {
    const c   = parseInt(b.dataset.corner, 10);
    const d   = parseInt(b.dataset.dir, 10);
    const cor = CORNER_NAMES[c];
    const dir = DIR_NAMES[d];
    attachHoldButton(b, cor, dir);
  });

  const modeSel = document.getElementById("overviewMode");
  if (modeSel) {
    modeSel.addEventListener("change", () => {
      applyOverviewMode(modeSel.value);
      jpost("/api/intercept/modeswitch", { mode: modeSel.value === "preset" ? "preset" : "manual" });
    });
    applyOverviewMode(modeSel.value);
  }

  initSettings();

  document.getElementById("savePresetsBtn").addEventListener("click", savePresets);
  document.getElementById("otaUploadBtn").addEventListener("click", uploadOta);
  {
    const typeSel = document.getElementById("otaType");
    if (typeSel) {
      const done = otaLoadDone();
      if (done.includes("filesystem") && !done.includes("firmware")) typeSel.value = "firmware";
      typeSel.addEventListener("change", renderOtaSteps);
    }
    renderOtaSteps();
  }

  const learnStartBtn = document.getElementById("learnPresetsBtn");
  if (learnStartBtn) learnStartBtn.addEventListener("click", startLearn);
  const learnStopBtn = document.getElementById("learnStopBtn");
  if (learnStopBtn) learnStopBtn.addEventListener("click", stopLearn);
  const learnSaveBtn = document.getElementById("learnSaveBtn");
  if (learnSaveBtn) learnSaveBtn.addEventListener("click", saveLearn);

  const useBarCb = document.getElementById("useBar");
  if (useBarCb) {
    useBar = localStorage.getItem("useBar") === "1";
    useBarCb.checked = useBar;
    document.getElementById("pressureUnitLabel").textContent = useBar ? "bar" : "PSI";
    useBarCb.addEventListener("change", () => {
      useBar = useBarCb.checked;
      localStorage.setItem("useBar", useBar ? "1" : "0");
      document.getElementById("pressureUnitLabel").textContent = useBar ? "bar" : "PSI";
    });
  }

  document.getElementById("logClearBtn").addEventListener("click", async () => {
    await fetch("/api/log/clear", { method: "POST" });
    logSince = 0;
    document.getElementById("logView").textContent = "";
  });

  const highSideOverride = document.getElementById("highSideOverride");
  if (highSideOverride) {
    highSideOverride.addEventListener("change", async () => {
      const r = await jpost("/api/diagnostics/high-side", { on: highSideOverride.checked });
      if (!r.ok) showNotif("Could not change high-side GPIO", true);
      loadStatus();
    });
  }

  loadSetup();
  loadStatus();
  setInterval(loadStatus, 250);
  setInterval(pollLog, 750);
}

function initSettings() {
  // Checkboxes — save immediately on change
  const checkboxIds = ["passThroughMode", "airOutOnIgnOff", "ignitionSenseGpio", "canBroadcastEnabled",
    "airUpOnFobDouble", "airDownOnFobDouble"];
  checkboxIds.forEach((id) => {
    const el = document.getElementById(id);
    if (el) el.addEventListener("change", () => saveSetting(id, el.checked));
  });

  const canSourcePair = ["usePowertrainCan", "useComfortCan"];
  [[canSourcePair[0], canSourcePair[1]], [canSourcePair[1], canSourcePair[0]]].forEach(([firstId, secondId]) => {
    const first = document.getElementById(firstId);
    const second = document.getElementById(secondId);
    if (!first) return;
    first.addEventListener("change", () => {
      if (second) second.checked = !first.checked;
      saveSetting(first.id, first.checked);
    });
  });

  const savvyPair = ["savvyCanWifiEnabled", "savvyCanSerialEnabled"];
  [[savvyPair[0], savvyPair[1]], [savvyPair[1], savvyPair[0]]].forEach(([firstId, secondId]) => {
    const first = document.getElementById(firstId);
    const second = document.getElementById(secondId);
    if (!first) return;
    first.addEventListener("change", () => {
      if (first.checked && second) second.checked = false;
      saveSetting(first.id, first.checked);
    });
  });

  // Selects — save immediately on change
  const selectMap = {
    airUpPreset: (v) => parseInt(v, 10),
    airDownPreset: (v) => parseInt(v, 10),
  };
  Object.entries(selectMap).forEach(([id, parse]) => {
    const el = document.getElementById(id);
    if (el) el.addEventListener("change", () => saveSetting(id, parse(el.value)));
  });

  // Range sliders — save on release (change event)
  const rangeMap = {
    canSilenceSecRange: { key: "canSilenceSec" },
    canMinFpsRange:     { key: "canMinFps" },
    controllerBootDelayRange: { key: "controllerBootDelayMs", fmt: (v) => (v / 1000).toFixed(1) },
  };
  Object.entries(rangeMap).forEach(([elemId, { key, fmt }]) => {
    const rng = document.getElementById(elemId);
    const val = document.getElementById(elemId.replace("Range", "Value"));
    if (rng) {
      rng.addEventListener("input", () => { if (val) val.textContent = fmt ? fmt(rng.value) : rng.value; });
      rng.addEventListener("change", () => saveSetting(key, parseInt(rng.value, 10)));
    }
  });

  // CAN broadcast ID — save on blur
  const bcId = document.getElementById("canBroadcastIdHex");
  if (bcId) {
    bcId.addEventListener("change", () => {
      let id = parseInt(bcId.value, 16);
      if (!Number.isFinite(id) || id < 0) id = 0;
      if (id > 0x7FF) id = 0x7FF;
      saveSetting("canBroadcastId", id);
    });
  }

  ["airUpFrontPsi", "airUpRearPsi", "airDownFrontPsi", "airDownRearPsi"].forEach((id) => {
    const el = document.getElementById(id);
    if (el) el.addEventListener("change", () => {
      const value = Math.max(0, Math.min(200, parseInt(el.value, 10) || 0));
      el.value = value;
      saveSetting(id, value);
    });
  });
}

async function jpost(url, body) {
  const res = await fetch(url, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(body || {}),
  });
  return res.json().catch(() => ({}));
}

async function loadStatus() {
  try {
    const s = await (await fetch("/api/status")).json();
    const fwEl = document.getElementById("fwVersion");
    if (fwEl && s.fwVersion) fwEl.textContent = "v" + s.fwVersion;
    const stale = s.linActive === false;   // no LIN traffic for >5s
    const psi = (v) => stale ? "--" : fmt(v);
    document.getElementById("pFL").textContent = psi(s.psiFL);
    document.getElementById("pFR").textContent = psi(s.psiFR);
    document.getElementById("pRL").textContent = psi(s.psiRL);
    document.getElementById("pRR").textContent = psi(s.psiRR);
    document.getElementById("pTank").textContent = psi(s.psiTank);
    const comp = document.getElementById("compIndicator");
    if (comp) {
      if (stale) {
        comp.textContent = "Compressor: --";
        comp.className = "comp-pill comp-off";
      } else {
        const on  = !!s.compressor;
        comp.textContent = "Compressor: " + (on ? "On" : "Off");
        comp.className = "comp-pill " + (on ? "comp-on" : "comp-off");
      }
    }

    const ign = document.getElementById("ignBadge");
    ign.textContent = "Ignition: " + (s.ignition ? "ON" : "OFF");
    ign.className = "badge " + (s.ignition ? "on" : "off");

    const pwr = document.getElementById("pwrBadge");
    pwr.textContent = "Controller: " + (s.controllerPowered ? "ON" : "OFF");
    pwr.className = "badge " + (s.controllerPowered ? "on" : "off");

    document.getElementById("lastBtn").textContent = s.lastBtn || "--";
    document.getElementById("lastMan").textContent = s.lastMan || "--";
    document.getElementById("canFrames").textContent = s.framesC2M != null ? (s.canActive ? "active" : "idle") : "--";
    const highSideStatus = document.getElementById("highSideStatus");
    if (highSideStatus) {
      const on = !!s.controllerPowered;
      highSideStatus.textContent = (on ? "On" : "Off") + (s.highSideForced ? " (forced)" : " (auto)");
      highSideStatus.className = "value " + (on ? "ok" : "bad");
    }
    const canStatus = document.getElementById("canStatus");
    if (canStatus) {
      const running = !!s.canDriverRunning;
      canStatus.textContent = running ? "Listening" : "Stopped";
      canStatus.className = "value " + (running ? "ok" : "bad");
    }
    const canTrafficStatus = document.getElementById("canTrafficStatus");
    if (canTrafficStatus) {
      const active = !!s.canActive;
      const frames = s.canFramesSeen ?? 0;
      canTrafficStatus.textContent = active ? `${frames} frames` : "No frames";
      canTrafficStatus.className = "value " + (active ? "ok" : "warn");
    }
    const highSideOverride = document.getElementById("highSideOverride");
    // Reflect the OVERRIDE (force-on) state, not the live power state, so the
    // toggle doesn't fight the user: unchecked = auto (ignition/CAN control).
    if (highSideOverride) highSideOverride.checked = !!(s.highSideForced && s.highSideForcedOn);
    document.getElementById("rxBytesC").textContent  = s.rxBytesC  ?? "--";
    document.getElementById("rxBytesM").textContent  = s.rxBytesM  ?? "--";
    document.getElementById("framesC2M").textContent = s.framesC2M ?? "--";
    document.getElementById("framesM2C").textContent = s.framesM2C ?? "--";
    const liveFps = document.getElementById("canLiveFps");
    if (liveFps) liveFps.textContent = s.canFps != null ? s.canFps : "--";
    const bcSent = document.getElementById("canBcSent");
    if (bcSent) bcSent.textContent = s.canBroadcastSent ?? "--";
    const bcErr  = document.getElementById("canBcErrors");
    if (bcErr)  bcErr.textContent  = s.canBroadcastErrors ?? "--";
    const comfortLock = document.getElementById("comfortLockState");
    if (comfortLock) comfortLock.textContent = s.comfortLockState || "Unknown";
    const savvyDrops = document.getElementById("savvyCanDrops");
    if (savvyDrops) savvyDrops.textContent = s.savvyCanFramesDropped ?? "--";

    // Bus health (per LIN side). "Healthy" = normal traffic; else "No traffic".
    const setHealth = (id, healthy) => {
      const el = document.getElementById(id);
      if (!el) return;
      el.textContent = healthy ? "Healthy" : "No traffic";
      el.className = "value " + (healthy ? "ok" : "bad");
    };
    setHealth("busHealthH", !!s.handheldHealthy);
    setHealth("busHealthM", !!s.manifoldHealthy);
    const wiring = document.getElementById("busWiring");
    if (wiring) {
      wiring.textContent = s.busReversed ? "Reversed (auto-corrected)" : "Normal";
      wiring.className = "value " + (s.busReversed ? "warn" : "ok");
    }
  } catch (e) {}
}

async function loadSetup() {
  try {
    const s = await (await fetch("/api/setup")).json();
    setupCache = s;
    renderPresets(s.presets || []);
    renderPresetButtons(s.presets || []);
    renderSettings(s);
  } catch (e) {}
}

function hex(s) { return s && s.length ? s : "(empty)"; }

function renderPresets(rows) {
  const tbody = document.getElementById("presetRows");
  tbody.innerHTML = "";
  rows.forEach((r) => {
    const tr = document.createElement("tr");
    tr.innerHTML = `
      <td>${r.index + 1}</td>
      <td><input type="text" data-field="name" data-idx="${r.index}" value="${escapeHtml(r.name)}"></td>
      <td><input type="number" min="0" max="255" data-field="front" data-idx="${r.index}" value="${r.frontPsi | 0}"></td>
      <td><input type="number" min="0" max="255" data-field="rear"  data-idx="${r.index}" value="${r.rearPsi  | 0}"></td>`;
    tbody.appendChild(tr);
  });
}

async function savePresets() {
  const tbody = document.getElementById("presetRows");
  const rows  = Array.from(tbody.querySelectorAll("tr"));
  const presets = rows.map((tr) => {
    const name  = tr.querySelector('input[data-field="name"]').value;
    const front = parseInt(tr.querySelector('input[data-field="front"]').value, 10) || 0;
    const rear  = parseInt(tr.querySelector('input[data-field="rear"]').value,  10) || 0;
    return { name, frontPsi: front, rearPsi: rear };
  });
  const r = await jpost("/api/presets", { presets });
  document.getElementById("presetsStatus").textContent = r.ok ? "Saved." : "Save failed.";
  setTimeout(() => (document.getElementById("presetsStatus").textContent = ""), 2000);
  if (r.ok) loadSetup();
}

// ---- Learn presets from the physical controller ----------------------------
let learnTimer = null;

async function startLearn() {
  const r = await jpost("/api/learn/start", {});
  if (!r.ok) return;
  document.getElementById("learnCard").style.display = "";
  document.getElementById("learnStatus").textContent = "";
  pollLearn();
  if (learnTimer) clearInterval(learnTimer);
  learnTimer = setInterval(pollLearn, 700);
}

async function pollLearn() {
  let s;
  try { s = await (await fetch("/api/learn")).json(); } catch (e) { return; }
  const rem = document.getElementById("learnRemain");
  if (rem) rem.textContent = s.active ? `(${Math.ceil((s.remainMs || 0) / 1000)}s left)` : "(idle)";
  const tbody = document.getElementById("learnRows");
  tbody.innerHTML = "";
  (s.slots || []).forEach((c) => {
    const armed = s.armedSlot === c.slot;
    const tr = document.createElement("tr");
    if (armed) tr.className = "armed";
    const front = c.captured ? c.front : "—";
    const rear  = c.captured ? c.rear  : "—";
    const label = armed ? "Press it…" : (c.captured ? "Re-capture" : "Capture");
    tr.innerHTML =
      `<td>${c.slot + 1}</td><td>${front}</td><td>${rear}</td>` +
      `<td><button class="btn small secondary" data-arm="${c.slot}">${label}</button></td>`;
    tbody.appendChild(tr);
  });
  tbody.querySelectorAll("button[data-arm]").forEach((b) => {
    b.addEventListener("click", () => armLearnSlot(parseInt(b.dataset.arm, 10)));
  });
}

async function armLearnSlot(slot) {
  const r = await jpost("/api/learn/arm", { slot });
  const st = document.getElementById("learnStatus");
  if (st) st.textContent = r.ok
    ? `Armed slot ${slot + 1} — press that preset on the controller now.`
    : "Could not arm slot.";
  pollLearn();
}

async function stopLearn() {
  if (learnTimer) { clearInterval(learnTimer); learnTimer = null; }
  await jpost("/api/learn/stop", {});
  document.getElementById("learnCard").style.display = "none";
}

async function saveLearn() {
  const r = await jpost("/api/learn/save", {});
  const st = document.getElementById("learnStatus");
  st.textContent = r.ok ? "Saved to presets." : "Nothing captured / save failed.";
  if (r.ok) {
    if (learnTimer) { clearInterval(learnTimer); learnTimer = null; }
    await jpost("/api/learn/stop", {});
    setTimeout(() => {
      document.getElementById("learnCard").style.display = "none";
      st.textContent = "";
    }, 1500);
    loadSetup();
  }
}

function renderSettings(s) {
  const cb = (id, val) => { const el = document.getElementById(id); if (el) el.checked = !!val; };
  cb("airOutOnIgnOff",    s.airOutOnIgnOff);
  cb("ignitionSenseGpio", s.ignitionSenseGpio);
  cb("airUpOnFobDouble", s.airUpOnFobDouble);
  cb("airDownOnFobDouble", s.airDownOnFobDouble);
  cb("passThroughMode",   s.passThroughMode);
  cb("canBroadcastEnabled", s.canBroadcastEnabled);
  cb("usePowertrainCan", s.usePowertrainCan);
  cb("useComfortCan", s.useComfortCan);
  cb("savvyCanWifiEnabled", s.savvyCanWifiEnabled);
  cb("savvyCanSerialEnabled", s.savvyCanSerialEnabled);
  setSlider("canSilenceSecRange", "canSilenceSecValue", s.canSilenceSec);
  setSlider("canMinFpsRange",     "canMinFpsValue",     s.canMinFps);
  if (s.controllerBootDelayMs != null) {
    setSlider("controllerBootDelayRange", "controllerBootDelayValue", s.controllerBootDelayMs);
    const bdVal = document.getElementById("controllerBootDelayValue");
    if (bdVal) bdVal.textContent = (s.controllerBootDelayMs / 1000).toFixed(1);
  }
  const bcId = document.getElementById("canBroadcastIdHex");
  if (bcId && s.canBroadcastId != null) {
    bcId.value = (s.canBroadcastId & 0x7FF).toString(16).toUpperCase().padStart(3, "0");
  }
  ["airUpFrontPsi", "airUpRearPsi", "airDownFrontPsi", "airDownRearPsi"].forEach((id) => {
    const el = document.getElementById(id);
    if (el && s[id] != null) el.value = s[id];
  });
  ["airUpPreset", "airDownPreset"].forEach((id) => {
    const sel = document.getElementById(id);
    if (!sel) return;
    if (sel.options.length !== 9) {
      sel.innerHTML = "";
      for (let i = 0; i < 8; i++) {
        const opt = document.createElement("option");
        opt.value = i;
        sel.appendChild(opt);
      }
      const manual = document.createElement("option");
      manual.value = 255;
      manual.textContent = "Manual pressure target";
      sel.appendChild(manual);
    }
    Array.from(sel.options).forEach((option) => {
      const index = parseInt(option.value, 10);
      if (index < 8) option.textContent = (index + 1) + ". " + (s.presets?.[index]?.name || "Preset " + (index + 1));
    });
    sel.value = s[id] != null ? s[id] : 255;
  });
}

async function triggerManual(corner, dir, holdMs) {
  await jpost("/api/intercept/manual", { corner, dir, action: "tap", holdMs: holdMs || 250 });
}

// Wire up a button to mirror real-handheld press semantics:
//   pointerdown -> tell firmware to start sending the manual poll,
//                  then ping every 750 ms so the 1.5 s safety window stays open
//   pointerup / pointercancel / pointerleave -> tell firmware to release
//   (also a hard safety: if the page hides we release immediately)
function attachHoldButton(btn, corner, dir) {
  let heartbeat = null;
  let pressed   = false;

  const release = () => {
    if (!pressed) return;
    pressed = false;
    if (heartbeat) { clearInterval(heartbeat); heartbeat = null; }
    btn.classList.remove("active-press");
    jpost("/api/intercept/manual", { corner, dir, action: "release" });
  };

  const press = (e) => {
    e.preventDefault();
    if (pressed) return;
    pressed = true;
    btn.classList.add("active-press");
    btn.setPointerCapture?.(e.pointerId);
    console.log("press", corner, dir);
    jpost("/api/intercept/manual", { corner, dir, action: "press" });
    heartbeat = setInterval(() => {
      jpost("/api/intercept/manual", { corner, dir, action: "press" });
    }, 750);
  };

  btn.addEventListener("pointerdown",   press);
  btn.addEventListener("pointerup",     release);
  btn.addEventListener("pointercancel", release);
  // Lose-focus safety nets.
  window.addEventListener("blur",       release);
  document.addEventListener("visibilitychange", () => {
    if (document.hidden) release();
  });
}

async function triggerPreset(index) {
  const btn = document.querySelector(`#presetButtonGrid button[data-index="${index}"]`);
  if (btn) btn.classList.add("active-press");
  const r = await jpost("/api/intercept/preset", { index });
  if (btn) setTimeout(() => btn.classList.remove("active-press"), 400);
  const el = document.getElementById("overviewStatus");
  if (el) { el.textContent = r.ok ? "Preset sent." : (r.error || "Failed."); setTimeout(() => { el.textContent = ""; }, 2000); }
}

function escapeHtml(s) {
  return String(s || "").replace(/[&<>"']/g, (c) => ({
    "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;", "'": "&#39;"
  }[c]));
}

function setSlider(rangeId, valId, value) {
  const rng = document.getElementById(rangeId);
  const val = document.getElementById(valId);
  if (rng != null && value != null) rng.value = value;
  if (val != null && value != null) val.textContent = value;
}

function applyOverviewMode(mode) {
  const m = document.getElementById("manualControls");
  const p = document.getElementById("presetControls");
  if (!m || !p) return;
  const usePreset = (mode === "preset");
  m.classList.toggle("hidden", usePreset);
  p.classList.toggle("hidden", !usePreset);
}

function renderPresetButtons(rows) {
  const grid = document.getElementById("presetButtonGrid");
  if (!grid) return;
  grid.innerHTML = "";
  rows.forEach((r) => {
    const configured = (r.frontPsi | 0) > 0 || (r.rearPsi | 0) > 0;
    const btn = document.createElement("button");
    btn.className = "btn preset-btn" + (configured ? "" : " empty");
    btn.dataset.index = r.index;
    const labelName = r.name || ("Preset " + (r.index + 1));
    btn.innerHTML = `<div class="preset-name">${(r.index + 1)}. ${escapeHtml(labelName)}</div>`
      + `<div class="preset-psi">${(r.frontPsi | 0)}/${(r.rearPsi | 0)} psi</div>`;
    btn.addEventListener("click", () => triggerPreset(r.index));
    grid.appendChild(btn);
  });
}

async function uploadOta() {
  const fileInput = document.getElementById("otaBin");
  const type = (document.getElementById("otaType") || {}).value || "firmware";
  if (!fileInput.files || !fileInput.files.length) { setOta("Pick a .bin first."); return; }
  const file = fileInput.files[0];
  if (!file.name.toLowerCase().endsWith(".bin")) { setOta("Please choose a .bin file."); return; }
  const isFs = type === "filesystem";
  const url = isFs ? "/api/ota/fs" : "/api/ota";
  const field = isFs ? "filesystem" : "firmware";
  const fd = new FormData(); fd.append(field, file, file.name);
  const wrap = document.getElementById("otaProgressWrap");
  const bar = document.getElementById("otaProgressBar");
  const label = document.getElementById("otaProgressLabel");
  const btn = document.getElementById("otaUploadBtn");
  if (btn) btn.disabled = true;
  if (wrap) wrap.hidden = false;
  if (bar) bar.style.width = "0%"; if (label) label.textContent = "0%";
  setOta("Uploading " + type + "\u2026");
  const xhr = new XMLHttpRequest();
  xhr.open("POST", url);
  xhr.upload.addEventListener("progress", (e) => {
    if (!e.lengthComputable) return;
    const p = Math.round((e.loaded / e.total) * 100);
    if (bar) bar.style.width = p + "%"; if (label) label.textContent = p + "%";
  });
  xhr.addEventListener("load", () => {
    let ok = false;
    try { const j = JSON.parse(xhr.responseText); ok = j.ok === true || j.success === true; }
    catch (e) { ok = xhr.status === 200; }
    if (ok) {
      if (bar) bar.style.width = "100%"; if (label) label.textContent = "100%";
      otaMarkDone(type);
      if (isFs) {
        // filesystem does not reboot — advance to the firmware step
        const sel = document.getElementById("otaType");
        if (sel) sel.value = "firmware";
        renderOtaSteps();
        setOta("Filesystem updated. Now upload the firmware.");
        if (wrap) wrap.hidden = true;
        if (btn) btn.disabled = false;
        if (fileInput) fileInput.value = "";
      } else {
        setOta("Done. Rebooting\u2026");
      }
    }
    else { setOta("Failed."); if (wrap) wrap.hidden = true; if (btn) btn.disabled = false; }
  });
  xhr.addEventListener("error", () => { setOta("Upload error. Retry."); if (wrap) wrap.hidden = true; if (btn) btn.disabled = false; });
  xhr.send(fd);
}
function setOta(t) { document.getElementById("otaStatus").textContent = t; }

// OTA two-step sequence: filesystem first, then firmware. Completed steps
// persist in localStorage so the highlight survives the firmware reboot.
const OTA_STEPS_KEY = "oh_ota_steps";
function otaLoadDone() {
  try {
    const raw = JSON.parse(localStorage.getItem(OTA_STEPS_KEY) || "{}");
    if (!raw.ts || Date.now() - raw.ts > 15 * 60 * 1000) return [];
    return Array.isArray(raw.done) ? raw.done : [];
  } catch (e) { return []; }
}
function otaSaveDone(done) {
  localStorage.setItem(OTA_STEPS_KEY, JSON.stringify({ done, ts: Date.now() }));
}
function renderOtaSteps() {
  const sel = document.getElementById("otaType");
  const cur = sel ? sel.value : "filesystem";
  const done = otaLoadDone();
  document.querySelectorAll("#otaSteps .ota-step").forEach((el) => {
    const s = el.dataset.step;
    el.classList.toggle("done", done.includes(s));
    el.classList.toggle("active", s === cur && !done.includes(s));
  });
}
function otaMarkDone(type) {
  const done = otaLoadDone();
  if (!done.includes(type)) done.push(type);
  otaSaveDone(done);
  renderOtaSteps();
}

// Populate the OTA device-info card from the common /api/ota/info endpoint.
document.addEventListener("DOMContentLoaded", () => {
  fetch("/api/ota/info").then((r) => r.json()).then((i) => {
    const set = (id, v) => { const e = document.getElementById(id); if (e) e.textContent = v || "--"; };
    set("otaFwVersion", i.version); set("otaHardware", i.hardware); set("otaBoard", i.board);
  }).catch(() => {});
});

// ---------- Diagnostic log ----------
let logSince = 0;

async function pollLog() {
  try {
    const r = await (await fetch("/api/log?since=" + logSince)).json();
    if (!r || !Array.isArray(r.entries) || r.entries.length === 0) return;
    const view = document.getElementById("logView");
    const auto = document.getElementById("logAutoScroll").checked;
    const lines = r.entries.map((e) => {
      const ts = (e.ms / 1000).toFixed(3).padStart(10, " ");
      return `[${ts}] ${e.t}`;
    });
    view.textContent += (view.textContent ? "\n" : "") + lines.join("\n");
    // Trim to last 5000 chars so the DOM doesn't grow without bound.
    if (view.textContent.length > 50000) {
      view.textContent = view.textContent.slice(-40000);
    }
    if (auto) view.scrollTop = view.scrollHeight;
    logSince = r.writeIndex;
  } catch (e) {}
}
