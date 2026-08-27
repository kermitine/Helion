const $ = (id) => document.getElementById(id);

let state = null;
let selectedProtocol = "private";
let busy = false;

const commandButtons = [...document.querySelectorAll("[data-command]")];
const updateButtons = [$("updateRepoBtn"), $("showUpdateLogBtn")].filter(Boolean);
const configControlIds = [
  "transportInput",
  "interfaceInput",
  "serialPortInput",
  "serialBaudInput",
  "motorIdInput",
  "hostIdInput",
  "feedbackIdInput",
  "modelInput",
];
const positionControlIds = [
  "positionTargetInput",
  "positionVelocityInput",
  "positionAccelerationInput",
];
const speedControlIds = ["speedSlider"];
const dirtyControls = new Set();

function hex(value, width = 2) {
  if (typeof value !== "number") return "--";
  return `0x${value.toString(16).toUpperCase().padStart(width, "0")}`;
}

function fixed(value, digits, suffix) {
  if (typeof value !== "number" || Number.isNaN(value)) return "--";
  return `${value.toFixed(digits)} ${suffix}`;
}

function setControlValue(id, value) {
  const el = $(id);
  if (document.activeElement !== el && !dirtyControls.has(id)) {
    el.value = value;
  }
}

function markDirty(id) {
  dirtyControls.add(id);
}

function clearDirty(ids) {
  ids.forEach((id) => dirtyControls.delete(id));
}

function clearCommandDirty(command, result) {
  if (result && result.ok === false) return;
  if (command === "move-position") clearDirty(positionControlIds);
  if (command === "set-speed") clearDirty(speedControlIds);
}

function numberInput(id) {
  const value = Number($(id).value);
  return Number.isFinite(value) ? value : 0;
}

function commandPayload(command) {
  if (command === "move-position") {
    return {
      positionRad: numberInput("positionTargetInput"),
      velocityLimit: numberInput("positionVelocityInput"),
      acceleration: numberInput("positionAccelerationInput"),
    };
  }
  return {};
}

async function post(path, payload) {
  const response = await fetch(path, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(payload),
  });
  if (!response.ok) throw new Error(await response.text());
  return response.json();
}

async function applyConfig() {
  selectedProtocol =
    $("mitProtocolBtn").classList.contains("active") ? "mit" : "private";
  await post("/api/config", {
    transport: $("transportInput").value,
    interface: $("interfaceInput").value.trim(),
    serialPort: $("serialPortInput").value.trim(),
    serialBaud: $("serialBaudInput").value.trim(),
    motorId: $("motorIdInput").value.trim(),
    hostId: $("hostIdInput").value.trim(),
    feedbackId: $("feedbackIdInput").value.trim(),
    model: $("modelInput").value,
    protocol: selectedProtocol,
  });
  clearDirty([...configControlIds, "protocol"]);
}

async function sendCommand(command, extra = {}) {
  if (busy && !["stop", "zero-speed", "clear-fault"].includes(command)) return;
  busy = true;
  renderBusy(true);
  try {
    await applyConfig();
    const result = await post("/api/command", { command, ...extra });
    if (result && result.ok === false && result.message) {
      appendLocalLog(`Command failed: ${result.message}`);
    }
    clearCommandDirty(command, result);
    await refresh();
  } catch (error) {
    appendLocalLog(`UI error: ${error.message}`);
  } finally {
    busy = false;
    renderBusy(false);
  }
}

function renderBusy(isBusy) {
  commandButtons.forEach((button) => {
    const command = button.dataset.command;
    button.disabled = isBusy && !["stop", "zero-speed", "clear-fault"].includes(command);
  });
  const updateRunning = Boolean(state && state.repo && state.repo.updateRunning);
  updateButtons.forEach((button) => {
    button.disabled = isBusy || updateRunning;
  });
}

function appendLocalLog(line) {
  const output = $("logOutput");
  output.textContent = `${output.textContent}\n${line}`.trim();
  output.scrollTop = output.scrollHeight;
}

function renderChips(id, items) {
  const el = $(id);
  el.innerHTML = "";
  if (!items || items.length === 0) {
    const empty = document.createElement("span");
    empty.className = "chip";
    empty.textContent = "--";
    el.appendChild(empty);
    return;
  }
  items.forEach((item) => {
    const chip = document.createElement("button");
    chip.className = "chip";
    chip.textContent = item;
    chip.addEventListener("click", async () => {
      $("motorIdInput").value = item;
      await applyConfig();
      await refresh();
    });
    el.appendChild(chip);
  });
}

function renderProtocol(protocol) {
  if (!dirtyControls.has("protocol")) {
    selectedProtocol = protocol;
  }
  $("privateProtocolBtn").classList.toggle("active", selectedProtocol === "private");
  $("mitProtocolBtn").classList.toggle("active", selectedProtocol === "mit");
}

function render(state) {
  setControlValue("transportInput", state.transport || "robstride-serial");
  setControlValue("interfaceInput", state.interface);
  setControlValue("serialPortInput", state.serialPort || "auto");
  setControlValue("serialBaudInput", state.serialBaud || 921600);
  setControlValue("motorIdInput", state.motorIdHex);
  setControlValue("hostIdInput", state.hostIdHex);
  setControlValue("feedbackIdInput", state.feedbackIdHex);
  setControlValue("modelInput", state.model);
  renderProtocol(state.protocol);

  const status = $("connectionStatus");
  status.textContent = state.connected ? "Online" : "Offline";
  status.className = `status-pill ${state.connected ? "online" : "offline"}`;
  $("subtitle").textContent = state.openError || state.transportLabel || `${state.interface} at 1 Mbps`;
  $("appVersion").textContent = state.appVersion ? `v${state.appVersion}` : "v--";
  $("configuredState").textContent = state.positionConfigured
    ? "Position configured"
    : state.velocityConfigured
      ? "Velocity configured"
      : "Not configured";
  $("busyState").textContent = state.busy ? "Busy" : "Idle";
  $("selectedMotor").textContent = state.motorIdHex;

  setControlValue("speedSlider", state.testSpeed);
  $("speedValue").textContent = `${Number($("speedSlider").value).toFixed(2)} rad/s`;
  setControlValue("positionTargetInput", Number(state.positionTarget || 0).toFixed(2));
  setControlValue("positionVelocityInput", Number(state.positionVelocityLimit || 1).toFixed(2));
  setControlValue("positionAccelerationInput", Number(state.positionAcceleration || 10).toFixed(1));
  $("activeReportsToggle").checked = state.activeReports;

  const feedback = state.lastFeedback;
  $("feedbackAge").textContent = feedback ? `${feedback.ageMs} ms ago` : "No feedback";
  $("positionMetric").textContent = feedback
    ? fixed(feedback.positionRad, 3, "rad")
    : "--";
  $("velocityMetric").textContent = feedback
    ? fixed(feedback.velocityRadS, 3, "rad/s")
    : fixed(state.commandedSpeed, 2, "rad/s cmd");
  $("torqueMetric").textContent = feedback ? fixed(feedback.torqueNm, 3, "Nm") : "--";
  $("tempMetric").textContent = feedback ? fixed(feedback.temperatureC, 1, "C") : "--";
  $("modeState").textContent = feedback && feedback.modeState !== null ? `mode ${feedback.modeState}` : "mode --";

  const privateFault = state.lastPrivateFault;
  const privateFaultNames = privateFault && privateFault.faults && privateFault.faults.length
    ? privateFault.faults.join(", ")
    : "";
  const privateWarningNames = privateFault && privateFault.warnings && privateFault.warnings.length
    ? privateFault.warnings.join(", ")
    : "";
  const hasPrivateFault = Boolean(privateFault && privateFault.faultRaw);
  const hasPrivateWarning = Boolean(privateFault && privateFault.warningRaw);
  $("faultState").textContent = privateFault
    ? `fault ${privateFaultNames || privateFault.faultRawHex}`
    : feedback
      ? `fault ${feedback.fault ? 1 : 0}`
      : "fault --";
  $("faultState").title = privateFault ? privateFault.faultRawHex : "";
  $("faultState").classList.toggle("fault", hasPrivateFault || Boolean(feedback && feedback.fault));
  $("warningState").textContent = privateFault
    ? `warn ${privateWarningNames || privateFault.warningRawHex}`
    : feedback
      ? `warn ${feedback.warning ? 1 : 0}`
      : "warn --";
  $("warningState").title = privateFault ? privateFault.warningRawHex : "";
  $("warningState").classList.toggle("warn", hasPrivateWarning || Boolean(feedback && feedback.warning));

  const stats = state.canStats || {};
  $("rxPackets").textContent = stats.rx_packets || "--";
  $("txPackets").textContent = stats.tx_packets || "--";
  $("rxErrors").textContent = stats.rx_errors || "--";
  $("txErrors").textContent = stats.tx_errors || "--";
  $("droppedPackets").textContent = `${stats.rx_dropped || "--"} / ${stats.tx_dropped || "--"}`;

  const frame = state.lastRawFrame;
  $("lastFrame").textContent = frame
    ? `${frame.kind} ${frame.idHex} dlc=${frame.dlc}\n${frame.dataHex}`
    : "No CAN frame yet";

  const repo = state.repo || {};
  $("repoBranch").textContent = repo.branch || "--";
  $("repoCommit").textContent = repo.commit || "--";
  $("repoDirty").textContent =
    typeof repo.dirtyCount === "number" ? `${repo.dirtyCount} file(s)` : "--";
  $("repoDirty").classList.toggle("warn", Boolean(repo.dirty));
  $("repoRemote").textContent = repo.remote || "No git remote found";
  if (repo.updateRunning) {
    $("updateState").textContent = "Updating";
  } else if (repo.updateLastExit === 0) {
    $("updateState").textContent = "Updated";
  } else if (typeof repo.updateLastExit === "number") {
    $("updateState").textContent = "Update failed";
  } else {
    $("updateState").textContent = "Ready";
  }
  $("updateState").classList.toggle("warn", Boolean(repo.updateRunning));
  $("updateState").classList.toggle(
    "fault",
    typeof repo.updateLastExit === "number" && repo.updateLastExit !== 0
  );
  $("updateRepoBtn").disabled = Boolean(repo.updateRunning || busy);

  renderChips("privateList", state.discoveredPrivate);
  renderChips("mitList", state.discoveredMit);

  $("logOutput").textContent = (state.logs || []).join("\n");
  $("logOutput").scrollTop = $("logOutput").scrollHeight;
}

async function refresh() {
  const response = await fetch("/api/state", { cache: "no-store" });
  state = await response.json();
  render(state);
}

document.querySelectorAll("[data-protocol]").forEach((button) => {
  button.addEventListener("click", async () => {
    markDirty("protocol");
    renderProtocol(button.dataset.protocol);
    await applyConfig();
    await refresh();
  });
});

commandButtons.forEach((button) => {
  button.addEventListener("click", () => {
    const command = button.dataset.command;
    sendCommand(command, commandPayload(command));
  });
});

configControlIds.forEach((id) => {
  const el = $(id);
  el.addEventListener("input", () => markDirty(id));
  el.addEventListener("change", () => markDirty(id));
});

positionControlIds.forEach((id) => {
  const el = $(id);
  el.addEventListener("input", () => markDirty(id));
  el.addEventListener("change", () => markDirty(id));
});

$("speedSlider").addEventListener("input", () => {
  markDirty("speedSlider");
  $("speedValue").textContent = `${Number($("speedSlider").value).toFixed(2)} rad/s`;
});

$("speedSlider").addEventListener("change", () => {
  sendCommand("set-speed", { speed: Number($("speedSlider").value) });
});

$("activeReportsToggle").addEventListener("change", () => {
  sendCommand("active-report", { enabled: $("activeReportsToggle").checked });
});

$("clearLogBtn").addEventListener("click", () => {
  $("logOutput").textContent = "";
  post("/api/logs/clear", {}).then(refresh).catch((error) => {
    appendLocalLog(`UI error: ${error.message}`);
  });
});

$("updateRepoBtn").addEventListener("click", async () => {
  if (!confirm("Pull the current GitHub branch and restart the dashboard?")) return;
  busy = true;
  renderBusy(true);
  try {
    const result = await post("/api/update", { remote: "origin" });
    appendLocalLog(result.ok ? `Update started pid=${result.pid}` : `Update failed: ${result.message}`);
    setTimeout(refresh, 1000);
  } catch (error) {
    appendLocalLog(`UI error: ${error.message}`);
  } finally {
    busy = false;
    renderBusy(false);
  }
});

$("showUpdateLogBtn").addEventListener("click", async () => {
  try {
    const result = await fetch("/api/update/log", { cache: "no-store" }).then((r) => r.json());
    $("logOutput").textContent = result.log || "No update log yet";
    $("logOutput").scrollTop = $("logOutput").scrollHeight;
  } catch (error) {
    appendLocalLog(`UI error: ${error.message}`);
  }
});

refresh();
setInterval(refresh, 500);
