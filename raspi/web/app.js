const $ = (id) => document.getElementById(id);

let state = null;
let busy = false;
let ikViewYaw = -0.72;
let ikViewPitch = 0.58;
let ikDrag = null;

const commandButtons = [...document.querySelectorAll("[data-command]")];
const updateButtons = [$("updateRepoBtn"), $("showUpdateLogBtn")].filter(Boolean);
const configControlIds = [
  "serialPortInput",
  "serialBaudInput",
  "motorIdInput",
  "hostIdInput",
  "modelInput",
];
const positionControlIds = [
  "positionTargetInput",
  "positionVelocityInput",
  "positionAccelerationInput",
  "positionKpInput",
];
const armControlIds = [
  "armBaseMotorIdInput",
  "armShoulderMotorIdInput",
  "armElbowMotorIdInput",
  "armLink1Input",
  "armLink2Input",
  "armElbowUpToggle",
  "armTargetXInput",
  "armTargetYInput",
  "armTargetZInput",
  "armVelocityInput",
  "armAccelerationInput",
  "armKpInput",
  "armBaseOffsetInput",
  "armBaseDirectionInput",
  "armShoulderOffsetInput",
  "armShoulderDirectionInput",
  "armElbowOffsetInput",
  "armElbowDirectionInput",
];
const speedControlIds = ["speedSlider"];
const dirtyControls = new Set();

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

function setControlChecked(id, checked) {
  const el = $(id);
  if (document.activeElement !== el && !dirtyControls.has(id)) {
    el.checked = Boolean(checked);
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
  if (command === "arm-move") clearDirty(armControlIds);
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
      positionKp: numberInput("positionKpInput"),
    };
  }
  if (command === "arm-move") {
    return {
      armBaseMotorId: $("armBaseMotorIdInput").value.trim(),
      armShoulderMotorId: $("armShoulderMotorIdInput").value.trim(),
      armElbowMotorId: $("armElbowMotorIdInput").value.trim(),
      armLink1: numberInput("armLink1Input"),
      armLink2: numberInput("armLink2Input"),
      armElbowUp: $("armElbowUpToggle").checked,
      armTargetX: numberInput("armTargetXInput"),
      armTargetY: numberInput("armTargetYInput"),
      armTargetZ: numberInput("armTargetZInput"),
      armVelocityLimit: numberInput("armVelocityInput"),
      armAcceleration: numberInput("armAccelerationInput"),
      armPositionKp: numberInput("armKpInput"),
      armBaseOffset: numberInput("armBaseOffsetInput"),
      armBaseDirection: $("armBaseDirectionInput").value,
      armShoulderOffset: numberInput("armShoulderOffsetInput"),
      armShoulderDirection: $("armShoulderDirectionInput").value,
      armElbowOffset: numberInput("armElbowOffsetInput"),
      armElbowDirection: $("armElbowDirectionInput").value,
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
  await post("/api/config", {
    serialPort: $("serialPortInput").value.trim(),
    serialBaud: $("serialBaudInput").value.trim(),
    motorId: $("motorIdInput").value.trim(),
    hostId: $("hostIdInput").value.trim(),
    model: $("modelInput").value,
  });
  clearDirty(configControlIds);
}

async function sendCommand(command, extra = {}) {
  if (busy && !["stop", "zero-speed", "clear-fault", "arm-stop", "arm-clear-fault"].includes(command)) return;
  busy = true;
  renderBusy(true);
  try {
    await applyConfig();
    const result = await post("/api/command", { command, ...extra });
    if (result && result.ok === false && result.message) {
      appendLocalLog(`Command failed: ${result.message}`);
    } else if (result && result.message) {
      appendLocalLog(result.message);
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
    button.disabled = isBusy && !["stop", "zero-speed", "clear-fault", "arm-stop", "arm-clear-fault"].includes(command);
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

function armInputState() {
  const direction = (id) => (Number($(id).value) < 0 ? -1 : 1);
  return {
    link1: Math.max(Math.abs(numberInput("armLink1Input")), 0.001),
    link2: Math.max(Math.abs(numberInput("armLink2Input")), 0.001),
    elbowUp: $("armElbowUpToggle").checked,
    target: {
      x: numberInput("armTargetXInput"),
      y: numberInput("armTargetYInput"),
      z: numberInput("armTargetZInput"),
    },
    offsets: {
      base: numberInput("armBaseOffsetInput"),
      shoulder: numberInput("armShoulderOffsetInput"),
      elbow: numberInput("armElbowOffsetInput"),
    },
    directions: {
      base: direction("armBaseDirectionInput"),
      shoulder: direction("armShoulderDirectionInput"),
      elbow: direction("armElbowDirectionInput"),
    },
  };
}

function solveArmIk(arm) {
  const { x, y, z } = arm.target;
  const radial = Math.hypot(x, y);
  const reach = Math.hypot(radial, z);
  const maxReach = arm.link1 + arm.link2;
  const minReach = Math.abs(arm.link1 - arm.link2);
  if (reach > maxReach || reach < minReach) {
    throw new Error(`unreachable: reach=${reach.toFixed(3)}, allowed=${minReach.toFixed(3)}..${maxReach.toFixed(3)}`);
  }

  const base = Math.atan2(y, x);
  const cosElbow = Math.max(
    -1,
    Math.min(1, (radial * radial + z * z - arm.link1 * arm.link1 - arm.link2 * arm.link2) / (2 * arm.link1 * arm.link2))
  );
  let elbow = Math.acos(cosElbow);
  if (arm.elbowUp) elbow = -elbow;
  const shoulder = Math.atan2(z, radial) - Math.atan2(
    arm.link2 * Math.sin(elbow),
    arm.link1 + arm.link2 * Math.cos(elbow)
  );
  return { base, shoulder, elbow, reach, minReach, maxReach };
}

function armPreview() {
  const arm = armInputState();
  try {
    const joints = solveArmIk(arm);
    const motorTargets = {
      base: arm.offsets.base + arm.directions.base * joints.base,
      shoulder: arm.offsets.shoulder + arm.directions.shoulder * joints.shoulder,
      elbow: arm.offsets.elbow + arm.directions.elbow * joints.elbow,
    };
    const baseDir = { x: Math.cos(joints.base), y: Math.sin(joints.base) };
    const p0 = { x: 0, y: 0, z: 0 };
    const p1 = {
      x: arm.link1 * Math.cos(joints.shoulder) * baseDir.x,
      y: arm.link1 * Math.cos(joints.shoulder) * baseDir.y,
      z: arm.link1 * Math.sin(joints.shoulder),
    };
    const p2 = {
      x: p1.x + arm.link2 * Math.cos(joints.shoulder + joints.elbow) * baseDir.x,
      y: p1.y + arm.link2 * Math.cos(joints.shoulder + joints.elbow) * baseDir.y,
      z: p1.z + arm.link2 * Math.sin(joints.shoulder + joints.elbow),
    };
    return { ok: true, arm, joints, motorTargets, points: [p0, p1, p2], message: "" };
  } catch (error) {
    const radial = Math.hypot(arm.target.x, arm.target.y);
    const reach = Math.hypot(radial, arm.target.z);
    return {
      ok: false,
      arm,
      joints: { reach, minReach: Math.abs(arm.link1 - arm.link2), maxReach: arm.link1 + arm.link2 },
      motorTargets: null,
      points: [{ x: 0, y: 0, z: 0 }, arm.target],
      message: error.message,
    };
  }
}

function renderArmSolution(preview) {
  if (!preview.ok) {
    $("armSolution").textContent = preview.message;
    $("ikReachValue").textContent = `${preview.joints.reach.toFixed(3)} m`;
    $("ikBaseValue").textContent = "--";
    $("ikShoulderValue").textContent = "--";
    $("ikElbowValue").textContent = "--";
    return;
  }
  const { joints, motorTargets } = preview;
  $("armSolution").textContent =
    `joints rad  base=${joints.base.toFixed(3)} ` +
    `shoulder=${joints.shoulder.toFixed(3)} ` +
    `elbow=${joints.elbow.toFixed(3)}\n` +
    `motors rad  base=${motorTargets.base.toFixed(3)} ` +
    `shoulder=${motorTargets.shoulder.toFixed(3)} ` +
    `elbow=${motorTargets.elbow.toFixed(3)}`;
  $("ikReachValue").textContent = `${joints.reach.toFixed(3)} m`;
  $("ikBaseValue").textContent = `${joints.base.toFixed(3)} rad`;
  $("ikShoulderValue").textContent = `${joints.shoulder.toFixed(3)} rad`;
  $("ikElbowValue").textContent = `${joints.elbow.toFixed(3)} rad`;
}

function projected(point, scale, centerX, centerY) {
  const yawCos = Math.cos(ikViewYaw);
  const yawSin = Math.sin(ikViewYaw);
  const pitchCos = Math.cos(ikViewPitch);
  const pitchSin = Math.sin(ikViewPitch);
  const xYaw = point.x * yawCos - point.y * yawSin;
  const depth = point.x * yawSin + point.y * yawCos;
  const yPitch = point.z * pitchCos - depth * pitchSin;
  return {
    x: centerX + xYaw * scale,
    y: centerY - yPitch * scale,
    depth: depth * pitchCos + point.z * pitchSin,
  };
}

function drawGroundCircle(ctx, radius, project) {
  if (radius <= 0) return;
  ctx.beginPath();
  for (let i = 0; i <= 96; i += 1) {
    const theta = (i / 96) * Math.PI * 2;
    const point = project({ x: Math.cos(theta) * radius, y: Math.sin(theta) * radius, z: 0 });
    if (i === 0) ctx.moveTo(point.x, point.y);
    else ctx.lineTo(point.x, point.y);
  }
  ctx.stroke();
}

function drawIkCanvas(preview) {
  const canvas = $("armIkCanvas");
  const ctx = canvas.getContext("2d");
  const rect = canvas.getBoundingClientRect();
  const width = Math.max(320, rect.width || canvas.width);
  const height = Math.max(260, rect.height || canvas.height);
  const dpr = window.devicePixelRatio || 1;
  canvas.width = Math.floor(width * dpr);
  canvas.height = Math.floor(height * dpr);
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  ctx.clearRect(0, 0, width, height);

  const target = preview.arm.target;
  const sceneRadius = Math.max(
    preview.arm.link1 + preview.arm.link2,
    Math.hypot(target.x, target.y, target.z),
    0.2
  );
  const scale = Math.min(width / (sceneRadius * 2.7), height / (sceneRadius * 2.25));
  const centerX = width * 0.50;
  const centerY = height * 0.70;
  const project = (point) => projected(point, scale, centerX, centerY);

  const gradient = ctx.createLinearGradient(0, 0, width, height);
  gradient.addColorStop(0, "#101819");
  gradient.addColorStop(1, "#182424");
  ctx.fillStyle = gradient;
  ctx.fillRect(0, 0, width, height);

  ctx.lineWidth = 1;
  ctx.strokeStyle = "#273938";
  for (let i = -4; i <= 4; i += 1) {
    const offset = (sceneRadius * i) / 4;
    const a = project({ x: -sceneRadius, y: offset, z: 0 });
    const b = project({ x: sceneRadius, y: offset, z: 0 });
    const c = project({ x: offset, y: -sceneRadius, z: 0 });
    const d = project({ x: offset, y: sceneRadius, z: 0 });
    ctx.beginPath();
    ctx.moveTo(a.x, a.y);
    ctx.lineTo(b.x, b.y);
    ctx.moveTo(c.x, c.y);
    ctx.lineTo(d.x, d.y);
    ctx.stroke();
  }

  ctx.setLineDash([6, 5]);
  ctx.strokeStyle = "#395453";
  drawGroundCircle(ctx, preview.joints.maxReach || preview.arm.link1 + preview.arm.link2, project);
  ctx.strokeStyle = "#54443a";
  drawGroundCircle(ctx, preview.joints.minReach || 0, project);
  ctx.setLineDash([]);

  const origin = project({ x: 0, y: 0, z: 0 });
  const axes = [
    [{ x: sceneRadius * 0.45, y: 0, z: 0 }, "#ff7d73", "X"],
    [{ x: 0, y: sceneRadius * 0.45, z: 0 }, "#72b7ff", "Y"],
    [{ x: 0, y: 0, z: sceneRadius * 0.45 }, "#56d6a8", "Z"],
  ];
  axes.forEach(([axisPoint, color, label]) => {
    const end = project(axisPoint);
    ctx.strokeStyle = color;
    ctx.fillStyle = color;
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(origin.x, origin.y);
    ctx.lineTo(end.x, end.y);
    ctx.stroke();
    ctx.font = "12px ui-monospace, SFMono-Regular, Menlo, Consolas, monospace";
    ctx.fillText(label, end.x + 5, end.y - 5);
  });

  const targetTop = project(target);
  const targetBase = project({ x: target.x, y: target.y, z: 0 });
  ctx.strokeStyle = preview.ok ? "#ffd166" : "#ff6b5f";
  ctx.lineWidth = 2;
  ctx.setLineDash([5, 4]);
  ctx.beginPath();
  ctx.moveTo(targetBase.x, targetBase.y);
  ctx.lineTo(targetTop.x, targetTop.y);
  ctx.stroke();
  ctx.setLineDash([]);
  ctx.fillStyle = preview.ok ? "#ffd166" : "#ff6b5f";
  ctx.beginPath();
  ctx.arc(targetTop.x, targetTop.y, 6, 0, Math.PI * 2);
  ctx.fill();

  if (preview.ok) {
    const points = preview.points.map(project);
    ctx.lineCap = "round";
    ctx.lineJoin = "round";
    ctx.shadowColor = "rgba(0, 0, 0, 0.35)";
    ctx.shadowBlur = 12;
    ctx.strokeStyle = "#56d6a8";
    ctx.lineWidth = 13;
    ctx.beginPath();
    ctx.moveTo(points[0].x, points[0].y);
    ctx.lineTo(points[1].x, points[1].y);
    ctx.lineTo(points[2].x, points[2].y);
    ctx.stroke();
    ctx.shadowBlur = 0;
    ctx.strokeStyle = "#baf8e0";
    ctx.lineWidth = 4;
    ctx.beginPath();
    ctx.moveTo(points[0].x, points[0].y);
    ctx.lineTo(points[1].x, points[1].y);
    ctx.lineTo(points[2].x, points[2].y);
    ctx.stroke();
    points.forEach((point, index) => {
      ctx.fillStyle = index === 2 ? "#ffd166" : "#f3f7f5";
      ctx.beginPath();
      ctx.arc(point.x, point.y, index === 1 ? 7 : 6, 0, Math.PI * 2);
      ctx.fill();
    });
  } else {
    ctx.strokeStyle = "#ff6b5f";
    ctx.lineWidth = 3;
    ctx.setLineDash([8, 7]);
    ctx.beginPath();
    ctx.moveTo(origin.x, origin.y);
    ctx.lineTo(targetTop.x, targetTop.y);
    ctx.stroke();
    ctx.setLineDash([]);
  }
}

function renderIkPreview() {
  const preview = armPreview();
  $("armConfiguredState").classList.toggle("fault", !preview.ok);
  renderArmSolution(preview);
  drawIkCanvas(preview);
}

function render(state) {
  setControlValue("serialPortInput", state.serialPort || "auto");
  setControlValue("serialBaudInput", state.serialBaud || 921600);
  setControlValue("motorIdInput", state.motorIdHex);
  setControlValue("hostIdInput", state.hostIdHex);
  setControlValue("modelInput", state.model);

  const status = $("connectionStatus");
  status.textContent = state.connected ? "Online" : "Offline";
  status.className = `status-pill ${state.connected ? "online" : "offline"}`;
  $("subtitle").textContent = state.openError || state.transportLabel || "RobStride USB";
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
  setControlValue("positionKpInput", Number(state.positionKp || 5).toFixed(1));

  const arm = state.arm || {};
  const armIds = arm.motorIdHex || {};
  const armOffsets = arm.offsets || {};
  const armDirections = arm.directions || {};
  const armTarget = arm.target || {};
  setControlValue("armBaseMotorIdInput", armIds.base || "0x7F");
  setControlValue("armShoulderMotorIdInput", armIds.shoulder || "0x01");
  setControlValue("armElbowMotorIdInput", armIds.elbow || "0x02");
  setControlValue("armLink1Input", Number(arm.link1 || 0.25).toFixed(3));
  setControlValue("armLink2Input", Number(arm.link2 || 0.25).toFixed(3));
  setControlChecked("armElbowUpToggle", arm.elbowUp);
  setControlValue("armTargetXInput", Number(armTarget.x || 0).toFixed(3));
  setControlValue("armTargetYInput", Number(armTarget.y || 0).toFixed(3));
  setControlValue("armTargetZInput", Number(armTarget.z || 0).toFixed(3));
  setControlValue("armVelocityInput", Number(arm.velocityLimit || 1).toFixed(2));
  setControlValue("armAccelerationInput", Number(arm.acceleration || 10).toFixed(1));
  setControlValue("armKpInput", Number(arm.positionKp || 5).toFixed(1));
  setControlValue("armBaseOffsetInput", Number(armOffsets.base || 0).toFixed(3));
  setControlValue("armBaseDirectionInput", String(armDirections.base || 1));
  setControlValue("armShoulderOffsetInput", Number(armOffsets.shoulder || 0).toFixed(3));
  setControlValue("armShoulderDirectionInput", String(armDirections.shoulder || 1));
  setControlValue("armElbowOffsetInput", Number(armOffsets.elbow || 0).toFixed(3));
  setControlValue("armElbowDirectionInput", String(armDirections.elbow || 1));
  $("armConfiguredState").textContent = arm.configured ? "Holding IK" : "Idle";
  renderIkPreview();

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

  $("logOutput").textContent = (state.logs || []).join("\n");
  $("logOutput").scrollTop = $("logOutput").scrollHeight;
}

async function refresh() {
  const response = await fetch("/api/state", { cache: "no-store" });
  state = await response.json();
  render(state);
}

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

armControlIds.forEach((id) => {
  const el = $(id);
  const update = () => {
    markDirty(id);
    renderIkPreview();
  };
  el.addEventListener("input", update);
  el.addEventListener("change", update);
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

const ikCanvas = $("armIkCanvas");
ikCanvas.addEventListener("pointerdown", (event) => {
  ikDrag = {
    id: event.pointerId,
    x: event.clientX,
    y: event.clientY,
    yaw: ikViewYaw,
    pitch: ikViewPitch,
  };
  ikCanvas.setPointerCapture(event.pointerId);
});

ikCanvas.addEventListener("pointermove", (event) => {
  if (!ikDrag || event.pointerId !== ikDrag.id) return;
  ikViewYaw = ikDrag.yaw + (event.clientX - ikDrag.x) * 0.01;
  ikViewPitch = Math.max(0.2, Math.min(1.25, ikDrag.pitch + (event.clientY - ikDrag.y) * 0.008));
  renderIkPreview();
});

ikCanvas.addEventListener("pointerup", () => {
  ikDrag = null;
});

ikCanvas.addEventListener("pointercancel", () => {
  ikDrag = null;
});

window.addEventListener("resize", renderIkPreview);

refresh();
setInterval(refresh, 500);
