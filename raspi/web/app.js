const $ = (id) => document.getElementById(id);

let state = null;
let busy = false;
let ikViewYaw = -0.72;
let ikViewPitch = 0.58;
let ikDrag = null;
let targetViewYaw = -0.74;
let targetViewPitch = 0.62;
let targetEditorDrag = null;
let targetGizmoHitZones = [];
let wizardStepIndex = 0;

const commandButtons = [...document.querySelectorAll("[data-command]")];
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
const valueButtons = [$("saveValuesBtn"), $("downloadValuesBtn"), $("uploadValuesBtn")].filter(Boolean);
const allValueControlIds = [
  ...configControlIds,
  ...positionControlIds,
  ...armControlIds,
  ...speedControlIds,
  "wizardJointCountInput",
];
const allValueControlIdSet = new Set(allValueControlIds);
const dirtyControls = new Set();
const wizardSteps = [
  { key: "joints", title: "Joints", visual: "3-axis base + shoulder + elbow" },
  { key: "lengths", title: "Lengths", visual: "Set link lengths and total reach" },
  { key: "home", title: "Home", visual: "Straight ahead, level shoulder, straight elbow" },
  { key: "save", title: "Save", visual: "Save, download, or upload the setup" },
];

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
  if (allValueControlIdSet.has(id)) setValuesState("Unsaved");
}

function clearDirty(ids) {
  ids.forEach((id) => dirtyControls.delete(id));
}

function clearCommandDirty(command, result) {
  if (result && result.ok === false) return;
  if (command === "move-position") clearDirty(positionControlIds);
  if (command === "arm-move" || command === "arm-home-zero") clearDirty(armControlIds);
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
  if (command === "arm-move" || command === "arm-home-zero") {
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

function hasValue(value) {
  return value !== undefined && value !== null && value !== "";
}

function objectValue(value) {
  return value && typeof value === "object" && !Array.isArray(value) ? value : {};
}

function firstValue(...values) {
  for (const value of values) {
    if (hasValue(value)) return value;
  }
  return undefined;
}

function idText(value, fallback) {
  if (!hasValue(value)) return fallback;
  if (typeof value === "number" && Number.isFinite(value)) {
    return `0x${(value & 0xff).toString(16).toUpperCase().padStart(2, "0")}`;
  }
  return String(value);
}

function directionText(value) {
  if (!hasValue(value)) return undefined;
  return String(Number(value) < 0 ? -1 : 1);
}

function setValuesState(text, title = "") {
  const el = $("valuesState");
  if (!el) return;
  el.textContent = text;
  if (title) el.title = title;
}

function setDirtyValue(id, value) {
  const el = $(id);
  if (!el || !hasValue(value)) return;
  el.value = String(value);
  markDirty(id);
}

function setDirtyNumber(id, value, digits) {
  const numeric = Number(value);
  if (!Number.isFinite(numeric)) return;
  setDirtyValue(id, numeric.toFixed(digits));
}

function setDirtyChecked(id, checked) {
  const el = $(id);
  if (!el || checked === undefined || checked === null) return;
  el.checked = Boolean(checked);
  markDirty(id);
}

function collectValues() {
  return {
    schemaVersion: 1,
    appVersion: state && state.appVersion ? state.appVersion : undefined,
    exportedAt: new Date().toISOString(),
    serialPort: $("serialPortInput").value.trim(),
    serialBaud: $("serialBaudInput").value.trim(),
    motorId: $("motorIdInput").value.trim(),
    hostId: $("hostIdInput").value.trim(),
    model: $("modelInput").value,
    testSpeed: numberInput("speedSlider"),
    position: {
      target: numberInput("positionTargetInput"),
      velocityLimit: numberInput("positionVelocityInput"),
      acceleration: numberInput("positionAccelerationInput"),
      positionKp: numberInput("positionKpInput"),
    },
    arm: {
      motorIds: {
        base: $("armBaseMotorIdInput").value.trim(),
        shoulder: $("armShoulderMotorIdInput").value.trim(),
        elbow: $("armElbowMotorIdInput").value.trim(),
      },
      jointCount: numberInput("wizardJointCountInput") || 3,
      link1: numberInput("armLink1Input"),
      link2: numberInput("armLink2Input"),
      elbowUp: $("armElbowUpToggle").checked,
      target: {
        x: numberInput("armTargetXInput"),
        y: numberInput("armTargetYInput"),
        z: numberInput("armTargetZInput"),
      },
      velocityLimit: numberInput("armVelocityInput"),
      acceleration: numberInput("armAccelerationInput"),
      positionKp: numberInput("armKpInput"),
      offsets: {
        base: numberInput("armBaseOffsetInput"),
        shoulder: numberInput("armShoulderOffsetInput"),
        elbow: numberInput("armElbowOffsetInput"),
      },
      directions: {
        base: $("armBaseDirectionInput").value,
        shoulder: $("armShoulderDirectionInput").value,
        elbow: $("armElbowDirectionInput").value,
      },
    },
  };
}

function applyValuePayload(payload) {
  const position = objectValue(payload.position);
  const arm = objectValue(payload.arm);
  const motorIds = objectValue(arm.motorIds || arm.motorIdHex);
  const target = objectValue(arm.target);
  const offsets = objectValue(arm.offsets);
  const directions = objectValue(arm.directions);

  setDirtyValue("serialPortInput", payload.serialPort);
  setDirtyValue("serialBaudInput", payload.serialBaud);
  setDirtyValue("motorIdInput", hasValue(payload.motorId) ? idText(payload.motorId, "") : undefined);
  setDirtyValue("hostIdInput", hasValue(payload.hostId) ? idText(payload.hostId, "") : undefined);
  setDirtyValue("modelInput", payload.model);
  setDirtyNumber("speedSlider", payload.testSpeed, 2);
  $("speedValue").textContent = `${Number($("speedSlider").value).toFixed(2)} rad/s`;

  setDirtyNumber("positionTargetInput", firstValue(position.target, payload.positionTarget), 3);
  setDirtyNumber(
    "positionVelocityInput",
    firstValue(position.velocityLimit, payload.positionVelocityLimit),
    3,
  );
  setDirtyNumber(
    "positionAccelerationInput",
    firstValue(position.acceleration, payload.positionAcceleration),
    2,
  );
  setDirtyNumber("positionKpInput", firstValue(position.positionKp, payload.positionKp), 2);

  setDirtyValue(
    "armBaseMotorIdInput",
    hasValue(firstValue(motorIds.base, payload.armBaseMotorId))
      ? idText(firstValue(motorIds.base, payload.armBaseMotorId), "")
      : undefined,
  );
  setDirtyValue(
    "armShoulderMotorIdInput",
    hasValue(firstValue(motorIds.shoulder, payload.armShoulderMotorId))
      ? idText(firstValue(motorIds.shoulder, payload.armShoulderMotorId), "")
      : undefined,
  );
  setDirtyValue(
    "armElbowMotorIdInput",
    hasValue(firstValue(motorIds.elbow, payload.armElbowMotorId))
      ? idText(firstValue(motorIds.elbow, payload.armElbowMotorId), "")
      : undefined,
  );
  setDirtyNumber("armLink1Input", firstValue(arm.link1, payload.armLink1), 3);
  setDirtyNumber("armLink2Input", firstValue(arm.link2, payload.armLink2), 3);
  if (Number(firstValue(arm.jointCount, payload.armJointCount)) === 3) {
    setDirtyValue("wizardJointCountInput", "3");
  }
  setDirtyChecked("armElbowUpToggle", firstValue(arm.elbowUp, payload.armElbowUp));
  setDirtyNumber("armTargetXInput", firstValue(target.x, payload.armTargetX), 3);
  setDirtyNumber("armTargetYInput", firstValue(target.y, payload.armTargetY), 3);
  setDirtyNumber("armTargetZInput", firstValue(target.z, payload.armTargetZ), 3);
  setDirtyNumber("armVelocityInput", firstValue(arm.velocityLimit, payload.armVelocityLimit), 3);
  setDirtyNumber("armAccelerationInput", firstValue(arm.acceleration, payload.armAcceleration), 2);
  setDirtyNumber("armKpInput", firstValue(arm.positionKp, payload.armPositionKp), 2);
  setDirtyNumber("armBaseOffsetInput", firstValue(offsets.base, payload.armBaseOffset), 3);
  setDirtyValue("armBaseDirectionInput", directionText(firstValue(directions.base, payload.armBaseDirection)));
  setDirtyNumber("armShoulderOffsetInput", firstValue(offsets.shoulder, payload.armShoulderOffset), 3);
  setDirtyValue(
    "armShoulderDirectionInput",
    directionText(firstValue(directions.shoulder, payload.armShoulderDirection)),
  );
  setDirtyNumber("armElbowOffsetInput", firstValue(offsets.elbow, payload.armElbowOffset), 3);
  setDirtyValue("armElbowDirectionInput", directionText(firstValue(directions.elbow, payload.armElbowDirection)));
  renderIkPreview();
}

async function saveValues(successText = "Values saved") {
  if (busy) return;
  busy = true;
  renderBusy(true);
  try {
    const result = await post("/api/values", collectValues());
    if (result.ok === false) {
      appendLocalLog(`Values failed: ${result.message || "save rejected"}`);
      setValuesState("Save failed");
    } else {
      clearDirty(allValueControlIds);
      setValuesState("Saved", result.path || "");
      appendLocalLog(`${successText}${result.path ? `: ${result.path}` : ""}`);
      await refresh();
    }
  } catch (error) {
    appendLocalLog(`UI error: ${error.message}`);
    setValuesState("Save failed");
  } finally {
    busy = false;
    renderBusy(false);
  }
}

function downloadValues() {
  const data = JSON.stringify(collectValues(), null, 2);
  const url = URL.createObjectURL(new Blob([`${data}\n`], { type: "application/json" }));
  const link = document.createElement("a");
  link.href = url;
  link.download = "helion-dashboard-values.json";
  document.body.appendChild(link);
  link.click();
  link.remove();
  URL.revokeObjectURL(url);
  setValuesState("Downloaded");
  appendLocalLog("Values downloaded");
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
  if (
    command === "arm-home-zero" &&
    !confirm("Place the arm at its home zero pose. This will disable the arm motors, read their current positions, and save those readings as IK offsets.")
  ) {
    return;
  }
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
    if (command === "arm-home-zero" && result && result.ok !== false) {
      setValuesState("Saved", result.path || "");
    }
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
  valueButtons.forEach((button) => {
    button.disabled = isBusy;
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

function projectPoint(point, scale, centerX, centerY, yaw, pitch) {
  const yawCos = Math.cos(yaw);
  const yawSin = Math.sin(yaw);
  const pitchCos = Math.cos(pitch);
  const pitchSin = Math.sin(pitch);
  const xYaw = point.x * yawCos - point.y * yawSin;
  const depth = point.x * yawSin + point.y * yawCos;
  const yPitch = point.z * pitchCos - depth * pitchSin;
  return {
    x: centerX + xYaw * scale,
    y: centerY - yPitch * scale,
    depth: depth * pitchCos + point.z * pitchSin,
  };
}

function projected(point, scale, centerX, centerY) {
  return projectPoint(point, scale, centerX, centerY, ikViewYaw, ikViewPitch);
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

function drawProjectedPath(ctx, points, project) {
  points.forEach((point, index) => {
    const projectedPoint = project(point);
    if (index === 0) ctx.moveTo(projectedPoint.x, projectedPoint.y);
    else ctx.lineTo(projectedPoint.x, projectedPoint.y);
  });
}

function drawIkCanvas(preview) {
  const canvas = $("armIkCanvas");
  if (!canvas) return;
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
  gradient.addColorStop(0, "#060606");
  gradient.addColorStop(1, "#151007");
  ctx.fillStyle = gradient;
  ctx.fillRect(0, 0, width, height);

  ctx.lineWidth = 1;
  ctx.strokeStyle = "#2d2616";
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
  ctx.strokeStyle = "#59451f";
  drawGroundCircle(ctx, preview.joints.maxReach || preview.arm.link1 + preview.arm.link2, project);
  ctx.strokeStyle = "#5f332e";
  drawGroundCircle(ctx, preview.joints.minReach || 0, project);
  ctx.setLineDash([]);

  const origin = project({ x: 0, y: 0, z: 0 });
  const axes = [
    [{ x: sceneRadius * 0.45, y: 0, z: 0 }, "#d99a24", "X"],
    [{ x: 0, y: sceneRadius * 0.45, z: 0 }, "#f6c445", "Y"],
    [{ x: 0, y: 0, z: sceneRadius * 0.45 }, "#ffe7a3", "Z"],
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
    ctx.strokeStyle = "#d99a24";
    ctx.lineWidth = 13;
    ctx.beginPath();
    ctx.moveTo(points[0].x, points[0].y);
    ctx.lineTo(points[1].x, points[1].y);
    ctx.lineTo(points[2].x, points[2].y);
    ctx.stroke();
    ctx.shadowBlur = 0;
    ctx.strokeStyle = "#ffe7a3";
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

function drawGizmoArrow(ctx, start, end, color, label) {
  const dx = end.x - start.x;
  const dy = end.y - start.y;
  const length = Math.max(1, Math.hypot(dx, dy));
  const ux = dx / length;
  const uy = dy / length;
  const head = 12;
  ctx.strokeStyle = color;
  ctx.fillStyle = color;
  ctx.lineWidth = 4;
  ctx.lineCap = "round";
  ctx.beginPath();
  ctx.moveTo(start.x, start.y);
  ctx.lineTo(end.x, end.y);
  ctx.stroke();
  ctx.beginPath();
  ctx.moveTo(end.x, end.y);
  ctx.lineTo(end.x - ux * head - uy * head * 0.45, end.y - uy * head + ux * head * 0.45);
  ctx.lineTo(end.x - ux * head + uy * head * 0.45, end.y - uy * head - ux * head * 0.45);
  ctx.closePath();
  ctx.fill();
  ctx.font = "13px ui-monospace, SFMono-Regular, Menlo, Consolas, monospace";
  ctx.fillText(label, end.x + ux * 8, end.y + uy * 8);
}

function drawTargetScene(preview, canvasId, options = {}) {
  const canvas = $(canvasId);
  if (!canvas) return;
  const ctx = canvas.getContext("2d");
  const rect = canvas.getBoundingClientRect();
  const width = Math.max(320, rect.width || canvas.width);
  const height = Math.max(options.compact ? 170 : 260, rect.height || canvas.height);
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
  const scale = Math.min(width / (sceneRadius * (options.compact ? 3.0 : 2.75)), height / (sceneRadius * 2.35));
  const centerX = width * 0.5;
  const centerY = height * (options.compact ? 0.72 : 0.68);
  const project = (point) => projectPoint(point, scale, centerX, centerY, targetViewYaw, targetViewPitch);

  const gradient = ctx.createLinearGradient(0, 0, width, height);
  gradient.addColorStop(0, "#060606");
  gradient.addColorStop(1, "#151007");
  ctx.fillStyle = gradient;
  ctx.fillRect(0, 0, width, height);

  ctx.lineWidth = 1;
  ctx.strokeStyle = "#2d2616";
  for (let i = -4; i <= 4; i += 1) {
    const offset = (sceneRadius * i) / 4;
    ctx.beginPath();
    drawProjectedPath(ctx, [
      { x: -sceneRadius, y: offset, z: 0 },
      { x: sceneRadius, y: offset, z: 0 },
    ], project);
    drawProjectedPath(ctx, [
      { x: offset, y: -sceneRadius, z: 0 },
      { x: offset, y: sceneRadius, z: 0 },
    ], project);
    ctx.stroke();
  }

  const origin = project({ x: 0, y: 0, z: 0 });
  [
    [{ x: sceneRadius * 0.55, y: 0, z: 0 }, "#d99a24", "X"],
    [{ x: 0, y: sceneRadius * 0.55, z: 0 }, "#f6c445", "Y"],
    [{ x: 0, y: 0, z: sceneRadius * 0.55 }, "#ffe7a3", "Z"],
  ].forEach(([axisPoint, color, label]) => {
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

  ctx.setLineDash([6, 5]);
  ctx.strokeStyle = "#59451f";
  drawGroundCircle(ctx, preview.joints.maxReach || preview.arm.link1 + preview.arm.link2, project);
  ctx.setLineDash([]);

  const targetBase = project({ x: target.x, y: target.y, z: 0 });
  const targetTop = project(target);
  ctx.strokeStyle = preview.ok ? "#ffd166" : "#ff6b5f";
  ctx.lineWidth = 2;
  ctx.setLineDash([5, 4]);
  ctx.beginPath();
  ctx.moveTo(targetBase.x, targetBase.y);
  ctx.lineTo(targetTop.x, targetTop.y);
  ctx.stroke();
  ctx.setLineDash([]);

  if (preview.ok) {
    const points = preview.points.map(project);
    ctx.lineCap = "round";
    ctx.lineJoin = "round";
    ctx.strokeStyle = "#d99a24";
    ctx.lineWidth = 10;
    ctx.beginPath();
    ctx.moveTo(points[0].x, points[0].y);
    ctx.lineTo(points[1].x, points[1].y);
    ctx.lineTo(points[2].x, points[2].y);
    ctx.stroke();
    ctx.strokeStyle = "#ffe7a3";
    ctx.lineWidth = 3;
    ctx.beginPath();
    ctx.moveTo(points[0].x, points[0].y);
    ctx.lineTo(points[1].x, points[1].y);
    ctx.lineTo(points[2].x, points[2].y);
    ctx.stroke();
  }

  ctx.fillStyle = preview.ok ? "#ffd166" : "#ff6b5f";
  ctx.beginPath();
  ctx.arc(targetTop.x, targetTop.y, 8, 0, Math.PI * 2);
  ctx.fill();
  ctx.strokeStyle = "#080808";
  ctx.lineWidth = 3;
  ctx.stroke();

  if (options.gizmo) {
    const gizmoLength = sceneRadius * 0.34;
    const xEnd = project({ x: target.x + gizmoLength, y: target.y, z: target.z });
    const yEnd = project({ x: target.x, y: target.y + gizmoLength, z: target.z });
    const zEnd = project({ x: target.x, y: target.y, z: target.z + gizmoLength });
    drawGizmoArrow(ctx, targetTop, xEnd, "#d99a24", "X");
    drawGizmoArrow(ctx, targetTop, yEnd, "#f6c445", "Y");
    drawGizmoArrow(ctx, targetTop, zEnd, "#ffe7a3", "Z");
    targetGizmoHitZones = [
      { axis: "x", start: targetTop, end: xEnd, scale, length: gizmoLength },
      { axis: "y", start: targetTop, end: yEnd, scale, length: gizmoLength },
      { axis: "z", start: targetTop, end: zEnd, scale, length: gizmoLength },
      { axis: "target", start: targetTop, end: targetTop, scale, length: gizmoLength },
    ];
  }

  ctx.fillStyle = "#cddbd7";
  ctx.font = "13px ui-monospace, SFMono-Regular, Menlo, Consolas, monospace";
  ctx.fillText(options.compact ? "Click Edit Target for transform controls" : "Drag X / Y / Z handles. Drag empty space to rotate.", 14, 22);
}

function drawTargetPad(preview) {
  drawTargetScene(preview, "targetControlCanvas", { compact: true });
}

function drawTargetEditor(preview) {
  drawTargetScene(preview, "targetEditorCanvas", { gizmo: true });
}

function renderIkPreview() {
  const preview = armPreview();
  $("armConfiguredState").classList.toggle("fault", !preview.ok);
  renderArmSolution(preview);
  drawIkCanvas(preview);
  drawTargetPad(preview);
  drawTargetEditor(preview);
  renderTargetEditorInputs(preview);
  updateWizardVisual(preview);
  const reachInput = $("wizardTotalReachInput");
  if (reachInput && document.activeElement !== reachInput) {
    reachInput.value = (preview.arm.link1 + preview.arm.link2).toFixed(3);
  }
}

function updateWizardVisual(preview) {
  const step = wizardSteps[wizardStepIndex];
  const el = $("wizardVisualText");
  if (!step || !el) return;
  const arm = preview ? preview.arm : armInputState();
  if (step.key === "joints") {
    el.textContent =
      `3 joints: base ${$("armBaseMotorIdInput").value}, ` +
      `shoulder ${$("armShoulderMotorIdInput").value}, elbow ${$("armElbowMotorIdInput").value}`;
  } else if (step.key === "lengths") {
    el.textContent = `Reach ${(arm.link1 + arm.link2).toFixed(3)} m from links ${arm.link1.toFixed(3)} + ${arm.link2.toFixed(3)} m`;
  } else if (step.key === "home") {
    el.textContent =
      `Home will save offsets from the current motor positions: base ${arm.offsets.base.toFixed(3)}, shoulder ${arm.offsets.shoulder.toFixed(3)}, elbow ${arm.offsets.elbow.toFixed(3)} rad`;
  } else {
    el.textContent = state && state.valuesPath ? `Values path ${state.valuesPath}` : step.visual;
  }
}

function openWizard() {
  $("ikWizardFlow").hidden = false;
  document.body.classList.add("modal-open");
  setWizardStep(wizardStepIndex);
  requestAnimationFrame(renderIkPreview);
}

function closeWizard() {
  $("ikWizardFlow").hidden = true;
  document.body.classList.remove("modal-open");
}

function setWizardStep(index) {
  wizardStepIndex = Math.max(0, Math.min(wizardSteps.length - 1, Number(index) || 0));
  const step = wizardSteps[wizardStepIndex];
  $("wizardStepCount").textContent = `${wizardStepIndex + 1} / ${wizardSteps.length}`;
  $("wizardStepTitle").textContent = step.title;
  $("wizardBackBtn").disabled = wizardStepIndex === 0;
  $("wizardNextBtn").textContent = wizardStepIndex === wizardSteps.length - 1 ? "Done" : "Next";
  document.querySelectorAll("[data-flow-panel]").forEach((panel) => {
    panel.classList.toggle("active", panel.dataset.flowPanel === step.key);
  });
  updateWizardVisual();
}

function stepWizard(direction) {
  if (direction > 0 && wizardStepIndex >= wizardSteps.length - 1) {
    closeWizard();
    return;
  }
  setWizardStep(wizardStepIndex + direction);
}

function setArmTarget(x, y, z) {
  setDirtyNumber("armTargetXInput", x, 3);
  setDirtyNumber("armTargetYInput", y, 3);
  setDirtyNumber("armTargetZInput", z, 3);
  renderIkPreview();
}

function syncTargetEditorInputs() {
  const x = numberInput("targetEditorXInput");
  const y = numberInput("targetEditorYInput");
  const z = numberInput("targetEditorZInput");
  setArmTarget(x, y, z);
}

function renderTargetEditorInputs(preview) {
  const target = preview.arm.target;
  const inputMap = [
    ["targetEditorXInput", target.x],
    ["targetEditorYInput", target.y],
    ["targetEditorZInput", target.z],
  ];
  inputMap.forEach(([id, value]) => {
    const el = $(id);
    if (el && document.activeElement !== el) el.value = value.toFixed(3);
  });
  const readouts = [
    ["targetReadoutX", `${target.x.toFixed(3)} m`],
    ["targetReadoutY", `${target.y.toFixed(3)} m`],
    ["targetReadoutZ", `${target.z.toFixed(3)} m`],
    ["targetReadoutReach", `${preview.joints.reach.toFixed(3)} m`],
  ];
  readouts.forEach(([id, value]) => {
    const el = $(id);
    if (el) el.textContent = value;
  });
}

function applyTargetPreset(name) {
  const arm = armInputState();
  const reach = Math.max(arm.link1 + arm.link2, 0.002);
  const safe = reach * 0.68;
  if (name === "forward") {
    setArmTarget(safe * 0.95, 0, reach * 0.16);
  } else if (name === "left") {
    setArmTarget(safe * 0.68, safe * 0.46, reach * 0.18);
  } else if (name === "high") {
    setArmTarget(safe * 0.48, 0, reach * 0.56);
  } else {
    setArmTarget(safe * 0.82, 0, reach * 0.22);
  }
}

function nudgeTarget(axis, direction) {
  const id = `armTarget${axis.toUpperCase()}Input`;
  const step = Math.max(0.001, Math.abs(numberInput("wizardNudgeStepInput") || 0.01));
  setDirtyNumber(id, numberInput(id) + direction * step, 3);
  renderIkPreview();
}

function openTargetEditor() {
  $("targetEditorFlow").hidden = false;
  document.body.classList.add("modal-open");
  requestAnimationFrame(renderIkPreview);
}

function closeTargetEditor() {
  $("targetEditorFlow").hidden = true;
  document.body.classList.remove("modal-open");
  targetEditorDrag = null;
}

function distanceToSegment(point, start, end) {
  const dx = end.x - start.x;
  const dy = end.y - start.y;
  const lengthSq = dx * dx + dy * dy;
  if (!lengthSq) return Math.hypot(point.x - start.x, point.y - start.y);
  const t = Math.max(0, Math.min(1, ((point.x - start.x) * dx + (point.y - start.y) * dy) / lengthSq));
  return Math.hypot(point.x - (start.x + dx * t), point.y - (start.y + dy * t));
}

function pickTargetGizmo(point) {
  const center = targetGizmoHitZones.find((zone) => zone.axis === "target");
  if (center && Math.hypot(point.x - center.start.x, point.y - center.start.y) <= 16) return center;
  return targetGizmoHitZones
    .filter((zone) => zone.axis !== "target")
    .find((zone) => distanceToSegment(point, zone.start, zone.end) <= 14);
}

function moveTargetOnAxis(event) {
  if (!targetEditorDrag) return;
  if (targetEditorDrag.mode === "orbit") {
    targetViewYaw = targetEditorDrag.yaw + (event.clientX - targetEditorDrag.x) * 0.01;
    targetViewPitch = Math.max(0.2, Math.min(1.25, targetEditorDrag.pitch + (event.clientY - targetEditorDrag.y) * 0.008));
    renderIkPreview();
    return;
  }

  const dx = event.clientX - targetEditorDrag.x;
  const dy = event.clientY - targetEditorDrag.y;
  const axis = targetEditorDrag.zone.axis;
  const screenX = targetEditorDrag.zone.end.x - targetEditorDrag.zone.start.x;
  const screenY = targetEditorDrag.zone.end.y - targetEditorDrag.zone.start.y;
  const screenLength = Math.max(1, Math.hypot(screenX, screenY));
  const direction = (dx * screenX + dy * screenY) / screenLength;
  const meters = (direction / Math.max(1, targetEditorDrag.zone.scale)) * 1.35;
  const next = { ...targetEditorDrag.target };
  if (axis === "target") {
    next.x += dx / Math.max(1, targetEditorDrag.zone.scale);
    next.z -= dy / Math.max(1, targetEditorDrag.zone.scale);
  } else {
    next[axis] += meters;
  }
  setArmTarget(next.x, next.y, next.z);
}

function zeroOffsets() {
  ["Base", "Shoulder", "Elbow"].forEach((axis) => {
    setDirtyNumber(`arm${axis}OffsetInput`, 0, 3);
  });
  renderIkPreview();
}

function zeroHere() {
  const preview = armPreview();
  if (!preview.ok) {
    appendLocalLog(`Zero Here skipped: ${preview.message}`);
    return;
  }
  setDirtyNumber("armBaseOffsetInput", -preview.arm.directions.base * preview.joints.base, 3);
  setDirtyNumber("armShoulderOffsetInput", -preview.arm.directions.shoulder * preview.joints.shoulder, 3);
  setDirtyNumber("armElbowOffsetInput", -preview.arm.directions.elbow * preview.joints.elbow, 3);
  renderIkPreview();
}

function invertAxis(axis) {
  const map = {
    base: "armBaseDirectionInput",
    shoulder: "armShoulderDirectionInput",
    elbow: "armElbowDirectionInput",
  };
  const id = map[axis];
  if (!id) return;
  setDirtyValue(id, Number($(id).value) < 0 ? "1" : "-1");
  renderIkPreview();
}

function splitLinks() {
  const total = Math.max(0.002, Math.abs(numberInput("wizardTotalReachInput") || 0.5));
  setDirtyNumber("armLink1Input", total / 2, 3);
  setDirtyNumber("armLink2Input", total / 2, 3);
  renderIkPreview();
}

function syncReach() {
  const arm = armInputState();
  $("wizardTotalReachInput").value = (arm.link1 + arm.link2).toFixed(3);
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
  $("subtitle").textContent = state.openError || state.transportLabel || "Helion control surface";
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

  renderChips("privateList", state.discoveredPrivate);

  $("logOutput").textContent = (state.logs || []).join("\n");
  $("logOutput").scrollTop = $("logOutput").scrollHeight;
  if (state.valuesPath && $("valuesState").textContent === "Not saved") {
    setValuesState("Ready", state.valuesPath);
  }
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

$("wizardJointCountInput").addEventListener("change", () => {
  markDirty("wizardJointCountInput");
});

$("clearLogBtn").addEventListener("click", () => {
  $("logOutput").textContent = "";
  post("/api/logs/clear", {}).then(refresh).catch((error) => {
    appendLocalLog(`UI error: ${error.message}`);
  });
});

$("saveValuesBtn").addEventListener("click", () => {
  saveValues();
});

$("downloadValuesBtn").addEventListener("click", downloadValues);

$("uploadValuesBtn").addEventListener("click", () => {
  $("uploadValuesInput").click();
});

$("uploadValuesInput").addEventListener("change", async (event) => {
  const [file] = event.target.files || [];
  if (!file) return;
  try {
    const payload = JSON.parse(await file.text());
    applyValuePayload(payload);
    await saveValues("Values uploaded and saved");
  } catch (error) {
    appendLocalLog(`UI error: ${error.message}`);
    setValuesState("Upload failed");
  } finally {
    event.target.value = "";
  }
});

$("setupIkBtn").addEventListener("click", openWizard);
$("wizardCloseBtn").addEventListener("click", closeWizard);
$("wizardBackBtn").addEventListener("click", () => stepWizard(-1));
$("wizardNextBtn").addEventListener("click", () => stepWizard(1));
$("ikWizardFlow").addEventListener("click", (event) => {
  if (event.target === $("ikWizardFlow")) closeWizard();
});

$("targetEditorFlow").addEventListener("click", (event) => {
  if (event.target === $("targetEditorFlow")) closeTargetEditor();
});

window.addEventListener("keydown", (event) => {
  if (event.key === "Escape" && !$("ikWizardFlow").hidden) closeWizard();
  if (event.key === "Escape" && !$("targetEditorFlow").hidden) closeTargetEditor();
});

document.querySelectorAll("[data-target-preset]").forEach((button) => {
  button.addEventListener("click", () => applyTargetPreset(button.dataset.targetPreset));
});

document.querySelectorAll("[data-nudge-axis]").forEach((button) => {
  button.addEventListener("click", () => {
    nudgeTarget(button.dataset.nudgeAxis, Number(button.dataset.nudgeDir || 1));
  });
});

$("wizardSplitLinksBtn").addEventListener("click", splitLinks);
$("wizardSyncReachBtn").addEventListener("click", syncReach);

const targetCanvas = $("targetControlCanvas");
targetCanvas.addEventListener("click", openTargetEditor);
$("editTargetBtn").addEventListener("click", openTargetEditor);
$("targetEditorCloseBtn").addEventListener("click", closeTargetEditor);
$("targetEditorApplyBtn").addEventListener("click", closeTargetEditor);
$("targetEditorHomeBtn").addEventListener("click", () => applyTargetPreset("home"));
["targetEditorXInput", "targetEditorYInput", "targetEditorZInput"].forEach((id) => {
  const el = $(id);
  el.addEventListener("input", syncTargetEditorInputs);
  el.addEventListener("change", syncTargetEditorInputs);
});

const targetEditorCanvas = $("targetEditorCanvas");
targetEditorCanvas.addEventListener("pointerdown", (event) => {
  const preview = armPreview();
  const rect = targetEditorCanvas.getBoundingClientRect();
  const point = { x: event.clientX - rect.left, y: event.clientY - rect.top };
  const zone = pickTargetGizmo(point);
  targetEditorDrag = zone
    ? {
        id: event.pointerId,
        mode: "axis",
        zone,
        x: event.clientX,
        y: event.clientY,
        target: { ...preview.arm.target },
      }
    : {
        id: event.pointerId,
        mode: "orbit",
        x: event.clientX,
        y: event.clientY,
        yaw: targetViewYaw,
        pitch: targetViewPitch,
      };
  targetEditorCanvas.classList.add("dragging");
  targetEditorCanvas.setPointerCapture(event.pointerId);
});

targetEditorCanvas.addEventListener("pointermove", (event) => {
  if (!targetEditorDrag || event.pointerId !== targetEditorDrag.id) return;
  moveTargetOnAxis(event);
});

function endTargetEditorDrag() {
  targetEditorDrag = null;
  targetEditorCanvas.classList.remove("dragging");
}

targetEditorCanvas.addEventListener("pointerup", endTargetEditorDrag);
targetEditorCanvas.addEventListener("pointercancel", endTargetEditorDrag);

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
