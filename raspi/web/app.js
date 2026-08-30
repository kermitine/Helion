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

const TAU = Math.PI * 2;
const DEFAULT_TWIST_LIMIT_DEG = 180;
const AXIS_LABELS = {
  base: "Base",
  shoulder: "Shoulder",
  elbow: "Elbow",
};
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
  "armBaseModelInput",
  "armShoulderModelInput",
  "armElbowModelInput",
  "armLink1Input",
  "armLink2Input",
  "armLink1RadiusInput",
  "armLink2RadiusInput",
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
  "armBaseTwistLimitInput",
  "armShoulderTwistLimitInput",
  "armElbowTwistLimitInput",
];
const speedControlIds = ["speedSlider"];
const valueButtons = [$("saveValuesBtn"), $("downloadValuesBtn"), $("uploadValuesBtn")].filter(Boolean);
const idSetupButtons = [$("idSetupScanBtn"), $("idSetupAssignBtn")].filter(Boolean);
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
  { key: "joints", title: "Joints", visual: "Base + shoulder, optional elbow" },
  { key: "lengths", title: "Lengths", visual: "Set link lengths and total reach" },
  { key: "safety", title: "Safety", visual: "Set link radius and wire twist limits" },
  { key: "home", title: "Home", visual: "Straight ahead, level shoulder, straight elbow" },
  { key: "save", title: "Save", visual: "Save, download, or upload the setup" },
];
const idSetupRoleDefaults = {
  base: "0x01",
  shoulder: "0x02",
  elbow: "0x03",
};

function fixed(value, digits, suffix) {
  if (typeof value !== "number" || Number.isNaN(value)) return "--";
  return `${value.toFixed(digits)} ${suffix}`;
}

function degToRad(value) {
  const numeric = Number(value);
  return Number.isFinite(numeric) ? (numeric * Math.PI) / 180 : Math.PI;
}

function radToDeg(value) {
  const numeric = Number(value);
  return Number.isFinite(numeric) ? (numeric * 180) / Math.PI : DEFAULT_TWIST_LIMIT_DEG;
}

function normalizeTwistLimitRad(value) {
  const numeric = Math.abs(Number(value));
  if (!Number.isFinite(numeric) || numeric <= 0) return Math.PI;
  return Math.min(numeric, Math.PI);
}

function twistLimitInputRad(id) {
  return normalizeTwistLimitRad(degToRad(numberInput(id) || DEFAULT_TWIST_LIMIT_DEG));
}

function activeAxesForArm(arm) {
  return Number(arm.jointCount) === 2 ? ["base", "shoulder"] : ["base", "shoulder", "elbow"];
}

function routedAngleWithinTwist(angle, reference, twistLimit) {
  const limit = normalizeTwistLimitRad(twistLimit);
  const target = Number(angle);
  const current = Number(reference);
  if (!Number.isFinite(target)) return 0;

  const referenceAngle = Number.isFinite(current) ? current : 0;
  const centerTurn = Math.round((referenceAngle - target) / TAU);
  const candidates = [];
  for (let turn = centerTurn - 2; turn <= centerTurn + 2; turn += 1) {
    const candidate = target + turn * TAU;
    if (Math.abs(candidate) <= limit + 0.000001) candidates.push(candidate);
  }
  if (!candidates.length) return target + centerTurn * TAU;

  return candidates.reduce((best, candidate) => {
    const bestDistance = Math.abs(best - referenceAngle);
    const candidateDistance = Math.abs(candidate - referenceAngle);
    if (candidateDistance < bestDistance) return candidate;
    if (candidateDistance === bestDistance && Math.abs(candidate) < Math.abs(best)) return candidate;
    return best;
  });
}

function routeJointAngles(arm, joints) {
  const previous = state && state.arm && state.arm.jointAngles ? state.arm.jointAngles : {};
  const routed = { ...joints };
  for (const axis of activeAxesForArm(arm)) {
    routed[axis] = routedAngleWithinTwist(
      joints[axis],
      previous[axis],
      arm.twistLimits && arm.twistLimits[axis],
    );
  }
  if (arm.jointCount === 2) routed.elbow = 0;
  return routed;
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

function jointCountInput() {
  return Number($("wizardJointCountInput").value) === 2 ? 2 : 3;
}

function activeArmAxes() {
  return jointCountInput() === 2 ? ["base", "shoulder"] : ["base", "shoulder", "elbow"];
}

function selectedMotorValue(id, fallback = "") {
  const el = $(id);
  const value = el && typeof el.value === "string" ? el.value.trim() : "";
  if (el && el.tagName === "SELECT" && !el.disabled) return value;
  return value || fallback || "";
}

function currentArmMotorValue(axis, fallback = "") {
  const arm = state && state.arm ? state.arm : {};
  const ids = arm.motorIdHex || {};
  return ids[axis] || fallback;
}

function strictMotorValue(id) {
  const el = $(id);
  return el && typeof el.value === "string" ? el.value.trim() : "";
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
      armJointCount: jointCountInput(),
      armBaseMotorId: selectedMotorValue("armBaseMotorIdInput", currentArmMotorValue("base")),
      armShoulderMotorId: selectedMotorValue("armShoulderMotorIdInput", currentArmMotorValue("shoulder")),
      armElbowMotorId: selectedMotorValue("armElbowMotorIdInput", currentArmMotorValue("elbow")),
      armBaseModel: $("armBaseModelInput").value,
      armShoulderModel: $("armShoulderModelInput").value,
      armElbowModel: $("armElbowModelInput").value,
      armLink1: numberInput("armLink1Input"),
      armLink2: numberInput("armLink2Input"),
      armLink1Radius: numberInput("armLink1RadiusInput"),
      armLink2Radius: numberInput("armLink2RadiusInput"),
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
      armBaseTwistLimit: twistLimitInputRad("armBaseTwistLimitInput"),
      armShoulderTwistLimit: twistLimitInputRad("armShoulderTwistLimitInput"),
      armElbowTwistLimit: twistLimitInputRad("armElbowTwistLimitInput"),
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
    motorId: selectedMotorValue("motorIdInput", state ? state.motorIdHex : ""),
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
        base: selectedMotorValue("armBaseMotorIdInput", currentArmMotorValue("base")),
        shoulder: selectedMotorValue("armShoulderMotorIdInput", currentArmMotorValue("shoulder")),
        elbow: selectedMotorValue("armElbowMotorIdInput", currentArmMotorValue("elbow")),
      },
      models: {
        base: $("armBaseModelInput").value,
        shoulder: $("armShoulderModelInput").value,
        elbow: $("armElbowModelInput").value,
      },
      jointCount: jointCountInput(),
      link1: numberInput("armLink1Input"),
      link2: numberInput("armLink2Input"),
      radii: {
        link1: numberInput("armLink1RadiusInput"),
        link2: numberInput("armLink2RadiusInput"),
      },
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
      twistLimits: {
        base: twistLimitInputRad("armBaseTwistLimitInput"),
        shoulder: twistLimitInputRad("armShoulderTwistLimitInput"),
        elbow: twistLimitInputRad("armElbowTwistLimitInput"),
      },
    },
  };
}

function applyValuePayload(payload) {
  const position = objectValue(payload.position);
  const arm = objectValue(payload.arm);
  const motorIds = objectValue(arm.motorIds || arm.motorIdHex);
  const models = objectValue(arm.models);
  const radii = objectValue(arm.radii);
  const target = objectValue(arm.target);
  const offsets = objectValue(arm.offsets);
  const directions = objectValue(arm.directions);
  const twistLimits = objectValue(arm.twistLimits);

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
  setDirtyValue("armBaseModelInput", firstValue(models.base, payload.armBaseModel));
  setDirtyValue("armShoulderModelInput", firstValue(models.shoulder, payload.armShoulderModel));
  setDirtyValue("armElbowModelInput", firstValue(models.elbow, payload.armElbowModel));
  setDirtyNumber("armLink1Input", firstValue(arm.link1, payload.armLink1), 3);
  setDirtyNumber("armLink2Input", firstValue(arm.link2, payload.armLink2), 3);
  const jointCount = Number(firstValue(arm.jointCount, payload.armJointCount));
  if (jointCount === 2 || jointCount === 3) {
    setDirtyValue("wizardJointCountInput", String(jointCount));
  }
  setDirtyChecked("armElbowUpToggle", firstValue(arm.elbowUp, payload.armElbowUp));
  setDirtyNumber("armLink1RadiusInput", firstValue(radii.link1, payload.armLink1Radius), 3);
  setDirtyNumber("armLink2RadiusInput", firstValue(radii.link2, payload.armLink2Radius), 3);
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
  setDirtyNumber(
    "armBaseTwistLimitInput",
    radToDeg(normalizeTwistLimitRad(firstValue(twistLimits.base, payload.armBaseTwistLimit))),
    1,
  );
  setDirtyNumber(
    "armShoulderTwistLimitInput",
    radToDeg(normalizeTwistLimitRad(firstValue(twistLimits.shoulder, payload.armShoulderTwistLimit))),
    1,
  );
  setDirtyNumber(
    "armElbowTwistLimitInput",
    radToDeg(normalizeTwistLimitRad(firstValue(twistLimits.elbow, payload.armElbowTwistLimit))),
    1,
  );
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
    motorId: selectedMotorValue("motorIdInput", state ? state.motorIdHex : ""),
    hostId: $("hostIdInput").value.trim(),
    model: $("modelInput").value,
  });
  clearDirty(configControlIds);
}

function validateArmCommandMotors() {
  const inputIds = {
    base: "armBaseMotorIdInput",
    shoulder: "armShoulderMotorIdInput",
    elbow: "armElbowMotorIdInput",
  };
  const labels = {
    base: "Base",
    shoulder: "Shoulder",
    elbow: "Elbow",
  };
  const seen = new Map();
  const missing = [];
  for (const axis of activeArmAxes()) {
    const value = strictMotorValue(inputIds[axis]);
    if (!value) {
      missing.push(labels[axis]);
      continue;
    }
    if (seen.has(value)) {
      appendLocalLog(`Arm IK blocked: ${labels[axis]} and ${seen.get(value)} both use ${value}`);
      return false;
    }
    seen.set(value, labels[axis]);
  }
  if (missing.length) {
    appendLocalLog(`Arm IK blocked: select detected motors for ${missing.join(", ")}`);
    return false;
  }
  return true;
}

async function sendCommand(command, extra = {}) {
  if (busy && !["stop", "zero-speed", "clear-fault", "arm-stop", "arm-clear-fault"].includes(command)) return;
  if ((command === "arm-move" || command === "arm-home-zero") && !validateArmCommandMotors()) return;
  if (command === "arm-move") {
    const preview = armPreview();
    if (!preview.ok || !preview.safe) {
      const warnings = preview.safety && preview.safety.warnings ? preview.safety.warnings : [preview.message || "unsafe IK target"];
      appendLocalLog(`Arm IK blocked: ${warnings.join("; ")}`);
      renderIkPreview();
      return;
    }
  }
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
    } else if ((command === "scan" || command === "scan-private") && result && Array.isArray(result.motors)) {
      appendLocalLog(`Scan found ${result.motors.length} motor(s): ${result.motors.join(", ") || "none"}`);
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
  idSetupButtons.forEach((button) => {
    button.disabled = isBusy;
  });
  if (!isBusy) renderArmSafety(armPreview());
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

function normalizedMotorList(items) {
  return [...new Set((items || []).map((item) => idText(item, "")).filter(Boolean))];
}

function setMotorSelectOptions(id, motors, preferred, fallback = "") {
  const el = $(id);
  if (!el) return "";
  const current = document.activeElement === el || dirtyControls.has(id) ? el.value : "";
  const wanted = current || idText(preferred, "");
  const selected = motors.includes(wanted) ? wanted : fallback;
  el.innerHTML = "";

  if (!motors.length) {
    const option = document.createElement("option");
    option.value = "";
    option.textContent = "Scan motors first";
    el.appendChild(option);
    el.disabled = true;
    return "";
  }

  const placeholder = document.createElement("option");
  placeholder.value = "";
  placeholder.textContent = "Pick detected motor";
  el.appendChild(placeholder);
  motors.forEach((motor) => {
    const option = document.createElement("option");
    option.value = motor;
    option.textContent = motor;
    el.appendChild(option);
  });
  el.disabled = false;
  el.value = motors.includes(selected) ? selected : "";
  return el.value;
}

function chooseDetectedMotor(motors, preferred, used) {
  const normalized = idText(preferred, "");
  const uniquePreferred = motors.includes(normalized) && !used.has(normalized) ? normalized : "";
  const selected = uniquePreferred || motors.find((motor) => !used.has(motor)) || "";
  if (selected) used.add(selected);
  return selected;
}

function roleLabel(role) {
  if (role === "base") return "Base";
  if (role === "shoulder") return "Shoulder";
  if (role === "elbow") return "Elbow";
  return "No IK Slot";
}

function populateIdSetupNewOptions() {
  const el = $("idSetupNewMotorInput");
  if (!el || el.options.length) return;
  for (let id = 1; id <= 0x7f; id += 1) {
    const option = document.createElement("option");
    option.value = idText(id, "");
    option.textContent = option.value;
    el.appendChild(option);
  }
  el.value = idSetupRoleDefaults.base;
}

function setIdSetupMotorOptions(motors) {
  const el = $("idSetupOldMotorInput");
  if (!el) return;
  const current = document.activeElement === el ? el.value : "";
  const choices = motors.length ? motors : [state && state.motorIdHex ? state.motorIdHex : "0x7F"];
  const selected = choices.includes(current) ? current : choices[0];
  el.innerHTML = "";
  choices.forEach((motor) => {
    const option = document.createElement("option");
    option.value = motor;
    option.textContent = motor;
    el.appendChild(option);
  });
  el.value = selected;
}

function setIdSetupRoleDefault(force = false) {
  const role = $("idSetupRoleInput").value;
  const newId = idSetupRoleDefaults[role];
  if (!newId) return;
  const el = $("idSetupNewMotorInput");
  if (force || !el.value) el.value = newId;
}

function setIdSetupMessage(text, show = Boolean(text)) {
  const el = $("idSetupMessage");
  if (!el) return;
  el.hidden = !show;
  el.textContent = text || "";
}

function updateIdSetupUi() {
  populateIdSetupNewOptions();
  setIdSetupRoleDefault();
  const oldId = $("idSetupOldMotorInput").value || "0x7F";
  const newId = $("idSetupNewMotorInput").value || "--";
  const role = $("idSetupRoleInput").value;
  $("idSetupOldPreview").textContent = oldId;
  $("idSetupNewPreview").textContent = newId;
  $("idSetupRolePreview").textContent = roleLabel(role);
  $("idSetupBasePreview").textContent = $("armBaseMotorIdInput").value || "--";
  $("idSetupShoulderPreview").textContent = $("armShoulderMotorIdInput").value || "--";
  $("idSetupElbowPreview").textContent = $("armElbowMotorIdInput").value || "--";

  const motors = normalizedMotorList(state ? state.discoveredPrivate : []);
  $("idSetupState").textContent = motors.length === 1
    ? `Detected ${motors[0]}`
    : motors.length > 1
      ? "Unique IDs online"
      : "Ready";
}

function armInputState() {
  const direction = (id) => (Number($(id).value) < 0 ? -1 : 1);
  return {
    jointCount: jointCountInput(),
    link1: Math.max(Math.abs(numberInput("armLink1Input")), 0.001),
    link2: Math.max(Math.abs(numberInput("armLink2Input")), 0.001),
    radii: {
      link1: Math.max(0, numberInput("armLink1RadiusInput")),
      link2: Math.max(0, numberInput("armLink2RadiusInput")),
    },
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
    twistLimits: {
      base: twistLimitInputRad("armBaseTwistLimitInput"),
      shoulder: twistLimitInputRad("armShoulderTwistLimitInput"),
      elbow: twistLimitInputRad("armElbowTwistLimitInput"),
    },
  };
}

function solveArmIk(arm) {
  const { x, y, z } = arm.target;
  const radial = Math.hypot(x, y);
  const reach = Math.hypot(radial, z);
  const maxReach = arm.link1 + arm.link2;
  const minReach = arm.jointCount === 2 ? 0 : Math.abs(arm.link1 - arm.link2);
  if (arm.jointCount === 2) {
    if (reach <= 0.0001) {
      throw new Error("unreachable: 2-joint target cannot be at the base origin");
    }
    if (reach > maxReach) {
      throw new Error(`unreachable: reach=${reach.toFixed(3)}, allowed=0.000..${maxReach.toFixed(3)}`);
    }
    return {
      base: Math.atan2(y, x),
      shoulder: Math.atan2(z, radial),
      elbow: 0,
      reach,
      minReach,
      maxReach,
    };
  }
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

function pointSub(a, b) {
  return { x: a.x - b.x, y: a.y - b.y, z: a.z - b.z };
}

function pointAdd(a, b) {
  return { x: a.x + b.x, y: a.y + b.y, z: a.z + b.z };
}

function pointScale(a, scale) {
  return { x: a.x * scale, y: a.y * scale, z: a.z * scale };
}

function pointDot(a, b) {
  return a.x * b.x + a.y * b.y + a.z * b.z;
}

function pointLength(a) {
  return Math.sqrt(pointDot(a, a));
}

function trimSegment(start, end, trimStart, trimEnd) {
  const vector = pointSub(end, start);
  const length = pointLength(vector);
  if (length <= 0.000001 || trimStart + trimEnd >= length) return null;
  const direction = pointScale(vector, 1 / length);
  return [
    pointAdd(start, pointScale(direction, trimStart)),
    pointAdd(end, pointScale(direction, -trimEnd)),
  ];
}

function closestPointOnSegmentDistance(point, start, end) {
  const segment = pointSub(end, start);
  const lengthSq = pointDot(segment, segment);
  if (lengthSq <= 0.000001) return pointLength(pointSub(point, start));
  const t = Math.max(0, Math.min(1, pointDot(pointSub(point, start), segment) / lengthSq));
  const closest = pointAdd(start, pointScale(segment, t));
  return pointLength(pointSub(point, closest));
}

function segmentDistance(a0, a1, b0, b1) {
  const distances = [];
  for (let i = 0; i <= 16; i += 1) {
    const t = i / 16;
    const point = pointAdd(a0, pointScale(pointSub(a1, a0), t));
    distances.push(closestPointOnSegmentDistance(point, b0, b1));
  }
  for (let i = 0; i <= 16; i += 1) {
    const t = i / 16;
    const point = pointAdd(b0, pointScale(pointSub(b1, b0), t));
    distances.push(closestPointOnSegmentDistance(point, a0, a1));
  }
  return Math.min(...distances);
}

function armTwistWarnings(arm, joints) {
  const warnings = [];
  if (!joints) return warnings;
  for (const axis of activeAxesForArm(arm)) {
    const angle = Number(joints[axis] || 0);
    if (!Number.isFinite(angle)) continue;
    const limit = normalizeTwistLimitRad(arm.twistLimits && arm.twistLimits[axis]);
    if (Math.abs(angle) > limit + 0.000001) {
      warnings.push(
        `${AXIS_LABELS[axis]} twist ${radToDeg(angle).toFixed(1)} deg exceeds +/-${radToDeg(limit).toFixed(1)} deg from home`,
      );
    }
  }
  return warnings;
}

function armSafetyCheck(arm, points, joints = null) {
  const warnings = [];
  if (arm.jointCount === 3 && points.length >= 3) {
    const radius1 = Math.max(0, arm.radii.link1 || 0);
    const radius2 = Math.max(0, arm.radii.link2 || 0);
    const required = radius1 + radius2;
    if (required > 0) {
      const link1Length = pointLength(pointSub(points[1], points[0]));
      const link2Length = pointLength(pointSub(points[2], points[1]));
      if (required >= Math.min(link1Length, link2Length)) {
        warnings.push(`link radii are too large for the configured lengths: ${required.toFixed(3)} m clearance needed`);
      } else {
        const upper = trimSegment(points[0], points[1], 0, required);
        const forearm = trimSegment(points[1], points[2], required, 0);
        if (upper && forearm) {
          const clearance = segmentDistance(upper[0], upper[1], forearm[0], forearm[1]);
          if (clearance < required) {
            warnings.push(`link radii overlap: clearance ${clearance.toFixed(3)} m is below ${required.toFixed(3)} m`);
          }
        }
      }
    }
  }
  warnings.push(...armTwistWarnings(arm, joints));
  return { ok: warnings.length === 0, warnings };
}

function armPreview() {
  const arm = armInputState();
  try {
    const joints = routeJointAngles(arm, solveArmIk(arm));
    const motorTargets = {
      base: arm.offsets.base + arm.directions.base * joints.base,
      shoulder: arm.offsets.shoulder + arm.directions.shoulder * joints.shoulder,
      elbow: arm.offsets.elbow + arm.directions.elbow * joints.elbow,
    };
    const baseDir = { x: Math.cos(joints.base), y: Math.sin(joints.base) };
    const p0 = { x: 0, y: 0, z: 0 };
    if (arm.jointCount === 2) {
      const p1 = {
        x: arm.target.x,
        y: arm.target.y,
        z: arm.target.z,
      };
      const points = [p0, p1];
      const safety = armSafetyCheck(arm, points, joints);
      return { ok: true, safe: safety.ok, arm, joints, motorTargets, points, safety, message: "" };
    }
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
    const points = [p0, p1, p2];
    const safety = armSafetyCheck(arm, points, joints);
    return { ok: true, safe: safety.ok, arm, joints, motorTargets, points, safety, message: "" };
  } catch (error) {
    const radial = Math.hypot(arm.target.x, arm.target.y);
    const reach = Math.hypot(radial, arm.target.z);
    return {
      ok: false,
      arm,
      joints: {
        reach,
        minReach: arm.jointCount === 2 ? 0 : Math.abs(arm.link1 - arm.link2),
        maxReach: arm.link1 + arm.link2,
      },
      motorTargets: null,
      points: [{ x: 0, y: 0, z: 0 }, arm.target],
      safe: false,
      safety: { ok: false, warnings: [error.message] },
      message: error.message,
    };
  }
}

function twistReadout(preview) {
  return activeAxesForArm(preview.arm)
    .map((axis) => {
      const angle = radToDeg(preview.joints[axis] || 0).toFixed(1);
      const limit = radToDeg(normalizeTwistLimitRad(preview.arm.twistLimits[axis])).toFixed(0);
      return `${axis}=${angle}/${limit}`;
    })
    .join(" ");
}

function routeDeltaReadout(preview) {
  const previous = state && state.arm && state.arm.jointAngles ? state.arm.jointAngles : {};
  return activeAxesForArm(preview.arm)
    .map((axis) => {
      const start = Number(previous[axis] || 0);
      const target = Number(preview.joints[axis] || 0);
      return `${axis}=${radToDeg(target - start).toFixed(1)}`;
    })
    .join(" ");
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
  if (preview.arm.jointCount === 2) {
    $("armSolution").textContent =
      `joints rad  base=${joints.base.toFixed(3)} ` +
      `shoulder=${joints.shoulder.toFixed(3)}\n` +
      `motors rad  base=${motorTargets.base.toFixed(3)} ` +
      `shoulder=${motorTargets.shoulder.toFixed(3)}\n` +
      `twist deg  ${twistReadout(preview)}\n` +
      `route deg  ${routeDeltaReadout(preview)}`;
  } else {
    $("armSolution").textContent =
      `joints rad  base=${joints.base.toFixed(3)} ` +
      `shoulder=${joints.shoulder.toFixed(3)} ` +
      `elbow=${joints.elbow.toFixed(3)}\n` +
      `motors rad  base=${motorTargets.base.toFixed(3)} ` +
      `shoulder=${motorTargets.shoulder.toFixed(3)} ` +
      `elbow=${motorTargets.elbow.toFixed(3)}\n` +
      `twist deg  ${twistReadout(preview)}\n` +
      `route deg  ${routeDeltaReadout(preview)}`;
  }
  $("ikReachValue").textContent = `${joints.reach.toFixed(3)} m`;
  $("ikBaseValue").textContent = `${joints.base.toFixed(3)} rad`;
  $("ikShoulderValue").textContent = `${joints.shoulder.toFixed(3)} rad`;
  $("ikElbowValue").textContent = preview.arm.jointCount === 2 ? "--" : `${joints.elbow.toFixed(3)} rad`;
}

function renderArmSafety(preview) {
  const warning = $("armSafetyWarning");
  const moveButton = document.querySelector('[data-command="arm-move"]');
  const warnings = preview.safety && preview.safety.warnings ? preview.safety.warnings : [];
  const unsafe = !preview.ok || !preview.safe;
  if (warning) {
    warning.hidden = !warnings.length || !preview.ok;
    warning.textContent = warnings.length ? `Safety warning: ${warnings.join("; ")}` : "";
  }
  if (moveButton) {
    moveButton.disabled = busy || unsafe;
    moveButton.title = unsafe && warnings.length ? warnings.join("; ") : "";
  }
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

function previewLinkRadii(preview) {
  const radii = preview.arm.radii || {};
  const radius1 = Math.max(0, radii.link1 || 0);
  const radius2 = Math.max(0, radii.link2 || 0);
  return preview.arm.jointCount === 2 ? [Math.max(radius1, radius2)] : [radius1, radius2];
}

function drawArmCapsules(ctx, points, preview, scale, options = {}) {
  const radii = previewLinkRadii(preview);
  const unsafe = !preview.safe;
  ctx.lineCap = "round";
  ctx.lineJoin = "round";
  ctx.shadowColor = "rgba(0, 0, 0, 0.35)";
  ctx.shadowBlur = options.shadow ? 12 : 0;
  points.slice(1).forEach((point, index) => {
    const previous = points[index];
    const radiusWidth = radii[index] * scale * 2;
    const outerWidth = Math.max(options.minOuterWidth || 8, radiusWidth);
    ctx.strokeStyle = unsafe ? "#ff6b5f" : "#d99a24";
    ctx.lineWidth = outerWidth;
    ctx.beginPath();
    ctx.moveTo(previous.x, previous.y);
    ctx.lineTo(point.x, point.y);
    ctx.stroke();
  });
  ctx.shadowBlur = 0;
  points.slice(1).forEach((point, index) => {
    const previous = points[index];
    const radiusWidth = radii[index] * scale * 2;
    const outerWidth = Math.max(options.minOuterWidth || 8, radiusWidth);
    ctx.strokeStyle = unsafe ? "#ffd0cc" : "#ffe7a3";
    ctx.lineWidth = Math.max(options.minInnerWidth || 3, outerWidth * 0.28);
    ctx.beginPath();
    ctx.moveTo(previous.x, previous.y);
    ctx.lineTo(point.x, point.y);
    ctx.stroke();
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
  const maxLinkRadius = Math.max(...previewLinkRadii(preview));
  const sceneRadius = Math.max(
    preview.arm.link1 + preview.arm.link2,
    Math.hypot(target.x, target.y, target.z),
    0.2
  ) + maxLinkRadius * 1.2;
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
    [{ x: sceneRadius * 0.45, y: 0, z: 0 }, "#ff5d55", "X"],
    [{ x: 0, y: sceneRadius * 0.45, z: 0 }, "#4ba3ff", "Y"],
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
  ctx.strokeStyle = preview.ok && preview.safe ? "#ffd166" : "#ff6b5f";
  ctx.lineWidth = 2;
  ctx.setLineDash([5, 4]);
  ctx.beginPath();
  ctx.moveTo(targetBase.x, targetBase.y);
  ctx.lineTo(targetTop.x, targetTop.y);
  ctx.stroke();
  ctx.setLineDash([]);
  ctx.fillStyle = preview.ok && preview.safe ? "#ffd166" : "#ff6b5f";
  ctx.beginPath();
  ctx.arc(targetTop.x, targetTop.y, 6, 0, Math.PI * 2);
  ctx.fill();

  if (preview.ok) {
    const points = preview.points.map(project);
    drawArmCapsules(ctx, points, preview, scale, { minOuterWidth: 9, minInnerWidth: 3, shadow: true });
    points.forEach((point, index) => {
      ctx.fillStyle = index === points.length - 1 ? "#ffd166" : "#f3f7f5";
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
  const maxLinkRadius = Math.max(...previewLinkRadii(preview));
  const sceneRadius = Math.max(
    preview.arm.link1 + preview.arm.link2,
    Math.hypot(target.x, target.y, target.z),
    0.2
  ) + maxLinkRadius * 1.2;
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
    [{ x: sceneRadius * 0.55, y: 0, z: 0 }, "#ff5d55", "X"],
    [{ x: 0, y: sceneRadius * 0.55, z: 0 }, "#4ba3ff", "Y"],
    [{ x: 0, y: 0, z: sceneRadius * 0.55 }, "#56d6a8", "Z"],
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
  ctx.strokeStyle = preview.ok && preview.safe ? "#ffd166" : "#ff6b5f";
  ctx.lineWidth = 2;
  ctx.setLineDash([5, 4]);
  ctx.beginPath();
  ctx.moveTo(targetBase.x, targetBase.y);
  ctx.lineTo(targetTop.x, targetTop.y);
  ctx.stroke();
  ctx.setLineDash([]);

  if (preview.ok) {
    const points = preview.points.map(project);
    drawArmCapsules(ctx, points, preview, scale, { minOuterWidth: 8, minInnerWidth: 2 });
  }

  ctx.fillStyle = preview.ok && preview.safe ? "#ffd166" : "#ff6b5f";
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
    drawGizmoArrow(ctx, targetTop, xEnd, "#ff5d55", "X");
    drawGizmoArrow(ctx, targetTop, yEnd, "#4ba3ff", "Y");
    drawGizmoArrow(ctx, targetTop, zEnd, "#56d6a8", "Z");
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
  updateJointModeUi();
  const preview = armPreview();
  $("armConfiguredState").classList.toggle("fault", !preview.ok || !preview.safe);
  renderArmSolution(preview);
  renderArmSafety(preview);
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
    if (arm.jointCount === 2) {
      el.textContent =
        `2 joints: base ${$("armBaseMotorIdInput").value || "--"} (${$("armBaseModelInput").value}), ` +
        `shoulder ${$("armShoulderMotorIdInput").value || "--"} (${$("armShoulderModelInput").value})`;
    } else {
      el.textContent =
        `3 joints: base ${$("armBaseMotorIdInput").value || "--"} (${$("armBaseModelInput").value}), ` +
        `shoulder ${$("armShoulderMotorIdInput").value || "--"} (${$("armShoulderModelInput").value}), ` +
        `elbow ${$("armElbowMotorIdInput").value || "--"} (${$("armElbowModelInput").value})`;
    }
  } else if (step.key === "lengths") {
    el.textContent =
      `Reach ${(arm.link1 + arm.link2).toFixed(3)} m from links ${arm.link1.toFixed(3)} + ${arm.link2.toFixed(3)} m`;
  } else if (step.key === "safety") {
    const axes = activeAxesForArm(arm).map((axis) => {
      const degrees = radToDeg(normalizeTwistLimitRad(arm.twistLimits[axis])).toFixed(0);
      return `${axis} ${degrees} deg`;
    });
    el.textContent = `Link radii ${arm.radii.link1.toFixed(3)} / ${arm.radii.link2.toFixed(3)} m. Max twist ${axes.join(", ")}`;
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

function openIdSetup() {
  populateIdSetupNewOptions();
  setIdSetupMotorOptions(normalizedMotorList(state ? state.discoveredPrivate : []));
  updateIdSetupUi();
  $("idSetupFlow").hidden = false;
  document.body.classList.add("modal-open");
}

function closeIdSetup() {
  $("idSetupFlow").hidden = true;
  document.body.classList.remove("modal-open");
}

async function scanSingleMotorForIdSetup() {
  if (busy) return;
  busy = true;
  renderBusy(true);
  setIdSetupMessage("Scanning IDs on the bus...", true);
  try {
    await applyConfig();
    const result = await post("/api/command", { command: "id-scan" });
    if (result && result.ok === false) {
      setIdSetupMessage(`Scan failed: ${result.message || "no reply"}`, true);
      appendLocalLog(`ID setup scan failed: ${result.message || "no reply"}`);
    } else {
      appendLocalLog("ID setup scan complete");
    }
    await refresh();
    setIdSetupMotorOptions(normalizedMotorList(state ? state.discoveredPrivate : []));
    updateIdSetupUi();
    const motors = normalizedMotorList(state ? state.discoveredPrivate : []);
    setIdSetupMessage(
      motors.length === 1
        ? `Detected ${motors[0]}. Choose the IK role and set the new ID.`
        : motors.length > 1
          ? "Multiple unique IDs are online. That is OK if only one motor uses the Current ID you are changing."
          : "No IDs found. Check power/CAN wiring, or enter the current ID manually.",
      true,
    );
  } catch (error) {
    setIdSetupMessage(`UI error: ${error.message}`, true);
    appendLocalLog(`UI error: ${error.message}`);
  } finally {
    busy = false;
    renderBusy(false);
  }
}

async function assignMotorId() {
  if (busy) return;
  const oldMotorId = $("idSetupOldMotorInput").value;
  const newMotorId = $("idSetupNewMotorInput").value;
  const role = $("idSetupRoleInput").value;
  if (!oldMotorId || !newMotorId) {
    setIdSetupMessage("Select the current ID and new ID first.", true);
    return;
  }
  if (oldMotorId === newMotorId) {
    setIdSetupMessage("Pick a different new ID.", true);
    return;
  }
  const detectedMotors = normalizedMotorList(state ? state.discoveredPrivate : []);
  if (detectedMotors.includes(newMotorId) && newMotorId !== oldMotorId) {
    setIdSetupMessage(`${newMotorId} is already online. Choose a free ID.`, true);
    return;
  }
  if (!confirm(`Only one connected motor should currently use ${oldMotorId}. Already-assigned motors on other IDs can stay connected. If multiple motors still share ${oldMotorId}, they may all change or the command may fail.`)) {
    return;
  }

  busy = true;
  renderBusy(true);
  setIdSetupMessage(`Setting ${oldMotorId} to ${newMotorId}...`, true);
  try {
    await applyConfig();
    const result = await post("/api/command", {
      command: "assign-motor-id",
      oldMotorId,
      newMotorId,
      role,
      store: $("idSetupStoreToggle").checked,
    });
    if (result && result.ok === false) {
      setIdSetupMessage(`Set ID failed: ${result.message || "not verified"}`, true);
      appendLocalLog(`ID setup failed: ${result.message || "not verified"}`);
    } else {
      const message = result && result.message ? result.message : `Motor ID assigned ${oldMotorId} -> ${newMotorId}`;
      setIdSetupMessage(message, true);
      appendLocalLog(message);
      if (result && result.role && result.stored) {
        setValuesState("Saved");
      } else if (result && result.role) {
        setValuesState("Unsaved");
      }
    }
    await refresh();
    setIdSetupMotorOptions(normalizedMotorList(state ? state.discoveredPrivate : []));
    updateIdSetupUi();
  } catch (error) {
    setIdSetupMessage(`UI error: ${error.message}`, true);
    appendLocalLog(`UI error: ${error.message}`);
  } finally {
    busy = false;
    renderBusy(false);
  }
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

function updateJointModeUi() {
  const twoJoint = jointCountInput() === 2;
  document.querySelectorAll(".elbow-only").forEach((el) => {
    el.classList.toggle("mode-hidden", twoJoint);
  });
  ["armElbowMotorIdInput", "armElbowModelInput", "armElbowUpToggle", "armElbowTwistLimitInput"].forEach((id) => {
    const el = $(id);
    if (el) el.disabled = twoJoint;
  });
  const visual = document.querySelector(".wizard-joint-visual");
  if (visual) visual.classList.toggle("two-joint", twoJoint);
  document.querySelectorAll(".home-pose-visual").forEach((el) => {
    el.classList.toggle("two-joint", twoJoint);
  });
}

function render(state) {
  const detectedMotors = normalizedMotorList(state.discoveredPrivate);
  setControlValue("serialPortInput", state.serialPort || "auto");
  setControlValue("serialBaudInput", state.serialBaud || 921600);
  setControlValue("hostIdInput", state.hostIdHex);
  setControlValue("modelInput", state.model);
  setMotorSelectOptions("motorIdInput", detectedMotors, state.motorIdHex, detectedMotors[0] || "");

  const status = $("connectionStatus");
  status.textContent = state.connected ? "Online" : "Offline";
  status.className = `status-pill ${state.connected ? "online" : "offline"}`;
  $("subtitle").textContent = state.openError || state.transportLabel || "Helion control surface";
  $("appVersion").textContent = state.appVersion ? `v${state.appVersion}` : "v--";
  setIdSetupMotorOptions(detectedMotors);
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
  setControlValue("wizardJointCountInput", Number(arm.jointCount) === 2 ? "2" : "3");
  updateJointModeUi();
  const armIds = arm.motorIdHex || {};
  const armModels = arm.models || {};
  const armRadii = arm.radii || {};
  const armOffsets = arm.offsets || {};
  const armDirections = arm.directions || {};
  const armTwistLimits = arm.twistLimits || {};
  const armTarget = arm.target || {};
  const usedArmMotors = new Set();
  const baseMotor = chooseDetectedMotor(detectedMotors, armIds.base, usedArmMotors);
  const shoulderMotor = chooseDetectedMotor(detectedMotors, armIds.shoulder, usedArmMotors);
  const elbowMotor = chooseDetectedMotor(detectedMotors, armIds.elbow, usedArmMotors);
  setMotorSelectOptions("armBaseMotorIdInput", detectedMotors, baseMotor, baseMotor);
  setMotorSelectOptions("armShoulderMotorIdInput", detectedMotors, shoulderMotor, shoulderMotor);
  setMotorSelectOptions("armElbowMotorIdInput", detectedMotors, elbowMotor, elbowMotor);
  setControlValue("armBaseModelInput", armModels.base || state.model || "rs-05");
  setControlValue("armShoulderModelInput", armModels.shoulder || state.model || "rs-05");
  setControlValue("armElbowModelInput", armModels.elbow || state.model || "rs-05");
  setControlValue("armLink1Input", Number(arm.link1 || 0.25).toFixed(3));
  setControlValue("armLink2Input", Number(arm.link2 || 0.25).toFixed(3));
  setControlValue("armLink1RadiusInput", Number(armRadii.link1 ?? 0.015).toFixed(3));
  setControlValue("armLink2RadiusInput", Number(armRadii.link2 ?? 0.015).toFixed(3));
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
  setControlValue(
    "armBaseTwistLimitInput",
    radToDeg(normalizeTwistLimitRad(armTwistLimits.base)).toFixed(1),
  );
  setControlValue(
    "armShoulderTwistLimitInput",
    radToDeg(normalizeTwistLimitRad(armTwistLimits.shoulder)).toFixed(1),
  );
  setControlValue(
    "armElbowTwistLimitInput",
    radToDeg(normalizeTwistLimitRad(armTwistLimits.elbow)).toFixed(1),
  );
  $("armConfiguredState").textContent = Number(arm.routeRemaining || 0) > 0
    ? "Routing IK"
    : arm.configured
      ? "Holding IK"
      : "Idle";
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
  updateIdSetupUi();
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
  updateJointModeUi();
  renderIkPreview();
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
$("idSetupIkBtn").addEventListener("click", openIdSetup);
$("wizardCloseBtn").addEventListener("click", closeWizard);
$("wizardBackBtn").addEventListener("click", () => stepWizard(-1));
$("wizardNextBtn").addEventListener("click", () => stepWizard(1));
$("ikWizardFlow").addEventListener("click", (event) => {
  if (event.target === $("ikWizardFlow")) closeWizard();
});

$("idSetupBtn").addEventListener("click", openIdSetup);
$("idSetupCloseBtn").addEventListener("click", closeIdSetup);
$("idSetupScanBtn").addEventListener("click", scanSingleMotorForIdSetup);
$("idSetupAssignBtn").addEventListener("click", assignMotorId);
$("idSetupFlow").addEventListener("click", (event) => {
  if (event.target === $("idSetupFlow")) closeIdSetup();
});
$("idSetupRoleInput").addEventListener("change", () => {
  setIdSetupRoleDefault(true);
  updateIdSetupUi();
});
["idSetupOldMotorInput", "idSetupNewMotorInput", "idSetupStoreToggle"].forEach((id) => {
  $(id).addEventListener("input", updateIdSetupUi);
  $(id).addEventListener("change", updateIdSetupUi);
});

$("targetEditorFlow").addEventListener("click", (event) => {
  if (event.target === $("targetEditorFlow")) closeTargetEditor();
});

window.addEventListener("keydown", (event) => {
  if (event.key === "Escape" && !$("ikWizardFlow").hidden) closeWizard();
  if (event.key === "Escape" && !$("idSetupFlow").hidden) closeIdSetup();
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
