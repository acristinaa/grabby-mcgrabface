"use strict";

const $ = (id) => document.getElementById(id);
const view = $("view");

let ws = null, meta = null, lastUrl = null;
let mountSide = "left";
let mount = null, mountDefault = null;
const actInputs = new Map();          // Aktuatorindex -> <input>

const cam = { lookat: [0.35, 0, 0.95], distance: 1.8, azimuth: 135, elevation: -15 };

// ---------- Verbindung ----------

function connect() {
  const proto = location.protocol === "https:" ? "wss:" : "ws:";
  ws = new WebSocket(`${proto}//${location.host}/ws`);
  ws.binaryType = "arraybuffer";

  ws.onopen = () => setConn(true, "verbunden");
  ws.onclose = () => { setConn(false, "getrennt – neuer Versuch…"); setTimeout(connect, 1200); };
  ws.onerror = () => setConn(false, "Fehler");

  ws.onmessage = (ev) => {
    if (typeof ev.data !== "string") { showFrame(ev.data); return; }
    const msg = JSON.parse(ev.data);
    if (msg.type === "meta") onMeta(msg.data);
    else if (msg.type === "state") onState(msg.data);
  };
}

function setConn(ok, text) {
  $("dot").classList.toggle("on", ok);
  $("connTxt").textContent = text;
}

function send(cmd) {
  if (ws && ws.readyState === WebSocket.OPEN) ws.send(JSON.stringify(cmd));
}

function showFrame(buf) {
  const url = URL.createObjectURL(new Blob([buf], { type: "image/jpeg" }));
  view.src = url;
  if (lastUrl) URL.revokeObjectURL(lastUrl);   // sonst laeuft der Speicher voll
  lastUrl = url;
}

// ---------- Aufbau nach Metadaten ----------

function onMeta(data) {
  meta = data;
  if (!mountDefault) mountDefault = structuredClone(data.mount);
  mount = structuredClone(data.mount);
  buildCameras(data.cameras);
  buildGroups(data.actuators);
  buildMount();
  $("mountPanel").hidden = !data.supports_mount_tuning;
}

function buildCameras(names) {
  const sel = $("camera");
  if (sel.options.length === names.length) return;
  sel.innerHTML = "";
  for (const n of names) {
    const o = document.createElement("option");
    o.value = n;
    o.textContent = n === "__frei__" ? "frei bewegen" : n;
    sel.appendChild(o);
  }
  sel.onchange = () => send({ type: "camera", value: sel.value });
}

function buildGroups(actuators) {
  const host = $("groups");
  if (actInputs.size === actuators.length) return;   // nur einmal aufbauen
  host.innerHTML = "";
  actInputs.clear();

  const order = [];
  const byGroup = new Map();
  for (const a of actuators) {
    if (!byGroup.has(a.group)) { byGroup.set(a.group, []); order.push(a.group); }
    byGroup.get(a.group).push(a);
  }

  for (const g of order) {
    const det = document.createElement("details");
    if (g.includes("Hand") || g.includes("Arm")) det.open = true;
    const sum = document.createElement("summary");
    sum.textContent = `${g} (${byGroup.get(g).length})`;
    det.appendChild(sum);

    for (const a of byGroup.get(g)) {
      const label = document.createElement("label");
      label.className = "slider";
      const short = a.name.replace(/^(lh_left_|rh_right_|left_|right_)/, "").replace(/_joint$/, "");
      label.innerHTML = `<span><i class="name">${short}</i><b>0.00</b></span>`;
      const inp = document.createElement("input");
      inp.type = "range";
      inp.min = a.low; inp.max = a.high; inp.step = 0.005; inp.value = 0;
      inp.dataset.index = a.index;
      inp.oninput = () => {
        label.querySelector("b").textContent = (+inp.value).toFixed(2);
        send({ type: "ctrl", index: a.index, value: +inp.value });
      };
      label.appendChild(inp);
      det.appendChild(label);
      actInputs.set(a.index, inp);
    }
    host.appendChild(det);
  }
}

// ---------- Handmontage ----------

const MOUNT_FIELDS = [
  { key: "pos", i: 0, label: "Position X", min: -0.15, max: 0.15, step: 0.001, unit: "m" },
  { key: "pos", i: 1, label: "Position Y", min: -0.15, max: 0.15, step: 0.001, unit: "m" },
  { key: "pos", i: 2, label: "Position Z", min: -0.15, max: 0.15, step: 0.001, unit: "m" },
  { key: "euler", i: 0, label: "Drehung X", min: -3.15, max: 3.15, step: 0.01, unit: "rad" },
  { key: "euler", i: 1, label: "Drehung Y", min: -3.15, max: 3.15, step: 0.01, unit: "rad" },
  { key: "euler", i: 2, label: "Drehung Z", min: -3.15, max: 3.15, step: 0.01, unit: "rad" },
];

let mountTimer = null;
function pushMount() {                     // entprellt, Neuaufbau kostet ~180 ms
  clearTimeout(mountTimer);
  mountTimer = setTimeout(() => send({ type: "mount", value: mount }), 160);
}

function buildMount() {
  const host = $("mountSliders");
  host.innerHTML = "";
  for (const f of MOUNT_FIELDS) {
    const val = mount[mountSide][f.key][f.i];
    const label = document.createElement("label");
    label.className = "slider";
    label.innerHTML = `<span>${f.label}<b>${val.toFixed(3)} ${f.unit}</b></span>`;
    const inp = document.createElement("input");
    inp.type = "range";
    inp.min = f.min; inp.max = f.max; inp.step = f.step; inp.value = val;
    inp.oninput = () => {
      mount[mountSide][f.key][f.i] = +inp.value;
      label.querySelector("b").textContent = `${(+inp.value).toFixed(3)} ${f.unit}`;
      pushMount();
    };
    label.appendChild(inp);
    host.appendChild(label);
  }
}

function fmtMount() {
  const one = (s) => {
    const m = mount[s];
    const p = m.pos.map((v) => v.toFixed(4)).join(", ");
    const e = m.euler.map((v) => v.toFixed(4)).join(", ");
    return `    "${s}": {"pos": [${p}], "euler": [${e}]},`;
  };
  return "DEFAULT_MOUNT = {\n" + one("left") + "\n" + one("right") + "\n}";
}

// ---------- Zustand ----------

function onState(s) {
  $("t").textContent = (s.time ?? 0).toFixed(2);
  $("sl").textContent = s.spread?.left != null ? s.spread.left.toFixed(1) : "–";
  $("sr").textContent = s.spread?.right != null ? s.spread.right.toFixed(1) : "–";
  $("err").textContent = s.error || "";
  $("pause").textContent = s.paused ? "Weiter" : "Pause";

  if (s.ctrl) {
    for (const [i, inp] of actInputs) {
      if (inp === document.activeElement) continue;    // nicht gegen den Nutzer schreiben
      const v = s.ctrl[i];
      if (v == null || Math.abs(+inp.value - v) < 1e-4) continue;
      inp.value = v;
      inp.parentElement.querySelector("b").textContent = v.toFixed(2);
    }
  }
}

// ---------- Bedienung ----------

$("pause").onclick = () => send({ type: "pause", value: $("pause").textContent === "Pause" });
$("reset").onclick = () => send({ type: "reset" });
$("zero").onclick = () => { send({ type: "zero" }); $("hands").value = 0; $("handVal").textContent = "0.00"; };

document.querySelectorAll("[data-hands]").forEach((b) => {
  b.onclick = () => {
    const v = +b.dataset.hands;
    send({ type: "hands", value: v });
    $("hands").value = v;
    $("handVal").textContent = v.toFixed(2);
  };
});
$("hands").oninput = (e) => {
  $("handVal").textContent = (+e.target.value).toFixed(2);
  send({ type: "hands", value: +e.target.value });
};

$("mountSide").onclick = (e) => {
  const b = e.target.closest("[data-side]");
  if (!b) return;
  mountSide = b.dataset.side;
  document.querySelectorAll("#mountSide .seg-btn").forEach((x) => x.classList.toggle("active", x === b));
  buildMount();
};
$("mountReset").onclick = () => { mount = structuredClone(mountDefault); buildMount(); pushMount(); };
$("mountApply").onclick = () => {
  const out = $("mountOut");
  out.hidden = false;
  out.textContent = "# in dashboard/scene.py einsetzen:\n" + fmtMount();
};

// Freie Kamera per Maus
let drag = null;
view.addEventListener("pointerdown", (e) => {
  drag = { x: e.clientX, y: e.clientY };
  view.classList.add("drag");
  view.setPointerCapture(e.pointerId);
});
view.addEventListener("pointermove", (e) => {
  if (!drag) return;
  cam.azimuth -= (e.clientX - drag.x) * 0.4;
  cam.elevation = Math.max(-89, Math.min(89, cam.elevation - (e.clientY - drag.y) * 0.3));
  drag = { x: e.clientX, y: e.clientY };
  send({ type: "cam_params", value: cam });
});
view.addEventListener("pointerup", (e) => {
  drag = null; view.classList.remove("drag");
  view.releasePointerCapture(e.pointerId);
});
view.addEventListener("wheel", (e) => {
  e.preventDefault();
  cam.distance = Math.max(0.15, Math.min(8, cam.distance * (1 + Math.sign(e.deltaY) * 0.1)));
  send({ type: "cam_params", value: cam });
}, { passive: false });

connect();
