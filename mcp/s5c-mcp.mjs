#!/usr/bin/env node
import net from "node:net";
import http from "node:http";

const VERSION = "1.0.0";
const PROTOCOLS = ["2024-11-05", "2025-03-26", "2025-06-18"];
const SERVER = (process.env.S5C_SERVER || "http://127.0.0.1:8080").replace(/\/+$/, "");
const KEY = process.env.S5C_KEY || "";
const DRONE_HOST = process.env.S5C_HOST || "172.19.10.1";
const MIN_BATTERY = Number(process.env.S5C_MIN_BATTERY ?? 20);
const TOOL_NAME = /^s5c_[a-z][a-z0-9_]*$/;
const LABEL = /^[a-z][a-z0-9_]*$/;
const MAX_BYTES = 64 * 1024;

function refuse(message) {
  process.stderr.write(`s5c-mcp: ${message}\n`);
  process.exit(78);
}

function scalar(v) {
  if (v === null || v === undefined) return "null";
  if (typeof v === "boolean") return v ? "true" : "false";
  if (typeof v === "number") return Number.isFinite(v) ? String(v) : "nan";
  const s = String(v).replace(/\r?\n/g, "\\n").trim();
  return s.length > 2000 ? s.slice(0, 2000) + "..." : s;
}

function flatten(value, prefix, out) {
  if (Array.isArray(value)) {
    out[prefix] = value.map(scalar).join(",");
    return out;
  }
  if (value && typeof value === "object") {
    for (const [k, v] of Object.entries(value)) flatten(v, prefix ? `${prefix}.${k}` : k, out);
    return out;
  }
  out[prefix] = scalar(value);
  return out;
}

function block(tool, outcome, fields) {
  const flat = flatten({ ...fields }, "", {});
  const keys = Object.keys(flat).sort();
  for (const k of keys) {
    for (const part of k.split(".")) {
      if (!LABEL.test(part)) throw new Error(`illegal label ${k}`);
    }
  }
  return [`S5C-ELITE ${tool} ${outcome}`, ...keys.map((k) => `${k}=${flat[k]}`)].join("\n");
}

function ok(tool, fields) {
  return { content: [{ type: "text", text: block(tool, "ok", fields) }] };
}

function fail(tool, error, detail = "") {
  return {
    content: [{ type: "text", text: block(tool, "error", { error, detail }) }],
    isError: true,
  };
}

function url(path) {
  const u = new URL(`${SERVER}${path}`);
  if (KEY) u.searchParams.set("k", KEY);
  return u;
}

function request(method, path, body) {
  return new Promise((resolve, reject) => {
    const payload = body === undefined ? null : JSON.stringify(body);
    const u = url(path);
    const req = http.request(
      {
        method,
        hostname: u.hostname,
        port: u.port || 80,
        path: `${u.pathname}${u.search}`,
        headers: payload
          ? { "content-type": "application/json", "content-length": Buffer.byteLength(payload) }
          : {},
        timeout: 8000,
      },
      (res) => {
        const chunks = [];
        let size = 0;
        res.on("data", (c) => {
          size += c.length;
          if (size <= MAX_BYTES) chunks.push(c);
        });
        res.on("end", () => {
          resolve({ status: res.statusCode, body: Buffer.concat(chunks).toString("utf8") });
        });
      },
    );
    req.on("timeout", () => req.destroy(new Error("control server did not answer within 8 s")));
    req.on("error", reject);
    if (payload) req.write(payload);
    req.end();
  });
}

async function state() {
  const r = await request("GET", "/state");
  if (r.status !== 200) throw new Error(`control server answered ${r.status}`);
  return JSON.parse(r.body);
}

async function command(tool, cmd, extra = {}) {
  const before = await state();
  await request("POST", "/cmd", { cmd });
  return ok(tool, {
    command: cmd,
    sent: true,
    "drone.battery": before.battery,
    "drone.connected": before.connected,
    ...extra,
  });
}

async function flying(tool) {
  const s = await state();
  if (!s.connected) return { stop: fail(tool, "not_connected", "run s5c_connect and power the drone on") };
  if (s.battery < MIN_BATTERY) return { stop: fail(tool, "battery_too_low", `${s.battery}% is under S5C_MIN_BATTERY=${MIN_BATTERY}`) };
  return { stop: null, state: s };
}

function droneShell(command, seconds) {
  return new Promise((resolve, reject) => {
    const mark = "__S5C_DONE__";
    const socket = net.createConnection({ host: DRONE_HOST, port: 23 });
    let out = "";
    let ready = false;
    let settled = false;
    const finish = (error) => {
      if (settled) return;
      settled = true;
      socket.destroy();
      if (error) return reject(error);
      const text = out.replace(/\r/g, "").split("\n").slice(1).join("\n");
      resolve(text.split(mark)[0].trimEnd());
    };
    socket.setTimeout((seconds + 5) * 1000);
    socket.on("timeout", () => finish(new Error(`telnet to ${DRONE_HOST}:23 timed out`)));
    socket.on("error", (e) => finish(e));
    socket.on("connect", () => {
      setTimeout(() => {
        ready = true;
        socket.write(`${command} 2>&1; echo ${mark}
`);
      }, 600);
    });
    socket.on("data", (buf) => {
      if (!ready) {
        const reply = [];
        for (let i = 0; i + 2 < buf.length; ) {
          if (buf[i] === 255) {
            if (buf[i + 1] === 251 || buf[i + 1] === 252) reply.push(255, 254, buf[i + 2]);
            else if (buf[i + 1] === 253 || buf[i + 1] === 254) reply.push(255, 252, buf[i + 2]);
            i += 3;
          } else i += 1;
        }
        if (reply.length) socket.write(Buffer.from(reply));
        return;
      }
      out += buf.toString("utf8");
      if (out.split(mark).length >= 3) finish(null);
    });
    setTimeout(() => finish(null), (seconds + 0.5) * 1000);
  });
}

const TOOLS = [
  {
    name: "s5c_status",
    description: "Read the drone's fused pose, battery, link and video health once. Works on the ground and airborne.",
    inputSchema: { type: "object", properties: {}, additionalProperties: false },
    run: async () => {
      const s = await state();
      const { ms, ...rest } = s;
      return ok("s5c_status", { drone: rest, read_at_ms: ms });
    },
  },
  {
    name: "s5c_connect",
    description: "Log in to the drone's control link and keep reconnecting while it stays reachable. Run it after powering the drone on.",
    inputSchema: { type: "object", properties: {}, additionalProperties: false },
    run: async () => {
      await request("POST", "/connect");
      return ok("s5c_connect", { connecting: true });
    },
  },
  {
    name: "s5c_disconnect",
    description: "Stop trying to reach the drone and close the control link. Only safe on the ground.",
    inputSchema: { type: "object", properties: {}, additionalProperties: false },
    run: async () => {
      await request("POST", "/disconnect");
      return ok("s5c_disconnect", { disconnecting: true });
    },
  },
  {
    name: "s5c_takeoff",
    description: "Arm the motors and take off straight up to the drone's hover height. Needs confirm=true and 20 percent battery.",
    inputSchema: {
      type: "object",
      properties: { confirm: { type: "boolean", description: "Must be true, or the call is refused." } },
      additionalProperties: false,
    },
    run: async (args) => {
      if (args.confirm !== true) return fail("s5c_takeoff", "confirmation_required", "call again with confirm=true");
      const gate = await flying("s5c_takeoff");
      if (gate.stop) return gate.stop;
      return command("s5c_takeoff", "takeoff", { "drone.fly_before": gate.state.fly });
    },
  },
  {
    name: "s5c_land",
    description: "Cut the sticks and descend until the drone reports it is on the ground. Works on the ground and airborne.",
    inputSchema: { type: "object", properties: {}, additionalProperties: false },
    run: async () => command("s5c_land", "land"),
  },
  {
    name: "s5c_stop",
    description: "Stop the motors at once and cancel every stick input. The immediate answer to anything going wrong.",
    inputSchema: { type: "object", properties: {}, additionalProperties: false },
    run: async () => command("s5c_stop", "stop"),
  },
  {
    name: "s5c_home",
    description: "Fly back to the pinned takeoff point using vision and soft-land there. Needs confirm=true.",
    inputSchema: {
      type: "object",
      properties: { confirm: { type: "boolean", description: "Must be true, or the call is refused." } },
      additionalProperties: false,
    },
    run: async (args) => {
      if (args.confirm !== true) return fail("s5c_home", "confirmation_required", "call again with confirm=true");
      const gate = await flying("s5c_home");
      if (gate.stop) return gate.stop;
      if (!gate.state.home_set) return fail("s5c_home", "home_not_set", "take off once so the takeoff point is pinned");
      return command("s5c_home", "home");
    },
  },
  {
    name: "s5c_calibrate",
    description: "Send the drone's gyro calibration command. Put the drone still and level on the ground before calling.",
    inputSchema: { type: "object", properties: {}, additionalProperties: false },
    run: async () => command("s5c_calibrate", "calibrate"),
  },
  {
    name: "s5c_hold",
    description: "Toggle hover hold, which keeps the drone's view when the sticks go back to centre.",
    inputSchema: { type: "object", properties: {}, additionalProperties: false },
    run: async () => {
      const before = await state();
      await request("POST", "/cmd", { cmd: "hold" });
      const after = await state();
      return ok("s5c_hold", { "hold.was": before.hold_on, "hold.now": after.hold_on });
    },
  },
  {
    name: "s5c_fly",
    description: "Hold one stick input for a set time, then centre every stick. Roll and pitch move, throttle climbs, yaw turns. Needs confirm=true and to leave every value between -1 and 1.",
    inputSchema: {
      type: "object",
      properties: {
        confirm: { type: "boolean", description: "Must be true, or the call is refused." },
        roll: { type: "number", description: "Right positive, -1 to 1." },
        pitch: { type: "number", description: "Forward positive, -1 to 1." },
        throttle: { type: "number", description: "Climb positive, -1 to 1." },
        yaw: { type: "number", description: "Turn right positive, -1 to 1." },
        duration_ms: { type: "integer", description: "How long to hold it, 100 to 4000." },
      },
      required: ["roll", "pitch", "throttle", "yaw", "duration_ms"],
      additionalProperties: false,
    },
    run: async (args) => {
      const tool = "s5c_fly";
      if (args.confirm !== true) return fail(tool, "confirmation_required", "call again with confirm=true");
      const gate = await flying(tool);
      if (gate.stop) return gate.stop;
      const stick = { roll: args.roll, pitch: args.pitch, throttle: args.throttle, yaw: args.yaw };
      for (const [k, v] of Object.entries(stick)) {
        if (typeof v !== "number" || !Number.isFinite(v) || Math.abs(v) > 1) {
          return fail(tool, "stick_out_of_range", `${k} must be a number between -1 and 1`);
        }
      }
      if (!Number.isInteger(args.duration_ms) || args.duration_ms < 100 || args.duration_ms > 4000) {
        return fail(tool, "duration_out_of_range", "duration_ms must be an integer from 100 to 4000");
      }
      await request("POST", "/control", stick);
      await new Promise((r) => setTimeout(r, args.duration_ms));
      await request("POST", "/control", { roll: 0, pitch: 0, throttle: 0, yaw: 0 });
      const after = await state();
      return ok(tool, {
        stick,
        "stick.duration_ms": args.duration_ms,
        "drone.x": after.x,
        "drone.y": after.y,
        "drone.z": after.z,
        "drone.heading": after.heading,
        "drone.battery": after.battery,
      });
    },
  },
  {
    name: "s5c_hovertest",
    description: "Take off, hover for 20 s with no stick input, soft-land and record the flight. Needs confirm=true.",
    inputSchema: {
      type: "object",
      properties: { confirm: { type: "boolean", description: "Must be true, or the call is refused." } },
      additionalProperties: false,
    },
    run: async (args) => {
      if (args.confirm !== true) return fail("s5c_hovertest", "confirmation_required", "call again with confirm=true");
      const gate = await flying("s5c_hovertest");
      if (gate.stop) return gate.stop;
      return command("s5c_hovertest", "hovertest", { "test.duration_s": 20 });
    },
  },
  {
    name: "s5c_wait",
    description: "Poll the drone until it reaches a named condition or the timeout runs out. Use it instead of sleeping between steps.",
    inputSchema: {
      type: "object",
      properties: {
        until: {
          type: "string",
          enum: ["connected", "disconnected", "airborne", "landed", "homing_done", "battery_at_least"],
          description: "The condition to wait for.",
        },
        battery: { type: "integer", description: "Percent, used only when until is battery_at_least." },
        timeout_ms: { type: "integer", description: "Give up after this long, 500 to 60000. Default 20000." },
      },
      required: ["until"],
      additionalProperties: false,
    },
    run: async (args) => {
      const tool = "s5c_wait";
      const timeout = Number.isInteger(args.timeout_ms) ? Math.min(Math.max(args.timeout_ms, 500), 60000) : 20000;
      const test = {
        connected: (s) => s.connected === true,
        disconnected: (s) => s.connected === false,
        airborne: (s) => s.fly === 2 || s.fly === 3,
        landed: (s) => s.fly === 0 || s.fly === 1,
        homing_done: (s) => s.homing === false,
        battery_at_least: (s) => s.battery >= args.battery,
      }[args.until];
      if (!test) return fail(tool, "unknown_condition", "until must be connected, disconnected, airborne, landed, homing_done or battery_at_least");
      if (args.until === "battery_at_least" && !Number.isInteger(args.battery)) {
        return fail(tool, "battery_required", "battery must be an integer percent");
      }
      const start = Date.now();
      let s = await state();
      while (!test(s) && Date.now() - start < timeout) {
        await new Promise((r) => setTimeout(r, 400));
        s = await state();
      }
      const met = test(s);
      return ok(tool, {
        "wait.until": args.until,
        "wait.met": met,
        "wait.waited_ms": Date.now() - start,
        "drone.connected": s.connected,
        "drone.fly": s.fly,
        "drone.battery": s.battery,
        "drone.status": s.status,
      });
    },
  },
  {
    name: "s5c_snapshot",
    description: "Grab the newest 1080p camera frame as a JPEG image, so the agent can look at what the drone sees.",
    inputSchema: { type: "object", properties: {}, additionalProperties: false },
    run: async () => {
      const tool = "s5c_snapshot";
      const buf = await new Promise((resolve, reject) => {
        const u = url("/video.mjpg");
        const req = http.request(
          { hostname: u.hostname, port: u.port || 80, path: `${u.pathname}${u.search}`, timeout: 5000 },
          (res) => {
            if (res.statusCode !== 200) {
              reject(new Error(`control server answered ${res.statusCode}`));
              res.resume();
              return;
            }
            let acc = Buffer.alloc(0);
            res.on("data", (c) => {
              acc = Buffer.concat([acc, c]);
              const head = acc.indexOf("Content-Type: image/jpeg");
              const sois = acc.indexOf(Buffer.from([0xff, 0xd8, 0xff]), head < 0 ? 0 : head);
              if (sois < 0) return;
              const eoi = acc.indexOf(Buffer.from([0xff, 0xd9]), sois + 3);
              if (eoi < 0) return;
              res.destroy();
              resolve(acc.subarray(sois, eoi + 2));
            });
          },
        );
        req.on("timeout", () => req.destroy(new Error("no camera frame within 5 s")));
        req.on("error", reject);
        req.end();
      });
      const s = await state();
      return {
        content: [
          { type: "text", text: block(tool, "ok", { "video.bytes": buf.length, "video.mime": "image/jpeg", "drone.frame_age": s.frame_age ?? "null", "drone.fps": s.fps }) },
          { type: "image", data: buf.toString("base64"), mimeType: "image/jpeg" },
        ],
      };
    },
  },
  {
    name: "s5c_shell",
    description: "Run one BusyBox shell command on the drone's own CPU over telnet and return its output. Needs confirm=true.",
    inputSchema: {
      type: "object",
      properties: {
        confirm: { type: "boolean", description: "Must be true, or the call is refused." },
        command: { type: "string", description: "One shell command, for example ls / or cat /proc/meminfo." },
        wait_s: { type: "integer", description: "How long to collect output, 1 to 20. Default 6." },
      },
      required: ["command"],
      additionalProperties: false,
    },
    run: async (args) => {
      const tool = "s5c_shell";
      if (args.confirm !== true) return fail(tool, "confirmation_required", "call again with confirm=true");
      if (typeof args.command !== "string" || !args.command.trim()) return fail(tool, "empty_command", "command must be one non-empty line");
      const line = args.command.replace(/\r?\n/g, " ; ");
      const seconds = Number.isInteger(args.wait_s) ? Math.min(Math.max(args.wait_s, 1), 20) : 6;
      const out = await droneShell(line, seconds);
      return ok(tool, { "shell.command": line, "shell.host": DRONE_HOST, "shell.output": out });
    },
  },
];

for (const t of TOOLS) {
  if (!TOOL_NAME.test(t.name)) refuse(`illegal tool name ${t.name}`);
  if (!t.description.endsWith(".")) refuse(`description of ${t.name} must end with a full stop`);
}

async function call(name, args) {
  const tool = TOOLS.find((t) => t.name === name);
  if (!tool) return fail(String(name).replace(/[^a-z0-9_]/gi, "_"), "unknown_tool", `no tool named ${name}`);
  if (args !== null && (typeof args !== "object" || Array.isArray(args))) {
    return fail(name, "bad_arguments", "arguments must be an object");
  }
  const schema = tool.inputSchema;
  const given = args || {};
  if (schema.additionalProperties === false) {
    for (const k of Object.keys(given)) {
      if (!(k in schema.properties)) return fail(name, "unknown_argument", `${k} is not an argument of ${name}`);
    }
  }
  for (const k of schema.required || []) {
    if (!(k in given)) return fail(name, "missing_argument", `${k} is required`);
  }
  try {
    return await tool.run(given);
  } catch (e) {
    return fail(name, e.code || "failed", e.message);
  }
}

function send(message) {
  process.stdout.write(`${JSON.stringify(message)}\n`);
}

const queue = [];
let busy = false;
let buffer = "";

function pump() {
  if (busy) return;
  const line = queue.shift();
  if (line === undefined) return;
  busy = true;
  handle(line).finally(() => {
    busy = false;
    pump();
  });
}

process.stdin.setEncoding("utf8");
process.stdin.on("data", (chunk) => {
  buffer += chunk;
  let cut;
  while ((cut = buffer.indexOf("\n")) >= 0) {
    const line = buffer.slice(0, cut).trim();
    buffer = buffer.slice(cut + 1);
    if (line) queue.push(line);
  }
  pump();
});
process.stdin.on("end", () => {
  const drain = () => (busy || queue.length ? setTimeout(drain, 20) : (process.stdin.pause(), process.exit(0)));
  drain();
});

async function handle(line) {
  let msg;
  try {
    msg = JSON.parse(line);
  } catch {
    send({ jsonrpc: "2.0", id: null, error: { code: -32700, message: "parse error" } });
    return;
  }
  const { id, method, params } = msg;
  if (method === undefined) return;
  if (id === undefined) return;
  try {
    if (method === "initialize") {
      const asked = params?.protocolVersion;
      send({
        jsonrpc: "2.0",
        id,
        result: {
          protocolVersion: PROTOCOLS.includes(asked) ? asked : PROTOCOLS[0],
          capabilities: { tools: { listChanged: false } },
          serverInfo: { name: "s5c-elite-drone", version: VERSION },
          instructions: "Control a SNAPTAIN S5C Elite drone. Call s5c_status first, then s5c_connect, then the flight tools. Every result is the line S5C-ELITE <tool> <ok|error> followed by sorted key=value labels and nothing else.",
        },
      });
    } else if (method === "ping") {
      send({ jsonrpc: "2.0", id, result: {} });
    } else if (method === "tools/list") {
      send({
        jsonrpc: "2.0",
        id,
        result: { tools: TOOLS.map(({ name, description, inputSchema }) => ({ name, description, inputSchema })) },
      });
    } else if (method === "tools/call") {
      const name = params?.name;
      send({ jsonrpc: "2.0", id, result: await call(name, params?.arguments) });
    } else {
      send({ jsonrpc: "2.0", id, error: { code: -32601, message: `method not found: ${method}` } });
    }
  } catch (e) {
    send({ jsonrpc: "2.0", id, error: { code: -32603, message: e.message } });
  }
}

process.on("SIGINT", () => process.exit(0));
process.on("SIGTERM", () => process.exit(0));
process.stderr.write(`s5c-mcp ${VERSION} ready: control server ${SERVER}${KEY ? " (key set)" : ""}, drone ${DRONE_HOST}, tools ${TOOLS.length}\n`);
