# s5c-elite-mcp

An MCP server that gives Claude Code, Claude Desktop or any MCP client the controls of a **SNAPTAIN S5C Elite** drone: telemetry, takeoff, sticks, return home, camera frames and the drone's own BusyBox shell.

It is a plain Node script with no dependencies. It talks to the control server this repository builds (`http://127.0.0.1:8080`), so the drone app must be running on the same PC.

## Install

Claude Code, from the repository root:

```
claude mcp add s5c-elite -- node mcp/s5c-mcp.mjs
```

Claude Code also reads the `.mcp.json` in this repository, so opening the folder and approving the `s5c-elite` server is enough.

Claude Desktop, in `claude_desktop_config.json`:

```json
{
  "mcpServers": {
    "s5c-elite": { "command": "node", "args": ["C:\\path\\to\\s5c-elite\\mcp\\s5c-mcp.mjs"] }
  }
}
```

Any other client: run `node mcp/s5c-mcp.mjs` and speak MCP over stdin and stdout.

## Configuration

| Variable | Default | What it does |
|---|---|---|
| `S5C_SERVER` | `http://127.0.0.1:8080` | The control server. Point it at a tunnel if you run the drone app elsewhere. |
| `S5C_KEY` | empty | The access key in `runtime/access-key.txt`. Only needed when the control server is not on `127.0.0.1`. |
| `S5C_HOST` | `172.19.10.1` | The drone's address on its own hotspot, used by `s5c_shell`. |
| `S5C_MIN_BATTERY` | `20` | Percent under which every tool that leaves the ground refuses. |

## The output format, and why it is strict

Every tool returns exactly one text block in this shape and nothing else:

```
S5C-ELITE s5c_status ok
drone.battery=87
drone.connected=true
drone.heading=12.34
...
```

The rules, which the server enforces rather than suggests:

1. Line one is three space-separated tokens: the fixed word `S5C-ELITE`, the tool name exactly as it was called, and `ok` or `error`.
2. Every following line is one `label=value` with no spaces around the equals sign.
3. A label is one or more `snake_case` parts joined by full stops, each part matching `[a-z][a-z0-9_]*`. Labels are sorted with a plain byte comparison, never in insertion order.
4. A value is `true`, `false`, `null`, a decimal number with no unit, or single-line text. A newline inside a value is written as `\n`, and a value longer than 2000 characters is cut and ends in `...`.
5. There are no blank lines, no prose, no comments, no repeated labels and no trailing spaces.
6. A failure is the same shape with `error`, with the outcome token `error`, `isError: true`, and at least the labels `error=<machine code>` and `detail=<one line>`.
7. Tool names are `s5c_` plus lower snake case. The server refuses to start if a tool name breaks that pattern or a description does not end in a full stop, and it refuses to answer with any label that breaks rule 3.
8. Arguments are checked against the schema before anything is sent: an argument the tool does not take is refused with `unknown_argument`, a missing required argument with `missing_argument`.

Error codes in use: `confirmation_required`, `not_connected`, `battery_too_low`, `battery_required`, `home_not_set`, `stick_out_of_range`, `duration_out_of_range`, `empty_command`, `unknown_argument`, `missing_argument`, `unknown_condition`, `unknown_tool`, `bad_arguments`, `failed`.

## Tools

| Tool | confirm | Arguments | Reaches |
|---|---|---|---|
| `s5c_status` | no | — | `GET /state` |
| `s5c_connect` | no | — | `POST /connect` |
| `s5c_disconnect` | no | — | `POST /disconnect` |
| `s5c_takeoff` | yes | `confirm` | `POST /cmd` `takeoff` |
| `s5c_land` | no | — | `POST /cmd` `land` |
| `s5c_stop` | no | — | `POST /cmd` `stop` |
| `s5c_home` | yes | `confirm` | `POST /cmd` `home` |
| `s5c_calibrate` | no | — | `POST /cmd` `calibrate` |
| `s5c_hold` | no | — | `POST /cmd` `hold` |
| `s5c_fly` | yes | `confirm`, `roll`, `pitch`, `throttle`, `yaw`, `duration_ms` | `POST /control`, then centred sticks |
| `s5c_hovertest` | yes | `confirm` | `POST /cmd` `hovertest` |
| `s5c_wait` | no | `until`, `battery`, `timeout_ms` | `GET /state` until a condition holds |
| `s5c_snapshot` | no | — | `GET /video.mjpg`, one JPEG returned as an image block |
| `s5c_shell` | yes | `confirm`, `command`, `wait_s` | telnet to `S5C_HOST:23` |

`until` is one of `connected`, `disconnected`, `airborne`, `landed`, `homing_done`, `battery_at_least`.

## The state labels

`s5c_status` renames the control server's fields with a `drone.` prefix and adds `read_at_ms`:

| Label | Meaning |
|---|---|
| `drone.x`, `drone.y`, `drone.z` | Fused position in metres from the takeoff point. |
| `drone.heading` | Degrees, from image motion while turning and the stick model in between. |
| `drone.lock` | Camera lock confidence, 0 to 1. |
| `drone.battery` | The drone's own battery field, a loaded voltage, not state of charge. |
| `drone.fly` | The drone's flight state byte. `0` and `1` on the ground, `2` and `3` airborne. |
| `drone.connected` | The control link is logged in. |
| `drone.hold_on`, `drone.holding` | Hover hold is armed, and is holding right now. |
| `drone.homing`, `drone.landing` | Return home or soft land is running. |
| `drone.home_set` | A takeoff point is pinned. |
| `drone.calibrate`, `drone.blocked` | The drone asks for gyro calibration, or refuses to take off. |
| `drone.frame_age`, `drone.fps`, `drone.kbps`, `drone.lag_ms` | Video health. |
| `drone.status` | The control server's last log line, with newlines removed. |
| `drone.build`, `drone.update` | Build stamp and whether a newer build is waiting for the ground. |
| `read_at_ms` | When the state was read, in Unix milliseconds. |

## Safety

- `s5c_takeoff`, `s5c_fly`, `s5c_home`, `s5c_hovertest` and `s5c_shell` refuse to run without `confirm: true`.
- Anything that leaves the ground refuses when the link is down or the battery is under `S5C_MIN_BATTERY`.
- `s5c_fly` clamps every stick to `-1..1` and holds it for at most 4 seconds, then writes centred sticks whether the hold finished or not.
- `s5c_stop` is the emergency action and never asks for confirmation.
- `s5c_home` refuses without a pinned takeoff point.

A toy drone still weighs something and still has spinning blades. Fly it where the law and common sense allow, and keep the remote in reach.
