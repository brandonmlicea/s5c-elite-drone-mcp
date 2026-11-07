# S5C Elite drone MCP

**Give Claude Code MCP access to a SNAPTAIN S5C Elite.** A Rust program on your PC talks to the store-bought drone over a second Wi-Fi adapter, serves live 1080p H.264 video, fused position and stick control on `127.0.0.1:8080`, and ships with an MCP server so Claude can fly it, read its telemetry and look through its camera.

No soldering, no jailbreak, no SDK from the manufacturer: the drone is reached over the protocols it already speaks.

```
you / Claude  ->  MCP server (mcp/s5c-mcp.mjs)  ->  control server 127.0.0.1:8080  ->  drone 172.19.10.1
                                                          |
                                                   live video, pose, battery
```

## What you get

- `S5C Elite.exe`: a desktop window with the live video and the sticks, built on the same engine Tauri uses (`wry` + `tao`, WebView2). Nothing runs at login and closing the window stops everything it started.
- `serve.exe`: the control server. Logs in to the drone by itself, pushes telemetry 25 times a second, streams H.264 to browsers over a WebSocket, and needs an access key from anywhere other than this PC.
- `s5c-elite-mcp`: 14 tools for Claude Code, Claude Desktop or any MCP client, with a strict, documented output format.
- `s5c-elite`: the browser control page, served by the app itself, the same UI the window shows.
- `replay`, `fit`, `sim`, `shell`: rebuild motion from recorded flights, fit the yaw model, fly the tracker in a simulated room, and run commands on the drone's BusyBox shell.

## Hardware

| You need | Why |
|---|---|
| A SNAPTAIN S5C Elite | The firmware this was measured on. Other S5C models speak the same protocols until they do not. |
| A Windows PC | The Wi-Fi keeper uses `wlanapi`; the window uses WebView2. |
| Two Wi-Fi adapters | One stays on your home network, the other joins the drone's hotspot. A USB dongle is enough. |
| Rust 1.75 or newer | `cargo` builds four binaries. |

## Quick start

```
cargo build --release
target\release\s5c-elite.exe --pin
```

Double-click **S5C Elite** on your desktop. Power the drone on. The window finds a network starting with `SNAPTAIN`, saves the hotspot profile, joins it and logs in; the video appears on its own.

Then, in Claude Code, from this folder:

```
node --version          # 18 or newer; the MCP server has no dependencies
claude mcp add s5c-elite -- node mcp/s5c-mcp.mjs
```

This repository also carries a `.mcp.json`, so opening the folder in Claude Code and approving the server is enough. Ask for what you want:

> Check the drone's battery, take off, hover hold, snapshot the camera, then come home.

The first thing an agent should call is `s5c_status`, and the full tool list is in [mcp/README.md](mcp/README.md).

### The MCP format is strict on purpose

Every tool answers in one block and nothing else:

```
S5C-ELITE s5c_status ok
drone.battery=87
drone.connected=true
drone.heading=12.34
```

The first line is the fixed word `S5C-ELITE`, the tool name and `ok` or `error`. Every line after it is one sorted `label=value`, `snake_case` labels, `true`/`false`/number/single-line text, no prose and no blank lines. The server refuses to start if a tool name or a label breaks its pattern, and it refuses arguments a tool does not take. A failure is the same shape with `error=<code>` and `detail=<one line>`. The full contract and every label are in [mcp/README.md](mcp/README.md).

### Safety built into the tools

`s5c_takeoff`, `s5c_fly`, `s5c_home`, `s5c_hovertest` and `s5c_shell` will not run without `confirm: true`. Anything that leaves the ground refuses when the link is down or the battery is under `S5C_MIN_BATTERY` (20 percent by default). `s5c_fly` clamps the sticks to `-1..1`, holds them for at most 4 seconds, and writes centred sticks afterwards. `s5c_stop` is the emergency action and never asks twice.

## Configuration

| Where | What |
|---|---|
| `runtime/home-ssid.txt` or `S5C_HOME_SSID` | Your own Wi-Fi network name. The app puts the home adapter back on it after a flight. Without it the app leaves that adapter alone. |
| `runtime/access-key.txt` | Generated on first run. Requests from anywhere but `127.0.0.1` need `?k=<key>` or the cookie the page sets. |
| `runtime/tunnel-token.txt` | Optional. A Cloudflare tunnel token; when present the app keeps a tunnel running for [the Worker](#remote-control-through-a-cloudflare-tunnel). |
| `S5C_HOST`, `S5C_PORT` | The drone's address on its hotspot, default `172.19.10.1:8866`. |
| `S5C_SERVER`, `S5C_KEY`, `S5C_MIN_BATTERY` | Read by the MCP server; see [mcp/README.md](mcp/README.md). |

`runtime/` is generated, ignored by git, and holds the key, the tunnel token, calibration, logs and recorded flights.

## Pieces

| Binary | What it does |
|---|---|
| `s5c-elite` | The app, `S5C Elite.exe` on the desktop. A window with the live video and controls; keeps the home adapter at home and the second adapter on any `SNAPTAIN*` hotspot it can see, saving new drones itself; runs and restarts the control server and the Cloudflare tunnel. `--pin` copies the exe to the desktop. |
| `serve` | The control server on `127.0.0.1:8080`: login, sticks at 14 Hz, H.264 to browsers over a WebSocket, cached group of pictures so a new viewer gets a picture at once, state 25 times a second. |
| `replay`, `fit` | Rebuild motion and fit the yaw model from the flights in `runtime/`. |
| `sim` | Flies the real tracker and Return Home code in a rendered room. Kept for regression only; real flights are the reference. |
| `shell` | Runs commands on the drone's BusyBox shell over telnet. `s5c_shell` does the same from the MCP server. |
| `worker/` | An optional Cloudflare Worker that reaches this PC through a tunnel, so the drone page works from a phone. |
| `mcp/` | The MCP server, its package and the format contract. |

Hover test: with the drone on the ground and at least 40 percent battery, `POST /cmd {"cmd":"hovertest"}` takes off, hovers for 20 s without touching the sticks, soft-lands, and writes `runtime/hover-test-<time>.csv` with camera lock, frame age, delay, fps, heading and XYZ every 100 ms.

## The drone is an open system

Nothing here treats the drone as a closed box that does what the sticks say. Matter, energy, momentum and information cross its boundary all the time, and the code measures what crosses instead of assuming it.

| Crosses the boundary | In | Out | How the code accounts for it |
|---|---|---|---|
| Energy | Chemical energy from the LiPo cell | Thrust work on the air, heat in the motors, ESCs and cell, radio | Battery level is a loaded voltage, not state of charge: logged flights show it sagging 30 to 50 points in the first seconds of thrust and recovering on the ground. The UI warns at 30 percent, where video and Wi-Fi drop out. |
| Mass and momentum | Still air drawn through the rotors | A downwash jet | Hover is a momentum balance. Translation follows the stick through a first-order lag (`nav.rs`), and nothing assumes the drone stops when the stick does. |
| Angular momentum | Yaw torque from differential rotor speed | Reaction on the air | The yaw response is fitted from real video: rate in image pixels per second is `378·s − 69·s|s|` with a 0.20 s time constant (`fit.rs`, `nav.rs`). |
| Coupled slide | None | Sideways drift while yawing | Turning either way adds the same extra image shift, about 12 percent of the turn. The tracker separates it from true rotation by comparing left and right turns and keeps it in `runtime/calib.txt` between flights. |
| Information | Sticks at 14 Hz over UDP | 1080p H.264 at 25 fps, keyframe every 50 frames | Frames are timed by the drone's own microsecond clock; repeated frames during fast turns are merged. |

Position is therefore a fused estimate: heading from frame-to-frame image motion (`odo.rs`) while the drone turns, the stick model filling the gaps, and the gyro trusted when it does not. XYZ is shown to the millimetre in the top-right corner of the video, 25 times a second.

## Hacking on the window app

The desktop window is deliberately small and is a good place to start if you like vibecoding:

- `src/app.rs` builds it in about 30 lines of `wry` and `tao`, the crate pair Tauri is itself built on, so nearly every Tauri habit applies: a `WebViewBuilder`, a `tao` event loop, a WebView2 profile under `runtime/webview`.
- The UI is one file, `src/page.html`, embedded with `include_str!`. Edit it, run `cargo build --release`, and `serve.exe` restarts into the new build once the drone is on the ground. It is plain HTML, CSS and script, no bundler, no framework.
- The editing loop is fast because the server watches its own binary: `serve` notices a newer `target\release\serve.exe`, refuses to swap mid-flight, and exits into the new build the moment the drone lands.
- Want a different shell? Keep `serve` and throw away the window: the control server is a plain HTTP and WebSocket API (`/state`, `/cmd`, `/control`, `/video.mjpg`, `/ws`), so a Tauri app, an Electron app or a phone page can drive the same drone.
- `sim` runs the whole tracker and Return Home path in a rendered room with no drone at all, which is the cheapest way to try a change.

## Remote control through a Cloudflare tunnel

1. Make a named Cloudflare tunnel and its VPC service, then copy `worker/wrangler.example.json` to `worker/wrangler.json`.
2. Fill in `routes[0].pattern` with your own domain, `vars.ORIGIN` with the tunnel hostname, and `vpc_services[0].service_id` with your tunnel's VPC service id.
3. Put the tunnel token in `runtime/tunnel-token.txt`; the app keeps the tunnel running.
4. `cd worker && npx wrangler deploy`.

`worker/wrangler.json` is ignored by git, so your account id, service id and hostnames stay on your machine.

## Protocol notes

- Control: AES-128-ECB framed requests on TCP 8866 (`link.rs`); video is opened with command 4/1 on its own connection.
- Video container: a 44-byte header (`00 00 01 a0`, or `a1` for a keyframe) with the payload length at byte 16 and the drone timestamp in microseconds at byte 32, followed by Annex B H.264 (Main profile).
- The SDK has a make-keyframe call (command 4/0x1d), but this firmware rejects it with error 21, so fast start comes from the cached group of pictures instead.
- Sticks: a 9-byte UDP packet `66 roll pitch throttle yaw flags xor 99` every 70 ms.
- Shell: telnet on the drone's hotspot address, BusyBox `ash`.

## Before you fly

- Fly where you are allowed to fly. A toy drone still has spinning blades and still weighs something.
- Keep the drone in sight and the remote in reach. The software is a control surface, not a safety system.
- The drone's own battery gauge is a voltage under load; treat 30 percent as empty.

## License

MIT. See [LICENSE](LICENSE).
