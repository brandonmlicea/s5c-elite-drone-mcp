# S5C Elite

PC control for the SNAPTAIN S5C Elite drone in Rust, plus an MCP server so Claude can fly the same drone.

## Commands

- `cargo build --release` builds `s5c-elite`, `serve`, `replay`, `fit`, `sim` and `shell`.
- `cargo check --all-targets` is the type gate; it is clean, the remaining warnings are dead code in the tracker.
- `node --check mcp/s5c-mcp.mjs` is the MCP type gate; the server has no dependencies and no build step.
- Never commit `runtime/` or `worker/wrangler.json`: they hold the access key, the tunnel token, calibration, flights and your Cloudflare ids.

## The MCP server

- Tool names are `s5c_` plus lower snake case. Descriptions are one sentence and end with a full stop; the server refuses to start otherwise.
- Every answer is the line `S5C-ELITE <tool> <ok|error>` and then sorted `label=value` lines, nothing else. Labels are `snake_case` parts joined by full stops; values are `true`, `false`, `null`, a bare number or single-line text.
- A failure uses the same shape with `error=<code>` and `detail=<one line>`, and sets `isError`.
- Every argument is checked against the schema before anything reaches the drone: `unknown_argument` and `missing_argument` come first, the tool's own refusal second.
- Anything that leaves the ground needs `confirm: true`, a live link and `S5C_MIN_BATTERY`. `s5c_fly` clamps the sticks to `-1..1`, holds at most 4 s and always writes centred sticks afterwards.
- The documentation of every tool and label is `mcp/README.md`. Change the code and that file together.

## The app

- `wry` and `tao` are the window; `src/page.html` is the UI, embedded with `include_str!`. There is no bundler and no framework.
- `serve` watches its own binary and swaps builds only with the drone on the ground, so `cargo build --release` is the whole edit loop.
- The HTTP surface is `/state`, `/cmd`, `/control`, `/video.mjpg`, `/ws`, `/connect`, `/disconnect`; the MCP server is a client of that surface alone.
- Requests from anywhere but `127.0.0.1` need the key in `runtime/access-key.txt`.
