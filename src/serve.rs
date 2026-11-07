mod drone;
mod home;
mod link;
mod nav;
mod odo;
mod sim;
mod stream;
mod video;

use drone::{capture_file, runtime_dir, unix_ms, Drone, CALIBRATE, LAND, STOP, TAKEOFF};
use jpeg_encoder::{ColorType, Encoder};
use link::Link;
use std::fs::{File, OpenOptions};
use std::io::{self, Read, Write};
use std::path::PathBuf;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{mpsc, Arc, Mutex, OnceLock};
use std::thread;
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};
use tiny_http::{Header, Response, Server};
use home::{mouse_stick, Hold, Homing, Phase, Sense};
use odo::Img;
use video::{Frame, Framer, HeaderScan, Stream};

struct App {
    drone: Mutex<Option<Drone>>,
    connected: AtomicBool,
    busy: AtomicBool,
    landing: AtomicBool,
    homing: AtomicBool,
    update_ready: AtomicBool,
    hold_on: AtomicBool,
    holding: AtomicBool,
    user: Mutex<[f32; 4]>,
    status: Mutex<String>,
    frame: Mutex<Option<Arc<Frame>>>,
    frame_id: AtomicU64,
    jpeg: Mutex<Arc<Vec<u8>>>,
    jpeg_id: AtomicU64,
    frame_ts: Mutex<Option<Instant>>,
    fps: AtomicU64,
    kbps: AtomicU64,
    lag_ms: AtomicU64,
    build: u64,
    hub: Arc<stream::Hub>,
    key: String,
    auto: AtomicBool,
    jpeg_viewers: AtomicU64,
}

fn log(app: &App, msg: &str) {
    *app.status.lock().unwrap() = msg.to_string();
    let ms = unix_ms();
    if let Ok(mut f) = OpenOptions::new().create(true).append(true).open(runtime_dir().join("serve.log")) {
        let _ = writeln!(f, "[{}.{:03}] {msg}", ms / 1000, ms % 1000);
    }
}

fn src_exe() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("target").join("release").join("serve.exe")
}

fn mtime(p: &PathBuf) -> u64 {
    std::fs::metadata(p)
        .and_then(|m| m.modified())
        .ok()
        .and_then(|t| t.duration_since(UNIX_EPOCH).ok())
        .map(|d| d.as_secs())
        .unwrap_or(0)
}

fn updater(app: Arc<App>) {
    let mut seen = app.build;
    loop {
        thread::sleep(Duration::from_secs(2));
        let now = mtime(&src_exe());
        if now == 0 || now == app.build {
            continue;
        }
        if now != seen {
            seen = now;
            continue;
        }
        let flying = app.drone.lock().unwrap().as_ref().is_some_and(|d| d.telemetry().fly != 0);
        if flying || app.busy.load(Ordering::Relaxed) || app.homing.load(Ordering::Relaxed) || app.landing.load(Ordering::Relaxed) {
            if !app.update_ready.swap(true, Ordering::Relaxed) {
                log(&app, "new build ready - it loads once the drone is on the ground (never mid-flight)");
            }
            continue;
        }
        if let Some(d) = app.drone.lock().unwrap().take() {
            d.close();
        }
        log(&app, "new build detected - restarting into it");
        std::process::exit(0);
    }
}

fn telnet_reboot(app: &App) -> bool {
    use std::net::TcpStream;
    let Ok(addr) = "172.19.10.1:23".parse() else { return false };
    let Ok(mut s) = TcpStream::connect_timeout(&addr, Duration::from_secs(2)) else {
        log(app, "telnet shell (port 23) unreachable - the whole CPU is down; physical power cycle needed");
        return false;
    };
    let _ = s.set_read_timeout(Some(Duration::from_secs(2)));
    let mut buf = [0u8; 1024];
    for _ in 0..3 {
        let Ok(n) = s.read(&mut buf) else { break };
        if n == 0 {
            break;
        }
        let mut reply = Vec::new();
        let mut i = 0;
        while i + 2 < n {
            if buf[i] == 255 {
                let (cmd, opt) = (buf[i + 1], buf[i + 2]);
                match cmd {
                    251 | 252 => reply.extend_from_slice(&[255, 254, opt]),
                    253 | 254 => reply.extend_from_slice(&[255, 252, opt]),
                    _ => {}
                }
                i += 3;
            } else {
                i += 1;
            }
        }
        if !reply.is_empty() {
            let _ = s.write_all(&reply);
        }
    }
    let _ = s.write_all(b"reboot\n");
    thread::sleep(Duration::from_millis(500));
    log(app, "sent reboot over the drone's telnet shell - programmatic power cycle triggered");
    true
}

fn wait_online() -> bool {
    for _ in 0..15 {
        thread::sleep(Duration::from_secs(1));
        if drone_subnet_ip().is_some() && port_open("172.19.10.1", 8866, 1200) {
            return true;
        }
    }
    false
}

fn recover_hung_drone(app: &App) -> bool {
    if !telnet_reboot(app) {
        return false;
    }
    let mut delay = 4u64;
    for attempt in 1..=6 {
        log(app, &format!("backoff: waiting {delay}s for the drone to finish rebooting (attempt {attempt}/6)"));
        thread::sleep(Duration::from_secs(delay));
        if wait_online() {
            log(app, "*** ALERT: drone rebooted and is BACK ONLINE ***");
            return true;
        }
        delay = (delay * 2).min(40);
    }
    log(app, "drone did not come back after reboot + backoff - physical power cycle needed");
    false
}

fn port_open(ip: &str, port: u16, ms: u64) -> bool {
    probe(ip, port, ms).0
}

fn probe(ip: &str, port: u16, ms: u64) -> (bool, String) {
    use std::net::ToSocketAddrs;
    let Ok(mut addrs) = format!("{ip}:{port}").to_socket_addrs() else {
        return (false, "bad addr".into());
    };
    let Some(addr) = addrs.next() else { return (false, "no addr".into()) };
    match std::net::TcpStream::connect_timeout(&addr, Duration::from_millis(ms)) {
        Ok(_) => (true, "open".into()),
        Err(e) => (false, format!("{:?}", e.kind())),
    }
}

fn drone_subnet_ip() -> Option<String> {
    let s = std::net::UdpSocket::bind(("0.0.0.0", 0)).ok()?;
    s.connect("172.19.10.1:8866").ok()?;
    let ip = s.local_addr().ok()?.ip().to_string();
    if ip.starts_with("172.19.") {
        Some(ip)
    } else {
        None
    }
}

fn restart_session(app: &App) -> bool {
    if let Some(d) = app.drone.lock().unwrap().take() {
        d.close();
    }
    thread::sleep(Duration::from_millis(1500));
    for attempt in 1..=3 {
        match Drone::connect() {
            Ok(d) => {
                *app.drone.lock().unwrap() = Some(d);
                log(app, "video: drone session restarted - video is coming back");
                return true;
            }
            Err(e) => {
                log(app, &format!("video: session restart attempt {attempt} failed ({e}) - retrying in 5s"));
                thread::sleep(Duration::from_secs(5));
            }
        }
    }
    app.connected.store(false, Ordering::Relaxed);
    log(app, "video: could not restart the drone session - press Connect again");
    false
}

fn video_supervisor(app: Arc<App>) {
    let (tx, rx) = mpsc::channel::<Vec<u8>>();
    let a = app.clone();
    thread::spawn(move || decode_loop(a, rx));
    let stamp = SystemTime::now().duration_since(UNIX_EPOCH).map(|d| d.as_secs()).unwrap_or(0);
    let mut raw = File::create(runtime_dir().join(format!("video-raw-{stamp}.h264"))).ok();
    let (mut written, mut session, mut stuck) = (0u64, 0u32, 0u32);
    while app.connected.load(Ordering::Relaxed) {
        let Some(kind) = app.drone.lock().unwrap().as_ref().map(|d| d.kind()) else { break };
        session += 1;
        log(&app, &format!("video: opening stream #{session}..."));
        match Drone::open_video(kind) {
            Ok(mut v) => {
                let _ = tx.send(Vec::new());
                let (why, bytes) = pump(&app, &mut v, &tx, &mut raw, &mut written);
                log(&app, &format!("video: stream #{session} ended after {} KB: {why}", bytes / 1024));
                stuck = if why.contains("EOF") || bytes == 0 { stuck + 1 } else { 0 };
            }
            Err(e) => {
                log(&app, &format!("video: open #{session} failed: {e}"));
                stuck += 1;
            }
        }
        if !app.connected.load(Ordering::Relaxed) {
            break;
        }
        if stuck >= 2 {
            let flying = app.drone.lock().unwrap().as_ref().is_some_and(|d| d.telemetry().fly != 0);
            if !flying && !app.homing.load(Ordering::SeqCst) && !app.landing.load(Ordering::SeqCst) {
                log(&app, "video: the drone stopped serving video in this session - restarting the session (on the ground, safe)");
                if !restart_session(&app) {
                    break;
                }
                stuck = 0;
                continue;
            }
            log(&app, "video: lost in flight - the drone only gives video back after a session restart; land (L) and it comes back by itself");
        }
        thread::sleep(Duration::from_secs(1));
    }
    app.kbps.store(0, Ordering::Relaxed);
    app.fps.store(0, Ordering::Relaxed);
}

fn pump(app: &App, v: &mut Link, tx: &mpsc::Sender<Vec<u8>>, raw: &mut Option<File>, written: &mut u64) -> (String, u64) {
    let mut tmp = vec![0u8; 1 << 16];
    let (mut total, mut idle, mut win, mut tick, opened) = (0u64, 0u32, 0u64, Instant::now(), Instant::now());
    let (mut scan, mut best, mut stalled, mut last_ts) = (HeaderScan::new(), i64::MAX, false, 0u32);
    let mut timing = capture_file("video-timing");
    while app.connected.load(Ordering::Relaxed) {
        match v.recv_bytes(&mut tmp) {
            Ok(0) => return ("drone closed the stream (EOF)".into(), total),
            Ok(n) => {
                idle = 0;
                total += n as u64;
                win += n as u64;
                if let Some(f) = raw.as_mut().filter(|_| *written < 300 << 20) {
                    let _ = f.write_all(&tmp[..n]);
                    *written += n as u64;
                }
                if tx.send(tmp[..n].to_vec()).is_err() {
                    return ("decoder stopped".into(), total);
                }
                let now = unix_ms() as i64;
                for s in scan.push(&tmp[..n]) {
                    if s.ts_us < last_ts {
                        best = i64::MAX;
                    }
                    last_ts = s.ts_us;
                    let offset = now - s.ts_us as i64 / 1000;
                    best = best.min(offset);
                    let late = offset - best;
                    app.lag_ms.store(late as u64, Ordering::Relaxed);
                    if let Some(f) = timing.as_mut() {
                        let _ = writeln!(f, "{{\"ms\":{now},\"seq\":{},\"drone_us\":{},\"key\":{},\"late_ms\":{late}}}", s.seq, s.ts_us, s.key);
                    }
                    if late > 400 && !stalled {
                        log(app, &format!("video: drone delivery stalled - frame {} arrived {late} ms late (Wi-Fi/TCP retransmit), catching up", s.seq));
                    }
                    stalled = late > 400;
                }
            }
            Err(e) if matches!(e.kind(), io::ErrorKind::WouldBlock | io::ErrorKind::TimedOut) => {
                idle += 1;
                app.kbps.store(0, Ordering::Relaxed);
                if idle >= 2 {
                    return (format!("stalled - no bytes for {}s", idle * 3), total);
                }
            }
            Err(e) => return (format!("socket error {:?}: {e}", e.kind()), total),
        }
        let secs = tick.elapsed().as_secs_f32();
        if secs >= 1.0 {
            app.kbps.store((win as f32 * 8.0 / 1000.0 / secs) as u64, Ordering::Relaxed);
            (win, tick) = (0, Instant::now());
            let last = (*app.frame_ts.lock().unwrap()).map_or(opened.elapsed(), |t| t.elapsed().min(opened.elapsed()));
            if total > 0 && last > Duration::from_secs(4) {
                return (format!("{} KB arrived but nothing decoded for 4s - reopening for a fresh keyframe", total / 1024), total);
            }
        }
    }
    ("disconnected".into(), total)
}

fn decode_loop(app: Arc<App>, rx: mpsc::Receiver<Vec<u8>>) {
    let mut stream = None::<Stream>;
    let (mut first, mut errors, mut count, mut tick) = (true, 0u64, 0u64, Instant::now());
    let (mut framer, mut offset, mut prev_ts) = (Framer::new(), f64::MAX, None::<u32>);
    for chunk in rx {
        if chunk.is_empty() {
            stream = Stream::new().map_err(|e| log(&app, &format!("video: decoder init failed: {e}"))).ok();
            (first, framer, offset, prev_ts) = (true, Framer::new(), f64::MAX, None);
            continue;
        }
        let Some(s) = stream.as_mut() else { continue };
        let now = unix_ms() as f64;
        for (bytes, ts) in framer.push(&chunk) {
            app.hub.video(&bytes, ts);
            if prev_ts.is_some_and(|p| ts < p || ts - p > 5_000_000) {
                offset = f64::MAX;
            }
            offset = offset.min(now - ts as f64 / 1000.0);
            let shot = prev_ts.map(|p| p as f64 / 1000.0 + offset);
            prev_ts = Some(ts);
            let (frames, errs) = s.push(&bytes);
            for e in errs {
                errors += 1;
                if errors % 100 == 1 {
                    log(&app, &format!("video: decode error #{errors} ({e}) - holding last good picture until the next keyframe"));
                }
            }
            for f in frames {
                if first {
                    log(&app, &format!("video: LIVE - first {}x{} frame decoded", f.w, f.h));
                    first = false;
                }
                if let Some(t) = shot {
                    let img = Img::from_frame(&f);
                    let nav = app.drone.lock().unwrap().as_ref().map(|d| d.nav.clone());
                    if let Some(nav) = nav {
                        nav.lock().unwrap().frame(img, t);
                    }
                }
                *app.frame.lock().unwrap() = Some(Arc::new(f));
                *app.frame_ts.lock().unwrap() = Some(Instant::now());
                app.frame_id.fetch_add(1, Ordering::Relaxed);
                count += 1;
            }
        }
        let secs = tick.elapsed().as_secs_f32();
        if secs >= 1.0 {
            app.fps.store((count as f32 / secs).round() as u64, Ordering::Relaxed);
            (count, tick) = (0, Instant::now());
        }
    }
}

fn encoder(app: Arc<App>) {
    let mut last = 0;
    loop {
        let id = app.frame_id.load(Ordering::Relaxed);
        if id == last || app.jpeg_viewers.load(Ordering::Relaxed) == 0 {
            thread::sleep(Duration::from_millis(3));
            continue;
        }
        last = id;
        let Some(f) = app.frame.lock().unwrap().clone() else { continue };
        *app.jpeg.lock().unwrap() = Arc::new(f.jpeg(80));
        app.jpeg_id.fetch_add(1, Ordering::Relaxed);
    }
}

fn go_home(app: Arc<App>) {
    if app.homing.swap(true, Ordering::SeqCst) {
        return;
    }
    app.landing.store(false, Ordering::SeqCst);
    let flying = app.drone.lock().unwrap().as_ref().is_some_and(|d| d.telemetry().fly == 2);
    if !flying {
        app.homing.store(false, Ordering::SeqCst);
        log(&app, "return home: not flying - nothing to do");
        return;
    }
    let mut h = Homing::new(unix_ms() as f64);
    let (mut seen, mut view, mut said) = (f64::NAN, None, String::new());
    loop {
        if !app.homing.load(Ordering::SeqCst) {
            return;
        }
        let Some(nav) = app.drone.lock().unwrap().as_ref().filter(|d| d.telemetry().fly != 0).map(|d| d.nav.clone()) else { break };
        let (pose, pair) = {
            let n = nav.lock().unwrap();
            let fresh = n.latest.as_ref().map(|(_, t)| *t).filter(|t| *t != seen);
            (n.pose(), fresh.and_then(|t| Some((n.home.clone()?, n.latest.clone()?.0, t))))
        };
        if let Some((home, cur, t)) = pair {
            seen = t;
            view = Some((odo::compare(&home, &cur), t));
        }
        let sticks = h.tick(&Sense { now: unix_ms() as f64, pose, view });
        let status = format!("return home: {}", h.status());
        if status != said {
            log(&app, &status);
            said = status;
        }
        if let Some(d) = app.drone.lock().unwrap().as_ref() {
            d.set(sticks[0] as f32, sticks[1] as f32, sticks[2] as f32, sticks[3] as f32);
        }
        match &h.phase {
            Phase::Land => {
                nav.lock().unwrap().at_home();
                app.homing.store(false, Ordering::SeqCst);
                soft_land(app);
                return;
            }
            Phase::Fail(_) => break,
            _ => thread::sleep(Duration::from_millis(70)),
        }
    }
    if let Some(d) = app.drone.lock().unwrap().as_ref() {
        d.set(0.0, 0.0, 0.0, 0.0);
    }
    app.homing.store(false, Ordering::SeqCst);
}

fn assist(app: Arc<App>) {
    let mut hold = Hold::new();
    loop {
        thread::sleep(Duration::from_millis(70));
        let Some((nav, sticks, fly)) = app.drone.lock().unwrap().as_ref().map(|d| (d.nav.clone(), d.sticks.clone(), d.telemetry().fly)) else {
            continue;
        };
        let user = *app.user.lock().unwrap();
        let neutral = app.hold_on.load(Ordering::Relaxed)
            && fly == 2
            && !app.homing.load(Ordering::Relaxed)
            && !app.landing.load(Ordering::Relaxed)
            && user.iter().all(|v| *v == 0.0);
        let latest = nav.lock().unwrap().latest.clone();
        let [pitch, yaw] = hold.tick(unix_ms() as f64, neutral, latest.as_ref().map(|(i, t)| (i, *t)));
        app.holding.store(hold.active, Ordering::Relaxed);
        let mut s = sticks.lock().unwrap();
        (s.hold_pitch, s.hold_yaw) = (pitch as f32, yaw as f32);
    }
}

fn soft_land(app: Arc<App>) {
    if app.landing.swap(true, Ordering::SeqCst) {
        return;
    }
    log(&app, "soft land: descending, flaring to a slow final approach near the floor");
    let (start, mut floor) = (Instant::now(), None::<Instant>);
    let reason = loop {
        if !app.landing.load(Ordering::SeqCst) {
            log(&app, "soft land: cancelled by pilot");
            return;
        }
        let el = start.elapsed().as_secs_f32();
        let (fly, z) = {
            let g = app.drone.lock().unwrap();
            let Some(d) = g.as_ref() else {
                app.landing.store(false, Ordering::SeqCst);
                return;
            };
            let z = d.nav.lock().unwrap().z;
            d.sticks.lock().unwrap().throttle = if z > 0.35 { -40.0 } else { -25.0 } * (el / 0.6).min(1.0);
            (d.telemetry().fly, z)
        };
        if fly == 0 {
            break format!("motors stopped after {el:.1}s");
        }
        if z <= 0.0 && floor.get_or_insert_with(Instant::now).elapsed() > Duration::from_millis(1500) {
            break format!("estimated on the floor after {el:.1}s");
        }
        if el > 10.0 {
            break "10s cap".into();
        }
        thread::sleep(Duration::from_millis(50));
    };
    let end = Instant::now() + Duration::from_secs(3);
    while Instant::now() < end {
        let g = app.drone.lock().unwrap();
        let Some(d) = g.as_ref().filter(|d| d.telemetry().fly != 0) else { break };
        d.set(0.0, 0.0, -100.0, 0.0);
        d.command(LAND);
        drop(g);
        thread::sleep(Duration::from_millis(100));
    }
    if let Some(d) = app.drone.lock().unwrap().as_ref() {
        d.set(0.0, 0.0, 0.0, 0.0);
    }
    log(&app, &format!("soft land: {reason}; throttle down + LAND to stop the motors"));
    app.landing.store(false, Ordering::SeqCst);
}
fn connect(app: Arc<App>) {
    if app.busy.swap(true, Ordering::SeqCst) || app.connected.load(Ordering::Relaxed) {
        app.busy.store(false, Ordering::SeqCst);
        return;
    }
    let host = std::env::var("S5C_HOST").unwrap_or_else(|_| "172.19.10.1".into());
    log(&app, &format!("connecting to the drone at {host} over the Nighthawk adapter - your home Wi-Fi is never touched"));
    for i in 0..15 {
        if port_open(&host, 8866, 800) {
            return login(app);
        }
        if i == 0 && drone_subnet_ip().is_none() {
            log(&app, "the Nighthawk is not on the drone hotspot yet - waiting (Windows joins it automatically once the drone is powered on)");
        }
        thread::sleep(Duration::from_secs(1));
    }
    if drone_subnet_ip().is_some() {
        log(&app, "on the drone hotspot but its control port is not answering - attempting automatic recovery (telnet reboot)...");
        if recover_hung_drone(&app) {
            return login(app);
        }
    }
    log(&app, "drone not reachable - check it is powered on and close to the PC, then press Connect again");
    app.busy.store(false, Ordering::SeqCst);
}

fn login(app: Arc<App>) {
    log(&app, "logging in...");
    let mut result = Drone::connect();
    for attempt in 2..=10 {
        let Err(ref e) = result else { break };
        if attempt == 2 {
            log(&app, &format!("login refused ({e}) - the drone is still holding the previous session; retrying every second"));
        }
        thread::sleep(Duration::from_secs(1));
        result = Drone::connect();
    }
    match result {
        Ok(d) => {
            *app.drone.lock().unwrap() = Some(d);
            app.connected.store(true, Ordering::Relaxed);
            log(&app, "connected - live");
            let a = app.clone();
            thread::spawn(move || video_supervisor(a));
        }
        Err(e) => log(&app, &format!("login failed after retry: {e}")),
    }
    app.busy.store(false, Ordering::SeqCst);
}

fn disconnect(app: Arc<App>) {
    if app.busy.swap(true, Ordering::SeqCst) {
        return;
    }
    app.connected.store(false, Ordering::Relaxed);
    app.landing.store(false, Ordering::SeqCst);
    app.homing.store(false, Ordering::SeqCst);
    if let Some(d) = app.drone.lock().unwrap().take() {
        d.close();
    }
    *app.frame.lock().unwrap() = None;
    *app.jpeg.lock().unwrap() = Arc::new(Vec::new());
    app.jpeg_id.fetch_add(1, Ordering::Relaxed);
    *app.frame_ts.lock().unwrap() = None;
    log(&app, "idle");
    app.busy.store(false, Ordering::SeqCst);
}

fn num(body: &str, key: &str) -> f32 {
    let pat = format!("\"{key}\"");
    let Some(i) = body.find(&pat) else { return 0.0 };
    let rest = &body[i + pat.len()..];
    let Some(c) = rest.find(':') else { return 0.0 };
    let s: String = rest[c + 1..].chars().skip_while(|c| c.is_whitespace())
        .take_while(|c| c.is_ascii_digit() || *c == '-' || *c == '.' || *c == 'e').collect();
    s.parse().unwrap_or(0.0)
}

fn field<'a>(body: &'a str, key: &str) -> Option<&'a str> {
    let pat = format!("\"{key}\"");
    let i = body.find(&pat)? + pat.len();
    let rest = &body[i..];
    let c = rest.find(':')? + 1;
    let after = rest[c..].trim_start();
    if after.starts_with('"') {
        let a = &after[1..];
        a.find('"').map(|e| &a[..e])
    } else {
        Some(after.split(|c: char| c == ',' || c == '}').next()?.trim())
    }
}

const PAGE: &str = include_str!("page.html");

fn main() {
    let app = Arc::new(App {
        drone: Mutex::new(None),
        connected: AtomicBool::new(false),
        busy: AtomicBool::new(false),
        landing: AtomicBool::new(false),
        homing: AtomicBool::new(false),
        update_ready: AtomicBool::new(false),
        hold_on: AtomicBool::new(true),
        holding: AtomicBool::new(false),
        user: Mutex::new([0.0; 4]),
        status: Mutex::new("idle".into()),
        frame: Mutex::new(None),
        frame_id: AtomicU64::new(0),
        jpeg: Mutex::new(Arc::new(Vec::new())),
        jpeg_id: AtomicU64::new(0),
        frame_ts: Mutex::new(None),
        fps: AtomicU64::new(0),
        kbps: AtomicU64::new(0),
        lag_ms: AtomicU64::new(0),
        build: mtime(&src_exe()),
        hub: Arc::new(stream::Hub::new()),
        key: access_key(),
        auto: AtomicBool::new(true),
        jpeg_viewers: AtomicU64::new(0),
    });
    let http = format!("127.0.0.1:{}", std::env::var("S5C_HTTP").unwrap_or_else(|_| "8080".into()));
    let server = loop {
        match Server::http(&http) {
            Ok(s) => break Arc::new(s),
            Err(e) => {
                let _ = e;
                thread::sleep(Duration::from_secs(1));
            }
        }
    };
    log(&app, &format!("serving on http://{http} (build {})", app.build));
    let a = app.clone();
    thread::spawn(move || encoder(a));
    let a = app.clone();
    thread::spawn(move || updater(a));
    let a = app.clone();
    thread::spawn(move || assist(a));
    let a = app.clone();
    thread::spawn(move || pusher(a));
    let a = app.clone();
    thread::spawn(move || autoconnect(a));
    let mut workers = Vec::new();
    for _ in 0..6 {
        let (app, server) = (app.clone(), server.clone());
        workers.push(thread::spawn(move || worker(app, server)));
    }
    for w in workers {
        let _ = w.join();
    }
}

fn json_hdr() -> Header {
    Header::from_bytes(&b"Content-Type"[..], &b"application/json"[..]).unwrap()
}

fn worker(app: Arc<App>, server: Arc<Server>) {
    loop {
        let Ok(mut req) = server.recv() else { break };
        let (method, url) = (req.method().to_string(), req.url().to_string());
        let Some(cookie) = allowed(&app, &req) else {
            let _ = req.respond(Response::from_string("This drone link needs its key - open it from the S5C Elite shortcut on the desktop.").with_status_code(403));
            continue;
        };
        let url = url.split('?').next().unwrap_or("").to_string();
        if method == "GET" && url == "/" {
            let mut r = Response::from_string(PAGE)
                .with_header(Header::from_bytes(&b"Content-Type"[..], &b"text/html; charset=utf-8"[..]).unwrap())
                .with_header(Header::from_bytes(&b"Cache-Control"[..], &b"no-store"[..]).unwrap());
            if cookie {
                r.add_header(Header::from_bytes(&b"Set-Cookie"[..], format!("k={}; Path=/; Max-Age=31536000; HttpOnly; Secure; SameSite=Strict", app.key).as_bytes()).unwrap());
            }
            let _ = req.respond(r);
        } else if method == "GET" && url.starts_with("/video.mjpg") {
            app.jpeg_viewers.fetch_add(1, Ordering::Relaxed);
            let reader = MjpegReader { app: app.clone(), buf: Vec::new(), pos: 0, last: u64::MAX };
            let hdr = Header::from_bytes(&b"Content-Type"[..],
                &b"multipart/x-mixed-replace; boundary=f"[..]).unwrap();
            thread::spawn(move || {
                let _ = req.respond(Response::new(tiny_http::StatusCode(200), vec![hdr], reader, None, None));
            });
        } else if method == "GET" && url == "/state" {
            let _ = req.respond(Response::from_string(state_json(&app)).with_header(json_hdr()));
        } else if method == "GET" && url.starts_with("/ws") {
            let a = app.clone();
            thread::spawn(move || stream::serve(a.hub.clone(), req));
        } else if method == "POST" && url == "/control" {
            let mut body = String::new();
            let _ = req.as_reader().read_to_string(&mut body);
            let (r, p, t, mut y) = (num(&body, "roll"), num(&body, "pitch"), num(&body, "throttle"), num(&body, "yaw"));
            let turn = num(&body, "turn");
            if turn != 0.0 && y == 0.0 {
                let gain = app.drone.lock().unwrap().as_ref().map_or(1.0, |d| d.nav.lock().unwrap().yaw_gain);
                y = mouse_stick(turn as f64, gain) as f32;
            }
            *app.user.lock().unwrap() = [r, p, t, y];
            let climb_cancels = t > 0.0 && app.landing.swap(false, Ordering::SeqCst);
            let input = r != 0.0 || p != 0.0 || t != 0.0 || y != 0.0;
            if input && app.homing.swap(false, Ordering::SeqCst) {
                log(&app, "return home cancelled - pilot took the sticks");
            }
            if let Some(d) = app.drone.lock().unwrap().as_ref() {
                if app.homing.load(Ordering::SeqCst) {
                } else if app.landing.load(Ordering::SeqCst) {
                    let mut s = d.sticks.lock().unwrap();
                    (s.roll, s.pitch, s.yaw) = (r, p, y);
                } else {
                    d.set(r, p, t, y);
                }
            }
            if climb_cancels {
                log(&app, "soft land cancelled - climb input");
            }
            let _ = req.respond(Response::empty(204));
        } else if method == "POST" && url == "/cmd" {
            let mut body = String::new();
            let _ = req.as_reader().read_to_string(&mut body);
            let cmd = field(&body, "cmd").unwrap_or("").to_string();
            let fly = app.drone.lock().unwrap().as_ref().map(|d| d.telemetry().fly);
            log(&app, &format!("cmd: {cmd} (fly state {fly:?})"));
            if cmd != "land" {
                app.landing.store(false, Ordering::SeqCst);
            }
            if cmd != "home" {
                app.homing.store(false, Ordering::SeqCst);
            }
            if let Some(d) = app.drone.lock().unwrap().as_ref() {
                match cmd.as_str() {
                    "takeoff" => d.command(TAKEOFF),
                    "home" => {
                        let a = app.clone();
                        thread::spawn(move || go_home(a));
                    }
                    "land" => {
                        let a = app.clone();
                        thread::spawn(move || soft_land(a));
                    }
                    "stop" => {
                        d.set(0.0, 0.0, 0.0, 0.0);
                        d.command(STOP);
                    }
                    "calibrate" => d.command(CALIBRATE),
                    "hovertest" => {
                        let a = app.clone();
                        thread::spawn(move || hover_test(a, 20.0));
                    }
                    "hold" => {
                        let on = !app.hold_on.load(Ordering::Relaxed);
                        app.hold_on.store(on, Ordering::Relaxed);
                        log(&app, if on { "hover hold ON - the drone holds its view when you let go" } else { "hover hold OFF" });
                    }
                    _ => {}
                }
            }
            let _ = req.respond(Response::empty(204));
        } else if method == "POST" && url == "/connect" {
            app.auto.store(true, Ordering::Relaxed);
            let a = app.clone();
            thread::spawn(move || connect(a));
            let _ = req.respond(Response::empty(204));
        } else if method == "POST" && url == "/disconnect" {
            app.auto.store(false, Ordering::Relaxed);
            let a = app.clone();
            thread::spawn(move || disconnect(a));
            let _ = req.respond(Response::empty(204));
        } else {
            let _ = req.respond(Response::empty(404));
        }
    }
}

struct MjpegReader {
    app: Arc<App>,
    buf: Vec<u8>,
    pos: usize,
    last: u64,
}

impl Read for MjpegReader {
    fn read(&mut self, out: &mut [u8]) -> io::Result<usize> {
        if self.pos >= self.buf.len() {
            let until = Instant::now() + Duration::from_secs(1);
            while self.app.jpeg_id.load(Ordering::Relaxed) == self.last && Instant::now() < until {
                thread::sleep(Duration::from_millis(4));
            }
            self.last = self.app.jpeg_id.load(Ordering::Relaxed);
            let j = self.app.jpeg.lock().unwrap().clone();
            let frame: &[u8] = if j.is_empty() { placeholder() } else { &j };
            self.buf.clear();
            self.buf.extend_from_slice(
                format!("--f\r\nContent-Type: image/jpeg\r\nContent-Length: {}\r\n\r\n", frame.len()).as_bytes());
            self.buf.extend_from_slice(frame);
            self.buf.extend_from_slice(b"\r\n");
            self.pos = 0;
        }
        let n = out.len().min(self.buf.len() - self.pos);
        out[..n].copy_from_slice(&self.buf[self.pos..self.pos + n]);
        self.pos += n;
        Ok(n)
    }
}

fn placeholder() -> &'static [u8] {
    static P: OnceLock<Vec<u8>> = OnceLock::new();
    P.get_or_init(|| {
        let (w, h) = (640u16, 360u16);
        let mut out = Vec::new();
        let _ = Encoder::new(&mut out, 60).encode(&vec![18u8; w as usize * h as usize * 3], w, h, ColorType::Rgb);
        out
    })
}

fn access_key() -> String {
    let path = runtime_dir().join("access-key.txt");
    let saved = std::fs::read_to_string(&path).unwrap_or_default().trim().to_string();
    if saved.len() >= 32 {
        return saved;
    }
    let key = stream::secret();
    let _ = std::fs::write(&path, &key);
    key
}

fn allowed(app: &App, req: &tiny_http::Request) -> Option<bool> {
    let header = |name: &'static str| req.headers().iter().find(|h| h.field.equiv(name)).map(|h| h.value.to_string());
    let host = header("Host").unwrap_or_default();
    let local = (host.starts_with("127.0.0.1") || host.starts_with("localhost")) && header("Cf-Connecting-Ip").is_none();
    if local {
        return Some(false);
    }
    let query = req.url().split_once('?').map_or("", |(_, q)| q);
    if query.split('&').any(|p| p == format!("k={}", app.key)) {
        return Some(true);
    }
    let cookie = header("Cookie").unwrap_or_default();
    cookie.split(';').any(|c| c.trim() == format!("k={}", app.key)).then_some(false)
}

fn state_json(app: &App) -> String {
    let (t, n) = app.drone.lock().unwrap().as_ref().map(|d| (d.telemetry(), d.nav.lock().unwrap().pose())).unwrap_or_default();
    let age = app.frame_ts.lock().unwrap().map(|i| i.elapsed().as_secs_f32());
    format!(
        "{{\"x\":{:.4},\"y\":{:.4},\"z\":{:.4},\"heading\":{:.2},\"lock\":{:.2},\"home_set\":{},\"holding\":{},\"hold_on\":{},\"homing\":{},\"connected\":{},\"status\":\"{}\",\"battery\":{},\"fly\":{},\"frame_age\":{},\"calibrate\":{},\"blocked\":{},\"fps\":{},\"kbps\":{},\"lag_ms\":{},\"landing\":{},\"build\":{},\"update\":{},\"ms\":{}}}",
        n.x, n.y, n.z, n.heading, n.lock, n.home_set, app.holding.load(Ordering::Relaxed), app.hold_on.load(Ordering::Relaxed),
        app.homing.load(Ordering::Relaxed),
        app.connected.load(Ordering::Relaxed),
        app.status.lock().unwrap().replace('"', "'"),
        t.battery, t.fly,
        age.map(|a| format!("{a:.2}")).unwrap_or("null".into()),
        t.needs_calibration() as u8, t.cannot_take_off() as u8,
        app.fps.load(Ordering::Relaxed), app.kbps.load(Ordering::Relaxed), app.lag_ms.load(Ordering::Relaxed),
        app.landing.load(Ordering::Relaxed),
        app.build, app.update_ready.load(Ordering::Relaxed), unix_ms())
}

fn pusher(app: Arc<App>) {
    loop {
        thread::sleep(Duration::from_millis(40));
        if app.hub.count() > 0 {
            app.hub.text(&state_json(&app));
        }
    }
}

fn autoconnect(app: Arc<App>) {
    loop {
        thread::sleep(Duration::from_secs(2));
        if app.auto.load(Ordering::Relaxed) && !app.connected.load(Ordering::Relaxed) && !app.busy.load(Ordering::Relaxed) && port_open("172.19.10.1", 8866, 600) {
            connect(app.clone());
        }
    }
}

impl Drop for MjpegReader {
    fn drop(&mut self) {
        self.app.jpeg_viewers.fetch_sub(1, Ordering::Relaxed);
    }
}

fn hover_test(app: Arc<App>, secs: f32) {
    let ready = app.drone.lock().unwrap().as_ref().map(|d| d.telemetry());
    let Some(t) = ready.filter(|t| t.fly == 0 && !app.homing.load(Ordering::SeqCst) && !app.landing.load(Ordering::SeqCst)) else {
        log(&app, "hover test: needs the drone connected and on the ground");
        return;
    };
    if t.battery < 40 {
        log(&app, &format!("hover test: battery {}% - charge it first (video cuts out below about 35%)", t.battery));
        return;
    }
    log(&app, &format!("hover test: takeoff, hover {secs:.0}s without touching the sticks, land (battery {}%)", t.battery));
    if let Some(d) = app.drone.lock().unwrap().as_ref() {
        d.command(TAKEOFF);
    }
    let start = Instant::now();
    let mut samples: Vec<[f64; 9]> = Vec::new();
    let mut airborne = None::<Instant>;
    let reason = loop {
        thread::sleep(Duration::from_millis(100));
        let Some((t, p)) = app.drone.lock().unwrap().as_ref().map(|d| (d.telemetry(), d.nav.lock().unwrap().pose())) else { break "disconnected" };
        if *app.user.lock().unwrap() != [0.0; 4] || app.homing.load(Ordering::SeqCst) || app.landing.load(Ordering::SeqCst) {
            break "pilot took over";
        }
        let age = app.frame_ts.lock().unwrap().map_or(99.0, |i| i.elapsed().as_secs_f64());
        let el = start.elapsed().as_secs_f64();
        samples.push([el, t.fly as f64, t.battery as f64, p.lock, age, app.lag_ms.load(Ordering::Relaxed) as f64, app.fps.load(Ordering::Relaxed) as f64, p.heading, p.x.hypot(p.y)]);
        if t.fly == 2 && airborne.is_none() {
            airborne = Some(Instant::now());
        }
        if airborne.is_some_and(|a| a.elapsed().as_secs_f32() >= secs) {
            break "hover time reached";
        }
        if el > secs as f64 + 15.0 {
            break "never reached hover";
        }
    };
    log(&app, &format!("hover test: {reason} - landing"));
    soft_land(app.clone());
    let air: Vec<&[f64; 9]> = samples.iter().filter(|s| s[1] == 2.0).collect();
    let worst = |k: usize| air.iter().map(|s| s[k]).fold(0.0f64, f64::max);
    let least = |k: usize| air.iter().map(|s| s[k]).fold(f64::MAX, f64::min);
    let lost = air.iter().filter(|s| s[3] < 0.5).count() as f64 * 0.1;
    let frozen = air.iter().filter(|s| s[4] > 0.5).count() as f64 * 0.1;
    let headings: Vec<f64> = air.iter().map(|s| s[7]).collect();
    let drift = headings.iter().map(|h| ((h - headings.first().copied().unwrap_or(0.0) + 180.0).rem_euclid(360.0) - 180.0).abs()).fold(0.0f64, f64::max);
    let summary = format!(
        "hover test result: {:.1}s in the air ({reason}); camera lock lost {lost:.1}s, video frozen >0.5s for {frozen:.1}s, worst frame gap {:.2}s, worst drone-to-PC delay {:.0} ms, lowest fps {:.0}; heading moved {drift:.2} deg; XYZ moved {:.3} m; battery {:.0}% -> {:.0}%",
        air.len() as f64 * 0.1, worst(4), worst(5), least(6), worst(8),
        samples.first().map_or(0.0, |s| s[2]), samples.last().map_or(0.0, |s| s[2])
    );
    log(&app, &summary);
    let rows: String = samples.iter().map(|s| format!("{}\n", s.iter().map(|v| format!("{v:.3}")).collect::<Vec<_>>().join(","))).collect();
    let _ = std::fs::write(runtime_dir().join(format!("hover-test-{}.csv", unix_ms() / 1000)), format!("t,fly,battery,lock,frame_age,lag_ms,fps,heading,xy\n{rows}"));
}
