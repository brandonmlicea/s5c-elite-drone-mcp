use crate::link::{bad, Link};
use crate::nav::Nav;
use std::fs::{self, File};
use std::io::{self, Write};
use std::net::UdpSocket;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

pub const TAKEOFF: u8 = 1;
pub const LAND: u8 = 2;
pub const STOP: u8 = 4;
pub const CALIBRATE: u8 = 128;
fn host() -> String {
    std::env::var("S5C_HOST").unwrap_or_else(|_| "172.19.10.1".into())
}

fn ctrl_port() -> u16 {
    std::env::var("S5C_PORT").ok().and_then(|p| p.parse().ok()).unwrap_or(8866)
}
const LOCAL_PORT: u16 = 8620;

#[derive(Clone, Copy)]
pub struct Sticks {
    pub roll: f32,
    pub pitch: f32,
    pub throttle: f32,
    pub yaw: f32,
    pub trim_roll: f32,
    pub trim_pitch: f32,
    pub hold_pitch: f32,
    pub hold_yaw: f32,
    pub flags: u8,
    pub flags_until: Instant,
}

#[derive(Clone, Copy, Default)]
pub struct Telemetry {
    pub seen: bool,
    pub battery: u8,
    pub fly: u8,
    pub status: u8,
}

impl Telemetry {
    pub fn needs_calibration(&self) -> bool {
        self.status & 0x20 != 0
    }
    pub fn cannot_take_off(&self) -> bool {
        self.status & 0x80 != 0
    }
    pub fn bad_flow(&self) -> bool {
        self.status & 0x10 != 0
    }
}

pub struct Drone {
    main: Arc<Mutex<Link>>,
    _notify: Link,
    kind: u8,
    local_port: u16,
    pub sticks: Arc<Mutex<Sticks>>,
    pub telemetry: Arc<Mutex<Telemetry>>,
    running: Arc<AtomicBool>,
    pub nav: Arc<Mutex<Nav>>,
}

fn packet(roll: f32, pitch: f32, throttle: f32, yaw: f32, flags: u8) -> [u8; 9] {
    let axis = |v: f32| (v.clamp(-100.0, 100.0) / 100.0 * 128.0 + 128.0).clamp(0.0, 255.0) as u8;
    let mut p = [0x66, axis(roll), axis(pitch), axis(throttle), axis(yaw), flags, 0, 0, 0x99];
    p[7] = p[1..7].iter().fold(0, |a, b| a ^ b);
    p
}

#[cfg(test)]
mod tests {
    use super::*;

    fn xor_ok(p: &[u8; 9]) -> bool {
        p[0] == 0x66 && p[8] == 0x99 && p[7] == p[1..7].iter().fold(0, |a, b| a ^ b)
    }

    #[test]
    fn neutral_centres_all_axes() {
        let p = packet(0.0, 0.0, 0.0, 0.0, 0);
        assert_eq!(&p[1..5], &[0x80, 0x80, 0x80, 0x80]);
        assert_eq!(p[5], 0);
        assert!(xor_ok(&p));
    }

    #[test]
    fn takeoff_sets_flag_bit() {
        let p = packet(0.0, 0.0, 0.0, 0.0, TAKEOFF);
        assert_eq!(p[5], 1);
        assert!(xor_ok(&p));
    }

    #[test]
    fn axes_map_and_clamp() {
        assert_eq!(packet(100.0, 0.0, 0.0, 0.0, 0)[1], 255);
        assert_eq!(packet(-100.0, 0.0, 0.0, 0.0, 0)[1], 0);
        assert_eq!(packet(200.0, 0.0, 0.0, 0.0, 0)[1], 255);
        assert_eq!(packet(0.0, 50.0, 0.0, 0.0, 0)[2], 0xC0);
    }

    #[test]
    fn status_bits_decode() {
        let t = Telemetry { seen: true, battery: 70, fly: 2, status: 0xA0 };
        assert!(t.needs_calibration());
        assert!(t.cannot_take_off());
        assert!(!t.bad_flow());
    }
}

pub fn runtime_dir() -> std::path::PathBuf {
    let dir = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("runtime");
    let _ = fs::create_dir_all(&dir);
    dir
}

pub fn unix_ms() -> u128 {
    SystemTime::now().duration_since(UNIX_EPOCH).map(|d| d.as_millis()).unwrap_or(0)
}

pub fn capture_file(kind: &str) -> Option<File> {
    let stamp = SystemTime::now().duration_since(UNIX_EPOCH).map(|d| d.as_secs()).unwrap_or(0);
    File::create(runtime_dir().join(format!("{kind}-{stamp}.jsonl"))).ok()
}

fn serial_request(local_port: u16) -> Vec<u8> {
    let mut r = 1u32.to_le_bytes().to_vec();
    r.extend_from_slice(&local_port.to_le_bytes());
    r.extend_from_slice(&1u16.to_le_bytes());
    r
}

impl Drone {
    pub fn connect() -> io::Result<Drone> {
        let addr = format!("{}:{}", host(), ctrl_port());
        let mut main = Link::open(&addr)?;
        let kind = match main.call(0, 1, 1, &[])?.first().copied().unwrap_or(8) {
            1 => 0x61,
            7 => 0x81,
            8 => 0x62,
            9 => 0x83,
            10 => 0x63,
            other => return Err(bad(format!("unknown device family {other}"))),
        };
        let mut notify = Link::open(&addr)?;
        notify.call(kind, 7, 1, &[])?;

        let mut config = Vec::new();
        for v in [115200u32, 8, 1, 0, 0] {
            config.extend_from_slice(&v.to_le_bytes());
        }
        main.call(kind, 3, 0x25, &config)?;
        let udp = UdpSocket::bind(("0.0.0.0", LOCAL_PORT))
            .or_else(|_| UdpSocket::bind(("0.0.0.0", 0)))?;
        udp.set_read_timeout(Some(Duration::from_millis(500)))?;
        let local_port = udp.local_addr()?.port();
        let request = serial_request(local_port);
        let ack = match main.call(kind, 0x0E, 0x0F, &request) {
            Ok(ack) => ack,
            Err(_) => {
                main.call(kind, 0x0E, 0x11, &request)?;
                main.call(kind, 0x0E, 0x0F, &request)?
            }
        };
        if ack.len() != 2 {
            return Err(bad(format!("serial start reply has {} bytes", ack.len())));
        }
        let port = u16::from_le_bytes([ack[0], ack[1]]);

        let sticks = Arc::new(Mutex::new(Sticks {
            roll: 0.0,
            pitch: 0.0,
            throttle: 0.0,
            yaw: 0.0,
            trim_roll: 0.0,
            trim_pitch: 0.0,
            hold_pitch: 0.0,
            hold_yaw: 0.0,
            flags: 0,
            flags_until: Instant::now(),
        }));
        let telemetry = Arc::new(Mutex::new(Telemetry::default()));
        let running = Arc::new(AtomicBool::new(true));

        let mut learned = Nav::default();
        learned.load(&runtime_dir().join("calib.txt"));
        let nav = Arc::new(Mutex::new(learned));

        let (tx, live, shared, tel, pos) = (udp.try_clone()?, running.clone(), sticks.clone(), telemetry.clone(), nav.clone());
        let mut flight = capture_file("flight");
        thread::spawn(move || {
            let (mut sr, mut sp, mut st, mut sy) = (0.0f32, 0.0f32, 0.0f32, 0.0f32);
            let mut tick = Instant::now();
            let mut saved = Instant::now();
            let ease = |now: &mut f32, target: f32, step: f32| {
                let d = target - *now;
                *now += d.clamp(-step, step);
            };
            while live.load(Ordering::Relaxed) {
                let t = *tel.lock().unwrap();
                let dt = tick.elapsed().as_secs_f32().min(0.3);
                tick = Instant::now();
                let n = {
                    let mut n = pos.lock().unwrap();
                    n.step(sr as f64, sp as f64, st as f64, sy as f64, t.fly, dt as f64, unix_ms() as f64);
                    if saved.elapsed() > Duration::from_secs(10) {
                        n.save(&runtime_dir().join("calib.txt"));
                        saved = Instant::now();
                    }
                    n.pose()
                };
                let (tr, tp, tt, ty, flags) = {
                    let s = shared.lock().unwrap();
                    let flags = if Instant::now() < s.flags_until { s.flags } else { 0 };
                    (s.roll + s.trim_roll, s.pitch + s.trim_pitch + s.hold_pitch, s.throttle, s.yaw + s.hold_yaw, flags)
                };
                ease(&mut sr, tr, 12.0);
                ease(&mut sp, tp, 12.0);
                ease(&mut st, tt, 12.0);
                ease(&mut sy, ty, 12.0);
                let _ = tx.send_to(&packet(sr, sp, st, sy, flags), (host().as_str(), port));
                if let Some(f) = flight.as_mut() {
                    let _ = writeln!(f, "{{\"ms\":{},\"roll\":{sr:.0},\"pitch\":{sp:.0},\"throttle\":{st:.0},\"yaw\":{sy:.0},\"flags\":{flags},\"fly\":{},\"battery\":{},\"x\":{:.4},\"y\":{:.4},\"z\":{:.4},\"heading\":{:.2},\"lock\":{:.2},\"yaw_gain\":{:.3}}}",
                        unix_ms(), t.fly, t.battery, n.x, n.y, n.z, n.heading, n.lock, n.yaw_gain);
                }
                thread::sleep(Duration::from_millis(70));
            }
        });

        let (live, shared) = (running.clone(), telemetry.clone());
        let mut cap = capture_file("telemetry");
        let mut raw = capture_file("serial-raw");
        let start = Instant::now();
        thread::spawn(move || {
            let mut window: Vec<u8> = Vec::new();
            let mut buf = [0u8; 4096];
            while live.load(Ordering::Relaxed) {
                let Ok((n, _)) = udp.recv_from(&mut buf) else { continue };
                if let Some(f) = raw.as_mut() {
                    let hex: String = buf[..n].iter().map(|b| format!("{b:02x}")).collect();
                    let _ = writeln!(f, "{{\"t\":{:.3},\"len\":{},\"raw\":\"{}\"}}",
                        start.elapsed().as_secs_f32(), n, hex);
                }
                window.extend_from_slice(&buf[..n]);
                let cut = window.len().saturating_sub(64);
                window.drain(..cut);
                let found = window.windows(11).rposition(|w| w[0] == 0xCC && w[1] == 3 && w[10] == 0x33);
                if let Some(i) = found {
                    let t = &window[i..i + 11];
                    *shared.lock().unwrap() =
                        Telemetry { seen: true, battery: t[2], fly: (t[3] >> 5) & 3, status: t[5] };
                    if let Some(f) = cap.as_mut() {
                        let hex: String = t.iter().map(|b| format!("{b:02x}")).collect();
                        let _ = writeln!(
                            f,
                            "{{\"t\":{:.3},\"ms\":{},\"battery\":{},\"fly\":{},\"status\":{},\"calibrate\":{},\"blocked\":{},\"flow_bad\":{},\"raw\":\"{}\"}}",
                            start.elapsed().as_secs_f32(),
                            unix_ms(),
                            t[2],
                            (t[3] >> 5) & 3,
                            t[5],
                            (t[5] >> 5) & 1,
                            (t[5] >> 7) & 1,
                            (t[5] >> 4) & 1,
                            hex
                        );
                    }
                }
            }
        });

        let main = Arc::new(Mutex::new(main));
        let (live, beat) = (running.clone(), main.clone());
        thread::spawn(move || {
            while live.load(Ordering::Relaxed) {
                let _ = beat.lock().unwrap().call(kind, 7, 3, &[]);
                thread::sleep(Duration::from_secs(4));
            }
        });

        Ok(Drone { main, _notify: notify, kind, local_port, sticks, telemetry, running, nav })
    }

    pub fn telemetry(&self) -> Telemetry {
        *self.telemetry.lock().unwrap()
    }

    pub fn set(&self, roll: f32, pitch: f32, throttle: f32, yaw: f32) {
        let mut s = self.sticks.lock().unwrap();
        (s.roll, s.pitch, s.throttle, s.yaw) = (roll, pitch, throttle, yaw);
    }

    pub fn command(&self, flag: u8) {
        if flag == TAKEOFF && self.telemetry().fly == 0 {
            self.nav.lock().unwrap().reset();
        }
        let mut s = self.sticks.lock().unwrap();
        s.flags = flag;
        s.flags_until = Instant::now() + Duration::from_secs(1);
    }

    pub fn trim(&self, droll: f32, dpitch: f32) -> (f32, f32) {
        let mut s = self.sticks.lock().unwrap();
        s.trim_roll = (s.trim_roll + droll).clamp(-20.0, 20.0);
        s.trim_pitch = (s.trim_pitch + dpitch).clamp(-20.0, 20.0);
        (s.trim_roll, s.trim_pitch)
    }

    pub fn kind(&self) -> u8 {
        self.kind
    }

    pub fn open_video(kind: u8) -> io::Result<Link> {
        let mut v = Link::open(&format!("{}:{}", host(), ctrl_port()))?;
        v.call(kind, 4, 1, &[1, 1, 0, 0, 0, 0, 5, 0, 0, 0, 1])?;
        Ok(v)
    }

    pub fn stop(&self) {
        self.running.store(false, Ordering::Relaxed);
    }

    pub fn close(self) {
        self.running.store(false, Ordering::Relaxed);
        self.nav.lock().unwrap().save(&runtime_dir().join("calib.txt"));
        thread::sleep(Duration::from_millis(200));
        let mut main = self.main.lock().unwrap();
        let _ = main.call(self.kind, 0x0E, 0x11, &serial_request(self.local_port));
        let _ = main.call(self.kind, 1, 3, &[]);
    }
}
