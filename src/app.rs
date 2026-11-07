#![windows_subsystem = "windows"]

use std::ffi::c_void;
use std::io::Write;
use std::path::{Path, PathBuf};
use std::process::{Command, Stdio};
use std::ptr::null_mut;
use std::time::{Duration, Instant};

const DRONE_PREFIX: &str = "SNAPTAIN";

#[repr(C)]
#[derive(Clone, Copy, PartialEq)]
struct Guid {
    a: u32,
    b: u16,
    c: u16,
    d: [u8; 8],
}

#[repr(C)]
struct InterfaceInfo {
    guid: Guid,
    description: [u16; 256],
    state: u32,
}

#[repr(C)]
struct InterfaceList {
    count: u32,
    index: u32,
    items: [InterfaceInfo; 1],
}

#[repr(C)]
struct ConnectParams {
    mode: u32,
    profile: *const u16,
    ssid: *const c_void,
    bssids: *const c_void,
    bss_type: u32,
    flags: u32,
}

#[link(name = "wlanapi")]
extern "system" {
    fn WlanOpenHandle(version: u32, reserved: *mut c_void, negotiated: *mut u32, handle: *mut *mut c_void) -> u32;
    fn WlanEnumInterfaces(handle: *mut c_void, reserved: *mut c_void, list: *mut *mut InterfaceList) -> u32;
    fn WlanConnect(handle: *mut c_void, guid: *const Guid, params: *const ConnectParams, reserved: *mut c_void) -> u32;
    fn WlanFreeMemory(memory: *mut c_void);
    fn WlanGetProfileList(handle: *mut c_void, guid: *const Guid, reserved: *mut c_void, list: *mut *mut u8) -> u32;
    fn WlanGetAvailableNetworkList(handle: *mut c_void, guid: *const Guid, flags: u32, reserved: *mut c_void, list: *mut *mut u8) -> u32;
    fn WlanScan(handle: *mut c_void, guid: *const Guid, ssid: *const c_void, ie: *const c_void, reserved: *mut c_void) -> u32;
    fn WlanSetProfile(handle: *mut c_void, guid: *const Guid, flags: u32, xml: *const u16, security: *const u16, overwrite: i32, reserved: *mut c_void, reason: *mut u32) -> u32;
}

#[link(name = "iphlpapi")]
extern "system" {
    fn GetAdaptersAddresses(family: u32, flags: u32, reserved: *mut c_void, addresses: *mut u8, size: *mut u32) -> u32;
}

struct Adapter {
    guid: Guid,
    name: String,
    netgear: bool,
}

fn runtime() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("runtime")
}

fn note(msg: &str) {
    if let Ok(mut f) = std::fs::OpenOptions::new().create(true).append(true).open(runtime().join("agent.log")) {
        let secs = std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).map(|d| d.as_secs()).unwrap_or(0);
        let _ = writeln!(f, "[{secs}] {msg}");
    }
}

fn wide_name(raw: &[u16]) -> String {
    String::from_utf16_lossy(&raw[..raw.iter().position(|c| *c == 0).unwrap_or(raw.len())])
}

fn guid_text(g: &Guid) -> String {
    format!("{{{:08X}-{:04X}-{:04X}-{:02X}{:02X}-{}}}", g.a, g.b, g.c, g.d[0], g.d[1], g.d[2..].iter().map(|b| format!("{b:02X}")).collect::<String>())
}

fn adapters(handle: *mut c_void) -> Vec<Adapter> {
    let mut list = null_mut();
    if unsafe { WlanEnumInterfaces(handle, null_mut(), &mut list) } != 0 || list.is_null() {
        return Vec::new();
    }
    let count = unsafe { (*list).count } as usize;
    let first = unsafe { std::ptr::addr_of!((*list).items) as *const InterfaceInfo };
    let out = (0..count)
        .map(|i| {
            let info = unsafe { &*first.add(i) };
            let name = wide_name(&info.description);
            Adapter { guid: info.guid, netgear: name.to_uppercase().contains("NETGEAR"), name }
        })
        .collect();
    unsafe { WlanFreeMemory(list as *mut c_void) };
    out
}

fn ipv4_of(guid: &Guid) -> Option<[u8; 4]> {
    let want = guid_text(guid);
    let mut size = 64 * 1024u32;
    let mut buf = vec![0u8; size as usize];
    if unsafe { GetAdaptersAddresses(2, 0x0e, null_mut(), buf.as_mut_ptr(), &mut size) } != 0 {
        return None;
    }
    let mut p = buf.as_ptr();
    while !p.is_null() {
        unsafe {
            let name = *(p.add(16) as *const *const u8);
            if std::ffi::CStr::from_ptr(name as *const i8).to_string_lossy().to_uppercase() == want {
                let uni = *(p.add(24) as *const *const u8);
                if uni.is_null() {
                    return None;
                }
                let sa = *(uni.add(16) as *const *const u8);
                return Some([*sa.add(4), *sa.add(5), *sa.add(6), *sa.add(7)]);
            }
            p = *(p.add(8) as *const *const u8);
        }
    }
    None
}

fn connect(handle: *mut c_void, guid: &Guid, profile: &str) -> u32 {
    let wide: Vec<u16> = profile.encode_utf16().chain(std::iter::once(0)).collect();
    let params = ConnectParams { mode: 0, profile: wide.as_ptr(), ssid: std::ptr::null(), bssids: std::ptr::null(), bss_type: 1, flags: 0 };
    unsafe { WlanConnect(handle, guid, &params, null_mut()) }
}

fn on_drone(ip: Option<[u8; 4]>) -> bool {
    ip.is_some_and(|a| a[0] == 172 && a[1] == 19)
}

fn saved_drones(handle: *mut c_void, guid: &Guid) -> Vec<String> {
    let mut list = null_mut();
    if unsafe { WlanGetProfileList(handle, guid, null_mut(), &mut list) } != 0 || list.is_null() {
        return Vec::new();
    }
    let count = unsafe { *(list as *const u32) } as usize;
    let out = (0..count)
        .map(|i| wide_name(unsafe { std::slice::from_raw_parts(list.add(8 + i * 516) as *const u16, 256) }))
        .filter(|n| n.starts_with(DRONE_PREFIX))
        .collect();
    unsafe { WlanFreeMemory(list as *mut c_void) };
    out
}

fn visible_drones(handle: *mut c_void, guid: &Guid) -> (Vec<String>, usize) {
    let mut list = null_mut();
    if unsafe { WlanGetAvailableNetworkList(handle, guid, 0, null_mut(), &mut list) } != 0 || list.is_null() {
        return (Vec::new(), 0);
    }
    let count = unsafe { *(list as *const u32) } as usize;
    let mut found: Vec<(u32, String)> = (0..count)
        .map(|i| unsafe {
            let n = list.add(8 + i * 628);
            let len = (*(n.add(512) as *const u32)).min(32) as usize;
            (*(n.add(604) as *const u32), String::from_utf8_lossy(std::slice::from_raw_parts(n.add(516), len)).to_string())
        })
        .filter(|(_, s)| s.starts_with(DRONE_PREFIX))
        .collect();
    unsafe { WlanFreeMemory(list as *mut c_void) };
    found.sort_by(|a, b| b.0.cmp(&a.0));
    (found.into_iter().map(|(_, s)| s).collect(), count)
}

fn add_open_profile(handle: *mut c_void, guid: &Guid, ssid: &str) -> u32 {
    let xml = format!(
        "<?xml version=\"1.0\"?><WLANProfile xmlns=\"http://www.microsoft.com/networking/WLAN/profile/v1\"><name>{ssid}</name><SSIDConfig><SSID><name>{ssid}</name></SSID></SSIDConfig><connectionType>ESS</connectionType><connectionMode>manual</connectionMode><MSM><security><authEncryption><authentication>open</authentication><encryption>none</encryption><useOneX>false</useOneX></authEncryption></security></MSM></WLANProfile>"
    );
    let wide: Vec<u16> = xml.encode_utf16().chain(std::iter::once(0)).collect();
    let mut reason = 0;
    unsafe { WlanSetProfile(handle, guid, 0, wide.as_ptr(), std::ptr::null(), 1, null_mut(), &mut reason) }
}

fn home_ssid() -> Option<String> {
    let from_env = std::env::var("S5C_HOME_SSID").ok();
    let from_file = std::fs::read_to_string(runtime().join("home-ssid.txt")).ok();
    from_env
        .or(from_file)
        .map(|s| s.lines().next().unwrap_or("").trim().to_string())
        .filter(|s| !s.is_empty())
}

fn keep_wifi() {
    let home = home_ssid();
    if home.is_none() {
        note("no home Wi-Fi set: put your own network name in runtime/home-ssid.txt (or S5C_HOME_SSID) and this PC will rejoin it after flying");
    }
    let mut handle = null_mut();
    let mut version = 0;
    if unsafe { WlanOpenHandle(2, null_mut(), &mut version, &mut handle) } != 0 {
        note("could not open the Wi-Fi service");
        return;
    }
    let (mut home_try, mut drone_try) = (Instant::now() - Duration::from_secs(60), Instant::now() - Duration::from_secs(60));
    let mut said = String::new();
    loop {
        let list = adapters(handle);
        let drone_side = list.iter().find(|a| a.netgear).or_else(|| list.get(1));
        let home_side = list.iter().find(|a| drone_side.is_none_or(|d| d.guid != a.guid));
        if let Some(a) = home_side {
            let ip = ipv4_of(&a.guid);
            if (ip.is_none() || on_drone(ip)) && home_try.elapsed() > Duration::from_secs(10) {
                home_try = Instant::now();
                if let Some(home) = home.as_deref() {
                    note(&format!("{} is on {ip:?} - reconnecting it to {home}: result {}", a.name, connect(handle, &a.guid, home)));
                }
            }
        }
        if let Some(a) = drone_side.filter(|a| !on_drone(ipv4_of(&a.guid))) {
            if drone_try.elapsed() > Duration::from_secs(20) {
                drone_try = Instant::now();
                unsafe { WlanScan(handle, &a.guid, std::ptr::null(), std::ptr::null(), null_mut()) };
                let ((seen, total), saved) = (visible_drones(handle, &a.guid), saved_drones(handle, &a.guid));
                if let Some(ssid) = seen.first().or(saved.first()).cloned() {
                    if !saved.contains(&ssid) {
                        note(&format!("found a new drone {ssid} - saving it: result {}", add_open_profile(handle, &a.guid, &ssid)));
                    }
                    let r = connect(handle, &a.guid, &ssid);
                    let msg = format!("{} joining drone {ssid} ({} drones of {total} networks in range): result {r}", a.name, seen.len());
                    if msg != said {
                        note(&msg);
                        said = msg;
                    }
                }
            }
        }
        std::thread::sleep(Duration::from_secs(3));
    }
}

fn hidden(cmd: &mut Command) -> &mut Command {
    use std::os::windows::process::CommandExt;
    cmd.creation_flags(0x0800_0000).stdin(Stdio::null()).stdout(Stdio::null()).stderr(Stdio::null())
}

fn newer(src: &Path, dst: &Path) -> bool {
    let at = |p: &Path| std::fs::metadata(p).and_then(|m| m.modified()).ok();
    at(src).is_some_and(|s| at(dst).is_none_or(|d| s > d))
}

fn keep_server() {
    let src = Path::new(env!("CARGO_MANIFEST_DIR")).join("target").join("release").join("serve.exe");
    let live = runtime().join("bin").join("serve.exe");
    loop {
        if newer(&src, &live) {
            let _ = std::fs::create_dir_all(live.parent().unwrap());
            match std::fs::copy(&src, &live) {
                Ok(_) => note("loaded the newest control server build"),
                Err(e) => note(&format!("could not copy the new server build yet: {e}")),
            }
        }
        match hidden(&mut Command::new(&live)).current_dir(env!("CARGO_MANIFEST_DIR")).status() {
            Ok(s) => note(&format!("control server stopped ({s}) - restarting")),
            Err(e) => note(&format!("control server would not start: {e}")),
        }
        std::thread::sleep(Duration::from_secs(1));
    }
}

fn keep_tunnel() {
    let exe = ["C:\\Program Files (x86)\\cloudflared\\cloudflared.exe", "C:\\Program Files\\cloudflared\\cloudflared.exe"]
        .into_iter()
        .find(|p| Path::new(p).exists())
        .unwrap_or("cloudflared");
    loop {
        let token = std::fs::read_to_string(runtime().join("tunnel-token.txt")).unwrap_or_default().trim().to_string();
        if token.is_empty() {
            std::thread::sleep(Duration::from_secs(30));
            continue;
        }
        let status = hidden(Command::new(exe).env("TUNNEL_TOKEN", token).args(["tunnel", "--no-autoupdate", "--logfile"]).arg(runtime().join("cloudflared.log")).arg("run")).status();
        note(&format!("Cloudflare tunnel stopped ({status:?}) - restarting"));
        std::thread::sleep(Duration::from_secs(3));
    }
}

#[link(name = "kernel32")]
extern "system" {
    fn CreateJobObjectW(attrs: *mut c_void, name: *const u16) -> *mut c_void;
    fn SetInformationJobObject(job: *mut c_void, class: u32, info: *const c_void, len: u32) -> i32;
    fn AssignProcessToJobObject(job: *mut c_void, process: *mut c_void) -> i32;
    fn GetCurrentProcess() -> *mut c_void;
}

fn stop_children_with_this_window() {
    unsafe {
        let job = CreateJobObjectW(null_mut(), std::ptr::null());
        let mut info = [0u8; 144];
        info[16..20].copy_from_slice(&0x2000u32.to_le_bytes());
        SetInformationJobObject(job, 9, info.as_ptr() as *const c_void, info.len() as u32);
        AssignProcessToJobObject(job, GetCurrentProcess());
    }
}

fn pin_to_desktop() {
    let Ok(me) = std::env::current_exe() else { return };
    let script = format!(
        "$d = [Environment]::GetFolderPath('Desktop'); \
         Copy-Item -LiteralPath '{}' -Destination ($d + '\\S5C Elite.exe') -Force; \
         Remove-Item -LiteralPath ($d + '\\S5C Elite.lnk'), ([Environment]::GetFolderPath('Startup') + '\\S5C Elite agent.lnk') -ErrorAction SilentlyContinue",
        me.display()
    );
    let _ = hidden(Command::new("powershell").args(["-NoProfile", "-Command", &script])).status();
}

const STARTING: &str = "<body style='margin:0;height:100vh;display:grid;place-items:center;background:#12161e;color:#ebeef5;font:15px system-ui'>Starting S5C Elite...</body>";

fn window() {
    use tao::event::{Event, WindowEvent};
    use tao::event_loop::{ControlFlow, EventLoopBuilder};
    use tao::platform::windows::IconExtWindows;
    use tao::window::{Icon, WindowBuilder};
    let event_loop = EventLoopBuilder::<()>::with_user_event().build();
    let window = WindowBuilder::new()
        .with_title("S5C Elite")
        .with_inner_size(tao::dpi::LogicalSize::new(1400.0, 880.0))
        .with_window_icon(Icon::from_resource(1, None).ok())
        .build(&event_loop)
        .expect("window");
    let mut context = wry::WebContext::new(Some(runtime().join("webview")));
    let webview = wry::WebViewBuilder::new_with_web_context(&mut context).with_html(STARTING).build(&window).expect("WebView2");
    let proxy = event_loop.create_proxy();
    std::thread::spawn(move || {
        let addr = "127.0.0.1:8080".parse().unwrap();
        while std::net::TcpStream::connect_timeout(&addr, Duration::from_millis(500)).is_err() {
            std::thread::sleep(Duration::from_millis(300));
        }
        let _ = proxy.send_event(());
    });
    event_loop.run(move |event, _, flow| {
        let _ = (&window, &context);
        *flow = ControlFlow::Wait;
        match event {
            Event::UserEvent(()) => {
                let _ = webview.load_url("http://127.0.0.1:8080/");
            }
            Event::WindowEvent { event: WindowEvent::CloseRequested, .. } => *flow = ControlFlow::Exit,
            _ => {}
        }
    });
}

fn main() {
    if std::env::args().any(|a| a == "--pin") {
        pin_to_desktop();
        return;
    }
    let lock = std::net::TcpListener::bind("127.0.0.1:8079");
    if lock.is_ok() {
        stop_children_with_this_window();
        note("app started: control server, Cloudflare tunnel and Wi-Fi keeper");
        std::thread::spawn(keep_wifi);
        std::thread::spawn(keep_tunnel);
        std::thread::spawn(keep_server);
    }
    window();
}
