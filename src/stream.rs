use std::collections::VecDeque;
use std::io::Write;
use std::sync::{Arc, Condvar, Mutex};
use std::time::Duration;
use tiny_http::{Header, Request, Response, StatusCode};

const BACKLOG: usize = 8;
const GOP_CAP: usize = 120;
const GUID: &str = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11";

type Msg = Arc<Vec<u8>>;

#[derive(Default)]
struct Queue {
    items: Mutex<(VecDeque<Msg>, bool)>,
    ready: Condvar,
}

impl Queue {
    fn take(&self) -> Option<Msg> {
        let mut g = self.items.lock().unwrap();
        loop {
            if g.1 {
                return None;
            }
            if let Some(m) = g.0.pop_front() {
                return Some(m);
            }
            g = self.ready.wait_timeout(g, Duration::from_secs(5)).unwrap().0;
        }
    }
}

#[derive(Default)]
pub struct Hub {
    viewers: Mutex<Vec<Arc<Queue>>>,
    gop: Mutex<Vec<Msg>>,
}

impl Hub {
    pub fn new() -> Hub {
        Hub::default()
    }

    pub fn count(&self) -> usize {
        self.viewers.lock().unwrap().len()
    }

    fn join(&self) -> Arc<Queue> {
        let q = Arc::new(Queue::default());
        q.items.lock().unwrap().0.extend(self.gop.lock().unwrap().iter().cloned());
        self.viewers.lock().unwrap().push(q.clone());
        q
    }

    pub fn video(&self, frame: &[u8], ts: u32) {
        let key = frame.get(3) == Some(&0xa1);
        let body = frame.get(44..).unwrap_or(&[]);
        let mut msg = Vec::with_capacity(body.len() + 6);
        msg.extend_from_slice(&[1, key as u8]);
        msg.extend_from_slice(&ts.to_le_bytes());
        msg.extend_from_slice(body);
        let msg = Arc::new(msg);
        let gop = {
            let mut g = self.gop.lock().unwrap();
            if key {
                g.clear();
            }
            if g.len() < GOP_CAP && (key || !g.is_empty()) {
                g.push(msg.clone());
            }
            g.clone()
        };
        self.viewers.lock().unwrap().retain(|q| {
            if Arc::strong_count(q) == 1 {
                return false;
            }
            let mut items = q.items.lock().unwrap();
            let video_waiting = items.0.iter().filter(|m| m[0] == 1).count();
            if video_waiting >= BACKLOG {
                items.0.retain(|m| m[0] != 1);
                items.0.extend(gop.iter().cloned());
            } else {
                items.0.push_back(msg.clone());
            }
            q.ready.notify_one();
            true
        });
    }

    pub fn text(&self, json: &str) {
        let msg = Arc::new([&[2u8][..], json.as_bytes()].concat());
        for q in self.viewers.lock().unwrap().iter() {
            let mut items = q.items.lock().unwrap();
            items.0.retain(|m| m[0] != 2);
            items.0.push_back(msg.clone());
            q.ready.notify_one();
        }
    }
}

fn base64(data: &[u8]) -> String {
    const T: &[u8] = b"ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    let mut out = String::new();
    for c in data.chunks(3) {
        let n = (c[0] as u32) << 16 | (*c.get(1).unwrap_or(&0) as u32) << 8 | *c.get(2).unwrap_or(&0) as u32;
        for i in 0..4 {
            out.push(if i <= c.len() { T[(n >> (18 - 6 * i) & 63) as usize] as char } else { '=' });
        }
    }
    out
}

fn frame(out: &mut impl Write, msg: &[u8]) -> std::io::Result<()> {
    let (opcode, body) = if msg[0] == 2 { (0x81, &msg[1..]) } else { (0x82, msg) };
    let mut head = vec![opcode];
    match body.len() {
        n if n < 126 => head.push(n as u8),
        n if n < 65536 => {
            head.push(126);
            head.extend_from_slice(&(n as u16).to_be_bytes());
        }
        n => {
            head.push(127);
            head.extend_from_slice(&(n as u64).to_be_bytes());
        }
    }
    out.write_all(&head)?;
    out.write_all(body)?;
    out.flush()
}

pub fn serve(hub: Arc<Hub>, req: Request) {
    let key = req.headers().iter().find(|h| h.field.equiv("Sec-WebSocket-Key")).map(|h| h.value.to_string());
    let Some(key) = key else {
        let _ = req.respond(Response::empty(400));
        return;
    };
    let accept = base64(&sha1_smol::Sha1::from(format!("{key}{GUID}")).digest().bytes());
    let resp = Response::empty(StatusCode(101)).with_header(Header::from_bytes(&b"Sec-WebSocket-Accept"[..], accept.as_bytes()).unwrap());
    let q = hub.join();
    let mut sock = req.upgrade("websocket", resp);
    while let Some(msg) = q.take() {
        if frame(&mut sock, &msg).is_err() {
            break;
        }
    }
    q.items.lock().unwrap().1 = true;
}

pub fn secret() -> String {
    #[link(name = "bcrypt")]
    extern "system" {
        fn BCryptGenRandom(alg: *mut std::ffi::c_void, buf: *mut u8, len: u32, flags: u32) -> i32;
    }
    let mut b = [0u8; 24];
    unsafe { BCryptGenRandom(std::ptr::null_mut(), b.as_mut_ptr(), b.len() as u32, 2) };
    b.iter().map(|x| format!("{x:02x}")).collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn drone_frame(key: bool) -> Vec<u8> {
        [0, 0, 1, if key { 0xa1 } else { 0xa0 }].iter().copied().chain(std::iter::repeat(0).take(48)).collect()
    }

    fn drain(q: &Queue) -> Vec<(bool, u32)> {
        q.items.lock().unwrap().0.drain(..).filter(|m| m[0] == 1).map(|m| (m[1] == 1, u32::from_le_bytes(m[2..6].try_into().unwrap()))).collect()
    }

    #[test]
    fn handshake_accept_matches_rfc6455_example() {
        let digest = sha1_smol::Sha1::from(format!("dGhlIHNhbXBsZSBub25jZQ=={GUID}")).digest().bytes();
        assert_eq!(base64(&digest), "s3pPLMBiTxaQ9kYGzzhZRbK+xOo=");
    }

    #[test]
    fn a_new_viewer_gets_the_picture_instantly_from_the_cached_keyframe() {
        let hub = Hub::new();
        hub.video(&drone_frame(false), 1);
        hub.video(&drone_frame(true), 2);
        for t in 3..6 {
            hub.video(&drone_frame(false), t);
        }
        let q = hub.join();
        assert_eq!(drain(&q), vec![(true, 2), (false, 3), (false, 4), (false, 5)]);
    }

    #[test]
    fn a_slow_viewer_catches_up_to_live_instead_of_falling_behind() {
        let hub = Hub::new();
        hub.video(&drone_frame(true), 1);
        let q = hub.join();
        for t in 2..40 {
            hub.video(&drone_frame(false), t);
        }
        let got = drain(&q);
        assert!(got.len() <= BACKLOG + 40 && got[0] == (true, 1), "{got:?}");
        assert_eq!(got.last(), Some(&(false, 39)));
        let mut prev = 0;
        for (_, t) in &got {
            assert!(*t > prev || *t == 1, "out of order {got:?}");
            prev = *t;
        }
    }
}
