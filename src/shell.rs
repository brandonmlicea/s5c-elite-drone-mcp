use std::io::{Read, Write};
use std::net::TcpStream;
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};


fn telnet(host: &str) -> std::io::Result<TcpStream> {
    let mut s = TcpStream::connect_timeout(&format!("{host}:23").parse().unwrap(), Duration::from_secs(3))?;
    s.set_read_timeout(Some(Duration::from_millis(400)))?;
    let mut buf = [0u8; 2048];
    let end = Instant::now() + Duration::from_secs(2);
    while Instant::now() < end {
        let Ok(n) = s.read(&mut buf) else { continue };
        let mut reply = Vec::new();
        let mut i = 0;
        while i + 2 < n {
            if buf[i] == 255 {
                match buf[i + 1] {
                    251 | 252 => reply.extend_from_slice(&[255, 254, buf[i + 2]]),
                    253 | 254 => reply.extend_from_slice(&[255, 252, buf[i + 2]]),
                    _ => {}
                }
                i += 3;
            } else {
                i += 1;
            }
        }
        s.write_all(&reply)?;
    }
    Ok(s)
}

fn run(s: &mut TcpStream, cmd: &str, secs: u64) -> String {
    let mark = "__S5C_DONE__";
    let _ = s.write_all(format!("{cmd} 2>&1; echo {mark}\n").as_bytes());
    let (mut out, mut buf, end) = (Vec::new(), [0u8; 4096], Instant::now() + Duration::from_secs(secs));
    while Instant::now() < end {
        if let Ok(n) = s.read(&mut buf) {
            out.extend(buf[..n].iter().filter(|b| **b != 255 && **b != 0 && **b != b'\r'));
            if String::from_utf8_lossy(&out).matches(mark).count() >= 2 {
                break;
            }
        }
    }
    let text = String::from_utf8_lossy(&out).to_string();
    let body = text.split_once('\n').map_or(text.as_str(), |(_, b)| b);
    body.split(mark).next().unwrap_or("").trim_end().to_string()
}

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let lines: Vec<String> = if args.is_empty() {
        std::io::stdin().lines().map_while(Result::ok).filter(|l| !l.trim().is_empty()).collect()
    } else {
        vec![args.join(" ")]
    };
    if lines.is_empty() {
        eprintln!("usage: shell \"<command>\" | commands on stdin");
        return;
    }
    let host = std::env::var("S5C_HOST").unwrap_or_else(|_| "172.19.10.1".into());
    let stamp = SystemTime::now().duration_since(UNIX_EPOCH).map(|d| d.as_secs()).unwrap_or(0);
    let path = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("runtime").join(format!("shell-{stamp}.txt"));
    let mut log = String::new();
    match telnet(&host) {
        Ok(mut s) => {
            for c in lines {
                let (cmd, secs) = c.split_once(" #wait=").map_or((c.as_str(), 6), |(a, b)| (a, b.trim().parse().unwrap_or(6)));
                let block = format!("$ {cmd}\n{}\n\n", run(&mut s, cmd, secs));
                print!("{block}");
                log += &block;
            }
        }
        Err(e) => {
            let msg = format!("telnet to {host}:23 failed: {e} (drone off, or the Nighthawk is not on its hotspot)\n");
            eprint!("{msg}");
            log += &msg;
        }
    }
    let _ = std::fs::write(&path, &log);
}
