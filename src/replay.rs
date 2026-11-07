mod home;
mod nav;
mod odo;
mod sim;
mod video;

use odo::{difference, estimate, Img};
use std::fs;
use std::path::PathBuf;
use video::Stream;

fn runtime() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("runtime")
}

fn nearest(prefix: &str, stamp: u64) -> Option<PathBuf> {
    fs::read_dir(runtime())
        .ok()?
        .filter_map(|e| e.ok())
        .filter_map(|e| {
            let name = e.file_name().to_string_lossy().to_string();
            let s: u64 = name.strip_prefix(prefix)?.split('.').next()?.parse().ok()?;
            (s >= stamp && s - stamp <= 5).then_some((s, e.path()))
        })
        .min_by_key(|(s, _)| *s)
        .map(|(_, p)| p)
}

fn num(line: &str, key: &str) -> Option<f64> {
    let i = line.find(&format!("\"{key}\":"))? + key.len() + 3;
    line[i..].split([',', '}']).next()?.trim().parse().ok()
}

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let stamp: u64 = args.first().and_then(|s| s.parse().ok()).expect("usage: replay <stamp> [jpeg-every-n]");
    let jpeg_every: usize = args.get(1).and_then(|s| s.parse().ok()).unwrap_or(0);
    let raw = fs::read(runtime().join(format!("video-raw-{stamp}.h264"))).expect("video-raw file");
    let timing: Vec<(f64, f64)> = nearest("video-timing-", stamp)
        .and_then(|p| fs::read_to_string(p).ok())
        .map(|t| t.lines().filter_map(|l| Some((num(l, "drone_us")?, num(l, "ms")?))).collect())
        .unwrap_or_default();
    let offset = timing.iter().map(|(us, ms)| ms - us / 1000.0).fold(f64::MAX, f64::min);
    let flight: Vec<[f64; 6]> = fs::read_to_string(runtime().join(format!("flight-{stamp}.jsonl")))
        .unwrap_or_default()
        .lines()
        .filter_map(|l| Some([num(l, "ms")?, num(l, "roll")?, num(l, "pitch")?, num(l, "throttle")?, num(l, "yaw")?, num(l, "fly")?]))
        .collect();
    let stick_at = |ms: f64| flight.iter().take_while(|r| r[0] <= ms).last().copied().unwrap_or([ms, 0.0, 0.0, 0.0, 0.0, 0.0]);

    let mut bounds: Vec<(usize, u32)> = Vec::new();
    for i in 0..raw.len().saturating_sub(36) {
        if raw[i..i + 3] == [0, 0, 1] && (raw[i + 3] == 0xa0 || raw[i + 3] == 0xa1) && raw[i + 11] == 0x19 {
            bounds.push((i, u32::from_le_bytes(raw[i + 32..i + 36].try_into().unwrap())));
        }
    }
    bounds.push((raw.len(), u32::MAX));
    let mut stream = Stream::new().unwrap();
    let mut prev: Option<(Img, u32)> = None;
    let mut out = String::from("idx,drone_us,ms,dt_ms,dx,dy,scale,roll,conf,roll_stick,pitch,throttle,yaw,fly,diff\n");
    let mut decoded = 0;
    let mut thumbs: Vec<(u32, Vec<f32>)> = Vec::new();
    let mut shots: Vec<(f64, Img)> = Vec::new();
    let start = std::time::Instant::now();
    for (k, w) in bounds.windows(2).enumerate() {
        let (frames, _) = stream.push(&raw[w[0].0..w[1].0]);
        let Some(f) = frames.into_iter().last() else { continue };
        let us = bounds[k.saturating_sub(1)].1;
        if jpeg_every > 0 && decoded % jpeg_every == 0 {
            let _ = fs::write(runtime().join(format!("replay-{stamp}-{decoded:04}.jpg")), f.jpeg(70));
        }
        decoded += 1;
        let img = Img::from_frame(&f);
        if std::env::var("NAV").is_ok() {
            shots.push((us as f64 / 1000.0 + offset, img.clone()));
        }
        let ms = us as f64 / 1000.0 + offset;
        let s = stick_at(ms);
        if let Some((p, pus)) = &prev {
            if difference(p, &img) < odo::REPEAT {
                continue;
            }
            let m = estimate(p, &img);
            if std::env::var("EXP").is_ok() && m.conf > 0.03 && m.dx.abs() < 40.0 {
                let pts = odo::patch_motion(p, &img);
                let lin = |f: &dyn Fn(&odo::Patch) -> (f32, f32)| {
                    let w: f32 = pts.iter().map(|p| p.4).sum();
                    let mx = pts.iter().map(|p| f(p).0 * p.4).sum::<f32>() / w;
                    let my = pts.iter().map(|p| f(p).1 * p.4).sum::<f32>() / w;
                    let num: f32 = pts.iter().map(|p| p.4 * (f(p).0 - mx) * (f(p).1 - my)).sum();
                    let den: f32 = pts.iter().map(|p| p.4 * (f(p).0 - mx).powi(2)).sum();
                    num / den.max(1e-3)
                };
                if pts.len() >= 8 {
                    println!("EXP {} {} {:.3} {:.5} {:.5}", s[4], s[2], m.dx, lin(&|p| (p.0, p.2)), lin(&|p| (p.1, p.3)));
                }
            }
            if std::env::var("COLS").is_ok() && s[4].abs() >= 30.0 {
                let mut cols = [(0f32, 0f32); 5];
                for (px, _, x, _, c) in odo::patch_motion(p, &img) {
                    let k = ((px + 160.0) / 64.0) as usize;
                    cols[k.min(4)].0 += x * c;
                    cols[k.min(4)].1 += c;
                }
                let v: Vec<String> = cols.iter().map(|(s, w)| if *w > 0.0 { format!("{:7.2}", s / w) } else { "    -  ".into() }).collect();
                println!("yaw {:4} dx {:7.2} cols {}", s[4], m.dx, v.join(" "));
            }
            out += &format!(
                "{decoded},{us},{ms:.0},{:.0},{:.3},{:.3},{:.5},{:.5},{:.2},{},{},{},{},{},{:.4}\n",
                us.wrapping_sub(*pus) as f64 / 1000.0, m.dx, m.dy, m.scale, m.roll, m.conf, s[1], s[2], s[3], s[4], s[5], difference(p, &img)
            );
        }
        if std::env::var("LOOP").is_ok() {
            thumbs.push((us, img.pyramid()[2].p.clone()));
        }
        prev = Some((img, us));
    }
    if std::env::var("LOOP").is_ok() {
        let lines: Vec<Vec<f64>> = out.lines().skip(1).map(|l| l.split(',').map(|v| v.parse().unwrap_or(0.0)).collect()).collect();
        let mut acc = vec![(0u32, 0f64, 0f64)];
        for r in &lines {
            let ok = r[8] > 0.03 && r[4].abs() < 40.0;
            let last = *acc.last().unwrap();
            acc.push((r[1] as u32, last.1 + if ok { r[4] } else { 0.0 }, last.2 + if ok { 0.0 } else { 1.0 }));
        }
        let at = |us: u32| acc.iter().find(|a| a.0 == us).copied();
        for i in (0..thumbs.len()).step_by(3) {
            let (ui, ti) = &thumbs[i];
            let mut best = (f32::MAX, 0, 0);
            for (j, (_, tj)) in thumbs.iter().enumerate().skip(i + 30) {
                for sx in -20..=20 {
                    let mut c = 0.0;
                    for y in 4..41 {
                        for x in 22..58 {
                            c += (ti[y * 80 + x] - tj[y * 80 + (x as i32 + sx) as usize]).abs();
                        }
                    }
                    if c < best.0 {
                        best = (c, j, sx);
                    }
                }
            }
            let (c, j, sx) = best;
            if let (Some(a), Some(b)) = (at(*ui), at(thumbs[j].0)) {
                println!("LOOP {i} -> {j}: cost {:.3} shift {sx} (x4 = {} px)  integrated dx {:.1} px  unmeasured frames {}", c / (37.0 * 36.0), sx * 4, b.1 - a.1, b.2 - a.2);
            }
        }
    }
    if std::env::var("NAV").is_ok() {
        nav_check(&flight, shots);
        return;
    }
    let path = runtime().join(format!("motion-{stamp}.csv"));
    fs::write(&path, out).unwrap();
    eprintln!("{decoded} frames in {:.1}s -> {}", start.elapsed().as_secs_f32(), path.display());
}

fn nav_check(flight: &[[f64; 6]], shots: Vec<(f64, Img)>) {
    let mut n = nav::Nav::default();
    n.load(&runtime().join("calib.txt"));
    let mut shots = shots.into_iter().peekable();
    let (mut prev, mut fly_before, mut checks) = (None::<[f64; 6]>, 0.0, Vec::new());
    for row in flight {
        while shots.peek().is_some_and(|(t, _)| *t <= row[0]) {
            let (t, img) = shots.next().unwrap();
            let home = n.home.clone();
            n.frame(img.clone(), t);
            if let Some(h) = home {
                let v = odo::compare(&h, &img);
                if v.err < 0.2 && v.conf > 0.6 {
                    let seen = -(v.dx as f64) / n.px_per_deg;
                    let tracked = (n.heading() + 180.0).rem_euclid(360.0) - 180.0;
                    checks.push((t, seen, tracked, n.swept / n.px_per_deg));
                }
            }
        }
        if row[5] >= 1.0 && fly_before == 0.0 {
            n.reset();
        }
        fly_before = row[5];
        if let Some(p) = prev {
            n.step(p[1], p[2], p[3], p[4], p[5] as u8, ((row[0] - p[0]) / 1000.0).min(0.3), row[0]);
        }
        prev = Some(*row);
    }
    let mut last_turn = f64::NAN;
    let mut worst: f64 = 0.0;
    for (t, seen, tracked, turned) in &checks {
        if (turned - last_turn).abs() < 5.0 {
            continue;
        }
        last_turn = *turned;
        let err = ((tracked - seen) + 180.0).rem_euclid(360.0) - 180.0;
        worst = worst.max(err.abs());
        println!("t {:.1}s  after {:6.0} deg of camera-tracked turning: takeoff view says heading {:+7.2}, tracker says {:+7.2}, error {:+6.2} deg", (t - checks[0].0) / 1000.0, turned, seen, tracked, err);
    }
    println!("{} sightings of the takeoff view; worst heading error {worst:.2} deg; learned slide {:.3}, turn gain {:.3}", checks.len(), n.slide(), n.yaw_gain);
}
