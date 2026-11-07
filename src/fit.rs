use std::fs;
use std::path::PathBuf;

fn runtime() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("runtime")
}

fn num(line: &str, key: &str) -> Option<f64> {
    let i = line.find(&format!("\"{key}\":"))? + key.len() + 3;
    line[i..].split([',', '}']).next()?.trim().parse().ok()
}

fn integrate(sticks: &[(f64, f64)], t0: f64, t1: f64, dead: f64, tau: f64) -> Vec<(f64, f64, f64)> {
    let (mut w1, mut w2, mut r1, mut r2, mut out) = (0.0, 0.0, 0.0, 0.0, Vec::new());
    let mut k = 0;
    let mut t = t0;
    while t < t1 {
        while k + 1 < sticks.len() && sticks[k + 1].0 <= t - dead {
            k += 1;
        }
        let s = if sticks[k].0 <= t - dead { sticks[k].1 / 100.0 } else { 0.0 };
        let a = (5.0 / tau).min(1.0);
        w1 += (s - w1) * a;
        w2 += (s * s.abs() - w2) * a;
        r1 += w1 * 0.005;
        r2 += w2 * 0.005;
        out.push((t, r1, r2));
        t += 5.0;
    }
    out
}

fn main() {
    let stamp = std::env::args().nth(1).expect("usage: fit <stamp>");
    let flight = fs::read_to_string(runtime().join(format!("flight-{stamp}.jsonl"))).unwrap();
    let sticks: Vec<(f64, f64)> = flight.lines().filter_map(|l| Some((num(l, "ms")?, num(l, "yaw")?))).collect();
    let csv = fs::read_to_string(runtime().join(format!("motion-{stamp}.csv"))).unwrap();
    let rows: Vec<Vec<f64>> = csv.lines().skip(1).map(|l| l.split(',').map(|v| v.parse().unwrap_or(0.0)).collect()).collect();
    let good: Vec<&Vec<f64>> = rows.iter().filter(|r| r[3] <= 81.0 && r[8] >= 0.04 && r[4].abs() < 40.0 && r[13] == 2.0 && r[2] < std::env::args().nth(2).and_then(|s| s.parse().ok()).unwrap_or(f64::MAX)).collect();
    let (t0, t1) = (sticks[0].0, sticks.last().unwrap().0);
    let mut best = (f64::MAX, 0.0, 0.0, 0.0, 0.0);
    for dead in (0..=600).step_by(20) {
        for tau in (40..=800).step_by(20) {
            let r = integrate(&sticks, t0, t1, dead as f64, tau as f64);
            let at = |t: f64| r[(((t - t0) / 5.0).max(0.0) as usize).min(r.len() - 1)];
            let (mut a11, mut a12, mut a22, mut b1, mut b2) = (0.0, 0.0, 0.0, 0.0, 0.0);
            let feats: Vec<(f64, f64, f64)> = good
                .iter()
                .map(|row| {
                    let (p, c) = (at(row[2] - row[3]), at(row[2]));
                    (c.1 - p.1, c.2 - p.2, -row[4])
                })
                .collect();
            for (x1, x2, y) in &feats {
                a11 += x1 * x1;
                a12 += x1 * x2;
                a22 += x2 * x2;
                b1 += x1 * y;
                b2 += x2 * y;

            }
            let det = a11 * a22 - a12 * a12;
            if det.abs() < 1e-12 {
                continue;
            }
            let (a, b) = ((b1 * a22 - b2 * a12) / det, (a11 * b2 - a12 * b1) / det);
            let sse = feats.iter().map(|(x1, x2, y)| (y - a * x1 - b * x2).powi(2)).sum::<f64>();
            if sse < best.0 {
                best = (sse, dead as f64, tau as f64, a, b);
            }
        }
    }
    let (sse, dead, tau, a, b) = best;
    let n = good.len() as f64;
    let total: f64 = good.iter().map(|r| r[4] * r[4]).sum();
    println!("frames {n}  dead {dead} ms  tau {tau} ms  rate px/s = {a:.1}*s + {b:.1}*s|s|  (s in -1..1)");
    println!("rms residual {:.2} px/frame, rms signal {:.2} px/frame, explained {:.0}%", (sse / n).sqrt(), (total / n).sqrt(), 100.0 * (1.0 - sse / total));
    let r = integrate(&sticks, t0, t1, dead, tau);
    let at = |t: f64| r[(((t - t0) / 5.0).max(0.0) as usize).min(r.len() - 1)];
    let pts: Vec<(f64, f64)> = good
        .iter()
        .map(|row| {
            let (p, c) = (at(row[2] - row[3]), at(row[2]));
            let pred = a * (c.1 - p.1) + b * (c.2 - p.2);
            (pred, -row[4] - pred)
        })
        .collect();
    for (name, pick) in [("turning left", -1.0), ("turning right", 1.0)] {
        let sel: Vec<_> = pts.iter().filter(|(p, _)| p * pick > 3.0).collect();
        let n = sel.len() as f64;
        let mean = sel.iter().map(|(_, e)| e).sum::<f64>() / n.max(1.0);
        let sd = (sel.iter().map(|(_, e)| (e - mean).powi(2)).sum::<f64>() / (n - 1.0).max(1.0)).sqrt();
        let rot = sel.iter().map(|(p, _)| p.abs()).sum::<f64>() / n.max(1.0);
        println!("{name}: {n} frames, mean rotation {rot:.2} px/frame, unexplained extra shift {mean:+.3} px/frame (+-{:.3})", sd / n.max(1.0).sqrt());
    }
    for s in [0.1, 0.2, 0.35, 0.5, 0.6, 0.7, 0.85, 1.0] {
        println!("  stick {:>3.0}% -> {:>6.1} px/s", s * 100.0, a * s + b * s * s);
    }
}
