mod home;
mod nav;
mod odo;
mod sim;
mod video;

fn main() {
    let args: Vec<u64> = std::env::args().skip(1).filter_map(|a| a.parse().ok()).collect();
    let (from, to) = (args.first().copied().unwrap_or(1), args.get(1).copied().unwrap_or(args.first().copied().unwrap_or(1)));
    let trace = from == to;
    let secs = std::env::var("SECS").ok().and_then(|s| s.parse().ok()).unwrap_or(20.0);
    let (mut landed, mut crashed, mut errs) = (0, 0, Vec::new());
    for seed in from..=to {
        let Some(o) = sim::mission(seed, secs, trace) else {
            println!("seed {seed}: pilot flew too close to a wall, skipped");
            continue;
        };
        println!("seed {seed:3}: {:?} | {:.3} m from takeoff point (was {:.2} m away) | heading off {:+.2} deg | closest wall {:.2} m | {:.1}s{}", o.phase, o.error_m, o.away_m, o.heading_err, o.clearance, o.secs, if o.crashed { " | CRASHED" } else { "" });
        crashed += o.crashed as u32;
        if o.phase == home::Phase::Land {
            landed += 1;
            errs.push(o.error_m);
        }
    }
    errs.sort_by(|a, b| a.total_cmp(b));
    if let Some(worst) = errs.last() {
        println!("landed {landed}, crashed {crashed}, median miss {:.3} m, worst miss {worst:.3} m", errs[errs.len() / 2]);
    }
}
