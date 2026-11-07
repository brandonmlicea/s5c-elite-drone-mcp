use crate::home::{mouse_stick, Homing, Phase, Sense};
use crate::nav::{yaw_rate, Nav, PX_PER_DEG};
use crate::odo::{Img, TH, TW};

pub const ROOM: [f64; 3] = [5.0, 4.0, 2.6];
pub const HOME: [f64; 2] = [2.0, 2.0];
const SUB: f64 = 0.01;

fn hash(x: i32, y: i32) -> f32 {
    let mut h = (x as u32).wrapping_mul(0x8da6_b343) ^ (y as u32).wrapping_mul(0xd816_3841);
    h = (h ^ (h >> 13)).wrapping_mul(0x85eb_ca6b);
    h ^= h >> 16;
    (h & 0xffff) as f32 / 65535.0
}

fn noise(u: f32, v: f32) -> f32 {
    let (x, y) = (u.floor(), v.floor());
    let (fx, fy) = (u - x, v - y);
    let (sx, sy) = (fx * fx * (3.0 - 2.0 * fx), fy * fy * (3.0 - 2.0 * fy));
    let (x, y) = (x as i32, y as i32);
    let top = hash(x, y) + (hash(x + 1, y) - hash(x, y)) * sx;
    let bot = hash(x, y + 1) + (hash(x + 1, y + 1) - hash(x, y + 1)) * sx;
    top + (bot - top) * sy
}

pub fn texture(u: f32, v: f32) -> f32 {
    40.0 + 120.0 * noise(u, v) + 60.0 * noise(u * 2.3 + 17.0, v * 2.3 - 5.0) + 35.0 * noise(u * 5.1 - 3.0, v * 5.1 + 11.0)
}

pub struct Rng(u64);

impl Rng {
    pub fn new(seed: u64) -> Rng {
        Rng(seed.wrapping_mul(0x9E37_79B9_7F4A_7C15) | 1)
    }
    pub fn f(&mut self) -> f64 {
        self.0 ^= self.0 << 13;
        self.0 ^= self.0 >> 7;
        self.0 ^= self.0 << 17;
        (self.0 >> 11) as f64 / (1u64 << 53) as f64
    }
    pub fn range(&mut self, a: f64, b: f64) -> f64 {
        a + (b - a) * self.f()
    }
}

fn wall(face: usize, a: f64, b: f64, plain: f32) -> f32 {
    let (a, b) = (a as f32, b as f32);
    let blocks: [(f32, f32, f32, f32, f32); 2] = match face {
        0 => [(1.2, 2.1, 0.0, 2.0, 25.0), (2.8, 3.6, 0.9, 1.6, 200.0)],
        1 => [(0.5, 1.4, 0.0, 2.0, 30.0), (2.5, 3.5, 0.0, 0.8, 70.0)],
        2 => [(3.0, 3.9, 0.0, 2.0, 210.0), (0.8, 2.0, 1.1, 1.8, 40.0)],
        3 => [(1.0, 1.8, 0.0, 2.0, 35.0), (2.6, 4.2, 0.0, 0.9, 90.0)],
        _ => [(0.0, 0.0, 0.0, 0.0, 0.0); 2],
    };
    for (a0, a1, b0, b1, v) in blocks {
        if a >= a0 && a <= a1 && b >= b0 && b <= b1 {
            return v + 10.0 * noise(a * 9.0, b * 9.0);
        }
    }
    let base = texture(a * 3.0 + face as f32 * 50.0, b * 3.0);
    130.0 + (base - 130.0) * plain
}

pub fn render(pos: [f64; 3], heading_deg: f64, ppd: f64, plain: f32, grain: &mut Rng) -> Img {
    let mut p = Vec::with_capacity(TW * TH);
    for v in 0..TH {
        let el = ((TH as f64 / 2.0 - v as f64 - 0.5) / ppd).to_radians();
        for u in 0..TW {
            let az = heading_deg + (u as f64 + 0.5 - TW as f64 / 2.0) / ppd;
            let (s, c) = az.to_radians().sin_cos();
            let d = [el.cos() * c, el.cos() * s, el.sin()];
            let (mut t, mut axis) = (f64::MAX, 0);
            for k in 0..3 {
                if d[k].abs() > 1e-9 {
                    let tk = if d[k] > 0.0 { (ROOM[k] - pos[k]) / d[k] } else { -pos[k] / d[k] };
                    if tk < t {
                        (t, axis) = (tk, k);
                    }
                }
            }
            let hit = [pos[0] + d[0] * t, pos[1] + d[1] * t, pos[2] + d[2] * t];
            let val = match (axis, d[axis] > 0.0) {
                (0, true) => wall(0, hit[1], hit[2], plain),
                (0, false) => wall(1, ROOM[1] - hit[1], hit[2], plain),
                (1, true) => wall(2, ROOM[0] - hit[0], hit[2], plain),
                (1, false) => wall(3, hit[0], hit[2], plain),
                (2, false) => 60.0 + 0.5 * texture(hit[0] as f32 * 4.0 + 200.0, hit[1] as f32 * 4.0),
                _ => 200.0 + 0.15 * texture(hit[0] as f32 + 300.0, hit[1] as f32),
            };
            p.push(val + (grain.f() as f32 - 0.5) * 3.0);
        }
    }
    Img::new(TW, TH, p)
}

#[derive(Clone, Copy, Debug)]
pub struct Truth {
    pub pos: [f64; 3],
    pub heading: f64,
    pub w: f64,
    pub v: [f64; 2],
    pub fly: u8,
}

pub struct World {
    pub t: f64,
    pub truth: Truth,
    pub gain: f64,
    pub tau: f64,
    pub speed: f64,
    pub ppd: f64,
    pub plain: f32,
    pub drift: [f64; 2],
    pub spin_drift: [f64; 2],
    pub clearance: f64,
    pub crashed: bool,
    climb_until: f64,
    rng: Rng,
}

impl World {
    pub fn new(seed: u64) -> World {
        let mut rng = Rng::new(seed);
        let dir = rng.range(0.0, std::f64::consts::TAU);
        let sdir = rng.range(0.0, std::f64::consts::TAU);
        let (d, sd) = (rng.range(0.0, 0.04), rng.range(0.05, 0.35));
        World {
            t: 0.0,
            truth: Truth { pos: [HOME[0], HOME[1], 0.0], heading: 0.0, w: 0.0, v: [0.0; 2], fly: 0 },
            gain: rng.range(0.8, 1.2),
            tau: rng.range(0.18, 0.28),
            speed: rng.range(0.8, 1.6),
            ppd: PX_PER_DEG,
            plain: rng.range(0.35, 1.0) as f32,
            drift: [d * dir.cos(), d * dir.sin()],
            spin_drift: [sd * sdir.cos(), sd * sdir.sin()],
            clearance: f64::MAX,
            crashed: false,
            climb_until: f64::MAX,
            rng,
        }
    }

    pub fn takeoff(&mut self) {
        self.truth.fly = 1;
        self.climb_until = self.t + 2200.0;
    }

    pub fn advance(&mut self, sticks: [f64; 4], ms: f64) {
        let steps = (ms / (SUB * 1000.0)).round() as usize;
        for _ in 0..steps {
            self.t += SUB * 1000.0;
            let tr = &mut self.truth;
            if tr.fly == 0 {
                continue;
            }
            if tr.fly == 1 {
                tr.pos[2] = (tr.pos[2] + 0.5 * SUB).min(1.0);
                if self.t >= self.climb_until {
                    tr.fly = 2;
                }
            }
            let [roll, pitch, throttle, yaw] = sticks;
            tr.w += (self.gain * yaw_rate(yaw) - tr.w) * (1.0 - (-SUB / self.tau).exp());
            tr.heading += tr.w / self.ppd * SUB;
            let k = 1.0 - (-SUB / 0.4).exp();
            tr.v[0] += (pitch / 100.0 * self.speed - tr.v[0]) * k;
            tr.v[1] += (roll / 100.0 * self.speed - tr.v[1]) * k;
            let (s, c) = tr.heading.to_radians().sin_cos();
            let spin = (tr.w / self.ppd).abs() / 60.0;
            let (bs, bc) = (self.spin_drift[0], self.spin_drift[1]);
            let sx = (bs * c - bc * s) * spin;
            let sy = (bs * s + bc * c) * spin;
            tr.pos[0] += (tr.v[0] * c - tr.v[1] * s + self.drift[0] + sx) * SUB;
            tr.pos[1] += (tr.v[0] * s + tr.v[1] * c + self.drift[1] + sy) * SUB;
            if tr.fly == 2 {
                tr.pos[2] = (tr.pos[2] + throttle / 100.0 * 0.8 * SUB).clamp(0.3, ROOM[2] - 0.3);
            }
            let gap = [tr.pos[0], ROOM[0] - tr.pos[0], tr.pos[1], ROOM[1] - tr.pos[1]].into_iter().fold(f64::MAX, f64::min);
            self.clearance = self.clearance.min(gap);
            if gap < 0.12 {
                self.crashed = true;
                for k in 0..2 {
                    tr.pos[k] = tr.pos[k].clamp(0.12, ROOM[k] - 0.12);
                }
            }
        }
    }

    pub fn camera(&mut self) -> Img {
        let tr = self.truth;
        let blur = (tr.w / self.ppd).abs() > 30.0;
        let mut img = render(tr.pos, tr.heading, self.ppd, self.plain, &mut self.rng);
        if blur {
            let back = render(tr.pos, tr.heading - tr.w / self.ppd * 0.02, self.ppd, self.plain, &mut self.rng);
            for (a, b) in img.p.iter_mut().zip(&back.p) {
                *a = (*a + b) * 0.5;
            }
        }
        img
    }
}

pub struct Flight {
    pub world: World,
    pub nav: Nav,
    sent: [f64; 4],
    queue: Vec<(f64, Img, f64)>,
    last: Option<Img>,
    frame_n: u64,
    next_frame: f64,
    stall_until: f64,
    drop_left: u32,
    pub trace: bool,
    trail: Vec<[f64; 4]>,
}

#[derive(Debug, Clone)]
pub struct Outcome {
    pub seed: u64,
    pub phase: Phase,
    pub error_m: f64,
    pub heading_err: f64,
    pub away_m: f64,
    pub clearance: f64,
    pub crashed: bool,
    pub secs: f64,
}

impl Flight {
    pub fn new(seed: u64) -> Flight {
        let mut nav = Nav::default();
        nav.reset();
        Flight { world: World::new(seed), nav, sent: [0.0; 4], queue: Vec::new(), last: None, frame_n: 0, next_frame: 0.0, stall_until: 0.0, drop_left: 0, trace: false, trail: Vec::new() }
    }

    pub fn tick(&mut self, target: [f64; 4]) {
        for k in 0..4 {
            self.sent[k] += (target[k] - self.sent[k]).clamp(-12.0, 12.0);
        }
        let w = &mut self.world;
        for _ in 0..7 {
            w.advance(self.sent, 10.0);
            if self.next_frame > w.t {
                continue;
            }
            let tc = self.next_frame;
            self.next_frame += 40.0;
            self.frame_n += 1;
            let fast = (w.truth.w / w.ppd).abs() > 45.0;
            let img = match (&self.last, fast && self.frame_n % 2 == 0) {
                (Some(prev), true) => prev.clone(),
                _ => w.camera(),
            };
            self.last = Some(img.clone());
            if self.drop_left > 0 {
                self.drop_left -= 1;
                continue;
            }
            if w.rng.f() < 0.004 {
                self.drop_left = w.rng.range(5.0, 35.0) as u32;
            }
            if w.rng.f() < 0.01 {
                self.stall_until = tc + w.rng.range(300.0, 900.0);
            }
            let arrive = (tc + 120.0 + w.rng.range(0.0, 80.0)).max(self.stall_until);
            self.queue.push((arrive, img, tc));
        }
        let now = w.t;
        let fly = w.truth.fly;
        self.nav.step(self.sent[0], self.sent[1], self.sent[2], self.sent[3], fly, 0.07, now);
        let (ready, wait): (Vec<_>, Vec<_>) = self.queue.drain(..).partition(|(a, _, _)| *a <= now);
        self.queue = wait;
        for (_, img, tc) in ready {
            self.nav.frame(img, tc);
        }
    }

    pub fn hold(&mut self, secs: f64, sticks: [f64; 4]) {
        for _ in 0..(secs / 0.07).round() as usize {
            self.tick(sticks);
        }
    }

    pub fn takeoff(&mut self) {
        self.hold(0.5, [0.0; 4]);
        self.world.takeoff();
        self.hold(4.5, [0.0; 4]);
    }

    pub fn pilot(&mut self, rng: &mut Rng, secs: f64) {
        let end = self.world.t + secs * 1000.0;
        while self.world.t < end {
            let (dur, sticks) = match (rng.f() * 5.0) as u32 {
                0 | 1 => (rng.range(1.0, 5.0), [0.0, 0.0, 0.0, mouse_stick(rng.range(-1.0, 1.0), self.nav.yaw_gain)]),
                2 => (rng.range(0.3, 0.9), [0.0, if rng.f() < 0.5 { 35.0 } else { -35.0 }, 0.0, 0.0]),
                3 => (rng.range(0.3, 0.8), [if rng.f() < 0.5 { 35.0 } else { -35.0 }, 0.0, 0.0, 0.0]),
                _ => (rng.range(0.5, 2.0), [0.0; 4]),
            };
            for _ in 0..(dur / 0.07).round() as usize {
                self.tick(sticks);
                if self.sent.iter().any(|v| *v != 0.0) {
                    self.trail.push(self.sent);
                }
            }
        }
        self.hold(1.0, [0.0; 4]);
    }

    pub fn go_home(&mut self) -> Phase {
        let mut h = Homing::new(self.world.t);
        let mut view = None;
        let mut seen = 0.0;
        while !matches!(h.phase, Phase::Land | Phase::Fail(_)) && self.world.t < 400_000.0 {
            if let Some((_, t)) = &self.nav.latest {
                if *t != seen {
                    seen = *t;
                    view = self.nav.view_home();
                }
            }
            let s = Sense { now: self.world.t, pose: self.nav.pose(), view };
            let sticks = h.tick(&s);
            if self.trace {
                let tr = self.world.truth;
                let v = view.map(|(v, t)| format!("dx {:7.2} zoom {:.3} conf {:.2} err {:.3} age {:4.0}", v.dx, v.zoom, v.conf, v.err, self.world.t - t)).unwrap_or_default();
                println!("{:7.0} {:?} true ({:.2},{:.2}) hdg {:7.2} nav hdg {:7.2} lock {:.2} | {v} | pitch {:6.1} yaw {:6.1}", self.world.t, h.phase, tr.pos[0] - HOME[0], tr.pos[1] - HOME[1], tr.heading, self.nav.heading(), self.nav.lock, sticks[1], sticks[3]);
            }
            self.tick(sticks);
        }
        if h.phase == Phase::Land {
            self.nav.at_home();
        }
        h.phase
    }

    pub fn old_retrace(&mut self) -> Phase {
        let trail = std::mem::take(&mut self.trail);
        for s in trail.iter().rev() {
            self.tick([-s[0], -s[1], 0.0, -s[3]]);
        }
        self.hold(0.6, [0.0; 4]);
        Phase::Land
    }

    pub fn error(&self) -> f64 {
        let p = self.world.truth.pos;
        (p[0] - HOME[0]).hypot(p[1] - HOME[1])
    }
}

pub fn mission(seed: u64, secs: f64, trace: bool) -> Option<Outcome> {
    let mut f = Flight::new(seed);
    f.trace = trace;
    let mut rng = Rng::new(seed ^ 0xA5A5);
    f.takeoff();
    f.pilot(&mut rng, secs);
    if f.world.clearance < 0.5 {
        return None;
    }
    let away = f.error();
    let start = f.world.t;
    let phase = if std::env::var("OLD").is_ok() { f.old_retrace() } else { f.go_home() };
    let h = (f.world.truth.heading + 180.0).rem_euclid(360.0) - 180.0;
    Some(Outcome {
        seed,
        phase,
        error_m: f.error(),
        heading_err: h,
        away_m: away,
        clearance: f.world.clearance,
        crashed: f.world.crashed,
        secs: (f.world.t - start) / 1000.0,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn tracker_heading_follows_true_heading_through_spins_and_gaps() {
        let (mut worst, mut both) = (0.0f64, 0.0f64);
        for seed in 1..=8 {
            let mut f = Flight::new(seed);
            let mut rng = Rng::new(seed);
            f.takeoff();
            for _ in 0..6 {
                let frac = rng.range(-1.0, 1.0);
                f.hold(rng.range(1.0, 4.0), [0.0, 0.0, 0.0, mouse_stick(frac, f.nav.yaw_gain)]);
                f.hold(1.0, [0.0; 4]);
            }
            let err = ((f.nav.heading() - f.world.truth.heading) + 180.0).rem_euclid(360.0) - 180.0;
            eprintln!("seed {seed}: true {:.1} tracked {:.1} err {err:.2} deg gain {:.2} (true {:.2}) excess {:?} seen {:?} plain {:.2} lock {:.2} slide right {:.2} m/s fwd {:.2}", f.world.truth.heading, f.nav.heading(), f.nav.yaw_gain, f.world.gain, f.nav.excess, f.nav.seen, f.world.plain, f.nav.lock, f.world.spin_drift[1], f.world.spin_drift[0]);
            worst = worst.max(err.abs());
            if f.nav.seen[0].min(f.nav.seen[1]) >= 20.0 {
                both = both.max(err.abs());
            }
        }
        assert!(both < 10.0 && worst < 25.0, "heading error {both} with turns both ways, {worst} overall");
    }

    #[test]
    fn takeoff_view_matches_only_from_near_the_takeoff_point() {
        let mut g = Rng::new(3);
        for plain in [1.0f32, 0.35] {
            let home = render([HOME[0], HOME[1], 1.0], 0.0, PX_PER_DEG, plain, &mut g);
            let mut worst_true = 0.0f32;
            for (dx, dy, h) in [(0.0, 0.0, 0.0), (0.3, 0.0, 0.0), (-0.4, 0.0, 0.0), (0.0, 0.3, 0.0), (0.0, 0.0, 15.0), (0.2, -0.2, -25.0), (-0.6, 0.2, 10.0), (1.0, 0.0, 0.0), (-1.0, 0.0, 0.0), (0.0, 0.8, 0.0), (0.0, -1.2, 0.0), (-0.8, 0.8, 0.0), (0.7, -0.7, 5.0)] {
                let cur = render([HOME[0] + dx, HOME[1] + dy, 1.0], h, PX_PER_DEG, plain, &mut g);
                let v = crate::odo::compare(&home, &cur);
                eprintln!("plain {plain}: true view off ({dx},{dy}) {h} deg -> dx {:.1} zoom {:.3} conf {:.2} err {:.3}", v.dx, v.zoom, v.conf, v.err);
                worst_true = worst_true.max(v.err);
            }
            let mut best_false = f32::MAX;
            for h in [70.0, 90.0, 135.0, 180.0, 225.0, 270.0, 300.0] {
                let cur = render([HOME[0], HOME[1], 1.0], h, PX_PER_DEG, plain, &mut g);
                let v = crate::odo::compare(&home, &cur);
                eprintln!("plain {plain}: wrong wall {h} deg -> err {:.3} conf {:.2}", v.err, v.conf);
                best_false = best_false.min(v.err);
            }
            assert!(worst_true < crate::home::VIEW_ERR && best_false > crate::odo::MATCH_ERR, "true {worst_true} false {best_false}");
        }
    }

    #[test]
    fn slide_while_turning_is_learned_with_the_right_sign() {
        for right in [0.0, 0.2, -0.2] {
            let mut f = Flight::new(5);
            (f.world.spin_drift, f.world.drift, f.world.gain) = ([0.0, right], [0.0; 2], 1.0);
            f.takeoff();
            for k in 0..8 {
                let dir = if k % 2 == 0 { 0.8 } else { -0.8 };
                f.hold(2.0, [0.0, 0.0, 0.0, mouse_stick(dir, f.nav.yaw_gain)]);
                f.hold(0.8, [0.0; 4]);
            }
            let err = ((f.nav.heading() - f.world.truth.heading) + 180.0).rem_euclid(360.0) - 180.0;
            let slide = (f.nav.excess[1] - f.nav.excess[0]) / 2.0;
            eprintln!("slide right {right}: learned slide {slide:+.3} gain {:.3} heading err {err:+.2}", f.nav.yaw_gain);
            assert!(err.abs() < 5.0 && slide * right >= 0.0, "slide {right}: err {err} learned {slide}");
        }
    }

    #[test]
    fn mouse_turn_rate_matches_the_mouse_across_the_full_range() {
        let mut f = Flight::new(7);
        f.takeoff();
        f.hold(4.0, [0.0, 0.0, 0.0, mouse_stick(0.5, f.nav.yaw_gain)]);
        for frac in [0.2, 0.4, 0.6, 0.8, 1.0] {
            f.hold(2.0, [0.0, 0.0, 0.0, mouse_stick(frac, f.nav.yaw_gain)]);
            let rate = f.world.truth.w;
            eprintln!("mouse {frac:.1}: {rate:.1} px/s wanted {:.1}", frac * crate::home::MOUSE_RATE);
            assert!((rate / (frac * crate::home::MOUSE_RATE) - 1.0).abs() < 0.1, "{frac}: {rate}");
        }
    }

    #[test]
    fn return_home_lands_over_the_takeoff_point_without_touching_a_wall() {
        let ran: Vec<Outcome> = std::thread::scope(|s| {
            let jobs: Vec<_> = (125..141).map(|seed| s.spawn(move || mission(seed, 20.0, false))).collect();
            jobs.into_iter().filter_map(|j| j.join().ok().flatten()).collect()
        });
        for o in &ran {
            eprintln!("{o:?}");
        }
        assert!(ran.len() >= 12 && ran.iter().all(|o| !o.crashed), "crashed");
        assert!(ran.iter().all(|o| matches!(o.phase, Phase::Land | Phase::Fail(_))));
        let mut misses: Vec<f64> = ran.iter().filter(|o| o.phase == Phase::Land).map(|o| o.error_m).collect();
        misses.sort_by(|a, b| a.total_cmp(b));
        assert!(misses.len() * 10 >= ran.len() * 7, "only {} of {} landed", misses.len(), ran.len());
        assert!(misses[misses.len() / 2] < 0.45, "median miss {:?}", misses);
    }
}
