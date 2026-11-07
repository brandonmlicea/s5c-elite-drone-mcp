use crate::odo::{compare, difference, estimate, Img, View, MIN_TEXTURE, REPEAT};
use std::collections::VecDeque;

pub const MAX_SPEED: f64 = 1.2;
pub const MAX_CLIMB: f64 = 0.8;
pub const LAG: f64 = 0.35;
pub const TAKEOFF_HEIGHT: f64 = 1.0;
pub const YAW_A: f64 = 378.1;
pub const YAW_B: f64 = 68.9;
pub const YAW_TAU: f64 = 0.20;
pub const SLIDE: f64 = 0.12;
pub const PX_PER_DEG: f64 = 3.2;
pub const SNAPSHOT_AFTER_MS: f64 = 1200.0;

pub fn yaw_rate(stick: f64) -> f64 {
    let s = (stick / 100.0).clamp(-1.0, 1.0);
    YAW_A * s - YAW_B * s * s.abs()
}

pub fn yaw_stick(rate: f64) -> f64 {
    let r = rate.abs().min(YAW_A - YAW_B);
    let s = (YAW_A - (YAW_A * YAW_A - 4.0 * YAW_B * r).max(0.0).sqrt()) / (2.0 * YAW_B);
    rate.signum() * s * 100.0
}

#[derive(Clone, Copy, Default, Debug)]
pub struct Pose {
    pub x: f64,
    pub y: f64,
    pub z: f64,
    pub heading: f64,
    pub lock: f64,
    pub steady: f64,
    pub yaw_gain: f64,
    pub home_set: bool,
}

#[derive(Clone)]
pub struct Nav {
    pub x: f64,
    pub y: f64,
    pub z: f64,
    pub vf: f64,
    pub vr: f64,
    pub vz: f64,
    pub w: f64,
    pub head_px: f64,
    pub yaw_gain: f64,
    pub excess: [f64; 2],
    pub swept: f64,
    pub seen: [f64; 2],
    pub px_per_deg: f64,
    pub lock: f64,
    pub steady: f64,
    pub armed: bool,
    pub home: Option<Img>,
    pub latest: Option<(Img, f64)>,
    model_px: f64,
    hist: VecDeque<(f64, f64)>,
    flying_since: Option<f64>,
    now: f64,
}

impl Default for Nav {
    fn default() -> Nav {
        Nav {
            x: 0.0,
            y: 0.0,
            z: 0.0,
            vf: 0.0,
            vr: 0.0,
            vz: 0.0,
            w: 0.0,
            head_px: 0.0,
            yaw_gain: 1.0,
            excess: [-SLIDE, SLIDE],
            swept: 0.0,
            seen: [0.0; 2],
            px_per_deg: PX_PER_DEG,
            lock: 0.0,
            steady: 0.0,
            armed: false,
            home: None,
            latest: None,
            model_px: 0.0,
            hist: VecDeque::new(),
            flying_since: None,
            now: 0.0,
        }
    }
}

impl Nav {
    pub fn reset(&mut self) {
        let (gain, excess, seen, ppd, latest) = (self.yaw_gain, self.excess, self.seen, self.px_per_deg, self.latest.take());
        *self = Nav { armed: true, yaw_gain: gain, excess, seen, px_per_deg: ppd, latest, ..Nav::default() };
    }

    pub fn load(&mut self, path: &std::path::Path) {
        let v: Vec<f64> = std::fs::read_to_string(path).unwrap_or_default().split_whitespace().filter_map(|s| s.parse().ok()).collect();
        if let [gain, left, right, n_left, n_right] = v[..] {
            (self.yaw_gain, self.excess, self.seen) = (gain.clamp(0.5, 1.8), [left, right], [n_left.min(40.0), n_right.min(40.0)]);
        }
    }

    pub fn save(&self, path: &std::path::Path) {
        let _ = std::fs::write(path, format!("{} {} {} {} {}", self.yaw_gain, self.excess[0], self.excess[1], self.seen[0], self.seen[1]));
    }

    pub fn slide(&self) -> f64 {
        let sure = (self.seen[0].min(self.seen[1]) / 20.0).min(1.0);
        (self.excess[1] - self.excess[0]) / 2.0 * sure
    }

    fn heading_px(&self) -> f64 {
        self.head_px - self.slide() * self.swept
    }

    pub fn heading(&self) -> f64 {
        (self.heading_px() / self.px_per_deg).rem_euclid(360.0)
    }

    pub fn pose(&self) -> Pose {
        Pose {
            x: self.x,
            y: self.y,
            z: self.z,
            heading: self.heading(),
            lock: self.lock,
            steady: self.steady,
            yaw_gain: self.yaw_gain,
            home_set: self.home.is_some(),
        }
    }

    pub fn step(&mut self, roll: f64, pitch: f64, throttle: f64, yaw: f64, fly: u8, dt: f64, now: f64) {
        self.now = now;
        if !self.armed {
            return;
        }
        if fly == 0 {
            (self.vf, self.vr, self.vz, self.w, self.z) = (0.0, 0.0, 0.0, 0.0, 0.0);
            self.flying_since = None;
        } else {
            let k = 1.0 - (-dt / LAG).exp();
            self.vf += (pitch / 100.0 * MAX_SPEED - self.vf) * k;
            self.vr += (roll / 100.0 * MAX_SPEED - self.vr) * k;
            let climb = if fly == 1 { TAKEOFF_HEIGHT / 2.0 } else { throttle / 100.0 * MAX_CLIMB };
            self.vz += (climb - self.vz) * k;
            self.w += (self.yaw_gain * yaw_rate(yaw) - self.w) * (1.0 - (-dt / YAW_TAU).exp());
            self.model_px += self.w * dt;
            self.head_px += self.w * dt;
            let (s, c) = self.heading().to_radians().sin_cos();
            self.x += (self.vf * c - self.vr * s) * dt;
            self.y += (self.vf * s + self.vr * c) * dt;
            self.z = (self.z + self.vz * dt).max(0.0);
            if fly == 2 {
                self.flying_since.get_or_insert(now);
            }
        }
        self.hist.push_back((now, self.model_px));
        while self.hist.len() > 2 && now - self.hist[0].0 > 6000.0 {
            self.hist.pop_front();
        }
    }

    fn model_at(&self, t: f64) -> f64 {
        let Some(first) = self.hist.front() else { return self.model_px };
        if t <= first.0 {
            return first.1;
        }
        for w in self.hist.iter().zip(self.hist.iter().skip(1)) {
            let ((t0, m0), (t1, m1)) = (*w.0, *w.1);
            if t <= t1 {
                return m0 + (m1 - m0) * ((t - t0) / (t1 - t0).max(1e-6));
            }
        }
        self.model_px + self.w * ((t - self.now) / 1000.0).min(0.0)
    }

    pub fn frame(&mut self, img: Img, t: f64) {
        if let Some((prev, pt)) = &self.latest {
            if difference(prev, &img) < REPEAT {
                return;
            }
            let span = t - pt;
            if self.armed && span > 0.0 && span < 400.0 {
                let m = estimate(prev, &img);
                let pred = self.model_at(t) - self.model_at(*pt);
                let meas = -m.dx as f64;
                let still = meas.abs() < 0.5 && pred.abs() < 0.5;
                let usable = m.conf > 0.03 && img.sd >= MIN_TEXTURE && (span <= 130.0 || still) && (meas - pred).abs() < 20.0 + pred.abs();
                let turning = pred.abs() > 15.0 * span / 1000.0;
                if usable && turning {
                    self.head_px += meas - pred;
                    self.swept += pred.abs();
                    if pred.abs() > 1.5 && meas * pred > 0.0 {
                        let side = (pred > 0.0) as usize;
                        let rate = (1.0 / (self.seen[side] + 8.0)).max(0.03);
                        self.seen[side] += 1.0;
                        self.excess[side] += ((meas / pred - 1.0).clamp(-0.6, 0.6) - self.excess[side]) * rate;
                        let f = 1.0 + 0.04 * ((self.excess[0] + self.excess[1]) / 2.0).clamp(-0.5, 0.5);
                        let before = self.yaw_gain;
                        self.yaw_gain = (self.yaw_gain * f).clamp(0.5, 1.8);
                        let applied = self.yaw_gain / before - 1.0;
                        self.excess.iter_mut().for_each(|e| *e -= applied);
                    }
                }
                self.lock += ((usable as u8 as f64) - self.lock) * 0.25;
                self.steady += (meas.abs() * 40.0 / span - self.steady) * 0.3;
            }
        }
        let settled = self.flying_since.is_some_and(|s| self.now - s > SNAPSHOT_AFTER_MS);
        if self.armed && self.home.is_none() && settled && self.lock > 0.6 && self.steady < 1.0 && img.sd >= MIN_TEXTURE {
            self.home = Some(img.clone());
            (self.head_px, self.swept) = (0.0, 0.0);
        }
        self.latest = Some((img, t));
    }

    pub fn view_home(&self) -> Option<(View, f64)> {
        let (home, (cur, t)) = (self.home.as_ref()?, self.latest.as_ref()?);
        Some((compare(home, cur), *t))
    }

    pub fn at_home(&mut self) {
        (self.x, self.y) = (0.0, 0.0);
        (self.head_px, self.swept) = (self.heading_px(), 0.0);
        self.head_px = self.head_px.rem_euclid(360.0 * self.px_per_deg);
        if self.head_px > 180.0 * self.px_per_deg {
            self.head_px -= 360.0 * self.px_per_deg;
        }
    }

    pub fn distance(&self) -> f64 {
        self.x.hypot(self.y)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const DT: f64 = 0.07;

    fn fly(n: &mut Nav, t: &mut f64, secs: f64, roll: f64, pitch: f64, yaw: f64) {
        for _ in 0..(secs / DT) as usize {
            *t += DT * 1000.0;
            n.step(roll, pitch, 0.0, yaw, 2, DT, *t);
        }
    }

    #[test]
    fn stick_model_inverts() {
        for s in [-100.0, -60.0, -12.0, 0.0, 5.0, 35.0, 70.0, 100.0] {
            assert!((yaw_stick(yaw_rate(s)) - s).abs() < 1e-6, "{s}");
        }
    }

    #[test]
    fn starts_at_origin_and_ignores_ground_time() {
        let mut n = Nav::default();
        n.reset();
        for i in 0..50 {
            n.step(50.0, 50.0, 50.0, 0.0, 0, DT, i as f64 * 70.0);
        }
        assert_eq!((n.x, n.y, n.z), (0.0, 0.0, 0.0));
    }

    #[test]
    fn forward_then_turn_around_and_back_returns_near_origin() {
        let (mut n, mut t) = (Nav::default(), 0.0);
        n.reset();
        fly(&mut n, &mut t, 3.0, 0.0, 40.0, 0.0);
        fly(&mut n, &mut t, 1.5, 0.0, 0.0, 0.0);
        assert!(n.x > 1.0 && n.y.abs() < 0.01, "{:?}", n.pose());
        let s = yaw_stick(180.0 * PX_PER_DEG / 3.0);
        fly(&mut n, &mut t, 3.0, 0.0, 0.0, s);
        fly(&mut n, &mut t, 1.0, 0.0, 0.0, 0.0);
        let h = n.heading();
        assert!((h - 180.0).abs() < 25.0, "{h}");
    }
}
