use crate::nav::{yaw_stick, Pose, PX_PER_DEG};
use crate::odo::{estimate, Img, View, MIN_TEXTURE};

pub const VIEW_CONF: f32 = 0.5;
pub const VIEW_ERR: f32 = 0.45;
pub const VIEW_HEADING: f64 = 50.0;
pub const FRESH_MS: f64 = 700.0;
pub const TURN_RATE: f64 = 110.0;
pub const MOUSE_RATE: f64 = 150.0;
pub const DONE_ZOOM: f64 = 0.03;
pub const DONE_LAT: f64 = 8.0;
pub const DONE_HEADING: f64 = 4.0;

#[derive(Clone, Debug, PartialEq)]
pub enum Phase {
    Settle,
    Turn,
    Seek,
    Approach,
    Land,
    Fail(String),
}

pub struct Homing {
    pub phase: Phase,
    start: f64,
    phase_at: f64,
    good_since: Option<f64>,
    aligned_since: Option<f64>,
    last_view: f64,
    last_zoom: f64,
}

pub struct Sense {
    pub now: f64,
    pub pose: Pose,
    pub view: Option<(View, f64)>,
}

fn wrap180(a: f64) -> f64 {
    (a + 180.0).rem_euclid(360.0) - 180.0
}

pub fn turn_stick(rate: f64, gain: f64) -> f64 {
    yaw_stick(rate / gain.max(0.3))
}

pub fn mouse_stick(frac: f64, gain: f64) -> f64 {
    turn_stick(frac.clamp(-1.0, 1.0) * MOUSE_RATE, gain)
}

fn face_home(heading: f64, gain: f64) -> f64 {
    let err = -wrap180(heading);
    if err.abs() < 1.0 {
        return 0.0;
    }
    err.signum() * turn_stick((err.abs() * PX_PER_DEG * 1.5).clamp(25.0, TURN_RATE), gain)
}

impl Homing {
    pub fn new(now: f64) -> Homing {
        Homing { phase: Phase::Settle, start: now, phase_at: now, good_since: None, aligned_since: None, last_view: now, last_zoom: 1.0 }
    }

    fn go(&mut self, p: Phase, now: f64) {
        self.phase = p;
        self.phase_at = now;
        self.good_since = None;
        self.aligned_since = None;
        self.last_view = now;
    }

    pub fn status(&self) -> String {
        match &self.phase {
            Phase::Settle => "holding still to calibrate".into(),
            Phase::Turn => "turning back to the takeoff direction".into(),
            Phase::Seek => "creeping toward the takeoff point until the takeoff view is recognised".into(),
            Phase::Approach => format!("lining up with the takeoff view (zoom {:+.1}%)", (self.last_zoom - 1.0) * 100.0),
            Phase::Land => "over the takeoff point - landing".into(),
            Phase::Fail(why) => why.clone(),
        }
    }

    fn fail(&mut self, why: &str, now: f64) -> [f64; 4] {
        self.go(Phase::Fail(format!("{why} - hovering, you have control")), now);
        [0.0; 4]
    }

    pub fn tick(&mut self, s: &Sense) -> [f64; 4] {
        let (now, el) = (s.now, s.now - self.phase_at);
        let h = wrap180(s.pose.heading);
        let facing = h.abs() <= VIEW_HEADING;
        let good = s.view.filter(|(v, t)| facing && v.conf >= VIEW_CONF && v.err <= VIEW_ERR && now - t <= FRESH_MS).map(|(v, _)| v);
        if now - self.start > 60_000.0 && !matches!(self.phase, Phase::Land | Phase::Fail(_)) {
            return self.fail("return home took over 60s", now);
        }
        match self.phase {
            Phase::Settle => {
                let calm = s.pose.steady < 1.0 && s.pose.lock > 0.5;
                if el >= 1000.0 && (calm || el >= 4000.0) {
                    if !s.pose.home_set {
                        return self.fail("no takeoff view was captured", now);
                    }
                    self.go(Phase::Turn, now);
                }
                [0.0; 4]
            }
            Phase::Turn => {
                if good.is_some() {
                    self.go(Phase::Approach, now);
                    return [0.0; 4];
                }
                if h.abs() <= DONE_HEADING {
                    if now - *self.aligned_since.get_or_insert(now) > 1000.0 {
                        self.go(Phase::Seek, now);
                    }
                    return [0.0; 4];
                }
                self.aligned_since = None;
                if el > 20_000.0 {
                    return self.fail("could not turn back to the takeoff direction", now);
                }
                [0.0, 0.0, 0.0, face_home(h, s.pose.yaw_gain)]
            }
            Phase::Seek => {
                if good.is_some() {
                    self.go(Phase::Approach, now);
                    return [0.0; 4];
                }
                let (x, y) = (s.pose.x, s.pose.y);
                if x.hypot(y) < 0.15 || el > 8000.0 {
                    if now - *self.aligned_since.get_or_insert(now) > 2000.0 {
                        return self.fail("the takeoff view is not visible from here", now);
                    }
                    return [0.0, 0.0, 0.0, face_home(h, s.pose.yaw_gain)];
                }
                let (sn, cs) = h.to_radians().sin_cos();
                let (fwd, right) = (-x * cs - y * sn, x * sn - y * cs);
                [(right * 40.0).clamp(-12.0, 12.0), (fwd * 40.0).clamp(-12.0, 12.0), 0.0, face_home(h, s.pose.yaw_gain)]
            }
            Phase::Approach => {
                let Some(v) = good else {
                    if now - self.last_view > 4000.0 {
                        return self.fail("lost sight of the takeoff view", now);
                    }
                    return [0.0, 0.0, 0.0, face_home(h, s.pose.yaw_gain)];
                };
                self.last_view = now;
                self.last_zoom = v.zoom as f64;
                let lz = (v.zoom as f64).ln();
                let lat = v.dx as f64 + h * PX_PER_DEG;
                let mut roll = if lat.abs() > 3.0 { (0.25 * lat).clamp(-14.0, 14.0) } else { 0.0 };
                let mut pitch = (-150.0 * lz).clamp(-18.0, 18.0);
                if h.abs() > 15.0 {
                    (roll, pitch) = (roll * 0.3, pitch * 0.3);
                }
                if lz.abs() < DONE_ZOOM && lat.abs() < DONE_LAT && h.abs() < DONE_HEADING {
                    if now - *self.good_since.get_or_insert(now) >= 1000.0 {
                        self.go(Phase::Land, now);
                        return [0.0; 4];
                    }
                } else {
                    self.good_since = None;
                }
                if el > 30_000.0 {
                    if lz.abs() < 0.08 && lat.abs() < 20.0 {
                        self.go(Phase::Land, now);
                        return [0.0; 4];
                    }
                    return self.fail("could not settle over the takeoff point", now);
                }
                [roll, pitch, 0.0, face_home(h, s.pose.yaw_gain)]
            }
            Phase::Land | Phase::Fail(_) => [0.0; 4],
        }
    }
}

pub struct Hold {
    reference: Option<Img>,
    neutral_since: Option<f64>,
    pub active: bool,
}

impl Hold {
    pub fn new() -> Hold {
        Hold { reference: None, neutral_since: None, active: false }
    }

    pub fn tick(&mut self, now: f64, neutral: bool, latest: Option<(&Img, f64)>) -> [f64; 2] {
        self.active = false;
        let Some((img, t)) = latest.filter(|(i, t)| neutral && now - t <= FRESH_MS && i.sd >= MIN_TEXTURE) else {
            (self.reference, self.neutral_since) = (None, None);
            return [0.0; 2];
        };
        if now - *self.neutral_since.get_or_insert(now) < 800.0 {
            return [0.0; 2];
        }
        let Some(r) = self.reference.as_ref() else {
            self.reference = Some(img.clone());
            return [0.0; 2];
        };
        let m = estimate(r, img);
        if m.conf < 0.08 || m.dx.abs() > 60.0 || m.scale.abs() > 0.08 {
            self.reference = Some(img.clone());
            return [0.0; 2];
        }
        let _ = t;
        self.active = true;
        let yaw = if m.dx.abs() > 2.0 { (0.35 * m.dx as f64).clamp(-10.0, 10.0) } else { 0.0 };
        let pitch = if m.scale.abs() > 0.008 { (-160.0 * m.scale as f64).clamp(-10.0, 10.0) } else { 0.0 };
        [pitch, yaw]
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::nav::yaw_rate;

    #[test]

    fn mouse_rate_is_linear_in_mouse_position() {
        for f in [0.1, 0.25, 0.5, 0.75, 1.0] {
            let r = yaw_rate(mouse_stick(f, 1.0));
            assert!((r - f * MOUSE_RATE).abs() < 1e-6, "{f} -> {r}");
        }
        let weak = yaw_rate(mouse_stick(0.5, 0.8)) * 0.8;
        assert!((weak - 0.5 * MOUSE_RATE).abs() < 1e-6);
    }
}
