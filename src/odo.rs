use crate::video::Frame;

pub const TW: usize = 320;
pub const TH: usize = 180;
pub const REPEAT: f32 = 0.0015;
const LEVELS: usize = 4;
const COARSE_R: i32 = 9;
const COLS: usize = 5;
const ROWS: usize = 3;

#[derive(Clone)]
pub struct Img {
    pub w: usize,
    pub h: usize,
    pub p: Vec<f32>,
    pub sd: f32,
}

impl Img {
    pub fn new(w: usize, h: usize, p: Vec<f32>) -> Img {
        let n = p.len() as f32;
        let mean = p.iter().sum::<f32>() / n;
        let raw = (p.iter().map(|v| (v - mean).powi(2)).sum::<f32>() / n).sqrt();
        let sd = raw.max(1.0);
        Img { w, h, p: p.iter().map(|v| (v - mean) / sd).collect(), sd: raw }
    }

    pub fn zoomed(&self, z: f32) -> Img {
        let (cx, cy) = (self.w as f32 / 2.0, self.h as f32 / 2.0);
        let mut p = Vec::with_capacity(self.p.len());
        for y in 0..self.h {
            for x in 0..self.w {
                let sx = (cx + (x as f32 + 0.5 - cx) / z - 0.5).clamp(0.0, self.w as f32 - 1.001);
                let sy = (cy + (y as f32 + 0.5 - cy) / z - 0.5).clamp(0.0, self.h as f32 - 1.001);
                let (x0, y0) = (sx as usize, sy as usize);
                let (fx, fy) = (sx - x0 as f32, sy - y0 as f32);
                let at = |xx: usize, yy: usize| self.p[yy * self.w + xx];
                let top = at(x0, y0) + (at(x0 + 1, y0) - at(x0, y0)) * fx;
                let bot = at(x0, y0 + 1) + (at(x0 + 1, y0 + 1) - at(x0, y0 + 1)) * fx;
                p.push(top + (bot - top) * fy);
            }
        }
        Img { w: self.w, h: self.h, p, sd: self.sd }
    }

    pub fn from_luma(w: usize, h: usize, y: &[u8]) -> Img {
        let (sx, sy) = (w / TW, h / TH);
        let mut p = vec![0f32; TW * TH];
        for ty in 0..TH {
            for yy in 0..sy {
                let row = &y[(ty * sy + yy) * w..];
                for tx in 0..TW {
                    p[ty * TW + tx] += row[tx * sx..tx * sx + sx].iter().map(|v| *v as u32).sum::<u32>() as f32;
                }
            }
        }
        let k = 1.0 / (sx * sy) as f32;
        Img::new(TW, TH, p.into_iter().map(|v| v * k).collect())
    }

    pub fn from_frame(f: &Frame) -> Img {
        Img::from_luma(f.w, f.h, &f.y)
    }

    fn half(&self) -> Img {
        let (w, h) = (self.w / 2, self.h / 2);
        let mut p = Vec::with_capacity(w * h);
        for y in 0..h {
            for x in 0..w {
                let i = 2 * y * self.w + 2 * x;
                p.push((self.p[i] + self.p[i + 1] + self.p[i + self.w] + self.p[i + self.w + 1]) * 0.25);
            }
        }
        Img { w, h, p, sd: self.sd }
    }

    pub fn pyramid(&self) -> Vec<Img> {
        let mut v = vec![self.clone()];
        for _ in 1..LEVELS {
            let next = v.last().unwrap().half();
            v.push(next);
        }
        v
    }
}

#[derive(Clone, Copy, Debug, Default)]
pub struct Motion {
    pub dx: f32,
    pub dy: f32,
    pub scale: f32,
    pub roll: f32,
    pub conf: f32,
}

struct Rect {
    x0: i32,
    y0: i32,
    x1: i32,
    y1: i32,
}

fn cost(a: &Img, b: &Img, r: &Rect, dx: i32, dy: i32) -> f32 {
    let x0 = r.x0.max(-dx).max(0);
    let x1 = r.x1.min(b.w as i32 - dx).min(a.w as i32);
    let y0 = r.y0.max(-dy).max(0);
    let y1 = r.y1.min(b.h as i32 - dy).min(a.h as i32);
    if x1 - x0 < 4 || y1 - y0 < 4 {
        return f32::MAX;
    }
    let mut s = 0.0;
    for y in y0..y1 {
        let ra = &a.p[y as usize * a.w..];
        let rb = &b.p[(y + dy) as usize * b.w..];
        for x in x0..x1 {
            s += (ra[x as usize] - rb[(x + dx) as usize]).abs();
        }
    }
    s / ((x1 - x0) * (y1 - y0)) as f32
}

fn vertex(m: f32, c: f32, p: f32) -> f32 {
    let d = m - 2.0 * c + p;
    if d <= 1e-6 {
        0.0
    } else {
        (0.5 * (m - p) / d).clamp(-0.5, 0.5)
    }
}

fn search(a: &Img, b: &Img, r: &Rect, cx: i32, cy: i32, rad: i32) -> (f32, f32, f32) {
    let (mut best, mut bx, mut by) = (f32::MAX, cx, cy);
    for dy in cy - rad..=cy + rad {
        for dx in cx - rad..=cx + rad {
            let c = cost(a, b, r, dx, dy);
            if c < best {
                (best, bx, by) = (c, dx, dy);
            }
        }
    }
    if best == f32::MAX {
        return (cx as f32, cy as f32, 0.0);
    }
    let around = [(1, 0), (-1, 0), (0, 1), (0, -1)].map(|(ox, oy)| cost(a, b, r, bx + ox, by + oy));
    let fx = vertex(around[1], best, around[0]);
    let fy = vertex(around[3], best, around[2]);
    let ring = around.iter().filter(|c| **c < f32::MAX).sum::<f32>() / 4.0;
    let conf = ((ring - best) / ring.max(1e-3)).clamp(0.0, 1.0);
    (bx as f32 + fx, by as f32 + fy, conf)
}

pub fn difference(a: &Img, b: &Img) -> f32 {
    a.p.iter().zip(&b.p).map(|(x, y)| (x - y).abs()).sum::<f32>() / a.p.len() as f32
}

pub type Patch = (f32, f32, f32, f32, f32);

fn patches(pa: &[Img], pb: &[Img], gx: f32, gy: f32) -> Vec<Patch> {
    let mut pts = Vec::new();
    let (a1, b1, a0, b0) = (&pa[1], &pb[1], &pa[0], &pb[0]);
    let (pw, ph) = (a0.w / COLS, a0.h / ROWS);
    for row in 0..ROWS {
        for col in 0..COLS {
            let r1 = Rect { x0: (col * pw / 2) as i32, y0: (row * ph / 2) as i32, x1: ((col + 1) * pw / 2) as i32, y1: ((row + 1) * ph / 2) as i32 };
            let (x1, y1, _) = search(a1, b1, &r1, (gx / 2.0).round() as i32, (gy / 2.0).round() as i32, 7);
            let r0 = Rect { x0: (col * pw) as i32, y0: (row * ph) as i32, x1: ((col + 1) * pw) as i32, y1: ((row + 1) * ph) as i32 };
            let (x, y, c) = search(a0, b0, &r0, (x1 * 2.0).round() as i32, (y1 * 2.0).round() as i32, 2);
            if c > 0.05 {
                let px = (col as f32 + 0.5) * pw as f32 - a0.w as f32 / 2.0;
                let py = (row as f32 + 0.5) * ph as f32 - a0.h as f32 / 2.0;
                pts.push((px, py, x, y, c));
            }
        }
    }
    pts
}

pub fn patch_motion(prev: &Img, cur: &Img) -> Vec<Patch> {
    let (pa, pb) = (prev.pyramid(), cur.pyramid());
    let g = estimate(prev, cur);
    patches(&pa, &pb, g.dx, g.dy)
}

pub fn estimate(prev: &Img, cur: &Img) -> Motion {
    let (pa, pb) = (prev.pyramid(), cur.pyramid());
    let (mut gx, mut gy) = (0f32, 0f32);
    let mut conf = 0.0;
    for l in (1..LEVELS).rev() {
        let (a, b) = (&pa[l], &pb[l]);
        let m = if l == LEVELS - 1 { COARSE_R } else { 2 };
        let r = Rect { x0: 0, y0: 0, x1: a.w as i32, y1: a.h as i32 };
        let (cx, cy) = (gx.round() as i32, gy.round() as i32);
        let (x, y, c) = search(a, b, &r, cx, cy, m);
        (gx, gy, conf) = (x * 2.0, y * 2.0, c);
    }
    let pts = patches(&pa, &pb, gx, gy);
    let fit = |pts: &[Patch]| {
        let w: f32 = pts.iter().map(|p| p.4).sum();
        let (mx, my) = (pts.iter().map(|p| p.2 * p.4).sum::<f32>() / w, pts.iter().map(|p| p.3 * p.4).sum::<f32>() / w);
        let (cx, cy) = (pts.iter().map(|p| p.0 * p.4).sum::<f32>() / w, pts.iter().map(|p| p.1 * p.4).sum::<f32>() / w);
        let (mut sn, mut rn, mut d) = (0.0, 0.0, 0.0);
        for (px, py, x, y, c) in pts {
            let (qx, qy, ex, ey) = (px - cx, py - cy, x - mx, y - my);
            sn += c * (ex * qx + ey * qy);
            rn += c * (ey * qx - ex * qy);
            d += c * (qx * qx + qy * qy);
        }
        let (s, r) = if d > 1.0 { (sn / d, rn / d) } else { (0.0, 0.0) };
        (mx - s * cx + r * cy, my - s * cy - r * cx, s, r)
    };
    if pts.len() < 4 {
        return Motion { dx: gx, dy: gy, scale: 0.0, roll: 0.0, conf: conf * 0.5 };
    }
    let mut pts = pts;
    let mut model = fit(&pts);
    for _ in 0..2 {
        let res: Vec<f32> = pts
            .iter()
            .map(|(px, py, x, y, _)| {
                let (tx, ty, s, r) = model;
                ((tx + s * px - r * py - x).powi(2) + (ty + s * py + r * px - y).powi(2)).sqrt()
            })
            .collect();
        let mut sorted = res.clone();
        sorted.sort_by(|a, b| a.total_cmp(b));
        let lim = (sorted[sorted.len() / 2] * 2.5).max(0.35);
        let kept: Vec<_> = pts.iter().zip(&res).filter(|(_, r)| **r <= lim).map(|(p, _)| *p).collect();
        if kept.len() < 4 || kept.len() == pts.len() {
            break;
        }
        pts = kept;
        model = fit(&pts);
    }
    let (tx, ty, s, r) = model;
    let agree = pts.len() as f32 / (COLS * ROWS) as f32;
    Motion { dx: tx, dy: ty, scale: s, roll: r, conf: conf.max(0.2) * agree }
}

#[derive(Clone, Copy, Debug, Default)]
pub struct View {
    pub dx: f32,
    pub dy: f32,
    pub zoom: f32,
    pub conf: f32,
    pub err: f32,
}

pub const MIN_TEXTURE: f32 = 5.0;
pub const MATCH_ERR: f32 = 0.32;

pub fn compare(home: &Img, cur: &Img) -> View {
    if home.sd < MIN_TEXTURE || cur.sd < MIN_TEXTURE {
        return View { zoom: 1.0, err: f32::MAX, ..View::default() };
    }
    let (hp, cp) = (home.pyramid(), cur.pyramid());
    let (h2, c2) = (&hp[2], &cp[2]);
    let r = Rect { x0: 6, y0: 4, x1: h2.w as i32 - 6, y1: h2.h as i32 - 4 };
    let (mut best, mut all) = ((f32::MAX, 1.0f32, 0, 0), Vec::new());
    let mut z = 0.7f32;
    while z < 1.5 {
        let hz = h2.zoomed(z);
        for dy in -6..=6 {
            for dx in -30..=30 {
                let c = cost(&hz, c2, &r, dx, dy);
                if c < f32::MAX {
                    all.push(c);
                    if c < best.0 {
                        best = (c, z, dx, dy);
                    }
                }
            }
        }
        z *= 1.06;
    }
    if all.is_empty() {
        return View { zoom: 1.0, err: f32::MAX, ..View::default() };
    }
    all.sort_by(|a, b| a.total_cmp(b));
    let median = all[all.len() / 2];
    let (c, mut z, bx, by) = best;
    let mut fine = (f32::MAX, z, 0.0, 0.0);
    for k in [0.97, 0.985, 1.0, 1.015, 1.03] {
        let h1 = hp[1].zoomed(z * k);
        let r1 = Rect { x0: 12, y0: 8, x1: h1.w as i32 - 12, y1: h1.h as i32 - 8 };
        let (x, y, _) = search(&h1, &cp[1], &r1, bx * 2, by * 2, 2);
        let cc = cost(&h1, &cp[1], &r1, x.round() as i32, y.round() as i32);
        if cc < fine.0 {
            fine = (cc, z * k, x, y);
        }
    }
    z = fine.1;
    let h0 = home.zoomed(z);
    let r0 = Rect { x0: 24, y0: 16, x1: h0.w as i32 - 24, y1: h0.h as i32 - 16 };
    let (x, y, _) = search(&h0, cur, &r0, (fine.2 * 2.0).round() as i32, (fine.3 * 2.0).round() as i32, 2);
    let conf = ((median - c) / median.max(1e-3)).clamp(0.0, 1.0);
    let err = cost(&h0, cur, &r0, x.round() as i32, y.round() as i32);
    View { dx: x, dy: y, zoom: z, conf, err }
}

#[cfg(test)]
mod tests {
    use super::*;

    pub fn scene(w: usize, h: usize, ox: f32, oy: f32, zoom: f32) -> Img {
        let mut p = Vec::with_capacity(w * h);
        for y in 0..h {
            for x in 0..w {
                let u = (x as f32 - w as f32 / 2.0) / zoom + w as f32 / 2.0 + ox;
                let v = (y as f32 - h as f32 / 2.0) / zoom + h as f32 / 2.0 + oy;
                p.push(crate::sim::texture(u * 0.08, v * 0.08));
            }
        }
        Img::new(w, h, p)
    }

    #[test]
    fn recovers_subpixel_shifts_in_both_axes() {
        for (sx, sy) in [(0.0, 0.0), (3.4, -1.2), (-17.6, 2.5), (41.3, -6.8), (-60.0, 0.0)] {
            let a = scene(TW, TH, 0.0, 0.0, 1.0);
            let b = scene(TW, TH, -sx, -sy, 1.0);
            let m = estimate(&a, &b);
            assert!((m.dx - sx).abs() < 0.15 && (m.dy - sy).abs() < 0.15, "{sx},{sy} -> {m:?}");
            assert!(m.scale.abs() < 0.004 && m.conf > 0.3, "{m:?}");
        }
    }

    #[test]
    fn home_view_is_found_turned_and_moved() {
        let home = scene(TW, TH, 0.0, 0.0, 1.0);
        for (sx, z) in [(0.0, 1.0), (40.0, 1.0), (-90.0, 1.12), (25.0, 0.85), (0.0, 1.3)] {
            let cur = scene(TW, TH, -sx / z, 0.0, z);
            let v = compare(&home, &cur);
            assert!((v.dx - sx).abs() < 2.5 && (v.zoom / z - 1.0).abs() < 0.03 && v.conf > 0.3, "{sx} {z} -> {v:?}");
        }
        let other = scene(TW, TH, 900.0, 300.0, 1.0);
        assert!(compare(&home, &other).conf < 0.3);
    }

    #[test]
    fn expansion_reads_as_positive_scale() {
        let a = scene(TW, TH, 0.0, 0.0, 1.0);
        let b = scene(TW, TH, 0.0, 0.0, 1.03);
        let m = estimate(&a, &b);
        assert!((m.scale - 0.03).abs() < 0.008 && m.dx.abs() < 0.3, "{m:?}");
    }
}
