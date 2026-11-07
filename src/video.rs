use jpeg_encoder::{ColorType, Encoder};
use openh264::decoder::{Decoder, DecoderConfig, Flush};
use openh264::formats::YUVSource;
use openh264::OpenH264API;

const START: [u8; 4] = [0, 0, 0, 1];

fn find_start(data: &[u8], from: usize) -> Option<usize> {
    data.get(from..)?.windows(3).position(|w| w == [0, 0, 1]).map(|p| p + from)
}

fn wanted(head: u8) -> bool {
    head & 0x80 == 0 && [1, 5, 6, 7, 8].contains(&(head & 0x1f))
}

pub struct Splitter {
    buf: Vec<u8>,
    synced: bool,
    scan: usize,
}

impl Splitter {
    pub fn new() -> Splitter {
        Splitter { buf: Vec::new(), synced: false, scan: 3 }
    }

    pub fn push(&mut self, data: &[u8]) -> Vec<Vec<u8>> {
        self.buf.extend_from_slice(data);
        let mut out = Vec::new();
        if !self.synced {
            let Some(first) = find_start(&self.buf, 0) else {
                self.buf.drain(..self.buf.len().saturating_sub(2));
                return out;
            };
            self.buf.drain(..first);
            self.synced = true;
        }
        while let Some(next) = find_start(&self.buf, self.scan) {
            let body = &self.buf[3..next];
            let body = &body[..body.iter().rposition(|b| *b != 0).map_or(0, |p| p + 1)];
            if body.first().is_some_and(|h| wanted(*h)) {
                out.push([&START[..], body].concat());
            }
            self.buf.drain(..next);
            self.scan = 3;
        }
        self.scan = self.buf.len().saturating_sub(2).max(3);
        if self.buf.len() > 8_000_000 {
            *self = Splitter::new();
        }
        out
    }
}

pub struct Frame {
    pub w: usize,
    pub h: usize,
    pub y: Vec<u8>,
    pub u: Vec<u8>,
    pub v: Vec<u8>,
}

impl Frame {
    pub fn from_yuv(img: &impl YUVSource) -> Frame {
        let (w, h) = img.dimensions();
        let (ys, us, vs) = img.strides();
        let (cw, ch) = (w.div_ceil(2), h.div_ceil(2));
        let pack = |src: &[u8], stride: usize, pw: usize, ph: usize| {
            let mut out = Vec::with_capacity(pw * ph);
            for row in 0..ph {
                out.extend_from_slice(&src[row * stride..row * stride + pw]);
            }
            out
        };
        Frame { w, h, y: pack(img.y(), ys, w, h), u: pack(img.u(), us, cw, ch), v: pack(img.v(), vs, cw, ch) }
    }

    pub fn jpeg(&self, quality: u8) -> Vec<u8> {
        let (w, h, cw) = (self.w, self.h, self.w.div_ceil(2));
        let mut ycc = vec![0u8; w * h * 3];
        for row in 0..h {
            let (yr, cr) = (row * w, (row / 2) * cw);
            for col in 0..w {
                let o = (yr + col) * 3;
                ycc[o] = self.y[yr + col];
                ycc[o + 1] = self.u[cr + col / 2];
                ycc[o + 2] = self.v[cr + col / 2];
            }
        }
        let mut out = Vec::with_capacity(w * h / 6);
        let _ = Encoder::new(&mut out, quality).encode(&ycc, w as u16, h as u16, ColorType::Ycbcr);
        out
    }
}

pub fn decoder() -> Result<Decoder, openh264::Error> {
    Decoder::with_api_config(OpenH264API::from_source(), DecoderConfig::new().flush_after_decode(Flush::NoFlush))
}

pub struct Stamp {
    pub seq: u8,
    pub ts_us: u32,
    pub key: bool,
}

pub struct HeaderScan {
    tail: Vec<u8>,
}

impl HeaderScan {
    pub fn new() -> HeaderScan {
        HeaderScan { tail: Vec::new() }
    }

    pub fn push(&mut self, data: &[u8]) -> Vec<Stamp> {
        let buf = [&self.tail[..], data].concat();
        let mut out = Vec::new();
        for i in 0..buf.len().saturating_sub(35) {
            if buf[i..i + 3] == [0, 0, 1] && (buf[i + 3] == 0xa0 || buf[i + 3] == 0xa1) && buf[i + 11] == 0x19 {
                let ts_us = u32::from_le_bytes(buf[i + 32..i + 36].try_into().unwrap());
                out.push(Stamp { seq: buf[i + 10], ts_us, key: buf[i + 3] == 0xa1 });
            }
        }
        self.tail = buf[buf.len().saturating_sub(35)..].to_vec();
        out
    }
}

fn header_at(buf: &[u8], i: usize) -> bool {
    buf[i..i + 3] == [0, 0, 1] && (buf[i + 3] == 0xa0 || buf[i + 3] == 0xa1) && buf[i + 11] == 0x19
}

pub struct Framer {
    buf: Vec<u8>,
    synced: bool,
    scan: usize,
}

impl Framer {
    pub fn new() -> Framer {
        Framer { buf: Vec::new(), synced: false, scan: 1 }
    }

    fn find(&self, from: usize) -> Option<usize> {
        (from..(self.buf.len() + 1).saturating_sub(36)).find(|i| header_at(&self.buf, *i))
    }

    pub fn push(&mut self, data: &[u8]) -> Vec<(Vec<u8>, u32)> {
        self.buf.extend_from_slice(data);
        let mut out = Vec::new();
        if !self.synced {
            let Some(p) = self.find(0) else {
                self.buf.drain(..self.buf.len().saturating_sub(35));
                return out;
            };
            self.buf.drain(..p);
            (self.synced, self.scan) = (true, 1);
        }
        while self.buf.len() >= 44 {
            let ts = u32::from_le_bytes(self.buf[32..36].try_into().unwrap());
            let need = 44 + u32::from_le_bytes(self.buf[16..20].try_into().unwrap()) as usize;
            let trusted = need <= 4_000_000 && (self.buf.len() < need + 36 || header_at(&self.buf, need));
            let cut = if trusted { Some(need).filter(|n| *n <= self.buf.len()) } else { self.find(self.scan) };
            let Some(cut) = cut else {
                self.scan = (self.buf.len() + 1).saturating_sub(36).max(1);
                break;
            };
            out.push((self.buf.drain(..cut).collect(), ts));
            self.scan = 1;
        }
        if self.buf.len() > 8_000_000 {
            *self = Framer::new();
        }
        out
    }
}

pub struct Stream {
    dec: Decoder,
    split: Splitter,
    tainted: bool,
    keyframe_in: bool,
}

impl Stream {
    pub fn new() -> Result<Stream, openh264::Error> {
        Ok(Stream { dec: decoder()?, split: Splitter::new(), tainted: true, keyframe_in: false })
    }

    pub fn push(&mut self, data: &[u8]) -> (Vec<Frame>, Vec<String>) {
        let (mut frames, mut errors) = (Vec::new(), Vec::new());
        for nal in self.split.push(data) {
            match self.dec.decode(&nal) {
                Ok(out) => {
                    if let Some(img) = out.filter(|_| !self.tainted || self.keyframe_in) {
                        frames.push(Frame::from_yuv(&img));
                        self.tainted = false;
                    }
                    if self.tainted && nal[4] & 0x1f == 5 {
                        self.keyframe_in = true;
                    }
                }
                Err(e) => {
                    (self.tainted, self.keyframe_in) = (true, false);
                    errors.push(e.to_string());
                }
            }
        }
        (frames, errors)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::time::Instant;

    fn stream() -> Vec<u8> {
        std::fs::read(concat!(env!("CARGO_MANIFEST_DIR"), "/testdata/stream.h264")).unwrap()
    }

    fn split_all(size: usize) -> Vec<Vec<u8>> {
        let mut split = Splitter::new();
        stream().chunks(size).chain(std::iter::once(&START[..])).flat_map(|c| split.push(c)).collect()
    }

    fn decode_in_chunks(size: usize, loops: usize) -> (usize, usize, Option<Frame>, Vec<Vec<u8>>) {
        let data = stream().repeat(loops);
        let mut s = Stream::new().unwrap();
        let (mut frames, mut errors, mut last, mut thumbs) = (0, 0, None, Vec::new());
        for chunk in data.chunks(size).chain(std::iter::once(&START[..])) {
            let (out, errs) = s.push(chunk);
            errors += errs.len();
            for f in out {
                thumbs.push(f.y.iter().step_by(97).copied().collect());
                frames += 1;
                last = Some(f);
            }
        }
        (frames, errors, last, thumbs)
    }

    #[test]
    fn splitter_is_identical_for_any_chunking_and_drops_container_headers() {
        let whole = split_all(1 << 24);
        assert!(whole.iter().all(|n| n[..4] == START && wanted(n[4])));
        assert_eq!(whole.iter().filter(|n| [1, 5].contains(&(n[4] & 0x1f))).count(), 102);
        for size in [1, 2, 3, 997, 1460, 65536] {
            assert!(split_all(size) == whole, "chunk size {size}");
        }
    }

    #[test]
    fn decodes_every_complete_picture() {
        for size in [1460, 65536] {
            let (frames, errors, last, _) = decode_in_chunks(size, 1);
            assert_eq!(errors, 1, "only the truncated final keyframe may fail");
            assert!(frames >= 100, "chunk size {size}: {frames} frames");
            let f = last.unwrap();
            assert_eq!((f.w, f.h), (1920, 1080));
        }
    }

    #[test]
    fn a_broken_keyframe_never_reaches_the_screen_and_video_recovers() {
        let (_, _, _, clean) = decode_in_chunks(65536, 1);
        let (frames, errors, _, looped) = decode_in_chunks(1460, 3);
        let bad = looped.iter().filter(|t| !clean.contains(t)).count();
        eprintln!("3 loops: {frames} frames, {errors} errors, {bad} pictures not in the clean set");
        assert_eq!(errors, 3);
        assert!(frames >= 300, "{frames} frames");
        assert_eq!(bad, 0, "a corrupt picture got through");
    }

    fn live() -> Vec<u8> {
        std::fs::read(concat!(env!("CARGO_MANIFEST_DIR"), "/testdata/live-raw.bin")).unwrap()
    }

    #[test]
    fn real_drone_stream_decodes_every_frame_and_every_header_is_found() {
        let data = live();
        let whole: Vec<Stamp> = HeaderScan::new().push(&data);
        assert!(whole.len() > 300, "{} headers", whole.len());
        assert!(whole.windows(2).all(|w| w[1].seq == w[0].seq.wrapping_add(1) && w[1].ts_us.wrapping_sub(w[0].ts_us) == 40_000));
        for size in [1, 1460, 65536] {
            let mut scan = HeaderScan::new();
            let found: usize = data.chunks(size).map(|c| scan.push(c).len()).sum();
            assert_eq!(found, whole.len(), "chunk size {size}");
        }
        let mut s = Stream::new().unwrap();
        let (mut frames, mut errors) = (0, 0);
        for c in data.chunks(1460) {
            let (f, e) = s.push(c);
            frames += f.len();
            errors += e.len();
        }
        eprintln!("live capture: {} drone frames, {frames} decoded, {errors} errors", whole.len());
        assert_eq!(errors, 0);
        assert!(frames + 2 >= whole.len(), "{frames} of {}", whole.len());
    }

    #[test]
    fn framer_cuts_every_drone_frame_with_its_timestamp_for_any_chunking() {
        let data = live();
        let stamps: Vec<u32> = HeaderScan::new().push(&data).iter().map(|s| s.ts_us).collect();
        for size in [1, 1460, 65536] {
            let mut f = Framer::new();
            let frames: Vec<(Vec<u8>, u32)> = data.chunks(size).flat_map(|c| f.push(c)).collect();
            assert!(frames.len() + 1 >= stamps.len() && frames.len() <= stamps.len(), "chunk size {size}: {} of {}", frames.len(), stamps.len());
            assert!(frames.iter().zip(&stamps).all(|((_, a), b)| a == b));
            let first = (0..data.len() - 36).find(|i| header_at(&data, *i)).unwrap();
            assert_eq!(frames.iter().map(|(b, _)| b.len()).sum::<usize>() + f.buf.len(), data.len() - first);
        }
        let mut s = Stream::new().unwrap();
        let decoded: usize = Framer::new().push(&data).iter().map(|(b, _)| s.push(b).0.len()).sum();
        assert!(decoded + 3 >= stamps.len(), "{decoded} of {}", stamps.len());
    }

    #[test]
    fn header_length_field_gives_the_exact_frame_size() {
        let data = live();
        let heads: Vec<usize> = (0..data.len() - 36).filter(|i| header_at(&data, *i)).collect();
        let fits = heads.windows(2).filter(|w| 44 + u32::from_le_bytes(data[w[0] + 16..w[0] + 20].try_into().unwrap()) as usize == w[1] - w[0]).count();
        eprintln!("{fits} of {} frames: next header = 44 + length; payload starts {:02x?}", heads.len() - 1, &data[heads[0] + 44..heads[0] + 50]);
        assert_eq!(fits, heads.len() - 1);
    }

    #[test]
    fn keyframe_spacing() {
        for path in std::fs::read_dir(concat!(env!("CARGO_MANIFEST_DIR"), "/runtime")).unwrap().filter_map(|e| e.ok()).map(|e| e.path()) {
            let name = path.file_name().unwrap().to_string_lossy().to_string();
            if !name.starts_with("video-raw-17919") {
                continue;
            }
            let stamps = HeaderScan::new().push(&std::fs::read(&path).unwrap());
            let keys: Vec<usize> = stamps.iter().enumerate().filter(|(_, s)| s.key).map(|(i, _)| i).collect();
            let gaps: Vec<usize> = keys.windows(2).map(|w| w[1] - w[0]).collect();
            eprintln!("{name}: {} frames, {} keyframes, gaps {:?}", stamps.len(), keys.len(), &gaps[..gaps.len().min(12)]);
        }
    }

    #[test]
    fn full_res_jpeg_keeps_up_with_20fps() {
        let (_, _, last, _) = decode_in_chunks(65536, 1);
        let f = last.unwrap();
        let t = Instant::now();
        let jpg = f.jpeg(80);
        let ms = t.elapsed().as_secs_f32() * 1000.0;
        assert_eq!(&jpg[..2], &[0xFF, 0xD8]);
        eprintln!("1080p jpeg {} KB in {ms:.1} ms", jpg.len() / 1024);
        assert!(ms < 50.0, "jpeg took {ms} ms");
    }
}