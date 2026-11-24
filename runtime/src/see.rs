// SPDX-License-Identifier: Apache-2.0
//! Seeing: the camera's frames turned into where one coloured thing is.
//!
//! The first thing the robot looks for is a toy of one strong colour (a ball): finding a colour costs almost
//! nothing, works at the camera's full frame rate and needs no trained model. What comes out is a direction (and,
//! for a ball whose size is known, a distance), which is what the walking policy's command and the head's gaze are
//! made from. The policy never sees a picture.
//!
//! Frames come from `v4l2-ctl` as a child process, uncompressed (YUYV 4:2:2, [`camera`]), so nothing here links a
//! video library or decodes JPEG, and any `Read` of such frames can be looked at in a test. In YUYV the colour of
//! every two pixels is two bytes (U and V) apart from their brightness, so "this colour, however brightly lit" is
//! a comparison of two bytes.

use std::io::{self, Read};
use std::process::{Child, ChildStdout, Command, Stdio};

/// The frame size asked of the camera. Small on purpose: a ball 40 mm across is 20 pixels wide at 0.5 m.
pub const WIDTH: usize = 320;
pub const HEIGHT: usize = 240;
/// Bytes in one YUYV frame.
pub const FRAME_BYTES: usize = WIDTH * HEIGHT * 2;
/// The grid a frame is searched on: one cell is 2 x 2 pixels.
const GRID_W: usize = WIDTH / 2;
const GRID_H: usize = HEIGHT / 2;

/// A colour to look for: its U and V, how far a pixel's may be from them, and how bright the pixel must be (dark
/// pixels have no colour worth trusting).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Chroma {
    pub u: u8,
    pub v: u8,
    pub tolerance: u8,
    pub min_luma: u8,
}

impl Chroma {
    pub fn new(u: u8, v: u8) -> Self {
        Chroma { u, v, tolerance: 14, min_luma: 40 }
    }

    /// How far from grey this colour is. A colour near grey (128, 128) cannot be told from walls and floors.
    pub fn strength(&self) -> f64 {
        (f64::from(self.u) - 128.0).hypot(f64::from(self.v) - 128.0)
    }

    fn matches(&self, y: u8, u: u8, v: u8) -> bool {
        y >= self.min_luma && u.abs_diff(self.u) <= self.tolerance && v.abs_diff(self.v) <= self.tolerance
    }
}

/// The cell at grid position (`gx`, `gy`): its brightness, U and V, from the frame's upper row of the cell.
fn cell(frame: &[u8], gx: usize, gy: usize) -> Option<(u8, u8, u8)> {
    let at = (gy * 2 * WIDTH + gx * 2) * 2;
    match frame.get(at..at + 4) {
        Some(&[y0, u, y1, v]) => Some((((u16::from(y0) + u16::from(y1)) / 2) as u8, u, v)),
        _ => None,
    }
}

/// The colour in the middle of the frame (the median over the central eighth of each side): hold the toy in front
/// of the camera and call this.
pub fn learn(frame: &[u8]) -> Option<Chroma> {
    let (mut us, mut vs) = (Vec::new(), Vec::new());
    for gy in GRID_H * 7 / 16..GRID_H * 9 / 16 {
        for gx in GRID_W * 7 / 16..GRID_W * 9 / 16 {
            let (_, u, v) = cell(frame, gx, gy)?;
            us.push(u);
            vs.push(v);
        }
    }
    us.sort_unstable();
    vs.sort_unstable();
    Some(Chroma::new(*us.get(us.len() / 2)?, *vs.get(vs.len() / 2)?))
}

/// One connected patch of the colour, in pixels of the frame (x to the right, y down).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Blob {
    /// The centre of its bounding box.
    pub x: f64,
    pub y: f64,
    pub width: f64,
    pub height: f64,
    /// Matching cells in it (each 2 x 2 pixels).
    pub cells: usize,
}

impl Blob {
    /// How much of its bounding box it fills: about 0.79 for a disc, less for a ragged or hollow patch.
    pub fn fill(&self) -> f64 {
        self.cells as f64 * 4.0 / (self.width * self.height).max(1.0)
    }
}

/// The largest connected patch of `colour` in a YUYV frame of [`WIDTH`] x [`HEIGHT`], if it has at least
/// `min_cells` cells. `None` also for a frame of the wrong size.
pub fn find(frame: &[u8], colour: &Chroma, min_cells: usize) -> Option<Blob> {
    if frame.len() != FRAME_BYTES {
        return None;
    }
    let mut todo: Vec<bool> = (0..GRID_W * GRID_H)
        .map(|i| cell(frame, i % GRID_W, i / GRID_W).is_some_and(|(y, u, v)| colour.matches(y, u, v)))
        .collect();
    let mut best: Option<Blob> = None;
    let mut stack = Vec::new();
    for start in 0..todo.len() {
        if !todo.get(start).copied().unwrap_or(false) {
            continue;
        }
        // Flood this patch, taking its cells out of `todo`.
        let (mut cells, mut x0, mut x1, mut y0, mut y1) = (0usize, usize::MAX, 0usize, usize::MAX, 0usize);
        stack.push(start);
        if let Some(t) = todo.get_mut(start) {
            *t = false;
        }
        while let Some(i) = stack.pop() {
            let (gx, gy) = (i % GRID_W, i / GRID_W);
            cells += 1;
            (x0, x1, y0, y1) = (x0.min(gx), x1.max(gx), y0.min(gy), y1.max(gy));
            let neighbours = [
                (gx > 0).then(|| i - 1),
                (gx + 1 < GRID_W).then(|| i + 1),
                (gy > 0).then(|| i - GRID_W),
                (gy + 1 < GRID_H).then(|| i + GRID_W),
            ];
            for n in neighbours.into_iter().flatten() {
                if let Some(t) = todo.get_mut(n).filter(|t| **t) {
                    *t = false;
                    stack.push(n);
                }
            }
        }
        if cells >= min_cells && best.is_none_or(|b| cells > b.cells) {
            let (width, height) = (((x1 - x0 + 1) * 2) as f64, ((y1 - y0 + 1) * 2) as f64);
            best = Some(Blob { x: (x0 * 2) as f64 + width / 2.0, y: (y0 * 2) as f64 + height / 2.0, width, height, cells });
        }
    }
    best
}

/// The camera's lens, as the focal length in pixels of a [`WIDTH`]-wide frame.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Lens {
    pub focal_px: f64,
}

impl Lens {
    /// From the angle the frame covers from its left edge to its right.
    pub fn from_fov(horizontal_deg: f64) -> Self {
        Lens { focal_px: WIDTH as f64 / 2.0 / (horizontal_deg.to_radians() / 2.0).tan() }
    }

    /// From a ball of known size seen at a known distance: the calibration to do once per camera.
    pub fn from_ball(blob: &Blob, diameter_m: f64, distance_m: f64) -> Self {
        Lens { focal_px: blob.width.max(blob.height) * distance_m / diameter_m }
    }

    /// The angle of `blob` to the left of where the camera points, radians (to the left is positive, as a turn is).
    pub fn bearing(&self, blob: &Blob) -> f64 {
        ((WIDTH as f64 / 2.0 - blob.x) / self.focal_px).atan()
    }

    /// The angle of `blob` above where the camera points, radians.
    pub fn elevation(&self, blob: &Blob) -> f64 {
        ((HEIGHT as f64 / 2.0 - blob.y) / self.focal_px).atan()
    }

    /// How far away a ball `diameter_m` across is, metres, from its larger side (the smaller may be cut off by
    /// the frame's edge or by something in front of it).
    pub fn distance(&self, blob: &Blob, diameter_m: f64) -> f64 {
        self.focal_px * diameter_m / blob.width.max(blob.height).max(1.0)
    }
}

/// A camera being read by `v4l2-ctl`. Dropping it stops the stream.
pub struct Camera {
    child: Child,
    frames: ChildStdout,
}

/// Start streaming uncompressed frames of [`WIDTH`] x [`HEIGHT`] from the video device `device` (for example
/// `/dev/video0`), 30 a second if the light allows: with automatic exposure a camera slows down in a dim room.
pub fn camera(device: &str) -> io::Result<Camera> {
    let mut child = Command::new("v4l2-ctl")
        .args(["-d", device])
        .arg(format!("--set-fmt-video=width={WIDTH},height={HEIGHT},pixelformat=YUYV"))
        .args(["--set-parm=30", "--stream-mmap", "--stream-to=-"])
        .stdin(Stdio::null())
        .stdout(Stdio::piped())
        .stderr(Stdio::null()) // it draws a progress line there
        .spawn()
        .map_err(|e| io::Error::new(e.kind(), format!("could not start v4l2-ctl (the v4l-utils package): {e}")))?;
    match child.stdout.take() {
        Some(frames) => Ok(Camera { child, frames }),
        None => {
            let _ = child.kill();
            Err(io::Error::other("v4l2-ctl gave no output to read"))
        }
    }
}

impl Read for Camera {
    fn read(&mut self, buf: &mut [u8]) -> io::Result<usize> {
        self.frames.read(buf)
    }
}

impl Drop for Camera {
    fn drop(&mut self) {
        let _ = self.child.kill();
        let _ = self.child.wait();
    }
}

/// Read the next frame into `frame` (resized to [`FRAME_BYTES`]). `Ok(false)`: the stream ended.
pub fn read_frame(from: &mut impl Read, frame: &mut Vec<u8>) -> io::Result<bool> {
    frame.resize(FRAME_BYTES, 0);
    match from.read_exact(frame) {
        Ok(()) => Ok(true),
        Err(e) if e.kind() == io::ErrorKind::UnexpectedEof => Ok(false),
        Err(e) => Err(e),
    }
}
