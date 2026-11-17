// SPDX-License-Identifier: Apache-2.0
//! Client for the UNO Q's router: MessagePack-RPC over a Unix socket.
//!
//! The router forwards each call over the board's internal serial link to the microcontroller sketch, which
//! registered the method with Arduino's Bridge library. A request is `[0, id, method, [params]]`, the reply is
//! `[1, id, error, result]`.

use std::fmt;
use std::io::{self, Read, Write};
use std::os::unix::net::UnixStream;
use std::path::Path;
use std::time::Duration;

use rmpv::Value;

/// Where the router listens on the board.
pub const ROUTER_SOCKET: &str = "/var/run/arduino-router.sock";

#[derive(Debug)]
pub enum BridgeError {
    Io(io::Error),
    /// The bytes that came back were not a MessagePack-RPC reply.
    Malformed(String),
    /// The reply answers a different request.
    WrongId { sent: u32, got: u64 },
    /// The router or the sketch refused the call (for example: method not registered).
    Remote(String),
    /// The result was not shaped as the caller expected.
    Shape(&'static str),
}

impl fmt::Display for BridgeError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            BridgeError::Io(e) => write!(f, "router socket: {e}"),
            BridgeError::Malformed(why) => write!(f, "malformed reply: {why}"),
            BridgeError::WrongId { sent, got } => write!(f, "reply for request {got}, expected {sent}"),
            BridgeError::Remote(why) => write!(f, "refused: {why}"),
            BridgeError::Shape(what) => write!(f, "unexpected result: {what}"),
        }
    }
}

impl std::error::Error for BridgeError {}

impl From<io::Error> for BridgeError {
    fn from(e: io::Error) -> Self {
        BridgeError::Io(e)
    }
}

/// One connection to the router. Generic over the stream so tests can use a socket pair.
pub struct Bridge<S> {
    stream: S,
    next_id: u32,
}

impl Bridge<UnixStream> {
    /// Connect to the router socket with a per-call timeout.
    pub fn connect(path: impl AsRef<Path>, timeout: Duration) -> Result<Self, BridgeError> {
        let stream = UnixStream::connect(path)?;
        let mut bridge = Bridge::new(stream);
        bridge.set_timeout(timeout)?;
        Ok(bridge)
    }

    /// How long a call may wait to send its request and for its reply. A reply that comes later is dropped when it
    /// arrives, so a loop can wait one tick and no longer.
    pub fn set_timeout(&mut self, timeout: Duration) -> io::Result<()> {
        self.stream.set_read_timeout(Some(timeout))?;
        self.stream.set_write_timeout(Some(timeout))
    }
}

impl<S: Read + Write> Bridge<S> {
    pub fn new(stream: S) -> Self {
        Bridge { stream, next_id: 1 }
    }

    /// Call `method` with `params` and return its result.
    pub fn call(&mut self, method: &str, params: Vec<Value>) -> Result<Value, BridgeError> {
        let id = self.next_id;
        self.next_id = self.next_id.wrapping_add(1).max(1);
        let request = Value::Array(vec![Value::from(0), Value::from(id), Value::from(method), Value::Array(params)]);
        let mut bytes = Vec::with_capacity(64);
        rmpv::encode::write_value(&mut bytes, &request).map_err(|e| BridgeError::Malformed(e.to_string()))?;
        self.stream.write_all(&bytes)?;
        self.stream.flush()?;

        loop {
            let (got, error, result) = self.read_reply()?;
            if got == u64::from(id) {
                return if error.is_nil() { Ok(result) } else { Err(BridgeError::Remote(error.to_string())) };
            }
            // The late answer to an earlier call that already timed out: stale, so dropped. Taking it as this call's
            // answer would leave every later call one reply behind.
            let behind = u32::try_from(got).map(|got| id.wrapping_sub(got));
            if !matches!(behind, Ok(1..=0x7fff_ffff)) {
                return Err(BridgeError::WrongId { sent: id, got });
            }
        }
    }

    /// One reply off the stream: its id, error and result.
    fn read_reply(&mut self) -> Result<(u64, Value, Value), BridgeError> {
        let reply = rmpv::decode::read_value(&mut self.stream).map_err(|e| match e {
            rmpv::decode::Error::InvalidMarkerRead(io) | rmpv::decode::Error::InvalidDataRead(io) => BridgeError::Io(io),
            other => BridgeError::Malformed(other.to_string()),
        })?;
        let Value::Array(fields) = reply else {
            return Err(BridgeError::Malformed("reply is not an array".into()));
        };
        let [kind, got_id, error, result] = <[Value; 4]>::try_from(fields)
            .map_err(|f| BridgeError::Malformed(format!("reply has {} fields, expected 4", f.len())))?;
        if kind.as_u64() != Some(1) {
            return Err(BridgeError::Malformed("reply type is not 1".into()));
        }
        let got = got_id.as_u64().ok_or_else(|| BridgeError::Malformed("reply id is not a number".into()))?;
        Ok((got, error, result))
    }

    /// Call a method that returns a list of integers.
    pub fn call_ints(&mut self, method: &str, params: Vec<Value>) -> Result<Vec<i64>, BridgeError> {
        let Value::Array(items) = self.call(method, params)? else {
            return Err(BridgeError::Shape("expected a list"));
        };
        items.iter().map(|v| v.as_i64().ok_or(BridgeError::Shape("expected integers"))).collect()
    }

    /// Call a method that returns one integer.
    pub fn call_int(&mut self, method: &str, params: Vec<Value>) -> Result<i64, BridgeError> {
        self.call(method, params)?.as_i64().ok_or(BridgeError::Shape("expected a whole number"))
    }
}
