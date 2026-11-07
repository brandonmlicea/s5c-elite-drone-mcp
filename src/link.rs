use aes::cipher::{generic_array::GenericArray, BlockDecrypt, BlockEncrypt, KeyInit};
use aes::Aes128;
use std::io::{self, Read, Write};
use std::net::{TcpStream, ToSocketAddrs};
use std::time::Duration;

const KEY: &[u8; 16] = b"guanxukj@fh8620.";
const USER: &[u8] = b"guanxukeji";
const PASS: &[u8] = b"gxrdw60";

pub struct Link {
    sock: TcpStream,
    aes: Aes128,
}

pub fn bad(msg: String) -> io::Error {
    io::Error::new(io::ErrorKind::Other, msg)
}

impl Link {
    pub fn open(addr: &str) -> io::Result<Link> {
        let target = addr.to_socket_addrs()?.next().ok_or_else(|| bad("bad address".into()))?;
        let mut tries = 0;
        let sock = loop {
            match TcpStream::connect_timeout(&target, Duration::from_secs(2)) {
                Ok(sock) => break sock,
                Err(e) if tries >= 2 => return Err(e),
                Err(_) => tries += 1,
            }
        };
        sock.set_read_timeout(Some(Duration::from_secs(3)))?;
        sock.set_nodelay(true)?;
        Ok(Link { sock, aes: Aes128::new(GenericArray::from_slice(KEY)) })
    }

    pub fn recv_bytes(&mut self, buf: &mut [u8]) -> io::Result<usize> {
        self.sock.read(buf)
    }

    pub fn call(&mut self, kind: u8, main: u8, sub: u8, payload: &[u8]) -> io::Result<Vec<u8>> {
        let mut plain = vec![0u8; 0x52];
        plain[0] = kind;
        plain[1] = 0x51;
        plain[3] = main;
        plain[4] = sub;
        plain[10..10 + USER.len()].copy_from_slice(USER);
        plain[42..42 + PASS.len()].copy_from_slice(PASS);
        plain[79..81].copy_from_slice(&(payload.len() as u16 + 1).to_le_bytes());
        plain.extend_from_slice(payload);
        let len = plain.len();
        plain.resize(len.div_ceil(16) * 16, 0);
        for block in plain.chunks_exact_mut(16) {
            self.aes.encrypt_block(GenericArray::from_mut_slice(block));
        }
        let mut frame = b"IT".to_vec();
        frame.extend_from_slice(&(plain.len() as u32 + 4).to_le_bytes());
        frame.extend_from_slice(&(len as u32).to_le_bytes());
        frame.extend_from_slice(&plain);
        self.sock.write_all(&frame)?;

        for _ in 0..10 {
            let mut head = [0u8; 10];
            self.sock.read_exact(&mut head)?;
            let size = u32::from_le_bytes(head[2..6].try_into().unwrap()) as usize;
            if &head[..2] != b"IT" || !(4..0x100000).contains(&size) {
                return Err(bad(format!("unexpected frame {:02x?}", head)));
            }
            let mut body = vec![0u8; size - 4];
            self.sock.read_exact(&mut body)?;
            for block in body.chunks_exact_mut(16) {
                self.aes.decrypt_block(GenericArray::from_mut_slice(block));
            }
            body.truncate(u32::from_le_bytes(head[6..10].try_into().unwrap()) as usize);
            if body.len() < 0x51 || body[3] != main || body[4] != sub + 1 {
                continue;
            }
            let err = u32::from_le_bytes(body[5..9].try_into().unwrap());
            if err != 0 {
                return Err(bad(format!("drone refused command {main}/{sub} with error {err}")));
            }
            let n = u16::from_le_bytes(body[79..81].try_into().unwrap()) as usize;
            return Ok(body.get(82..81 + n.max(1)).unwrap_or(&[]).to_vec());
        }
        Err(bad(format!("no reply to command {main}/{sub}")))
    }
}
