#!/usr/bin/env python3
"""LoRa Scanner - receive-only LoRa survey tool for M5Stack CardputerZero + Cap LoRa-1262.

Never transmits. Drives the Cap's SX1262 directly (spidev + GPIO character
device ioctls, no extra packages).

Pages
  1 Spectrum   live signal level across the US 902-928 MHz band
  2 Waterfall  the last ~2 minutes of sweeps, newest at the top
  3 Finder     listens on each Meshtastic preset's default frequency; decodes
               public default-channel packets (text, node info, telemetry...)
  4 LoRaWAN    hops the US915 channels of a sub-band (uplinks) or the gateway
               downlink channels and decodes LoRaWAN headers (device address,
               network, frame counter, join requests)
  5 Packets    every packet heard, decoded; Enter shows all fields + hex
  6 Map        coverage map: packets heard plotted where you were (Cap GPS),
               exported as a Google Earth KML file

Keys (the Cardputer's arrow keys: F up, X down, Z left, C right)
  Z / C, Tab, 1-6   change page      ESC / Q   quit
  Finder: F / X select preset, Enter lock, + / - dwell time
  LoRaWAN: F / X select channel, Enter lock, S spreading factor,
           B sub-band, D uplinks / downlinks, + / - dwell
  Packets: F / X select, Enter details, T filter
  Map: K save KML        Spectrum: M clear max-hold

Run with --preview out.png PAGE --mock to render without hardware.
"""

import ctypes
import fcntl
import glob
import math
import mmap
import os
import random
import select
import signal
import struct
import subprocess
import sys
import threading
import time
from collections import deque
from datetime import datetime

from PIL import Image, ImageDraw, ImageFont

try:
    import numpy as np
    HAS_NUMPY = True
except ImportError:
    HAS_NUMPY = False

VERSION = "1.1.1"
APP = "pocket-lorascan"

W, H = 320, 170
FB_DEVICE = os.environ.get("APPLAUNCH_LINUX_FBDEV_DEVICE", "/dev/fb0")
FONT_DIR = "/usr/share/fonts/truetype/dejavu"
HOME = os.path.expanduser("~")
LOG_DIR = os.environ.get("LORASCAN_DIR", os.path.join(HOME, "LoRaScan"))

# Cap LoRa-1262 wiring on CardputerZero (same as Meshtastic's lora-cardputer-zero.yaml
# and M5Stack's Cap-LoRa-1262 app)
SPI_DEV = os.environ.get("LORA_SPI_DEV", "/dev/spidev0.1")
GPIO_CHIP = os.environ.get("LORA_GPIO_CHIP", "/dev/gpiochip0")
PIN_BUSY, PIN_IRQ, PIN_RESET = 22, 23, 26
SPI_SPEED = 1000000
EXT5V = "/sys/class/leds/ext_5v_out/brightness"
I2C_BUS = "/dev/i2c-1"
PI4IO_ADDR = 0x43
TCXO_VOLTAGE_CODE = 0x06      # 3.0 V

# US band
BAND_LO, BAND_HI = 902.0, 928.0
CAP_MAX_MHZ = 923.0           # Cap LoRa-1262 rated 868-923 MHz
SWEEP_STEP = 0.25             # MHz
SWEEP_POINTS = int((BAND_HI - BAND_LO) / SWEEP_STEP)   # 104

MESH_SYNC_WORD = 0x2B
MESH_PREAMBLE = 16
# Meshtastic's default key ("AQ==" expanded), used to tell the public default
# channel apart from private ones by the 1-byte channel hash in the header.
DEFAULT_PSK = bytes([0xd4, 0xf1, 0xbb, 0x3a, 0x20, 0x29, 0x07, 0x59,
                     0xf0, 0xbc, 0xff, 0xab, 0xcf, 0x4e, 0x69, 0x01])

# name: (bandwidth kHz, spreading factor, coding rate 4/x) - from Meshtastic firmware MeshRadio.h
PRESETS = [
    ("LongFast", 250, 11, 5), ("MediumFast", 250, 9, 5), ("MediumSlow", 250, 10, 5),
    ("ShortFast", 250, 7, 5), ("ShortSlow", 250, 8, 5), ("LongMod", 125, 11, 8),
    ("LongSlow", 125, 12, 8), ("LongTurbo", 500, 11, 8), ("ShortTurbo", 500, 7, 5),
    ("MediumTurbo", 500, 9, 5),
]

PAGES = ["Spectrum", "Waterfall", "Finder", "LoRaWAN", "Packets", "Map"]
P_SPEC, P_WATER, P_FIND, P_LWAN, P_PKTS, P_MAP = range(6)

FBIOGET_VSCREENINFO = 0x4600
FBIOGET_FSCREENINFO = 0x4602
EV_KEY = 0x01
KEY_ESC, KEY_1, KEY_2, KEY_3, KEY_4 = 1, 2, 3, 4, 5
KEY_MINUS, KEY_EQUAL, KEY_TAB, KEY_Q, KEY_ENTER = 12, 13, 15, 16, 28
KEY_M, KEY_N, KEY_Y = 50, 49, 21
KEY_SEMICOLON, KEY_COMMA, KEY_DOT, KEY_SLASH, KEY_SPACE = 39, 51, 52, 53, 57
KEY_KPMINUS, KEY_KPPLUS = 74, 78
KEY_UP, KEY_LEFT, KEY_RIGHT, KEY_DOWN = 103, 105, 106, 108
KEY_5, KEY_6 = 6, 7
KEY_B, KEY_D, KEY_F, KEY_K, KEY_S = 48, 32, 33, 37, 31
KEY_X, KEY_Z, KEY_C, KEY_T = 45, 44, 46, 20   # Cardputer arrows: F up, X down, Z left, C right
NUM_KEYS = {KEY_1: 0, KEY_2: 1, KEY_3: 2, KEY_4: 3, KEY_5: 4, KEY_6: 5}

BG = (10, 12, 16)
PANEL = (24, 28, 36)
GRID = (40, 46, 58)
SEL = (34, 52, 74)
TEXT = (230, 234, 240)
DIM = (130, 138, 152)
GREEN = (60, 220, 130)
CYAN = (60, 190, 255)
YELLOW = (255, 205, 60)
ORANGE = (255, 140, 50)
RED = (255, 75, 75)
KEY_RED = (235, 70, 70)   # the red arrow markings on the keyboard
PURPLE = (175, 125, 255)

_running = True


def _stop(_s, _f):
    global _running
    _running = False


# --------------------------------------------------------------------------
# Meshtastic channel math (verified: LongFast US -> 906.875, MediumFast -> 913.125)
# --------------------------------------------------------------------------
def djb2(name):
    h = 5381
    for c in name.encode():
        h = (h * 33 + c) & 0xFFFFFFFF
    return h


def preset_frequency(name, bw_khz, lo=BAND_LO, hi=BAND_HI):
    width = bw_khz / 1000.0
    slots = int(round((hi - lo) / width))
    slot = djb2(name) % slots
    return lo + bw_khz / 2000.0 + slot * width


def xor_hash(data):
    h = 0
    for b in data:
        h ^= b
    return h


def default_channel_hash(preset_name):
    return xor_hash(preset_name.encode()) ^ xor_hash(DEFAULT_PSK)


def parse_mesh_header(payload):
    """Meshtastic over-the-air header (16 bytes, little endian). Payload after it is encrypted."""
    if len(payload) < 16:
        return None
    to, frm, pid = struct.unpack_from("<III", payload, 0)
    flags, chash, next_hop, relay = payload[12], payload[13], payload[14], payload[15]
    hop_limit = flags & 0x07
    hop_start = (flags & 0xE0) >> 5
    return {"to": to, "from": frm, "id": pid, "hop_limit": hop_limit, "hop_start": hop_start,
            "hops": max(0, hop_start - hop_limit) if hop_start else None,
            "want_ack": bool(flags & 0x08), "via_mqtt": bool(flags & 0x10),
            "chash": chash, "relay": relay}


def node_id(n):
    return "^all" if n == 0xFFFFFFFF else "!%08x" % n


# --------------------------------------------------------------------------
# Decoders: protobuf, AES-128-CTR (built in, no extra packages),
# Meshtastic public channel, LoRaWAN headers
# --------------------------------------------------------------------------
def _xtime(a):
    a <<= 1
    return (a ^ 0x11B) & 0xFF if a & 0x100 else a


def _build_sbox():
    def rotl(x, s):
        return ((x << s) | (x >> (8 - s))) & 0xFF
    sbox = [0] * 256
    p = q = 1
    while True:
        p = p ^ ((p << 1) & 0xFF) ^ (0x1B if p & 0x80 else 0)          # p *= 3
        q ^= q << 1                                                      # q /= 3
        q ^= q << 2
        q ^= q << 4
        q &= 0xFF
        if q & 0x80:
            q ^= 0x09
        sbox[p] = (q ^ rotl(q, 1) ^ rotl(q, 2) ^ rotl(q, 3) ^ rotl(q, 4) ^ 0x63) & 0xFF
        if p == 1:
            break
    sbox[0] = 0x63
    return bytes(sbox)


SBOX = _build_sbox()


class AES128:
    """Encrypt-only AES-128 (all CTR mode needs)."""

    def __init__(self, key):
        rk = list(key)
        rcon = 1
        for i in range(16, 176, 4):
            t = rk[i - 4:i]
            if i % 16 == 0:
                t = [SBOX[t[1]] ^ rcon, SBOX[t[2]], SBOX[t[3]], SBOX[t[0]]]
                rcon = _xtime(rcon)
            rk += [rk[i - 16 + j] ^ t[j] for j in range(4)]
        self.rk = rk

    def encrypt_block(self, block):
        rk = self.rk
        s = [block[i] ^ rk[i] for i in range(16)]
        for r in range(1, 11):
            s = [SBOX[x] for x in s]
            s = [s[0], s[5], s[10], s[15], s[4], s[9], s[14], s[3],
                 s[8], s[13], s[2], s[7], s[12], s[1], s[6], s[11]]
            if r != 10:
                t = []
                for c in range(0, 16, 4):
                    a0, a1, a2, a3 = s[c:c + 4]
                    x = a0 ^ a1 ^ a2 ^ a3
                    t += [a0 ^ x ^ _xtime(a0 ^ a1), a1 ^ x ^ _xtime(a1 ^ a2),
                          a2 ^ x ^ _xtime(a2 ^ a3), a3 ^ x ^ _xtime(a3 ^ a0)]
                s = t
            k = rk[16 * r:16 * r + 16]
            s = [s[i] ^ k[i] for i in range(16)]
        return bytes(s)


def aes_ctr(key, nonce, data):
    aes = AES128(key)
    ctr = int.from_bytes(nonce, "big")
    out = bytearray()
    for i in range(0, len(data), 16):
        ks = aes.encrypt_block(ctr.to_bytes(16, "big"))
        ctr = (ctr + 1) & ((1 << 128) - 1)
        out += bytes(a ^ b for a, b in zip(data[i:i + 16], ks))
    return bytes(out)


# ---- protobuf --------------------------------------------------------------
def _rd_varint(buf, i):
    shift = r = 0
    while True:
        if i >= len(buf) or shift > 63:
            raise ValueError("varint")
        b = buf[i]
        i += 1
        r |= (b & 0x7F) << shift
        shift += 7
        if not b & 0x80:
            return r, i


def pb_decode(buf):
    out, i, n = {}, 0, len(buf)
    while i < n:
        key, i = _rd_varint(buf, i)
        num, wt = key >> 3, key & 7
        if num == 0:
            raise ValueError("field 0")
        if wt == 0:
            v, i = _rd_varint(buf, i)
        elif wt == 1:
            if i + 8 > n:
                raise ValueError("short")
            v = struct.unpack_from("<Q", buf, i)[0]
            i += 8
        elif wt == 2:
            ln, i = _rd_varint(buf, i)
            if i + ln > n:
                raise ValueError("short")
            v = bytes(buf[i:i + ln])
            i += ln
        elif wt == 5:
            if i + 4 > n:
                raise ValueError("short")
            v = struct.unpack_from("<I", buf, i)[0]
            i += 4
        else:
            raise ValueError("wire type")
        out.setdefault(num, []).append(v)
    return out


def pg(msg, num, default=None):
    v = msg.get(num)
    return v[0] if v else default


def pb_f32(v):
    return struct.unpack("<f", struct.pack("<I", v & 0xFFFFFFFF))[0]


def pb_i32(v):
    v &= 0xFFFFFFFF
    return v - (1 << 32) if v & 0x80000000 else v


def pb_str(v):
    return v.decode("utf-8", "replace") if isinstance(v, bytes) else ""


# ---- Meshtastic --------------------------------------------------------------
MESH_PORTS = {1: "text", 3: "position", 4: "node info", 5: "routing", 6: "admin",
              7: "compressed text", 8: "waypoint", 32: "reply", 34: "paxcounter",
              64: "serial", 65: "store & forward", 66: "range test", 67: "telemetry",
              68: "zps", 69: "simulator", 70: "traceroute", 71: "neighbor info",
              72: "ATAK", 73: "map report", 256: "private app"}
MESH_ROLES = {0: "client", 1: "client mute", 2: "router", 3: "router client", 4: "repeater",
              5: "tracker", 6: "sensor", 7: "TAK", 8: "client hidden", 9: "lost & found",
              10: "TAK tracker", 11: "router late"}
MESH_HW = {4: "T-Beam", 9: "RAK4631", 37: "Linux (portduino)", 43: "Heltec V3"}
DEFAULT_HASHES = {}


def default_hash_lookup(chash):
    if not DEFAULT_HASHES:
        for n, *_ in PRESETS:
            DEFAULT_HASHES.setdefault(default_channel_hash(n), n)
    return DEFAULT_HASHES.get(chash)


def mesh_decrypt(hdr, payload):
    nonce = struct.pack("<QI", hdr["id"], hdr["from"]) + b"\x00" * 4
    return aes_ctr(DEFAULT_PSK, nonce, payload)


def decode_mesh(data, nodes):
    """Return a decoded-packet dict for a Meshtastic over-the-air frame, or None."""
    hdr = parse_mesh_header(data)
    if not hdr:
        return None
    frm, to = hdr["from"], hdr["to"]

    def who(n):
        if n == 0xFFFFFFFF:
            return "everyone"
        nm = nodes.get(n, {}).get("short")
        return "{} ({})".format(node_id(n), nm) if nm else node_id(n)

    out = {"proto": "mesh", "hdr": hdr, "from": frm, "to": to,
           "fields": [("From", who(frm)), ("To", who(to)), ("Packet id", "0x%08x" % hdr["id"]),
                      ("Hops", "-" if hdr["hops"] is None else str(hdr["hops"])),
                      ("Hop limit left", str(hdr["hop_limit"])),
                      ("Channel hash", "0x%02x" % hdr["chash"]),
                      ("Relayed by", "…%02x" % hdr["relay"] if hdr["relay"] else "-"),
                      ("Flags", ", ".join(x for x, y in (("want ack", hdr["want_ack"]),
                                                         ("via MQTT", hdr["via_mqtt"])) if y) or "-")]}
    route = "{} → {}".format(who(frm), who(to))
    chan = default_hash_lookup(hdr["chash"])
    if chan is None:
        if hdr["chash"] == 0 and to != 0xFFFFFFFF:
            out.update(kind="private DM", summary="{} · private message (end-to-end encrypted)".format(route),
                       color="dim")
        else:
            out.update(kind="private ch", summary="{} · private channel 0x{:02x}".format(route, hdr["chash"]),
                       color="dim")
        return out
    out["fields"].append(("Channel", chan + " (public default key)"))
    try:
        plain = mesh_decrypt(hdr, data[16:])
        d = pb_decode(plain)
        port = pg(d, 1, 0)
        if port not in MESH_PORTS:
            raise ValueError("port")
        body = pg(d, 2, b"")
    except (ValueError, struct.error, IndexError):
        out.update(kind="undecodable", summary="{} · couldn't decrypt".format(route), color="dim")
        return out
    out["port"] = port
    out["fields"].append(("Type", "{} (port {})".format(MESH_PORTS[port], port)))
    kind, text, color = MESH_PORTS[port], MESH_PORTS[port], "text"
    try:
        if port == 1:
            msg = pb_str(body)
            out["text"] = msg
            out["fields"].append(("Message", msg))
            text, color = "“{}”".format(msg), "green"
        elif port == 4:
            u = pb_decode(body)
            long_, short = pb_str(pg(u, 2, b"")), pb_str(pg(u, 3, b""))
            hw, role = pg(u, 5, 0), pg(u, 7, 0)
            nodes.setdefault(frm, {}).update({k: v for k, v in (("long", long_), ("short", short)) if v})
            out["fields"] += [("Name", long_ or "-"), ("Short name", short or "-"),
                              ("Hardware", MESH_HW.get(hw, "model #{}".format(hw))),
                              ("Role", MESH_ROLES.get(role, str(role)))]
            text, color = "node info: {} “{}”".format(short or "?", long_ or "?"), "cyan"
            route = "{} → {}".format(who(frm), who(to))
        elif port == 3:
            p = pb_decode(body)
            lat, lon = pg(p, 1), pg(p, 2)
            if lat is not None and lon is not None:
                la, lo = pb_i32(lat) / 1e7, pb_i32(lon) / 1e7
                out["pos"] = (la, lo)
                out["fields"].append(("Shared position", "{:.2f}, {:.2f} (rounded)".format(la, lo)))
                if 3 in p:
                    out["fields"].append(("Altitude", "{} m".format(pb_i32(pg(p, 3)))))
                text = "position shared"
            else:
                text = "position (none set)"
        elif port == 67:
            t = pb_decode(body)
            if 2 in t:
                m = pb_decode(t[2][0])
                bits = []
                if 1 in m:
                    bl = pg(m, 1)
                    bits.append("battery {}%".format(bl) if bl <= 100 else "on USB power")
                if 2 in m:
                    bits.append("{:.2f} V".format(pb_f32(pg(m, 2))))
                if 3 in m:
                    bits.append("channel busy {:.1f}%".format(pb_f32(pg(m, 3))))
                if 5 in m:
                    bits.append("up {}h".format(pg(m, 5) // 3600))
                out["fields"].append(("Device", ", ".join(bits) or "-"))
                text = "telemetry: " + (", ".join(bits[:2]) or "device")
            elif 3 in t:
                m = pb_decode(t[3][0])
                bits = []
                if 1 in m:
                    bits.append("{:.1f}°C".format(pb_f32(pg(m, 1))))
                if 2 in m:
                    bits.append("{:.0f}% RH".format(pb_f32(pg(m, 2))))
                if 3 in m:
                    bits.append("{:.0f} hPa".format(pb_f32(pg(m, 3))))
                out["fields"].append(("Environment", ", ".join(bits) or "-"))
                text = "weather: " + (", ".join(bits) or "sensor")
            else:
                text = "telemetry"
        elif port == 70:
            r = pb_decode(body)
            hops = []
            for v in r.get(1, []):
                if isinstance(v, bytes):
                    hops += [struct.unpack_from("<I", v, k)[0] for k in range(0, len(v) - 3, 4)]
                else:
                    hops.append(v)
            out["fields"].append(("Route", " → ".join(node_id(h) for h in hops) or "direct"))
            text = "traceroute ({} hop{})".format(len(hops), "" if len(hops) == 1 else "s")
        elif port == 71:
            r = pb_decode(body)
            text = "neighbor info: {} neighbors".format(len(r.get(4, [])))
        elif port == 73:
            r = pb_decode(body)
            long_, short = pb_str(pg(r, 1, b"")), pb_str(pg(r, 2, b""))
            if short or long_:
                nodes.setdefault(frm, {}).update({k: v for k, v in (("long", long_), ("short", short)) if v})
            fw = pb_str(pg(r, 6, b""))
            out["fields"] += [("Name", long_ or "-"), ("Firmware", fw or "-")]
            text = "map report: {} {}".format(short or "", ("fw " + fw) if fw else "").strip()
        elif port == 5:
            r = pb_decode(body)
            err = pg(r, 3, 0)
            text = "ack" if not err else "routing error {}".format(err)
        elif port == 66:
            msg = pb_str(body)
            out["fields"].append(("Range test", msg))
            text = "range test “{}”".format(msg[:24])
        elif port == 8:
            r = pb_decode(body)
            text = "waypoint “{}”".format(pb_str(pg(r, 6, b""))[:24])
    except (ValueError, struct.error, IndexError):
        pass
    out.update(kind=kind, summary="{} · {}".format(route, text), color=color)
    return out


# ---- LoRaWAN ------------------------------------------------------------------
LW_MTYPES = ["Join request", "Join accept", "Unconfirmed up", "Unconfirmed down",
             "Confirmed up", "Confirmed down", "Rejoin request", "Proprietary"]
NWKID_BITS = [6, 6, 9, 11, 12, 13, 15, 17]
NETWORKS = {0x000000: "private/test", 0x000001: "private/test", 0x000013: "The Things Network",
            0x00003C: "Helium", 0x60002D: "Helium", 0xC00053: "Helium"}
EUI_VENDORS = {"70B3D57ED": "TTN-issued", "A84041": "Dragino", "2CF7F1": "Seeed"}


def netid_from_devaddr(addr):
    t = 0
    while t < 8 and addr & (0x80000000 >> t):
        t += 1
    if t > 7:
        return None
    nbits = NWKID_BITS[t]
    nwkid = (addr >> (32 - (t + 1) - nbits)) & ((1 << nbits) - 1)
    return (t << 21) | nwkid


def network_name(addr):
    nid = netid_from_devaddr(addr)
    if nid is None:
        return None, "unknown"
    return nid, NETWORKS.get(nid, "NetID %06X" % nid)


def eui_str(b):
    return b[::-1].hex().upper()


def eui_vendor(s):
    for k, v in EUI_VENDORS.items():
        if s.startswith(k):
            return v
    return None


def decode_lorawan(data):
    if len(data) < 5:
        return None
    mhdr = data[0]
    mtype, major = mhdr >> 5, mhdr & 0x03
    if major != 0:
        return None
    out = {"proto": "lwan", "mtype": mtype, "fields": [("Message type", LW_MTYPES[mtype])]}
    if mtype == 0:
        if len(data) != 23:
            return None
        join_eui, dev_eui = eui_str(data[1:9]), eui_str(data[9:17])
        nonce = struct.unpack_from("<H", data, 17)[0]
        v = eui_vendor(dev_eui)
        out["fields"] += [("DevEUI", dev_eui + (" (" + v + ")" if v else "")), ("JoinEUI", join_eui),
                          ("DevNonce", str(nonce)), ("MIC", data[19:23].hex())]
        out.update(kind="join", dev=dev_eui,
                   summary="Join request · device {}{}".format(dev_eui, " (" + v + ")" if v else ""),
                   color="cyan")
        return out
    if mtype == 1:
        if len(data) not in (17, 33):
            return None
        out.update(kind="join accept", summary="Join accept (encrypted) · {} bytes".format(len(data)),
                   color="cyan")
        return out
    if mtype == 7:
        out.update(kind="proprietary", summary="Proprietary frame · {} bytes".format(len(data)), color="dim")
        return out
    if mtype == 6 or len(data) < 12:
        out.update(kind="other", summary="{} · {} bytes".format(LW_MTYPES[mtype], len(data)), color="dim")
        return out
    addr = struct.unpack_from("<I", data, 1)[0]
    fctrl = data[5]
    fcnt = struct.unpack_from("<H", data, 6)[0]
    fopts_len = fctrl & 0x0F
    pos = 8 + fopts_len
    if pos + 4 > len(data):
        return None
    fport = data[pos] if pos < len(data) - 4 else None
    payload_len = max(0, len(data) - 4 - pos - (1 if fport is not None else 0))
    nid, net = network_name(addr)
    up = mtype in (2, 4)
    flags = [n for n, b in (("ADR", 0x80), ("ADR ack req" if up else "-", 0x40), ("ACK", 0x20),
                            ("class B" if up else "more data", 0x10)) if fctrl & b and n != "-"]
    out["fields"] += [("Device address", "%08X" % addr), ("Network", net),
                      ("Frame counter", str(fcnt)), ("Port", "-" if fport is None else str(fport)),
                      ("Encrypted payload", "{} bytes".format(payload_len)),
                      ("MAC options", "{} bytes".format(fopts_len)),
                      ("Flags", ", ".join(flags) or "-"), ("MIC", data[-4:].hex())]
    out.update(kind="data", addr=addr, net=net, up=up,
               summary="{} · {:08X} {} · #{}{}".format(
                   LW_MTYPES[mtype], addr, "(" + net + ")" if net else "", fcnt,
                   " · port {}".format(fport) if fport is not None else ""),
               color="text")
    return out


# US915 channel plan
def lw_uplink_channels(subband):
    base = (subband - 1) * 8
    chans = [{"ch": base + k, "freq": round(902.3 + 0.2 * (base + k), 3), "bw": 125,
              "sfs": [7, 8, 9, 10]} for k in range(8)]
    m = subband - 1
    chans.append({"ch": 64 + m, "freq": round(903.0 + 1.6 * m, 3), "bw": 500, "sfs": [8]})
    return chans


def lw_downlink_channels():
    return [{"ch": k, "freq": round(923.3 + 0.6 * k, 3), "bw": 500, "sfs": [7, 8, 9, 10, 11, 12]}
            for k in range(8)]


# --------------------------------------------------------------------------
# Linux SPI (spidev) and GPIO (character device v2) via ioctl - no extra packages
# --------------------------------------------------------------------------
SPI_IOC_MESSAGE_1 = 0x40206b00
SPI_IOC_WR_MODE = 0x40016b01
SPI_IOC_WR_BITS_PER_WORD = 0x40016b03
SPI_IOC_WR_MAX_SPEED_HZ = 0x40046b04


class SpiDev:
    def __init__(self, path, speed=SPI_SPEED):
        self.fd = os.open(path, os.O_RDWR)
        self.speed = speed
        fcntl.ioctl(self.fd, SPI_IOC_WR_MODE, struct.pack("B", 0))
        fcntl.ioctl(self.fd, SPI_IOC_WR_BITS_PER_WORD, struct.pack("B", 8))
        fcntl.ioctl(self.fd, SPI_IOC_WR_MAX_SPEED_HZ, struct.pack("I", speed))

    def xfer(self, data):
        n = len(data)
        tx = ctypes.create_string_buffer(bytes(data), n)
        rx = ctypes.create_string_buffer(n)
        # struct spi_ioc_transfer: tx_buf u64, rx_buf u64, len u32, speed_hz u32,
        # delay_usecs u16, bits_per_word u8, cs_change u8, tx_nbits u8, rx_nbits u8,
        # word_delay_usecs u8, pad u8  (32 bytes)
        msg = struct.pack("QQIIHBBBBBB", ctypes.addressof(tx), ctypes.addressof(rx), n,
                          self.speed, 0, 8, 0, 0, 0, 0, 0)
        fcntl.ioctl(self.fd, SPI_IOC_MESSAGE_1, msg)
        return rx.raw

    def close(self):
        os.close(self.fd)


GPIO_V2_GET_LINE_IOCTL = 0xc250b407
GPIO_V2_LINE_SET_VALUES_IOCTL = 0xc010b40f
GPIO_V2_LINE_GET_VALUES_IOCTL = 0xc010b40e
GPIO_FLAG_INPUT = 0x4
GPIO_FLAG_OUTPUT = 0x8
GPIO_ATTR_OUTPUT_VALUES = 2


class GpioLines:
    """Request lines with the GPIO character device v2 API (struct gpio_v2_line_request, 592 bytes)."""

    def __init__(self, chip, offsets, output, initial=0):
        self.offsets = list(offsets)
        req = bytearray(592)
        struct.pack_into("%dI" % len(offsets), req, 0, *offsets)          # offsets[64]
        req[256:256 + 32] = b"lorascan".ljust(32, b"\0")                   # consumer[32]
        flags = GPIO_FLAG_OUTPUT if output else GPIO_FLAG_INPUT
        struct.pack_into("Q", req, 288, flags)                             # config.flags
        if output:
            struct.pack_into("I", req, 296, 1)                             # config.num_attrs
            # config.attrs[0] at 288+32: attr{id u32, pad u32, values u64}, mask u64
            mask = (1 << len(offsets)) - 1
            struct.pack_into("IIQQ", req, 320, GPIO_ATTR_OUTPUT_VALUES, 0,
                             mask if initial else 0, mask)
        struct.pack_into("I", req, 560, len(offsets))                      # num_lines
        cfd = os.open(chip, os.O_RDWR)
        try:
            fcntl.ioctl(cfd, GPIO_V2_GET_LINE_IOCTL, req)
        finally:
            os.close(cfd)
        self.fd = struct.unpack_from("i", req, 588)[0]

    def get(self, i=0):
        buf = bytearray(struct.pack("QQ", 0, 1 << i))
        fcntl.ioctl(self.fd, GPIO_V2_LINE_GET_VALUES_IOCTL, buf)
        return (struct.unpack("QQ", buf)[0] >> i) & 1

    def set(self, value, i=0):
        bits = (1 << i) if value else 0
        fcntl.ioctl(self.fd, GPIO_V2_LINE_SET_VALUES_IOCTL, bytearray(struct.pack("QQ", bits, 1 << i)))

    def close(self):
        try:
            os.close(self.fd)
        except OSError:
            pass


# --------------------------------------------------------------------------
# Cap power + RF antenna switch (PI4IOE5V6408 P0 high), same as cap-lora-power
# --------------------------------------------------------------------------
I2C_SLAVE = 0x0703


def cap_power_on():
    """Returns (ok, message, previous_5v_value or None)."""
    prev = None
    if os.path.exists(EXT5V):
        try:
            with open(EXT5V) as f:
                prev = int(f.read().strip() or 0)
            if prev <= 0:
                with open(EXT5V, "w") as f:
                    f.write("1\n")
                time.sleep(0.2)
        except (OSError, ValueError) as e:
            return False, "Cap 5 V: {}".format(getattr(e, "strerror", e)), None
    try:
        fd = os.open(I2C_BUS, os.O_RDWR)
    except OSError as e:
        return False, "{}: {}".format(I2C_BUS, e.strerror), prev
    try:
        fcntl.ioctl(fd, I2C_SLAVE, PI4IO_ADDR)

        def rd(reg):
            os.write(fd, bytes([reg]))
            return os.read(fd, 1)[0]

        def wr(reg, val):
            os.write(fd, bytes([reg, val & 0xFF]))

        for _ in range(5):
            try:
                out, hiz, dr = rd(0x05), rd(0x07), rd(0x03)
                break
            except OSError:
                time.sleep(0.2)
        else:
            return False, "Cap not answering on I2C (is it attached?)", prev
        wr(0x05, out | 0x01)
        wr(0x07, hiz & 0xFE)
        wr(0x03, dr | 0x01)
        return True, "Cap powered, antenna switch on", prev
    except OSError as e:
        return False, "I2C: {}".format(e.strerror), prev
    finally:
        os.close(fd)


# --------------------------------------------------------------------------
# SX1262 driver (receive only)
# --------------------------------------------------------------------------
BW_CODE = {62.5: 0x03, 125: 0x04, 250: 0x05, 500: 0x06}
IRQ_RX_DONE, IRQ_PREAMBLE, IRQ_HEADER_VALID = 1 << 1, 1 << 2, 1 << 4
IRQ_HEADER_ERR, IRQ_CRC_ERR, IRQ_TIMEOUT = 1 << 5, 1 << 6, 1 << 9
IRQ_MASK = IRQ_RX_DONE | IRQ_PREAMBLE | IRQ_HEADER_VALID | IRQ_HEADER_ERR | IRQ_CRC_ERR | IRQ_TIMEOUT


class RadioError(Exception):
    pass


class SX1262:
    """Minimal SX1262 driver. It never issues SetTx - this tool cannot transmit."""

    def __init__(self, spi, busy, reset):
        self.spi, self.busy, self.reset_line = spi, busy, reset
        self.freq = None

    # -- low level --
    def wait_busy(self, timeout=0.1):
        end = time.monotonic() + timeout
        while self.busy.get():
            if time.monotonic() > end:
                raise RadioError("SX1262 BUSY stuck high (radio not responding)")
            time.sleep(0.0002)

    def cmd(self, op, params=()):
        assert op != 0x83, "SetTx is never used"   # receive-only tool
        self.wait_busy()
        self.spi.xfer([op] + list(params))

    def cmd_read(self, op, n, params=()):
        """Get* commands: opcode, params, then a status byte, then n data bytes."""
        self.wait_busy()
        rx = self.spi.xfer([op] + list(params) + [0] * (1 + n))
        return rx[1 + len(params) + 1:]

    def write_reg(self, addr, data):
        self.cmd(0x0D, [addr >> 8, addr & 0xFF] + list(data))

    def read_reg(self, addr, n=1):
        self.wait_busy()
        rx = self.spi.xfer([0x1D, addr >> 8, addr & 0xFF, 0] + [0] * n)
        return rx[4:4 + n]

    # -- setup --
    def hard_reset(self):
        self.reset_line.set(0)
        time.sleep(0.002)
        self.reset_line.set(1)
        time.sleep(0.01)
        self.wait_busy(0.5)

    def init(self):
        self.hard_reset()
        self.cmd(0x80, [0x00])                                   # SetStandby(STDBY_RC)
        self.cmd(0x97, [TCXO_VOLTAGE_CODE, 0x00, 0x01, 0x40])    # SetDIO3AsTcxoCtrl 3.0 V, 5 ms
        self.cmd(0x89, [0x7F])                                   # Calibrate(all)
        time.sleep(0.005)
        self.wait_busy(0.5)
        self.cmd(0x07, [0x00, 0x00])                             # ClearDeviceErrors
        self.cmd(0x96, [0x01])                                   # SetRegulatorMode(DC-DC)
        self.cmd(0x9D, [0x01])                                   # SetDIO2AsRfSwitchCtrl
        self.cmd(0x80, [0x01])                                   # SetStandby(STDBY_XOSC)
        self.cmd(0x8A, [0x01])                                   # SetPacketType(LoRa)
        self.set_frequency(915.0)
        self.cmd(0x98, [0xE1, 0xE9])                             # CalibrateImage 902-928 MHz
        time.sleep(0.005)
        self.wait_busy(0.5)
        self.cmd(0x8F, [0x00, 0x00])                             # SetBufferBaseAddress
        self.write_reg(0x08AC, [0x96])                           # RX boosted gain
        self.set_sync_word(MESH_SYNC_WORD)
        self.cmd(0x08, [IRQ_MASK >> 8, IRQ_MASK & 0xFF, 0, 0, 0, 0, 0, 0])  # SetDioIrqParams
        sw = self.read_reg(0x0740, 2)
        if sw != bytes([0x24, 0xB4]):
            raise RadioError("SX1262 not answering on SPI (read {} expected 24b4)".format(sw.hex()))
        errs = self.cmd_read(0x17, 2)                            # GetDeviceErrors
        err = (errs[0] << 8) | errs[1]
        if err & 0x20:
            raise RadioError("SX1262 oscillator (TCXO) failed to start, errors=0x%04x" % err)
        return err

    def set_sync_word(self, sw):
        # RadioLib mapping with control bits 0x44: 0x2B -> 0x24 0xB4
        self.write_reg(0x0740, [(sw & 0xF0) | 0x04, ((sw & 0x0F) << 4) | 0x04])

    @staticmethod
    def freq_reg(mhz):
        return int(round(mhz * 1e6 * (1 << 25) / 32e6))

    def set_frequency(self, mhz):
        r = self.freq_reg(mhz)
        self.cmd(0x86, [(r >> 24) & 0xFF, (r >> 16) & 0xFF, (r >> 8) & 0xFF, r & 0xFF])
        self.freq = mhz

    def set_lora(self, bw_khz, sf, cr, preamble=MESH_PREAMBLE, crc=True, iq_inv=False):
        sym_ms = (1 << sf) / bw_khz
        ldro = 1 if sym_ms >= 16.0 else 0
        self.cmd(0x8B, [sf, BW_CODE[bw_khz], cr - 4, ldro])      # SetModulationParams
        # SetPacketParams: preamble, explicit header, max length, CRC, IQ
        self.cmd(0x8C, [preamble >> 8, preamble & 0xFF, 0x00, 0xFF, 0x01 if crc else 0x00,
                        0x01 if iq_inv else 0x00])
        # Errata 15.1: modulation quality for 500 kHz; 15.4: IQ polarity register
        r = self.read_reg(0x0889, 1)[0]
        self.write_reg(0x0889, [(r & ~0x04) if bw_khz == 500 else (r | 0x04)])
        r = self.read_reg(0x0736, 1)[0]
        self.write_reg(0x0736, [(r & ~0x04) if iq_inv else (r | 0x04)])

    def standby(self):
        self.cmd(0x80, [0x01])

    def rx_continuous(self):
        self.clear_irq()
        self.cmd(0x82, [0xFF, 0xFF, 0xFF])

    def rssi_inst(self):
        return -self.cmd_read(0x15, 1)[0] / 2.0

    def irq(self):
        d = self.cmd_read(0x12, 2)
        return (d[0] << 8) | d[1]

    def clear_irq(self, mask=0xFFFF):
        self.cmd(0x02, [mask >> 8, mask & 0xFF])

    def read_packet(self):
        ln, start = self.cmd_read(0x13, 2)
        self.wait_busy()
        rx = self.spi.xfer([0x1E, start, 0] + [0] * ln)
        data = rx[3:3 + ln]
        st = self.cmd_read(0x14, 3)
        rssi = -st[0] / 2.0
        snr = struct.unpack("b", bytes([st[1]]))[0] / 4.0
        return data, rssi, snr

    def sleep(self):
        try:
            self.cmd(0x84, [0x00])   # SetSleep, cold start (meshtasticd resets it anyway)
        except Exception:
            pass


# --------------------------------------------------------------------------
# Hardware session: other programs using the radio, power, open devices
# --------------------------------------------------------------------------
RADIO_USERS = ("meshtasticd", "meshtastic", "pocketmesh", "cardputerzero-mesh")


def radio_in_use():
    """Name of another running program that drives the LoRa radio (e.g. Meshtastic), or None."""
    me = os.getpid()
    for pdir in glob.glob("/proc/[0-9]*"):
        try:
            pid = int(os.path.basename(pdir))
            if pid == me:
                continue
            with open(pdir + "/comm") as f:
                comm = f.read().strip()
            with open(pdir + "/cmdline", "rb") as f:
                cmd = f.read().replace(b"\x00", b" ").decode("utf-8", "replace").lower()
        except (OSError, ValueError):
            continue
        if "lorascan" in cmd:
            continue
        for name in RADIO_USERS:
            if name in comm.lower() or name in cmd:
                return comm or name
    return None


class HardwareRadio:
    def __init__(self):
        self.spi = self.busy = self.reset = None
        self.radio = None
        self.prev_5v = None

    def open(self):
        ok, msg, self.prev_5v = cap_power_on()
        if not ok:
            raise RadioError(msg)
        try:
            self.spi = SpiDev(SPI_DEV)
        except OSError as e:
            raise RadioError("{}: {}".format(SPI_DEV, e.strerror))
        try:
            self.busy = GpioLines(GPIO_CHIP, [PIN_BUSY], output=False)
            self.reset = GpioLines(GPIO_CHIP, [PIN_RESET], output=True, initial=1)
        except OSError as e:
            raise RadioError("GPIO busy/reset lines: {} (is Meshtastic or another LoRa app "
                             "still running?)".format(e.strerror))
        self.radio = SX1262(self.spi, self.busy, self.reset)
        self.radio.init()
        return self.radio

    def close(self):
        if self.radio:
            self.radio.sleep()
        for x in (self.busy, self.reset, self.spi):
            if x:
                try:
                    x.close()
                except Exception:
                    pass
        self.radio = None


# --------------------------------------------------------------------------
# GPS (the Cap's GNSS receiver on /dev/ttyS0, as in Trail Recorder)
# --------------------------------------------------------------------------
GPS_DEVICE = os.environ.get("LORASCAN_GPS_DEV", "/dev/ttyS0")


def _nmea_ok(line):
    if "*" not in line:
        return False
    body, _, cs = line[1:].partition("*")
    try:
        want = int(cs[:2], 16)
    except ValueError:
        return False
    got = 0
    for ch in body:
        got ^= ord(ch)
    return got == want


def _nmea_latlon(v, hemi):
    if not v or "." not in v:
        return None
    try:
        dot = v.index(".")
        val = float(v[:dot - 2]) + float(v[dot - 2:]) / 60.0
    except ValueError:
        return None
    return -val if hemi in ("S", "W") else val


def haversine(lat1, lon1, lat2, lon2):
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dp, dl = p2 - p1, math.radians(lon2 - lon1)
    a = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    return 2 * 6371008.8 * math.asin(min(1.0, math.sqrt(a)))


class GpsReader:
    def __init__(self, path=GPS_DEVICE):
        self.path = path
        self.lat = self.lon = None
        self.quality = 0
        self.valid = False
        self.sats = 0
        self.hdop = None
        self.last_fix = 0.0
        self.last_data = 0.0
        self.in_view = {}        # constellation talker -> (satellites in view, time)
        self.error = None
        self.stop_flag = False
        self.track = []          # (t, lat, lon) every ~10 s / 20 m while listening
        self.lock = threading.Lock()
        self.t = threading.Thread(target=self._run, daemon=True)

    @property
    def has_fix(self):
        return self.valid and self.quality > 0 and self.lat is not None and time.monotonic() - self.last_fix < 5

    def position(self):
        return (self.lat, self.lon) if self.has_fix else None

    def sats_in_view(self):
        now = time.monotonic()
        return sum(n for n, t in self.in_view.values() if now - t < 5)

    def status(self):
        view = self.sats_in_view()
        if self.has_fix:
            return "GPS fix · using {} of {} satellites".format(self.sats, max(view, self.sats)), GREEN
        if self.error:
            return "GPS: " + self.error, ORANGE
        if time.monotonic() - self.last_data < 3:
            if view:
                return "GPS searching… {} satellites in view".format(view), YELLOW
            return "GPS searching… no satellites yet (needs sky view)", YELLOW
        return "GPS: no data yet", DIM

    def start(self):
        self.t.start()

    def feed(self, line):
        line = line.strip()
        if not line.startswith("$") or not _nmea_ok(line):
            return
        self.last_data = time.monotonic()
        p = line.split("*")[0].split(",")
        kind = p[0][3:]
        try:
            if kind == "GSV" and len(p) > 3 and p[3]:
                self.in_view[p[0][1:3]] = (int(p[3]), time.monotonic())
            elif kind == "GGA" and len(p) > 9:
                self.quality = int(p[6] or 0)
                self.sats = int(p[7] or 0)
                self.hdop = float(p[8]) if p[8] else self.hdop
                if self.quality:
                    la, lo = _nmea_latlon(p[2], p[3]), _nmea_latlon(p[4], p[5])
                    if la is not None and lo is not None:
                        self.lat, self.lon, self.last_fix = la, lo, time.monotonic()
            elif kind == "RMC" and len(p) > 6:
                self.valid = p[2] == "A"
                if self.valid:
                    la, lo = _nmea_latlon(p[3], p[4]), _nmea_latlon(p[5], p[6])
                    if la is not None and lo is not None:
                        self.lat, self.lon, self.last_fix = la, lo, time.monotonic()
        except ValueError:
            return
        if self.has_fix:
            with self.lock:
                if not self.track or (time.time() - self.track[-1][0] > 10 and
                                      haversine(self.track[-1][1], self.track[-1][2], self.lat, self.lon) > 20):
                    self.track.append((time.time(), self.lat, self.lon))

    def _run(self):
        try:
            fd = os.open(self.path, os.O_RDONLY | os.O_NOCTTY | os.O_NONBLOCK)
        except OSError as e:
            self.error = "{} ({})".format(e.strerror.lower(), self.path) if e.errno != 13 else \
                "no permission (reboot once after installing)"
            return
        saved = None
        try:
            try:
                import termios
                attrs = termios.tcgetattr(fd)
                saved = list(attrs)
                cc = list(attrs[6])
                cc[termios.VMIN], cc[termios.VTIME] = 0, 0
                termios.tcsetattr(fd, termios.TCSANOW, [0, 0, termios.CS8 | termios.CREAD | termios.CLOCAL,
                                                        0, termios.B115200, termios.B115200, cc])
            except Exception:
                saved = None
            buf = b""
            while not self.stop_flag:
                r, _, _ = select.select([fd], [], [], 0.5)
                if not r:
                    continue
                try:
                    chunk = os.read(fd, 4096)
                except (BlockingIOError, InterruptedError):
                    continue
                if not chunk:
                    time.sleep(0.2)
                    continue
                buf = (buf + chunk)[-8192:]
                *lines, buf = buf.split(b"\n")
                for ln in lines:
                    self.feed(ln.decode("ascii", "replace"))
        finally:
            if saved:
                try:
                    import termios
                    termios.tcsetattr(fd, termios.TCSANOW, saved)
                except Exception:
                    pass
            os.close(fd)


class NoGps:
    error = "off"
    track = []
    lock = threading.Lock()
    has_fix = False

    def position(self):
        return None

    def status(self):
        return "GPS off", DIM

    def start(self):
        pass


# --------------------------------------------------------------------------
# Scanner engine (background thread)
# --------------------------------------------------------------------------
LW_SYNC_WORD = 0x34
LW_PREAMBLE = 8


class Scanner:
    def __init__(self, radio_factory, gps=None):
        self.radio_factory = radio_factory
        self.gps = gps or NoGps()
        self.lock = threading.Lock()
        self.mode = "sweep"            # sweep | find | lwan
        self.listen_mode = "find"      # what the Packets / Map pages listen with
        self.sweep = [None] * SWEEP_POINTS
        self.maxhold = [None] * SWEEP_POINTS
        self.waterfall = deque(maxlen=100)
        self.sweeps_done = 0
        self.sweep_pos = 0
        self.presets = [{"name": n, "bw": bw, "sf": sf, "cr": cr,
                         "freq": preset_frequency(n, bw), "preambles": 0, "headers": 0,
                         "packets": 0, "bad": 0, "last_rssi": None, "last_t": None, "listen_s": 0.0}
                        for n, bw, sf, cr in PRESETS]
        self.current = 0
        self.locked = False
        self.dwell = 15.0
        self.dwell_start = time.monotonic()
        # LoRaWAN listening
        self.lw_subband = 2
        self.lw_dir = "up"
        self.lw_lock_row = None        # index into lw_rows(), or None = hop all channels
        self.lw_sf = None              # None = all spreading factors
        self.lw_dwell = 2.0
        self.lw_pos = 0
        self.lw_cells = {}             # (dir, subband, ch, sf) -> {"pkts", "pre", "last_rssi", "last_t"}
        self.lw_cur = None
        self.packets = deque(maxlen=300)
        self.nodes = {}
        self.coverage = []             # packets heard with a GPS position (this session)
        self.error = None
        self.status = "starting"
        self.thread = None
        self.stop_flag = False
        self.log_path = None
        self.counts = {"mesh": 0, "lwan": 0, "other": 0}

    # -- control --
    def start(self):
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def stop(self):
        self.stop_flag = True
        if self.thread:
            self.thread.join(timeout=5)

    def set_mode(self, mode):
        with self.lock:
            if mode in ("find", "lwan"):
                self.listen_mode = mode
            if mode != self.mode:
                self.mode = mode
                self.dwell_start = time.monotonic()
                self._reconfigure = True

    def select(self, idx):
        with self.lock:
            self.current = idx % len(self.presets)
            self.dwell_start = time.monotonic()
            self._reconfigure = True

    # -- LoRaWAN plan --
    def lw_rows(self):
        return lw_uplink_channels(self.lw_subband) if self.lw_dir == "up" else lw_downlink_channels()

    def lw_combos(self):
        rows = self.lw_rows()
        idxs = [self.lw_lock_row] if self.lw_lock_row is not None and self.lw_lock_row < len(rows) \
            else range(len(rows))
        combos = []
        for i in idxs:
            sfs = rows[i]["sfs"]
            if self.lw_sf in sfs:
                sfs = [self.lw_sf]
            combos += [(i, sf) for sf in sfs]
        return combos

    def lw_cell(self, row, sf):
        key = (self.lw_dir, self.lw_subband if self.lw_dir == "up" else 0, row["ch"], sf)
        return self.lw_cells.setdefault(key, {"pkts": 0, "pre": 0, "last_rssi": None, "last_t": None})

    def lw_changed(self):
        with self.lock:
            self.lw_pos = 0
            self.dwell_start = time.monotonic()
            self._reconfigure = True

    # -- worker --
    _reconfigure = True

    def _run(self):
        try:
            radio = self.radio_factory()
        except RadioError as e:
            self.error = str(e)
            self.status = "error"
            return
        except Exception as e:
            self.error = "radio init: {}".format(e)
            self.status = "error"
            return
        self.status = "running"
        configured = None
        try:
            while not self.stop_flag:
                with self.lock:
                    mode = self.mode
                    reconf = self._reconfigure
                    self._reconfigure = False
                if mode == "sweep":
                    if configured != "sweep" or reconf:
                        radio.standby()
                        radio.set_sync_word(MESH_SYNC_WORD)
                        radio.set_lora(125, 7, 5)
                        configured = "sweep"
                    self._sweep_step(radio)
                elif mode == "find":
                    if configured != ("find", self.current) or reconf:
                        p = self.presets[self.current]
                        radio.standby()
                        radio.set_sync_word(MESH_SYNC_WORD)
                        radio.set_lora(p["bw"], p["sf"], p["cr"])
                        radio.set_frequency(p["freq"])
                        radio.rx_continuous()
                        configured = ("find", self.current)
                    self._listen(radio)
                else:
                    with self.lock:
                        combos = self.lw_combos()
                        ri, sf = combos[self.lw_pos % len(combos)]
                        row = self.lw_rows()[ri]
                        up = self.lw_dir == "up"
                    key = ("lwan", self.lw_dir, self.lw_subband, row["ch"], sf)
                    if configured != key or reconf:
                        radio.standby()
                        radio.set_sync_word(LW_SYNC_WORD)
                        radio.set_lora(row["bw"], sf, 5, preamble=LW_PREAMBLE, crc=up, iq_inv=not up)
                        radio.set_frequency(row["freq"])
                        radio.rx_continuous()
                        configured = key
                        self._rx_since = None
                        with self.lock:
                            self.lw_cur = (row["ch"], sf)
                    self._listen_lwan(radio, row, sf, len(combos))
        except RadioError as e:
            self.error = str(e)
            self.status = "error"
        except Exception as e:
            self.error = "radio: {}".format(e)
            self.status = "error"

    def _sweep_step(self, radio):
        """Measure a handful of points, then return so mode changes stay responsive."""
        for _ in range(8):
            i = self.sweep_pos
            f = BAND_LO + SWEEP_STEP / 2 + i * SWEEP_STEP
            radio.standby()
            radio.set_frequency(f)
            radio.rx_continuous()
            time.sleep(0.002)
            vals = []
            for _ in range(6):
                vals.append(radio.rssi_inst())
                time.sleep(0.0008)
            peak = max(vals)
            with self.lock:
                self.sweep[i] = peak
                mh = self.maxhold[i]
                self.maxhold[i] = peak if mh is None else max(mh, peak)
            self.sweep_pos = (i + 1) % SWEEP_POINTS
            if self.sweep_pos == 0:
                with self.lock:
                    self.waterfall.appendleft(list(self.sweep))
                    self.sweeps_done += 1
                return

    def _listen(self, radio):
        p = self.presets[self.current]
        t0 = time.monotonic()
        while time.monotonic() - t0 < 0.25 and not self.stop_flag:
            flags = radio.irq()
            if flags:
                radio.clear_irq(flags)
                now = time.time()
                with self.lock:
                    if flags & IRQ_PREAMBLE:
                        p["preambles"] += 1
                        p["last_t"] = now
                    if flags & IRQ_HEADER_VALID:
                        p["headers"] += 1
                if flags & IRQ_RX_DONE:
                    data, rssi, snr = radio.read_packet()
                    crc_bad = bool(flags & (IRQ_CRC_ERR | IRQ_HEADER_ERR))
                    with self.lock:
                        if crc_bad:
                            p["bad"] += 1
                        else:
                            p["packets"] += 1
                        p["last_rssi"] = rssi
                        p["last_t"] = now
                    self._record({"mode": "mesh", "label": p["name"], "freq": p["freq"], "sf": p["sf"],
                                  "bw": p["bw"]}, data, rssi, snr, crc_bad, now)
                elif flags & (IRQ_CRC_ERR | IRQ_HEADER_ERR):
                    with self.lock:
                        p["bad"] += 1
            time.sleep(0.01)
        with self.lock:
            p["listen_s"] += time.monotonic() - t0
            if not self.locked and time.monotonic() - self.dwell_start >= self.dwell:
                self.current = (self.current + 1) % len(self.presets)
                self.dwell_start = time.monotonic()
                self._reconfigure = True

    def _listen_lwan(self, radio, row, sf, ncombos):
        cell = self.lw_cell(row, sf)
        t0 = time.monotonic()
        receiving = getattr(self, "_rx_since", None)
        max_air = 0.4 * (1 << max(0, sf - 7)) * (125.0 / row["bw"])   # generous packet airtime
        while time.monotonic() - t0 < 0.25 and not self.stop_flag:
            flags = radio.irq()
            if flags:
                radio.clear_irq(flags)
                now = time.time()
                if flags & (IRQ_PREAMBLE | IRQ_HEADER_VALID):
                    receiving = time.monotonic()
                    with self.lock:
                        if flags & IRQ_PREAMBLE:
                            cell["pre"] += 1
                if flags & IRQ_RX_DONE:
                    receiving = None
                    data, rssi, snr = radio.read_packet()
                    crc_bad = bool(flags & (IRQ_CRC_ERR | IRQ_HEADER_ERR))
                    if not crc_bad:
                        with self.lock:
                            cell["pkts"] += 1
                            cell["last_rssi"], cell["last_t"] = rssi, now
                    self._record({"mode": "lwan", "label": "ch{} SF{}{}".format(
                        row["ch"], sf, "" if self.lw_dir == "up" else " ↓"),
                        "freq": row["freq"], "sf": sf, "bw": row["bw"]}, data, rssi, snr, crc_bad, now)
                elif flags & (IRQ_CRC_ERR | IRQ_HEADER_ERR):
                    receiving = None
            time.sleep(0.005)
        busy = receiving is not None and time.monotonic() - receiving < max_air
        self._rx_since = receiving if busy else None
        with self.lock:
            # don't hop away in the middle of a packet
            if ncombos > 1 and not busy and time.monotonic() - self.dwell_start >= self.lw_dwell:
                self.lw_pos = (self.lw_pos + 1) % ncombos
                self.dwell_start = time.monotonic()
                self._reconfigure = True

    def _record(self, ctx, data, rssi, snr, crc_bad, now):
        dec = None
        if not crc_bad:
            try:
                dec = decode_mesh(data, self.nodes) if ctx["mode"] == "mesh" else decode_lorawan(data)
            except Exception:
                dec = None
        proto = dec["proto"] if dec else "other"
        pos = self.gps.position()
        entry = dict(ctx, t=now, rssi=rssi, snr=snr, len=len(data), crc_bad=crc_bad, data=bytes(data),
                     dec=dec, proto=proto, rx_pos=pos)
        with self.lock:
            self.packets.appendleft(entry)
            if not crc_bad:
                self.counts[proto] = self.counts.get(proto, 0) + 1
            if pos and not crc_bad:
                self.coverage.append(entry)
        self._log(entry)

    def _log(self, e):
        try:
            os.makedirs(LOG_DIR, exist_ok=True)
            path = os.path.join(LOG_DIR, "packets.csv")
            if os.path.exists(path):
                with open(path) as f:
                    first = f.readline()
                if first.startswith("time,preset,"):          # 1.0 format: keep it aside
                    os.replace(path, os.path.join(LOG_DIR, "packets-1.0.csv"))
            new = not os.path.exists(path)
            with open(path, "a") as f:
                if new:
                    f.write("time,listening,channel,freq_mhz,sf,bw_khz,rssi_dbm,snr_db,length,crc_ok,"
                            "protocol,summary,rx_lat,rx_lon,hex\n")
                dec = e["dec"] or {}
                summ = (dec.get("summary") or ("damaged" if e["crc_bad"] else "")).replace('"', "'")
                pos = e["rx_pos"]
                f.write('{},{},{},{:.3f},{},{},{:.1f},{:.2f},{},{},{},"{}",{},{},{}\n'.format(
                    datetime.fromtimestamp(e["t"]).isoformat(timespec="seconds"), e["mode"], e["label"],
                    e["freq"], e["sf"], e["bw"], e["rssi"], e["snr"], e["len"], int(not e["crc_bad"]),
                    e["proto"], summ, "%.6f" % pos[0] if pos else "", "%.6f" % pos[1] if pos else "",
                    e["data"].hex()))
            self.log_path = path
        except OSError:
            pass


def _kml_color(rssi):
    r, g, b = level_color(rssi)
    return "ff{:02x}{:02x}{:02x}".format(b, g, r)


def write_kml(scanner, path=None):
    """Coverage map for Google Earth: packets heard (colored by signal) + where you listened."""
    path = path or os.path.join(LOG_DIR, "coverage-{}.kml".format(datetime.now().strftime("%Y-%m-%d_%H%M")))
    with scanner.lock:
        pts = list(scanner.coverage)
    with scanner.gps.lock:
        track = list(scanner.gps.track)

    def x(s):
        return (s.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;"))
    out = ['<?xml version="1.0" encoding="UTF-8"?>',
           '<kml xmlns="http://www.opengis.net/kml/2.2"><Document>',
           "<name>LoRa coverage {}</name>".format(datetime.now().strftime("%Y-%m-%d %H:%M"))]
    if len(track) >= 2:
        out.append("<Placemark><name>Where I listened</name><Style><LineStyle><color>ff9a9a9a</color>"
                   "<width>3</width></LineStyle></Style><LineString><tessellate>1</tessellate><coordinates>")
        out.append(" ".join("{:.6f},{:.6f},0".format(lo, la) for _, la, lo in track))
        out.append("</coordinates></LineString></Placemark>")
    for e in pts:
        la, lo = e["rx_pos"]
        dec = e["dec"] or {}
        name = "{:.0f} dBm".format(e["rssi"])
        desc = "{}<br/>{} {:.3f} MHz SF{}<br/>RSSI {:.0f} dBm, SNR {:+.1f} dB<br/>{}".format(
            datetime.fromtimestamp(e["t"]).strftime("%Y-%m-%d %H:%M:%S"), x(e["label"]), e["freq"], e["sf"],
            e["rssi"], e["snr"], x(dec.get("summary", e["proto"])))
        out.append("<Placemark><name>{}</name><description><![CDATA[{}]]></description><Style><IconStyle>"
                   "<color>{}</color><scale>0.8</scale><Icon><href>http://maps.google.com/mapfiles/kml/"
                   "shapes/shaded_dot.png</href></Icon></IconStyle></Style><Point><coordinates>{:.6f},{:.6f},0"
                   "</coordinates></Point></Placemark>".format(name, desc, _kml_color(e["rssi"]), lo, la))
    out.append("</Document></kml>")
    os.makedirs(LOG_DIR, exist_ok=True)
    with open(path, "w") as f:
        f.write("\n".join(out) + "\n")
    return path, len(pts), len(track)


class SimRadio:
    """Stand-in radio for previews and tests (produces real, decodable frames)."""

    def __init__(self, seed=1):
        self.rng = random.Random(seed)
        self.freq = 915.0
        self.cfg = None
        self.sync = MESH_SYNC_WORD

    def standby(self):
        pass

    def set_sync_word(self, sw):
        self.sync = sw

    def set_lora(self, bw, sf, cr, preamble=MESH_PREAMBLE, crc=True, iq_inv=False):
        self.cfg = (bw, sf, cr, iq_inv)

    def set_frequency(self, f):
        self.freq = f

    def rx_continuous(self):
        pass

    def rssi_inst(self):
        v = -118 + self.rng.gauss(0, 1.5)
        for c, amp, w in ((913.125, 18, 0.15), (906.875, 9, 0.15), (904.3, 14, 0.1), (915.0, 6, 1.5)):
            if abs(self.freq - c) < w and self.rng.random() < 0.6:
                v = max(v, -118 + amp + self.rng.gauss(0, 2))
        return v

    def _active(self):
        if not self.cfg:
            return False
        if self.sync == MESH_SYNC_WORD:
            return abs(self.freq - 906.875) < 0.01 or abs(self.freq - 913.125) < 0.01
        return self.cfg[1] in (9, 10) and not self.cfg[3]

    def irq(self):
        if self._active() and self.rng.random() < 0.08:
            return IRQ_PREAMBLE | IRQ_HEADER_VALID | IRQ_RX_DONE
        if self.rng.random() < 0.01:
            return IRQ_PREAMBLE
        return 0

    def clear_irq(self, m=0xFFFF):
        pass

    def read_packet(self):
        rssi, snr = -104 + self.rng.gauss(0, 6), 2.5 + self.rng.gauss(0, 3)
        if self.sync == LW_SYNC_WORD:
            return sim_lorawan_frame(self.rng), rssi, snr
        name = "LongFast" if abs(self.freq - 906.875) < 0.01 else "MediumFast"
        return sim_mesh_frame(self.rng, name), rssi, snr

    def sleep(self):
        pass


def _pb_varint(n):
    out = bytearray()
    while True:
        b = n & 0x7F
        n >>= 7
        out.append(b | (0x80 if n else 0))
        if not n:
            return bytes(out)


def _pbf(num, wt, v):
    k = _pb_varint((num << 3) | wt)
    if wt == 0:
        return k + _pb_varint(v & ((1 << 64) - 1))
    if wt == 2:
        return k + _pb_varint(len(v)) + v
    return k + struct.pack("<I", v & 0xFFFFFFFF)


SIM_NODES = [(0xa1b2c3d4, "Tyler Ridge Repeater", "TRR"), (0x5e6f7081, "Ann's T-Echo", "ANN"),
             (0x0badf00d, "Base Station", "BASE")]


def sim_mesh_frame(rng, preset="LongFast", port=None):
    num, long_, short = rng.choice(SIM_NODES)
    port = port or rng.choice([1, 1, 4, 67, 3, 70])
    if port == 1:
        body = rng.choice(["Anyone on tonight?", "Testing from the water tower", "Good morning mesh!"]).encode()
    elif port == 4:
        body = _pbf(1, 2, b"!%08x" % num) + _pbf(2, 2, long_.encode()) + _pbf(3, 2, short.encode()) + \
            _pbf(5, 0, 43) + _pbf(7, 0, 2 if short == "TRR" else 0)
    elif port == 67:
        body = _pbf(1, 5, int(time.time())) + _pbf(2, 2, _pbf(1, 0, 87) + _pbf(2, 5, struct.unpack(
            "<I", struct.pack("<f", 4.02))[0]) + _pbf(3, 5, struct.unpack("<I", struct.pack("<f", 3.5))[0]))
    elif port == 3:
        body = _pbf(1, 5, int(41.88 * 1e7)) + _pbf(2, 5, int(-87.63 * 1e7) & 0xFFFFFFFF) + _pbf(3, 0, 190)
    else:
        body = _pbf(1, 2, struct.pack("<II", 0x5e6f7081, 0x0badf00d))
    data = _pbf(1, 0, port) + _pbf(2, 2, body)
    pid = rng.getrandbits(32)
    hdr = {"id": pid, "from": num}
    enc = aes_ctr(DEFAULT_PSK, struct.pack("<QI", pid, num) + b"\x00" * 4, data)
    return struct.pack("<III", 0xFFFFFFFF, num, pid) + bytes([(3 << 5) | 2, default_channel_hash(preset), 0, num & 0xFF]) + enc


def sim_lorawan_frame(rng):
    k = rng.random()
    if k < 0.15:
        return bytes([0x00]) + bytes.fromhex("0000000000000000")[::-1] + \
            bytes.fromhex("A840417E61829A3C")[::-1] + struct.pack("<H", rng.getrandbits(16)) + os.urandom(4)
    addr = rng.choice([0x260C1A2B, 0x78000412, 0xFC014D55, 0x00ABCDEF])
    mtype = 4 if k > 0.85 else 2
    return bytes([mtype << 5]) + struct.pack("<IBH", addr, 0x80, rng.randint(1, 9000)) + \
        bytes([rng.choice([1, 2, 10])]) + os.urandom(rng.randint(4, 20)) + os.urandom(4)


# --------------------------------------------------------------------------
# Drawing
# --------------------------------------------------------------------------
def load_font(size, bold=False):
    name = "DejaVuSans-Bold.ttf" if bold else "DejaVuSans.ttf"
    for path in (os.path.join(FONT_DIR, name), "/usr/share/fonts/TTF/" + name):
        try:
            return ImageFont.truetype(path, size)
        except (OSError, IOError):
            pass
    return ImageFont.load_default()


def level_color(dbm):
    """Map -125..-60 dBm to a blue->green->yellow->red ramp."""
    t = max(0.0, min(1.0, (dbm + 125) / 65.0))
    stops = [(0.0, (12, 20, 60)), (0.25, (30, 90, 200)), (0.45, (40, 200, 140)),
             (0.65, (240, 220, 60)), (0.85, (255, 120, 40)), (1.0, (255, 50, 50))]
    for (t0, c0), (t1, c1) in zip(stops, stops[1:]):
        if t <= t1:
            k = (t - t0) / (t1 - t0)
            return tuple(int(a + (b - a) * k) for a, b in zip(c0, c1))
    return stops[-1][1]


def ago(t):
    if not t:
        return "never"
    s = int(time.time() - t)
    if s < 60:
        return "{}s ago".format(s)
    if s < 3600:
        return "{}m ago".format(s // 60)
    return "{}h ago".format(s // 3600)


COLORS = {"green": GREEN, "cyan": CYAN, "dim": DIM, "text": TEXT}
FILTERS = [("All", None), ("Meshtastic", "mesh"), ("LoRaWAN", "lwan")]


def filtered_packets(sc, flt):
    with sc.lock:
        pk = list(sc.packets)
    want = FILTERS[flt][1]
    return [e for e in pk if want is None or e["mode"] == want]


def packet_line(e):
    if e["crc_bad"]:
        return "damaged packet ({} bytes)".format(e["len"]), ORANGE
    dec = e["dec"]
    if dec:
        return dec["summary"], COLORS.get(dec.get("color"), TEXT)
    what = "Meshtastic" if e["mode"] == "mesh" else "LoRaWAN"
    return "LoRa frame, not {} ({} bytes)".format(what, e["len"]), PURPLE


class Renderer:
    def __init__(self):
        self.f_title = load_font(13, bold=True)
        self.f_big = load_font(18, bold=True)
        self.f_med = load_font(12, bold=True)
        self.f_reg = load_font(11)
        self.f_sm = load_font(10)
        self.f_smb = load_font(10, bold=True)
        self.f_xs = load_font(9)
        self.f_mono = None
        for p in (os.path.join(FONT_DIR, "DejaVuSansMono.ttf"), "/usr/share/fonts/TTF/DejaVuSansMono.ttf"):
            try:
                self.f_mono = ImageFont.truetype(p, 9)
                break
            except (OSError, IOError):
                pass
        self.f_mono = self.f_mono or self.f_xs

    def tr(self, d, xr, y, s, font, fill):
        d.text((xr - d.textlength(s, font=font), y), s, font=font, fill=fill)

    def tc(self, d, xc, y, s, font, fill):
        d.text((xc - d.textlength(s, font=font) / 2, y), s, font=font, fill=fill)

    def fit(self, d, s, font, width):
        if d.textlength(s, font=font) <= width:
            return s
        while s and d.textlength(s + "…", font=font) > width:
            s = s[:-1]
        return s + "…"

    def panel(self, d, box):
        d.rounded_rectangle(box, radius=6, fill=PANEL)

    def header(self, d, page, sc, ui):
        d.rectangle((0, 0, W, 18), fill=PANEL)
        d.text((6, 2), "LoRa Scan", font=self.f_title, fill=CYAN)
        for i in range(len(PAGES)):
            cx = 84 + i * 7
            d.ellipse((cx, 7, cx + 4, 11), fill=CYAN if i == page else GRID)
        d.text((130, 4), PAGES[page], font=self.f_smb, fill=TEXT)
        if sc.status == "running":
            txt, col = ("RX ONLY", GREEN)
        elif sc.status == "error":
            txt, col = ("ERROR", RED)
        else:
            txt, col = ("starting", YELLOW)
        self.tr(d, 270, 4, txt, self.f_smb, col)
        self.tr(d, 314, 3, datetime.now().strftime("%H:%M"), self.f_reg, TEXT)

    def hint_text(self, d, x, y, text, font, col):
        """Draw a key hint; the arrow triangles are drawn red like the keys printed on the keyboard."""
        run, red = "", False
        for ch in text + "\0":
            is_red = ch in "\u25b2\u25bc\u25c0\u25b6"
            if (is_red != red or ch == "\0") and run:
                d.text((x, y), run, font=font, fill=KEY_RED if red else col)
                x += d.textlength(run, font=font)
                run = ""
            red = is_red
            if ch != "\0":
                run += ch

    def footer(self, d, text, right=None, rcol=DIM):
        self.hint_text(d, 6, H - 13, text, self.f_sm, DIM)
        if right:
            self.tr(d, 314, H - 13, right, self.f_sm, rcol)

    def freq_x(self, f, x0, x1):
        return x0 + (x1 - x0) * (f - BAND_LO) / (BAND_HI - BAND_LO)

    def axis(self, d, x0, x1, y):
        for f in (902, 906, 910, 914, 918, 922, 926):
            x = self.freq_x(f, x0, x1)
            d.line((x, y, x, y + 3), fill=DIM)
            self.tc(d, x, y + 3, str(f), self.f_xs, DIM)
        xc = self.freq_x(CAP_MAX_MHZ, x0, x1)
        d.line((xc, y - 2, x1, y - 2), fill=(90, 40, 40), width=2)

    # -- pages --
    def page_spectrum(self, d, sc):
        self.panel(d, (4, 22, 316, 152))
        x0, x1, y0, y1 = 32, 310, 30, 128
        lo, hi = -125.0, -70.0
        for db in (-120, -110, -100, -90, -80):
            y = y1 - (y1 - y0) * (db - lo) / (hi - lo)
            d.line((x0, y, x1, y), fill=GRID)
            self.tr(d, x0 - 3, y - 5, str(db), self.f_xs, DIM)
        for p in sc.presets:
            x = self.freq_x(p["freq"], x0, x1)
            d.line((x, y0, x, y1), fill=(50, 40, 80))
        with sc.lock:
            cur = list(sc.sweep)
            mh = list(sc.maxhold)
            pos = sc.sweep_pos
        bw = (x1 - x0) / SWEEP_POINTS
        noise = sorted(v for v in cur if v is not None)
        floor = noise[len(noise) // 4] if noise else None
        for i in range(SWEEP_POINTS):
            xa = x0 + i * bw
            if mh[i] is not None:
                ym = y1 - (y1 - y0) * (max(lo, min(hi, mh[i])) - lo) / (hi - lo)
                d.line((xa, ym, xa + bw, ym), fill=(110, 110, 130))
            v = cur[i]
            if v is None:
                continue
            yv = y1 - (y1 - y0) * (max(lo, min(hi, v)) - lo) / (hi - lo)
            d.rectangle((xa, yv, xa + max(1, bw - 0.6), y1), fill=level_color(v))
        xp = x0 + pos * bw
        d.line((xp, y0, xp, y1), fill=TEXT)
        self.axis(d, x0, x1, y1 + 2)
        d.text((10, 24), "dBm", font=self.f_xs, fill=DIM)
        if floor is not None:
            pk_i = max(range(SWEEP_POINTS), key=lambda k: mh[k] if mh[k] is not None else -999)
            pk_f = BAND_LO + SWEEP_STEP / 2 + pk_i * SWEEP_STEP
            self.tr(d, 310, 24, "floor {:.0f}  peak {:.0f} @ {:.2f}".format(
                floor, mh[pk_i], pk_f), self.f_xs, TEXT)
        self.footer(d, "purple = Meshtastic presets   M clear max", "{} sweeps".format(sc.sweeps_done))

    def page_waterfall(self, d, sc):
        self.panel(d, (4, 22, 316, 152))
        x0, x1, y0, y1 = 10, 310, 26, 130
        with sc.lock:
            rows = list(sc.waterfall)
        bw = (x1 - x0) / SWEEP_POINTS
        rh = 1 if len(rows) > 50 else 2
        y = y0
        for row in rows:
            if y + rh > y1:
                break
            for i, v in enumerate(row):
                if v is not None:
                    d.rectangle((x0 + i * bw, y, x0 + (i + 1) * bw, y + rh - 1), fill=level_color(v))
            y += rh
        if not rows:
            self.tc(d, 160, 70, "Building waterfall… (one row per sweep)", self.f_reg, DIM)
        for p in sc.presets:
            x = self.freq_x(p["freq"], x0, x1)
            d.polygon([(x - 3, y1 + 1), (x + 3, y1 + 1), (x, y1 + 5)], fill=PURPLE)
        self.axis(d, x0, x1, y1 + 7)
        self.footer(d, "newest at top · ~1.5 s per row", "{} rows".format(len(rows)))

    def page_finder(self, d, sc, ui):
        self.panel(d, (4, 22, 316, 152))
        d.text((10, 24), "PRESET", font=self.f_smb, fill=DIM)
        d.text((100, 24), "MHz", font=self.f_smb, fill=DIM)
        self.tr(d, 196, 24, "ACTIVITY", self.f_smb, DIM)
        self.tr(d, 244, 24, "PKTS", self.f_smb, DIM)
        self.tr(d, 310, 24, "LAST", self.f_smb, DIM)
        with sc.lock:
            rows = [dict(p) for p in sc.presets]
            cur, locked = sc.current, sc.locked
            left = max(0.0, sc.dwell - (time.monotonic() - sc.dwell_start))
        sel = ui["sel"]
        top = max(0, min(sel - 4, len(rows) - 9))
        y = 37
        for i in range(top, min(top + 9, len(rows))):
            p = rows[i]
            if i == sel:
                d.rectangle((6, y - 1, 314, y + 11), fill=SEL)
            if i == cur:
                d.polygon([(8, y + 1), (8, y + 9), (13, y + 5)], fill=GREEN if not locked else YELLOW)
            col = TEXT if p["freq"] <= CAP_MAX_MHZ else DIM
            d.text((16, y), p["name"], font=self.f_sm, fill=col)
            d.text((100, y), "{:.3f}".format(p["freq"]), font=self.f_sm, fill=col)
            act = p["preambles"]
            self.tr(d, 196, y, str(act), self.f_reg, YELLOW if act else DIM)
            pk = p["packets"]
            self.tr(d, 244, y, str(pk), self.f_med if pk else self.f_reg, GREEN if pk else DIM)
            self.tr(d, 310, y, "{:.0f}dBm".format(p["last_rssi"]) if p["last_rssi"] is not None
                    else (ago(p["last_t"]) if p["last_t"] else "-"), self.f_xs, TEXT if p["last_t"] else DIM)
            y += 12
        state = "LOCKED on {}".format(rows[cur]["name"]) if locked else \
            "{} · next in {:.0f}s".format(rows[cur]["name"], left)
        self.footer(d, "▲▼ sel  Enter lock  +/- dwell {:.0f}s".format(sc.dwell), state,
                    YELLOW if locked else GREEN)

    def page_lorawan(self, d, sc, ui):
        self.panel(d, (4, 22, 316, 152))
        with sc.lock:
            rows = sc.lw_rows()
            up = sc.lw_dir == "up"
            cur = sc.lw_cur
            lock_row, sf_lock = sc.lw_lock_row, sc.lw_sf
            cells = {k: dict(v) for k, v in sc.lw_cells.items()}
            sb = sc.lw_subband
        sfs = [7, 8, 9, 10] if up else [7, 8, 9, 10, 11, 12]
        title = "Sub-band {} · device uplinks".format(sb) if up else "Gateway downlinks (923–928 MHz)"
        d.text((10, 24), title, font=self.f_smb, fill=PURPLE)
        x0 = 112
        cw = (310 - x0) / len(sfs)
        for j, sf in enumerate(sfs):
            col = YELLOW if sf_lock == sf else DIM
            self.tc(d, x0 + cw * (j + 0.5), 34, "SF{}".format(sf), self.f_xs, col)
        d.text((10, 34), "channel  MHz", font=self.f_xs, fill=DIM)
        y = 46
        sel = ui["lw_sel"] % len(rows)
        for i, r in enumerate(rows):
            if i == sel:
                d.rectangle((6, y - 1, 314, y + 10), fill=SEL)
            col = TEXT if r["freq"] <= CAP_MAX_MHZ else DIM
            mark = "▸" if lock_row == i else " "
            d.text((8, y - 1), "{}ch{:<2} {:.1f}{}".format(mark, r["ch"], r["freq"],
                                                          "" if r["bw"] == 125 else " 500k"),
                   font=self.f_xs, fill=YELLOW if lock_row == i else col)
            for j, sf in enumerate(sfs):
                cx0 = x0 + cw * j + 1
                if sf not in r["sfs"]:
                    continue
                key = (sc.lw_dir, sb if up else 0, r["ch"], sf)
                c = cells.get(key, {})
                n = c.get("pkts", 0)
                fill = level_color(c["last_rssi"]) if n and c.get("last_rssi") is not None else \
                    ((40, 34, 60) if c.get("pre") else (30, 34, 44))
                d.rectangle((cx0, y, cx0 + cw - 2, y + 9), fill=fill)
                if cur == (r["ch"], sf) and sc.mode == "lwan":
                    d.rectangle((cx0 - 1, y - 1, cx0 + cw - 1, y + 10), outline=GREEN, width=1)
                if n:
                    self.tc(d, cx0 + cw / 2 - 1, y - 1, str(n), self.f_xs, BG)
            y += 11 if up else 12
        total = sum(v["pkts"] for v in cells.values())
        hint = "▲▼ Enter lock  S SF  B band  D up/dn" if up else "▲▼ Enter lock  S SF  D up/down"
        self.footer(d, hint, "{} pkts".format(total), GREEN if total else DIM)

    def page_packets(self, d, sc, ui):
        self.panel(d, (4, 22, 316, 152))
        pk = filtered_packets(sc, ui["pk_filter"])
        listening = "Finder (Meshtastic)" if sc.listen_mode == "find" else "LoRaWAN page"
        if not pk:
            self.tc(d, 160, 46, "No packets yet", self.f_med, DIM)
            self.tc(d, 160, 66, "Listening with the " + listening + " settings.", self.f_reg, DIM)
            self.tc(d, 160, 82, "Meshtastic nodes can be quiet for minutes to hours;", self.f_sm, DIM)
            self.tc(d, 160, 95, "LoRaWAN sensors usually report every few minutes.", self.f_sm, DIM)
        ui["pk_sel"] = max(0, min(ui["pk_sel"], len(pk) - 1))
        sel = ui["pk_sel"]
        top = max(0, min(sel - 2, len(pk) - 6))
        y = 25
        for k in range(top, min(top + 6, len(pk))):
            e = pk[k]
            if k == sel:
                d.rounded_rectangle((6, y - 1, 314, y + 19), radius=3, fill=SEL)
            t = datetime.fromtimestamp(e["t"]).strftime("%H:%M:%S")
            d.text((10, y), t, font=self.f_xs, fill=DIM)
            tag = "MESH" if e["mode"] == "mesh" else "LWAN"
            d.text((54, y - 1), tag, font=self.f_smb, fill=CYAN if tag == "MESH" else PURPLE)
            d.text((88, y), e["label"], font=self.f_xs, fill=DIM)
            self.tr(d, 310, y - 1, "{:.0f} dBm  {:+.1f} dB".format(e["rssi"], e["snr"]), self.f_sm,
                    level_color(e["rssi"]))
            line, col = packet_line(e)
            d.text((10, y + 9), self.fit(d, line, self.f_sm, 300), font=self.f_sm, fill=col)
            y += 21
        self.footer(d, "▲▼ select  Enter details  T {}".format(FILTERS[ui["pk_filter"]][0]),
                    "{} heard".format(len(pk)), GREEN if pk else DIM)

    def detail_lines(self, d, e):
        lines = []
        dec = e["dec"] or {}
        line, col = packet_line(e)
        for part in self.wrap(d, line, self.f_smb, 296):
            lines.append((part, col, self.f_smb))
        for k, v in dec.get("fields", []):
            parts = self.wrap(d, "{}: {}".format(k, v), self.f_sm, 296)
            for i, part in enumerate(parts):
                lines.append((part, TEXT if i or ":" not in part else TEXT, self.f_sm))
        lines.append(("", DIM, self.f_sm))
        lines.append(("{} · {:.3f} MHz · SF{} · {} kHz".format(e["label"], e["freq"], e["sf"], e["bw"]),
                      DIM, self.f_sm))
        lines.append(("RSSI {:.1f} dBm · SNR {:+.2f} dB · {} bytes · {}".format(
            e["rssi"], e["snr"], e["len"], datetime.fromtimestamp(e["t"]).strftime("%H:%M:%S")), DIM, self.f_sm))
        data = e["data"]
        for i in range(0, len(data), 16):
            lines.append(("{:03x}  {}".format(i, data[i:i + 16].hex(" ")), (150, 160, 175), self.f_mono))
        return lines

    def wrap(self, d, s, font, width):
        out, cur = [], ""
        for w in s.split(" "):
            t = w if not cur else cur + " " + w
            if d.textlength(t, font=font) <= width or not cur:
                cur = t
            else:
                out.append(cur)
                cur = w
        out.append(cur)
        return out

    def page_detail(self, d, sc, ui):
        pk = filtered_packets(sc, ui["pk_filter"])
        if not pk:
            ui["detail"] = False
            return
        e = pk[max(0, min(ui["pk_sel"], len(pk) - 1))]
        d.rounded_rectangle((4, 22, 316, 152), radius=6, fill=(16, 20, 28), outline=GRID)
        lines = self.detail_lines(d, e)
        ui["detail_scroll"] = max(0, min(ui["detail_scroll"], max(0, len(lines) - 10)))
        y = 26
        for text, col, font in lines[ui["detail_scroll"]:ui["detail_scroll"] + 10]:
            d.text((10, y), text, font=font, fill=col)
            y += 12
        more = len(lines) - ui["detail_scroll"] - 10
        self.footer(d, "▲▼ scroll   Enter/ESC close", "{} more ↓".format(more) if more > 0 else None)

    def page_map(self, d, sc, ui):
        self.panel(d, (4, 22, 316, 152))
        st, col = sc.gps.status()
        with sc.lock:
            pts = [(e["rx_pos"][0], e["rx_pos"][1], e["rssi"]) for e in sc.coverage]
        with sc.gps.lock:
            track = [(la, lo) for _, la, lo in sc.gps.track]
        here = sc.gps.position()
        allp = [(a, b) for a, b, _ in pts] + track + ([here] if here else [])
        d.text((10, 24), st, font=self.f_xs, fill=col)
        if not allp:
            self.tc(d, 160, 60, "No positions yet", self.f_med, DIM)
            self.tc(d, 160, 80, "With a GPS fix, every packet heard is mapped", self.f_sm, DIM)
            self.tc(d, 160, 93, "where you were, colored by signal strength.", self.f_sm, DIM)
            self.tc(d, 160, 106, "Drive or walk around to map coverage.", self.f_sm, DIM)
        else:
            lat0 = sum(p[0] for p in allp) / len(allp)
            kx = math.cos(math.radians(lat0)) * 111320.0
            ky = 110540.0
            xs = [p[1] * kx for p in allp]
            ys = [p[0] * ky for p in allp]
            span = max(max(xs) - min(xs), max(ys) - min(ys), 200.0)
            bx0, by0, bx1, by1 = 14, 36, 306, 138
            sc_px = min((bx1 - bx0) / span, (by1 - by0) / span) * 0.92
            cxm, cym = (max(xs) + min(xs)) / 2, (max(ys) + min(ys)) / 2

            def proj(la, lo):
                return ((bx0 + bx1) / 2 + (lo * kx - cxm) * sc_px, (by0 + by1) / 2 - (la * ky - cym) * sc_px)
            if len(track) >= 2:
                d.line([proj(a, b) for a, b in track], fill=(90, 96, 110), width=2)
            for la, lo, rssi in pts:
                x, y = proj(la, lo)
                d.ellipse((x - 3, y - 3, x + 3, y + 3), fill=level_color(rssi), outline=BG)
            if here:
                x, y = proj(*here)
                d.ellipse((x - 4, y - 4, x + 4, y + 4), outline=TEXT, width=2)
            # scale bar
            m = span * 0.92 / 3
            mag = 10 ** int(math.floor(math.log10(m)))
            step = max(s for s in (1, 2, 5) if s * mag <= m) * mag
            px = step * sc_px
            d.line((bx0, by1 + 4, bx0 + px, by1 + 4), fill=TEXT, width=2)
            lbl = "{:.0f} m".format(step) if step < 1000 else "{:g} km".format(step / 1000)
            d.text((bx0 + px + 4, by1 - 1), lbl, font=self.f_xs, fill=TEXT)
            self.tc(d, 300, 36, "N↑", self.f_xs, DIM)
        self.footer(d, "K save Google Earth (KML) file", "{} mapped".format(len(pts)), GREEN if pts else DIM)

    def page_message(self, d, title, lines, color):
        box = (14, 32, 306, 140)
        d.rounded_rectangle(box, radius=8, fill=(14, 16, 22), outline=color, width=2)
        self.tc(d, 160, 40, title, self.f_med, color)
        import textwrap
        y = 60
        for ln in lines:
            for part in textwrap.wrap(ln, 48) or [""]:
                self.tc(d, 160, y, part, self.f_sm, TEXT)
                y += 13

    def render(self, page, sc, ui):
        img = Image.new("RGB", (W, H), BG)
        d = ImageDraw.Draw(img)
        self.header(d, page, sc, ui)
        if page == P_SPEC:
            self.page_spectrum(d, sc)
        elif page == P_WATER:
            self.page_waterfall(d, sc)
        elif page == P_FIND:
            self.page_finder(d, sc, ui)
        elif page == P_LWAN:
            self.page_lorawan(d, sc, ui)
        elif page == P_PKTS:
            if ui.get("detail"):
                self.page_detail(d, sc, ui)
            else:
                self.page_packets(d, sc, ui)
        else:
            self.page_map(d, sc, ui)
        if ui.get("confirm"):
            self.page_message(d, "LoRa radio in use",
                              ["“{}” is running and is using the LoRa radio.".format(ui["confirm"]),
                               "Close it first, then open the scanner again.", "",
                               "ESC = quit     Enter = try anyway"], YELLOW)
        elif sc.status == "error":
            self.page_message(d, "Radio problem", [sc.error or "unknown error", "", "ESC to quit"], RED)
        if ui.get("toast") and time.monotonic() < ui.get("toast_until", 0):
            msg, col = ui["toast"]
            w = d.textlength(msg, font=self.f_reg) + 20
            d.rounded_rectangle((160 - w / 2, 128, 160 + w / 2, 150), radius=6, fill=(14, 16, 22), outline=col)
            self.tc(d, 160, 132, msg, self.f_reg, col)
        return img


# --------------------------------------------------------------------------
# Framebuffer + keyboard (as in SysMon / Trail Recorder)
# --------------------------------------------------------------------------
class Framebuffer:
    def __init__(self, path):
        self.fd = os.open(path, os.O_RDWR)
        vbuf = bytearray(160)
        fcntl.ioctl(self.fd, FBIOGET_VSCREENINFO, vbuf)
        self.xres, self.yres = struct.unpack_from("2I", vbuf, 0)
        self.bpp = struct.unpack_from("I", vbuf, 24)[0]
        fix_fmt = "16sL" + "I" * 6 + "HHHI"
        fbuf = bytearray(struct.calcsize(fix_fmt))
        fcntl.ioctl(self.fd, FBIOGET_FSCREENINFO, fbuf)
        fields = struct.unpack(fix_fmt, fbuf)
        self.smem_len, self.line_length = fields[2], fields[8]
        if self.bpp not in (16, 32):
            os.close(self.fd)
            raise RuntimeError("unsupported framebuffer depth: {} bpp".format(self.bpp))
        self.mm = mmap.mmap(self.fd, self.smem_len, mmap.MAP_SHARED, mmap.PROT_READ | mmap.PROT_WRITE)

    def _convert(self, img):
        if self.bpp == 32:
            return img.convert("RGBA").tobytes("raw", "BGRA")
        if HAS_NUMPY:
            a = np.asarray(img, dtype=np.uint16)
            v = ((a[..., 0] >> 3) << 11) | ((a[..., 1] >> 2) << 5) | (a[..., 2] >> 3)
            return v.astype("<u2").tobytes()
        out = bytearray()
        for r, g, b in img.getdata():
            v = ((r >> 3) << 11) | ((g >> 2) << 5) | (b >> 3)
            out += bytes((v & 0xFF, v >> 8))
        return bytes(out)

    def show(self, img):
        if (self.xres, self.yres) != img.size:
            img = img.rotate(90, expand=True) if (self.yres, self.xres) == img.size else img.resize((self.xres, self.yres))
        data = self._convert(img)
        row = self.xres * self.bpp // 8
        if row == self.line_length:
            self.mm.seek(0)
            self.mm.write(data)
        else:
            for y in range(self.yres):
                self.mm.seek(y * self.line_length)
                self.mm.write(data[y * row:(y + 1) * row])

    def clear(self):
        self.mm.seek(0)
        self.mm.write(b"\x00" * min(self.smem_len, self.line_length * self.yres))

    def close(self):
        try:
            self.mm.close()
        finally:
            os.close(self.fd)


EVENT_FMT = "llHHi"
EVENT_SIZE = struct.calcsize(EVENT_FMT)


class Keys:
    def __init__(self):
        self.fds = []
        for path in sorted(glob.glob("/dev/input/event*")):
            try:
                self.fds.append(os.open(path, os.O_RDONLY | os.O_NONBLOCK))
            except OSError:
                pass

    def poll(self, timeout):
        if not self.fds:
            time.sleep(timeout)
            return []
        try:
            r, _, _ = select.select(self.fds, [], [], timeout)
        except (OSError, ValueError):
            return []
        out = []
        for fd in r:
            try:
                data = os.read(fd, EVENT_SIZE * 32)
            except (BlockingIOError, OSError):
                continue
            for off in range(0, len(data) - EVENT_SIZE + 1, EVENT_SIZE):
                _, _, et, code, val = struct.unpack_from(EVENT_FMT, data, off)
                if et == EV_KEY and val in (1, 2):
                    out.append((code, val))
        return out

    def close(self):
        for fd in self.fds:
            try:
                os.close(fd)
            except OSError:
                pass


# --------------------------------------------------------------------------
# App
# --------------------------------------------------------------------------
class App:
    def __init__(self, scanner):
        self.sc = scanner
        self.page = P_SPEC
        self.ui = {"sel": 0, "confirm": None, "toast": None, "toast_until": 0, "lw_sel": 0,
                   "pk_sel": 0, "pk_filter": 0, "detail": False, "detail_scroll": 0}

    def toast(self, msg, col=CYAN, secs=2.0):
        self.ui["toast"] = (msg, col)
        self.ui["toast_until"] = time.monotonic() + secs

    def set_page(self, p):
        self.page = p % len(PAGES)
        self.ui["detail"] = False
        if self.page in (P_SPEC, P_WATER):
            self.sc.set_mode("sweep")
        elif self.page == P_FIND:
            self.sc.set_mode("find")
        elif self.page == P_LWAN:
            self.sc.set_mode("lwan")
        else:
            self.sc.set_mode(self.sc.listen_mode)

    def handle(self, code, repeat):
        """Return False to quit."""
        ui = self.ui
        if repeat and code not in (KEY_UP, KEY_DOWN, KEY_SEMICOLON, KEY_DOT, KEY_F, KEY_X):
            return True
        up = code in (KEY_UP, KEY_SEMICOLON, KEY_F)
        down = code in (KEY_DOWN, KEY_DOT, KEY_X)
        if self.page == P_PKTS and ui["detail"]:
            if code in (KEY_ESC, KEY_ENTER, KEY_SPACE, KEY_Q):
                ui["detail"] = False
            elif up:
                ui["detail_scroll"] = max(0, ui["detail_scroll"] - 1)
            elif down:
                ui["detail_scroll"] += 1
            return True
        if code in (KEY_ESC, KEY_Q):
            return False
        if code in (KEY_RIGHT, KEY_TAB, KEY_SLASH, KEY_C):
            self.set_page(self.page + 1)
        elif code in (KEY_LEFT, KEY_COMMA, KEY_Z):
            self.set_page(self.page - 1)
        elif code in NUM_KEYS:
            self.set_page(NUM_KEYS[code])
        elif self.page == P_SPEC and code == KEY_M:
            with self.sc.lock:
                self.sc.maxhold = [None] * SWEEP_POINTS
            self.toast("Max-hold cleared")
        elif self.page == P_FIND:
            n = len(self.sc.presets)
            if up:
                ui["sel"] = (ui["sel"] - 1) % n
            elif down:
                ui["sel"] = (ui["sel"] + 1) % n
            elif code in (KEY_ENTER, KEY_SPACE):
                sel = ui["sel"]
                with self.sc.lock:
                    already = self.sc.locked and self.sc.current == sel
                if already:
                    self.sc.locked = False
                    self.toast("Cycling through all presets", GREEN)
                else:
                    self.sc.locked = True
                    self.sc.select(sel)
                    self.toast("Listening only on " + self.sc.presets[sel]["name"], YELLOW)
            elif code in (KEY_EQUAL, KEY_KPPLUS):
                self.sc.dwell = min(300.0, self.sc.dwell * 2)
                self.toast("Dwell {:.0f} s per preset".format(self.sc.dwell))
            elif code in (KEY_MINUS, KEY_KPMINUS):
                self.sc.dwell = max(5.0, self.sc.dwell / 2)
                self.toast("Dwell {:.0f} s per preset".format(self.sc.dwell))
        elif self.page == P_LWAN:
            sc = self.sc
            n = len(sc.lw_rows())
            if up:
                ui["lw_sel"] = (ui["lw_sel"] - 1) % n
            elif down:
                ui["lw_sel"] = (ui["lw_sel"] + 1) % n
            elif code in (KEY_ENTER, KEY_SPACE):
                sel = ui["lw_sel"] % n
                if sc.lw_lock_row == sel:
                    sc.lw_lock_row = None
                    self.toast("Hopping all channels", GREEN)
                else:
                    sc.lw_lock_row = sel
                    self.toast("Listening on ch{} only".format(sc.lw_rows()[sel]["ch"]), YELLOW)
                sc.lw_changed()
            elif code == KEY_S:
                opts = [None, 7, 8, 9, 10] if sc.lw_dir == "up" else [None, 7, 8, 9, 10, 11, 12]
                sc.lw_sf = opts[(opts.index(sc.lw_sf) + 1) % len(opts) if sc.lw_sf in opts else 0]
                sc.lw_changed()
                self.toast("All spreading factors" if sc.lw_sf is None else "SF{} only".format(sc.lw_sf))
            elif code == KEY_B and sc.lw_dir == "up":
                sc.lw_subband = sc.lw_subband % 8 + 1
                sc.lw_lock_row = None
                sc.lw_changed()
                hint = " (TTN / Helium)" if sc.lw_subband == 2 else ""
                self.toast("Sub-band {}{}".format(sc.lw_subband, hint))
            elif code == KEY_D:
                sc.lw_dir = "down" if sc.lw_dir == "up" else "up"
                sc.lw_lock_row, sc.lw_sf = None, None
                ui["lw_sel"] = 0
                sc.lw_changed()
                self.toast("Gateway downlinks (above the Cap's rated 923 MHz)" if sc.lw_dir == "down"
                           else "Device uplinks", YELLOW if sc.lw_dir == "down" else CYAN, 3.0)
            elif code in (KEY_EQUAL, KEY_KPPLUS):
                sc.lw_dwell = min(30.0, sc.lw_dwell * 2)
                self.toast("{:.0f} s per channel/SF".format(sc.lw_dwell))
            elif code in (KEY_MINUS, KEY_KPMINUS):
                sc.lw_dwell = max(1.0, sc.lw_dwell / 2)
                self.toast("{:.0f} s per channel/SF".format(sc.lw_dwell))
        elif self.page == P_PKTS:
            if up:
                ui["pk_sel"] = max(0, ui["pk_sel"] - 1)
            elif down:
                ui["pk_sel"] += 1
            elif code in (KEY_ENTER, KEY_SPACE):
                if filtered_packets(self.sc, ui["pk_filter"]):
                    ui["detail"], ui["detail_scroll"] = True, 0
            elif code == KEY_T:
                ui["pk_filter"] = (ui["pk_filter"] + 1) % len(FILTERS)
                ui["pk_sel"] = 0
                self.toast("Showing: " + FILTERS[ui["pk_filter"]][0])
        elif self.page == P_MAP and code == KEY_K:
            try:
                path, n, t = write_kml(self.sc)
                self.toast("Saved ~/LoRaScan/" + os.path.basename(path), GREEN, 3.0)
            except OSError as e:
                self.toast("Couldn't save: " + e.strerror, RED, 3.0)
        return True


class FakeGps:
    def __init__(self, track, here):
        self.track, self.here = track, here
        self.lock = threading.Lock()
        self.has_fix = True

    def position(self):
        return self.here

    def status(self):
        return "GPS fix · using 9 of 14 satellites", GREEN

    def start(self):
        pass


def preview(path, page, mock):
    sc = Scanner(lambda: SimRadio())
    app = App(sc)
    if mock:
        r = SimRadio()
        r.set_lora(125, 7, 5)
        for _ in range(60):
            for i in range(SWEEP_POINTS):
                r.set_frequency(BAND_LO + SWEEP_STEP / 2 + i * SWEEP_STEP)
                v = max(r.rssi_inst() for _ in range(6))
                sc.sweep[i] = v
                sc.maxhold[i] = v if sc.maxhold[i] is None else max(sc.maxhold[i], v)
            sc.waterfall.appendleft(list(sc.sweep))
            sc.sweeps_done += 1
        sc.sweep_pos = 37
        sc.status = "running"
        # a short drive with packets heard along it (Chicago, sample data)
        track, rng = [], random.Random(3)
        la, lo = 41.8806, -87.6742
        for k in range(40):
            la += 0.0006 + rng.gauss(0, 0.0001)
            lo += 0.0009 * math.sin(k / 7) + rng.gauss(0, 0.0001)
            track.append((time.time() - 1200 + k * 30, la, lo))
        sc.gps = FakeGps(track, (la, lo))
        now = time.time()
        lf = next(p for p in sc.presets if p["name"] == "LongFast")
        lf.update(preambles=11, headers=7, packets=6, last_rssi=-104.5, last_t=now - 40)
        sc.current = 0
        calls = []
        for k in range(6):
            fr = sim_mesh_frame(rng, "LongFast", port=[4, 1, 67, 1, 3, 70][k])
            calls.append((now - 900 + k * 150, track[k * 6][1:], {"mode": "mesh", "label": "LongFast",
                          "freq": lf["freq"], "sf": 11, "bw": 250}, fr, -112 + k * 4 + rng.gauss(0, 2),
                          rng.gauss(0, 4), False))
        for k in range(9):
            row = lw_uplink_channels(2)[k % 8]
            sf = [9, 10, 8, 7][k % 4]
            calls.append((now - 860 + k * 95, track[min(39, 4 + k * 4)][1:], {
                "mode": "lwan", "label": "ch{} SF{}".format(row["ch"], sf), "freq": row["freq"], "sf": sf,
                "bw": 125}, sim_lorawan_frame(rng), -118 + k * 5 + rng.gauss(0, 3), rng.gauss(-3, 4), k == 5))
            if k != 5:
                sc.lw_cell(row, sf)["pkts"] += 1 + k % 3
                sc.lw_cell(row, sf)["last_rssi"] = -118 + k * 5
        for t, here, ctx, fr, rssi, snr, bad in sorted(calls, key=lambda c: c[0]):
            sc.gps.here = here
            sc._record(ctx, fr, rssi, snr, bad, t)
        sc.gps.here = (la, lo)
        sc.lw_cur = (10, 9)
        sc.mode = "lwan" if page == P_LWAN else sc.mode
        sc.log_path = None
    app.page = page
    app.ui["sel"] = 0
    app.ui["lw_sel"] = 2
    if "--detail" in sys.argv:
        app.ui["detail"] = True
        app.ui["pk_sel"] = int(os.environ.get("PK_SEL", "0"))
    Renderer().render(page, sc, app.ui).save(path)
    print("wrote", path)
    return 0


def main():
    args = sys.argv[1:]
    if args and args[0] == "--preview":
        global LOG_DIR
        LOG_DIR = "/tmp/lorascan-preview"
        return preview(args[1] if len(args) > 1 else "preview.png",
                       int(args[2]) if len(args) > 2 and args[2].isdigit() else 0, "--mock" in args)
    if args and args[0] in ("--version", "-V"):
        print("LoRa Scanner", VERSION)
        return 0

    signal.signal(signal.SIGINT, _stop)
    signal.signal(signal.SIGTERM, _stop)
    signal.signal(signal.SIGHUP, _stop)
    try:
        fb = Framebuffer(FB_DEVICE)
    except Exception as e:
        print("LoRa Scanner: cannot open framebuffer {}: {}".format(FB_DEVICE, e), file=sys.stderr)
        return 3

    hw = HardwareRadio()
    gps = GpsReader()
    scanner = Scanner(hw.open, gps)
    app = App(scanner)
    keys = Keys()
    renderer = Renderer()
    app.ui["confirm"] = radio_in_use()
    started = False
    try:
        while _running:
            if not started and not app.ui["confirm"]:
                gps.start()
                scanner.start()
                started = True
            fb.show(renderer.render(app.page, scanner, app.ui))
            for code, val in keys.poll(0.25):
                if app.ui["confirm"]:
                    if code in (KEY_ENTER, KEY_Y, KEY_SPACE):
                        app.ui["confirm"] = None
                    elif code in (KEY_ESC, KEY_Q, KEY_N):
                        return 0
                    continue
                if not app.handle(code, val == 2):
                    return 0
    finally:
        scanner.stop()
        gps.stop_flag = True
        hw.close()
        if scanner.coverage:
            try:
                write_kml(scanner)
            except OSError:
                pass
        keys.close()
        try:
            fb.clear()
        except Exception:
            pass
        fb.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
