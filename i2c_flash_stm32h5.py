#!/usr/bin/env python3
# STM32H5 (H563) I2C ROM Bootloader flasher — NS erase/write 實作版
#
# - 支援：
#   - PCA9575 控 BOOT0/NRST 進入 boot ROM
#   - I2C bootloader: GET_VER / GET_ID / READ / GO / WRITE / WRITE_NS / ERASE_NS
#   - erase auto/pages/none, verify, go
# - 協定依據 AN4221 Rev14 / STM32H5:
#   - 所有封包使用 XOR checksum（資料+checksum XOR = 0）
#   - Read Memory: N = len-1, checksum = ~N
#   - No-Stretch Erase: N(16-bit)=pages-1 + XOR, page list + XOR
#   - No-Stretch Write: N = len-1, checksum = XOR(N, data...)

import argparse
import time
import sys
import os
import logging
from datetime import datetime
from typing import List, Optional, Union

try:
    from smbus2 import SMBus, i2c_msg
except Exception:
    print("smbus2 required. Install: pip3 install smbus2", file=sys.stderr)
    raise

ACK  = 0x79
NACK = 0x1F
BUSY = 0x76

CMD_GET_VER   = 0x01
CMD_GET_ID    = 0x02
CMD_READ      = 0x11
CMD_GO        = 0x21
CMD_WRITE     = 0x31
CMD_ERASE     = 0x44
CMD_WRITE_NS  = 0x32
CMD_ERASE_NS  = 0x45

FLASH_BASE   = 0x08000000
PAGE_SIZE    = 0x800      # 2KB
TOTAL_PAGES  = 512
POLL_DELAY   = 0.020

log = logging.getLogger("i2c_flash_h5")

# ---------- helpers ----------

def xor8(data: Union[List[int], bytes]) -> int:
    x = 0
    for v in data:
        x ^= v
    return x & 0xFF

# ---------- PCA9575 ----------

class PCA9575:
    REG_IN0   = 0x00
    REG_IN1   = 0x01
    REG_INV0  = 0x02
    REG_INV1  = 0x03
    REG_CFG0  = 0x08
    REG_CFG1  = 0x09
    REG_OUT0  = 0x0A
    REG_OUT1  = 0x0B

    def __init__(self, bus: int, addr: int):
        self.bus = SMBus(bus)
        self.addr = addr & 0x7F

    def _w(self, reg: int, val: int):
        self.bus.i2c_rdwr(i2c_msg.write(self.addr, bytes([reg, val & 0xFF])))

    def _r(self, reg: int) -> int:
        self.bus.i2c_rdwr(i2c_msg.write(self.addr, bytes([reg])))
        rd = i2c_msg.read(self.addr, 1)
        self.bus.i2c_rdwr(rd)
        return list(rd)[0]

    def setup_output(self, pin: int, level: int):
        pin = int(pin)
        level = 1 if level else 0
        if not (0 <= pin <= 15):
            raise ValueError("PCA9575 pin 0..15")
        if pin < 8:
            mask = 1 << pin
            cfg = self._r(self.REG_CFG0) & ~mask
            out = (self._r(self.REG_OUT0) & ~mask) | (level << pin)
            self._w(self.REG_CFG0, cfg)
            self._w(self.REG_OUT0, out)
        else:
            pin -= 8
            mask = 1 << pin
            cfg = self._r(self.REG_CFG1) & ~mask
            out = (self._r(self.REG_OUT1) & ~mask) | (level << pin)
            self._w(self.REG_CFG1, cfg)
            self._w(self.REG_OUT1, out)

    def write_pin(self, pin: int, level: int):
        pin = int(pin)
        level = 1 if level else 0
        if not (0 <= pin <= 15):
            raise ValueError("PCA9575 pin 0..15")
        if pin < 8:
            mask = 1 << pin
            out = (self._r(self.REG_OUT0) & ~mask) | (level << pin)
            self._w(self.REG_OUT0, out)
        else:
            pin -= 8
            mask = 1 << pin
            out = (self._r(self.REG_OUT1) & ~mask) | (level << pin)
            self._w(self.REG_OUT1, out)

    def close(self):
        try:
            self.bus.close()
        except Exception:
            pass

# ---------- GPIO wrapper (可選) ----------

class _GPIOBase:
    BCM = 0
    OUT = 1
    HIGH = 1
    LOW = 0
    def setmode(self, mode): ...
    def setup(self, pin, mode, initial=None): ...
    def output(self, pin, level): ...
    def cleanup(self): ...

def _get_gpio():
    try:
        import RPi.GPIO as G
    except Exception:
        return None

    class RPiGPIO(_GPIOBase):
        def __init__(self):
            self.G = G
        def setmode(self, mode):
            self.G.setmode(self.G.BCM)
        def setup(self, p, m, initial=None):
            if initial is None:
                self.G.setup(p, self.G.OUT)
            else:
                self.G.setup(p, self.G.OUT,
                             initial=self.G.HIGH if initial else self.G.LOW)
        def output(self, p, l):
            self.G.output(p, self.G.HIGH if l else self.G.LOW)
        def cleanup(self):
            self.G.cleanup()

    return RPiGPIO()

# ---------- Bootloader core ----------

class BL:
    def __init__(self, bus: int, addr: int,
                 busy_timeout: float = 8.0,
                 write_cmd: int = CMD_WRITE):
        self.bus = SMBus(bus)
        self.addr = addr & 0x7F
        self.busy_timeout = busy_timeout
        self.write_cmd = write_cmd
        self.chunk = 256

    # low-level I2C

    def _w(self, data: Union[List[int], bytes]):
        if not isinstance(data, (bytes, bytearray)):
            data = bytes(data)
        self.bus.i2c_rdwr(i2c_msg.write(self.addr, data))

    def _r(self, n: int) -> bytes:
        rd = i2c_msg.read(self.addr, n)
        self.bus.i2c_rdwr(rd)
        return bytes(rd)

    def _status_poll(self, where: str,
                     timeout: Optional[float] = None) -> int:
        t0 = time.time()
        to = timeout if timeout is not None else self.busy_timeout
        while True:
            b = self._r(1)[0]
            if b == BUSY:
                if time.time() - t0 > to:
                    raise TimeoutError("BUSY timeout @ %s" % where)
                time.sleep(POLL_DELAY)
                continue
            if b == NACK:
                raise IOError("NACK @ %s" % where)
            if b != ACK:
                raise IOError("Unexpected 0x%02X @ %s" % (b, where))
            return b

    def _cmd_ack(self, c: int, where: str,
                 timeout: Optional[float] = None):
        self._w([c & 0xFF, (0xFF - (c & 0xFF)) & 0xFF])
        self._status_poll(where, timeout)

    def _addr(self, a: int):
        A = [(a >> 24) & 0xFF,
             (a >> 16) & 0xFF,
             (a >> 8) & 0xFF,
             a & 0xFF]
        self._w(A + [xor8(A)])
        self._status_poll("ADDR")

    # ---- Commands ----

    def get_ver(self) -> int:
        self._cmd_ack(CMD_GET_VER, "GET_VER")
        ver = self._r(1)[0]
        self._status_poll("GET_VER-tail")
        return ver

    def get_id(self) -> int:
        self._cmd_ack(CMD_GET_ID, "GET_ID")
        r = self._r(3)
        if len(r) != 3:
            raise IOError("GET_ID length")
        _, hi, lo = r
        pid = (hi << 8) | lo
        self._status_poll("GET_ID-tail")
        return pid

    def read(self, a: int, n: int) -> bytes:
        """READ_MEMORY (0x11):
           N = len-1, checksum = ~N,
           ACK 後直接回資料，無尾巴狀態位。
        """
        if n <= 0:
            return b""
        if n > 256:
            n = 256

        self._cmd_ack(CMD_READ, "READ")
        self._addr(a)

        L = (n - 1) & 0xFF
        chk = (~L) & 0xFF
        self._w([L, chk])

        self._status_poll("READ-len")
        return self._r(n)

    def go(self, a: int):
        self._cmd_ack(CMD_GO, "GO")
        A = [(a >> 24) & 0xFF,
             (a >> 16) & 0xFF,
             (a >> 8) & 0xFF,
             a & 0xFF]
        self._w(A + [xor8(A)])
        self._status_poll("GO-tail")

    def close(self):
        try:
            self.bus.close()
        except Exception:
            pass

    # No-Stretch ERASE (0x45)

    def erase_pages_ns(self, pages: List[int]):
        if not pages:
            return
        pages = sorted(set(int(p) & 0xFFFF for p in pages))
        MAX_PER = 15

        i = 0
        while i < len(pages):
            chunk = pages[i:i+MAX_PER]
            i += len(chunk)

            self._cmd_ack(CMD_ERASE_NS, "ERASE_NS")

            N = len(chunk) - 1
            n_hi = (N >> 8) & 0xFF
            n_lo = N & 0xFF
            n_chk = xor8([n_hi, n_lo])
            self._w([n_hi, n_lo, n_chk])
            self._status_poll("ERASE_NS-N", timeout=2.0)

            pl = []
            chk = 0
            for p in chunk:
                hi = (p >> 8) & 0xFF
                lo = p & 0xFF
                pl.extend([hi, lo])
                chk ^= hi
                chk ^= lo
            self._w(pl + [chk])

            self._status_poll(
                "ERASE_NS-pages[%d..%d]" % (chunk[0], chunk[-1]),
                timeout=120.0
            )

    # WRITE / WRITE_NS — 每段完整 command

    def write_chunk(self, a: int, data: bytes):
        if not data:
            return

        max_payload = int(self.chunk)
        if max_payload < 1:
            max_payload = 1
        if max_payload > 256:
            max_payload = 256

        off = 0
        total = len(data)
        while off < total:
            seg = data[off:off+max_payload]
            addr = a + off

            # 1) CMD
            self._cmd_ack(self.write_cmd, "WRITE-cmd")

            # 2) ADDR
            self._addr(addr)

            # 3) frame: N, data, XOR(N + data)
            N = len(seg) - 1
            frame = [N & 0xFF] + list(seg)
            frame.append(xor8(frame))
            self._w(frame)

            # 4) Busy/ACK
            self._status_poll("WRITE-tail",
                              timeout=max(1.0, 0.002 * len(seg)))

            off += len(seg)
            time.sleep(0.001)

# ---------- Enter BL helpers ----------

def _exp_level_for_mcu(asserted: int, inverted: int) -> int:
    return (1 if asserted else 0) ^ (1 if inverted else 0)

def enter_bl_via_expander(bus: int, addr: int,
                          boot0_pin: int, nrst_pin: int,
                          boot0_inv: int, nrst_inv: int,
                          hold_ms: int = 10,
                          settle_ms: int = 200) -> PCA9575:
    xp = PCA9575(bus, addr)
    # 進 bootloader: BOOT0=1, NRST=0 -> 再放 NRST=1
    xp.setup_output(boot0_pin, _exp_level_for_mcu(1, boot0_inv))
    xp.setup_output(nrst_pin,  _exp_level_for_mcu(0, nrst_inv))
    time.sleep(max(0, hold_ms)/1000.0)
    xp.write_pin(nrst_pin, _exp_level_for_mcu(1, nrst_inv))
    time.sleep(max(0, settle_ms)/1000.0)
    return xp

def enter_bl_gpio(boot0: int, nrst: int,
                  hold_ms: int = 10,
                  settle_ms: int = 200):
    gpio = _get_gpio()
    if gpio is None:
        raise RuntimeError("RPi.GPIO not available")
    try:
        gpio.setmode(_GPIOBase.BCM)
        gpio.setup(boot0, _GPIOBase.OUT, initial=_GPIOBase.LOW)
        gpio.setup(nrst,  _GPIOBase.OUT, initial=_GPIOBase.LOW)
        gpio.output(boot0, _GPIOBase.HIGH)
        gpio.output(nrst,  _GPIOBase.LOW)
        time.sleep(max(0, hold_ms)/1000.0)
        gpio.output(nrst,  _GPIOBase.HIGH)
        time.sleep(max(0, settle_ms)/1000.0)
    finally:
        gpio.cleanup()

def post_run_reset_gpio(boot0: int, nrst: int,
                        pulse_ms: int = 5,
                        settle_ms: int = 50):
    gpio = _get_gpio()
    if gpio is None:
        return
    try:
        gpio.setmode(_GPIOBase.BCM)
        gpio.setup(boot0, _GPIOBase.OUT)
        gpio.setup(nrst,  _GPIOBase.OUT)
        gpio.output(boot0, _GPIOBase.LOW)
        gpio.output(nrst,  _GPIOBase.LOW)
        time.sleep(max(0, pulse_ms)/1000.0)
        gpio.output(nrst,  _GPIOBase.HIGH)
        time.sleep(max(0, settle_ms)/1000.0)
    finally:
        gpio.cleanup()

def xpdr_post_run_reset(xp: PCA9575,
                        boot0_pin: int,
                        nrst_pin: int,
                        boot0_inv: int,
                        nrst_inv: int,
                        pulse_ms: int = 5,
                        settle_ms: int = 50):
    """使用 PCA9575 讓 MCU 回到正常啟動:
       BOOT0=0, NRST 低脈衝後回高。
    """
    # BOOT0 -> 0
    xp.write_pin(boot0_pin, _exp_level_for_mcu(0, boot0_inv))
    # NRST pulse
    xp.write_pin(nrst_pin, _exp_level_for_mcu(0, nrst_inv))
    time.sleep(max(0, pulse_ms) / 1000.0)
    xp.write_pin(nrst_pin, _exp_level_for_mcu(1, nrst_inv))
    time.sleep(max(0, settle_ms) / 1000.0)

# ---------- erase/page helper ----------

def pages_for_range(start_addr: int, length: int) -> List[int]:
    if start_addr < FLASH_BASE:
        raise ValueError("start address < FLASH_BASE")
    first = (start_addr - FLASH_BASE) // PAGE_SIZE
    last  = (start_addr + max(0, length - 1) - FLASH_BASE) // PAGE_SIZE
    first = max(0, min(first, TOTAL_PAGES - 1))
    last  = max(0, min(last,  TOTAL_PAGES - 1))
    return list(range(first, last + 1))

def _parse_pages(s: Optional[str]) -> List[int]:
    if not s:
        return []
    out: List[int] = []
    for part in s.split(','):
        part = part.strip()
        if not part:
            continue
        if '-' in part:
            lo, hi = part.split('-', 1)
            lo = int(lo, 0); hi = int(hi, 0)
            out.extend(range(lo, hi + 1))
        else:
            out.append(int(part, 0))
    return out

def _read_u32(bl: BL, addr: int) -> int:
    return int.from_bytes(bl.read(addr, 4), "little")

# ---------- flash_file ----------

def flash_file(bus: int, addr: int, path: str, start: int, verify: bool,
               erase_mode: str, pages_opt: List[int],
               go: bool, chunk: int, write_nostretch: bool) -> bool:
    bl = BL(bus, addr,
            write_cmd=(CMD_WRITE_NS if write_nostretch else CMD_WRITE))
    bl.chunk = chunk

    try:
        ver = bl.get_ver()
        pid = bl.get_id()
        log.info("[BL] ver=0x%02X, pid=0x%04X", ver, pid)

        with open(path, "rb") as f:
            blob = f.read()
        log.info("[BL] File %s, %d bytes @ 0x%08X",
                 path, len(blob), start)

        # erase
        if erase_mode == "auto":
            pages = pages_for_range(start, len(blob))
        elif erase_mode == "pages":
            if not pages_opt:
                raise ValueError("--pages required with erase=pages")
            pages = pages_opt
        else:
            pages = []

        if pages:
            log.info("[BL] No-stretch erase pages: %d..%d (total %d)",
                     pages[0], pages[-1], len(pages))
            bl.erase_pages_ns(pages)

        # program
        log.info("[BL] Write %d bytes (chunk %d) using 0x%02X",
                 len(blob), chunk, bl.write_cmd)
        offs = 0
        while offs < len(blob):
            seg = blob[offs:offs+chunk]
            bl.write_chunk(start + offs, seg)
            offs += len(seg)

        # verify
        if verify:
            log.info("[BL] Verify...")
            offs = 0
            ok = True

            def _hex(bs: bytes) -> str:
                return " ".join(f"{b:02X}" for b in bs)

            while offs < len(blob):
                n = min(chunk, 256, len(blob) - offs)
                expect = blob[offs:offs+n]
                got = bl.read(start + offs, n)

                if got != expect:
                    head = min(16, len(expect), len(got))
                    log.error("Verify mismatch @ +0x%08X (len=%d)", offs, n)
                    log.error("  expected: %s", _hex(expect[:head]))
                    log.error("  got     : %s", _hex(got[:head]))
                    ok = False
                    break

                offs += n

            if not ok:
                return False

            log.info("[BL] Verify OK.")

        # GO
        if go:
            msp = _read_u32(bl, start)
            entry = _read_u32(bl, start + 4)
            if (entry & 1) == 0:
                entry |= 1
            log.info("[BL] GO: MSP=0x%08X, Reset=0x%08X", msp, entry)
            bl.go(entry)

        return True

    finally:
        bl.close()

# ---------- scan ----------

def scan(bus: int, likely: bool = True) -> Optional[int]:
    b = SMBus(bus)
    try:
        cand = ([0x29,0x2A,0x2B,0x2C,
                 0x45,0x46,0x47,0x48,
                 0x65,0x66,0x67,0x68] if likely
                else range(0x10, 0x7F))
        for a in cand:
            try:
                bl = BL(bus, a)
                try:
                    ver = bl.get_ver()
                    pid = bl.get_id()
                    log.info("[SCAN] 0x%02X ver=0x%02X pid=0x%04X",
                             a, ver, pid)
                    return a
                finally:
                    bl.close()
            except Exception:
                continue
        return None
    finally:
        b.close()

# ---------- CLI ----------

def main():
    ap = argparse.ArgumentParser(
        description="STM32H5 I2C ROM bootloader flasher"
    )
    ap.add_argument("--bus", type=int, required=True)
    ap.add_argument("--addr", type=lambda s:int(s,0))
    ap.add_argument("--file", required=True)
    ap.add_argument("--start", type=lambda s:int(s,0),
                    default=FLASH_BASE)
    ap.add_argument("--no-verify", dest="verify",
                    action="store_false")
    ap.add_argument("--go", action="store_true")
    ap.add_argument("--erase", choices=["auto","pages","none"],
                    default="auto")
    ap.add_argument("--pages")
    ap.add_argument("--chunk", type=int, default=64)
    ap.add_argument("--write-ns", action="store_true")
    ap.add_argument("--scan", choices=["likely","full"])

    # GPIO
    ap.add_argument("--boot0", type=int)
    ap.add_argument("--nrst",  type=int)

    # PCA9575
    ap.add_argument("--xpdr-addr", type=lambda s:int(s,0))
    ap.add_argument("--xpdr-boot0-pin", type=int)
    ap.add_argument("--xpdr-nrst-pin",  type=int)
    ap.add_argument("--xpdr-boot0-inverted", type=int,
                    default=0, choices=[0,1])
    ap.add_argument("--xpdr-nrst-inverted",  type=int,
                    default=1, choices=[0,1])

    ap.add_argument("--enter-bl", action="store_true")
    ap.add_argument("--enter-hold-ms",   type=int, default=10)
    ap.add_argument("--enter-settle-ms", type=int, default=200)

    ap.add_argument("--post-reset", action="store_true")
    ap.add_argument("--log-dir", default="./log")

    args = ap.parse_args()

    os.makedirs(args.log_dir, exist_ok=True)
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    pending = os.path.join(args.log_dir, f"{ts}_RUN.log")

    logging.basicConfig(level=logging.INFO,
                        format="%(asctime)s %(levelname)5s | %(message)s")
    fh = logging.FileHandler(pending, encoding="utf-8")
    fh.setFormatter(logging.Formatter(
        "%(asctime)s %(levelname)5s | %(message)s"))
    log.addHandler(fh)

    overall = "FAIL"
    xp = None  # 記住是否用 PCA9575 進 BL

    try:
        # 進 bootloader
        if args.enter_bl:
            if (args.xpdr_addr is not None and
                args.xpdr_boot0_pin is not None and
                args.xpdr_nrst_pin is not None):
                log.info("[XPDR] Enter BL via PCA9575@0x%02X",
                         args.xpdr_addr)
                xp = enter_bl_via_expander(args.bus, args.xpdr_addr,
                                           args.xpdr_boot0_pin,
                                           args.xpdr_nrst_pin,
                                           args.xpdr_boot0_inverted,
                                           args.xpdr_nrst_inverted,
                                           hold_ms=args.enter_hold_ms,
                                           settle_ms=args.enter_settle_ms)
            elif args.boot0 is not None and args.nrst is not None:
                log.info("[GPIO] Enter BL via BOOT0=%d NRST=%d",
                         args.boot0, args.nrst)
                enter_bl_gpio(args.boot0, args.nrst,
                              hold_ms=args.enter_hold_ms,
                              settle_ms=args.enter_settle_ms)
            else:
                log.error("--enter-bl 需要 PCA9575 或 GPIO 參數")
                return

        if args.scan:
            a = scan(args.bus, likely=(args.scan == "likely"))
            if a is None:
                log.error("Scan failed: no BL addr")
                return
            log.info("[BL] Found addr 0x%02X", a)
            if args.addr is None:
                args.addr = a

        if args.addr is None:
            log.error("--addr 未指定 (或用 --scan)")
            return

        if not (1 <= args.chunk <= 256):
            log.error("--chunk 必須在 1..256")
            return

        pages_opt = _parse_pages(args.pages)

        ok = flash_file(args.bus, args.addr, args.file,
                        args.start, args.verify,
                        args.erase, pages_opt,
                        args.go, args.chunk,
                        args.write_ns)

        overall = "OK" if ok else "FAIL"
        log.info("[MAIN] DONE." if ok else "[MAIN] FAILED.")

        # GPIO 後處理（僅在指定 post-reset 時）
        if (args.post_reset and
            args.boot0 is not None and
            args.nrst is not None):
            post_run_reset_gpio(args.boot0, args.nrst)

    finally:
        # 如果是用 PCA9575 進 BL，結束時把 BOOT0 放掉並 reset MCU（成功時）
        if xp is not None:
            try:
                if overall == "OK":
                    log.info("[XPDR] Release BOOT0 and reset target")
                    xpdr_post_run_reset(xp,
                                        args.xpdr_boot0_pin,
                                        args.xpdr_nrst_pin,
                                        args.xpdr_boot0_inverted,
                                        args.xpdr_nrst_inverted)
                else:
                    # 失敗時至少放掉 BOOT0，避免下一次上電還卡在 BL
                    log.info("[XPDR] Release BOOT0 (no reset, overall=%s)", overall)
                    xp.write_pin(args.xpdr_boot0_pin,
                                 _exp_level_for_mcu(0, args.xpdr_boot0_inverted))
            except Exception as e:
                log.error("[XPDR] post-run handling failed: %s", e)
            finally:
                try:
                    xp.close()
                except Exception:
                    pass

        for h in list(log.handlers):
            try:
                h.flush()
            except Exception:
                pass

        try:
            fh.close()
        except Exception:
            pass

        final = os.path.join(args.log_dir,
                             f"{ts}_{overall}.log")
        try:
            os.replace(pending, final)
        except Exception:
            with open(os.path.join(args.log_dir,
                                   f"{ts}_{overall}.txt"),
                      "w", encoding="utf-8") as f:
                f.write(f"{datetime.now().isoformat()} overall={overall}\n")

        log.info("[LOG] Saved: %s", final)

if __name__ == "__main__":
    main()
