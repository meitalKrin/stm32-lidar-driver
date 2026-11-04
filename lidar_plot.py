"""
Live polar plot of the STM32 LiDAR driver output (Phase 6.2 / 6.3).

The board sends one 52-byte frame per good packet over USART2 (ST-LINK COM port):
    AA 55 | 12 | 12 x (angle u16 LE in 0.01 deg, distance u16 LE in mm) | XOR checksum
Anything else on the line (boot text, the stats report) is skipped automatically.

Install once (Windows):   py -m pip install pyserial numpy matplotlib pillow PyQt6
Live view:                py lidar_plot.py COM3
Record README GIF:        py lidar_plot.py COM3 --gif lidar.gif --seconds 8

Close any serial monitor first: only one program can hold the COM port.
"""
import argparse
import struct
import time

import numpy as np

SYNC = b"\xAA\x55"
NPTS = 12
FRAME_LEN = 2 + 1 + NPTS * 4 + 1          # 52 bytes
BINS = 720                                 # 0.5 degree per bin = one full revolution
WARMUP_S = 1.0                             # fill a full revolution before recording


class FrameParser:
    """Turns a raw byte stream into (angle_deg, dist_mm) points. Resyncs on garbage."""

    def __init__(self):
        self.buf = bytearray()
        self.frames = 0
        self.bad = 0

    def feed(self, data):
        self.buf += data
        points = []
        while True:
            i = self.buf.find(SYNC)
            if i < 0:                                  # keep a trailing 0xAA, drop the rest
                del self.buf[:max(0, len(self.buf) - 1)]
                break
            if i:
                del self.buf[:i]
            if len(self.buf) < FRAME_LEN:
                break
            frame = bytes(self.buf[:FRAME_LEN])
            body = frame[2:-1]
            x = 0
            for b in body:
                x ^= b
            if body[0] != NPTS or x != frame[-1]:      # false sync (e.g. inside text): skip 1 byte
                self.bad += 1
                del self.buf[:1]
                continue
            del self.buf[:FRAME_LEN]
            self.frames += 1
            for k in range(NPTS):
                ang, dist = struct.unpack_from("<HH", body, 1 + 4 * k)
                points.append((ang / 100.0, dist))
        return points


def run(port, baud, max_mm, gif, seconds):
    import serial
    import matplotlib
    if gif:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation, PillowWriter

    ser = serial.Serial(port, baud, timeout=0)
    parser = FrameParser()
    dist = np.zeros(BINS)                      # latest distance per 0.5-degree bin (0 = no return)
    theta = np.deg2rad(np.arange(BINS) * 360.0 / BINS)

    def drain():
        for ang, d in parser.feed(ser.read(ser.in_waiting or 1)):
            dist[int(ang * BINS / 360.0) % BINS] = d

    fig = plt.figure(figsize=(6, 6))
    ax = fig.add_subplot(projection="polar")
    ax.set_theta_zero_location("N")
    ax.set_theta_direction(-1)                 # clockwise, like the sensor
    ax.set_ylim(0, max_mm)
    ax.set_title("LDROBOT LiDAR on STM32F411 (bare-metal UART + circular DMA)", fontsize=9)
    scat = ax.scatter([], [], s=4)
    info = ax.text(0.02, -0.08, "", transform=ax.transAxes, fontsize=8)

    fps = 15
    if gif:
        # Fill a full revolution so the first frame is not a partial scan.
        end = time.time() + WARMUP_S
        while time.time() < end:
            drain()
            time.sleep(0.005)

    t0 = time.time()
    last = [t0, parser.frames]

    def update(frame_idx):
        if gif:
            # Agg renders faster than real time; pace each frame to the wall clock.
            delay = t0 + frame_idx / fps - time.time()
            if delay > 0:
                time.sleep(delay)
        drain()
        ok = dist > 0
        scat.set_offsets(np.column_stack((theta[ok], dist[ok])))
        now = time.time()
        if now - last[0] >= 1.0:
            rate = (parser.frames - last[1]) / (now - last[0])
            info.set_text(f"{rate:.0f} packets/s   bad syncs skipped: {parser.bad}")
            last[0], last[1] = now, parser.frames
        return scat, info

    if gif:
        anim = FuncAnimation(fig, update, frames=int(seconds * fps), interval=1000 / fps, blit=False)
        anim.save(gif, writer=PillowWriter(fps=fps))
        print(f"saved {gif}")
    else:
        anim = FuncAnimation(fig, update, interval=1000 / fps, blit=False, cache_frame_data=False)
        plt.show()
    ser.close()


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("port", help="e.g. COM3")
    ap.add_argument("--baud", type=int, default=460800)
    ap.add_argument("--max-mm", type=int, default=3000, help="plot radius in mm")
    ap.add_argument("--gif", help="record a GIF instead of showing a window")
    ap.add_argument("--seconds", type=float, default=8)
    a = ap.parse_args()
    run(a.port, a.baud, a.max_mm, a.gif, a.seconds)
