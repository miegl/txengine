#!/usr/bin/env python3
"""Capture complex baseband from an SDR (SoapySDR) to a .cf32 file.

Used to verify txengine's RF output, e.g. with a USRP B210:

    tools/sdr_capture.py --freq 87.6e6 --rate 1e6 --seconds 5 -o capture.cf32

The Raspberry Pi outputs a square wave. A direct-conversion receiver tuned
exactly to the carrier also mixes the signal's odd harmonics with its own LO
harmonics, which land on top of the wanted signal (at -3x, +5x, -7x the
frequency offset). The receiver is therefore tuned `--lo-offset` away from
the carrier and the wanted channel is shifted back and decimated digitally.
"""
import argparse
import sys
import time

import numpy as np
import SoapySDR
from SoapySDR import SOAPY_SDR_RX, SOAPY_SDR_CF32


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--freq", type=float, default=87.6e6)
    ap.add_argument("--rate", type=float, default=1e6, help="output sample rate")
    ap.add_argument("--lo-offset", type=float, default=600e3, help="receiver LO offset from --freq")
    ap.add_argument("--hw-rate", type=float, default=4e6, help="receiver sample rate")
    ap.add_argument("--bw", type=float, default=0, help="analog bandwidth (0 = rate)")
    ap.add_argument("--gain", type=float, default=20)
    ap.add_argument("--seconds", type=float, default=5)
    ap.add_argument("--device", default="driver=uhd")
    ap.add_argument("--antenna", default="")
    ap.add_argument("--delay", type=float, default=0, help="wait before capturing")
    ap.add_argument("-o", "--output", default="capture.cf32")
    args = ap.parse_args()

    sdr = SoapySDR.Device(args.device)
    sdr.setSampleRate(SOAPY_SDR_RX, 0, args.hw_rate)
    sdr.setBandwidth(SOAPY_SDR_RX, 0, args.bw or args.hw_rate)
    sdr.setFrequency(SOAPY_SDR_RX, 0, args.freq + args.lo_offset)
    sdr.setGain(SOAPY_SDR_RX, 0, args.gain)
    if args.antenna:
        sdr.setAntenna(SOAPY_SDR_RX, 0, args.antenna)
    rate = sdr.getSampleRate(SOAPY_SDR_RX, 0)
    lo = sdr.getFrequency(SOAPY_SDR_RX, 0)
    total = int(args.seconds * rate)
    out = np.empty(total, dtype=np.complex64)
    # Timestamp samples with host (NTP) time via the device clock.
    try:
        sdr.setHardwareTime(int(time.time() * 1e9))
        timed = True
    except Exception:
        timed = False
    stream = sdr.setupStream(SOAPY_SDR_RX, SOAPY_SDR_CF32)
    if args.delay:
        time.sleep(args.delay)
    sdr.activateStream(stream)
    buf = np.empty(65536, dtype=np.complex64)
    got = 0
    start_ns = None
    # Discard the first 100 ms (AGC/DC settling).
    skip = int(0.1 * rate)
    overflows = 0
    while got < total:
        sr = sdr.readStream(stream, [buf], len(buf), timeoutUs=1000000)
        if sr.ret < 0:
            overflows += 1
            continue
        n = sr.ret
        d = 0
        if skip > 0:
            d = min(skip, n)
            skip -= d
            if d == n:
                continue
        chunk = buf[d:n]
        if start_ns is None:
            start_ns = sr.timeNs + int(d / rate * 1e9) if timed and sr.timeNs else int(time.time() * 1e9)
        m = min(len(chunk), total - got)
        out[got:got + m] = chunk[:m]
        got += m
    sdr.deactivateStream(stream)
    sdr.closeStream(stream)
    # Shift the wanted channel (at freq - lo) to DC and decimate.
    from scipy import signal
    shift = args.freq - lo
    t = np.arange(len(out)) / rate
    bb = out * np.exp(-2j * np.pi * shift * t).astype(np.complex64)
    dec = int(round(rate / args.rate))
    if dec > 1:
        bb = signal.resample_poly(bb, 1, dec, window=("kaiser", 10.0)).astype(np.complex64)
    bb.tofile(args.output)
    with open(args.output + ".json", "w") as f:
        f.write('{"start_time": %.6f, "rate": %.1f, "hw_timestamps": %s}\n' % (start_ns / 1e9, rate / dec, "true" if timed else "false"))
    # Full scale of the CF32 stream is 1.0: peaks near it mean the ADC clips
    # (strong stations anywhere in the analog bandwidth count too).
    peak = float(np.max(np.abs(out[:got]))) if got else 0.0
    print(f"captured {got} samples at {rate:.0f} S/s (LO {lo/1e6:.4f} MHz), wrote {len(bb)} at "
          f"{rate/dec:.0f} S/s to {args.output} ({overflows} stream errors), raw peak {20*np.log10(peak+1e-12):.1f} dBFS, "
          f"wanted channel {10*np.log10(np.mean(np.abs(bb)**2)+1e-20):.1f} dBFS", file=sys.stderr)
    if peak > 0.9:
        print("warning: the receiver input is close to clipping; reduce --gain", file=sys.stderr)


if __name__ == "__main__":
    main()
