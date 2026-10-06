#!/usr/bin/env python3
"""Analyse an FM capture (.cf32) made with sdr_capture.py.

Demodulates the FM signal and reports the carrier offset, peak deviation and
the strongest components of the multiplex (in Hz of deviation). Optionally
writes the demodulated multiplex as a WAV file (e.g. for an RDS decoder).

    tools/fm_analyze.py capture.cf32 --rate 1e6 [--mpx-wav mpx.wav] [--components 19000,38000,57000]
"""
import argparse
import sys
import wave

import numpy as np
from scipy import signal


def demod(iq, rate):
    ph = np.angle(iq[1:] * np.conj(iq[:-1]))
    return ph * rate / (2 * np.pi)


def tone_amplitude(x, rate, f, bw=50.0):
    """Peak amplitude of the sinusoidal component at f (flat-top window)."""
    n = len(x)
    w = signal.windows.flattop(n)
    spec = np.fft.rfft(x * w)
    freqs = np.fft.rfftfreq(n, 1 / rate)
    sel = (freqs > f - bw) & (freqs < f + bw)
    if not np.any(sel):
        return 0.0, f
    i = np.argmax(np.abs(spec[sel]))
    amp = 2 * np.abs(spec[sel][i]) / np.sum(w)
    return amp, freqs[sel][i]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("capture")
    ap.add_argument("--rate", type=float, default=1e6)
    ap.add_argument("--channel-bw", type=float, default=150e3, help="IF filter bandwidth")
    ap.add_argument("--mpx-wav", help="write demodulated MPX to this WAV file")
    ap.add_argument("--mpx-rate", type=float, default=250e3)
    ap.add_argument("--components", default="", help="comma separated frequencies to measure")
    ap.add_argument("--top", type=int, default=8)
    args = ap.parse_args()

    iq = np.fromfile(args.capture, dtype=np.complex64)
    rate = args.rate
    print(f"{len(iq)} samples, {len(iq)/rate:.2f} s, power {10*np.log10(np.mean(np.abs(iq)**2)+1e-20):.1f} dBFS")

    # Coarse carrier offset from the spectrum peak, then shift to DC.
    nfft = 1 << 18
    seg = iq[: nfft * 4]
    psd = np.abs(np.fft.fftshift(np.fft.fft(seg[:nfft] * np.hanning(nfft)))) ** 2
    f_axis = np.fft.fftshift(np.fft.fftfreq(nfft, 1 / rate))
    # Use the instantaneous frequency mean for the offset (robust for FM).
    taps = signal.firwin(255, args.channel_bw / 2, fs=rate)
    filt = signal.lfilter(taps, 1, iq)
    inst = demod(filt, rate)[255:]
    offset = np.mean(inst)
    mpx = inst - offset
    print(f"carrier offset: {offset:+.1f} Hz")
    print(f"deviation: peak {np.max(np.abs(mpx)):.0f} Hz, 99.9% {np.percentile(np.abs(mpx), 99.9):.0f} Hz, rms {np.std(mpx):.0f} Hz")

    # Strongest components.
    n = 1 << int(np.log2(min(len(mpx), int(rate * 2))))
    w = signal.windows.blackmanharris(n)
    spec = np.abs(np.fft.rfft(mpx[:n] * w)) * 2 / np.sum(w)
    freqs = np.fft.rfftfreq(n, 1 / rate)
    lim = freqs < 100e3
    s = spec.copy()
    s[~lim] = 0
    print("strongest components (Hz of deviation):")
    for _ in range(args.top):
        i = np.argmax(s)
        if s[i] <= 0:
            break
        print(f"  {freqs[i]:10.1f} Hz : {s[i]:9.1f} Hz ({20*np.log10(s[i]/75000+1e-20):6.1f} dB re 75 kHz)")
        lo, hi = max(0, i - 20), i + 21
        s[lo:hi] = 0
    if args.components:
        for f in [float(x) for x in args.components.split(",")]:
            a, fp = tone_amplitude(mpx[:n], rate, f)
            print(f"  component {f:.0f} Hz: {a:.1f} Hz deviation (at {fp:.2f} Hz)")

    if args.mpx_wav:
        q = signal.resample_poly(mpx, int(args.mpx_rate), int(rate))
        q = np.clip(q / 100e3, -1, 1)
        with wave.open(args.mpx_wav, "wb") as wf:
            wf.setnchannels(1)
            wf.setsampwidth(2)
            wf.setframerate(int(args.mpx_rate))
            wf.writeframes((q * 32767).astype("<i2").tobytes())
        print(f"wrote {args.mpx_wav} ({args.mpx_rate:.0f} Hz, full scale = 100 kHz deviation)", file=sys.stderr)
        try:
            # Propagate the capture timestamp; the demodulator dropped 255
            # samples and its channel filter delays by 127.
            import json
            meta = json.load(open(args.capture + ".json"))
            meta["start_time"] += (255 - 127) / rate
            json.dump(meta, open(args.mpx_wav + ".json", "w"))
        except OSError:
            pass


if __name__ == "__main__":
    main()
