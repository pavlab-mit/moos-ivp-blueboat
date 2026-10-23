#!/usr/bin/env python3
"""
gen_rc_sounds.py -- generate EdgeTX voice callouts for the BlueBoat
RC handsets.

Edit TRACKS below (filename -> spoken text), pick a voice, run:

    pip install piper-tts
    python3 gen_rc_sounds.py                       # default voice (amy)
    python3 gen_rc_sounds.py --voice en_US-ryan-medium
    python3 gen_rc_sounds.py --list-common-voices

Output: ./sounds/<name>.wav, EdgeTX-ready (16 kHz, 16-bit, mono).
Copy them to the handset SD card's SOUNDS/en/ folder. Filenames must
stay <= 8 characters (EdgeTX limit); the script enforces this.

Voice catalog with playable samples: https://rhasspy.github.io/piper-samples/
Voices auto-download on first use (cached afterwards).
"""

# ---- EDIT ME: filename -> what the radio says -----------------
TRACKS = {
    "rcmode":  "R C mode",
    "auto":    "auto mode",
    "siglow":  "signal low",
    "batlow":  "battery low",
    "batcrit": "battery critical",
    "tlmlost": "telemetry lost",
    "marked":  "mark",
    "killed":  "emergency stop",
    "running": "vehicle engaged",
    "dmanoff": "deadman disabled",
    "dmanon":  "deadman enabled",
}
# ---------------------------------------------------------------

COMMON_VOICES = [
    "en_US-amy-medium", "en_US-lessac-medium", "en_US-ryan-medium",
    "en_US-hfc_male-medium", "en_US-hfc_female-medium", "en_US-joe-medium",
    "en_US-kristin-medium", "en_GB-alan-medium", "en_GB-cori-medium",
    "en_GB-jenny_dioco-medium", "en_GB-southern_english_female-medium",
]

import argparse, os, subprocess, sys, wave

def resample_to_edgetx(src, dst):
    """16 kHz / 16-bit / mono. Prefer ffmpeg; fall back to audioop."""
    try:
        subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-i", src,
                        "-ar", "16000", "-ac", "1", "-sample_fmt", "s16", dst],
                       check=True)
        return
    except (FileNotFoundError, subprocess.CalledProcessError):
        pass
    import audioop
    with wave.open(src, "rb") as w:
        nch, sw, rate = w.getnchannels(), w.getsampwidth(), w.getframerate()
        frames = w.readframes(w.getnframes())
    if nch > 1:
        frames = audioop.tomono(frames, sw, 0.5, 0.5)
    if sw != 2:
        frames = audioop.lin2lin(frames, sw, 2)
    if rate != 16000:
        frames, _ = audioop.ratecv(frames, 2, 1, rate, 16000, None)
    with wave.open(dst, "wb") as w:
        w.setnchannels(1); w.setsampwidth(2); w.setframerate(16000)
        w.writeframes(frames)

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--voice", default="en_US-amy-medium")
    ap.add_argument("--out", default="sounds")
    ap.add_argument("--list-common-voices", action="store_true")
    args = ap.parse_args()

    if args.list_common_voices:
        print("\n".join(COMMON_VOICES))
        print("\nFull catalog + samples: https://rhasspy.github.io/piper-samples/")
        return

    bad = [n for n in TRACKS if len(n) > 8]
    if bad:
        sys.exit(f"filenames over EdgeTX's 8-char limit: {bad}")

    from piper import PiperVoice
    model = f"{args.voice}.onnx"
    if not os.path.exists(model):
        print(f"downloading voice {args.voice} ...")
        subprocess.run([sys.executable, "-m", "piper.download_voices",
                        args.voice], check=True)

    voice = PiperVoice.load(model)
    os.makedirs(args.out, exist_ok=True)
    for name, text in TRACKS.items():
        raw = os.path.join(args.out, f".{name}_raw.wav")
        with wave.open(raw, "wb") as w:
            voice.synthesize_wav(text, w)
        resample_to_edgetx(raw, os.path.join(args.out, f"{name}.wav"))
        os.remove(raw)
        print(f"  {name}.wav  <- \"{text}\"")
    print(f"\ndone -> {args.out}/  (copy to SD card SOUNDS/en/)")

if __name__ == "__main__":
    main()
