"""Offline FVP3 encoder: 320x240, 20fps, per-frame 256-colour RGB565 palette.

Requires ffmpeg and Pillow. FPGA performs playback; MP4 decoding happens here.
Existing files are never overwritten. Audio is 48kHz mono signed PCM16.
"""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import struct
import subprocess
import tempfile

from PIL import Image

WIDTH, HEIGHT, FPS, RATE = 320, 240, 20, 48000
SAMPLES = RATE // FPS
VIDEO_BYTES = 512 + WIDTH * HEIGHT
HEADER = struct.Struct('<4sHHHHIIIII')


def encode_frame(rgb):
    image = Image.frombytes('RGB', (WIDTH, HEIGHT), rgb)
    # Quantize each complete frame; the hardware expands its palette before
    # storing RGB565. No dependency on the preceding frame's colour table.
    quantized = image.quantize(colors=256, method=Image.Quantize.MEDIANCUT,
                              dither=Image.Dither.NONE)
    palette = quantized.getpalette()
    table = bytearray()
    for index in range(256):
        r, g, b = palette[index*3:index*3+3]
        table += struct.pack('<H', ((r >> 3) << 11) | ((g >> 2) << 5) | (b >> 3))
    return table + quantized.tobytes()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('input', type=Path)
    parser.add_argument('--output', type=Path, default=Path('VIDEO.FVP'))
    parser.add_argument('--seconds', type=float, default=30)
    parser.add_argument('--ffmpeg', default='ffmpeg')
    args = parser.parse_args()
    if not args.input.is_file():
        parser.error('Input media does not exist')
    if args.output.exists():
        parser.error('Output already exists; use a new directory or filename')
    max_frames=min(65535,(0xffffffff-32)//(VIDEO_BYTES+SAMPLES*2))
    if not 0 < args.seconds <= max_frames/FPS:
        parser.error('Duration exceeds the frame-count or FAT32 file-size limit')
    executable = shutil.which(args.ffmpeg)
    if not executable:
        parser.error('ffmpeg not found; supply --ffmpeg with its absolute path')
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix='fvp3_') as folder:
        folder = Path(folder)
        common = [executable, '-nostdin', '-v', 'error', '-i', str(args.input),
                  '-t', str(args.seconds)]
        subprocess.run(common + ['-map', '0:v:0', '-an', '-vf',
            f'fps={FPS},scale={WIDTH}:{HEIGHT}:force_original_aspect_ratio=decrease:flags=lanczos,'
            f'pad={WIDTH}:{HEIGHT}:(ow-iw)/2:(oh-ih)/2',
            '-pix_fmt', 'rgb24', '-f', 'rawvideo', str(folder/'video.rgb')], check=True)
        audio = subprocess.run(common + ['-map', '0:a:0?', '-vn', '-ac', '1', '-ar', str(RATE),
            '-c:a', 'pcm_s16le', '-f', 's16le', str(folder/'audio.pcm')], capture_output=True, text=True)
        if audio.returncode:
            if 'does not contain any stream' in audio.stderr:
                (folder/'audio.pcm').write_bytes(b'')
            else:
                raise RuntimeError(audio.stderr)
        frames = (folder/'video.rgb').stat().st_size // (WIDTH*HEIGHT*3)
        if not 1 <= frames <= 65535:
            raise ValueError('Decoded frame count outside supported range')
        with args.output.open('xb') as output, (folder/'video.rgb').open('rb') as video, (folder/'audio.pcm').open('rb') as pcm:
            output.write(HEADER.pack(b'FVP3', WIDTH, HEIGHT, FPS, 1, RATE, frames, VIDEO_BYTES, SAMPLES, 3))
            for frame in range(frames):
                image_data=encode_frame(video.read(WIDTH*HEIGHT*3))
                block = pcm.read(SAMPLES*2)
                output.write(block + bytes(SAMPLES*2-len(block)))
                output.write(image_data)
        expected = 32 + frames*(VIDEO_BYTES+SAMPLES*2)
        assert args.output.stat().st_size == expected
    report = dict(Format='FVP3', Width=WIDTH, Height=HEIGHT, FPS=FPS,
        ColoursPerFrame=256, AudioRate=RATE, AudioChannels=1, AudioBits=16,
        Frames=frames, Seconds=frames/FPS, Bytes=expected,
        BytesPerSecond=FPS*(VIDEO_BYTES+SAMPLES*2),
        SHA256=hashlib.sha256(args.output.read_bytes()).hexdigest().upper(),
        Source=str(args.input.resolve()), HardwareVerified=False)
    args.output.with_suffix('.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
