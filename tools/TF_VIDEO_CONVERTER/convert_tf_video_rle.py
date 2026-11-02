"""Offline MP4 -> FVP4, reusing the existing PAL8/PCM encoder.

The final FPGA implements RLE expansion and playback, not H.264/AAC decoding.
For an existing FVP3, use convert_fvp_rle.py to preserve its pixels/PCM exactly.
"""
import argparse,subprocess,sys,tempfile
from pathlib import Path

def main():
 ap=argparse.ArgumentParser(description=__doc__)
 ap.add_argument('input',type=Path);ap.add_argument('--output',type=Path,default=Path('VIDEO.FVP'))
 ap.add_argument('--seconds',type=float,default=30);ap.add_argument('--ffmpeg',default='ffmpeg')
 args=ap.parse_args();assert not args.output.exists(),args.output
 root=Path(__file__).resolve().parent
 with tempfile.TemporaryDirectory(prefix='fvp4_') as folder:
  intermediate=Path(folder)/'raw.FVP'
  subprocess.run([sys.executable,str(root/'convert_tf_video_plus.py'),str(args.input),'--output',str(intermediate),
                  '--seconds',str(args.seconds),'--ffmpeg',args.ffmpeg],check=True)
  subprocess.run([sys.executable,str(root/'convert_fvp_rle.py'),str(intermediate),str(args.output)],check=True)
if __name__=='__main__':main()
