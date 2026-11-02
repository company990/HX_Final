"""Losslessly repack FVP3 as FVP4: unchanged PCM/palette and RLE PAL8 indices.

Each packet is ctrl, then either 1 repeated index (ctrl[7]=1) or
ctrl[6:0]+1 literal indices (ctrl[7]=0). A frame ends at exactly 76800
decoded indices, so no per-frame length table is needed by the FPGA.
"""
from pathlib import Path
import argparse,hashlib,json,struct
import numpy as np
HEADER=struct.Struct('<4sHHHHIIIII')

def encode_rle(data):
 a=np.frombuffer(data,dtype='u1')
 bounds=np.r_[0,np.flatnonzero(a[1:]!=a[:-1])+1,len(a)]
 out=bytearray();literal_start=0;position=0
 def literals(start,end):
  while start<end:
   n=min(128,end-start);out.append(n-1);out.extend(data[start:start+n]);start+=n
 for left,right in zip(bounds[:-1],bounds[1:]):
  left=int(left);right=int(right)
  if right-left>=3:
   literals(literal_start,left)
   while left<right:
    n=min(128,right-left);out.extend((0x80|(n-1),data[left]));left+=n
   literal_start=right
 literals(literal_start,len(data))
 return bytes(out)

def decode_rle(data,count):
 out=bytearray();p=0
 while len(out)<count:
  c=data[p];p+=1;n=(c&127)+1
  if c&128:out.extend(bytes([data[p]])*n);p+=1
  else:out.extend(data[p:p+n]);p+=n
  if len(out)>count:raise ValueError('RLE run exceeds decoded frame length')
 return bytes(out),p

def main():
 ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('input',type=Path);ap.add_argument('output',type=Path);args=ap.parse_args()
 assert not args.output.exists(),args.output
 raw=args.input.read_bytes();hdr=HEADER.unpack_from(raw)
 assert hdr[:6]==(b'FVP3',320,240,20,1,48000) and hdr[7:]==(77312,2400,3),hdr
 frames=hdr[6];assert len(raw)==32+frames*82112
 args.output.parent.mkdir(parents=True,exist_ok=True)
 pcm_hash=hashlib.sha256();video_hash=hashlib.sha256();offsets=[];sizes=[]
 with args.output.open('xb') as f:
  f.write(HEADER.pack(b'FVP4',320,240,20,1,48000,frames,77312,2400,7))
  for i in range(frames):
   start=32+i*82112;prefix=raw[start:start+5312];indices=raw[start+5312:start+82112]
   coded=encode_rle(indices);decoded,used=decode_rle(coded,76800)
   assert decoded==indices and used==len(coded),i
   offsets.append(f.tell());sizes.append(len(prefix)+len(coded));f.write(prefix);f.write(coded)
   pcm_hash.update(prefix[:4800]);video_hash.update(prefix[4800:]+decoded)
 size=args.output.stat().st_size
 report=dict(Format='FVP4',Width=320,Height=240,FPS=20,Frames=frames,Seconds=frames/20,AudioRate=48000,AudioBits=16,AudioChannels=1,
  Source=str(args.input),SourceSHA256=hashlib.sha256(raw).hexdigest().upper(),SHA256=hashlib.sha256(args.output.read_bytes()).hexdigest().upper(),
  SourceBytes=len(raw),Bytes=size,ByteRatio=size/len(raw),BytesPerSecond=(size-32)/(frames/20),
  WorstFrameBytes=max(sizes),BestFrameBytes=min(sizes),FrameOffsets=offsets,FrameBytes=sizes,
  PCM_SHA256=pcm_hash.hexdigest().upper(),DecodedPaletteAndIndicesSHA256=video_hash.hexdigest().upper(),
  DecodedVideoAndPCMByteExact=True,HardwareVerified=False)
 args.output.with_suffix('.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
 print(json.dumps({k:v for k,v in report.items() if k not in ['FrameOffsets','FrameBytes']},indent=2),flush=True)

if __name__=='__main__':main()
