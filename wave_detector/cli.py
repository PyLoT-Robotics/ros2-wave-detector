import argparse
import json
from pathlib import Path
import numpy as np
from .core import WaveDetector


def demo_frames():
    for i,x in enumerate([20,40,70,40,20,50,75]):
        points=np.zeros((17,3));points[5]=[35,60,1];points[6]=[65,60,1];points[9]=[x,30,1]
        yield {'stamp':i*.1,'people':[{'track_id':1,'bbox':[0,0,100,200],'keypoints':points.tolist()}]}


def main(argv=None):
    p=argparse.ArgumentParser(description='Temporal wave detection from tracked COCO17 pose JSONL')
    s=p.add_subparsers(dest='command',required=True);s.add_parser('demo')
    run=s.add_parser('replay');run.add_argument('input',type=Path)
    a=p.parse_args(argv);detector=WaveDetector();count=0
    try:
        frames=list(demo_frames()) if a.command=='demo' else [json.loads(line) for line in a.input.read_text().splitlines() if line.strip()]
        for frame in frames:
            for person in frame['people']:
                event=detector.update(person['track_id'],frame['stamp'],person['keypoints'],person['bbox'],frame.get('camera_moving',False))
                if event:count+=1;print(json.dumps(event))
        if a.command=='demo':assert count>0
    except (ValueError,TypeError,KeyError,OSError) as e:p.exit(2,str(e)+'\n')
    return 0


if __name__=='__main__':raise SystemExit(main())
