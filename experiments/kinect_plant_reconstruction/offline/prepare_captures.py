"""Prepare the original 38-view capture ZIP for the dataset-specific replay."""
import argparse
import json
from pathlib import Path
import zipfile
import numpy as np

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('archive',type=Path)
    args=parser.parse_args()
    root=Path(__file__).parent
    if list(root.glob('view_*.npy')) or (root/'edges.json').exists():
        parser.error('Replay outputs already exist. Use a clean checkout/directory for another dataset.')
    # Validate all inputs before writing any output; never extract archive paths.
    arrays=[]
    with zipfile.ZipFile(args.archive) as archive:
        session=json.loads(archive.read('session.json'))
        if [v['view'] for v in session['views']]!=list(range(1,39)):
            parser.error('This replay is specific to the original 38 ordered views.')
        for i in range(1,39):
            data=archive.read(f'view_{i:06d}.ply')
            header,body=data.split(b'end_header\n',1)
            lines=header.decode('ascii').splitlines()
            if 'format binary_little_endian 1.0' not in lines:
                raise ValueError('Expected binary little-endian XYZ PLY')
            if [s for s in lines if s.startswith('property ')]!=['property float x','property float y','property float z']:
                raise ValueError('Expected float XYZ properties only')
            count=int(next(s.split()[-1] for s in lines if s.startswith('element vertex ')))
            if len(body)!=count*12:raise ValueError('Invalid PLY byte count')
            points=np.frombuffer(body,dtype='<f4').reshape(-1,3)
            if not np.isfinite(points).all():raise ValueError('Nonfinite input points')
            arrays.append(points.copy())
    for i,points in enumerate(arrays,1):np.save(root/f'view_{i:06d}.npy',points)
    print('Prepared 38 views. Run register_views.py, then export_model.py.')

if __name__=='__main__':main()
