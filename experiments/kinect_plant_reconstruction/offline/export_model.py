import json
import sys
import numpy as np
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from scipy.spatial import cKDTree
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from register_views import transform,score,crop
from view_plant_model import read_xyz

ROOT=Path(__file__).parent
def floor_crop(p):
    # Fit the floor from points outside the centred plant, in this camera frame.
    q=p[(abs(p[:,0])>.65)&(abs(p[:,0])<1.2)&(p[:,1]>.45)&(p[:,1]<1)&(p[:,2]>.7)&(p[:,2]<2.5)]
    rng=np.random.default_rng(18)
    q=q[rng.choice(len(q),min(len(q),8000),replace=False)]
    best=None;bestn=0
    for _ in range(100):
        s=q[rng.choice(len(q),3,replace=False)]
        A=np.c_[s[:,0],s[:,2],np.ones(3)]
        if np.linalg.cond(A)>1e5:continue
        c=np.linalg.solve(A,s[:,1])
        if np.linalg.norm(c[:2])>.3:continue
        mask=abs(q[:,1]-np.c_[q[:,0],q[:,2],np.ones(len(q))]@c)<.012
        if mask.sum()>bestn:bestn=mask.sum();best=(c,mask)
    if best is None:raise ValueError('Floor fit failed')
    c=np.linalg.lstsq(np.c_[q[best[1],0],q[best[1],2],np.ones(bestn)],q[best[1],1],rcond=None)[0]
    floor=np.c_[p[:,0],p[:,2],np.ones(len(p))]@c
    return p[(abs(p[:,0])<.6)&(p[:,1]>-.65)&(p[:,1]<floor-.025)&(p[:,2]>.7)&(p[:,2]<2.3)]
def voxel_mean(p,size):
    k,inv=np.unique(np.floor(p/size).astype(np.int64),axis=0,return_inverse=True)
    counts=np.bincount(inv)
    return k,np.column_stack([np.bincount(inv,weights=p[:,a])/counts for a in range(3)])
def write_ply(path,points):
    header=f'ply\nformat binary_little_endian 1.0\nelement vertex {len(points)}\nproperty float x\nproperty float y\nproperty float z\nend_header\n'
    with path.open('wb') as stream:stream.write(header.encode());stream.write(points.astype('<f4').tobytes())

poses=np.load(ROOT/'poses_optimized.npy')
views=[floor_crop(np.load(ROOT/f'view_{i:06d}.npy')) for i in range(1,39)]
allpoints=[];keys=[]
for p,T in zip(views,poses):
    p=transform(p,T)
    k,p=voxel_mean(p,.005);keys.append(k);allpoints.append(p)
points=np.concatenate(allpoints);keys=np.concatenate(keys)
_,inv=np.unique(keys,axis=0,return_inverse=True);support=np.bincount(inv)
points=np.column_stack([np.bincount(inv,weights=points[:,i])/support for i in range(3)])
points=points[support>=2] # At least two captures, not necessarily independent camera positions.
dist,_=cKDTree(points).query(points,k=7)
points=points[dist[:,-1]<.025]
# Rotate optical coordinates to the usual forward/left/up display convention.
model_points=np.c_[points[:,2],-points[:,0],-points[:,1]]
write_ply(ROOT/'plant_combined_38.ply',model_points)
np.testing.assert_allclose(read_xyz(ROOT/'plant_combined_38.ply'),model_points,atol=1e-6)
fig,axs=plt.subplots(1,3,figsize=(14,6))
for ax,(x,y),title in zip(axs,[(0,1),(0,2),(2,1)],['Front','Top','Side']):
    ax.scatter(points[:,x],-points[:,y] if y==1 else points[:,y],s=.15,c=-points[:,1],cmap='viridis',rasterized=True)
    ax.set_aspect('equal');ax.set_title(title);ax.set_xlabel(['X (m)','Y (m)','Z (m)'][x]);ax.set_ylabel('Height direction (m)' if y==1 else 'Z (m)')
fig.suptitle('Combined plant — 38 aligned captures\n5 mm fusion grid; grid size is not measured accuracy')
fig.tight_layout();fig.savefig(ROOT/'plant_combined_preview.png',dpi=170)
edges=json.loads((ROOT/'edges.json').read_text());reports=[]
for e in edges:
    if e['i']!=e['j']+1 and (e['i'],e['j'])!=(37,0):continue
    T=np.linalg.inv(poses[e['j']])@poses[e['i']]
    reports.append(dict(source=e['i']+1,target=e['j']+1,**score(crop(views[e['i']]),crop(views[e['j']]),T)))
report=dict(points=len(points),input_views=38,voxel_m=.005,min_capture_support=2,
            pose_frame='first camera optical view',poses=poses.tolist(),alignment_checks=reports,
            model_from_first_optical=[[0,0,1,0],[-1,0,0,0],[0,-1,0,0],[0,0,0,1]],
            limitations=['Geometry only; no RGB','Rigid alignment is approximate, not independently calibrated',
                         'Thin/occluded/unsupported surfaces may be missing','No surface completion or watertight mesh',
                         'Floor clearance removes bottom 25 mm near the floor',
                         'Two-capture support can include repeated camera positions'])
(ROOT/'alignment_report.json').write_text(json.dumps(report,indent=2))
print('EXPORTED',len(points), 'points',flush=True)
print('ADJACENT MEDIAN MM',np.median([e['median_mm'] for e in reports[:-1]],axis=0),flush=True)
print('CLOSURE',reports[-1],flush=True)
