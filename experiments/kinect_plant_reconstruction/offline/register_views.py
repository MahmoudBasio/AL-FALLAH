import json
from pathlib import Path
import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation
from scipy.optimize import least_squares
from scipy.sparse import lil_matrix

ROOT=Path(__file__).parent
def transform(p,T):return p@T[:3,:3].T+T[:3,3]
def voxel(p,size):
    _,ind=np.unique(np.floor(p/size).astype(np.int64),axis=0,return_index=True)
    return p[ind]
def crop(p):
    return p[(abs(p[:,0])<.6)&(p[:,1]<.55)&(p[:,1]>-.65)&(p[:,2]>.7)&(p[:,2]<2.3)]
def matrix(x):
    T=np.eye(4);T[:3,:3]=Rotation.from_rotvec(x[:3]).as_matrix();T[:3,3]=x[3:];return T
def vector(T):return np.r_[Rotation.from_matrix(T[:3,:3]).as_rotvec(),T[:3,3]]
def rigid(x,y):
    xc=x.mean(axis=0);yc=y.mean(axis=0)
    u,s,v=np.linalg.svd((x-xc).T@(y-yc));r=v.T@u.T
    if np.linalg.det(r)<0:v[-1]*=-1;r=v.T@u.T
    T=np.eye(4);T[:3,:3]=r;T[:3,3]=yc-r@xc;return T
def icp(a,b,T,stages=((.025,.13,45),(.012,.05,45),(.007,.025,30))):
    T=T.copy()
    for size,limit,iters in stages:
        s=voxel(a,size);d=voxel(b,size);tree=cKDTree(d)
        for _ in range(iters):
            q=transform(s,T);dist,ind=tree.query(q)
            keep=dist<limit
            if keep.sum()<100:break
            keep &= dist<=np.quantile(dist[keep],.85)
            delta=rigid(q[keep],d[ind[keep]]);T=delta@T
            if np.linalg.norm(delta-np.eye(4))<1e-6:break
    return T
def score(a,b,T):
    a=transform(voxel(a,.008),T);b=voxel(b,.008)
    d=cKDTree(b).query(a)[0];e=cKDTree(a).query(b)[0]
    return dict(median_mm=[float(np.median(d)*1000),float(np.median(e)*1000)],
                within_15mm=[float((d<.015).mean()),float((e<.015).mean())])
def pair(a,b):
    candidates=[]
    for deg in [-25,0,25]:
        T=np.eye(4);T[:3,:3]=Rotation.from_euler('y',deg,degrees=True).as_matrix()
        T[:3,3]=np.median(b,axis=0)-T[:3,:3]@np.median(a,axis=0)
        T=icp(a,b,T,((.025,.15,45),(.012,.06,35)))
        sc=score(a,b,T);cost=sum(sc['median_mm'])
        candidates.append((cost,T))
    T=icp(a,b,min(candidates,key=lambda c:c[0])[1])
    return T,score(a,b,T)

if __name__=='__main__':
    clouds=[crop(np.load(ROOT/f'view_{i:06d}.npy')) for i in range(1,39)]
    edges=[]
    cache=ROOT/'edges.json'
    if cache.exists():edges=json.loads(cache.read_text())
    pairs=[(i,i-1) for i in range(1,38)]+[(i,i-2) for i in range(2,38)]+[(37,0),(36,0),(37,1)]
    existing={(e['i'],e['j']) for e in edges}
    for i,j in pairs:
        if (i,j) in existing:continue
        T,sc=pair(clouds[i],clouds[j])
        edge=dict(i=i,j=j,T=T.tolist(),**sc)
        edges.append(edge);cache.write_text(json.dumps(edges,indent=2))
        print(i+1,j+1,sc,flush=True)
    # Review scores before graph optimization; avoid forcing weak edges.
    good=[e for e in edges if min(e['within_15mm'])>.55 and max(e['median_mm'])<13]
    missing=[i+1 for i in range(1,38) if not any(e['i']==i and e['j']==i-1 for e in good)]
    print('WEAK NEIGHBORS',missing,flush=True)
    poses=[np.eye(4)]
    for i in range(1,38):
        e=next(e for e in edges if e['i']==i and e['j']==i-1)
        poses.append(poses[-1]@np.array(e['T']))
    np.save(ROOT/'poses_chain.npy',poses)
    # Six residuals per relative-pose constraint; angle weighted at 0.5 m radius.
    def residual(x):
        p=[np.eye(4)]+[matrix(v) for v in x.reshape(-1,6)]
        r=[]
        for e in good:
            err=np.linalg.inv(np.array(e['T']))@np.linalg.inv(p[e['j']])@p[e['i']]
            r.extend(np.r_[Rotation.from_matrix(err[:3,:3]).as_rotvec()*.5,err[:3,3]])
        return np.array(r)
    sparsity=lil_matrix((len(good)*6,37*6))
    for k,e in enumerate(good):
        for i in [e['i'],e['j']]:
            if i:sparsity[k*6:k*6+6,(i-1)*6:i*6]=1
    fit=least_squares(residual,np.concatenate([vector(T) for T in poses[1:]]),
                      jac_sparsity=sparsity.tocsr(),loss='soft_l1',f_scale=.005,max_nfev=150)
    optimized=np.array([np.eye(4)]+[matrix(v) for v in fit.x.reshape(-1,6)])
    np.save(ROOT/'poses_optimized.npy',optimized)
    print('OPTIMIZED',fit.success,fit.cost,fit.message,flush=True)
    print('closure before/after',score(clouds[-1],clouds[0],poses[-1]),score(clouds[-1],clouds[0],optimized[-1]),flush=True)
