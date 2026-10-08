"""Render the actual saved XYZ model for the repository README (no ROS needed)."""
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from view_plant_model import read_xyz

root=Path(__file__).parent
points=read_xyz(root/'results/scan_38/plant_combined_38.ply')
rng=np.random.default_rng(12)
shown=points[rng.choice(len(points),min(120000,len(points)),replace=False)]
fig=plt.figure(figsize=(14,8),facecolor='#101b27')
ax=fig.add_axes([.05,.08,.9,.76],projection='3d',facecolor='#101b27')
ax.scatter(shown[:,0],shown[:,1],shown[:,2],c=shown[:,2],cmap='viridis',
           s=.45,alpha=.8,linewidths=0,depthshade=False)
lo,hi=points.min(axis=0),points.max(axis=0)
ax.set_box_aspect(hi-lo)
ax.set_xlim(lo[0]-.04,hi[0]+.04);ax.set_ylim(lo[1]-.04,hi[1]+.04)
ax.set_zlim(lo[2]-.04,hi[2]+.04)
ax.view_init(elev=12,azim=140)
ax.set_axis_off()
fig.text(.055,.925,'AL-FALLAH  /  Plant reconstruction',color='white',fontsize=24,weight='bold')
fig.text(.055,.875,'Kinect v1   ·   38 aligned captures   ·   283,501 points',color='#b2c6d6',fontsize=14)
fig.text(.055,.045,'Actual scan data · colours indicate height · geometry-only research prototype',color='#b2c6d6',fontsize=11)
fig.savefig(root/'results/scan_38/plant_3d_view.png',dpi=130,facecolor=fig.get_facecolor())
