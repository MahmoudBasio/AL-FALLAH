"""Render a small looping GIF from the saved cloud; no synthetic geometry."""
from pathlib import Path
import numpy as np
from PIL import Image, ImageDraw, ImageFont
from matplotlib import colormaps
from view_plant_model import read_xyz

root=Path(__file__).parent
p=read_xyz(root/'results/scan_38/plant_combined_38.ply')
rng=np.random.default_rng(12)
p=p[rng.choice(len(p),min(65000,len(p)),replace=False)].astype(float)
p-=(p.min(axis=0)+p.max(axis=0))/2
height=(p[:,2]-p[:,2].min())/np.ptp(p[:,2])
colour=(height*250).astype(np.uint8)+1
palette=[16,27,39]+(colormaps['viridis'](np.linspace(.25,1,251))[:,:3]*255).astype(np.uint8).ravel().tolist()
palette += [184,203,217]*3+[238,245,250]
assert len(palette)==768
width,height_px=720,720
scale=490/np.ptp(p[:,2])
try:
    title_font=ImageFont.truetype('DejaVuSans.ttf',21)
    small_font=ImageFont.truetype('DejaVuSans.ttf',13)
except OSError:
    title_font=small_font=ImageFont.load_default()
frames=[]
for angle in np.linspace(0,2*np.pi,48,endpoint=False):
    c,s=np.cos(angle),np.sin(angle)
    x=p[:,0]*c-p[:,1]*s;y=p[:,0]*s+p[:,1]*c
    v=p[:,2]*np.cos(.2)-x*np.sin(.2)
    depth=x*np.cos(.2)+p[:,2]*np.sin(.2)
    u=np.rint(width/2+y*scale).astype(int)
    v=np.rint(385-v*scale).astype(int)
    order=np.argsort(depth)
    raster=np.zeros((height_px,width),dtype=np.uint8)
    for dx,dy in [(0,0),(1,0),(0,1)]:
        a=u[order]+dx;b=v[order]+dy
        valid=(a>=0)&(a<width)&(b>=100)&(b<660)
        raster[b[valid],a[valid]]=colour[order][valid]
    image=Image.fromarray(raster).convert('P')
    image.putpalette(palette)
    draw=ImageDraw.Draw(image)
    draw.text((24,20),'AL-FALLAH / Plant reconstruction',font=title_font,fill=255)
    draw.text((24,55),'38 captures | 283,501-point model | height colours',font=small_font,fill=252)
    draw.text((24,670),'Sampled preview of actual scan data. Click to explore the full cloud.',font=small_font,fill=252)
    frames.append(image)
frames[0].save(root/'results/scan_38/plant_rotation.gif',save_all=True,
               append_images=frames[1:],duration=120,loop=0,optimize=False,disposal=2)
print('Rendered 48 frames from actual point-cloud data.')
