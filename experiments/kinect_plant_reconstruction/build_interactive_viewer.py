"""Embed the actual model in a standalone, dependency-free WebGL viewer."""
import base64
import json
from pathlib import Path
import numpy as np
from view_plant_model import read_xyz

root=Path(__file__).parent
points=read_xyz(root/'results/scan_38/plant_combined_38.ply')
low=points.min(axis=0);high=points.max(axis=0)
q=np.rint((points-low)/(high-low)*65535).astype('<u2')
reconstructed=q.astype(np.float64)/65535*(high-low)+low
assert np.max(abs(reconstructed-points))<.00002
model=dict(count=len(points),min=low.tolist(),max=high.tolist(),
           data=base64.b64encode(q.tobytes()).decode('ascii'))
template=(root/'point_cloud_viewer.template.html').read_text()
(root/'results/scan_38/interactive_plant.html').write_text(template.replace('__MODEL_JSON__',json.dumps(model,separators=(',',':'))))
print(f'Embedded all {len(points):,} points; quantization error <0.02 mm.')
