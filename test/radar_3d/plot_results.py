#!/usr/bin/env python3
"""只绘制真实采样数据；非Gazebo GUI截图，不用于替代运行验收。"""
import sys,json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
p=Path(sys.argv[1]);z=np.load(str(p/'zero.npz'))['points'][0];r=json.loads((p/'result.json').read_text())
fig=plt.figure(figsize=(12,5));ax=fig.add_subplot(121,projection='3d')
mask=np.isfinite(z['range']);x=z['x'][mask];y=z['y'][mask]-2;zz=z['z'][mask]+1
ax.scatter(x,y,zz,c=z['ring'][mask],cmap='viridis',s=7)
ax.scatter([0],[-2],[1],c='red',marker='x',s=50)
ax.set(xlabel='World X (m)',ylabel='World Y (m)',zlabel='World Z (m)',title='Measured zero-noise 3D Ray returns')
if hasattr(ax,'set_box_aspect'):
 ax.set_box_aspect((max(np.ptp(x),1),max(np.ptp(y),1),max(np.ptp(zz),1)))
else:
 radius=max(np.ptp(x),np.ptp(y),np.ptp(zz))/2
 for values,setter in [(x,ax.set_xlim),(y,ax.set_ylim),(zz,ax.set_zlim)]:
  center=(values.min()+values.max())/2;setter(center-radius,center+radius)
ax=fig.add_subplot(122);data=np.loadtxt(str(p/'selected_ray_errors.csv'),delimiter=',',skiprows=1)[:,1:].ravel()
ax.hist(data,bins=65,density=True,alpha=.65,label='Measured radial error')
t=np.linspace(-.09,.09,300);ax.plot(t,np.exp(-t*t/(2*.02**2))/(np.sqrt(2*np.pi)*.02),label='N(0, 0.02 m)')
ax.set(xlabel='Error (m)',ylabel='Density',title='9000 measured errors; mean=%.6f, sd=%.6f m'%(r['pooled_error_mean_m'],r['pooled_error_stddev_m']));ax.legend()
fig.tight_layout();fig.savefig(str(p/'measured_cloud_and_noise.png'),dpi=160);plt.close(fig)
c=np.loadtxt(str(p/'cross_ray_correlation.csv'),delimiter=',');a=np.loadtxt(str(p/'temporal_correlation_lags_1_to_10.csv'),delimiter=',')
fig,axes=plt.subplots(1,2,figsize=(10,4));np.fill_diagonal(c,np.nan)
i=axes[0].imshow(c,vmin=-.2,vmax=.2,cmap='coolwarm');axes[0].set(title='Cross-ray correlations (diagonal masked)',xlabel='Selected ray',ylabel='Selected ray');fig.colorbar(i,ax=axes[0])
i=axes[1].imshow(a,vmin=-.2,vmax=.2,cmap='coolwarm',aspect='auto',extent=(.5,10.5,14.5,-.5));axes[1].set(title='Temporal correlations',xlabel='Lag (scans)',ylabel='Selected ray');fig.colorbar(i,ax=axes[1]);fig.tight_layout();fig.savefig(str(p/'measured_correlations.png'),dpi=160)
