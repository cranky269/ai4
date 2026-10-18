#!/usr/bin/env python3
"""独立离线复算实际采集数据，不生成测量值。阈值固定在看数据前。"""
import json,sys
from pathlib import Path
import numpy as np
OUT=Path(sys.argv[1]);result={};checks={}
def check(name,condition,detail=None):
 checks[name]={'pass':bool(condition),'detail':detail}
def load(name):return np.load(str(OUT/(name+'.npz')))
zf=load('zero');nf=load('noisy');bf=load('boundary')
z,n,b=zf['points'],nf['points'],bf['points']
vertical=np.linspace(-np.pi/18,np.pi/18,5);horizontal=np.linspace(-np.pi/4,np.pi/4,181)
columns=[80,90,100]
truth=5/(np.cos(vertical[:,None])*np.cos(horizontal[columns])[None,:])
zero_ranges=z['range'][:,:,columns];noisy_ranges=n['range'][:,:,columns]
errors=(noisy_ranges-truth).reshape(len(n),-1)
zero_error=(zero_ranges-truth)
result['selected_rays']=[{'ring':int(v),'column':int(h),'true_range_m':float(truth[v,j])} for v in range(5) for j,h in enumerate(columns)]
check('zero_geometric_distance',np.max(np.abs(zero_error))<5e-5,float(np.max(np.abs(zero_error))))
check('zero_static',np.max(np.ptp(zero_ranges,axis=0))<1e-6,float(np.max(np.ptp(zero_ranges,axis=0))))
xyz=np.stack([z['x'][0,:,90],z['y'][0,:,90],z['z'][0,:,90]],axis=1)
world_xyz=xyz+np.array([0,-2,1])
expected_height=1+5*np.tan(vertical)
check('five_actual_target_heights',np.allclose(world_xyz[:,0],5,atol=1e-5) and np.allclose(world_xyz[:,2],expected_height,atol=1e-5),world_xyz.tolist())
check('noise_mean',abs(errors.mean())<.002,float(errors.mean()))
check('noise_stddev',.018<errors.std(ddof=1)<.022,float(errors.std(ddof=1)))
means=errors.mean(axis=0);stds=errors.std(axis=0,ddof=1)
check('per_ray_mean',np.max(np.abs(means))<.004,means.tolist())
check('per_ray_stddev',np.all((stds>.016)&(stds<.024)),stds.tolist())
corr=np.corrcoef(errors,rowvar=False);cross=corr[np.triu_indices(errors.shape[1],1)]
autocorr=np.array([[np.corrcoef(errors[:-lag,j],errors[lag:,j])[0,1] for lag in range(1,11)] for j in range(errors.shape[1])])
check('cross_ray_correlation',np.max(np.abs(cross))<.18,float(np.max(np.abs(cross))))
check('temporal_lags_1_to_10',np.max(np.abs(autocorr))<.18,float(np.max(np.abs(autocorr))))
check('no_repeated_frame_vector',len(np.unique(errors,axis=0))==len(errors),int(len(np.unique(errors,axis=0))))
check('each_ray_varies',np.all(np.ptp(noisy_ranges,axis=0)>.06),np.ptp(noisy_ranges,axis=0).tolist())
frame_mean_variance=float(np.var(errors.mean(axis=1),ddof=1));result['frame_mean_variance_m2']=frame_mean_variance
result['iid_expected_frame_mean_variance_m2']=.02**2/15
check('no_frame_shared_offset',.5*(.02**2/15)<frame_mean_variance<1.5*(.02**2/15))
# Interior sample rays have >4.8m margin from range boundaries.
check('selected_far_from_clipping',np.all(n['status'][:,:,columns]==1))
noecho=~np.isfinite(z['range'][0]);check('has_no_echo',np.any(noecho),int(noecho.sum()))
check('no_false_noise_echo',np.all(~np.isfinite(n['range'][:,noecho])) and np.all(n['status'][:,noecho]==0))
for name,data,file in [('zero',z,zf),('noisy',n,nf),('boundary',b,bf)]:
 stamp=file['stamps'];wall=file['wall'];finite=np.isfinite(data['range'])
 points_xyz=np.stack([data['x'],data['y'],data['z']],axis=-1)
 rnorm=np.linalg.norm(points_xyz,axis=-1)
 check(name+'_timestamp_monotonic',np.all(np.diff(stamp)>0))
 check(name+'_xyz_norm',np.max(np.abs(rnorm[finite]-data['range'][finite]))<3e-6)
 check(name+'_invalid_xyz_nan',np.all(np.isnan(points_xyz[~finite])))
 check(name+'_organized_indices',np.all(data['ring']==np.arange(5)[None,:,None]) and np.all(data['column']==np.arange(181)[None,None,:]))
 result[name]={'frames':len(data),'sim_duration_s':float(stamp[-1]-stamp[0]),'sim_hz':float((len(data)-1)/(stamp[-1]-stamp[0])),'wall_hz':float((len(data)-1)/(wall[-1]-wall[0])),'min_valid_range_m':float(data['range'][finite].min()),'max_valid_range_m':float(data['range'][finite].max()),'statuses':{str(i):int((data['status']==i).sum()) for i in range(5)}}
 check(name+'_sustained_output',result[name]['sim_hz']>4.5)
finite=np.isfinite(b['range'])
check('boundary_valid_range',np.all(b['range'][finite]>=4.99-1e-6) and np.all(b['range'][finite]<=5.01+1e-6))
center_status=b['status'][:,2,90]
check('boundary_native_min_clamp',np.any(center_status==3),int((center_status==3).sum()))
check('boundary_native_max_clamp',np.any(center_status==4),int((center_status==4).sum()))
check('boundary_no_echo_preserved',np.any(~finite) and np.all(np.isnan(b['x'][~finite])))
result.update({'sigma_m':.02,'pooled_error_mean_m':float(errors.mean()),'pooled_error_stddev_m':float(errors.std(ddof=1)),'total_error_samples':int(errors.size),'max_abs_cross_ray_corr':float(np.max(np.abs(cross))),'max_abs_time_corr':float(np.max(np.abs(autocorr))),'checks':checks,'pass':all(c['pass'] for c in checks.values())})
np.savetxt(str(OUT/'selected_ray_errors.csv'),np.column_stack([nf['stamps'],errors]),delimiter=',',header='sim_stamp,'+','.join('ring%d_col%d'%(v,h) for v in range(5) for h in columns),comments='')
np.savetxt(str(OUT/'cross_ray_correlation.csv'),corr,delimiter=',')
np.savetxt(str(OUT/'temporal_correlation_lags_1_to_10.csv'),autocorr,delimiter=',')
(OUT/'result.json').write_text(json.dumps(result,indent=2,ensure_ascii=False,allow_nan=False))
print(json.dumps({k:v for k,v in result.items() if k not in ['checks','selected_rays']},ensure_ascii=False,indent=2))
for key,value in checks.items():
 if not value['pass']:print('FAIL',key,value)
sys.exit(0 if result['pass'] else 1)
