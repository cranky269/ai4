#!/usr/bin/env python3
"""Isolated M5 -> M6 interface audit and separate controlled PID audit."""
import argparse
import json
import math
import time
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import rosgraph
import rospy
from mavros_msgs.msg import PositionTarget
from sensor_msgs.msg import Image
from uav_tracking.msg import VisualTrack, VisualTrackArray


class Flow:
    def __init__(self, args):
        self.a=args; self.t0=time.monotonic(); self.ros_t0=None; self.src=0; self.forwarded=0
        self.out=[]; self.out_wall=[]; self.out_ros=[]; self.frames=set(); self.masks=set(); self.rejections=[]
        self.latest_tracks=[]; self.latest_image=None; self.unknown=[]
        self.stage_tracks={}
        self.pub=rospy.Publisher('/test/visual_tracks',VisualTrackArray,queue_size=10)
        if args.mode=='real':
            rospy.Subscriber('/visual/tracks',VisualTrackArray,self.on_real,queue_size=50)
            rospy.Subscriber('/camera/image_raw',Image,self.on_image,queue_size=1,buff_size=2**22)
        rospy.Subscriber('/test/ctrl_visual',PositionTarget,self.on_output,queue_size=200)

    def elapsed(self): return time.monotonic()-self.t0
    def ros_elapsed(self):
        return 0.0 if self.ros_t0 is None else max(0.0, rospy.Time.now().to_sec()-self.ros_t0)
    def reject(self,s):
        if s not in self.rejections:self.rejections.append(s)
        rospy.logerr('DATA REJECTED: %s',s)

    def on_image(self,msg):
        try:
            raw=np.frombuffer(msg.data,dtype=np.uint8)
            ch=3 if msg.encoding.lower() in ('rgb8','bgr8') else 1
            arr=raw.reshape(msg.height,msg.step)[:,:msg.width*ch]
            arr=arr.reshape(msg.height,msg.width,ch) if ch==3 else np.repeat(arr.reshape(msg.height,msg.width,1),3,axis=2)
            if msg.encoding.lower()=='bgr8':arr=arr[:,:,::-1]
            self.latest_image=arr.copy()
        except Exception as exc: self.reject('camera decode failed: '+str(exc))

    def validate_tracks(self,msg):
        for x in msg.tracks:
            vals=[float(v) for v in x.bbox]+[float(x.area_ratio)]
            if not all(math.isfinite(v) for v in vals): self.reject('VisualTrack contains non-finite values');return False
            if len(x.bbox)!=4 or x.area_ratio<0: self.reject('VisualTrack bbox/area invalid');return False
        return True

    def on_real(self,msg):
        self.src+=1
        if not self.validate_tracks(msg): return
        self.latest_tracks=list(msg.tracks)
        if not (self.a.silence_start<=self.elapsed()<self.a.silence_end):
            self.pub.publish(msg);self.forwarded+=1

    def stage(self,t):
        if t<1:return 'WAIT'
        if t<3:return 'CENTER'
        if t<5:return 'RIGHT_DOWN_FAR'
        if t<7:return 'TERMINAL'
        if t<8:return 'SILENCE'
        return 'RECOVERY'

    @staticmethod
    def item(ident,cx,cy,ratio):
        side=math.sqrt(ratio*640*480);m=VisualTrack();m.id=ident
        m.bbox=[cx-side/2,cy-side/2,side,side];m.area_ratio=ratio;return m

    def publish_controlled(self,t):
        st=self.stage(t);msg=VisualTrackArray()
        if st in ('CENTER','RECOVERY'): msg.tracks=[self.item(1,320,240,.10)]
        elif st=='RIGHT_DOWN_FAR':
            # Larger area wins. Equal-area tie below is won by id=3 (right/down).
            msg.tracks=[self.item(8,180,160,.02),self.item(3,470,360,.05),self.item(9,150,150,.05)]
        elif st=='TERMINAL': msg.tracks=[self.item(4,320,240,.35)]
        elif st=='SILENCE': return
        else: return
        self.latest_tracks=list(msg.tracks);self.stage_tracks[st]=list(msg.tracks)
        self.pub.publish(msg);self.forwarded+=1;self.src+=1

    def on_output(self,msg):
        t=self.ros_elapsed();self.out_wall.append(time.monotonic());self.out_ros.append(rospy.Time.now().to_sec());self.frames.add(int(msg.coordinate_frame));self.masks.add(int(msg.type_mask))
        vals=(float(msg.velocity.x),float(msg.velocity.y),float(msg.velocity.z),float(msg.yaw_rate))
        if not all(math.isfinite(v) for v in vals):self.reject('PositionTarget output non-finite');return
        self.out.append((t,self.stage(t),)+vals)

    def safety(self):
        _,subs,_=rosgraph.Master(rospy.get_name()).getSystemState();nodes=dict(subs).get('/test/ctrl_visual',[])
        self.unknown=sorted(set(nodes)-{rospy.get_name()})
        if self.unknown or any('mux' in n.lower() or 'mavros' in n.lower() for n in nodes):
            self.reject('unexpected /test/ctrl_visual subscribers: %r'%nodes);return False
        return True

    def run(self):
        clock_deadline=time.monotonic()+5.0
        while (not rospy.is_shutdown() and rospy.Time.now().to_sec() <= 0.0
               and time.monotonic()<clock_deadline):
            time.sleep(.01)
        if rospy.Time.now().to_sec() <= 0.0:
            self.reject('ROS simulation clock did not become non-zero')
            return self.finish()
        self.ros_t0=rospy.Time.now().to_sec()
        self.out=[];self.out_wall=[];self.out_ros=[];self.frames=set();self.masks=set()
        rate=rospy.Rate(20);checked=False
        wall_deadline=time.monotonic()+max(20.0,self.a.duration*3.0)
        while (not rospy.is_shutdown() and self.ros_elapsed()<self.a.duration
               and time.monotonic()<wall_deadline):
            t=self.ros_elapsed()
            if self.a.mode=='controlled':self.publish_controlled(t)
            if t>.8 and not checked:checked=True
            if checked and not self.safety():break
            rate.sleep()
        return self.finish()

    @staticmethod
    def hz(x):return (len(x)-1)/(x[-1]-x[0]) if len(x)>1 and x[-1]>x[0] else 0.
    def finish(self):
        self.a.output.mkdir(parents=True,exist_ok=True);hz=self.hz(self.out_ros);wall_hz=self.hz(self.out_wall)
        by={s:[r for r in self.out if r[1]==s] for s in ['CENTER','RIGHT_DOWN_FAR','TERMINAL','SILENCE','RECOVERY']}
        def allp(stage,fn):return bool(by[stage]) and all(fn(r) for r in by[stage])
        checks={
            'center_zero':allp('CENTER',lambda r:abs(r[2])<1e-6 and abs(r[4])<1e-6 and abs(r[5])<1e-6),
            'right_yaw_negative':any(r[5]<0 for r in by['RIGHT_DOWN_FAR']),
            'down_vz_negative':any(r[4]<0 for r in by['RIGHT_DOWN_FAR']),
            'far_vx_positive':any(r[2]>0 for r in by['RIGHT_DOWN_FAR']),
            'terminal_vx_zero':bool([r for r in by['TERMINAL'] if r[0]>=5.25]) and all(
                abs(r[2])<1e-6 for r in by['TERMINAL'] if r[0]>=5.25),
            'vy_zero':all(abs(r[3])<1e-12 for r in self.out),
            'silence_zero_after_0_2':all(abs(v)<1e-6 for r in self.out if 7.25<=r[0]<8 for v in r[2:]),
            'recovery_output':any(abs(r[2])+abs(r[4])+abs(r[5])<1e-6 for r in by['RECOVERY'])}
        summary=dict(develop_sha='f540ae4e82ad614c8fb8560d7dad50675d66ad76',mode=self.a.mode,
            source_messages=self.src,forwarded_messages=self.forwarded,output_messages=len(self.out),output_rate_hz=hz,output_wall_rate_hz=wall_hz,
            coordinate_frames=sorted(self.frames),type_masks=sorted(self.masks),checks=checks,
            unknown_output_subscribers=self.unknown,data_rejected=self.rejections,
            coordinate_note='ROS body FLU only; node performs no FLU->FRD or ENU->NED conversion')
        base=not self.rejections and self.src and self.out and 45<=hz<=55 and self.frames=={9} and self.masks=={1479} and checks['vy_zero']
        summary['pass']=bool(base and (all(checks.values()) if self.a.mode=='controlled' else True))
        (self.a.output/'summary.json').write_text(json.dumps(summary,indent=2,ensure_ascii=False)+'\n',encoding='utf-8')
        self.plot(self.a.output/(self.a.mode+'_panels.png'),summary)
        print(json.dumps(summary,ensure_ascii=False));return 0 if summary['pass'] else 2

    def plot(self,path,summary):
        fig=plt.figure(figsize=(16,10));grid=fig.add_gridspec(2,3)
        image=self.latest_image if self.latest_image is not None else np.full((480,640,3),235,dtype=np.uint8)
        stages=['CENTER','RIGHT_DOWN_FAR','TERMINAL']
        for index,stage in enumerate(stages):
            ax=fig.add_subplot(grid[0,index]);ax.imshow(image);ax.scatter([320],[240],marker='+',s=100,c='blue',label='image center')
            for tr in self.stage_tracks.get(stage,[]):
                x,y,w,h=tr.bbox;ax.add_patch(plt.Rectangle((x,y),w,h,fill=False,lw=2,label='id=%d area=%.3f'%(tr.id,tr.area_ratio)))
                ax.scatter([x+w/2],[y+h/2],c='red')
            rows=[r for r in self.out if r[1]==stage]
            if rows:
                vx=float(np.mean([r[2] for r in rows]));vz=float(np.mean([r[4] for r in rows]));yaw=float(np.mean([r[5] for r in rows]))
                ax.text(10,465,'vx=%+.3f  vz=%+.3f  yaw=%+.3f'%(vx,vz,yaw),fontsize=9,bbox=dict(facecolor='white',alpha=.8))
                arrow_x=0 if abs(yaw)<1e-6 else -60*np.sign(yaw)
                arrow_y=0 if abs(vz)<1e-6 else -60*np.sign(vz)
                if arrow_x or arrow_y:
                    ax.arrow(320,240,arrow_x,arrow_y,width=3,color='purple',length_includes_head=True)
            ax.set(xlim=(0,640),ylim=(480,0),title=stage);ax.legend(fontsize=7)
        ax1=fig.add_subplot(grid[1,0:2]);ax2=fig.add_subplot(grid[1,2])
        if self.out:
            a=np.asarray([[r[0],r[2],r[3],r[4],r[5]] for r in self.out]);
            ax1.plot(a[:,0],a[:,1],label='velocity.x');ax1.plot(a[:,0],a[:,3],label='velocity.z');ax1.plot(a[:,0],a[:,4],label='yaw_rate')
            ax2.plot(a[:,0],a[:,1],label='forward');ax2.axvline(7,c='red',ls='--',label='silence');ax2.axvline(7.2,c='purple',ls='--',label='0.2s timeout');ax2.axvline(8,c='green',ls='--',label='recovery')
        for ax in (ax1,ax2):ax.grid(True);ax.legend(fontsize=8)
        ax1.set_title('PID outputs over time');ax2.set_title('timeout and zero result');ax2.set_xlabel('ROS time (s)')
        fig.suptitle('%s | frame=%s mask=%s | ROS-time %.2f Hz | %s'%(self.a.mode,summary['coordinate_frames'],summary['type_masks'],summary['output_rate_hz'],'PASS' if summary['pass'] else 'FAIL'))
        fig.tight_layout();fig.savefig(str(path),dpi=150);plt.close(fig)


def main():
    p=argparse.ArgumentParser();p.add_argument('--mode',choices=['real','controlled'],required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--duration',type=float,default=10);p.add_argument('--silence-start',type=float,default=7);p.add_argument('--silence-end',type=float,default=8);a=p.parse_args()
    rospy.init_node('m5_m6_receiver_audit_'+a.mode,anonymous=True);raise SystemExit(Flow(a).run())
if __name__=='__main__':main()
