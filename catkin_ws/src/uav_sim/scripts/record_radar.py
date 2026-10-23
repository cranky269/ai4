#!/usr/bin/env python3
"""Loss-aware single-writer ROS1 bag + every-frame organized NPZ recorder."""
import csv
import datetime
import json
from pathlib import Path
import queue
import threading
import time
import traceback
import numpy as np
import rosbag
import rospy
from sensor_msgs.msg import PointCloud2, Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from tf2_msgs.msg import TFMessage
from rosgraph_msgs.msg import Clock
from uav_tracking.msg import CentroidArray
from PIL import Image as PILImage


def utc():
    return datetime.datetime.now(datetime.timezone.utc).isoformat()


def save_cloud(path, msg):
    if msg.is_bigendian:
        endian = '>'
    else:
        endian = '<'
    fields = {f.name: f for f in msg.fields}
    arrays = {}
    for name in ('x', 'y', 'z', 'range', 'ring', 'column', 'status'):
        f = fields[name]
        dtype = {7: 'f4', 4: 'u2', 2: 'u1'}[f.datatype]
        arrays[name] = np.ndarray((msg.height, msg.width), dtype=endian+dtype,
                                 buffer=msg.data, offset=f.offset,
                                 strides=(msg.row_step, msg.point_step)).copy()
    valid = np.isfinite(arrays['x']) & np.isfinite(arrays['y']) & np.isfinite(arrays['z'])
    np.savez_compressed(str(path), **arrays, valid=valid,
                        stamp_sec=msg.header.stamp.secs, stamp_nsec=msg.header.stamp.nsecs,
                        frame_id=msg.header.frame_id)
    return int(valid.sum())


class Recorder:
    def __init__(self):
        self.root = Path(rospy.get_param('~run_dir'))
        (self.root/'frames').mkdir(exist_ok=True)
        self.camera_root = self.root/'interfaces/node__detector/images'
        self.camera_root.mkdir(parents=True,exist_ok=True)
        radar_interface = self.root/'interfaces/node__cluster'
        radar_interface.mkdir(parents=True,exist_ok=True)
        if not (radar_interface/'points').exists():
            (radar_interface/'points').symlink_to('../../frames',target_is_directory=True)
        (self.root/'interfaces/mapping.json').write_text(json.dumps({
            '/detector': {'directory':'node__detector','topic':'/camera/image_raw','message_type':'sensor_msgs/Image','status':'已本地暂存，未完成节点接收','reason':'上游已有detector代码与launch；本入口未启动detector，未验证模型权重与实际接收'},
            '/cluster': {'directory':'node__cluster','topic':'/radar/points','message_type':'sensor_msgs/PointCloud2','receipt':'见cluster_received.csv和接口验收报告'}},ensure_ascii=False,indent=2))
        self.camera_file=(self.root/'camera_frames.csv').open('w',newline='')
        self.camera_index=csv.writer(self.camera_file)
        self.camera_index.writerow(['frame','source_seq','stamp_sec','stamp_nsec','frame_id','encoding','width','height','file'])
        self.camera_count=0
        self.q = queue.Queue(maxsize=4096)
        self.lock = threading.Lock()
        self.accepting = True
        self.stats = dict(start_utc=utc(), radar_frames=0, valid_points=0,
                          queue_drops=0, sequence_gaps=0, timestamp_gaps=0,
                          write_errors=0, topic_messages={}, errors=[])
        self.started_monotonic = time.monotonic()
        self.first = self.last = self.prev_seq = None
        import yaml
        self.config = yaml.safe_load((self.root/'radar.yaml').read_text())
        self.bag = rosbag.Bag(str(self.root/'raw.bag'), 'w', compression='lz4')
        self.index_file = (self.root/'frames.csv').open('w', newline='')
        self.index = csv.writer(self.index_file)
        self.index.writerow(['frame','seq','stamp_sec','stamp_nsec','frame_id','valid_points','file'])
        self.worker = threading.Thread(target=self.write_loop)
        self.worker.start()
        self.pressure_stop=threading.Event()
        self.pressure=threading.Thread(target=self.backpressure)
        self.pressure.start()
        self.subs = []
        topics = [('/radar/points',PointCloud2),('/tf',TFMessage),('/tf_static',TFMessage),
                  ('/camera_motion_demo/actual_pose',PoseStamped),
                  ('/target/ground_truth/pose',PoseStamped),('/clock',Clock),
                  ('/uav/front_camera/image_raw',Image),('/uav/front_camera/camera_info',CameraInfo),
                  ('/camera/image_raw',Image),('/camera/camera_info',CameraInfo),
                  ('/target/pose',PoseStamped),('/radar/centroid',CentroidArray)]
        self.stats['topics'] = {t: cls._type for t,cls in topics}
        self.stats['config'] = self.config
        self.stats['mount'] = dict(parent='uav_base_link', child='radar_link', xyz_m=[.30,0,.12], quaternion_xyzw=[0,0,0,1])
        for topic, cls in topics:
            self.subs.append(rospy.Subscriber(topic, cls, self.receive, callback_args=topic,
                                             queue_size=200, buff_size=64*1024*1024))
        rospy.set_param('~ready',True)

    def backpressure(self):
        from std_srvs.srv import Empty
        events=[]
        while not self.pressure_stop.wait(.05):
            if self.q.qsize()<256:continue
            try:
                rospy.ServiceProxy('/gazebo/pause_physics',Empty)()
                events.append(dict(event='pause',utc=utc(),queue=self.q.qsize()))
                rospy.logwarn('Disk queue high; pausing simulation to preserve every sensor frame')
                while self.q.qsize()>32 and not self.pressure_stop.wait(.05):pass
                if not self.pressure_stop.is_set() and not (self.root/'stopping').exists():
                    rospy.ServiceProxy('/gazebo/unpause_physics',Empty)()
                    events.append(dict(event='resume',utc=utc(),queue=self.q.qsize()))
                (self.root/'backpressure.json').write_text(json.dumps(events,indent=2))
            except Exception as exc:
                rospy.logerr('Simulation backpressure failed: %s',exc)
    def receive(self, msg, topic):
        failed = False
        with self.lock:
            if not self.accepting:
                return
            try:
                self.q.put_nowait((topic, msg, rospy.Time.now()))
            except queue.Full:
                self.stats['queue_drops'] += 1
                failed = True
        if failed:
            rospy.logfatal('Radar recorder queue overflow; data loss explicitly recorded')
            rospy.signal_shutdown('recorder queue overflow')

    def write_loop(self):
        while True:
            item = self.q.get()
            if item is None:
                self.q.task_done()
                return
            try:
                topic,msg,received = item
                # Preserve all original message fields; bag time is measurement time if present.
                stamp = msg.header.stamp if hasattr(msg,'header') else received
                headers = dict(getattr(msg, '_connection_header', {}) or {})
                headers.update(topic=topic, type=msg._type, md5sum=msg._md5sum,
                               message_definition=msg._full_text)
                if topic == '/tf_static': headers['latching'] = '1'
                self.bag.write(topic,msg,stamp,connection_header=headers)
                counts = self.stats['topic_messages']
                counts[topic] = counts.get(topic,0)+1
                if topic == '/uav/front_camera/image_raw':
                    i=self.camera_count
                    name='interfaces/node__detector/images/%08d.png'%i
                    if msg.encoding not in ('rgb8','bgr8','mono8'):
                        raise ValueError('Unsupported image encoding '+msg.encoding)
                    channels=1 if msg.encoding=='mono8' else 3
                    a=np.frombuffer(msg.data,np.uint8).reshape(msg.height,msg.step)[:,:msg.width*channels]
                    a=a.reshape((msg.height,msg.width) if channels==1 else (msg.height,msg.width,channels))
                    if msg.encoding=='bgr8': a=a[:,:,::-1]
                    PILImage.fromarray(a).save(str(self.root/name),compress_level=1)
                    self.camera_index.writerow([i,msg.header.seq,msg.header.stamp.secs,msg.header.stamp.nsecs,msg.header.frame_id,msg.encoding,msg.width,msg.height,name])
                    self.camera_file.flush();self.camera_count+=1
                if topic == '/radar/points':
                    i = self.stats['radar_frames']
                    name = 'frames/%08d.npz' % i
                    valid = save_cloud(self.root/name,msg)
                    self.index.writerow([i,msg.header.seq,stamp.secs,stamp.nsecs,msg.header.frame_id,valid,name])
                    self.index_file.flush()
                    t = stamp.to_sec()
                    if self.last is not None:
                        if t <= self.last or t-self.last > 1.5/self.config['update_rate']:
                            self.stats['timestamp_gaps'] += 1
                            rospy.logerr('Radar timestamp discontinuity: %s -> %s',self.last,t)
                        delta = (msg.header.seq-self.prev_seq) % (2**32)
                        if delta != 1:
                            self.stats['sequence_gaps'] += max(1,delta-1)
                            rospy.logerr('Radar sequence gap: %s -> %s',self.prev_seq,msg.header.seq)
                    if self.first is None: self.first = t
                    self.last,self.prev_seq = t,msg.header.seq
                    self.stats['radar_frames'] += 1
                    self.stats['valid_points'] += valid
            except Exception:
                self.stats['write_errors'] += 1
                self.stats['errors'].append(traceback.format_exc())
                rospy.logfatal(self.stats['errors'][-1])
                rospy.signal_shutdown('radar write failed')
            finally:
                self.q.task_done()

    def close(self):
        self.pressure_stop.set()
        self.pressure.join(timeout=10)
        with self.lock:
            self.accepting = False
        for sub in self.subs: sub.unregister()
        self.q.put(None)
        self.worker.join()
        try:
            self.bag.close()
            self.index_file.close()
            self.camera_file.close()
        except Exception:
            self.stats['write_errors'] += 1
            self.stats['errors'].append(traceback.format_exc())
        wall = time.monotonic()-self.started_monotonic
        self.stats['camera_png_frames']=self.camera_count
        self.stats.update(end_utc=utc(),first_sim_time=self.first,last_sim_time=self.last,
                          wall_duration_s=wall, measured_wall_hz=self.stats['radar_frames']/wall)
        span = (self.last-self.first) if self.first is not None else 0
        self.stats['measured_sim_hz'] = (self.stats['radar_frames']-1)/span if span > 0 else None
        self.stats['status'] = 'failed' if (not self.stats['radar_frames'] or any(self.stats[k] for k in
            ('queue_drops','sequence_gaps','timestamp_gaps','write_errors'))) else 'closed'
        self.stats['loss_scope'] = '序号/时间间隔及本地队列检查；不证明Gazebo内部从未漏生成扫描。'
        (self.root/'statistics.json').write_text(json.dumps(self.stats,ensure_ascii=False,indent=2))
        rospy.loginfo('Radar recording closed: %s', self.stats)


if __name__ == '__main__':
    rospy.init_node('radar_recorder')
    rec = Recorder()
    try:
        rospy.spin()
    finally:
        rec.close()
    if rec.stats['status'] != 'closed':
        raise SystemExit(1)
