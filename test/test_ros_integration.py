"""Optional isolated DDS tests with synthetic publishers; no hardware access."""
import os
import signal
import subprocess
import sys
import time
import json
import pytest
rclpy=pytest.importorskip('rclpy')
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

class Harness:
    def __init__(self,package,domain,parameters):
        self.context=Context();rclpy.init(context=self.context,domain_id=domain)
        self.node=Node(package+'_test',context=self.context)
        self.executor=SingleThreadedExecutor(context=self.context);self.executor.add_node(self.node)
        self.env=dict(os.environ,ROS_DOMAIN_ID=str(domain))
        args=[sys.executable,'-c',f'from {package}.node import main; main()','--ros-args']
        for key,value in parameters.items():args+=['-p',f'{key}:={value}']
        self.process=subprocess.Popen(args,env=self.env,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,start_new_session=True)

    def wait(self,predicate,publish=None,timeout=8.):
        end=time.monotonic()+timeout
        while not predicate() and time.monotonic()<end:
            assert self.process.poll() is None,self.process.stdout.read().decode()
            if publish:publish()
            self.executor.spin_once(timeout_sec=.025)
        assert predicate()

    def call(self,name):
        from std_srvs.srv import Trigger
        client=self.node.create_client(Trigger,name);self.wait(client.service_is_ready)
        future=client.call_async(Trigger.Request());self.wait(future.done)
        return future.result()

    def close(self):
        try:os.killpg(self.process.pid,signal.SIGINT)
        except ProcessLookupError:pass
        try:self.process.wait(timeout=3)
        except subprocess.TimeoutExpired:
            os.killpg(self.process.pid,signal.SIGKILL);self.process.wait(timeout=2)
        output=self.process.stdout.read().decode();self.process.stdout.close()
        self.executor.shutdown();self.node.destroy_node();rclpy.shutdown(context=self.context)
        assert self.process.returncode==0,output

def test_real_pose_input_detects_wave_and_publishes_position():
    from std_msgs.msg import String
    from geometry_msgs.msg import PointStamped
    from wave_detector.cli import demo_frames
    h=Harness('wave_detector',84,{})
    events=[];positions=[]
    try:
        pub=h.node.create_publisher(String,'/wave_detector/poses',10)
        h.node.create_subscription(String,'/wave_detector/events',lambda m:events.append(json.loads(m.data)),10)
        h.node.create_subscription(PointStamped,'/wave_detector/person_point',lambda m:positions.append(m),10)
        h.wait(lambda:pub.get_subscription_count()>0)
        for frame in demo_frames():
            frame['stamp']=h.node.get_clock().now().nanoseconds*1e-9
            frame['frame_id']='optical';frame['intrinsics']={'fx':100.,'fy':100.,'cx':50.,'cy':100.}
            frame['people'][0]['depth_m']=2.
            pub.publish(String(data=json.dumps(frame)));h.executor.spin_once(timeout_sec=.05)
            time.sleep(.03)
        h.wait(lambda:bool(events) and bool(positions))
        assert events[-1]['track_id']==1
        assert positions[-1].header.frame_id=='optical' and positions[-1].point.z==2.
    finally:h.close()
