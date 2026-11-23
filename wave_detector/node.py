"""ROS adapter for tracked COCO poses or a local YOLO model on Image input."""
from dataclasses import fields
import json
import math
from .core import WaveConfig,WaveDetector,position_from_depth,sample_person_depth


def main(args=None):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from std_msgs.msg import String,Bool
    from sensor_msgs.msg import Image,CameraInfo
    from geometry_msgs.msg import PointStamped

    class WaveNode(Node):
        def __init__(self):
            super().__init__('wave_detector')
            defaults=WaveConfig()
            config={f.name:self.declare_parameter(f.name,getattr(defaults,f.name)).value for f in fields(defaults)}
            self.detector=WaveDetector(WaveConfig(**config));self.moving=False
            self.frame=self.declare_parameter('frame_id','camera_color_optical_frame').value
            self.age=float(self.declare_parameter('max_age_sec',.5).value)
            if not math.isfinite(self.age) or self.age<=0:raise ValueError('max_age_sec must be positive')
            self.events=self.create_publisher(String,'~/events',10)
            self.points=self.create_publisher(PointStamped,'~/person_point',10)
            self.create_subscription(Bool,'~/camera_moving',self.motion,10)
            self.create_subscription(String,'~/poses',self.poses,10)
            model=self.declare_parameter('model_path','').value
            topic=self.declare_parameter('image_topic','/camera/color/image_raw').value
            device=self.declare_parameter('device','cpu').value
            self.adapter=None;self.depth_message=None;self.camera_info=None
            depth_topic=self.declare_parameter('depth_topic','/camera/aligned_depth_to_color/image_raw').value
            info_topic=self.declare_parameter('camera_info_topic','/camera/color/camera_info').value
            if model:
                from cv_bridge import CvBridge
                from .yolo import YoloPoseAdapter
                self.bridge=CvBridge();self.adapter=YoloPoseAdapter(model,device)
                self.create_subscription(Image,topic,self.image,qos_profile_sensor_data)
                self.create_subscription(Image,depth_topic,self.depth,qos_profile_sensor_data)
                self.create_subscription(CameraInfo,info_topic,self.calibration,qos_profile_sensor_data)

        def depth(self,msg):self.depth_message=msg

        def calibration(self,msg):self.camera_info=msg

        def motion(self,msg):
            self.moving=msg.data
            if self.moving:self.detector.reset()

        def process(self,payload):
            stamp=float(payload['stamp']);now=self.get_clock().now().nanoseconds*1e-9
            if not math.isfinite(stamp) or stamp<=0 or not -.1<=now-stamp<=self.age:
                self.detector.reset();return
            frame=payload.get('frame_id',self.frame)
            for person in payload['people']:
                event=self.detector.update(person['track_id'],stamp,person['keypoints'],person['bbox'],self.moving)
                if event is None:continue
                event['frame_id']=frame
                self.events.publish(String(data=json.dumps(event,allow_nan=False)))
                if 'depth_m' in person and 'intrinsics' in payload:
                    k=payload['intrinsics'];x1,y1,x2,y2=person['bbox']
                    position=position_from_depth((x1+x2)/2,(y1+y2)/2,float(person['depth_m']),**k)
                    p=PointStamped();p.header.frame_id=frame
                    p.header.stamp.sec=int(stamp);p.header.stamp.nanosec=int((stamp-int(stamp))*1e9)
                    p.point.x,p.point.y,p.point.z=position;self.points.publish(p)

        def poses(self,msg):
            try:self.process(json.loads(msg.data))
            except (ValueError,TypeError,KeyError,AttributeError) as e:self.get_logger().warning(str(e),throttle_duration_sec=5.)

        def image(self,msg):
            try:
                stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
                if self.moving:self.detector.reset();return
                frame=self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
                people=self.adapter.predict(frame)
                payload={'stamp':stamp,'frame_id':msg.header.frame_id,'people':people}
                depth=self.depth_message;info=self.camera_info
                if depth is not None and info is not None:
                    depth_stamp=depth.header.stamp.sec+depth.header.stamp.nanosec*1e-9
                    if (abs(depth_stamp-stamp)<=.1 and depth.header.frame_id==msg.header.frame_id==info.header.frame_id
                            and (depth.width,depth.height)==(msg.width,msg.height)==(info.width,info.height)
                            and depth.encoding in ['16UC1','32FC1']):
                        image=self.bridge.imgmsg_to_cv2(depth,desired_encoding='passthrough');k=info.k
                        payload['intrinsics']={'fx':k[0],'fy':k[4],'cx':k[2],'cy':k[5]}
                        for person in people:
                            value=sample_person_depth(image,person['bbox'],'mm' if depth.encoding=='16UC1' else 'm')
                            if value is not None:person['depth_m']=value
                self.process(payload)
            except Exception as e:self.get_logger().warning(str(e),throttle_duration_sec=5.)

    rclpy.init(args=args);node=None
    try:node=WaveNode();rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        if node:node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
