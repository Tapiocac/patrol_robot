import rclpy
from geometry_msgs.msg import PoseStamped, Pose
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy.time
from tf_transformations import euler_from_quaternion, quaternion_from_euler
from rclpy.duration import Duration 
from tf2_ros import TransformListener, Buffer

from autopatrol_interfaces.srv import SpeachText

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class PatrolNode(BasicNavigator):
    def __init__(self, node_name='patrol_node', namespace=''):
        super().__init__(node_name, namespace)
        # 导航相关定义
        self.declare_parameter('initial_point', [0.0, 0.0, 0.0]) # x, y, yaw
        self.declare_parameter('target_points', [0.0, 0.0, 0.0, 1.0, 1.0, 1.57]) # 每三位表示一个点
        self.initial_point_ = self.get_parameter('initial_point').value
        self.target_points_ = self.get_parameter('target_points').value

        # 实时获取TF相关定义
        self.buffer_ = Buffer()
        self.listener_ = TransformListener(self.buffer_, self)

        # 语音功能客户端
        self.speach_client_ = self.create_client(SpeachText, 'speech_text')

        # 订阅与保存图像功能相关定义
        self.declare_parameter('image_save_path', '')
        self.image_save_path = self.get_parameter('image_save_path').value
        self.bridge = CvBridge()
        self.latest_image = None
        self.subscription_image = self.create_subscription(
            Image, '/camera_sensor/image_raw', self.image_callback, 10)

    def get_pose_by_xyyaw(self, x, y, yaw):
        """
        通过x, y, yaw合成PoseStamped(位姿信息)

        geometry_msgs/msg/PoseStamped:

        std_msgs/Header header
            builtin_interfaces/Time stamp
                int32 sec
                uint32 nanosec
            string frame_id
        Pose pose
            Point position
                float64 x
                float64 y
                float64 z
            Quaternion orientation
                float64 x 0
                float64 y 0
                float64 z 0
                float64 w 1
        """
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.pose.position.x = x
        pose.pose.position.y = y
        rotation_quat = quaternion_from_euler(0,0,yaw)
        pose.pose.orientation.x = rotation_quat[0]
        pose.pose.orientation.y = rotation_quat[1]
        pose.pose.orientation.z = rotation_quat[2]
        pose.pose.orientation.w = rotation_quat[3]
        return pose

    def init_robot_pose(self):
        """
        初始化机器人位姿
        """
        # 从参数获得初始化点
        self.initial_point_ = self.get_parameter('initial_point').value
        # 合成位姿并进行初始化
        self.setInitialPose(self.get_pose_by_xyyaw(self.initial_point_[0],
                                                   self.initial_point_[1],
                                                   self.initial_point_[2]))
        # 等待直到导航激活
        self.waitUntilNav2Active()

    def get_target_points(self):
        """
        通过参数值获得目标点集合
        """
        points = []
        # 从参数获得目标点
        self.target_points_ = self.get_parameter('target_points').value

        # 将目标点逐个读取放入列表中
        for index in range(int(len(self.target_points_)/3)):
            x = self.target_points_[index*3]
            y = self.target_points_[index*3+1]
            yaw = self.target_points_[index*3+2]
            points.append([x,y,yaw])
            self.get_logger().info(f'获得目标点：{index}-->({x},{y},{yaw})')
        return points


    def nav_to_pose(self, target_pose):
        """
        导航到指定位姿

        nav2_msgs/action/NavigateToPose:

        geometry_msgs/PoseStamped pose
            std_msgs/Header header
                builtin_interfaces/Time stamp
                        int32 sec
                        uint32 nanosec
                string frame_id
            Pose pose
                Point position
                        float64 x
                        float64 y
                        float64 z
                Quaternion orientation
                        float64 x 0
                        float64 y 0
                        float64 z 0
                        float64 w 1
        string behavior_tree
        ---
        #result definition
        std_msgs/Empty result
        ---
        #feedback definition
        geometry_msgs/PoseStamped current_pose
            std_msgs/Header header
                builtin_interfaces/Time stamp
                        int32 sec
                        uint32 nanosec
                string frame_id
          Pose pose
                Point position
                        float64 x
                        float64 y
                        float64 z
                Quaternion orientation
                        float64 x 0
                        float64 y 0
                        float64 z 0
                        float64 w 1
        builtin_interfaces/Duration navigation_time
            int32 sec
            uint32 nanosec
        builtin_interfaces/Duration estimated_time_remaining
            int32 sec
            uint32 nanosec
        int16 number_of_recoveries
        float32 distance_remaining
        """

        self.waitUntilNav2Active()
        result = self.goToPose(target_pose) # 返回True/False
        while not self.isTaskComplete():
            feedback = self.getFeedback()
            if feedback:
                self.get_logger().info(f'预计：{Duration.from_msg(feedback.estimated_time_remaining).nanoseconds/1e9}s后到达')
        
        # 结果判断
        result = self.getResult()
        if result == TaskResult.SUCCEEDED: self.get_logger().info("导航结果：成功")
        elif result == TaskResult.CANCELED: self.get_logger().warn("导航结果：取消")
        elif result == TaskResult.FAILED: self.get_logger().error("导航结果：失败")
        else: self.get_logger().error("导航结果：返回状态无效")

    def get_current_pose(self):
        """
        通过TF获得目前位姿
        """
        while rclpy.ok():
            try:
                tf = self.buffer_.lookup_transform('map', 'base_footprint', rclpy.time.Time(seconds=0),rclpy.time.Duration(seconds=1))
                transform = tf.transform
                rotation_euler = euler_from_quaternion([
                    transform.rotation.x,
                    transform.rotation.y,
                    transform.rotation.z,
                    transform.rotation.w])
                self.get_logger().info(f'平移:{transform.translation}, 旋转四元数：{transform.rotation}, 旋转欧拉角：{rotation_euler}')
                return transform
            except Exception as e:
                self.get_logger().warn(f'不能够获取目标变换，原因：{str(e)}')

    def speach_text(self, text):
        """
        调用服务播放语音
        """
        while not self.speach_client_.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('语音合成服务未上线，请稍后')
            
        request = SpeachText.Request()
        request.text = text
        future = self.speach_client_.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None :
            result = future.result().result
            if result:
                self.get_logger().info(f'语音合成成功：{text}')
            else:
                self.get_logger().warn(f'语音合成失败：{text}')
        else:
            self.get_logger().warn(f'语音合成服务请求失败')

    def image_callback(self, msg):
        """
        将最新的消息队列放到latest_image中
        """
        self.latest_image = msg

    def record_image(self):
        """
        记录图像
        """
        if self.latest_image is not None:
            pose = self.get_current_pose()
            cv_image = self.bridge.imgmsg_to_cv2(self.latest_image)
            cv2.imwrite(f'{self.image_save_path}image_{pose.translation.x:3.2f}_{pose.translation.y:3.2f}.png', cv_image)


def main():
    rclpy.init()
    patrol = PatrolNode()
    patrol.speach_text('正在初始化位置')
    patrol.init_robot_pose()
    patrol.speach_text('位置初始化完成')

    while rclpy.ok():
        points = patrol.get_target_points()
        for point in points:
            x, y, yaw = point[0], point[1], point[2]
            target_pose = patrol.get_pose_by_xyyaw(x, y, yaw)
            patrol.speach_text(f'正在前往目标点{x},{y}')
            patrol.nav_to_pose(target_pose)
            patrol.speach_text(text=f"已到达目标点{x},{y},准备记录图像")
            patrol.record_image()
            patrol.speach_text(text=f"图像记录完成")
    rclpy.shutdown()