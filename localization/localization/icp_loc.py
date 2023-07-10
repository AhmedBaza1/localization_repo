import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, Imu
import numpy as np
import open3d as o3d 
from .tools import*
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformException
from tf2_ros import TransformBroadcaster
import pygicp

class Pointcloudloc(Node):

    def __init__(self):
        super().__init__('icp_localization')
        
        self.params = self.declare_parameters(
            namespace='',
            parameters=[
                ('voxelSize', None),
                ('threshold', None),
                ('iteration_max', None),
                ('transformation_init.use_global_loc', None),
                ('transformation_init.init_trans', None),
                ('relative_fitness', None), 
                ('relative_rmse', None), 
                ('icp_normal_rad', None), 
                ('sampling_ratio', None)
            ])
        # self.frame_count = 0 # for cropping the map every 10 frames 
        # self.map = None
        self.tf_v2r = None
        self.tf_I2r = None
        self.first = True
        self.imu_init = True
        self.use_global_loc = self.params[3].value
        self.init_transform = np.asarray(self.params[4].value).reshape((4,4))
        self.icp_threshold = self.params[1].value
        self.max_iteration = self.params[2].value
        self.voxel_size = self.params[0].value
        self.relative_fitness = self.params[5].value
        self.relative_rmse = self.params[6].value
        self.icp_rad = self.params[7].value
        self.sampling_ratio = self.params[8].value
        self.transform = None
        self.imu_reset = None
        self.prev_imu = np.identity(4)
        self.imu_rotation = np.identity(4) 
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.pcd_publisher = self.create_publisher(sensor_msgs.PointCloud2, '/velo_pcd', 10)
        self.local_map_publisher = self.create_publisher(sensor_msgs.PointCloud2, '/local_map', 10)
        assert len(sys.argv) > 1, "No pcd file given."
        assert os.path.exists(sys.argv[1]), "File doesn't exist."
        self.scan = o3d.geometry.PointCloud()
        # self.local_map = o3d.geometry.PointCloud()
        map_path = sys.argv[1]
        self.map = o3d.io.read_point_cloud(map_path)
        self.get_logger().info(f'points1 :  {np.asarray(self.map.points).shape}')
        self.map.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=self.icp_rad, max_nn=self.max_iteration)) 
        self.ground_downsample()
        self.map = self.map.voxel_down_sample(voxel_size=2)
        self.get_logger().info(f'points2 :  {np.asarray(self.map.points).shape}')
        # self.map = self.map.random_down_sample(0.8)
        self.subscription = self.create_subscription(PointCloud2,'/velodyne_points',self.pointcloud_callback,10)
        self.subscription  
        self.imu_subscription = self.create_subscription(Imu,'/imu',self.imu_callback,10)
        self.imu_subscription
        self.creat_tf(self.init_transform)

        
    def pointcloud_callback(self, msg):
        
        points_as_array = np.array(list(read_points(msg, skip_nans=True))) # point cloud as np.array
        self.scan.points = o3d.utility.Vector3dVector(points_as_array)
        self.scan = self.scan.voxel_down_sample(voxel_size=1)
        # self.scan = self.scan.random_down_sample(0.5)
        # if self.first:
        #     # self.frame_count = 10
        #     if self.use_global_loc: 
        #         self.init_transform = execute_fast_global_registration(self.voxel_size, self.scan, self.map)
        #     # else:
        #     #     self.init_transform = np.asarray(self.params[4].value).reshape((4,4))
        #     #     self.creat_tf(self.init_transform)
        #     try:
        #         t = self.tf_buffer.lookup_transform('robot_base','velodyneLidar',rclpy.time.Time())
        #         p,q = tf_to_pq(t)
        #         self.tf_v2r = pq_2_trans(p,q)
        #         self.get_logger().info('transform recived')
        #     except TransformException as ex:
        #             self.get_logger().info(
        #                 f'Could not transform robot_base to velodyneLidar: {ex}')
        #     self.first = False
        # self.scan.transform(self.tf_v2r)
        # self.init_transform =  self.init_transform  @ self.imu_rotation
        # self.map_update()
        time = self.get_clock().now()
        self.transform = self.register(self.scan)
        self.init_transform = self.transform @ np.array([[1, 0, 0, 0.5],
                                                         [0, 1, 0, 0],
                                                         [0, 0, 1, 0], 
                                                         [0, 0, 0, 1]])
        self.get_logger().info(f'Registeration :  {self.get_clock().now()-time}')
        # self.get_logger().info(f'Transform: \n {self.transform}')
        self.creat_tf(self.transform)
        cloud = point_cloud(np.asarray(self.scan.points), 'robot_base')
        self.pcd_publisher.publish(cloud)
        
    def register(self, scan):
        result_icp = o3d.pipelines.registration.registration_icp(
                        scan,
                        self.map,
                        self.icp_threshold,
                        self.init_transform,
                        o3d.pipelines.registration.TransformationEstimationPointToPlane(),
                        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=self.max_iteration)
                        )
        self.get_logger().info(f'points2 :  {np.asarray(result_icp.correspondence_set).shape[0]}')
        return result_icp.transformation
    
    def map_update(self, threshold=50): #cropping the map with a radious = threshold
        if self.frame_count ==10: 
            map_cloud = np.array(self.map.points)
            try:
                t = self.tf_buffer.lookup_transform('Map','robot_base',rclpy.time.Time())
                p,q = tf_to_pq(t)
                observation_origin = p
            except TransformException as ex:
                    self.get_logger().info(
                        f'Could not transform robot to map: {ex}')
            filtered_cloud = []
            for point in map_cloud:
                distance = np.linalg.norm(point - observation_origin)
                if distance <= threshold:
                    filtered_cloud.append(point)
            self.frame_count = 0
            self.local_map.points= o3d.utility.Vector3dVector(filtered_cloud)
            cloud = point_cloud(np.asarray(self.local_map.points), 'Map')
            self.local_map_publisher.publish(cloud)
        else: 
            self.frame_count+=1

    # ! GICP registeration # testing 
    def register_gicp(self,scan):
        source = scan
        target = self.map_points
        target = pygicp.downsample(target, 0.6)
        source = pygicp.downsample(source, 0.25)
        matrix = pygicp.align_points(target, source,initial_guess = self.init_transform)
        return matrix
    
    def ground_downsample(self): 
        normals = np.asarray(self.map.normals)
        colors = 0.5 * (normals + 1)  # Normalize and shift the range to [0, 1]
        self.map.colors = o3d.utility.Vector3dVector(colors)
        idxs = colors[:, 2] < 0.5
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(np.asarray(self.map.points)[idxs])
        pcd.normals = o3d.utility.Vector3dVector(np.asarray(self.map.normals)[idxs])
        self.map.points = pcd.points 
        self.map.normals = pcd.normals 
        
    def imu_callback(self,msg):
        if self.imu_init:
            translation = np.array([0, 0 , 0])
            quat = np.array([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
            # quat = correct_imu(quat)
            T_init = pq_2_trans(translation,quat)
            self.imu_reset = np.linalg.inv(T_init)
            # self.imu_reset = T_init
            self.imu_init = 0

        translation = np.array([0, 0 , 0])
        quat = np.array([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
        # quat = correct_imu(quat)
        T = self.imu_reset @ pq_2_trans(translation,quat)
        self.imu_rotation = np.linalg.inv(self.prev_imu) @T
        self.imu_rotation[:3, 3] = [0, 0, 0]
        # self.get_logger().info(f'IMU: \n {self.imu_rotation}')
        self.prev_imu = T
        
        
        
                  
    def creat_tf(self,tf):
        ttf = tf.copy() 
        m = ttf[:3,:3]
        p = ttf[:3, 3]
        r = R.from_matrix(m)
        q = r.as_quat()
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'Map'
        t.child_frame_id = 'robot_base'
        t.transform.translation.x = p[0]
        t.transform.translation.y = p[1]
        t.transform.translation.z = p[2]
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        self.tf_broadcaster.sendTransform(t)
        
    def debuging_tap(self, pcd): 
        pass 

def main(args=None):
    rclpy.init(args=args)

    localize = Pointcloudloc()

    rclpy.spin(localize)

    localize.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
