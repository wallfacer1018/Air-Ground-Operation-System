import rospy
import tf
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from tf import transformations
from cv_bridge import CvBridge, CvBridgeError
from math import sqrt, sin, cos, pi


class Pixel2World:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/monocular/image_raw", Image, self.image_callback)
        self.odom_sub = rospy.Subscriber("/mavros/local_position/odom", Odometry, self.odom_callback)
        self.world_coords_pub = rospy.Publisher("/world_coords", Point, queue_size=10)

        self.K = np.array([[369.502083, 0, 1280],
                           [0, 369.502083, 720],
                           [0, 0, 1]])  # Intrinsic matrix
        self.odom_yaw = 0.0

        self.listener = tf.TransformListener()

    def pixel2image(self, pixel):
        """Convert pixel coordinates to image coordinates."""
        return np.array([pixel[0], pixel[1], 1.0])

    def image2camera(self, image_coords):
        """Convert image coordinates to camera coordinates using the intrinsic matrix."""
        K_inv = np.linalg.inv(self.K)
        return K_inv.dot(image_coords)

    def camera2world(self, camera_coords, transform):
        """Convert camera coordinates to world coordinates using the transform."""
        R = np.array(transform[:3, :3])  # Rotation matrix
        t = np.array(transform[:3, 3])  # Translation vector

        world_coords = R.dot(camera_coords) + t
        return world_coords

    def get_transform(self):
        """Get the transformation from the camera to the world."""
        try:
            self.listener.waitForTransform("/map", "/monocular_link", rospy.Time(0), rospy.Duration(0.02))
            (trans, rot) = self.listener.lookupTransform("/map", "/monocular_link", rospy.Time(0))
            transform_matrix = tf.transformations.compose_matrix(translate=trans,
                                                                 angles=tf.transformations.euler_from_quaternion(rot))

            return transform_matrix
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logerr("Transform error")

            return None

    def odom_callback(self, msg):
        # Convert the quaternion to a tuple (x, y, z, w)
        quaternion = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        )

        # Convert quaternion to Euler angles
        euler = transformations.euler_from_quaternion(quaternion)

        # Extract the yaw (rotation around Z-axis)
        self.odom_yaw = euler[2]

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(f"CvBridge Error: {e}")
            return

        # Example: Choose the center pixel of the image
        pixel = (cv_image.shape[1] // 2, cv_image.shape[0] // 2)

        # Step 1: Pixel to Image Coordinates
        image_coords = self.pixel2image(pixel)

        # Step 2: Image to Camera Coordinates
        camera_coords = self.image2camera(image_coords)

        # Step 3: Camera to World Coordinates
        transform = self.get_transform()
        if transform is not None:
            world_coords = self.camera2world(camera_coords, transform)
            self.publish_world_coords(world_coords)

    def publish_world_coords(self, world_coords):
        point_msg = Point()
        point_msg.x = world_coords[0] + sqrt(2) * cos(self.odom_yaw + 0.75 * pi) # offset on x axis
        point_msg.y = world_coords[1] + sqrt(2) * sin(self.odom_yaw + 0.75 * pi) # offset on y axis
        point_msg.z = 0  # Should be 0 as the camera is facing downwards

        self.world_coords_pub.publish(point_msg)
        rospy.loginfo(point_msg)


if __name__ == '__main__':
    rospy.init_node('pixel2world')
    
    Pixel2World()
    
    rospy.spin()
