#!/usr/bin/env python3

import rclpy
from rclpy.time import Time
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, MapMetaData
from tf2_ros import Buffer, TransformListener
from tf2_ros import TransformException  # base class of Lookup/Connectivity/Extrapolation exceptions
from tf_transformations import euler_from_quaternion
import math

PRIOR_PROB = 0.5
OCC_PROB = 0.9
FREE_PROB = 0.35

class Pose:
    def __init__(self, px=0, py=0):
        self.x = px
        self.y = py

def prob2logodds(p):
    return math.log(p / (1 - p))

def logodds2prob(l):
    try:
        return 1 - (1 / (1 + math.exp(l)))
    except OverflowError:
        return 1.0 if l > 0 else 0.0

def poseToCell(pose: Pose, map_info: MapMetaData): #map_info is of type MapMetaData, which contains information about the map such as its width, height, and resolution. The function takes a Pose object (which has x and y coordinates) and converts it to a cell index in the occupancy grid. The formula used is: cell_index = map_width * pose_y + pose_x. This formula calculates the linear index of the cell in a 1D array representation of the 2D occupancy grid.
    return map_info.width * pose.y + pose.x #finding i to iterate through the stored map data in a 1D array representation of the 2D occupancy grid.

def coordinatesToPose(px, py, map_info: MapMetaData): # World coordinates (m) -> grid cell indices
    pose = Pose() #basically finding aij in matrix sense (here ayx).
    pose.x = round((px - map_info.origin.position.x) / map_info.resolution)
    pose.y = round((py - map_info.origin.position.y) / map_info.resolution)
    return pose #behavior of this function is to convert the coordinates of a point in the world frame (px, py) to the corresponding cell indices in the occupancy grid map. It takes into account the origin and resolution of the map to perform this conversion.

def poseOnMap(pose: Pose, map_info: MapMetaData):
    return pose.x < map_info.width and pose.x >= 0 and pose.y < map_info.height and pose.y >= 0

def bresenham(start: Pose, end: Pose):
    line = []

    dx = end.x - start.x
    dy = end.y - start.y

    xsign = 1 if dx > 0 else -1
    ysign = 1 if dy > 0 else -1

    dx = abs(dx)
    dy = abs(dy)

    if dx > dy:
        xx = xsign
        xy = 0
        yx = 0
        yy = ysign
    else:
        tmp = dx
        dx = dy
        dy = tmp
        xx = 0
        xy = ysign
        yx = xsign
        yy = 0

    D = 2 * dy - dx
    y = 0

    for i in range(dx + 1):
        line.append(Pose(start.x + i * xx + y * yx, start.y + i * xy + y * yy))
        if D >= 0:
            y += 1
            D -= 2 * dx
        D += 2 * dy

    return line

def inverseSensorModel(p_robot: Pose, p_beam: Pose):
    occ_values = []
    line = bresenham(p_robot, p_beam)

    for pose in line[:-1]:
        occ_values.append((pose, FREE_PROB))  # FREE

    occ_values.append((line[-1], OCC_PROB))  # OCCUPIED
    return occ_values

class MappingWithKnownPoses(Node):
    def __init__(self, name):
        super().__init__(name)
        self.declare_parameter("width", 50.0)
        self.declare_parameter("height", 50.0)
        self.declare_parameter("resolution", 0.1)

        width = float(self.get_parameter("width").get_parameter_value().double_value)
        height = float(self.get_parameter("height").get_parameter_value().double_value)
        resolution = float(self.get_parameter("resolution").get_parameter_value().double_value)

        self.map_ = OccupancyGrid()
        self.map_.info.resolution = resolution
        self.map_.info.width = round(width / resolution) #in number of cells for grid
        self.map_.info.height = round(height / resolution) #in number of cells for grid
        self.map_.info.origin.position.x = float(-round(width / 2.0))
        self.map_.info.origin.position.y = float(-round(height / 2.0))
        self.map_.header.frame_id = "odom"
        self.map_.data = [-1] * (self.map_.info.width * self.map_.info.height) #vector of -1s for unknown cells
        self.probability_map_ = [prob2logodds(PRIOR_PROB)] * (self.map_.info.width * self.map_.info.height)

        # Step 1: Get the robot's current pose from TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self) #Listens to /tf and /tf_static and fills the buffer.
        self.scan_sub = self.create_subscription(LaserScan, "scan", self.scanCallback, 10)
        self.map_pub = self.create_publisher(OccupancyGrid, "map", 1)
        self.timer = self.create_timer(1.0, self.timerCallback)

    def scanCallback(self, scan: LaserScan):
        try: 
            t = self.tf_buffer.lookup_transform(self.map_.header.frame_id, scan.header.frame_id, Time()) #stroing transform between odom/map frame and laser frame in variable t.
        except TransformException: #Used when the requested transform isn't available
            self.get_logger().error("Unable to transform between /odom and /base_footprint")
            return
 
        # Step 2: Convert robot pose to grid coordinates
        robot_p = coordinatesToPose(t.transform.translation.x, t.transform.translation.y, self.map_.info)
        if not poseOnMap(robot_p, self.map_.info):
            self.get_logger().error("The robot is out of the map!")
            return

        (roll, pitch, yaw) = euler_from_quaternion(
            [t.transform.rotation.x, t.transform.rotation.y,
             t.transform.rotation.z, t.transform.rotation.w])
        
        # Step 3: For each laser beam
        for i, r in enumerate(scan.ranges):
            # Skip invalid readings (inf / NaN) and readings below min range
            if not math.isfinite(r) or r < scan.range_min:
                continue
 
            hit = r < scan.range_max
            r_used = min(r, scan.range_max)

            # Step 4: Calculate the beam's angle
            angle = scan.angle_min + (i * scan.angle_increment) + yaw
            
            # Step 5: Convert polar (range, angle) to Cartesian (px, py) in the LASER frame
            px = r_used * math.cos(angle) # <-- px is relative to laser
            py = r_used * math.sin(angle)

            # Step 6: Transform to the MAP/ODOM frame by adding the robot's position 
            px += t.transform.translation.x # <-- px is now in WORLD coordinates
            py += t.transform.translation.y

            # Step 7: Convert world coordinates to grid cell indices
            beam_p = coordinatesToPose(px, py, self.map_.info)
            if not poseOnMap(beam_p, self.map_.info):
                continue

            poses = inverseSensorModel(robot_p, beam_p)

            for pose, value in poses:
                if poseOnMap(pose, self.map_.info):
                    cell = poseToCell(pose, self.map_.info)
                    self.probability_map_[cell] += prob2logodds(value) - prob2logodds(PRIOR_PROB)

    def timerCallback(self):
        self.map_.data = [int(logodds2prob(value) * 100) for value in self.probability_map_]
        self.map_.header.stamp = self.get_clock().now().to_msg()
        self.map_pub.publish(self.map_)

def main(args=None):
    rclpy.init(args=args)
    node = MappingWithKnownPoses("mapping_with_known_poses")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()