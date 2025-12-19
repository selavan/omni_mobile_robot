import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

class AStarPlanner:
    def __init__(self, resolution: float, rr: float):
        self.resolution = resolution
        self.rr = rr
        self.obstacle_map = None
        self.min_x, self.min_y, self.max_x, self.max_y = 0, 0, 0, 0
        self.x_width, self.y_width = 0, 0
        self.motion = self.get_motion_model()

    class Node:
        def __init__(self, x: int, y: int, cost: float, parent_index: int):
            self.x = x
            self.y = y
            self.cost = cost
            self.parent_index = parent_index

    def planning(self, start, goal):
        start_node = self.Node(self.calc_xy_index(start[0], self.min_x),
                               self.calc_xy_index(start[1], self.min_y), 0.0, -1)
        goal_node = self.Node(self.calc_xy_index(goal[0], self.min_x),
                              self.calc_xy_index(goal[1], self.min_y), 0.0, -1)

        open_set, closed_set = dict(), dict()
        open_set[self.calc_grid_index(start_node)] = start_node

        while True:
            if len(open_set) == 0:
                print("Open set is empty..")
                break

            c_id = min(
                open_set,
                key=lambda o: open_set[o].cost + self.calc_heuristic(goal_node, open_set[o]))
            current = open_set[c_id]

            if current.x == goal_node.x and current.y == goal_node.y:
                print("Find goal")
                goal_node.parent_index = current.parent_index
                goal_node.cost = current.cost
                break

            del open_set[c_id]
            closed_set[c_id] = current

            for i, _ in enumerate(self.motion):
                node = self.Node(current.x + self.motion[i][0],
                                 current.y + self.motion[i][1],
                                 current.cost + self.motion[i][2], c_id)
                n_id = self.calc_grid_index(node)

                if not self.verify_node(node):
                    continue

                if n_id in closed_set:
                    continue

                if n_id not in open_set:
                    open_set[n_id] = node
                else:
                    if open_set[n_id].cost > node.cost:
                        open_set[n_id] = node

            # Print debug information
            print("Current Node: ({}, {})".format(current.x, current.y))
            print("Open Set Size:", len(open_set))
            print("Closed Set Size:", len(closed_set))

        rx, ry = self.calc_final_path(goal_node, closed_set)

        return rx, ry

    def calc_final_path(self, goal_node, closed_set):
        rx, ry = [self.calc_grid_position(goal_node.x, self.min_x)], [
            self.calc_grid_position(goal_node.y, self.min_y)]
        parent_index = goal_node.parent_index
        while parent_index != -1:
            n = closed_set[parent_index]
            rx.append(self.calc_grid_position(n.x, self.min_x))
            ry.append(self.calc_grid_position(n.y, self.min_y))
            parent_index = n.parent_index

        return rx, ry

    @staticmethod
    def calc_heuristic(n1, n2):
        w = 1.0
        d = w * math.hypot(n1.x - n2.x, n1.y - n2.y)
        return d

    def calc_grid_position(self, index, min_position):
        return index * self.resolution + min_position

    def calc_xy_index(self, position, min_pos):
        return round((position - min_pos) / self.resolution)

    def calc_grid_index(self, node):
        return (node.y - self.min_y) * self.x_width + (node.x - self.min_x)

    def verify_node(self, node):
    	px = self.calc_grid_position(node.x, self.min_x)
    	py = self.calc_grid_position(node.y, self.min_y)

    	if px < self.min_x or py < self.min_y or px >= self.max_x or py >= self.max_y:
        	return False

    	try:
        	if self.obstacle_map[node.x][node.y]:
            		return False
    	except IndexError:
        	print("Index Error: Node coordinates:", node.x, node.y)
        	print("Obstacle map dimensions:", len(self.obstacle_map), len(self.obstacle_map[0]))

    	return True


    def get_motion_model(self):
        return [[1, 0, 1],
                [0, 1, 1],
                [-1, 0, 1],
                [0, -1, 1],
                [-1, -1, math.sqrt(2)],
                [-1, 1, math.sqrt(2)],
                [1, -1, math.sqrt(2)],
                [1, 1, math.sqrt(2)]]

def map_callback(msg):
    global planner
    planner.obstacle_map = [msg.data[i:i + msg.info.width] for i in range(0, len(msg.data), msg.info.width)]
    planner.min_x = msg.info.origin.position.x
    planner.min_y = msg.info.origin.position.y
    planner.max_x = msg.info.width * msg.info.resolution + planner.min_x
    planner.max_y = msg.info.height * msg.info.resolution + planner.min_y
    planner.x_width = msg.info.width
    planner.y_width = msg.info.height
    print("Obstacle map dimensions:", planner.x_width, planner.y_width)

def odom_callback(msg):
    global initial_pose
    initial_pose = [msg.pose.pose.position.x, msg.pose.pose.position.y]

def goal_callback(msg):
    global goal
    goal = [msg.pose.position.x, msg.pose.position.y]

def main(args=None):
    global planner
    global initial_pose
    global goal
    initial_pose = None
    goal = None
    rclpy.init(args=args)

    node = rclpy.create_node('a_star_planner_node')

    planner = AStarPlanner(0.05, 0.105)

    node.create_subscription(OccupancyGrid, '/map', map_callback, 10)
    node.create_subscription(Odometry, '/odom', odom_callback, 10)
    node.create_subscription(PoseStamped, '/goal_pose', goal_callback, 10)
    path_pub = node.create_publisher(Path, '/planned_path', 10)

    while rclpy.ok():
        rclpy.spin_once(node)

        if planner.obstacle_map is not None and initial_pose is not None and goal is not None:
            rx, ry = planner.planning(initial_pose, goal)

            path_msg = Path()
            path_msg.header.frame_id = 'map'
            for x, y in zip(rx, ry):
                pose = PoseStamped()
                pose.pose.position.x = x
                pose.pose.position.y = y
                pose.pose.position.z = 0.0  # Ensure z is a float and default to 0.0
                path_msg.poses.append(pose)

            path_pub.publish(path_msg)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

