import rclpy
from rclpy.node import Node
from moveit_msgs.msg import PlanningScene, CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose

class TestScenePublisher(Node):
    def __init__(self):
        super().__init__('test_scene_publisher')
        self.scene_pub = self.create_publisher(PlanningScene, '/planning_scene', 10)
        self.timer = self.create_timer(1.0, self.publish_scene)

    def publish_scene(self):
        scene = PlanningScene()
        scene.is_diff = True

        ground = CollisionObject()
        ground.id = "ground_test"
        ground.header.frame_id = "base_link"
        ground.operation = CollisionObject.ADD

        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [2.0, 2.0, 0.1]

        ground_pose = Pose()
        ground_pose.position.z = -0.05

        ground.primitives.append(box)
        ground.primitive_poses.append(ground_pose)

        scene.world.collision_objects.append(ground)

        self.scene_pub.publish(scene)
        self.get_logger().info("Published test planning scene.")

def main(args=None):
    rclpy.init(args=args)
    node = TestScenePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()