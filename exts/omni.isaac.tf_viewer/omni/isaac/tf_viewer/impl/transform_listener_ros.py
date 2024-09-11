import time

import carb
import yaml


def acquire_transform_listener_interface(use_tf2: bool = True):
    interface = TFListener(use_tf2=use_tf2)
    return interface


def release_transform_listener_interface(interface):
    interface.finalize()


class TFListener:
    def __init__(self, node_name: str = "ros_tf_listener", use_tf2: bool = True) -> None:
        self._node_name = node_name
        self._use_tf2 = use_tf2

        self._listener = None

    def initialize(self, distro):
        import rosgraph
        import rospy

        # check ROS master
        try:
            rosgraph.Master("/rostopic").getPid()
        except:
            carb.log_warn("ROS master is not running")
            return False
        # start ROS node
        try:
            rospy.init_node(self._node_name)
            time.sleep(0.1)
            carb.log_info("ROS node started ({})".format(self._node_name))
        except rospy.ROSException as e:
            carb.log_error("ROS node ({}): {}".format(self._node_name, e))

        # tf2 implementation
        if self._use_tf2:
            import rospy
            import tf2_ros

            self._tf_buffer = tf2_ros.Buffer()
            self._listener = tf2_ros.TransformListener(self._tf_buffer)
            # internal methods
            self._time = rospy.Time
            self._lookup_transform = self._tf_buffer.lookup_transform
            self._all_frames_as_yaml = self._tf_buffer.all_frames_as_yaml
        # tf implementation
        else:
            import tf

            self._listener = tf.TransformListener()
            # internal methods
            self._time = self._listener.getLatestCommonTime
            self._lookup_transform = self._listener.lookupTransform
            self._all_frames_as_yaml = self._listener._buffer.all_frames_as_yaml
        return True

    def finalize(self):
        import rospy

        if self._listener:
            if self._use_tf2:
                self._listener.unregister()
            self._listener = None

        # shutdown ROS node
        # rospy.signal_shutdown("omni.isaac.tf_viewer")

    def get_transforms(self, root_frame):
        frames = set()
        relations = []
        transforms = {}

        if self._listener:
            frames_info = yaml.load(self._all_frames_as_yaml(), Loader=yaml.SafeLoader)
            if type(frames_info) is dict:
                for frame, info in frames_info.items():
                    frames.add(frame)
                    frames.add(info["parent"])
                    relations.append((frame, info["parent"]))
                    try:
                        transform = self._lookup_transform(
                            root_frame, frame, self._time() if self._use_tf2 else self._time(root_frame, frame)
                        )
                        if self._use_tf2:
                            translation = transform.transform.translation
                            rotation = transform.transform.rotation
                            transform = (
                                [translation.x, translation.y, translation.z],
                                [rotation.x, rotation.y, rotation.z, rotation.w],
                            )
                        transforms[frame] = transform
                    except:
                        pass

        return frames, transforms, relations

    def get_transform(self, target_frame, source_frame):
        try:
            transform = self._lookup_transform(
                target_frame, source_frame, self._time() if self._use_tf2 else self._time(target_frame, source_frame)
            )
        except Exception as e:
            return (), type(e).__name__
        if self._use_tf2:
            translation = transform.transform.translation
            rotation = transform.transform.rotation
            transform = (
                [translation.x, translation.y, translation.z],
                [rotation.x, rotation.y, rotation.z, rotation.w],
            )
        return transform, ""

    def reset(self):
        # remove "TF_OLD_DATA ignoring data from the past" warning
        if self._listener:
            carb.log_info("Reset TF listener (ROS)")
            self._tf_buffer.clear() if self._use_tf2 else self._listener._buffer.clear()

    def is_ready(self):
        return self._listener != None
