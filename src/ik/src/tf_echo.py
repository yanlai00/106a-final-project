import sys
import tf2_ros
import rospy

def tf_echo(target_frame, source_frame):
    tfBuffer = tf2_ros.Buffer()
    tfListener = tf2_ros.TransformListener(tfBuffer)
    r = rospy.Rate(1)
    while not rospy.is_shutdown():
        try:
            trans = tfBuffer.lookup_transform(target_frame, source_frame, rospy.Time())
            translation = trans.transform.translation
            quaternion = trans.transform.rotation
            print("Translation:", [translation.x, translation.y, translation.z])
            print("Rotation (in quaternions):", [quaternion.x, quaternion.y, quaternion.z, quaternion.w])
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            print("tf 2 Error!")
            print(e)
        r.sleep()


if __name__ == '__main__':
    target_frame = sys.argv[1]
    source_frame = sys.argv[2]
    rospy.init_node("listener", anonymous=True)
    tf_echo(target_frame, source_frame)