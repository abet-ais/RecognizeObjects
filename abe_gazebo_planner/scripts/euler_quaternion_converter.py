import tf
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Vector3
import math
import numpy as np

def euler_to_quaternion(euler):
    q = tf.transformations.quaternion_from_euler(euler.x, euler.y, euler.z)
    return Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

# x, y, z, w = euler_to_quaternion(Vector3(0.0, 0.0, 0.0))
q = euler_to_quaternion(Vector3(-2.12437, 0.314159, 0))
# q = euler_to_quaternion(Vector3(0, 0, 0 ))
print q
