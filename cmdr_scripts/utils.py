import numpy as np
import math


def rotation_enu2ned():
    Rx = np.array([[1, 0, 0],
                   [0, -1, 0],
                   [0, 0, -1]])
    Rz = np.array([[0, 1, 0],
                   [-1, 0, 0],
                   [0, 0, 1]])
    R = np.matmul(Rx, Rz)
    return R


def position_enu2ned(position):
    R = rotation_enu2ned()
    position_ned = np.matmul(R, position)
    return np.array(position_ned)  # rotate y 180 deg


def velocity_enu2ned(velocity):
    R = rotation_enu2ned()
    velocity_ned = np.matmul(R, velocity)
    return np.array(velocity_ned)


def orientation_enu2ned(quaternion):
    qw = quaternion[0]
    qx = quaternion[1]
    qy = quaternion[2]
    qz = quaternion[3]

    quat_ned = np.array([1/math.sqrt(2)*(qx-qy),
                         1/math.sqrt(2)*(-qw-qz),
                         1/math.sqrt(2)*(qw-qz),
                         1/math.sqrt(2)*(qx+qy)])
    quat_ned /= math.sqrt(quat_ned[0]**2 + quat_ned[1]**2 + quat_ned[2]**2 + quat_ned[3]**2)
    return quat_ned


def omega(quaternion, quaternion_prev, dt):
    dq = (quaternion - quaternion_prev) / dt
    w, x, y, z = quaternion
    omega = 2 * np.mat([[w, x, y, z],
                        [-x, w, z, -y],
                        [-y, -z, w, x],
                        [-z, y, -x, w]]) * np.vstack(dq)
    return np.asarray(omega[1:4]).reshape(-1)


def rpycalc(quaternion):
    phi = np.arctan2(2 * (quaternion[0] * quaternion[3] + quaternion[1] * quaternion[2]),
                     (1 - 2 * (quaternion[2] * quaternion[2] + quaternion[3] * quaternion[3])))
    theta = np.arcsin(2 * (quaternion[0] * quaternion[2] - quaternion[3] * quaternion[1]))
    psi = np.arctan2(2 * (quaternion[0] * quaternion[1] + quaternion[2] * quaternion[3]),
                     (1 - 2 * (quaternion[1] * quaternion[1] + quaternion[2] * quaternion[2])))
    return np.array([phi, theta, psi])

def rpycalc_ZYX(quaternion):
    phi = np.arctan2(2 * (quaternion[0] * quaternion[1] + quaternion[2] * quaternion[3]),
                     (1 - 2 * (quaternion[2] * quaternion[2] + quaternion[1] * quaternion[1])))
    theta = np.arcsin(2 * (quaternion[0] * quaternion[2] - quaternion[3] * quaternion[1]))
    psi = np.arctan2(2 * (quaternion[0] * quaternion[3] + quaternion[2] * quaternion[1]),
                     (1 - 2 * (quaternion[1] * quaternion[1] + quaternion[2] * quaternion[2])))
    return np.array([phi, theta, psi])

def rpy_YZX(quat0, quat1, quat2, quat3):
    rpy = np.array([0.0,0.0,0.0])
    roll = np.arctan2(-2*(quat2*quat3-quat0*quat1), 1-2*(quat1**2+quat3**2))
    pitch = np.arctan2(-2*(quat1*quat3-quat0*quat2), 1-2*(quat2**2+quat3**2))
    yaw = np.arcsin(2*(quat1*quat2+quat0*quat3))
    rpy = [roll,pitch,yaw]
    return rpy

def quat2rot(q):
    q0 = q[3]
    q1 = q[0]
    q2 = q[1]
    q3 = q[2]

    r00 = 2 * (q0 * q0 + q1 * q1) - 1
    r01 = 2 * (q1 * q2 - q0 * q3)
    r02 = 2 * (q1 * q3 + q0 * q2)

    # Second row of the rotation matrix
    r10 = 2 * (q1 * q2 + q0 * q3)
    r11 = 2 * (q0 * q0 + q2 * q2) - 1
    r12 = 2 * (q2 * q3 - q0 * q1)

    # Third row of the rotation matrix
    r20 = 2 * (q1 * q3 - q0 * q2)
    r21 = 2 * (q2 * q3 + q0 * q1)
    r22 = 2 * (q0 * q0 + q3 * q3) - 1

    # 3x3 rotation matrix
    rot_matrix = np.array([[r00, r01, r02],
                           [r10, r11, r12],
                           [r20, r21, r22]])

    return rot_matrix


def rot2quat(R):
    m00 = R[0, 0]
    m01 = R[0, 1]
    m02 = R[0, 2]
    m10 = R[1, 0]
    m11 = R[1, 1]
    m12 = R[1, 2]
    m20 = R[2, 0]
    m21 = R[2, 1]
    m22 = R[2, 2]

    qw = math.sqrt(1 + m00 + m11 + m22) / 2
    qx = (m21 - m12) / (4 * qw)
    qy = (m02 - m20) / (4 * qw)
    qz = (m10 - m01) / (4 * qw)

    return np.array([qw, qx, qy, qz])

# https://d3cw3dd2w32x2b.cloudfront.net/wp-content/uploads/2015/01/matrix-to-quat.pdf
def rot2quat2(R):
    m00 = R[0, 0]
    m01 = R[0, 1]
    m02 = R[0, 2]
    m10 = R[1, 0]
    m11 = R[1, 1]
    m12 = R[1, 2]
    m20 = R[2, 0]
    m21 = R[2, 1]
    m22 = R[2, 2]    
    if m22 < 0:
        if m00 > m11:
            t = 1 + m00 - m11 - m22
            tx = t
            ty = m01 + m10
            tz = m20 + m02
            tw = m12 - m21
        else:
            t = 1 - m00 + m11 - m22
            tx = m01 + m10
            ty = t
            tz = m12 + m21
            tw = m20 - m02
    else:
        if m00 < -m11:
            t = 1 - m00 - m11 + m22
            tx = m20 + m02
            ty = m12 + m21
            tz = t
            tw = m01 - m10
        else:
            t = 1 + m00 + m11 + m22
            tx = m12 - m21
            ty = m20 - m02
            tz = m01 - m10
            tw = t
    qw = tw * 0.5 / math.sqrt(t)
    qx = tx * 0.5 / math.sqrt(t)
    qy = ty * 0.5 / math.sqrt(t)
    qz = tz * 0.5 / math.sqrt(t)
    
    return np.array([qw, qx, qy, qz])

# https://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToQuaternion/            
def rot2quat3(R):
    m00 = R[0, 0]
    m01 = R[0, 1]
    m02 = R[0, 2]
    m10 = R[1, 0]
    m11 = R[1, 1]
    m12 = R[1, 2]
    m20 = R[2, 0]
    m21 = R[2, 1]
    m22 = R[2, 2]
    
    trace = m00 + m11 + m22
    if trace > 0:
        t  = 0.5 / math.sqrt(1 + trace)
        qw = 0.25 / t
        qx = (m21 - m12) * t
        qy = (m02 - m20) * t
        qz = (m10 - m01) * t
    else:
        if m00 > m11 and m00 > m22:
            t = 2 * math.sqrt(1+m00-m11-m22)
            qw = (m21-m12) / t
            qx = 0.25*t
            qy = (m01+m10)/t
            qz = (m02+m20)/t
        elif m11 > m22:
            t = 2 * math.sqrt(1+m11-m00-m22)
            qw = (m02-m20)/t
            qx = (m01+m10)/t
            qy = 0.25*t
            qz = (m12+m21)/t
        else:
            t = 2 * math.sqrt(1+m22-m00-m11)
            qw = (m10-m01)/t
            qx = (m02+m20)/t
            qy = (m12+m21)/t
            qz = 0.25*t
    
    return np.array([qw, qx, qy, qz])            

def multiply_q(q1, q2):
    q = np.zeros(4)
    # a b c d
    # e f g h
    q[0] = q1[0] * q2[0] - q1[1] * q2[1] - q1[2] * q2[2] - q1[3] * q2[3]
    q[1] = q1[1] * q2[0] + q1[0] * q2[1] + q1[2] * q2[3] - q1[3] * q2[2]
    q[2] = q1[0] * q2[2] - q1[1] * q2[3] + q1[2] * q2[0] + q1[3] * q2[1]
    q[3] = q1[0] * q2[3] + q1[1] * q2[2] - q1[2] * q2[1] + q1[3] * q2[0]
    return q

def quatCompress(q):
    # we send the values of the quaternion's smallest 3 elements.
    i_largest = 0
    for i in range(1, 4):
        if abs(q[i]) > abs(q[i_largest]):
            i_largest = i

    # since -q represents the same rotation as q,
    # transform the quaternion so the largest element is positive.
    # this avoids having to send its sign bit.
    negate = q[i_largest] < 0

    # 1/sqrt(2) is the largest possible value 
    # of the second-largest element in a unit quaternion.

    # do compression using sign bit and 9-bit precision per element.
    comp = i_largest
    for i in range(4):
        if i != i_largest:
            negbit = (q[i] < 0) ^ negate
            mag = int(((1 << 9) - 1) * (abs(q[i]) / math.sqrt(0.5)) + 0.5)
            comp = (comp << 10) | (negbit << 9) | mag

    return comp

def quatDecompress(inputQ):
    comp = np.uint32(inputQ)
    mask = (1 << 9) - 1

    i_largest = comp >> 30
    sum_squares = 0
    q = [0.0, 0.0, 0.0, 0.0]

    for i in range(3, -1, -1):
        if i != i_largest:
            mag = comp & mask
            negbit = (comp >> 9) & 0x1
            comp = comp >> 10
            q[i] = math.sqrt(0.5) * mag / mask
            if negbit == 1:
                q[i] = -q[i]
            sum_squares += q[i] * q[i]

    q[i_largest] = math.sqrt(1.0 - sum_squares)

    return q

if __name__ == '__main__':
    quat_r =  [ 0.70038243,  0.11304529, -0.07235717, -0.7010347 ]
    compQ = quatCompress(quat_r)
    quat_r_decom = quatDecompress(compQ)
    print('compQ = ', compQ)
    print('quat_r_decom = ', quat_r_decom)