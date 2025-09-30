#!/usr/bin/env python3
"""
比较三种不同的旋转平均方法：

方法 1：基于四元数平均

方法 2：基于旋转矩阵平均

方法 3：你提供的 sin/cos + atan2 分量平均法
"""
# 修正并检查三种平均方法的实现：
# 1) 基于四元数（使用 Markley 矩阵特征向量法）
# 2) 基于旋转矩阵（均值后 SVD 正交化，并确保 det=+1）
# 3) 你提供的分量 sin/cos + atan2 方法（原样）
#
# 然后随机生成旋转向量并比较三者（包括可视化和角度差）
import numpy as np
import cv2
import matplotlib.pyplot as plt

np.set_printoptions(precision=6, suppress=True)

# ---------- 工具函数 ----------
def rvec_to_rotmat(rvec):
    R, _ = cv2.Rodrigues(rvec.reshape(3,))
    return R

def rotmat_to_rvec(R):
    rvec, _ = cv2.Rodrigues(R)
    return rvec.reshape(3,)

def rotmat_to_quat(R):
    """Robust conversion from rotation matrix to quaternion (w,x,y,z)."""
    tr = R[0,0] + R[1,1] + R[2,2]
    if tr > 0.0:
        S = np.sqrt(tr + 1.0) * 2.0  # S = 4*qw
        qw = 0.25 * S
        qx = (R[2,1] - R[1,2]) / S
        qy = (R[0,2] - R[2,0]) / S
        qz = (R[1,0] - R[0,1]) / S
    else:
        # Find the major diagonal element
        if (R[0,0] > R[1,1]) and (R[0,0] > R[2,2]):
            S = np.sqrt(1.0 + R[0,0] - R[1,1] - R[2,2]) * 2.0  # S=4*qx
            qw = (R[2,1] - R[1,2]) / S
            qx = 0.25 * S
            qy = (R[0,1] + R[1,0]) / S
            qz = (R[0,2] + R[2,0]) / S
        elif R[1,1] > R[2,2]:
            S = np.sqrt(1.0 + R[1,1] - R[0,0] - R[2,2]) * 2.0  # S=4*qy
            qw = (R[0,2] - R[2,0]) / S
            qx = (R[0,1] + R[1,0]) / S
            qy = 0.25 * S
            qz = (R[1,2] + R[2,1]) / S
        else:
            S = np.sqrt(1.0 + R[2,2] - R[0,0] - R[1,1]) * 2.0  # S=4*qz
            qw = (R[1,0] - R[0,1]) / S
            qx = (R[0,2] + R[2,0]) / S
            qy = (R[1,2] + R[2,1]) / S
            qz = 0.25 * S
    q = np.array([qw, qx, qy, qz], dtype=float)
    return q / np.linalg.norm(q)

def quat_to_rotmat(q):
    """Quaternion (w,x,y,z) -> rotation matrix"""
    q = np.array(q, dtype=float)
    q = q / np.linalg.norm(q)
    w, x, y, z = q
    R = np.array([
        [1 - 2*(y*y + z*z),     2*(x*y - z*w),     2*(x*z + y*w)],
        [2*(x*y + z*w),     1 - 2*(x*x + z*z),     2*(y*z - x*w)],
        [2*(x*z - y*w),         2*(y*z + x*w), 1 - 2*(x*x + y*y)]
    ])
    return R

def rvec_to_euler_xyz_deg(rvec):
    R = rvec_to_rotmat(rvec)
    sy = np.sqrt(R[0,0]**2 + R[1,0]**2)
    singular = sy < 1e-6
    if not singular:
        x = np.arctan2(R[2,1], R[2,2])
        y = np.arctan2(-R[2,0], sy)
        z = np.arctan2(R[1,0], R[0,0])
    else:
        x = np.arctan2(-R[1,2], R[1,1])
        y = np.arctan2(-R[2,0], sy)
        z = 0
    return np.degrees([x, y, z])

def rotation_angle_between(Ra, Rb):
    """返回两个旋转矩阵之间的角度差（弧度）"""
    Rrel = Ra @ Rb.T
    # numerical stability for trace
    val = (np.trace(Rrel) - 1.0) / 2.0
    val = np.clip(val, -1.0, 1.0)
    return np.arccos(val)


# ---------- 方法1：基于四元数（Markley） ----------
def average_quaternions_markley(rvecs, weights=None):
    # 得到四元数数组 (w,x,y,z)
    Rs = [rvec_to_rotmat(rv) for rv in rvecs]
    quats = np.array([rotmat_to_quat(R) for R in Rs])  # shape (N,4)
    N = len(quats)
    if weights is None:
        weights = np.ones(N)
    # Markley matrix M = sum_i w_i * q_i * q_i^T
    M = np.zeros((4,4), dtype=float)
    for q, w in zip(quats, weights):
        q = q.reshape(4,1)
        M += w * (q @ q.T)
    # eigenvector of largest eigenvalue
    vals, vecs = np.linalg.eigh(M)
    q_avg = vecs[:, np.argmax(vals)]
    # ensure normalized
    q_avg = q_avg / np.linalg.norm(q_avg)
    # convert back to rvec
    R_avg = quat_to_rotmat(q_avg)
    return rotmat_to_rvec(R_avg)


# ---------- 方法2：基于旋转矩阵平均（均值后 SVD 投影） ----------
def average_rotmat_svd(rvecs):
    Rs = np.array([rvec_to_rotmat(rv) for rv in rvecs])
    R_mean = np.mean(Rs, axis=0)
    U, S, Vt = np.linalg.svd(R_mean)
    R_ortho = U @ Vt
    # 修正行列式为 +1（确保是有效的旋转）
    if np.linalg.det(R_ortho) < 0:
        U[:, -1] *= -1
        R_ortho = U @ Vt
    return rotmat_to_rvec(R_ortho)

# ---------- 方法3：分量 sin/cos + atan2 ----------
def average_componentwise(rvecs):
    rvecs = np.array(rvecs)
    rvec_sin = np.mean(np.sin(rvecs), axis=0)
    rvec_cos = np.mean(np.cos(rvecs), axis=0)
    rvec_avg = np.arctan2(rvec_sin, rvec_cos)
    return rvec_avg


# ---------- 生成测试数据 ----------
np.random.seed(0)
N = 15
# 随机轴，加上随机角度（-pi..pi）
axes = np.random.randn(N, 3)
axes /= np.linalg.norm(axes, axis=1, keepdims=True)
angles = np.random.uniform(-np.pi, np.pi, size=(N,1))
rvecs = axes * angles  # 每行为一个 rvec (Rodrigues vector)


# ---------- 计算三种平均 ----------
avg_q = average_quaternions_markley(rvecs)
avg_R = average_rotmat_svd(rvecs)
avg_comp = average_componentwise(rvecs)

# 转成旋转矩阵以及欧拉角
R_q = rvec_to_rotmat(avg_q)
R_R = rvec_to_rotmat(avg_R)
R_comp = rvec_to_rotmat(avg_comp)

euler_q = rvec_to_euler_xyz_deg(avg_q)
euler_R = rvec_to_euler_xyz_deg(avg_R)
euler_comp = rvec_to_euler_xyz_deg(avg_comp)

# 计算角度差矩阵（度）
ang_q_R = np.degrees(rotation_angle_between(R_q, R_R))
ang_q_comp = np.degrees(rotation_angle_between(R_q, R_comp))
ang_R_comp = np.degrees(rotation_angle_between(R_R, R_comp))

# ---------- 输出数值结果 ----------
print("平均结果（Rodrigues 向量）:")
print("avg_quaternion (rvec):", np.round(avg_q, 6))
print("avg_rotmat    (rvec):", np.round(avg_R, 6))
print("avg_component (rvec):", np.round(avg_comp, 6))
print()
print("对应的欧拉角 (XYZ, degrees):")
print("avg_quaternion (deg):", np.round(euler_q, 4))
print("avg_rotmat    (deg):", np.round(euler_R, 4))
print("avg_component (deg):", np.round(euler_comp, 4))
print()
print("三者两两的旋转角度差 (degrees):")
print("quat <-> rotmat : {:.6f} deg".format(ang_q_R))
print("quat <-> comp   : {:.6f} deg".format(ang_q_comp))
print("rotmat <-> comp : {:.6f} deg".format(ang_R_comp))

# ---------- 可视化：Rodrigues 向量空间的箭头比较 ----------
fig = plt.figure(figsize=(9,9))
ax = fig.add_subplot(111, projection='3d')
# 随机旋转向量（灰）
ax.quiver(np.zeros(N), np.zeros(N), np.zeros(N),
          rvecs[:,0], rvecs[:,1], rvecs[:,2],
          length=1.0, normalize=False, color='gray', alpha=0.6, linewidth=1)
# 三种平均（放大一些以便可视）
scale = 1.0
ax.quiver(0,0,0, avg_q[0]*scale, avg_q[1]*scale, avg_q[2]*scale, color='red', linewidth=4, label='Avg Quaternion')
ax.quiver(0,0,0, avg_R[0]*scale, avg_R[1]*scale, avg_R[2]*scale, color='green', linewidth=4, label='Avg RotMat')
ax.quiver(0,0,0, avg_comp[0]*scale, avg_comp[1]*scale, avg_comp[2]*scale, color='blue', linewidth=4, label='Avg Componentwise')

ax.set_xlim([-np.pi, np.pi])
ax.set_ylim([-np.pi, np.pi])
ax.set_zlim([-np.pi, np.pi])
ax.set_xlabel('Rx')
ax.set_ylabel('Ry')
ax.set_zlabel('Rz')
ax.set_title('Rotation Average Comparison (Rodrigues vectors)')
ax.legend()
plt.show()
