
import numpy as np
import open3d

R = np.array([[ 0.00879794,  0.01444133,  0.99985701],
              [-0.00816441, -0.99986134,  0.01451324],
              [ 0.99992797, -0.00829093, -0.00867882]])
T = np.array([  0.14906038, -0.07857089, -0.16104364])

RSpcd = open3d.io.read_point_cloud("./bag/1216RS/1608117950.048794031.pcd")
VLpcd = open3d.io.read_point_cloud("./bag/1216VL/1608117950.013565063.pcd")
VLpcd2 = open3d.geometry.PointCloud()

VLpoints = np.asarray(VLpcd.points)
VLpoints2 = np.zeros((VLpoints.shape[0],3))

for i in range(VLpoints.shape[0]):
    VLpoints2[i,0] = R[0,0] * VLpoints[i,0] + R[0,1] * VLpoints[i,1] + R[0,2] * VLpoints[i,2] + T[0]
    VLpoints2[i,1] = R[1,0] * VLpoints[i,0] + R[1,1] * VLpoints[i,1] + R[1,2] * VLpoints[i,2] + T[1]
    VLpoints2[i,2] = R[2,0] * VLpoints[i,0] + R[2,1] * VLpoints[i,1] + R[2,2] * VLpoints[i,2] + T[2]

VLpcd2.points = open3d.utility.Vector3dVector(VLpoints2)

RSpcd.paint_uniform_color([0.9,0.3,0])
VLpcd2.paint_uniform_color([0,0,1])

open3d.visualization.draw_geometries([RSpcd,VLpcd2])
# print(type(pcd))
# print(np.asarray(pcd.points))
