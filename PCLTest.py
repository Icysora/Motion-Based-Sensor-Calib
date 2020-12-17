import pcl
import pcl.pcl_visualization

cloud = pcl.load("/home/lwh/1120_left/rslidar_points/11_1605841539_351303577.pcd")
print(cloud.size)
visual = pcl.pcl_visualization.CloudViewing()
visual.ShowMonochromeCloud(cloud)

flag = True
while flag:
    flag = not(visual.WasStopped())