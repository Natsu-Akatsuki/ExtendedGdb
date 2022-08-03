import cv2
import numpy as np
import open3d as o3d
import gdb
from pathlib import Path
from datetime import datetime
from termcolor import cprint


class PCLViewer(gdb.Command):
    def __init__(self):
        super(PCLViewer, self).__init__('pcl_viewer',
                                        gdb.COMMAND_SUPPORT,
                                        gdb.COMPLETE_SYMBOL)

    def invoke(self, arg, from_tty):
        """
        arg (gdb.Value object):
        usage: (gdb) pcl_viewer <pcl_cloud>
        """

        args = gdb.string_to_argv(arg)
        pointcloud_name = args[0]
        # 等价于在gdb执行<...>.get().points
        pointcloud = gdb.parse_and_eval(f"{pointcloud_name}.get()")
        point_num = pointcloud['width'] * pointcloud['height']

        pointcloud_under_hood = gdb.parse_and_eval(f"{pointcloud_name}.get().points.data()")
        pointcloud_np = np.zeros((point_num, 3), dtype=np.float32)
        for i in range(point_num):
            pointcloud_np[i][0] = pointcloud_under_hood[i]['x']
            pointcloud_np[i][1] = pointcloud_under_hood[i]['y']
            pointcloud_np[i][2] = pointcloud_under_hood[i]['z']

        point_cloud_o3d = o3d.geometry.PointCloud()
        point_cloud_o3d.points = o3d.utility.Vector3dVector(pointcloud_np)
        o3d.visualization.draw_geometries([point_cloud_o3d], width=500, height=500)

        # 导出点云文件
        if len(args) > 1:
            if isinstance(args[1], str) and args[1] == '-s':
                temp_path = Path.home() / Path(".gdb/temp").resolve()
                temp_path.mkdir(parents=True, exist_ok=True)
                cur_time = datetime.now().strftime('%H-%M-%S')
                file_name = str(temp_path) + "/" + f"{cur_time}.pcd"
                o3d.io.write_point_cloud(file_name, point_cloud_o3d)
                cprint(f"the pointcloud file is saved as {file_name}", "red")

        self.dont_repeat()


PCLViewer()
