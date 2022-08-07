import numpy as np
import open3d as o3d
import gdb
from pathlib import Path
from datetime import datetime
from termcolor import cprint


def parse_pointcloud(cloud_name=None, doView=False):
    """
    已测试：pcl::PointCloud<pcl::PointXYZ>::Ptr
    """

    if cloud_name is None:
        return

    m_ptr_ = gdb.parse_and_eval(f"{cloud_name}").type.fields()[0].name
    #
    # 20.04: boost smarter pointer (px)
    # 22.04: std smarter pointer (_M_ptr)
    pointcloud = gdb.parse_and_eval(f"{cloud_name}")[f"{m_ptr_}"].dereference()

    pointcloud_vec = pointcloud['points']
    point_num = pointcloud['width'] * pointcloud['height']

    # note: 以向量的方式读取（如下迭代的时间较长，但够用了）
    # @https://stackoverflow.com/questions/57147836/indexing-c-vector-in-python-gdb-script
    i = 0
    value_reference = pointcloud_vec['_M_impl']['_M_start']
    pointcloud_np = np.zeros((point_num, 3), dtype=np.float32)

    while value_reference != pointcloud_vec['_M_impl']['_M_finish']:
        pointcloud_np[i][0] = value_reference.dereference()['x']
        pointcloud_np[i][1] = value_reference.dereference()['y']
        pointcloud_np[i][2] = value_reference.dereference()['z']
        i += 1
        value_reference += 1

    point_cloud_o3d = o3d.geometry.PointCloud()
    point_cloud_o3d.points = o3d.utility.Vector3dVector(pointcloud_np)
    if doView:
        o3d.visualization.draw_geometries([point_cloud_o3d], width=500, height=500)
    return point_cloud_o3d


class PCLViewer(gdb.Command):
    def __init__(self):
        super(PCLViewer, self).__init__('pcl_viewer',
                                        gdb.COMMAND_SUPPORT,
                                        gdb.COMPLETE_SYMBOL)
        # 按回车键不触发重复执行
        self.dont_repeat()

    def invoke(self, arg, from_tty):
        """
        arg (gdb.Value object):
        usage: (gdb) pcl_viewer <pcl_cloud>
        """

        args = gdb.string_to_argv(arg)
        pointcloud_name = args[0]

        point_cloud_o3d = parse_pointcloud(cloud_name=pointcloud_name, doView=True)
        # 导出点云文件
        if len(args) > 1:
            if isinstance(args[1], str) and args[1] == '-s':
                cur_time = datetime.now().strftime('%H-%M-%S')

                temp_path = Path.home() / Path("gdb/temp").resolve()
                temp_path.mkdir(parents=True, exist_ok=True)
                file_name = str(temp_path) + "/" + f"{cur_time}.pcd"
                o3d.io.write_point_cloud(file_name, point_cloud_o3d)
                cprint(f"the pointcloud file is saved as {file_name}", "red")


PCLViewer()
