import numpy as np
import open3d as o3d
import gdb
from pathlib import Path
from datetime import datetime
from termcolor import cprint
import re


def parse_pointcloud(cloud_name=None, doView=False):
    """
    已测试：pcl::PointCloud<pcl::PointXYZ>::Ptr
    """

    if cloud_name is None:
        return

    # 20.04: boost smarter pointer (px)
    # 22.04: std smarter pointer (_M_ptr)
    pointcloud = None
    ptr_version = gdb.parse_and_eval(f"target_cloud").type.unqualified().strip_typedefs().tag
    if bool(re.search("^std::shared_ptr<.*>$", ptr_version)):
        # e.g. "std::shared_ptr<pcl::PointCloud<pcl::PointXYZ> >"
        pointcloud = gdb.parse_and_eval(f"{cloud_name}")["_M_ptr"].dereference()
    elif bool(re.search("^boost::shared_ptr<.*>$", ptr_version)):
        # e.g.  "boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> >"
        pointcloud = gdb.parse_and_eval(f"{cloud_name}")["px"].dereference()
    else:
        return

    pointcloud_vec = pointcloud['points']  # ->vector
    point_num = pointcloud['width'] * pointcloud['height']

    # step1: 从内存中读取点云数据
    inferior = gdb.inferiors()[0]
    start_address = pointcloud_vec['_M_impl']['_M_start']
    print(pointcloud_vec['_M_impl']['_M_start'])
    end_address = pointcloud_vec['_M_impl']['_M_finish']
    print(type(start_address))
    memory_size = int(end_address) - int(start_address)
    underlying_data = inferior.read_memory(start_address, memory_size)  # bytes (e.g 128 bytes)

    # step2：将数据封装为numpy数据类型
    # note: 需要使用row-major（eigen存储数据为row-major）
    pointcloud_np = np.frombuffer(underlying_data, dtype=np.float32, count=-1)  # read all data
    pointcloud_np = pointcloud_np.reshape((point_num, 4), order='C')[:, :3]

    # step3：可视化
    if doView:
        point_cloud_o3d = o3d.geometry.PointCloud()
        point_cloud_o3d.points = o3d.utility.Vector3dVector(pointcloud_np)
        o3d.visualization.draw_geometries([point_cloud_o3d], width=500, height=500)
        return point_cloud_o3d

    return None


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
        if (len(args) > 1) and (point_cloud_o3d is not None):
            if isinstance(args[1], str) and args[1] == '-s':
                cur_time = datetime.now().strftime('%H-%M-%S')

                temp_path = Path.home() / Path("gdb/temp").resolve()
                temp_path.mkdir(parents=True, exist_ok=True)
                file_name = str(temp_path) + "/" + f"{cur_time}.pcd"
                o3d.io.write_point_cloud(file_name, point_cloud_o3d)
                cprint(f"the pointcloud file is saved as {file_name}", "red")


PCLViewer()
