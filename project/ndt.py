import sys
from datetime import datetime
import numpy as np
from scipy.spatial.transform import Rotation
from termcolor import cprint
from pathlib import Path
import pandas as pd
import open3d as o3d

import os
sys.path.append(os.getenv("HOME") + "/" + ".gdb")

import pointcloud_gdb
import eigen_gdb


class NDTBreakPoint(gdb.Breakpoint):
    def __init__(self, *args, **kwargs):
        super(NDTBreakPoint, self).__init__(*args, **kwargs)
        self.cols = ["点云时间戳s",
                     "GNSS时间戳s",
                     "[真值] x", "[预测值] x",
                     "[真值] y", "[预测值] y",
                     "[真值] z", "[预测值] z",
                     "[真值] yaw", "[预测值] yaw",
                     "[真值] pitch", "[预测值] pitch",
                     "[真值] roll", "[预测值] roll"]
        self.df = None

        cur_time = datetime.now().strftime('%H-%M-%S')
        self.temp_path = Path.home() / "NoeticTrt" / "result" / cur_time
        self.temp_path.mkdir(parents=True, exist_ok=True)
        self.csv_file = str(self.temp_path / "result.csv")

    def parse_data(self, doSave=True):

        data = np.zeros(14)
        data[0] = gdb.parse_and_eval("ros_time.sec") + gdb.parse_and_eval("ros_time.nsec") * 1e-9
        data[1] = 0

        # step1：读取NDT初值
        pose = "predict_pose"
        transform_mat = eigen_gdb.parse_eigen(pose)
        rotation_mat = transform_mat[:3, :3]
        rotation = Rotation.from_matrix(rotation_mat)
        euler = rotation.as_euler('ZYX', degrees=True)

        data[3] = transform_mat[0, 3]
        data[5] = transform_mat[1, 3]
        data[7] = transform_mat[2, 3]
        data[9] = euler[0]
        data[11] = euler[1]
        data[13] = euler[2]

        # step2：读取真值
        pose = "gnss_pose"
        transform_mat = eigen_gdb.parse_eigen(pose)
        rotation_mat = transform_mat[:3, :3]
        rotation = Rotation.from_matrix(rotation_mat)
        euler = rotation.as_euler('ZYX', degrees=True)

        data[2] = transform_mat[0, 3]
        data[4] = transform_mat[1, 3]
        data[6] = transform_mat[2, 3]
        data[8] = euler[0]
        data[10] = euler[1]
        data[12] = euler[2]

        data = np.array([str(data[0]), *data[1:]], dtype=object)
        new_df = pd.DataFrame(data.reshape(1, -1), columns=self.cols)

        print(new_df.to_string(index=False))

        if self.df is not None:
            self.df = pd.concat([self.df, new_df], ignore_index=True)
        else:
            self.df = new_df

        self.df.to_csv(self.csv_file, header=True, index=True)

        # 导出点云
        if doSave:
            pointcloud_name = "filtered_cloud_ptr"
            point_cloud_o3d = pointcloud_gdb.parse_pointcloud(pointcloud_name, doView=False)

            file_name = str(self.temp_path) + "/" + f"{data[0]}.pcd"
            o3d.io.write_point_cloud(file_name, point_cloud_o3d)
            cprint(f"the pointcloud file is saved as {file_name}", "red")

    def stop(self):
        self.parse_data()
        return False


def setup():
    gdb.execute("set pagination off")

    log_file = str(Path.home() / 'slam.log')
    gdb.execute(f'set logging file {log_file}')
    gdb.execute('set logging on')
    print('\nSetup complete !!\n')


def main():
    setup()
    ndt = NDTBreakPoint('matching.cpp:156')


main()
