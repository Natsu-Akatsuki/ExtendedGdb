import re

import gdb
import numpy as np


def type_map(data_type):
    type_dict = {
        "float": np.float32,
        "double": np.float64
    }
    return type_dict[data_type]


def parse_eigen(eigen_mat):
    # 获取数据类型
    tag = gdb.parse_and_eval(f"{eigen_mat}").type.unqualified().strip_typedefs().tag
    regex = re.compile("<.*>")
    m = regex.findall(tag)[0][1:-1]  # remove <>
    template_params = m.split(',')
    data_type = template_params[0]
    data_type = type_map(data_type)
    # 读取数据
    inferior = gdb.inferiors()
    data_address = gdb.parse_and_eval(f"&{eigen_mat}.m_storage.m_data.array")
    memory_size = gdb.parse_and_eval(f"sizeof({eigen_mat}.m_storage.m_data.array)")
    memory_data = inferior[0].read_memory(data_address, memory_size)  # bytes (e.g 128 bytes)
    # note: 需要使用column-major（eigen存储数据为column-major）
    mat = np.frombuffer(memory_data, dtype=data_type).reshape((4, 4), order='F')
    return mat
