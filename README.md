# GDB

- 本仓库记录基于python的gdb拓展命令

## 碎碎念

- 之前使用LLDB，但是拓展时遇到了一些奇奇怪怪的问题，例如：[detail](https://discourse.llvm.org/t/fail-to-use-pdb-module-to-debug-lldb-python-command-module/63984) + 相关的文档和案例不足 + CLion暂时只支持Bundled LLDB（使用第三方库时需要一些migration...）。于是回归GDB。暂时感觉LLDB有的功能（自己用过的功能），GDB也有。

## 依赖

Test in ubuntu 22.04，系统内置python（3.10）

```bash
$ pip3 install -r requirements.txt

$ pip3 install open3d

# 如果使用python3.10（e.g ubuntu22.04内置Python默认版本为3.10），则需要：
$ pip3 install --user --pre https://storage.googleapis.com/open3d-releases-master/python-wheels/open3d-0.15.2+a1f65dc-cp310-cp310-manylinux_2_27_x86_64.whl
```

## 安装

```bash
$ git clone https://github.com/Natsu-Akatsuki/ExtendedGdb ~/.gdb
```

## 用例

### 矩阵

#### Pretty Printer

- 适用于Eigen数据类型

```bash
# 导入脚本
$ echo "source ~/.gdb/eigen_gdb.py" >> ~/.gdbinit
```



<img src="https://natsu-akatsuki.oss-cn-guangzhou.aliyuncs.com/img/image-20220805111825205.png" alt="image-20220805111825205" style="zoom: 80%;" />

### 图片

#### Viewer

基于Matplotlib查看图片（效果类同于J家三方插件`OpenCV Image Viewer`）

```bash
# 导入脚本
$ echo "source ~/.gdb/img_gdb.py" >> ~/.gdbinit

# 可视化图片
(gdb) imshow <img>
```

![image-20220803112325107](https://natsu-akatsuki.oss-cn-guangzhou.aliyuncs.com/img/image-20220803112325107.png)

#### [Pretty_Printer](https://docs.opencv.org/4.x/d6/d25/tutorial_linux_gdb_pretty_printer.html#tutorial_linux_gdb_pretty_printer_installation)

```bash
# 导入脚本
$ echo "source ~/.gdb/mat_pretty_printer.py" >> ~/.gdbinit
```

### 点云

#### Viewer

基于Open3D查看点云

```bash
# 导入脚本
$ echo "source ~/.gdb/pointcloud_gdb.py" >> ~/.gdbinit

# 可视化点云
(gdb) pcl_viewer <pointcloud>
# 可视化和保存点云
(gdb) pcl_viewer <pointcloud> -s
```

![image-20220803112041923](https://natsu-akatsuki.oss-cn-guangzhou.aliyuncs.com/img/image-20220803112041923.png)

## Q&A

### IDE

- CLion GDB 调用pcl_viewer后显示 "Evaluation hung: call func(e) This may be caused by something like a deadlock or an infinite loop.To prevent this from happening when variables are calculated, please toggle 'Enable value renderers' off."

> （ctrl + shift + a） and typing "registry" and then enter. And then adjusting the "cidr.debugger.timeout.eveluate" setting to a larger，默认是30000ms（30s），可调大（ref：[detail](https://intellij-support.jetbrains.com/hc/en-us/community/posts/360001100139-Gdb-Debugging-Issue)）

### GDB

- 能否支持打开多个可视化窗口来查看数据

> 不能。实测，只有关闭了GUI才能进行其他操作，即开新的GUI和执行GDB指令（用多线程也不能解决这个问题）。原因参考gdb文档：“gdb install handlers for SIGCHLD and SIGINT. Python code must not override these, or even change the options using sigaction. If your program changes the handling of these signals, gdb will most likely stop working correctly. **Note that it is unfortunately common for GUI toolkits to install a SIGCHLD handler**.”

#### 内联

- 使用gdb.parse_and_eval("pointcloud.get().points.data()")时，显示 gdb.error: Cannot evaluate function -- may be inlined

> 需要对该函数显式地进行实例化（@[ref](https://stackoverflow.com/a/22163969/19371684)）
>
> ```
> // 在对应的函数中添加
> pointcloud.get()->points.data();
> ```

#### 寄存器

- Couldn't get registers: No such process

> 实测时发现这种方法取值 gdb.parse_and_eval("pointcloud.get().points.data()") 不太稳定，因此还是一步步来，先得到gdb.Value再取值

## 参考资料

- [github：cv_imshow](https://github.com/cuekoo/GDB-ImageWatch)
- [GDB 自动化脚本](https://blog.csdn.net/nirendao/article/details/105942335)
- [GDB 官网文档：Python-API](https://sourceware.org/gdb/onlinedocs/gdb/Python-API.html)
- [OpenCV官方 pretty printer](https://github.com/opencv/opencv/tree/4.x/samples/gdb)
- [red hat pretty printer](https://access.redhat.com/documentation/en-us/red_hat_enterprise_linux/6/html/developer_guide/debuggingprettyprinters)
- [vscode中使用clang+clangd+lldb](https://blog.mchook.cn/2021/08/17/vscode%E4%B8%AD%E4%BD%BF%E7%94%A8clang+clangd+lldb/)