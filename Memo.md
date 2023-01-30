# GDB

## LLDB vs GDB 

LLDB的文档不太完善（比如想拓展自定义命令行），暂倾向于使用GDB

之前使用LLDB，但是拓展时遇到了一些奇怪的问题，例如：[无法使用PDB](https://discourse.llvm.org/t/fail-to-use-pdb-module-to-debug-lldb-python-command-module/63984) ；以及其相关的文档和案例不足；CLion也暂时只支持Bundled LLDB（使用第三方库时需要一些migration）。于是回归GDB。暂时感觉LLDB有的功能（自己用过的功能），GDB也有。

## [Altering](https://sourceware.org/gdb/onlinedocs/gdb/Altering.html#Altering)

修改一个变量的值（`CLion`中对应的快捷键为`F2`）

```bash
(gdb) set var width=47
```

## Breakpoint

```bash
# 某行打断点
(gdb) break linenum
# 具体到某个文件
(gdb) break filename:linenum
# 具体到某个函数
(gdb) break filename:function
```

## CLI

- 更多可参考：[GDB reference card](https://users.ece.utexas.edu/~adnan/gdb-refcard.pdf)，[GDB cheatsheet](https://darkdust.net/files/GDB%20Cheat%20Sheet.pdf)

| 命令行                       | abbreviation / example | 作用                 |
| ---------------------------- | ---------------------- | -------------------- |
| python-interactive [command] | pi                     | 进入Python交互模式   |
| python [command]             | py [command]           | 执行Python命令行     |
| break [line]                 | break 23               | 打断点               |
| info vtlb                    | —                      | 查看虚函数表         |
| print <variable_name>        | —                      | 查看变量             |
| info threads                 | —                      | 查看线程信息         |
| info locals [variable_name]  | —                      | 查看函数栈的局部变量 |

## Configuration

- [自定义配置文件：GEF](https://gef.readthedocs.io/en/master/)；[GDB 配置文件](https://github.com/cyrus-and/gdb-dashboard)
- [保存历史命令](https://github.com/hellogcc/100-gdb-tips/blob/master/src/save-history-commands.md)，默认的历史命令行导出路径为`~/.gdb_history`

```bash
$ echo "set history save on" >> ~/.gdbinit
```

- [不显示线程启/闭信息](https://stackoverflow.com/questions/10937289/how-can-i-disable-new-thread-thread-exited-messages-in-gdb)

```bash
$ echo "set print thread-events off" >> ~/.gdbinit
```

- [修改prompt](https://sourceware.org/gdb/onlinedocs/gdb/Prompt.html)

```bash
$ echo "set extended-prompt \w (gdb) " >> ~/.gdbinit
```

- 隐藏启动时的提示信息和版权信息，[details](https://stackoverflow.com/questions/63918429/permanently-disable-gdb-startup-text)

```bash
# 方案一（CLI）：设置别名
$ alias gdb="gdb -q"

# 方案二（配置文档）：注意不是gdbinit (from gdb 11)
$ echo "set startup-quietly on" >> ~/.gdbearlyinit
```

## Disassemble

使用 `disassemble` 进一步看出现 `dump core` 出现的汇编位置

<img src="https://natsu-akatsuki.oss-cn-guangzhou.aliyuncs.com/img/JQptKjWwdwGZWkXZ.png!thumbnail" alt="img" style="zoom:67%; " />

## Frame

### Backtrace

```bash
# 查看调用栈
(gdb) backtrace
(gdb) where
(gdb) info stack
```

### Frame

```bash
# 切换到某一帧
(gdb) f <num>

# 查看该帧的局部变量
(gdb) info locals

# 查看形参
(gdb) info args
```

## Library

```bash
# 查看链接的动态库
$ info share
```

![image-20220810232749699](https://natsu-akatsuki.oss-cn-guangzhou.aliyuncs.com/img/image-20220810232749699.png)

## [Pretty Printer](https://sourceware.org/gdb/onlinedocs/gdb/Pretty-Printing.html#Pretty-Printing)

- [启动和关闭的区别](https://sourceware.org/gdb/onlinedocs/gdb/Pretty_002dPrinter-Example.html#Pretty_002dPrinter-Example)
- [gdb command](https://sourceware.org/gdb/onlinedocs/gdb/Pretty_002dPrinter-Commands.html#Pretty_002dPrinter-Commands)

```bash
# 查看已有的pretty printer，包括关闭的
(gdb) info pretty-printer
Print the list of installed pretty-printers. This includes disabled pretty-printers, which are marked as such.

# 关闭pretty printer
(gdb) disable pretty-printer

# 启动pretty printer
(gdb) enable pretty-printer
```

## Python

```bash
$ gdb python 

(gdb) set args <python文件名>
(gdb) run (gdb)
```

## [Custom GDB Command](https://sourceware.org/gdb/onlinedocs/gdb/Python-API.html)

- 一般会使用`regex`来判断输入变量的类型是否符合需求

```python
def vec_lookup_function(val):
    lookup_tag = val.type.tag
    if lookup_tag == None:
        return None

    regex = re.compile("^.*vector_base<.*,.*>$")
    if regex.match(lookup_tag):
        return VectorPrinter(val)

    return None
```

### Practice

- [pretty printer for std vector](https://hgad.net/posts/object-inspection-in-gdb/)：正则，迭代读数据（dereference）部分很OK

## [ROS](http://wiki.ros.org/roslaunch/Tutorials/Roslaunch%20Nodes%20in%20Valgrind%20or%20GDB)

- 启动GDB调试

```bash
# 追加tag
launch-prefix="gdb -ex run --args"

# option:
# -ex <command> 执行给定的GDB command
```

## [Signal](https://github.com/hellogcc/100-gdb-tips/blob/master/src/index.md#%E4%BF%A1%E5%8F%B7)

- [查看gdb如何处理信号](https://github.com/hellogcc/100-gdb-tips/blob/master/src/info-signals.md)（`Pass to program`即让程序执行完信号回调函数后，程序才暂停）

## Practice

### 诊断[rviz段错误](https://segmentfault.com/a/1190000015238799)

- ROS rviz增加 `camera` 或 `image` display时，会出现段错误（segmentation fault）

<img src="https://natsu-akatsuki.oss-cn-guangzhou.aliyuncs.com/img/fmh1yBcmwUYtSSwt.png!thumbnail" alt="img" style="zoom:67%; " />

步骤一：执行程序

```bash
$ gdb python
(gdb) run <py_file>.py
```

步骤二：添加display触发异常

[![img](https://natsu-akatsuki.oss-cn-guangzhou.aliyuncs.com/img/svJsNayoXZXXaa1v.png!thumbnail)](https://natsu-akatsuki.oss-cn-guangzhou.aliyuncs.com/img/svJsNayoXZXXaa1v.png!thumbnail)

步骤三：查看调用栈的情况，可定位到是哪个函数产生段错误（加上full会**同时输出局部变量**）

```bash
(gdb) bt full
```

<img src="https://natsu-akatsuki.oss-cn-guangzhou.aliyuncs.com/img/GncFdU91N5TBJGVo.png!thumbnail" alt="img" style="zoom:67%; " />

### [GDB无响应](https://stackoverflow.com/questions/8978777/why-would-gdb-hang)

原因未知，可通过下发相关信号解决

```bash
$ kill -CONT <pid of the process>
```

## Extension

### [GDBGUI](https://www.gdbgui.com/gettingstarted/)

暂时没感觉新颖的地方

- Install

```bash
$ pip install gdbgui
```

- Usage（[Youtube](https://www.youtube.com/channel/UCUCOSclB97r9nd54NpXMV5A)）

```bash
$ gdbgui
```

## Reference

- [GDB小技巧](https://github.com/hellogcc/100-gdb-tips)

### TODO

- [ ] 了解[Frame Filter](https://chromium.googlesource.com/native_client/nacl-gdb/+/refs/heads/upstream/gdb/python/lib/gdb/command/frame_filters.py)