import re
import gdb
import gdb.printing
import sys
from pathlib import Path

sys.path.append(str(Path(__file__).parent))
import pointcloud_gdb


class ExampleFunc(gdb.Function):
    """
    usage: print $greet("<...>")
    """

    def __init__(self):
        super(ExampleFunc, self).__init__("greet")

    def invoke(self, name):
        return "Hello, %s!" % name.string()


class ExampleCommand(gdb.Command):
    """
    usage: (gdb) hello-world
    """

    def __init__(self):
        super(ExampleCommand, self).__init__("hello-world", gdb.COMMAND_USER)

    def invoke(self, args, from_tty):
        # callback

        # Exception Handling
        argv = gdb.string_to_argv(args)
        if len(argv) != 0:
            raise gdb.GdbError("hello-world takes no arguments")

        print("Hello, World!")


class ExamplePrinter:
    """
    Custom print a bar object.
    struct foo { int a, b; };
    """

    def __init__(self, val):
        self.val = val

    def to_string(self):
        """
        重写to_string方法
        """
        return "x: " + str(self.val["a"]) + " y: " + str(self.val["b"])


def lookup_type(val):
    lookup_tag = val.type.tag
    if lookup_tag is None:
        return None
    regex = re.compile("^foo$")
    if regex.match(lookup_tag):
        return ExamplePrinter(val)
    return None


# register the pretty printer to gdb（这只是其中一种办法，还有其他的方案，具体看docs）
gdb.pretty_printers.append(lookup_type)


# 设置断点
class ExampleBreakpoint(gdb.Breakpoint):
    def stop(self):
        inf_val = gdb.parse_and_eval("var")
        if inf_val == 3:
            # 当其中的变量值为3时，则触发断点
            return True
        return False


class PointcloudBreakPoint(gdb.Breakpoint):
    def __init__(self, *args, **kwargs):
        super(PointcloudBreakPoint, self).__init__(*args, **kwargs)

    def parse_data(self, cloud_name=None, doView=False, doSave=False):
        pointcloud_gdb.parse_pointcloud(cloud_name=cloud_name, doView=doView)
        if doSave:
            pass

    def stop(self):
        self.parse_data(cloud_name="target_cloud", doView=True)
        return False


# ExampleBreakpoint("<file_name:line_num>")
PointcloudBreakPoint("test.cpp:29")

# trigger the registration of the command with eigen_gdb.py
ExampleCommand()
ExampleFunc()
