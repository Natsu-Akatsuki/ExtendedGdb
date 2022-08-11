import os
import sys

LOG_FILE = 'gdb.log'


class ExampleBreakPoint1(gdb.Breakpoint):
    def __init__(self, *args, **kwargs):
        super(ExampleBreakPoint1, self).__init__(*args, **kwargs)

    def stop(self):
        gdb.execute("p example_i")
        return False


class ExampleBreakPoint2(gdb.Breakpoint):
    def __init__(self, *args, **kwargs):
        super(ExampleBreakPoint2, self).__init__(*args, **kwargs)

    def stop(self):
        gdb.execute("p example_i")


class FunctionFinishBreakpoint(gdb.FinishBreakpoint):
    def __init__(self):
        gdb.FinishBreakpoint.__init__(self, gdb.newest_frame(),
                                      internal=True)
        self.silent = True

    def stop(self):
        if str(self.return_value) == "false":
            print("Function returned: %s" % self.return_value)
            return True
        else:
            return False


class ExampleFunctionBreakpoint(gdb.Breakpoint):
    def __init__(self, spec):
        gdb.Breakpoint.__init__(self, spec)
        self.silent = True

    def stop(self):
        FunctionFinishBreakpoint()  # set breakpoint on function return
        return False  # do not stop at function entry


def setup():
    print(f"Running GDB from: {gdb.PYTHONDIR}")
    # 不进行分页，避免设置断点时block
    gdb.execute("set pagination off")
    gdb.execute("set print pretty")

    # if connecting to remote target
    # gdb.execute('target remote <ip>:<port>')

    gdb.execute(f'set logging file {LOG_FILE}')
    gdb.execute('set logging enabled on')

    print('\nReading gdb env...\n')
    gdb.execute('show script-extension')
    gdb.execute('show sysroot')
    gdb.execute('show solib-search-path')
    print('\nSetup complete !!\n')


def main():
    setup()

    ExampleBreakPoint1('test.cpp:34')
    ExampleBreakPoint2('test.cpp:35')
    ExampleFunctionBreakpoint('test.cpp:test_automatic_breakpoint')
    gdb.execute("r")


main()
