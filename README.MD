# Interact with subprocesses running both bash.exe (CygWin) and cmd.exe without terminating them.

## pip install cmdbashinteractive

```python


Args:
	print_stdout (bool, optional): Whether to print stdout from the subprocess (default is True).
	print_stderr (bool, optional): Whether to print stderr from the subprocess (default is True).
	limit_stdout (int, optional): Maximum number of lines to keep in stdout history (default is None).
	limit_stderr (int, optional): Maximum number of lines to keep in stderr history (default is None).
	limit_stdin (int, optional): Maximum number of lines to keep in stdin history (default is None).
	convert_to_83 (bool, optional): Whether to convert file paths to short (8.3) format (default is True).
	exitcommand (str, optional): The command to signal the subprocess to exit (default is "▓▓▓▓▓▓▓▓▓▓▓").
	newline (str, optional): The newline character to use (default is "\n").
	getcomspec (str, optional): The command to get the shell executable (default is "bash.exe").
	wait_to_complete (float, optional): The time to wait for subprocess completion (default is 0.1 seconds).
	**kwargs: Additional keyword arguments for subprocess.Popen.

import os

from cmdbashinteractive import BashInteractive, CmdInteractive

CREATE_NEW_PROCESS_GROUP = 0x00000200
DETACHED_PROCESS = 0x00000008


sh = BashInteractive(
    print_stdout=True,
    print_stderr=True,
    limit_stdout=None,
    limit_stderr=None,
    limit_stdin=None,
    convert_to_83=True,
    exitcommand="▓▓▓▓▓▓▓▓▓▓▓",
    newline="\n",
    getcomspec=r"C:\cygwin\bin\bash.exe",
    wait_to_complete=0.1,
    shell=False,  # **kwargs
    creationflags=DETACHED_PROCESS | CREATE_NEW_PROCESS_GROUP,  # **kwargs
    bufsize=0,  # **kwargs
    env=os.environ.copy(),  # **kwargs
)
stdo00, stde00 = sh("ls")
sh.install("tar")
stdo01, stde01 = sh("ping google.com | grep bytes")
stdo02, stde02 = sh(
    r"cat C:\yolov5max - Copy\2023_08_17.ini"
)  # automatically converted to 8.3 and afterwards to cygwin  when convert_to_83 is True
```


```python
Args:
	print_stdout (bool, optional): Whether to print stdout from the subprocess (default is True).
	print_stderr (bool, optional): Whether to print stderr from the subprocess (default is True).
	limit_stdout (int, optional): Maximum number of lines to keep in stdout history (default is None).
	limit_stderr (int, optional): Maximum number of lines to keep in stderr history (default is None).
	limit_stdin (int, optional): Maximum number of lines to keep in stdin history (default is None).
	convert_to_83 (bool, optional): Whether to convert file paths to short (8.3) format (default is True).
	exitcommand (str, optional): The command to signal the subprocess to exit (default is "▓▓▓▓▓▓▓▓▓▓▓").
	getcomspec (str, optional): The command to get the shell executable (default is None).
	newline (str, optional): The newline character to use (default is "\r\n").
	wait_to_complete (float, optional): The interval to check for subprocess completion (default is 0.1 seconds).
	**kwargs: Additional keyword arguments for subprocess.Popen.


cmd = CmdInteractive(
    print_stdout=True,
    print_stderr=True,
    limit_stdout=10,
    limit_stderr=10,
    limit_stdin=10,
    convert_to_83=True,
    exitcommand="▓▓▓▓▓▓▓▓▓▓▓",
    wait_to_complete=0.1,
    shell=False,  # **kwargs
    creationflags=DETACHED_PROCESS | CREATE_NEW_PROCESS_GROUP,  # **kwargs
    bufsize=0,  # **kwargs
    env=os.environ.copy(),  # **kwargs
    getcomspec=None,
    newline="\r\n",
)
#
strfi = sh(
    r"strings C:\yolov5max - Copy\2023_08_17.ini"
)  # automatically converted to 8.3 when convert_to_83 is True

stdo0, stde0 = cmd(f"ping google.com", wait_to_complete=0.1)
stdo1, stde1 = cmd(f"dir", wait_to_complete=0.1)
stdo2, stde2 = cmd(f"whoami.exe", wait_to_complete=0.1)
stdo3, stde3 = cmd(
    f"ping google.com", wait_to_complete=0
)  # non blocking, not recommended - might mess up stdout,stderr
stdo4, stde4 = cmd(
    f"dir", wait_to_complete=0
)  # non blocking, not recommended - might mess up stdout,stderr
stdo5, stde5 = cmd(
    f"whoami.exe", wait_to_complete=0
)  # non blocking, not recommended - might mess up stdout,stderr


```