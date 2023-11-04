import ctypes
import itertools
import os
import platform
import shutil
import signal
import sys
import subprocess
import threading
from collections import deque
import re
from time import sleep as sleep_
from math import floor

compiledregex = re.compile(r"^[A-Z]:\\", flags=re.I)
from functools import cache

CREATE_NEW_PROCESS_GROUP = 0x00000200
DETACHED_PROCESS = 0x00000008


aptget = r"""#!/bin/bash
# apt-cyg: install tool for Cygwin similar to debian apt-get
#
# The MIT License (MIT)
#
# Copyright (c) 2013 Trans-code Design
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

if [ ${BASH_VERSINFO}${BASH_VERSINFO[1]} -lt 42 ]
then
  echo 'Bash version 4.2+ required'
  exit
fi

usage="\
NAME
  apt-cyg - package manager utility

SYNOPSIS
  apt-cyg [operation] [options] [targets]

DESCRIPTION
  apt-cyg is a package management utility that tracks installed packages on a
  Cygwin system. Invoking apt-cyg involves specifying an operation with any
  potential options and targets to operate on. A target is usually a package
  name, file name, URL, or a search string. Targets can be provided as command
  line arguments.

OPERATIONS
  install
    Install package(s).

  remove
    Remove package(s) from the system.

  update
    Download a fresh copy of the master package list (setup.ini) from the
    server defined in setup.rc.

  download
    Retrieve package(s) from the server, but do not install/upgrade anything.

  show
    Display information on given package(s).

  depends
    Produce a dependency tree for a package.

  rdepends
    Produce a tree of packages that depend on the named package.

  list
    Search each locally-installed package for names that match regexp. If no
    package names are provided in the command line, all installed packages will
    be queried.

  listall
    This will search each package in the master package list (setup.ini) for
    names that match regexp.

  category
    Display all packages that are members of a named category.

  listfiles
    List all files owned by a given package. Multiple packages can be specified
    on the command line.

  search
    Search for downloaded packages that own the specified file(s). The path can
    be relative or absolute, and one or more files can be specified.

  searchall
    Search cygwin.com to retrieve file information about packages. The provided
    target is considered to be a filename and searchall will return the
    package(s) which contain this file.

  mirror
    Set the mirror; a full URL to a location where the database, packages, and
    signatures for this repository can be found. If no URL is provided, display
    current mirror.

  cache
    Set the package cache directory. If a file is not found in cache directory,
    it will be downloaded. Unix and Windows forms are accepted, as well as
    absolute or regular paths. If no directory is provided, display current
    cache.

OPTIONS
  --nodeps
    Specify this option to skip all dependency checks.

  --version
    Display version and exit.
"

version="\
apt-cyg version 1

The MIT License (MIT)

Copyright (c) 2005-9 Stephen Jungels
"

function wget {
  if command wget -h &>/dev/null
  then
    command wget "$@"
  else
    warn wget is not installed, using lynx as fallback
    set "${*: -1}"
    lynx -source "$1" > "${1##*/}"
  fi
}

function find-workspace {
  # default working directory and mirror

  # work wherever setup worked last, if possible
  cache=$(awk '
  BEGIN {
    RS = "\n\\<"
    FS = "\n\t"
  }
  $1 == "last-cache" {
    print $2
  }
  ' /etc/setup/setup.rc)

  mirror=$(awk '
  /last-mirror/ {
    getline
    print $1
  }
  ' /etc/setup/setup.rc)
  mirrordir=$(sed '
  s / %2f g
  s : %3a g
  ' <<< "$mirror")

  mkdir -p "$cache/$mirrordir/$arch"
  cd "$cache/$mirrordir/$arch"
  if [ -e setup.ini ]
  then
    return 0
  else
    get-setup
    return 1
  fi
}

function get-setup {
  touch setup.ini
  mv setup.ini setup.ini-save
  wget -N $mirror/$arch/setup.bz2
  if [ -e setup.bz2 ]
  then
    bunzip2 setup.bz2
    mv setup setup.ini
    echo Updated setup.ini
  else
    echo Error updating setup.ini, reverting
    mv setup.ini-save setup.ini
  fi
}

function check-packages {
  if [[ $pks ]]
  then
    return 0
  else
    echo No packages found.
    return 1
  fi
}

function warn {
  printf '\e[1;31m%s\e[m\n' "$*" >&2
}

function apt-update {
  if find-workspace
  then
    get-setup
  fi
}

function apt-category {
  check-packages
  find-workspace
  for pkg in "${pks[@]}"
  do
    awk '
    $1 == "@" {
      pck = $2
    }
    $1 == "category:" && $0 ~ query {
      print pck
    }
    ' query="$pks" setup.ini
  done
}

function apt-list {
  local sbq
  for pkg in "${pks[@]}"
  do
    let sbq++ && echo
    awk 'NR>1 && $1~pkg && $0=$1' pkg="$pkg" /etc/setup/installed.db
  done
  let sbq && return
  awk 'NR>1 && $0=$1' /etc/setup/installed.db
}

function apt-listall {
  check-packages
  find-workspace
  local sbq
  for pkg in "${pks[@]}"
  do
    let sbq++ && echo
    awk '$1~pkg && $0=$1' RS='\n\n@ ' FS='\n' pkg="$pkg" setup.ini
  done
}

function apt-listfiles {
  check-packages
  find-workspace
  local pkg sbq
  for pkg in "${pks[@]}"
  do
    (( sbq++ )) && echo
    if [ ! -e /etc/setup/"$pkg".lst.gz ]
    then
      download "$pkg"
    fi
    gzip -cd /etc/setup/"$pkg".lst.gz
  done
}

function apt-show {
  find-workspace
  check-packages
  for pkg in "${pks[@]}"
  do
    (( notfirst++ )) && echo
    awk '
    $1 == query {
      print
      fd++
    }
    END {
      if (! fd)
        print "Unable to locate package " query
    }
    ' RS='\n\n@ ' FS='\n' query="$pkg" setup.ini
  done
}

function apt-depends {
  find-workspace
  check-packages
  for pkg in "${pks[@]}"
  do
    awk '
    @include "join"
    $1 == "@" {
      apg = $2
    }
    $1 == "requires:" {
      for (z=2; z<=NF; z++)
        reqs[apg][z-1] = $z
    }
    END {
      prpg(ENVIRON["pkg"])
    }
    function smartmatch(small, large,    values) {
      for (each in large)
        values[large[each]]
      return small in values
    }
    function prpg(fpg) {
      if (smartmatch(fpg, spath)) return
      spath[length(spath)+1] = fpg
      print join(spath, 1, length(spath), " > ")
      if (isarray(reqs[fpg]))
        for (each in reqs[fpg])
          prpg(reqs[fpg][each])
      delete spath[length(spath)]
    }
    ' setup.ini
  done
}

function apt-rdepends {
  find-workspace
  for pkg in "${pks[@]}"
  do
    awk '
    @include "join"
    $1 == "@" {
      apg = $2
    }
    $1 == "requires:" {
      for (z=2; z<=NF; z++)
        reqs[$z][length(reqs[$z])+1] = apg
    }
    END {
      prpg(ENVIRON["pkg"])
    }
    function smartmatch(small, large,    values) {
      for (each in large)
        values[large[each]]
      return small in values
    }
    function prpg(fpg) {
      if (smartmatch(fpg, spath)) return
      spath[length(spath)+1] = fpg
      print join(spath, 1, length(spath), " < ")
      if (isarray(reqs[fpg]))
        for (each in reqs[fpg])
          prpg(reqs[fpg][each])
      delete spath[length(spath)]
    }
    ' setup.ini
  done
}

function apt-download {
  check-packages
  find-workspace
  local pkg sbq
  for pkg in "${pks[@]}"
  do
    (( sbq++ )) && echo
    download "$pkg"
  done
}

function download {
  local pkg digest digactual
  pkg=$1
  # look for package and save desc file

  awk '$1 == pc' RS='\n\n@ ' FS='\n' pc=$pkg setup.ini > desc
  if [ ! -s desc ]
  then
    echo Unable to locate package $pkg
    exit 1
  fi

  # download and unpack the bz2 or xz file

  # pick the latest version, which comes first
  set -- $(awk '$1 == "install:"' desc)
  if (( ! $# ))
  then
    echo 'Could not find "install" in package description: obsolete package?'
    exit 1
  fi

  dn=$(dirname $2)
  bn=$(basename $2)

  # check the md5
  digest=$4
  case ${#digest} in
   32) hash=md5sum    ;;
  128) hash=sha512sum ;;
  esac
  mkdir -p "$cache/$mirrordir/$dn"
  cd "$cache/$mirrordir/$dn"
  if ! test -e $bn || ! $hash -c <<< "$digest $bn"
  then
    wget -O $bn $mirror/$dn/$bn
    $hash -c <<< "$digest $bn" || exit
  fi

  tar tf $bn | gzip > /etc/setup/"$pkg".lst.gz
  cd ~-
  mv desc "$cache/$mirrordir/$dn"
  echo $dn $bn > /tmp/dwn
}

function apt-search {
  check-packages
  echo Searching downloaded packages...
  for pkg in "${pks[@]}"
  do
    key=$(type -P "$pkg" | sed s./..)
    [[ $key ]] || key=$pkg
    for manifest in /etc/setup/*.lst.gz
    do
      if gzip -cd $manifest | grep -q "$key"
      then
        package=$(sed '
        s,/etc/setup/,,
        s,.lst.gz,,
        ' <<< $manifest)
        echo $package
      fi
    done
  done
}

function apt-searchall {
  cd /tmp
  for pkg in "${pks[@]}"
  do
    printf -v qs 'text=1&arch=%s&grep=%s' $arch "$pkg"
    wget -O matches cygwin.com/cgi-bin2/package-grep.cgi?"$qs"
    awk '
    NR == 1 {next}
    mc[$1]++ {next}
    /-debuginfo-/ {next}
    /^cygwin32-/ {next}
    {print $1}
    ' FS=-[[:digit:]] matches
  done
}

function apt-install {
  check-packages
  find-workspace
  local pkg dn bn requires wr package sbq script
  for pkg in "${pks[@]}"
  do

  if grep -q "^$pkg " /etc/setup/installed.db
  then
    echo Package $pkg is already installed, skipping
    continue
  fi
  (( sbq++ )) && echo
  echo Installing $pkg

  download $pkg
  read dn bn </tmp/dwn
  echo Unpacking...

  cd "$cache/$mirrordir/$dn"
  tar -x -C / -f $bn
  # update the package database

  awk '
  ins != 1 && pkg < $1 {
    print pkg, bz, 0
    ins = 1
  }
  1
  END {
    if (ins != 1) print pkg, bz, 0
  }
  ' pkg="$pkg" bz=$bn /etc/setup/installed.db > /tmp/awk.$$
  mv /etc/setup/installed.db /etc/setup/installed.db-save
  mv /tmp/awk.$$ /etc/setup/installed.db

  [ -v nodeps ] && continue
  # recursively install required packages

  requires=$(awk '$1=="requires", $0=$2' FS=': ' desc)
  cd ~-
  wr=0
  if [[ $requires ]]
  then
    echo Package $pkg requires the following packages, installing:
    echo $requires
    for package in $requires
    do
      if grep -q "^$package " /etc/setup/installed.db
      then
        echo Package $package is already installed, skipping
        continue
      fi
      apt-cyg install --noscripts $package || (( wr++ ))
    done
  fi
  if (( wr ))
  then
    echo some required packages did not install, continuing
  fi

  # run all postinstall scripts

  [ -v noscripts ] && continue
  find /etc/postinstall -name '*.sh' | while read script
  do
    echo Running $script
    $script
    mv $script $script.done
  done
  echo Package $pkg installed

  done
}

function apt-remove {
  check-packages
  cd /etc
  cygcheck awk bash bunzip2 grep gzip mv sed tar xz > setup/essential.lst
  for pkg in "${pks[@]}"
  do

  if ! grep -q "^$pkg " setup/installed.db
  then
    echo Package $pkg is not installed, skipping
    continue
  fi

  if [ ! -e setup/"$pkg".lst.gz ]
  then
    warn Package manifest missing, cannot remove $pkg. Exiting
    exit 1
  fi
  gzip -dk setup/"$pkg".lst.gz
  awk '
  NR == FNR {
    if ($NF) ess[$NF]
    next
  }
  $NF in ess {
    exit 1
  }
  ' FS='[/\\\\]' setup/{essential,$pkg}.lst
  esn=$?
  if [ $esn = 0 ]
  then
    echo Removing $pkg
    if [ -e preremove/"$pkg".sh ]
    then
      preremove/"$pkg".sh
      rm preremove/"$pkg".sh
    fi
    mapfile dt < setup/"$pkg".lst
    for each in ${dt[*]}
    do
      [ -f /$each ] && rm /$each
    done
    for each in ${dt[*]}
    do
      [ -d /$each ] && rmdir --i /$each
    done
    rm -f setup/"$pkg".lst.gz postinstall/"$pkg".sh.done
    awk -i inplace '$1 != ENVIRON["pkg"]' setup/installed.db
    echo Package $pkg removed
  fi
  rm setup/"$pkg".lst
  if [ $esn = 1 ]
  then
    warn apt-cyg cannot remove package $pkg, exiting
    exit 1
  fi

  done
}

function apt-mirror {
  if [ "$pks" ]
  then
    awk -i inplace '
    1
    /last-mirror/ {
      getline
      print "\t" pks
    }
    ' pks="$pks" /etc/setup/setup.rc
    echo Mirror set to "$pks".
  else
    awk '
    /last-mirror/ {
      getline
      print $1
    }
    ' /etc/setup/setup.rc
  fi
}

function apt-cache {
  if [ "$pks" ]
  then
    vas=$(cygpath -aw "$pks")
    awk -i inplace '
    1
    /last-cache/ {
      getline
      print "\t" vas
    }
    ' vas="${vas//\\/\\\\}" /etc/setup/setup.rc
    echo Cache set to "$vas".
  else
    awk '
    /last-cache/ {
      getline
      print $1
    }
    ' /etc/setup/setup.rc
  fi
}

if [ -p /dev/stdin ]
then
  mapfile -t pks
fi

# process options
until [ $# = 0 ]
do
  case "$1" in

    --nodeps)
      nodeps=1
      shift
    ;;

    --noscripts)
      noscripts=1
      shift
    ;;

    --version)
      printf "$version"
      exit
    ;;

    update)
      command=$1
      shift
    ;;

    list | cache  | remove | depends | listall  | download | listfiles |\
    show | mirror | search | install | category | rdepends | searchall )
      if [[ $command ]]
      then
        pks+=("$1")
      else
        command=$1
      fi
      shift
    ;;

    *)
      pks+=("$1")
      shift
    ;;

  esac
done

set -a

if type -t apt-$command | grep -q function
then
  readonly arch=${HOSTTYPE/i6/x}
  apt-$command
else
  printf "$usage"
fi
"""


@cache
def get_comspec(gcmd=None, convert_to_83=True):
    if not gcmd:
        comspec = os.environ.get("ComSpec")
        if not comspec:
            system_root = os.environ.get("SystemRoot", "")
            comspec = os.path.join(system_root, "System32", "cmd.exe")
            if not os.path.isabs(comspec):
                raise FileNotFoundError(
                    "shell not found: neither %ComSpec% nor %SystemRoot% is set"
                )
        cmd = comspec
        return [cmd, "/k"]

    else:
        if os.path.exists(gcmd):
            pa = os.path.normpath(gcmd)
        else:
            pa = shutil.which(gcmd)
        if convert_to_83:
            pa = get_short_path_name(pa)
        return [pa]


iswindows = "win" in platform.platform().lower()
if iswindows:
    startupinfo = subprocess.STARTUPINFO()
    startupinfo.dwFlags |= subprocess.STARTF_USESHOWWINDOW
    startupinfo.wShowWindow = subprocess.SW_HIDE
    creationflags = subprocess.CREATE_NO_WINDOW
    invisibledict = {
        "startupinfo": startupinfo,
        "creationflags": creationflags,
        "start_new_session": True,
    }
    from ctypes import wintypes

    windll = ctypes.LibraryLoader(ctypes.WinDLL)
    user32 = windll.user32
    kernel32 = windll.kernel32
    GetExitCodeProcess = windll.kernel32.GetExitCodeProcess
    CloseHandle = windll.kernel32.CloseHandle
    GetExitCodeProcess.argtypes = [
        ctypes.wintypes.HANDLE,
        ctypes.POINTER(ctypes.c_ulong),
    ]
    CloseHandle.argtypes = [ctypes.wintypes.HANDLE]
    GetExitCodeProcess.restype = ctypes.c_int
    CloseHandle.restype = ctypes.c_int

    GetWindowRect = user32.GetWindowRect
    GetClientRect = user32.GetClientRect
    _GetShortPathNameW = kernel32.GetShortPathNameW
    _GetShortPathNameW.argtypes = [wintypes.LPCWSTR, wintypes.LPWSTR, wintypes.DWORD]
    _GetShortPathNameW.restype = wintypes.DWORD
else:
    invisibledict = {}


def is_process_alive(pid):
    if re.search(
        b"ProcessId=" + str(pid).encode() + rb"\s*$",
        subprocess.run(
            "wmic process list FULL", capture_output=True, **invisibledict
        ).stdout,
        flags=re.MULTILINE,
    ):
        return True
    return False


@cache
def convert_path_to_short(string, returnstring=True):
    if not iswindows:
        return string

    l2 = [q - 1 for q in index_all(s=string, n=":\\")]
    addtostring = []
    try:
        result1 = list_split(l=string, indices_or_sections=l2)
    except Exception:
        return string
    try:
        for string in result1:
            if not compiledregex.search(string):
                addtostring.append(string)
                continue
            activepath = ""
            lastfoundpath = ""
            lastfoundpath_end = 0
            for ini, letter in enumerate(string):
                activepath += letter
                if ini < 3:
                    continue
                if letter.isspace():
                    continue
                if os.path.exists(activepath):
                    lastfoundpath = activepath
                    lastfoundpath_end = ini
            if lastfoundpath:
                addtostring.append(get_short_path_name(lastfoundpath))
                addtostring.append(string[lastfoundpath_end + 1 :])
            else:
                addtostring.append(string)
        if not returnstring:
            return addtostring
        return "".join(addtostring)
    except Exception:
        return string


def list_split(l, indices_or_sections):
    Ntotal = len(l)
    try:
        Nsections = len(indices_or_sections) + 1
        div_points = [0] + list(indices_or_sections) + [Ntotal]
    except TypeError:
        Nsections = int(indices_or_sections)
        if Nsections <= 0:
            raise ValueError("number sections must be larger than 0.") from None
        Neach_section, extras = divmod(Ntotal, Nsections)
        section_sizes = (
            [0] + extras * [Neach_section + 1] + (Nsections - extras) * [Neach_section]
        )
        div_points = []
        new_sum = 0
        for i in section_sizes:
            new_sum += i
            div_points.append(new_sum)

    sub_arys = []
    lenar = len(l)
    for i in range(Nsections):
        st = div_points[i]
        end = div_points[i + 1]
        if st >= lenar:
            break
        sub_arys.append((l[st:end]))

    return sub_arys


@cache
def get_short_path_name(long_name):
    try:
        if not iswindows:
            return long_name
        output_buf_size = 4096
        output_buf = ctypes.create_unicode_buffer(output_buf_size)
        _ = _GetShortPathNameW(long_name, output_buf, output_buf_size)
        return output_buf.value
    except Exception as e:
        sys.stderr.write(f"{e}\n")
        return long_name


def sleep(secs):
    if secs == 0:
        return
    maxrange = 50 * secs
    if isinstance(maxrange, float):
        sleeplittle = floor(maxrange)
        sleep_((maxrange - sleeplittle) / 50)
        maxrange = int(sleeplittle)
    if maxrange > 0:
        for _ in range(maxrange):
            sleep_(0.016)


def killthread(threadobject):
    # based on https://pypi.org/project/kthread/
    if not threadobject.is_alive():
        return True
    tid = -1
    for tid1, tobj in threading._active.items():
        if tobj is threadobject:
            tid = tid1
            break
    if tid == -1:
        sys.stderr.write(f"{threadobject} not found")
        return False
    res = ctypes.pythonapi.PyThreadState_SetAsyncExc(
        ctypes.c_long(tid), ctypes.py_object(SystemExit)
    )
    if res == 0:
        return False
    elif res != 1:
        ctypes.pythonapi.PyThreadState_SetAsyncExc(tid, 0)
        return False
    return True


def index_all(s, n):
    indototal = 0
    allindex = []

    while True:
        try:
            indno = s[indototal:].index(n)
            indototal += indno + 1
            allindex.append(indototal - 1)
        except ValueError:
            break
    return allindex


def send_ctrl_commands(pid, command=0):
    if iswindows:
        commandstring = r"""import ctypes, sys; CTRL_C_EVENT, CTRL_BREAK_EVENT, CTRL_CLOSE_EVENT, CTRL_LOGOFF_EVENT, CTRL_SHUTDOWN_EVENT = 0, 1, 2, 3, 4; kernel32 = ctypes.WinDLL("kernel32", use_last_error=True); (lambda pid, cmdtosend=CTRL_C_EVENT: [kernel32.FreeConsole(), kernel32.AttachConsole(pid), kernel32.SetConsoleCtrlHandler(None, 1), kernel32.GenerateConsoleCtrlEvent(cmdtosend, 0), sys.exit(0) if isinstance(pid, int) else None])(int(sys.argv[1]), int(sys.argv[2]) if len(sys.argv) > 2 else None) if __name__ == '__main__' else None"""
        subprocess.Popen(
            [sys.executable, "-c", commandstring, str(pid), str(command)],
            **invisibledict,
        )  # Send Ctrl-C
    else:
        os.kill(pid, signal.SIGINT)


class dequeslice(deque):
    def __getitem__(self, index):
        if isinstance(index, slice):
            return self.__class__(
                itertools.islice(self, index.start, index.stop, index.step)
            )
        return deque.__getitem__(self, index)


class CmdInteractive:
    def __init__(
        self,
        print_stdout=True,
        print_stderr=True,
        limit_stdout=None,
        limit_stderr=None,
        limit_stdin=None,
        convert_to_83=True,
        exitcommand="▓▓▓▓▓▓▓▓▓▓▓",
        getcomspec=None,
        newline="\r\n",
        wait_to_complete=0.1,
        **kwargs,
    ):
        r"""
        A class for interacting with a command-line shell in a subprocess.

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

        Attributes:
            (Attributes of the class and their descriptions.)

        Methods:
            (Methods of the class and their descriptions.)
        """
        self.wait_to_complete = wait_to_complete
        self.getcomspec = getcomspec
        separate_stdout_stderr_with_list = True
        invisible = True
        self.newline = newline
        self.convert_to_83 = convert_to_83
        self.cmd = get_comspec(gcmd=self.getcomspec, convert_to_83=self.convert_to_83)

        self.exitcommand = exitcommand
        self.invisible = invisible

        self.kwargs = kwargs.copy()
        self.separate_stdout_with_list = separate_stdout_stderr_with_list
        self.separate_stderr_with_list = separate_stdout_stderr_with_list
        self.lockobject = threading.Lock()
        self.limit_stdout = limit_stdout
        self.limit_stderr = limit_stderr
        self.limit_stdin = limit_stdin
        self.separate_stdout_stderr_with_list = separate_stdout_stderr_with_list
        if self.separate_stdout_with_list:
            self.stdout = dequeslice([[]], maxlen=limit_stdout)
        else:
            self.stdout = dequeslice([], maxlen=limit_stdout)
        if self.separate_stderr_with_list:
            self.stderr = dequeslice([[]], maxlen=limit_stderr)
        else:
            self.stderr = dequeslice([], maxlen=limit_stderr)

        self.stdin = dequeslice(self.cmd, maxlen=limit_stdin)
        self.print_stdout = print_stdout
        self.print_stderr = print_stderr
        self.p = self._start_subproc(**kwargs)
        self._t_stdout = threading.Thread(target=self._read_stdout)
        self._t_stderr = threading.Thread(target=self._read_stderr)
        self._t_stdout.start()
        self._t_stderr.start()

    def _start_subproc(self, **kwargs):
        kwargs.update(
            {
                "stdout": subprocess.PIPE,
                "stdin": subprocess.PIPE,
                "stderr": subprocess.PIPE,
                "bufsize": 0,
            }
        )
        kwargs.update(invisibledict)
        print(" ".join(self.cmd))
        return subprocess.Popen(" ".join(self.cmd), **kwargs)

    def _close_pipes(self, pipe):
        try:
            getattr(self.p, pipe).close()
        except Exception as e:
            sys.stderr.write(f"{e}\n")

    def _close_proc(self):
        try:
            self.p.terminate()
        except Exception as e:
            sys.stderr.write(f"{e}\n")

    def tkill(self):
        if iswindows:
            invi = invisibledict.copy()
            invi.update(
                {
                    "creationflags": DETACHED_PROCESS | CREATE_NEW_PROCESS_GROUP,
                }
            )
            _ = subprocess.Popen(f"taskkill /F /PID {self.p.pid} /T", **invi)

    def kill_proc(self, sleep_after_pipes=1, sleep_after_proc=1, shellkill=True):
        if iswindows:
            if shellkill and self.isalive():
                self.tkill()
        killthread(self._t_stdout)
        killthread(self._t_stderr)
        close_stdoutpipe = threading.Thread(target=lambda: self._close_pipes("stdout"))
        close_stderrpipe = threading.Thread(target=lambda: self._close_pipes("stdin"))
        close_stdinpipe = threading.Thread(target=lambda: self._close_pipes("stderr"))
        close_stdoutpipe.start()
        close_stderrpipe.start()
        close_stdinpipe.start()
        sleep(sleep_after_pipes)
        close_proc = threading.Thread(target=self._close_proc)
        close_proc.start()
        sleep(sleep_after_proc)
        if iswindows:
            self.tkill()
        for thr in [close_stdoutpipe, close_stderrpipe, close_stdinpipe, close_proc]:
            try:
                killthread(thr)
            except Exception as e:
                sys.stderr.write(f"{e}\n")
                sys.stderr.flush()

    def _read_stdout(self):
        for l in iter(self.p.stdout.readline, b""):
            try:
                if self.separate_stdout_with_list:
                    self.stdout[-1].append(l)
                else:
                    self.stdout.append(l)
                if self.print_stdout:
                    sys.stdout.write(f'{l.decode("utf-8", "backslashreplace")}')
                    sys.stdout.flush()

            except Exception:
                break

    def _read_stderr(self):
        for l in iter(self.p.stderr.readline, b""):
            try:
                if self.separate_stderr_with_list:
                    self.stderr[-1].append(l)
                else:
                    self.stderr.append(l)
                if self.print_stderr:
                    sys.stderr.write(f'{l.decode("utf-8", "backslashreplace")}')
                    sys.stderr.flush()
            except Exception:
                break

    def disable_stderr_print(self):
        self.print_stderr = False
        return self

    def disable_stdout_print(self):
        self.print_stdout = False
        return self

    def enable_stderr_print(self):
        self.print_stderr = True
        return self

    def enable_stdout_print(self):
        self.print_stdout = True
        return self

    def __call__(self, cmd, **kwargs):
        r"""
        Execute a command in the subprocess and return its stdout and stderr.

        Args:
            cmd (str or bytes): The command to execute in the subprocess.
            **kwargs: Additional keyword arguments for the command execution.

        Returns:
            list: A list containing two elements:
                - The stdout output as a list of lines (bytes).
                - The stderr output as a list of lines (bytes).

        Raises:
            Exception: If there is an error during command execution.
            KeyboardInterrupt: If the user interrupts the command execution.

        Note:
            - The 'exitcommand' specified in the class constructor is automatically added to the command.
            - If the subprocess is not alive, it will be restarted before executing the command.

        Example:
            cmd_interactive = CmdInteractive()
            stdout, stderr = cmd_interactive("dir /B", convert_to_83=True)
        """
        wait_to_complete = kwargs.get("wait_to_complete", self.wait_to_complete)
        if "wait_to_complete" in kwargs:
            del kwargs["wait_to_complete"]
        convert_to_83 = kwargs.get("convert_to_83", self.convert_to_83)
        if convert_to_83:
            cmd = convert_path_to_short(cmd)
        try:
            so, se = self.write(cmd, wait_to_complete=wait_to_complete, **kwargs)
            return [so, se]
        except Exception as e:
            sys.stderr.write(f"{e}\n")
            sys.stderr.flush()
            return [[], []]
        except KeyboardInterrupt:
            self.tkill()
            return [[], []]

    def write(self, cmd, wait_to_complete=0.1, **kwargs):
        exitcommand = kwargs.get("exitcommand", self.exitcommand)
        exitcommandx = f"{self.newline}echo {exitcommand}{self.newline}"
        if isinstance(cmd, str):
            cmd = cmd + exitcommandx
            cmd = cmd.encode()
        else:
            cmd = cmd + exitcommandx.encode()
        finalcommand = exitcommand.encode()
        try:
            if not self.isalive():
                self.kill_proc()
                self.__init__(
                    print_stdout=self.print_stdout,
                    print_stderr=self.print_stderr,
                    limit_stdout=self.limit_stdout,
                    limit_stderr=self.limit_stderr,
                    limit_stdin=self.limit_stdin,
                    convert_to_83=self.convert_to_83,
                    exitcommand=self.exitcommand,
                    newline=self.newline,
                    getcomspec=self.getcomspec,
                    **self.kwargs,
                )
            self.lockobject.acquire()

            stderrlist = []
            stdoutlist = []
            self.stdout.append(stdoutlist)
            self.stderr.append(stderrlist)
            self.p.stdin.write(cmd)
            try:
                self.p.stdin.flush()
            except OSError as e:
                sys.stderr.write("Connection broken")
                raise e
            self.stdin.append(cmd)

        finally:
            try:
                self.lockobject.release()
            except Exception:
                pass

        if wait_to_complete:
            while True:
                if finalcommand not in b"".join(
                    stdoutlist
                ) and finalcommand not in b"".join(stderrlist):
                    sleep(wait_to_complete)
                else:
                    break
            stdoutl = b""
            stderrl = b""
            if stdoutlist:
                stdoutl = b"".join(stdoutlist)
            if stderrlist:
                stderrl = b"".join(stderrlist)
            return self._format_stdout_stderr(
                stdoutl=stdoutl, stderrl=stderrl, finalcommand=finalcommand
            )
        return [stdoutlist, stderrlist]

    def _format_stdout_stderr(self, **kwargs):
        stdoutl = kwargs.get("stdoutl")
        stderrl = kwargs.get("stderrl")
        finalcommand = kwargs.get("finalcommand")

        try:
            stdoutl = stdoutl.split(b">", maxsplit=1)[1]
            stdoutl = (
                b"\n".join(
                    b"".join(stdoutl.split(finalcommand)[:-1])
                    .strip()
                    .splitlines()[1:-1]
                )
                .strip()
                .splitlines(keepends=True)
            )
        except Exception:
            pass
        try:
            stderrl = stderrl.split(b">", maxsplit=1)[1]
            stderrl = (
                b"\n".join(
                    b"".join(stderrl.split(finalcommand)[:-1]).strip().splitlines()[:-1]
                )
                .strip()
                .splitlines(keepends=True)
            )
        except Exception:
            pass
        return [stdoutl, stderrl]

    def get_lock(self):
        self.lockobject.acquire()
        return self

    def release_lock(self):
        self.lockobject.release()
        return self

    def flush_stdout(self):
        try:
            self.get_lock()
            self.stdout.clear()
        finally:
            try:
                self.release_lock()
            except Exception:
                pass
        return self

    def flush_stderr(self):
        try:
            self.get_lock()
            self.stderr.clear()
        finally:
            try:
                self.release_lock()
            except Exception:
                pass
        return self

    def flush_stdin(self):
        try:
            self.get_lock()
            self.stdin.clear()
        finally:
            try:
                self.release_lock()
            except Exception:
                pass
        return self

    def flush_all_pipes(self):
        self.stdin.clear()
        self.stdout.clear()
        self.stderr.clear()

    def send_ctrl_c(self):
        send_ctrl_commands(self.p.pid, command=0)
        return self

    def send_ctrl_break(self):
        send_ctrl_commands(self.p.pid, command=1)
        return self

    def send_ctrl_close(self):
        send_ctrl_commands(self.p.pid, command=2)
        return self

    def send_ctrl_logoff(self):
        send_ctrl_commands(self.p.pid, command=3)
        return self

    def send_ctrl_shutdown(self):
        send_ctrl_commands(self.p.pid, command=4)
        return self

    def isalive(self):
        if iswindows:
            return is_process_alive(self.p.pid)
        else:
            raise NotImplementedError



class BashInteractive(CmdInteractive):
    def __init__(
        self,
        print_stdout=True,
        print_stderr=True,
        limit_stdout=None,
        limit_stderr=None,
        limit_stdin=None,
        convert_to_83=True,
        exitcommand="▓▓▓▓▓▓▓▓▓▓▓",
        newline="\n",
        getcomspec="bash.exe",
            wait_to_complete=0.1,

            **kwargs,
    ):
        r"""
        A class for interacting with the Bash shell in a subprocess.

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

        Attributes:
            folderpath (str): The path to the folder containing Bash-related executables.
            cygpath (str): The path to the cygpath executable.
            aptinstallfile (str): The path to the apt.sh installation script.

        Methods:
            (Methods of the class and their descriptions.)
        """
        self.folderpath = None
        super().__init__(
            print_stdout=print_stdout,
            print_stderr=print_stderr,
            limit_stdout=limit_stdout,
            limit_stderr=limit_stderr,
            limit_stdin=limit_stdin,
            convert_to_83=convert_to_83,
            exitcommand=exitcommand,
            newline=newline,
            getcomspec=getcomspec,
            wait_to_complete=wait_to_complete,
            **kwargs,
        )
        self.cygpath = get_short_path_name(
            os.path.normpath(os.path.join(self.folderpath, "cygpath.exe"))
        )
        if os.path.exists(
            aptpath := os.path.normpath(os.path.join(self.folderpath, "apt.sh"))
        ):
            self.aptinstallfile = aptpath
        else:
            self.aptinstallfile = None

    def _install_apt(self):
        aptinstallfile = os.path.normpath(os.path.join(self.folderpath, "apt.sh"))

        with open(aptinstallfile, mode="w", encoding="utf-8", newline="\n") as f:
            f.write(aptget)
        self.aptinstallfile = aptinstallfile

    def _start_subproc(self, **kwargs):
        bia = os.environ.copy()
        bia.update({"TERM": "xterm"})
        kwargs.update(
            {
                "stdout": subprocess.PIPE,
                "stdin": subprocess.PIPE,
                "stderr": subprocess.PIPE,
            }
        )
        startupinfo = subprocess.STARTUPINFO()
        startupinfo.dwFlags |= subprocess.STARTF_USESHOWWINDOW
        startupinfo.wShowWindow = subprocess.SW_HIDE
        creationflags = (
            subprocess.CREATE_NO_WINDOW
        )
        invisibledi = {
            "startupinfo": startupinfo,
            "creationflags": creationflags,
            "start_new_session": True,
        }
        splitpath = os.sep.join(self.cmd[0].split(os.sep)[:-1])
        self.folderpath = splitpath
        kwargs.update({"bufsize": 0, "shell": False, "env": bia, "cwd": splitpath})
        kwargs.update(invisibledi)

        return subprocess.Popen(self.cmd, **kwargs)

    def _format_stdout_stderr(self, **kwargs):
        stdoutl = kwargs.get("stdoutl")
        stderrl = kwargs.get("stderrl")
        finalcommand = kwargs.get("finalcommand")
        stdoutl = stdoutl.split(finalcommand)[0].strip().splitlines(keepends=True)
        stderrl = stderrl.split(finalcommand)[0].strip().splitlines(keepends=True)
        return stdoutl, stderrl

    @cache
    def path_to_cygwin(self, path):
        path = get_short_path_name(path)
        ecommand = f"{self.cygpath} -p {path}"
        return (
            subprocess.run(ecommand, capture_output=True, shell=True, **invisibledict)
            .stdout.strip(b"\n")
            .decode()
        )

    def __call__(self, cmd, **kwargs):
        r"""
        Execute a command in the Bash shell subprocess and return its stdout and stderr.

        Args:
            cmd (str or bytes): The command to execute in the Bash shell subprocess.
            **kwargs: Additional keyword arguments for the command execution.

        Returns:
            list: A list containing two elements:
                - The stdout output as a list of lines (bytes).
                - The stderr output as a list of lines (bytes).

        Raises:
            Exception: If there is an error during command execution.
            KeyboardInterrupt: If the user interrupts the command execution.

        Note:
            - The 'exitcommand' specified in the class constructor is automatically added to the command.
            - If the subprocess is not alive, it will be restarted before executing the command.
            - Converts paths to 8.3 and afterwards to the Cygwin format if 'convert_to_83' is True.

        Example:
            bash_interactive = BashInteractive()
            stdout, stderr = bash_interactive("ls -l", convert_to_83=True)
        """
        wait_to_complete = kwargs.get("wait_to_complete", self.wait_to_complete)
        if "wait_to_complete" in kwargs:
            del kwargs["wait_to_complete"]
        cmd2 = []
        convert_to_83 = kwargs.get("convert_to_83", self.convert_to_83)
        if convert_to_83:
            cmd = convert_path_to_short(cmd, returnstring=False)
            for cc in cmd:
                if os.path.exists(cc):
                    if ":\\" in cc:
                        cc = self.path_to_cygwin(cc)
                cmd2.append(cc)
            cmd = " ".join(cmd2)
        try:
            so, se = self.write(cmd, wait_to_complete=wait_to_complete, **kwargs)
            return [so, se]
        except Exception as e:
            sys.stderr.write(f"{e}\n")
            sys.stderr.flush()
            return [[], []]
        except KeyboardInterrupt:
            self.tkill()
            return [[], []]

    def install(self, package, nodeps=False):
        r"""
        Install package(s)."""
        self._execute_apt(f"apt install {package}" + ("--nodeps" if nodeps else ""))

    def remove(self, package, nodeps=False):
        r"""
        Remove package(s) from the system."""
        self._execute_apt(f"apt remove {package}" + ("--nodeps" if nodeps else ""))

    def update(
        self,
    ):
        r"""
        Download a fresh copy of the master package list (setup.ini) from the
        server defined in setup.rc."""
        self._execute_apt(f"apt update")

    def download(self, package, nodeps=False):
        r"""
        Retrieve package(s) from the server, but do not install/upgrade anything."""
        self._execute_apt(f"apt download {package}" + ("--nodeps" if nodeps else ""))

    def show(self, package, nodeps=False):
        r"""
        Display information on given package(s)."""
        self._execute_apt(f"apt show {package}" + ("--nodeps" if nodeps else ""))

    def depends(self, package, nodeps=False):
        r"""
        Produce a dependency tree for a package."""
        self._execute_apt(f"apt depends {package}" + ("--nodeps" if nodeps else ""))

    def rdepends(self, package, nodeps=False):
        r"""
        Produce a tree of packages that depend on the named package."""
        self._execute_apt(f"apt rdepends {package}" + ("--nodeps" if nodeps else ""))

    def list(self, regexp):
        r"""
        Search each locally-installed package for names that match regexp. If no
        package names are provided in the command line, all installed packages will
        be queried."""
        self._execute_apt(f"apt list {regexp}")

    def listall(self, regexp):
        r"""
        This will search each package in the master package list (setup.ini) for
        names that match regexp."""
        self._execute_apt(f"apt listall {regexp}")

    def category(self, category):
        r"""
        Display all packages that are members of a named category."""
        self._execute_apt(f"apt category {category}")

    def listfiles(self, packages):
        r"""
        List all files owned by a given package. Multiple packa,ges can be specified
        on the command line."""
        self._execute_apt(f"apt listfiles {packages}")

    def search(self, path):
        r"""
        Search for downloaded packages that own the specified file(s). The path can
        be relative or absolute, and one or more files can be specified."""
        self._execute_apt(f"apt search {path}")

    def searchall(self, file):
        r"""
        Search cygwin.com to retrieve file information about packages. The provided
        target is considered to be a filename and searchall will return the
        package(s) which contain this file."""
        self._execute_apt(f"apt searchall {file}")

    def mirror(self, url):
        r"""
        Set the mirror; a full URL to a location where the database, packages, and
        signatures for this repository can be found. If no URL is provided, display
        current mirror."""
        self._execute_apt(f"apt mirror {url}")

    def cache(self, folder):
        r"""
        Set the package cache directory. If a file is not found in cache directory,
        it will be downloaded. Unix and Windows forms are accepted, as well as
        absolute or regular paths. If no directory is provided, display current
        cache."""
        self._execute_apt(f"apt cache {folder}")

    def _execute_apt(self, cmd):
        if not self.aptinstallfile:
            self._install_apt()

        os.system(
            rf'''start "" "{self.cmd[0]}" -c "{cmd} && read -n 1 -s -r -p 'Press any key to continue...'"'''
        )
