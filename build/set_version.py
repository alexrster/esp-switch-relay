FILENAME_BUILDNO = 'VERSION'
FILENAME_VERSION_H = 'src/version.h'
version = '0.1.'

import datetime
from pathlib import Path

def get_active_branch_name():
  head_dir = Path(".") / ".git" / "HEAD"
  with head_dir.open("r") as f: content = f.read().splitlines()
  for line in content:
    if line[0:4] == "ref:":
      return line.partition("refs/heads/")[2]

def get_commit_ref():
  head_dir = Path(".") / ".git" / "ORIG_HEAD"
  with head_dir.open("r") as f: content = f.read().splitlines()
  for line in content:
    return line[:8]

build_no = 0
try:
  with open(FILENAME_BUILDNO) as f:
    build_no = int(f.readline()) + 1
except:
  print('Starting build number from 1..')
  build_no = 1

with open(FILENAME_BUILDNO, 'w+') as f:
  f.write(str(build_no))
  print('Build number: {}'.format(build_no))

hf = """
#ifndef BUILD_NUMBER
  #define BUILD_NUMBER "{}"
#endif
#ifndef __VERSION
  #define __VERSION "{}-{}-{}"
#endif
#ifndef VERSION_SHORT
  #define VERSION_SHORT "{}"
#endif
#ifndef VERSION
  #ifdef DEBUG
    #define VERSION __VERSION "-debug"
  #else
    #define VERSION __VERSION
  #endif
#endif
""".format(build_no, version+str(build_no), get_commit_ref(), get_active_branch_name(), version+str(build_no))

with open(FILENAME_VERSION_H, 'w+') as f:
  f.write(hf)