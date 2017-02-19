# -*- coding: utf-8 -*-
import os, subprocess, fnmatch

dir = "Z:\\devel\\asr_object_database\\segmentable_objects"

for root, dirnames, filenames in os.walk(dir):
  for filename in fnmatch.filter(filenames, '*.iv'):
    filename = os.path.join(root, filename)
    command = ["ivvrml", "-2", filename, "-o", filename[:-3] + ".wrl"]
    print command
    subprocess.call(command)

