#!/usr/bin/python3

import rosbag
import os
import sys
import argparse


def rpl_msg_time_with_hdr(inbag,outbag):
  print('Processing input bagfile  : %s', inbag)
  print('Writing to output bagfile : %s', outbag)
  outbag = rosbag.Bag(outbag,'w')
  for topic, msg, t in rosbag.Bag(inbag,'r').read_messages():
    # This also replaces tf timestamps under the assumption
    # that all transforms in the message share the same timestamp
    if topic == "/tf" and msg.transforms:
      outbag.write(topic, msg, msg.transforms[0].header.stamp)
    else:
      outbag.write(topic, msg, msg.header.stamp if msg._has_header else t)
  print('Closing output bagfile and exit...')
  outbag.close();
  pass  # def rpl_msg_time_with_hdr(...)

if __name__ == "__main__":
  parser = argparse.ArgumentParser(
      description='Create a new bagfile from an existing one replacing the message time for the header time.')
  parser.add_argument('-o', metavar='OUTPUT_BAGFILE', required=True, help='output bagfile')
  parser.add_argument('-i', metavar='INPUT_BAGFILE', required=True, help='input bagfile')
  args = parser.parse_args()
  try:
    rpl_msg_time_with_hdr(args.i, args.o)
  except Exception as e:
    import traceback
    traceback.print_exc()
    pass
  pass  # if __name__ == "__main__"
