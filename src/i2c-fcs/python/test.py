#!/usr/bin/python
import time
import smbus2
import tlc59116
import fcs
import re

i = 0
bus = smbus2.SMBus(1)
myfcs = fcs(bus)
while True:
  i = i + 1
  myfcs.write("%04d" % i)
  time.sleep(0.1)
  myfcs.setBrightness(i%256)


