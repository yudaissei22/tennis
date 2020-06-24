#!/usr/bin/env python
# -*- coding: utf-8 -*-

import csv, sys, math
import os.path


fname = os.path.splitext(os.path.basename(sys.argv[1]))[0]
frname = fname + ".RobotHardware0_q"
shift_num = int(sys.argv[2])

fw = open(frname + "_shift" , 'w')
writer = csv.writer(fw, lineterminator=('\n'), delimiter=' ')

fr = open(frname, 'r')
reader = csv.reader(fr, delimiter=' ', skipinitialspace = True)
shift = []
loop = 0
for row in reader:
    row = filter(lambda x: x != '', row)
    shift.append(row)
    if loop >= shift_num:
        write_row = row
        write_row[0] = shift[0][0]
        write_row = ' '.join(write_row)
        fw.write(write_row)
        fw.write('\n')
        shift.pop(0)
    loop += 1
for i in range(shift_num):
    write_row = shift[i]
    write_row[1:] = shift[-1][1:]
    write_row = ' '.join(write_row)
    fw.write(write_row)
    fw.write('\n')
fw.close()
