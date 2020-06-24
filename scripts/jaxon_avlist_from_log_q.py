#!/usr/bin/env python
# -*- coding: utf-8 -*-

import csv, sys, math
import os.path
import struct
import numpy as np

# Usage:
# ./jaxon_avlist_from_log_q.py [hrpsys-log-name] [avlist-name] [{sh_q, st_q, rh_q, ss_q}] [ref/enc]
# [hrpsys-log-name] is the name without extension.
# [ref/enc] is available in ss_q


jointnum = 33
urata_len = 16 # Number of the ExtraServoState in iob_shm
refq_index = 0 # encoder: 5, refq: 9 (hrpsys_trans_bridge/iob/iob_shm.cpp)

if len(sys.argv) < 4:
    print "Usage: ./jaxon_avlist_from_log_q.py [hrpsys-log-name] [avlist-name] [{sh_q, rh_q, st_q, ss_q}] [ref/enc (optional)]"
else:
    f = os.path.splitext(os.path.basename(sys.argv[1]))[0]
    ext_label = sys.argv[3]

    if ext_label == "sh_q":
        f_ext = f + ".sh_qOut"
    elif ext_label == "st_q":
        f_ext = f + ".st_q"
    elif ext_label == "rh_q":
        f_ext = f + ".RobotHardware0_q"
    else:
        f_ext = f + ".RobotHardware0_servoState"
        refq_index = 9
        if (len(sys.argv) == 5) and (sys.argv[4] == 'enc'):
            refq_index = 5

    fw = open(sys.argv[2], 'w')
    writer = csv.writer(fw, lineterminator=('\n'), delimiter=' ')

    with open(f_ext, 'r') as f:
        fw.write('(progn (setq *real-avlist-' + ext_label + '* (list ')
        reader = csv.reader(f, delimiter=' ', skipinitialspace = True)
        for row in reader:
            row = filter(lambda x: x != '', row)
            if refq_index == 0:
                row = map(float, row[1:(1+jointnum)]) # remove hand joint in choreonoid
                refq = np.rad2deg(np.array(row)).tolist()
            else:
                row = row[1:]
                refq = np.rad2deg([struct.unpack('f', struct.pack('i', int(x)))[0] for x in row[refq_index::(urata_len+1)]]).tolist()
            refq_str = map(str, refq)
            refq_str.insert(0, '#f(')
            refq_str.append (')')
            refq_str = ' '.join(refq_str)
            fw.write(refq_str)
            fw.write('\n')
        fw.write(')) nil)')
    fw.close()
