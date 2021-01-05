#!/usr/bin/env python3

import os
import re
import calendar
import datetime
import numpy as np
in_dir = "/userdir/logs/motion-planning"
# filter_str = ""
filter_str = "2021-01-04"
jlist_dof = 11

# Feb-22-16-37-05-2019_forehand_maximize-speed_CCSA_48.0h_M-14_N-5_x-max-1.0_x-hit-0.5_maxvel-1_minjerk-0.0005_delta-1.7e-04_eqthre-1.0e-08_ftol-1.0e-15_xtol-1.0e-10_interval-20_sp-0-0-0-100-50_root-joint_modify-ec
def parse_dirname(dir_name):
    date_str = dir_name.split('_')[0]
    try:
        dt = datetime.datetime.strptime(date_str, '%b-%d-%H-%M-%S-%Y')
    except ValueError as e:
        dt = datetime.datetime.strptime(date_str, '%b--%d-%H-%M-%S-%Y')
    motion_str = dir_name.split('_')[1]
    target_str = dir_name.split('_')[2]
    alg_str = dir_name.split('_')[3]
    calctime_str = dir_name.split('_')[4]
    id_max_str = dir_name.split('_')[5].split('-')[1]
    recursive_order_str = dir_name.split('_')[6].split('-')[1]
    x_max_str = dir_name.split('_')[7].split('-')[2]
    x_hit_str = dir_name.split('_')[8].split('-')[2]
    maxvel_str = dir_name.split('_')[9].split('-')[1]
    minjerk_str = dir_name.split('_')[10].split('-')[1]
    delta_str = dir_name.split('_')[11].split('-')[1]
    eqthre_str = dir_name.split('_')[12].split('-')[1]
    ftol_str = dir_name.split('_')[13].split('-')[1]
    xtol_str = dir_name.split('_')[14].split('-')[1]
    interval_num_str = dir_name.split('_')[15].split('-')[1]
    sp_str = dir_name.split('_')[16].split('-')[1] # 5つあるけど最初のだけ考えている
    return { \
        "motion": motion_str, \
        "target": target_str, \
        "alg": alg_str, \
        "calctime": calctime_str, \
        "id_max": id_max_str, \
        "recursive-order": recursive_order_str, \
        "x_max": x_max_str, \
        "x_hit": x_hit_str, \
        "dt": dt, \
        "interval_num": interval_num_str, \
        "sp": sp_str, \
    }
    # root_joint_str
    # limb_joint_str
    # modify_ec_str


def parse_logfiles(log_dir):
    with open(in_dir + "/" + log_dir + "/maximize-speed_p.dat", "r") as f:
        whole_lines = f.readlines()
        p_lines_num = len(whole_lines)
        if whole_lines:
            p = whole_lines[-1].rstrip('\n')
        else:
            p = ""
    with open(in_dir + "/" + log_dir + "/maximize-speed_obj.dat", "r") as f:
        whole_lines = f.readlines()
        obj_lines_num = len(whole_lines)
        obj_list = [0.0] #  dummy
        for line in whole_lines:
            obj_list.append(float(line))
    with open(in_dir + "/" + log_dir + "/maximize-speed_eq.dat", "r") as f:
        whole_lines = f.readlines()
        eq_lines_num = len(whole_lines)
        eq_list = [np.zeros(7)]  # dummy
        for line in whole_lines:
            eq = np.array([float(i) for i in line.rstrip('\n').split()])
            eq[eq < 0] = 0.0
            eq_list.append(eq)
    with open(in_dir + "/" + log_dir + "/maximize-speed_ieq.dat", "r") as f:
        whole_lines = f.readlines()
        ieq_lines_num = len(whole_lines)
        ieq_list = [np.zeros(1)]  # dummy
        for line in whole_lines:
            ieq = np.array([float(i) for i in line.rstrip('\n').split()])
            ieq[ieq < 0] = 0.0
            ieq_list.append(ieq)
    return { \
        "p_lines_num": p_lines_num, \
        "obj_lines_num": obj_lines_num, \
        "eq_lines_num": eq_lines_num, \
        "ieq_lines_num": ieq_lines_num, \
        "p": p, \
        "obj_list": obj_list, \
        "eq_list": eq_list, \
        "ieq_list": ieq_list, \
    }

# os.scandir() might be better for python3.5 or upper
dirs_sorted = sorted(os.listdir(in_dir), key=lambda s: parse_dirname(s)["dt"])

def decide_read_or_not_from_attr(attr):
    return not filter_str or (filter_str in "{}".format(attr["dt"]))

for dirname in dirs_sorted:
    attr = parse_dirname(dirname)
    if not decide_read_or_not_from_attr(attr):
        continue

    log = parse_logfiles(dirname)
    print_line = \
    "{} " \
    "{} " \
    "{} " \
    "{} " \
    "{} " \
    "{} " \
    "{} " \
    "{} " \
    "{:<5} " \
    "{:<5} " \
    "{:<5} " \
    "{:8.1f} " \
    "{:8.2f} " \
    "{:8.2f} " \
    "{:8.1f} " \
    "{:3.5f} " \
    "{:9.1f} ".format(\
          attr["motion"], \
          attr["dt"], \
          attr["x_max"], \
          attr["x_hit"], \
          attr["id_max"], \
          attr["recursive-order"], \
          attr["alg"], \
          attr["sp"], \
          log["obj_lines_num"], \
          log["eq_lines_num"], \
          log["ieq_lines_num"], \
          log["obj_list"][-1], \
          log["eq_list"][-1][0], \
          log["eq_list"][-1][3], \
          np.linalg.norm(np.array(log["eq_list"][-1])), \
          np.linalg.norm(log["ieq_list"][-1][((int(attr["id_max"]) - 1) * jlist_dof):((int(attr["id_max"]) - 1) * jlist_dof)+int(attr["interval_num"])+1]), \
          np.linalg.norm(np.array(log["ieq_list"][-1])), \
    )
    print(print_line)
    with open(in_dir + "/" + dirname + "/" + attr["motion"] + "-p-orig.l", "w") as f:
        f.write(";; " + print_line + "\n")
        f.write("(progn (setq *p-orig* ")
        f.write(log["p"])
        f.write(") (setq *x-max-of-p-orig* " + attr["x_max"] + "))")
