#!/usr/bin/env python3
import subprocess
import os

"""
algs = [
    "DIRECT",
    "G_DIRECT",
    "DIRECT_L",
    "G_DIRECT_L",
    "CRS",
    "STOGO",
    "ISRES",
    "CCSA",
    "SLSQP",
    "L_BFGS",
    "TN",
    "SL_VM",
    "COBYLA",
    "BOBYQA",
    "NEWUOA",
    "PRAXIS",
    "NelderMeadSimplex",
    "Sbplx",
]
"""
algs = [
    "CCSA",
]

motions = [
    "forehand-volley-step",
]

margins = [
    "50.0",
]

output_dir = "/userdir/logs/motion-planning"
os.makedirs(output_dir, exist_ok=True)
# use config/p-orig.l or not
config_p_value=True

x_max = "5.0"
x_hit = "3.0"
maxvel_weight = "1e-2"
minjerk_weight = "3e0"
x_step = "0.02"
id_max = "20"
recursive_order = "5"
use_margin = "1"
use_all_joint = "t"
use_append_root_joint = "t"
x_takeoff = "1.0"
x_land = "2.0"


for alg in algs:
    for margin in margins:
        for motion in motions:
            os.makedirs("alg/" + motion + "/" + alg, exist_ok=True)
            with open("alg/" + motion + "/" + alg + "/" + margin + ".l", "w") as f:
                f.write("(comp::compile-file-if-src-newer \"qp-bspline-optimization.l\")\n")
                if config_p_value:
                    f.write("(setq *motion-choice* \"" + motion + "\")\n")
                    f.write("(load \"qp-bspline-optimization.so\")\n")
                    f.write("(qp-motion-optimize "\
                            + " :x-max " + x_max \
                            + " :x-hit " + x_hit \
                            + " :maxvel-weight " + maxvel_weight \
                            + " :minjerk-weight " + minjerk_weight \
                            + " :x-step " + x_step \
                            + " :id-max " + id_max \
                            + " :recursive-order " + recursive_order \
                            + " :use-margin " + use_margin \
                            + " :use-all-joint " + use_all_joint \
                            + " :use-append-root-joint " + use_append_root_joint \
                            + ")\n")
                    f.write("(setq *x-max-of-p-orig* " + x_max + ")")
                    f.write("(setq *p-orig* (concatenate float-vector *ret* (float-vector " + x_takeoff + " " + x_land + " " + x_hit + ")))")
                    f.write("(with-open-file (f \"/userdir/logs/p-orig.l\" :direction :output :if-exists :new-version) (format f \"~a\" *p-orig*))")
                    f.write("(format t \"(boundp '*p-orig*) => ~A~%\" (boundp '*p-orig*))\n")
                f.write("(comp::compile-file-if-src-newer \"nlopt_bspline_optimization.l\")\n")
                f.write("(load \"nlopt_bspline_optimization.so\")\n")
                f.write("(nlopt-init :x-max " + x_max + " :x-hit " + x_hit + " :id-max " + id_max + " :recursive-order " + recursive_order +  " :use-all-joint t :use-append-root-joint t :support-polygon-margin (list " + margin + " " + margin + " 0 100 50) :epsilon-c 30 :mu 0.3 :use-final-pose nil :default-switching-list nil :use-6dof-p t)\n")
                f.write("(nlopt-motion-optimize :x-max " + x_max + " :x-hit " + x_hit + " :id-max " + id_max + " :recursive-order " + recursive_order + " :max-eval 10000000000 :alg " + alg + " :delta (deg2rad 0.01) :eqthre 1e-8 :xtol 1e-10 :ftol 1e-15 :use-all-joint t :use-margin 0.5 :use-append-root-joint t :maxvel-weight 1 :minjerk-weight 5e-4 :modify-ec t :p *p* :interval-num 20 :title \"maximize-speed\" :max-time (* 14 24 60 60) :file-path \"" + output_dir + "\")\n")
                proc = subprocess.Popen(["roseus", "alg/" + motion + "/" + alg + "/" + margin + ".l"], shell=False, stdout=None, stderr=None)
                # result = proc.communicate() # wait block
                # (stdout, stderr) = (result[0], result[1])
                # print(stdout)
                # print(stderr)
