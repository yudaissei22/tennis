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
    "50.1",
    "50.2",
    "50.3",
    "50.4",
    "50.5",
    "50.6",
    "50.7",
    "50.8",
    "50.9",
]

output_dir = "/userdir/logs/motion-planning"
os.makedirs(output_dir, exist_ok=True)

for alg in algs:
    for margin in margins:
        for motion in motions:
            os.makedirs("alg/" + motion + "/" + alg, exist_ok=True)
            with open("alg/" + motion + "/" + alg + "/" + margin + ".l", "w") as f:
                f.write("(comp::compile-file-if-src-newer \"qp-bspline-optimization.l\")\n")
                f.write("(setq *motion-choice* \"" + motion + "\")\n")
                f.write("(load \"qp-bspline-optimization.so\")\n")
                f.write("(qp-motion-optimize-util :x-takeoff 0.2 :x-land 0.7 :x-hit 0.8)\n")
                f.write("(format t \"(boundp *p-orig*) => ~A~%\" (boundp *p-orig*))\n")
                f.write("(comp::compile-file-if-src-newer \"nlopt_bspline_optimization.l\")\n")
                f.write("(load \"nlopt_bspline_optimization.so\")\n")
                f.write("(nlopt-init :x-max 1.4 :x-hit 0.7 :id-max 14 :recursive-order 5 :use-all-joint t :use-append-root-joint t :support-polygon-margin (list " + margin + " " + margin + " 0 100 50) :epsilon-c 30 :mu 0.3 :use-final-pose nil :default-switching-list nil :use-6dof-p t)\n")
                f.write("(nlopt-motion-optimize :x-max 1.4 :x-hit 0.7 :id-max 14 :recursive-order 5 :max-eval 100000000 :alg " + alg + " :delta (deg2rad 0.01) :eqthre 1e-8 :xtol 1e-10 :ftol 1e-15 :use-all-joint t :use-margin 0.5 :use-append-root-joint t :maxvel-weight 1 :minjerk-weight 5e-4 :modify-ec t :p *p* :interval-num 20 :title \"maximize-speed\" :max-time (* 2 24 60 60) :file-path \"" + output_dir + "\")\n")
                # proc = subprocess.Popen("roseus alg/" + motion + "/" + alg + "/" + margin + ".l", shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
                proc = subprocess.Popen(["roseus", "alg/" + motion + "/" + alg + "/" + margin + ".l"])
                # result = proc.communicate() # wait block
                # (stdout, stderr) = (result[0], result[1])
                # print(stdout)
                # print(stderr)
