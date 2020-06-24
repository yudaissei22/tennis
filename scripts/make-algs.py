#!/usr/bin/env python3

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

for alg in algs:
    with open("alg/"+alg+".l", "w") as f:
        f.write("(comp::compile-file-if-src-newer \"nlopt_bspline_optimization.l\")\n")
        f.write("(load \"nlopt_bspline_optimization.so\")\n")
        f.write("(nlopt-init :x-max 1.0 :x-hit 0.5 :id-max 14 :recursive-order 5 :use-all-joint t :use-append-root-joint t :support-polygon-margin (list 50 50 0 100 50) :epsilon-c 30 :mu 0.3 :use-final-pose nil :default-switching-list nil :use-6dof-p t)\n")
        f.write("(nlopt-motion-optimize :x-max 1.0 :x-hit 0.5 :id-max 14 :recursive-order 5 :max-eval 100000000 :alg " + alg + " :delta (deg2rad 0.01) :eqthre 1e-8 :xtol 1e-10 :ftol 1e-15 :use-all-joint t :use-margin 0.5 :use-append-root-joint t :maxvel-weight 1 :minjerk-weight 5e-4 :modify-ec t :p *p* :interval-num 20 :title \"maximize-speed\" :max-time (* 2 24 60 60) :file-path \"/tmp\")\n")
