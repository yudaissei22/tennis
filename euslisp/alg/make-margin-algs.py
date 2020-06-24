#!/usr/bin/env python3

margins = [
    "80.0",
    "80.1",
    "80.2",
    "80.3",
    "80.4",
    "80.5",
    "80.6",
    "80.7",
    "80.8",
    "80.9",
    "81.0",
]

for margin in margins:
    with open("CCSA_"+margin+"_"+margin+".l", "w") as f:
        f.write("(load \"nlopt_bspline_optimization.so\")")
        f.write("(nlopt-init :x-max 1.4 :x-hit 0.7 :id-max 14 :recursive-order 5 :use-all-joint t :use-append-root-joint t :support-polygon-margin (list {} {} 0 100 50) :epsilon-c 30 :mu 0.3 :use-final-pose nil :default-switching-list nil :use-6dof-p t)".format(margin, margin))
        f.write("(nlopt-motion-optimize :x-max 1.4 :x-hit 0.7 :id-max 14 :recursive-order 5 :max-eval 100000000 :alg " + "CCSA" + " :delta (deg2rad 0.01) :eqthre 1e-8 :xtol 1e-10 :ftol 1e-15 :use-all-joint t :use-margin 1 :use-append-root-joint t :maxvel-weight 1 :minjerk-weight 5e-2 :modify-ec t :p *p* :interval-num 20 :title \"maximize-speed\" :max-time (* 2 24 60 60) :file-path \"/tmp\")")
