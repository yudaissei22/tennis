#!/usr/bin/env roseus
(setq *motion-choice* "backhand-volley-step")
(require "package://tennis/euslisp/nlopt_bspline_optimization.l")

(nlopt-init :x-max 2.0 :x-hit 1.0 :id-max 14 :recursive-order 5 :use-all-joint t :use-append-root-joint t :interval-num 20 :support-polygon-margin (list 50.0 50.0 0 100 50) :epsilon-c 30 :mu 0.3 :use-final-pose nil :default-switching-list nil :use-6dof-p t)

(setq v2 (instantiate float-vector 1000))

(motion-h *p-orig* v2)
(print v2)
