#!/usr/bin/env roseus
(setq *motion-choice* "forehand-volley-step")
(comp::compile-file-if-src-newer (ros::resolve-ros-path "package://tennis/euslisp/nlopt_bspline_optimization.l") (ros::resolve-ros-path "packege://tennis/euslisp/"))
(load "package://tennis/euslisp/nlopt_bspline_optimization.so")
;; when applying original file
;; (setq *p-orig* )
;; (setq *x-max-of-p-orig* 2.0)

(nlopt-init :x-max 2.0 :x-hit 1.0 :id-max 14 :recursive-order 5 :use-all-joint t :use-append-root-joint t :interval-num 20 :support-polygon-margin (list 50.0 50.0 0 100 50) :epsilon-c 30 :mu 0.3 :use-final-pose nil :default-switching-list nil :use-6dof-p t)
(nlopt-motion-optimize :dry-run t :x-max 2.0 :x-hit 1.0 :id-max 14 :recursive-order 5 :max-eval 0 :alg CCSA :delta (deg2rad 0.01) :eqthre 1e-8 :xtol 1e-10 :ftol 1e-15 :use-all-joint t :use-margin 0.5 :use-append-root-joint t :maxvel-weight 1 :minjerk-weight 5e-4 :modify-ec t :p *p* :interval-num 20 :title "dummy" :max-time 0 :file-path "/tmp/")

;; $ cat /userdir/logs/eq.dat && cat /userdir/logs/ieq.dat
