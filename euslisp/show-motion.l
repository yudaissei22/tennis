#!/usr/bin/env roseus
(setq *motion-choice* "backhand-volley-step")

(require "package://tennis/euslisp/experiment_utils.l")
(require "package://tennis/euslisp/global-variables.l")

(setq *is-choreonoid* t)
(setq *id-max* 14)
(setq *recursive-order* 5)
(nlopt-init :x-max 2.0 :x-hit 1.0 :id-max 14 :recursive-order 5 :use-all-joint t :use-append-root-joint t :interval-num 20 :support-polygon-margin (list 50.0 50.0 0 100 50) :epsilon-c 30 :mu 0.3 :use-final-pose nil :default-switching-list nil :use-6dof-p t)


(let* ((t-max *x-max-of-p-orig*) (step 0.05)(seq-num (round (+ 1 (/ t-max step)))))
    (setq *exp-jpos-deg1* (make-list seq-num))
    (setq *exp-tm-ms1* (make-list seq-num))
    (setq *exp-rc1* (make-list seq-num))
    (setq *exp-zmp-wc1* (make-list seq-num))
    (setq *exp-optional1* (make-list seq-num))
    (setq *exp-jpos-rad1* (make-list seq-num))
    (setq *exp-tm-s1* (make-list seq-num))
    (setq *exp-pos1* (make-list seq-num))
    (setq *exp-rpy1* (make-list seq-num))
    (setq *exp-zmp-rc1* (make-list seq-num))
    (setq *exp-wc1* (make-list seq-num))
    (setq *exp-cog-wc1* (make-list seq-num))
    (make-sequence-in-advance *exp-jpos-deg1* *exp-tm-ms1* *exp-rc1* *exp-zmp-wc1* *exp-optional1* *exp-jpos-rad1* *exp-tm-s1* *exp-pos1* *exp-rpy1* *exp-zmp-rc1* *exp-wc1* *exp-cog-wc1*
		      :step step :x-max t-max :choreonoid *is-choreonoid* :p nil :lfoot-ik t :rfoot-ik t))
