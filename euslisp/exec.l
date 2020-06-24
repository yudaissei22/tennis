#!/usr/bin/env roseus
(require "package://baseball/euslisp/is-choreonoid") ;; *is-choreonoid* is defined
(require "package://baseball/euslisp/experiment_utils.l")

(progn
  (experiment-init)
  (format t "experiment-init ~A~%" (send *robot* :angle-vector))
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
                              :step step :x-max t-max :choreonoid *is-choreonoid* :p *p* :lfoot-ik t :rfoot-ik t))
  (setq init-av (car *exp-jpos-deg1*))
  (send *robot* :angle-vector init-av)
  (send *robot* :head :neck-y :joint-angle 0)
  (setq init-av (send *robot* :angle-vector))
  (when (= (+ (length *jlist*) 6) (length *bspline*))
      ;; mm -> m in rtm-ros-robot-interface.l
      ;; *bspline*がすべて同じであることを仮定
      ;; irteusgl$ (send (car (send *robot* :links)) :worldcoords)
      ;; #<coordinates #X5517540  0.0 0.0 1032.5 / 0.0 0.0 0.0>
      ;; (with-append-root-joint (link-list-with-robot-6dof *robot* (list (cdr (send *robot* :links))) :joint-class 6dof-joint)
      ;; (send (send (car (send *robot* :links)) :joint) :joint-angle)))
      ;; #f(0.0 0.0 0.0 0.0 0.0 0.0)

      (with-append-root-joint
        (link-list-with-robot-6dof *robot* (list (cdr (send *robot* :links)))
                                :joint-class 6dof-joint)
        (let ((id 0))
          ;; 関節角設定
          (map float-vector #'deg2rad (mapcar
            #'(lambda (jl bs)
                (send jl :joint-angle
                      (send bs :calc *x-min* (subseq *p* id (setq id (+ id *id-max*))))))
            *jlist* *bspline*))
          ;; rootlinkの位置設定
          (send (send (car (send *robot* :links)) :joint) :joint-angle
             (setq xyzypr (coerce
               (mapcar
                 #'(lambda (bs)
                     (send bs :calc *x-min* (subseq *p* id (setq id (+ id *id-max*)))))
                 (subseq *bspline* (length *jlist*) (+ (length *jlist*) 6)))
               float-vector)))))
      ;; (setq *desired-init-pos* (send (car (send *robot* :links)) :worldpos))
      ;; (setq *desired-init-rpy* ((coerce (nth 0 (rpy-angle (send (car (send *robot* :links)) :worldrot))) float-vector)))

;      (print "init rootlink pos")
;      (send *ri* :set-base-pos
;            (print (send (car (send *robot* :links)) :worldpos)) 0)
;      ;; in hrpsys, rpy alignment is rpy and they are in rad
;      (print "init rootlink rpy")
;      (send *ri* :set-base-rpy
;            (print (coerce (nth 0 (rpy-angle (send (car (send *robot* :links)) :worldrot))) float-vector))
; 0)
      )
  (setq *initial-time* (if *is-choreonoid* 1000 10000))
  (experiment-angle-vector-sequence-full (list init-av) (list (car *exp-tm-ms1*)) (list (car *exp-rc1*)) (list (car *exp-zmp-wc1*)) (list (car *exp-optional1*))
                                         :initial-time *initial-time* :final-time 0 :log-fname "/tmp/init")
  (unless *is-choreonoid*
    (progn (print "Press Enter key after the robot stands on the ground.")
           (read-line)))
  (experiment-angle-vector-sequence-full (list (car *exp-jpos-deg1*)) (list (car *exp-tm-ms1*)) (list (car *exp-rc1*)) (list (car *exp-zmp-wc1*)) (list (car *exp-optional1*))
                                         :initial-time *initial-time* :final-time 0 :log-fname "/tmp/init")
  (swing-with-online-adjustment *exp-jpos-deg1* *exp-tm-ms1* *recursive-order* *id-max* *x-min* *default-x-hit* *x-max* *p* *exp-rc1* *exp-zmp-wc1* *exp-optional1* :initial-time *initial-time* :final-time 5000 :log-fname "/tmp/swing-with-online-adjustment")
)
