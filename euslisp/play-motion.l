#!/usr/bin/env roseus
(setq *motion-choice* "forehand-volley-step")
;; (setq *motion-choice* "backhand-volley-step")

(setq *is-choreonoid* t)
(require "package://tennis/euslisp/is-choreonoid") ;; *is-choreonoid* is defined
(require "package://tennis/euslisp/experiment_utils.l")
(require "package://tennis/euslisp/global-variables.l")

(setq *id-max* 14)
(setq *recursive-order* 5)
(nlopt-init :x-max 2.0 :x-hit 1.0 :id-max 14 :recursive-order 5 :use-all-joint t :use-append-root-joint t :interval-num 20 :support-polygon-margin (list 50.0 50.0 0 100 50) :epsilon-c 30 :mu 0.3 :use-final-pose nil :default-switching-list nil :use-6dof-p t)


(progn
  (experiment-init)
  (format t "experiment-init ~A~%" (send *robot* :angle-vector))
  (let* ((t-max *x-max-of-p-orig*) (step 0.1)(seq-num (round (+ 1 (/ t-max step)))))
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
                              :step step :x-max t-max :choreonoid *is-choreonoid* :p nil :zmp-offset #f(0 0 0) :lfoot-ik nil :rfoot-ik nil))
  (setq init-av (car *exp-jpos-deg1*))
  (send *robot* :angle-vector init-av)
  (send *robot* :head :neck-y :joint-angle 0)
  ;; (send *robot* :move-centroid-on-foot :both '(:rleg :lleg))
  (setq init-av-cp (send *robot* :angle-vector))
  (show-timing-variables)
  (show-bspline-variables)
  (when (= (+ (length *jlist*) 6) (length *bspline*))
      ;; mm -> m in rtm-ros-robot-interface.l
      ;; *bspline*がすべて同じであることを仮定
      ;; irteusgl$ (send (car (send *robot* :links)) :worldcoords)
      ;; #<coordinates #X5517540  0.0 0.0 1032.5 / 0.0 0.0 0.0>
      ;; (send *robot* :move-to (make-coords) :world)
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

      (print "init rootlink pos")
      (send *ri* :set-base-pos
            (print (send (car (send *robot* :links)) :worldpos)) 0)
      ;; in hrpsys, rpy alignment is rpy and they are in rad
      (print "init rootlink rpy")
      (send *ri* :set-base-rpy
            (print (coerce (nth 0 (rpy-angle (send (car (send *robot* :links)) :worldrot))) float-vector)) 0)
      )
  (setq *initial-time* (if *is-choreonoid* 1000 10000))
  (format t "sleep for 1 sec~%")
  (pose1)
  (format t "pose1-av: ~A~%" (send *robot* :angle-vector))
  (format t "init-av-cp: ~A~%" init-av-cp)
  (unix::sleep 1)
  (experiment-angle-vector-sequence-full (list init-av-cp) (list (car *exp-tm-ms1*)) (list (car *exp-rc1*)) (list (car *exp-zmp-wc1*)) (list (car *exp-optional1*))
                                         :initial-time *initial-time* :final-time 0 :log-fname (concatenate string "/userdir/logs/" *timestamp-str* "init"))
  (unless *is-choreonoid*
    (progn (print "Press Enter key after the robot stands on the ground.")
           (read-line)))
  (experiment-angle-vector-sequence-full (list (car *exp-jpos-deg1*)) (list (car *exp-tm-ms1*)) (list (car *exp-rc1*)) (list (car *exp-zmp-wc1*)) (list (car *exp-optional1*))
                                         :initial-time *initial-time* :final-time 0 :log-fname (concatenate string "/userdir/logs/" *timestamp-str* "init"))
  (unless *is-choreonoid*
    (progn (print "Press Enter key when you want to start st.")
           (read-line)))
  (format t "start st ~%")
  (send *ri* :start-st)
  (unless *is-choreonoid*
    (progn (print "Press Enter key when you want to play the motion.")
           (read-line)))
  (format t "play the motion~%")
  (experiment-angle-vector-sequence-full-with-bspline *exp-jpos-deg1* *exp-tm-ms1* *recursive-order* *id-max* *x-min* *default-x-hit* *x-max* *p* *exp-rc1* *exp-zmp-wc1* *exp-optional1*
                                         :initial-time *initial-time* :final-time 0 :log-fname (concatenate string "/userdir/logs/" *timestamp-str* "play_motion"))
)
