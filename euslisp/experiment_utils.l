#!/usr/bin/env roseus

(ros::roseus-add-msgs "ball_state_msgs")

(require "package://tennis/euslisp/is-choreonoid")
(comp::compile-file-if-src-newer (ros::resolve-ros-path "package://tennis/euslisp/nlopt_bspline_optimization.l") (ros::resolve-ros-path "package://tennis/euslisp/"))
(require "package://tennis/euslisp/nlopt_bspline_optimization.so")

(defun experiment-init ()
  ;; (jaxon-init)
  (jaxon_red-init)
  (send *ri* :start-grasp)
  (send *ri* :set-st-param :emergency-check-mode 0)
  (send *ri* :set-interpolation-mode :linear)
  (defvar *append-list* nil)
  (send *ri* :stop-auto-balancer)
  (send *ri* :stop-st) ;; 入っていないとダメなはず
  )



(defun send-pose (&optional (tm 10000))
  (send *ri* :angle-vector (send *robot* :angle-vector) tm)
  )

(if (probe-file "/home/yuda/logs/timestamp.txt")
    (let (l)
        (with-open-file (f "/home/yuda/logs/timestamp.txt" :direction :input)
            (while (setq l (read-line f nil nil))
                (setq *timestamp-str* l))))
    (setq *timestamp-str* ""))
(format t "*timestamp-str* is ~A~%" *timestamp-str*)

(defun experiment-loadpattern
  (sequence-fname &key (log-fname (concatenate string "/home/yuda/logs/" *timestamp-str* "loadpattern")) (reverse-fname nil #|"/home/yuda/logs/reverse"|#))
  "Sequence files consist of ***.pos, ***.zmp, ***.waist, ***.optionaldata.
   sequence-fname (***) should be full path.
   revese is the sequence file for returning to the initial pose."
  (send *ri* :start-log)
  (send *ri* :load-pattern sequence-fname 1)
  (send *ri* :wait-interpolation-seq)
  (when reverse-fname
    (send *ri* :load-pattern reverse-fname 3)
    (send *ri* :wait-interpolation-seq)
    )
  (send *ri* :save-log log-fname)
  )

(defun swing-with-online-adjustment (jpos tm bsorder id-max x-min x-hit x-max p rc zmp optional
                                          &key (initial-time 10000) (final-time 5000) (log-fname (concatenate string "/home/yuda/logs/" *timestamp-str* "swing-with-online-adjustment")))
  (format t "set base pos; start sec~%")
  (send *ri* :set-base-pos (send (car *exp-rc1*) :worldpos) initial-time)
  (send *ri* :set-base-rpy (coerce (reverse (nth 0 (rpy-angle (send (car *exp-rc1*) :worldrot)))) float-vector) initial-time)
  (send *ri* :angle-vector (send *robot* :angle-vector (car jpos)) initial-time) ;; initial-timeまでに最初の姿勢にする
  (send *ri* :wait-interpolation)
  (unless *is-choreonoid*
    (progn (print "Press Enter key when you want to start-st.")
           (read-line)))

  (format t "start-st begin~%")
  ;; (send *ri* :start-st)
  (format t "start-st done~%")
  (print "initialization process ended.")
  (setq *vision-initialized* nil)
  (print "TODO: virtual jointなしの場合も作る")
  ;; タスク達成時刻のロボットの状態を再現
  (format t "p ~A~%" p)
  (format t "angle-vector before ~A~%" (send *robot* :angle-vector))
  (send *robot* :move-to (make-coords) :world)
  (with-append-root-joint
    (link-list-with-robot-6dof *robot* (list (cdr (send *robot* :links)))
                            :joint-class 6dof-joint)
    (let ((id 0))
      ;; 関節角設定
      (map float-vector #'deg2rad (mapcar
        #'(lambda (jl bs)
            (send jl :joint-angle
                  (send bs :calc x-hit (subseq p id (setq id (+ id id-max))))))
        *jlist* *bspline*))
      ;; rootlinkの位置設定
      (send (send (car (send *robot* :links)) :joint) :joint-angle
         (setq xyzrpy (coerce
           (mapcar
             #'(lambda (bs)
                 (send bs :calc x-hit (subseq p id (setq id (+ id id-max)))))
             (subseq *bspline* (length *jlist*) (+ (length *jlist*) 6)))
           float-vector)))))
  (format t "angle-vector after ~A~%" (send *robot* :angle-vector))
;;  (print "hit virtual joint joint-angle pos")
;;  (print (scale 0.001 (subseq xyzrpy 0 3)))
;;  (print "hit virtual joint joint-angle rpy")
;;  (print (map float-vector #'deg2rad (subseq xyzrpy 3 6)))
;;  (print "expected hit rootlink pos/calc-ed rootlink pos")
;;  (print (scale 0.001 (send (car (send *robot* :links)) :worldpos)))
;;  (setq R (rpy-matrix (deg2rad (elt xyzrpy 5)) (deg2rad (elt xyzrpy 4)) (deg2rad (elt xyzrpy 3))))
;;  (print (v+ (scale 0.001 (subseq xyzrpy 0 3)) (transform R #f(0 0 1.0325))))
;;  (print "hit virtual joint joint-angle pos")
;;  (print (scale 0.001 (subseq xyzrpy 0 3)))
;;  (print "R * m_rootlink_6dof_offset")
;;  (print (transform R #f(0 0 1.0325)))
;;  (print "expected hit rootlink rpy/calc-ed rootlink rpy")
;;  (print (coerce (reverse (nth 0 (rpy-angle (send (car (send *robot* :links)) :worldrot)))) float-vector))
;;  (print (coerce (reverse (nth 0 (rpy-angle (m* R (rpy-matrix 0 0 0))))) float-vector))
;;  (print "R")
;;  (print R)
;;  (send *irtviewer* :draw-objects)
;;  (print "rarm ee pos")
;;  (print (scale 0.001 (send (send *robot* :rarm :end-coords) :worldpos)))
;;  (print "rarm ee rpy")
;;  (print (print (coerce (reverse (nth 0 (rpy-angle (send (send *robot* :rarm :end-coords) :worldrot)))) float-vector)))
;;  (send (send *robot* :rarm :end-coords) :draw-on :flush t :size 300)
  (print "expected hit target pos")
  (setq expected-hit-pos (print (scale 0.001 (send *sweet-spot* :worldpos))))
  (print "expected hit target rpy")
  (setq expected-hit-rpy (print (coerce (reverse (nth 0 (rpy-angle (send *sweet-spot* :worldrot)))) float-vector)))
  (send *sweet-spot* :draw-on :flush t :size 300)
  (send *ri* :start-log)
  (print "hrpsys log started")

  ;; jaxonが大体原点にいることを仮定
  (ros::subscribe "/pointgrey/estimated_ball_state" ball_state_msgs::PosAndVelWithCovarianceStamped
                  #'(lambda (msg)
                      (progn
                        ;; 原点を通る直線上に打点があると仮定
                        (let* ((x (send msg :point :x)) ;; ボールの位置
                               (y (send msg :point :y))
                               (z (send msg :point :z))
                               (vx (send msg :velocity :x)) ;; ボールの速度
                               (vy (send msg :velocity :y))
                               (vz (send msg :velocity :z))
                               (px (elt expected-hit-pos 0))
                               (py (elt expected-hit-pos 1))
                               (pz (elt expected-hit-pos 2))
                               (print "warning: These values are manually set because somehow hrpsys and eus expected-hit-pos differ.")
                               ;; (px 1.05138)
                               ;; (py 0.350219)
                               ;; (pz 0.561539)
                               (k-hit (/ (- (* vy x) (* vx y)) (- (* vy px) (* vx py)))) ;; TODO: avoid zero division
                               (hit-pos-x (* k-hit px))
                               (hit-pos-y (* k-hit py))
                               (ttc (/ (- hit-pos-y y) vy)) ;; this is equivalent to (/ (- hit-pos-y y) vy)
                               (var-trace 0.0))
                          (dotimes (i 6)
                            (setq var-trace (+ var-trace (elt (send msg :pos_and_vel_covariance) (+ (* i 6) i)))))
                          (format t "x: ~A vx: ~A y: ~A vy: ~A z: ~A vz: ~A px: ~A py: ~A pz: ~A k-hit: ~A ttc: ~A var-trace: ~A~%" x vx y vy z vz px py pz k-hit ttc var-trace)
                          (setq ttc-margin 0.4)
                          (when (and (not *vision-initialized*) (<= ttc (+ x-hit ttc-margin)) (<= 0 ttc) (<= var-trace 6.0))
                            (setq *vision-initialized* t)
                            (print "sending bspline init orbit")
                            (send *ri* :angle-vector-sequence-full-with-bspline jpos tm bsorder id-max x-min x-hit x-max p :root-coords rc :zmp zmp :optional optional)
                            (send *ri* :wait-interpolation-seq)
                            (print "sent bspline init orbit")
                            (unix:usleep (* final-time 1000))
                            (send *ri* :save-log log-fname))))))
  ;; TODO 一回きたらコールバックをやめる(exit以外の方法を考える)
  (while (and (not *vision-initialized*) (ros::ok)) (ros::spin-once) (ros::sleep))
  )


(defun send-angle-vector-sequence-full (jpos tm rc zmp optional &optional (initial-time 10000))
  (send *ri* :angle-vector (send *robot* :angle-vector (car jpos)) initial-time) ;; initial-timeまでに最初の姿勢にする
  (send *ri* :wait-interpolation)
  (send *ri* :angle-vector-sequence-full jpos tm :root-coords rc :zmp zmp :optional optional)
  )

(defun send-angle-vector-sequence-full-with-bspline (jpos tm bsorder id-max x-min x-hit x-max p rc zmp optional &optional (initial-time 10000))
  (send *ri* :angle-vector (send *robot* :angle-vector (car jpos)) initial-time) ;; initial-timeまでに最初の姿勢にする
  (send *ri* :wait-interpolation)
  (send *ri* :angle-vector-sequence-full-with-bspline jpos tm bsorder id-max x-min x-hit x-max p :root-coords rc :zmp zmp :optional optional)
  )


(defun send-angle-vector-sequence-full-no-wait (jpos tm rc zmp optional)
  (send *ri* :angle-vector-sequence-full jpos tm :root-coords rc :zmp zmp :optional optional)
  )




(defun experiment-angle-vector-sequence-full
  (jpos tm rc zmp optional
   &key (initial-time 10000) (final-time 5000) (log-fname (concatenate string "/home/yuda/logs/" *timestamp-str* "angle-vector-sequence-full")))

  ;;(send *ri* :set-interpolation-mode :linear)
  (send *ri* :start-log)
  (send-angle-vector-sequence-full jpos tm rc zmp optional initial-time)
  (send *ri* :wait-interpolation-seq)
  (unix:usleep (* final-time 1000))
  (send *ri* :save-log log-fname)
  ;;(send *ri* :set-interpolation-mode :hoffarbib)
  )

(defun experiment-angle-vector-sequence-full-with-bspline
  (jpos tm bsorder id-max x-min x-hit x-max p rc zmp optional
   &key (initial-time 10000) (final-time 5000) (log-fname (concatenate string "/home/yuda/logs/" *timestamp-str* "angle-vector-sequence-full-with-bspline")))

  ;;(send *ri* :set-interpolation-mode :linear)
  (send *ri* :start-log)
  (send-angle-vector-sequence-full-with-bspline jpos tm bsorder id-max x-min x-hit x-max p rc zmp optional initial-time)
  (send *ri* :wait-interpolation-seq)
  (unix:usleep (* final-time 1000))
  (send *ri* :save-log log-fname)
  ;;(send *ri* :set-interpolation-mode :hoffarbib)
  )


(defun make-sequence-in-advance
  (jpos-deg tm-ms rc zmp-wc optional
   jpos-rad tm-s pos rpy zmp-rc robot-wc cog-wc
            &key
            (step 0.1)
            (x-max *x-max*)
            (choreonoid nil)
            (modified-p-path nil);;"package://tennis/config/modified-p/modified-p_-70-0--50.l")
            (lfoot-ik nil)
            (rfoot-ik nil)
            (zmp-offset #f(0 0 0))
            (p nil)
            )
  (if p
      (setq *p* p)
    (setq *p* (copy-seq *p-orig*))
    )
  (when modified-p-path
    (load modified-p-path)
    (insert-modified-p *p* *modified-p*)
    )

  (send *robot* :move-to (make-coords) :world)
  (with-append-root-joint
   (link-list-with-robot-6dof *robot* (list (cdr (send *robot* :links)))
                              :joint-class 6dof-joint)
   (show-optimized-motion *p* :x-step step :x-max x-max :real-time nil :make-sequence t :choreonoid choreonoid :lfoot-ik lfoot-ik :rfoot-ik rfoot-ik :zmp-offset zmp-offset)
   )
  (when (= (length tm-ms) (length *bspline-avsf-tm-ms*))
    (replace jpos-deg (copy-object *bspline-avsf-jpos-deg*))
    (replace tm-ms (copy-object *bspline-avsf-tm-ms*))
    (replace rc (copy-object *bspline-avsf-rc*))
    (replace zmp-wc (copy-object *bspline-avsf-zmp-wc*))
    (replace optional (copy-object *bspline-avsf-optional*))
    (replace jpos-rad (copy-object *bspline-avsf-jpos-rad*))
    (replace tm-s (copy-object *bspline-avsf-tm-s*))
    (replace pos (copy-object *bspline-avsf-pos*))
    (replace rpy (copy-object *bspline-avsf-rpy*))
    (replace zmp-rc (copy-object *bspline-avsf-zmp-rc*))
    (replace robot-wc (copy-object *bspline-avsf-wc*))
    (replace cog-wc (copy-object *bspline-avsf-cog-wc*))
    )
  nil
  )

