#!/usr/bin/env roseus

(ros::roseus-add-msgs "ball_state_msgs")

(require "is-choreonoid")
(require "nlopt_bspline_optimization")

(defun experiment-init ()
  (jaxon-init)
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



(defun experiment-loadpattern
  (sequence-fname &key (log-fname "/userdir/logs/loadpattern") (reverse-fname nil #|"/userdir/logs/reverse"|#))
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
                                          &key (initial-time 10000) (final-time 5000) (log-fname "/userdir/logs/swing-with-online-adjustment"))
  (format t "set base pos; start sec~%")
  (send *ri* :set-base-pos (send (car *exp-rc1*) :worldpos) initial-time)
  (send *ri* :set-base-rpy (coerce (reverse (nth 0 (rpy-angle (send (car *exp-rc1*) :worldrot)))) float-vector) initial-time)
  (send *ri* :angle-vector (send *robot* :angle-vector (car jpos)) initial-time) ;; initial-timeまでに最初の姿勢にする
  (send *ri* :wait-interpolation)
  (unless *is-choreonoid*
    (progn (print "Press Enter key when you want to start-st.")
           (read-line)))

  (format t "start-st begin~%")
  (send *ri* :start-st)
  (format t "start-st done~%")
  (print "initialization process ended.")
  (setq *vision-initialized* nil)
  (print "TODO: virtual jointなしの場合も作る")
  ;; タスク達成時刻のロボットの状態を再現
  (send *robot* :reset-pose)
  (send *robot* :move-to (make-coords) :world)
  (format t "p ~A~%" p)
  (format t "angle-vector before ~A~%" (send *robot* :angle-vector))
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


(defun send-angle-vector-sequence-full-no-wait (jpos tm rc zmp optional)
  (send *ri* :angle-vector-sequence-full jpos tm :root-coords rc :zmp zmp :optional optional)
  )




(defun experiment-angle-vector-sequence-full
  (jpos tm rc zmp optional
   &key (initial-time 10000) (final-time 5000) (log-fname "/userdir/logs/angle-vector-sequence-full"))

  ;;(send *ri* :set-interpolation-mode :linear)
  (send *ri* :start-log)
  (send-angle-vector-sequence-full jpos tm rc zmp optional initial-time)
  (send *ri* :wait-interpolation-seq)
  (unix:usleep (* final-time 1000))
  (send *ri* :save-log log-fname)
  ;;(send *ri* :set-interpolation-mode :hoffarbib)
  )


(defun experiment-angle-vector-sequence-full-switch
  (jpos1 tm1 rc1 zmp1 optional1
   jpos2 tm2 rc2 zmp2 optional2
   &key (switch-time 0.0) (initial-time 10000) (log-fname "/userdir/logs/angle-vector-sequence-full-switch"))

  (let* ((step (* 1e-3 (car tm1))) ;; [s]
         (nthcdr-num (round (+ (/ switch-time step) 2 #|1|#))))
    (setq jpos2 (nthcdr nthcdr-num jpos2))
    (setq tm2 (nthcdr nthcdr-num tm2))
    (setq rc2 (nthcdr nthcdr-num rc2))
    (setq optional2 (nthcdr nthcdr-num optional2))
    (send *ri* :start-log)
    (let* ((timer (instance mtimer :init)))
      (send-angle-vector-sequence-full jpos1 tm1 rc1 zmp1 optional1 initial-time)
      (while (< (send timer :stop) (+ (* 1e-3 initial-time) switch-time))
        )
      (send-angle-vector-sequence-full-no-wait jpos2 tm2 rc2 zmp2 optional2)
      (send *ri* :wait-interpolation-seq)
      (send *ri* :save-log log-fname)
      )
    )
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




;; 特定の関節を一定の関節角度にする
(defun modify-joint-constant (jlist av &key (p *p*))
  (let* (link-id
         (modified-av-list
          (mapcar #'(lambda (x)
                      (coerce (make-list *id-max* :initial-element x) float-vector)
                      )
                  (coerce av cons))
          )
         )
    (print modified-av-list)
    (mapc
     #'(lambda (jl av-list)
         (setq link-id (position jl *jlist* :test #'equal))
         (replace *p* av-list
                  :start1 (* link-id *id-max*) :end1 (* link-id (+ *id-max* 1))
                  :start2 0 :end2 *id-max*
                  )
         )
     jlist modified-av-list)
    )
  (format t ";; (setq *p-orig* (copy-seq *p*))")
  nil
  )



(defun append-last-sequence
  (centroid-pos &key (append nil) (tm 1000) (optional #f(1 1 0 0 1 1 1 1)) (choreonoid t))
  (let* ((target-coords
          (list
           *lfoot*
           *rfoot*
           )))
    (with-move-target-link-list
     (mt ll *robot* '(:lleg :rleg))
     (send *robot*
           :fullbody-inverse-kinematics target-coords
           :root-link-virtual-joint-weight #f(1.0 1.0 1.0 0.1 0.1 0.1) ;; root-link has 6DOF
           :target-centroid-pos centroid-pos
           :centroid-thre 5
           :move-target mt
           :link-list ll
           :avoid-collision-distance 60
           :avoid-collision-null-gain 5.0
           :avoid-collision-joint-gain 0.8
           :additional-weight-list (list (list (send *robot* :head-neck-p :child-link) 0)
                                         (list (send *robot* :head-neck-y :child-link) 0))
           :rotation-axis (list t t)
           :debug-view nil) ;;:no-message) ;;t)
     ))
  (send *irtviewer* :draw-objects)

  (when append
    (let* ((rc (send (car (send *robot* :links)) :copy-worldcoords))
           (av (send *robot* :angle-vector)))
      (setq *exp-jpos-deg1* (append *exp-jpos-deg1* (list (concatenate float-vector av (if choreonoid #f(-80 80 -80 80) nil))) ))
      (setq *exp-tm-ms1* (append *exp-tm-ms1* (list tm) ))
      (setq *exp-rc1* (append *exp-rc1* (list rc) ))
      (setq *exp-zmp-wc1* (append *exp-zmp-wc1* (list centroid-pos) ))
      (setq *exp-optional1* (append *exp-optional1* (list optional)))
      (setq *exp-jpos-rad1* (append *exp-jpos-rad1* (list (map float-vector #'deg2rad (concatenate float-vector av (if choreonoid #f(-80 80 -80 80) nil)))) ))
      (setq *exp-tm-s1*  (append *exp-tm-s1* (list (* 1e-3 tm))))
      (setq *exp-pos1* (append *exp-pos1* (list (scale 1e-3 (send rc :worldpos))) ))
      (setq *exp-rpy1* (append *exp-rpy1* (list (reverse (nth 0 (rpy-angle (send rc :worldrot))))) ))
      (setq *exp-zmp-rc1* (append *exp-zmp-rc1* (list (scale 1e-3 (send rc :inverse-transform-vector centroid-pos))) ))
      (setq *exp-wc1* (append *exp-wc1* (list (send *robot* :copy-worldcoords)) ))
      (setq *exp-cog-wc1* (append *exp-cog-wc1* (list (send *robot* :centroid)) ))
      )
    )

  nil
  )

#|
;; (append-last-sequence (midpoint 0.35 (send *lfoot* :worldpos) (send *rfoot* :worldpos)) :tm 1000 :append nil)
(setq *append-list*
      (list
       (list (float-vector (aref (send *robot* :centroid) 0) (aref (send *robot* :centroid) 1) 0) 1000)
       ;; (list (car (last *exp-zmp-wc1*)) 1000)
       (list (midpoint 0.5 (send *lfoot* :worldpos) (send *rfoot* :worldpos)) 1000)
       )
      )
(dolist (als *append-list*) (append-last-sequence (nth 0 als) :tm (nth 1 als) :append nil))

;; (send *ri* :angle-vector-sequence-full *bspline-avsf-jpos-deg* *bspline-avsf-tm-ms* :root-coords *bspline-avsf-rc* :zmp *bspline-avsf-zmp-wc* :optional *bspline-avsf-optional*)

(experiment-angle-vector-sequence-full-switch *exp-jpos-deg2* *exp-tm-ms2* *exp-rc2* *exp-zmp-wc2* *exp-optional2* *exp-jpos-deg1* *exp-tm-ms1* *exp-rc1* *exp-zmp-wc1* *exp-optional1* :initial-time 10000 :log-fname "/userdir/logs/one-switch" :switch-time 6.1)

(modify-joint-constant (send-all (send *robot* :larm :links) :joint) (send *robot* :larm :angle-vector))



(let* ((time-scale 1.0))
  (setq *p* (copy-seq *p-orig*))
  (send *robot* :move-to (make-coords) :world)
  (with-append-root-joint
   (link-list-with-robot-6dof *robot* (list (cdr (send *robot* :links)))
                              :joint-class 6dof-joint)
   (experiment-online-trajectory-modification *exp-jpos-rad1* *exp-tm-s1* *exp-pos1* *exp-rpy1* *exp-zmp-rc1* *exp-optional1* (* *x-max-of-p-orig* time-scale) (* (eval *x-hit*) time-scale) (* (- (eval *x-hit*) (* 3.0 (car *exp-tm-s1*))) time-scale) :initial-time 2000 :final-time 5000 :test-target #f(0 0 -0.02 0 0 0) :real nil :debug? t :append *append-list* :lfoot-ik t :rfoot-ik t)
   (setq *p* (copy-seq *p-orig*))
   nil))

(experiment-angle-vector-sequence-full *exp-jpos-deg1* *exp-tm-ms1* *exp-rc1* *exp-zmp-wc1* *exp-optional1* :initial-time 10000 :final-time 5000 :log-fname "/userdir/logs/angle-vector-sequence-full")

(experiment-angle-vector-sequence-full (list (car *exp-jpos-deg1*)) (list (car *exp-tm-ms1*)) (list (car *exp-rc1*)) (list (car *exp-zmp-wc1*)) (list (car *exp-optional1*)) :initial-time 2000 :final-time 0 :log-fname "/userdir/logs/init")

(let* ((t-max *x-max-of-p-orig*) (step (/ *x-max-of-p-orig* 40.0)) (seq-num (round (+ 1 (/ t-max step)))))
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
  (make-sequence-in-advance
   *exp-jpos-deg1* *exp-tm-ms1* *exp-rc1* *exp-zmp-wc1* *exp-optional1*
   *exp-jpos-rad1* *exp-tm-s1* *exp-pos1* *exp-rpy1* *exp-zmp-rc1* *exp-wc1* *exp-cog-wc1*
   :step step :x-max t-max :choreonoid t :p *p* :lfoot-ik t :rfoot-ik t)
  )
|#

