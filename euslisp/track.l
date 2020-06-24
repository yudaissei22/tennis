#!/usr/bin/env roseus

(ros::roseus-add-msgs "ball_state_msgs")

(load "package://hrpsys_ros_bridge_tutorials/euslisp/jaxon-interface.l")
(load "../model/racket-model.l")
(load "fullbody-ik-jaxon.l")


(setq *real* nil)
(setq *target* (make-coords :pos (float-vector -359.901 -1015.541 820.884)
                            :rpy (mapcar #'rad2deg (list 1.461 0.592 -1.029))))
;;#<coordinates #X122d12b8  -353.488 -1026.209 812.513 / 2.307 0.97 0.161>
;;#<coordinates #X602d980  9.285 -761.511 780.014 / -1.029 0.592 1.461>


(defun update-robot-from-target (msg)
    (setq x (* (send msg :point :x) 1000))
    (setq y (* (send msg :point :y) 1000))
    (setq z (* (send msg :point :z) 1000))
    (setq vx (* (send msg :velocity :x) 1000))
    (setq vy (* (send msg :velocity :y) 1000))
    (setq vz (* (send msg :velocity :z) 1000))
    ;; x + vx * t = 0
    ;; y + vy * t = y'
    ;; z + vz * t - 9800 * t * t = y'
    (setq ttc (/ (- x) vx))
    ;; (when (< ttc 0) (return-from update-robot-from-target nil))
    (setq target-x (+ x (* vx ttc)))
    (setq target-y (+ y (* vy ttc)))
    (setq target-z (+ z (* vz ttc) (/ (* -9800 ttc ttc) 2)))
    (format t "ttc: ~A, target: ~A ~A ~A ~%" ttc target-x target-y target-z)
    (setq *target* (make-coords :pos (float-vector target-x target-y target-z)
                                :rpy (list 0 0 0)))
    (fullbody-ik-jaxon-with-target *target*)
    (send *racket* :move-coords (send *robot* :rarm :end-coords :copy-worldcoords) (send *racket* :get :gripper))
    (send *irtviewer* :draw-objects :flush nil)
    (send *target* :draw-on :flush t :color #f(0 1 0) :size 300))

(defun jaxon-with-racket (ik-callback)
    (ros::roseus "ball-track")
    (if *real* (jaxon-init))
    (jaxon)
    (setq *robot* *jaxon*)
    (objects (list *robot*))
    (send *robot* :angle-vector #f(-9.54233 -26.193 -40.7962 78.6206 -35.5597 26.135 33.8677 22.2323 -55.5383 78.2569 -21.1087 -20.8566 6.44576 18.0757 30.8128 22.4 0.0 47.3534 0.189209 -39.5928 -40.6674 -81.0774 -93.1095 -40.6221 12.5816 5.0 20.0 50.0 0.0 -100.0 45.0 0.0 0.0))
    (send *robot* :fix-leg-to-coords (make-coords :rpy (list (- (/ pi 2)) 0 0)))
    (setq *racket* (instance racket :init :move-origin nil))
    (send *racket* :move-coords (send *robot* :rarm :end-coords :copy-worldcoords) (send *racket* :get :gripper))
    (let* ((pr (send (send *robot* :rarm :end-coords) :parent))
           (tg (make-cascoords :coords (send (send *racket* :get :target) :copy-worldcoords))))
      (send pr :assoc tg)
      (send *robot* :put :racket-target tg)
      tg)

    (objects (list *robot* *racket*))
    (send *racket* :move-coords (send *robot* :rarm :end-coords :copy-worldcoords) (send *racket* :get :gripper))
    (send *robot* :head :angle-vector #f(55 0))
    (send *irtviewer* :draw-objects)
    (print (send *robot* :rarm :end-coords :copy-worldcoords))
    (print (send (send *racket* :get :target) :copy-worldcoords))
    (unix::sleep 5)
    (when *target*
      (fullbody-ik-jaxon-with-target *target*)
      (send *racket* :move-coords (send *robot* :rarm :end-coords :copy-worldcoords) (send *racket* :get :gripper))
      (send *irtviewer* :draw-objects :flush nil)
      (send *target* :draw-on :flush t :color #f(0 1 0) :size 300))
    (send *irtviewer* :draw-objects)
    ;; method call
    (defclass cb-class
      :super propertied-object
      :slots ())
    (defmethod cb-class
      (:init ()
             ;; (ros::subscribe "/pointgrey/estimated_ball_state" ball_state_msgs::PosAndVelWithCovarianceStamped #'send self :update-cb)
             (print "init")
      )
      (:update-cb (msg) (update-robot-from-target msg)))
    (setq m (instance cb-class :init))
    (while (ros::ok) (ros::spin-once) (ros::sleep))
    )

(jaxon-with-racket #'update-robot-from-target)
