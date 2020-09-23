#!/usr/bin/env roseus


;; (load "package://hrpsys_ros_bridge_tutorials/euslisp/jaxon-interface.l")
(load "package://hrpsys_ros_bridge_tutorials/euslisp/jaxon_red-interface.l")
(load "../model/racket-model.l")
(print "TODO pack")
(load "fullbody-ik-jaxon.l")


(setq *real* nil)
(setq *target* (make-coords :pos (float-vector 0 -800 1200)
                            :rpy (list 0 0 0)))

(defun update-target-from-cnt (cnt)
    (setq x 0)
    (setq y (+ -800 (* 100 (sin(/ (* pi 2 cnt) 30)))))
    (setq z (+ 1200 (* 200 (cos(/ (* pi 2 cnt) 30)))))
    (format t "~A ~A ~A ~%" x y z)
    ;; world座標系であってほしい
    (setq *target* (make-coords :pos (float-vector x y z)
                                :rpy (list 0 0 0))))

(defun ik-callback ()
    (when *target*
      (fullbody-ik-jaxon-with-target *target*)
      (send *racket* :move-coords (send *robot* :rarm :end-coords :copy-worldcoords) (send *racket* :get :gripper))
      (send *irtviewer* :draw-objects :flush nil)
      (send *target* :draw-on :flush t :color #f(0 1 0) :size 300)
      ))

(defun jaxon-with-racket (target_updater ik-callback)
    (ros::roseus "ball-track")
    ;; (if *real* (jaxon-init))
    ;; (jaxon)
    ;; (setq *robot* *jaxon*)
    (if *real* (jaxon_red-init))
    (jaxon_red)
    (setq *robot* *jaxon_red*)
    (objects (list *robot*))
    (send *robot* :angle-vector #f(-37.9024 -6.96155 -25.0467 31.111 -6.14905 6.94278 -7.88046 10.2854 -17.2764 32.1213 -14.9095 -10.3437 0.0 15.0 0.0 20.0 0.0 30.0 -30.0 -48.4355 5.0 -80.0 -30.0 0.0 0.0 -30.0 -30.0 40.0 -5.0 -80.0 0.0 0.0 0.0))
    (send *robot* :fix-leg-to-coords (make-coords :rpy (list (- (/ pi 2)) 0 0 )))
    (setq *racket* (instance racket :init :move-origin nil))
    (send *racket* :move-coords (send *robot* :rarm :end-coords :copy-worldcoords) (send *racket* :get :gripper))
    (let* ((pr (send (send *robot* :rarm :end-coords) :parent))
           (tg (make-cascoords :coords (send (send *racket* :get :target) :copy-worldcoords))))
      (send pr :assoc tg)
      (send *robot* :put :racket-target tg)
      tg) ;; TODO 返り値不要なので消す

    (objects (list *robot* *racket*))
    (send *racket* :move-coords (send *robot* :rarm :end-coords :copy-worldcoords) (send *racket* :get :gripper))
    (send *robot* :head :angle-vector #f(55 0))
    (send *irtviewer* :draw-objects)
    (when *target*
      (fullbody-ik-jaxon-with-target *target*))
    (ros::rate 30)
    (setq cnt 0)
    (while (ros::ok)
        (funcall target_updater cnt)
        (funcall ik-callback)
        (setq cnt (+ cnt 1))
        (ros::sleep))
    (ros::roseus "shutdown")
    (exit))

(jaxon-with-racket 'update-target-from-cnt 'ik-callback)
