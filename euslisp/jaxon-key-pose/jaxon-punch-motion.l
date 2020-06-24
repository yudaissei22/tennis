(require "package://tennis/euslisp/robot-interface.l")

(objects (list *robot* *punch-point*))

;; make coords of robot foots
(setq *lfoot* (make-coords :pos #f(-200 320 0) :rpy (list (deg2rad 60) 0 0)))
(setq *rfoot* (make-coords :pos #f(0 -320 0) :rpy (list (deg2rad 45) 0 0)))

;; punch motion http://www.i-l-fitness-jp.com/kakuto/streat.htm
(defun pose1 (&key (robot *robot*) (base-time 5000) (real nil))
  (send robot :angle-vector
        #f(
           0.0 0.0 -20.0 40.0 -20.0 0.0 ;; rleg_joint0~5 (<legs> crotch-y,r,p / knee-p / ankle-p, r)
           0.0 0.0 -20.0 40.0 -20.0 0.0 ;; lleg_joint0~5
           0 5 30 ;; chest_joint0~2 (<torso> waist-r,p,y)
           20 0.0 ;; head_joint0~1 (<head> neck-y,p)
           15.0 -50.0 -30.0 20.0 -110.0 0.0 0.0 0.0 ;; rarm_joint0~6 (<arms> collar-y shoulder-p,r,y elbow-p wrist-y,r,p)
           5.0 -30.0 30.0 10.0 -120.0 0.0 0.0 0.0 ;; larm_joint0~6
           ))

        ;; #f(
        ;;    3.20807 -20.7196 -24.2303 55.1402 -33.1479 20.7332 ;; rleg_joint0~5 (<legs> crotch-y,r,p / knee-p / ankle-p, r)
        ;;    4.14109 22.8213 -22.1177 52.6859 -30.9337 -22.8213  ;; lleg_joint0~5
        ;;    0 20 0 ;; chest_joint0~2 (<torso> waist-r,p,y)
        ;;    45 0 ;; head_joint0~1 (<head> neck-y,p)
        ;;    0 -61.2996 -22.099 3.48124 -136.099 -66.0525 -1.02826 78.5083 ;; rarm_joint0~6 (<arms> collar-y shoulder-p,r,y elbow-p wrist-y,r,p)
        ;;    8 0 80 0 -40 45 0 0;; larm_joint0~6
        ;;    ))

  ;; rotate robot in order to use waist
  (send robot :orient (deg2rad 10) :z :world)
  (send robot :locate (float-vector 0 0 -210) :world)


  ;; fullbody-inverse-kinematics
  (let* ((target-coords
          (list
           *lfoot*
           *rfoot*
           )))
    (with-move-target-link-list
     (mt ll robot '(:lleg :rleg))
     (send robot
           :fullbody-inverse-kinematics target-coords
           :root-link-virtual-joint-weight #f(0.1 0.5 0.1 0.1 0.1 0.1) ;; root-link has 6DOF
           :target-centroid-pos (midpoint 0.5 (send *lfoot* :worldpos) (send *rfoot* :worldpos))
           :centroid-thre 20
           :move-target mt
           :link-list ll
           :avoid-collision-distance 60
           :avoid-collision-null-gain 5.0
           :avoid-collision-joint-gain 0.8
           ;; :collision-avoidance-link-pair
           ;; (list
           ;;  (list (cadr (send robot :torso :links))
           ;;        (send robot :larm_link2_lk))
           ;;  (list (cadr (send robot :torso :links))
           ;;        (send robot :rarm_link2_lk))
           ;;  )
           :additional-weight-list (list (list (send robot :head-neck-p :child-link) 0)
                                         (list (send robot :head-neck-y :child-link) 0))
           :rotation-axis (list t t)
           :debug-view nil) ;;:no-message) ;;t)
     ))
  (send *punch-coords* :move-to (send robot :rarm :end-coords :copy-worldcoords) :world)
  (send *irtviewer* :draw-objects)

  (if real
      (progn
        (send *ri* :angle-vector (send robot :angle-vector) base-time)
        (send *ri* :wait-interpolation)
        )
    )

  (list :angle-vector (send robot :angle-vector)
        :root-coords (send (car (send robot :links)) :copy-worldcoords) ;; (send robot :copy-worldcoords)
        :worldcoords (send robot :copy-worldcoords)
        :time base-time)
  )




(defun pose2 (&key (robot *robot*) (base-time 5000) (real nil))
  (send robot :angle-vector
        ;; #f(
        ;;    3.20807 -20.7196 -24.2303 55.1402 -33.1479 20.7332 ;; rleg_joint0~5 (<legs> crotch-y,r,p / knee-p / ankle-p, r)
        ;;    4.14109 22.8213 -22.1177 52.6859 -30.9337 -22.8213  ;; lleg_joint0~5
        ;;    0 0 0 ;; chest_joint0~2 (<torso> waist-r,p,y)
        ;;    45 0 ;; head_joint0~1 (<head> neck-y,p)
        ;;    30 -60 -60 3.48124 -136.099 -66.0525 -1.02826 78.5083 ;; rarm_joint0~6 (<arms> collar-y shoulder-p,r,y elbow-p wrist-y,r,p)
        ;;    8 0 80 0 -40 45 0 0;; larm_joint0~6
        ;;    ))

        ;; angle-vector of pose1
        #f(
           31.9535 -11.0367 -11.0289 44.3874 -32.247 11.6587 46.5068 14.7349 -30.7081 40.4061 -8.77576 -13.8545
           -5.0 5.0 60.0 0.0 0.0
           10.0 -80.0 -30.0 80 #|40.0|# -60.0 60.0 0.0 0.0
           5.0 -30.0 40.0 20.0 -120.0 -25.0 0.0 0.0))

  ;; rotate robot in order to use waist
  (send robot :orient (deg2rad 55) :z :world)
  (send robot :locate (float-vector 0 50 -210) :world)

  (send *punch-coords* :locate #f(0 0 0) :world)
  (send *punch-coords* :orient (deg2rad 90) :z :world)
  (send *punch-coords* :rotate (deg2rad -90) :y :world)
  (send *punch-coords* :rotate (deg2rad 10) :x :world)
  (send *punch-coords* :locate #f(50 900 1240) :world)

  ;; fullbody-inverse-kinematics
  (let* ((target-coords
          (list
           *punch-coords*
           *lfoot*
           *rfoot*
           )))
    (with-move-target-link-list
     (mt ll robot '(:rarm :lleg :rleg))
     (send robot
           :fullbody-inverse-kinematics target-coords
           :root-link-virtual-joint-weight #f(0.1 0.1 0.1 0.1 0.1 0.1) ;; root-link has 6DOF
           :target-centroid-pos (midpoint 0.4 (send *lfoot* :worldpos) (send *rfoot* :worldpos))
           :centroid-thre 20
           :move-target mt
           :link-list ll
           :avoid-collision-distance 60
           :avoid-collision-null-gain 5.0
           :avoid-collision-joint-gain 0.8
           :collision-avoidance-link-pair
           (list
            (list (cadr (send robot :torso :links))
                  (send robot :larm_link2_lk))
            (list (cadr (send robot :torso :links))
                  (send robot :rarm_link2_lk))
            )
           :additional-weight-list (list (list (send robot :head-neck-p :child-link) 0)
                                         (list (send robot :head-neck-y :child-link) 0))
           :rotation-axis (list t t t)
           :debug-view nil) ;;:no-message) ;;t)
     ))

  (send *irtviewer* :draw-objects)

  (if real
      (progn
        (send *ri* :angle-vector (send robot :angle-vector) base-time)
        (send *ri* :wait-interpolation)
        )
    )

  (list :angle-vector (send robot :angle-vector)
        :root-coords (send (car (send robot :links)) :copy-worldcoords) ;; (send robot :copy-worldcoords)
        :worldcoords (send robot :copy-worldcoords)
        :time base-time)
  )


(defun pose3 (&key (robot *robot*) (base-time 5000) (real nil))
  (send robot :angle-vector
        #f(-7.30416 -18.6095 -10.6529 63.1169 -49.9798 20.6134 6.47035 13.7393 -52.5775 75.4908 -21.102 -11.2169
           -2.03975 14.8643 40.8586 0.0 0.0
           21.3688 -79.201 -28.8203 54.4843 -20 #|-53.717|# 43.0996 3.40249 30.0569
           5.0 0.0 50.0 10.0 -110.0 -25.0 0.0 -20.0)
        ;; #f(16.4438 -15.3323 -10.1833 53.4024 -41.1453 16.7237 30.449 15.8716 -39.9949 59.5911 -17.9601 -14.0254
        ;;    -5 8.57392 51.4752 5.0 0.0
        ;;    20.7845 -74.8289 -30 #|-22.1843|# 33.0465 -51.8258 59.2997 5.34388 18.3079
        ;;    5.0 -30.0 40.0 20.0 -120.0 -25.0 0.0 0.0)
        )

  ;; rotate robot in order to use waist
  (send robot :orient (deg2rad 40) :z :world)
  (send robot :locate (float-vector 0 -30 -240) :world)

  (send *punch-coords* :locate #f(0 0 0) :world)
  (send *punch-coords* :orient (deg2rad 90) :z :world)
  (send *punch-coords* :rotate (deg2rad -90) :y :world)
  (send *punch-coords* :rotate (deg2rad 10) :x :world)
  (send *punch-coords* :locate #f(50 1000 1300) :world)

  ;; fullbody-inverse-kinematics
  (let* ((target-coords
          (list
           *punch-coords*
           *lfoot*
           *rfoot*
           )))
    (with-move-target-link-list
     (mt ll robot '(:rarm :lleg :rleg))
     (send robot
           :fullbody-inverse-kinematics target-coords
           :root-link-virtual-joint-weight #f(0.1 0.1 0.1 0.1 0.1 0.1) ;; root-link has 6DOF
           :target-centroid-pos (midpoint 0.4 (send *lfoot* :worldpos) (send *rfoot* :worldpos))
           :centroid-thre 20
           :move-target mt
           :link-list ll
           :avoid-collision-distance 60
           :avoid-collision-null-gain 5.0
           :avoid-collision-joint-gain 0.8
           :collision-avoidance-link-pair
           (list
            (list (cadr (send robot :torso :links))
                  (send robot :larm_link2_lk))
            (list (cadr (send robot :torso :links))
                  (send robot :rarm_link2_lk))
            )
           :additional-weight-list (list (list (send robot :head-neck-p :child-link) 0)
                                         (list (send robot :head-neck-y :child-link) 0))
           :rotation-axis (list t t t)
           :debug-view nil) ;;:no-message) ;;t)
     ))

  (send *irtviewer* :draw-objects)

  (if real
      (progn
        (send *ri* :angle-vector (send robot :angle-vector) base-time)
        (send *ri* :wait-interpolation)
        )
    )

  (list :angle-vector (send robot :angle-vector)
        :root-coords (send (car (send robot :links)) :copy-worldcoords) ;; (send robot :copy-worldcoords)
        :worldcoords (send robot :copy-worldcoords)
        :time base-time)
  )



(defun load-punch-pose (&key (robot *robot*) (pose 0))
  (send (send robot :rarm :end-coords) :dissoc *punch-coords*)
  (let* ((robot-worldcoords nil))
    (cond ((eq pose 1) (pose1) (push (send robot :copy-worldcoords) robot-worldcoords))
          ((eq pose 2) (pose2) (push (send robot :copy-worldcoords) robot-worldcoords))
          ((eq pose 3) (pose3) (push (send robot :copy-worldcoords) robot-worldcoords))
          (t
           (setq *lp1* (pose1)) (push (send robot :copy-worldcoords) robot-worldcoords)
           (setq *lp2* (pose2)) (push (send robot :copy-worldcoords) robot-worldcoords)
           (setq *lp3* (pose3)) (push (send robot :copy-worldcoords) robot-worldcoords)
           (setq *lp-list* (list *lp1* *lp2* *lp3*))
           (setq robot-worldcoords (reverse robot-worldcoords))
           )
          )
    (send (send robot :rarm :end-coords) :assoc *punch-coords*)
    robot-worldcoords
    )
  )


