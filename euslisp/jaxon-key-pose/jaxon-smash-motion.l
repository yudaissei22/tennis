(require "package://tennis/euslisp/robot-interface.l")

(objects (list *robot* *racket-grip* *racket-model* *sweet-spot*))

;; make coords of robot foots
(setq *lfoot* (make-coords :pos #f(-250 275 0) :rpy (list (deg2rad 80) 0 0)))
(setq *rfoot* (make-coords :pos #f(0 -275 0) :rpy (list (deg2rad 50) 0 0)))


;; smash http://illumin.usc.edu/290/fast-furious-the-science-behind-badminton-smashes/
(defun pose1 (&key (robot *robot*) (base-time 5000) (real nil))
  (send robot :angle-vector
        #f(
           0.0 0.0 -20.0 40.0 -20.0 0.0 ;; rleg_joint0~5 (<legs> crotch-y,r,p / knee-p / ankle-p, r)
           0.0 0.0 -20.0 40.0 -20.0 0.0 ;; lleg_joint0~5
           10 5 20 ;; chest_joint0~2 (<torso> waist-r,p,y)
           22.4 -20.0 ;; head_joint0~1 (<head> neck-y,p)
           ;;; 0 #|15.0|# -80.0 -16.0 -90 #|-20.0|# -110.0 -30.0 -50.0 0.0 ;; rarm_joint0~6 (<arms> collar-y shoulder-p,r,y elbow-p wrist-y,r,p)
           0 0 -90 -90 -90 0 0 0
           0.0 -20.0 110.0 10.0 -60.0 0.0 0.0 0.0 ;; larm_joint0~6
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
  (send robot :locate (float-vector 0 200 -100) :world)
  ;; (send robot :move-to (make-coords) :world)
  ;; (send *racket-coords* :orient (deg2rad -30) :z :world)
  ;; (send *racket-coords* :locate (float-vector 0 0 0) :world)
  ;; (send *racket-coords* :rotate (deg2rad -20) :x :world)
  ;; (send *racket-coords* :rotate (deg2rad -30) :y :world)
  ;; (send *racket-coords* :locate (float-vector -50 -630 1770) :world)
  (send *racket-coords* :move-to (make-coords :pos #f(-50 -630 1750) :rpy (list -2.18792 -0.213187 0.839454)) :world)

  ;; fullbody-inverse-kinematics
  (let* ((target-coords
          (list
           *racket-grip*
           *lfoot*
           *rfoot*
           )))
    (with-move-target-link-list
     (mt ll robot '(:rarm :lleg :rleg))
     (send robot
           :fullbody-inverse-kinematics target-coords
           :root-link-virtual-joint-weight #f(0.1 0.5 0.1 0.1 0.1 0.1) ;; root-link has 6DOF
           :target-centroid-pos (midpoint 0.55 (send *lfoot* :worldpos) (send *rfoot* :worldpos))
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
        #f(35.0332 -12.8958 14.6233 69.9458 -60.1577 11.2977 58.82 32.6891 -18.0185 59.0309 -17.3196 -21.2493
           -5.97834 -1.56465 -30 #|-8.34507|# 22.4 -20.0
           4.94147 44.2269 -147.952 -160.0 -100.0 -50.0 10.5974 -50 #|8.93119|#
           ;; 20 #|-1.76013|# -77.382 -98.023 19.1813 -103.933 -90.6593 21.5397 24.9444
           ;; 0.0 -20.0 110.0 10.0 -40.0 0.0 0.0 0.0
           0.0 -10.0 40.0 10.0 -80.0 40.0 0.0 -20.0
           )
        ;; #f(35.2008 -13.743 -12.4174 51.255 -35.6476 13.6105 48.4486 18.4868 -28.9614 41.1118 -8.95254 -17.8586
        ;;    10.6808 -1.97576 -36.8652 22.4 -20.0
        ;;    20 #|2.39455|# -40 #|-129.445|# -40 #|-15.9489|# 80 #|-66.5771|# -88.6649 -8.80604 -65.1692 -48.799
        ;;    0.0 -10.0 40.0 10.0 -60.0 0.0 0.0 0.0)
        )

  ;; rotate robot in order to use waist
  (send robot :orient (deg2rad 70) :z :world)
  (send robot :locate (float-vector -100 300 -150) :world)
  ;; (send robot :move-to (make-coords) :world)

  ;; (send *racket-coords* :orient (deg2rad 110) :z :world)
  ;; (send *racket-coords* :locate (float-vector 0 0 0) :world)
  ;; (send *racket-coords* :rotate (deg2rad 150) :x :world)
  ;; (send *racket-coords* :rotate (deg2rad 30) :y :world)
  ;; (send *racket-coords* :locate (float-vector -50 -480 1570) :world)
  (send *racket-coords* :move-to (make-coords :pos #f(-85.73 -56.558 1874.83) :rpy (list -1.848 -1.374 -2.828)) :world)

  ;; fullbody-inverse-kinematics
  (let* ((target-coords
          (list
           *racket-grip*
           *lfoot*
           *rfoot*
           )))
    (with-move-target-link-list
     (mt ll robot '(:rarm :lleg :rleg))
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
        ;; #f(
        ;;    3.20807 -20.7196 -24.2303 55.1402 -33.1479 20.7332 ;; rleg_joint0~5 (<legs> crotch-y,r,p / knee-p / ankle-p, r)
        ;;    4.14109 22.8213 -22.1177 52.6859 -30.9337 -22.8213  ;; lleg_joint0~5
        ;;    0 0 0 ;; chest_joint0~2 (<torso> waist-r,p,y)
        ;;    45 0 ;; head_joint0~1 (<head> neck-y,p)
        ;;    30 -60 -60 3.48124 -136.099 -66.0525 -1.02826 78.5083 ;; rarm_joint0~6 (<arms> collar-y shoulder-p,r,y elbow-p wrist-y,r,p)
        ;;    8 0 80 0 -40 45 0 0;; larm_joint0~6
        ;;    ))
        ;; angle-vector of pose2
        #f(14.8349 -15.1749 -9.22134 64.8968 -50.0761 13.7909 52.2751 13.9498 -44.0201 64.2381 -14.9935 -11.6361
           -7.88173 -1.75459 30.0 -10 -20.0
           11.107 30 #|0.0|# -150.0 -170 #|-47.8503|# -30.0 -81.444 -10.8385 -10.0
           0.0 60.0 20.0 30.0 -110.0 40.0 0.0 -30.0
           )
        )

  ;; rotate robot in order to use waist
  (send robot :orient (deg2rad 70) :z :world)
  (send robot :locate (float-vector -100 300 -160) :world)
  ;; (send robot :move-to (make-coords) :world)

  (send *racket-coords* :orient (deg2rad 30) :z :world)
  (send *racket-coords* :locate (float-vector 0 0 0) :world)
  (send *racket-coords* :rotate (deg2rad -50) :y :world)
  ;; (send *racket-coords* :rotate (deg2rad -30) :x :world)
  (send *racket-coords* :locate (float-vector 100 650 1880) :world)

  ;; fullbody-inverse-kinematics
  (let* ((target-coords
          (list
           *racket-grip*
           *lfoot*
           *rfoot*
           )))
    (with-move-target-link-list
     (mt ll robot '(:rarm :lleg :rleg))
     (send robot
           :fullbody-inverse-kinematics target-coords
           :root-link-virtual-joint-weight #f(0.1 0.5 0.1 0.1 0.1 0.1) ;; root-link has 6DOF
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




(defun pose4 (&key (robot *robot*) (base-time 5000) (real nil))
  (send robot :angle-vector
        ;; #f(
        ;;    3.20807 -20.7196 -24.2303 55.1402 -33.1479 20.7332 ;; rleg_joint0~5 (<legs> crotch-y,r,p / knee-p / ankle-p, r)
        ;;    4.14109 22.8213 -22.1177 52.6859 -30.9337 -22.8213  ;; lleg_joint0~5
        ;;    0 0 0 ;; chest_joint0~2 (<torso> waist-r,p,y)
        ;;    45 0 ;; head_joint0~1 (<head> neck-y,p)
        ;;    30 -60 -60 3.48124 -136.099 -66.0525 -1.02826 78.5083 ;; rarm_joint0~6 (<arms> collar-y shoulder-p,r,y elbow-p wrist-y,r,p)
        ;;    8 0 80 0 -40 45 0 0;; larm_joint0~6
        ;;    ))
        ;; angle-vector of pose3
        #f(-16.1488 -22.5959 5.3254 50.3804 -50.0798 21.8743 21.197 7.96029 -48.1407 77.5293 -24.8434 -5.2678
           -9.46186 30 #|4.32954|# 25.3286 -20.0 -20.0
           30 #|8.45307|# 60 #|20.1646|# -155.875 -52.9487 -34.3756 -58.7143 -12.0544 27.3088
           0.0 60.0 20.0 30.0 -120.0 40.0 0.0 -30.0
           )
        ;; #f(-1.10986 -19.99 11.148 29.6545 -34.1883 18.904 10.3147 12.418 -25.7493 54.8251 -22.6104 -12.0476
        ;;    -11.3761 35 #|-0.408983|# 7.94282 -10 20.0
        ;;    17.1906 -116.335 -29.8142 90 #|-6.04664|# -60 #|-56.3718|# 10.0065 27.966 32.6213
        ;;    0.0 60.0 20.0 30.0 -100.0 0.0 0.0 0.0)
        )
  ;; rotate robot in order to use waist
  (send robot :orient (deg2rad 90) :z :world)
  (send robot :locate (float-vector -100 300 -220) :world)
  ;; (send robot :move-to (make-coords) :world)

  (send *racket-coords* :orient (deg2rad -60) :z :world)
  (send *racket-coords* :locate (float-vector 0 0 0) :world)
  (send *racket-coords* :rotate (deg2rad -10) :x :world)
  (send *racket-coords* :rotate (deg2rad -50) :y :world)
  (send *racket-coords* :rotate (deg2rad -180) :x :world)
  (send *racket-coords* :locate (float-vector -550 400 670) :world)

  ;; fullbody-inverse-kinematics
  (let* ((target-coords
          (list
           *racket-grip*
           *lfoot*
           *rfoot*
           )))
    (with-move-target-link-list
     (mt ll robot '(:rarm :lleg :rleg))
     (send robot
           :fullbody-inverse-kinematics target-coords
           :root-link-virtual-joint-weight #f(0.1 0.5 0.1 0.1 0.1 0.1) ;; root-link has 6DOF
           :target-centroid-pos (midpoint 0.45 (send *lfoot* :worldpos) (send *rfoot* :worldpos))
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



(defun load-smash-pose (&key (robot *robot*) (pose 0))
  (send (send robot :rarm :end-coords) :dissoc *racket-coords*)
  (let* ((robot-worldcoords nil))
    (cond ((eq pose 1) (pose1) (push (send robot :copy-worldcoords) robot-worldcoords))
          ((eq pose 2) (pose2) (push (send robot :copy-worldcoords) robot-worldcoords))
          ((eq pose 3) (pose3) (push (send robot :copy-worldcoords) robot-worldcoords))
          ((eq pose 4) (pose4) (push (send robot :copy-worldcoords) robot-worldcoords))
          (t
           (setq *lp1* (pose1)) (push (send robot :copy-worldcoords) robot-worldcoords)
           (setq *lp2* (pose2)) (push (send robot :copy-worldcoords) robot-worldcoords)
           (setq *lp3* (pose3)) (push (send robot :copy-worldcoords) robot-worldcoords)
           (setq *lp4* (pose4)) (push (send robot :copy-worldcoords) robot-worldcoords)
           (setq *lp-list* (list *lp1* *lp2* *lp3* *lp4*))
           (setq robot-worldcoords (reverse robot-worldcoords))
           )
          )
    (send (send robot :rarm :end-coords) :assoc *racket-coords*)
    (send robot :copy-worldcoords)
    robot-worldcoords
    )
  )
