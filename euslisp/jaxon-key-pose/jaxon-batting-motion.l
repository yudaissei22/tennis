(require "package://tennis/euslisp/robot-interface.l")

(objects (list *robot* *bat-model* *bat-coords* *bat-lhand-grip* *bat-rhand-grip*))

;; make coords of robot foots
(setq *lfoot-step* (make-coords :pos #f(-150 275 0) :rpy (list (deg2rad 10) 0 0)))
(setq *lfoot* (make-coords :pos #f(-100 350 0) :rpy (list (deg2rad 20) 0 0)))
(setq *rfoot* (make-coords :pos #f(0 -350 0)))


;; batting motion
(defun pose1 (&key (robot *robot*) (base-time 5000) (real nil))
  (send robot :angle-vector
        #f(
           3.20807 -20.7196 -24.2303 55.1402 -33.1479 20.7332 ;; rleg_joint0~5 (<legs> crotch-y,r,p / knee-p / ankle-p, r)
           4.14109 22.8213 -22.1177 52.6859 -30.9337 -22.8213  ;; lleg_joint0~5
           4.93 20 0 ;; chest_joint0~2 (<torso> waist-r,p,y)
           22 0 ;; head_joint0~1 (<head> neck-y,p)
           0  -40 #|-61.2996|# -40 #|-22.099|# 3.48124 -120 -50 -1.02826 50 ;; rarm_joint0~6 (<arms> collar-y shoulder-p,r,y elbow-p wrist-y,r,p)
           -40 -66.8636 20.0 -62.2054 -80 41.3328 3.89257 18.9208 ;; larm_joint0~6
           )
        )

  ;; rotate robot in order to use waist
  (send robot :orient (deg2rad 6) :z :world)
  (send robot :locate (float-vector 0 -10 -140) :world)
  ;; (send robot :move-to (make-coords) :world)

  ;; set bat pos
  (send *bat-coords* :orient (deg2rad -80) :z :world)
  (send *bat-coords* :locate (float-vector 0 0 0) :world)
  (send *bat-coords* :rotate (deg2rad -11) :x :world)
  (send *bat-coords* :rotate (deg2rad -25) :y :world)
  (send *bat-coords* :rotate (deg2rad 3) :z :world)
  (send *bat-coords* :locate (float-vector 420 -220 1390) :world)



  ;; fullbody-inverse-kinematics
  (let* ((target-coords
          (list
           *bat-lhand-grip*
           *bat-rhand-grip*
           *lfoot*
           *rfoot*
           )))
    (with-move-target-link-list
     (mt ll robot '(:larm :rarm :lleg :rleg))
     (send robot
           :fullbody-inverse-kinematics target-coords
           :root-link-virtual-joint-weight #f(0.1 0.5 0.1 0.1 0.1 0.1) ;; root-link has 6DOF
           :target-centroid-pos (midpoint 0.56 (send *lfoot* :worldpos) (send *rfoot* :worldpos))
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
           :rotation-axis (list t :z t t)
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
        #f(
           -4.77263 -15.5019 -31.9588 62.8962 -28.7424 15.8122 -6.16411 20.8193 -22.0385 57.3296 -33.0282 -20.5345
           -2.26762 11.09 -15 #|-9.82555|# 22.0 0.0
           -3.37264 -56.752 -36.3058 0.609499 -121.157 -82.0001 -0.642075 37.5632
           -27.0033 -53.9518 18.6505 -48.6998 -95.3719 20.6904 25.8837 -17.6464
           ;; rleg_joint0~5 (<legs> crotch-y,r,p / knee-p / ankle-p, r)
           ;; lleg_joint0~5
           ;; chest_joint0~2 (<torso> waist-r,p,y)
           ;; head_joint0~1 (<head> neck-y,p)
           ;; rarm_joint0~6 (<arms> collar-y shoulder-p,r,y elbow-p wrist-y,r,p)
           ;; larm_joint0~6
           )
        )

  ;; rotate robot in order to use waist
  (send robot :orient (deg2rad -6) :z :world)

  ;; set bat pos
  (send *bat-coords* :orient (deg2rad -90) :z :world)
  (send *bat-coords* :locate (float-vector 0 0 0) :world)
  (send *bat-coords* :rotate (deg2rad -17) :x :world)
  (send *bat-coords* :rotate (deg2rad -25) :y :world)
  (send *bat-coords* :rotate (deg2rad 3) :z :world)
  (send *bat-coords* :locate (float-vector 340 -440 1390) :world)

  (send robot :locate (float-vector 0 -70 -160) :world)
  ;; (send robot :move-to (make-coords) :world)


  ;; fullbody-inverse-kinematics
  (let* ((target-coords
          (list
           *bat-lhand-grip*
           *bat-rhand-grip*
           *lfoot*
           *rfoot*
           )))
    (with-move-target-link-list
     (mt ll robot '(:larm :rarm :lleg :rleg))
     (send robot
           :fullbody-inverse-kinematics target-coords
           :root-link-virtual-joint-weight #f(0.1 0.5 0.1 0.1 0.1 0.1) ;; root-link has 6DOF
           :target-centroid-pos (midpoint 0.62 (send *lfoot* :worldpos) (send *rfoot* :worldpos))
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
           :rotation-axis (list t :z t t)
           :debug-view nil);; :no-message) ;;t)
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
        #f(
           -15.6492 -19.0572 -34.4916 66.0129 -31.5211 19.0566
           -15.6487 26.1808 -31.5247 53.3326 -21.8081 -26.1803
           11 0.0 40.0
           10.0 20.0
           30.0 -20.0 -25.0 -10.0 -50.0 -79.9839 -13.4228 0.0
           -40.0 -40.0 15.8251 -20 -110.0 0 50 0 #|-60.0 0.0 24.4932|#
           ;; rleg_joint0~5 (<legs> crotch-y,r,p / knee-p / ankle-p, r)
           ;; lleg_joint0~5
           ;; chest_joint0~2 (<torso> waist-r,p,y)
           ;; head_joint0~1 (<head> neck-y,p)
           ;; rarm_joint0~6 (<arms> collar-y shoulder-p,r,y elbow-p wrist-y,r,p)
           ;; larm_joint0~6
           )
        )

  ;; rotate robot in order to use waist
  (send robot :orient (deg2rad 50) :z :world)
  (send robot :rotate (deg2rad 2) :x :world)
  ;; (send robot :move-to (make-coords) :world)

  ;; set bat pos
  (send *bat-coords* :orient (deg2rad -85) :z :world)
  (send *bat-coords* :locate (float-vector 0 0 0) :world)
  (send *bat-coords* :rotate (deg2rad 95) :x :world)
  (send *bat-coords* :rotate (deg2rad -45) :y :world)
  (send *bat-coords* :rotate (deg2rad 90) :z :world)
  (send *bat-coords* :locate (float-vector 230 540 1050) :world)

  ;; fullbody-inverse-kinematics
  (let* ((target-coords
          (list
           *bat-lhand-grip*
           *bat-rhand-grip*
           *lfoot*
           *rfoot*
           )))
    (with-move-target-link-list
     (mt ll robot '(:larm :rarm :lleg :rleg))
     (send robot
           :fullbody-inverse-kinematics target-coords
           :root-link-virtual-joint-weight #f(0.1 0.5 0.1 0.1 0.1 0.1) ;; root-link has 6DOF
           :target-centroid-pos (midpoint 0.54 (send *lfoot* :worldpos) (send *rfoot* :worldpos))
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
           :additional-weight-list (list (list (send robot :torso-waist-p :child-link) 0.05)
                                         (list (send robot :head-neck-p :child-link) 0)
                                         (list (send robot :head-neck-y :child-link) 0))
           :rotation-axis (list t nil t t)
           ;; :revert-if-fail nil
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
        #f(-47.6535 -27.9518 -28.9792 56.7006 -28.1155 24.819 -27.7139 16.1535 -40.2493 58.2381 -17.2148 -19.2184 11.3199 -0.201633 50.0 10.0 20.0 14.1567 -10.0 -50.0 40.0 -82.6787 -78.6805 0.74681 -4.05497 5.0 -30.4867 50.0 100.0 -115.0 -20.3494 5.70661 -10.0
        ;; #f(
        ;;    -54.5506 -21.5559 -21.8407 58.4919 -32.1874 22.7584
        ;;    -37.8895 26.2435 -36.6772 57.4459 -16.8569 -23.8008
        ;;    -10 0 -18.0787 15.0 3.0
        ;;    30.7303 -20.0 -35.0 30.0 -100.0 -77.0008 10.0 -3.37524
        ;;    7.74562 -22.0331 50.6219 137.464 -100.759 -157.633 -17.6672 56.1097
           ;; rleg_joint0~5 (<legs> crotch-y,r,p / knee-p / ankle-p, r)
           ;; lleg_joint0~5
           ;; chest_joint0~2 (<torso> waist-r,p,y)
           ;; head_joint0~1 (<head> neck-y,p)
           ;; rarm_joint0~6 (<arms> collar-y shoulder-p,r,y elbow-p wrist-y,r,p)
           ;; larm_joint0~6
           )
        )

  (send robot :move-to (make-coords :pos #f(-81.977 69.4496 -162.053) :rpy (list 0.82791 -0.036189 0.041464)) :world)
  (send *bat-coords* :move-to (make-coords :pos #f(-518.219 -437.635 1524.17) :rpy (list -2.28162 -0.762223 -0.67427)) :world)
  (send robot :rotate (deg2rad -5) :z :world)
  ;; ;; rotate robot in order to use waist
  ;; (send robot :orient (deg2rad 60) :z :world)

  ;; ;; set bat pos
  ;; (send *bat-coords* :orient (deg2rad -85) :z :world)
  ;; (send *bat-coords* :locate (float-vector 0 0 0) :world)
  ;; (send *bat-coords* :rotate (deg2rad 45) :x :world)
  ;; (send *bat-coords* :rotate (deg2rad -45) :y :world)
  ;; (send *bat-coords* :rotate (deg2rad 90) :z :world)
  ;; (send *bat-coords* :rotate (deg2rad 90) :z)
  ;; (send *bat-coords* :rotate (deg2rad 250) :z :world)

  ;; (send *bat-coords* :locate (float-vector -520 00 1380 #|820|#) :world)

  ;; (send robot :locate (float-vector 0 0 -160) :world)

  ;; ;; adjust *bat-coords*
  ;; (send *bat-coords* :rotate (deg2rad -70) :z)
  ;; (send *bat-coords* :translate (float-vector 2.79 38.62 61.04))


  ;; fullbody-inverse-kinematics
  (let* ((target-coords
          (list
           *bat-lhand-grip*
           ;; *bat-rhand-grip*
           *lfoot*
           *rfoot*
           )))
    (with-move-target-link-list
     (mt ll robot '(:larm #|:rarm|# :lleg :rleg))
     (send robot
           :fullbody-inverse-kinematics target-coords
           :root-link-virtual-joint-weight #f(0.1 0.5 0.1 1 0.1 0.1) ;; root-link has 6DOF
           :target-centroid-pos (midpoint 0.56 (send *lfoot* :worldpos) (send *rfoot* :worldpos))
           :centroid-thre 20
           :move-target mt
           :link-list ll
           :avoid-collision-distance 30
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
           :revert-if-fail nil
           :rotation-axis (list t #|nil|# t t)
           :debug-view nil) ;;:no-message) ;;t)
     ))
  (send (send robot :larm :end-coords) :assoc *bat-coords*)
  (send robot :torso-waist-r :joint-angle 10)
  (send robot :torso-waist-p :joint-angle 0)
  (send robot :torso-waist-y :joint-angle 50)
  (send (send robot :larm :end-coords) :dissoc *bat-coords*)

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



(defun pose5 (&key (robot *robot*) (base-time 5000) (real nil))
  (send robot :angle-vector
        #f(
           3.20807 -20.7196 -24.2303 55.1402 -33.1479 20.7332 ;; rleg_joint0~5 (<legs> crotch-y,r,p / knee-p / ankle-p, r)
           4.14109 22.8213 -22.1177 52.6859 -30.9337 -22.8213  ;; lleg_joint0~5
           0 20 0 ;; chest_joint0~2 (<torso> waist-r,p,y)
           22 0 ;; head_joint0~1 (<head> neck-y,p)
           0  -40 #|-61.2996|# -40 #|-22.099|# 3.48124 -120 -50 -1.02826 50 ;; rarm_joint0~6 (<arms> collar-y shoulder-p,r,y elbow-p wrist-y,r,p)
           -40 -66.8636 20.0 -62.2054 -80 41.3328 3.89257 18.9208 ;; larm_joint0~6
           )
        )

  ;; rotate robot in order to use waist
  (send robot :orient (deg2rad 6) :z :world)
  (send robot :locate (float-vector 0 -10 -140) :world)
  ;; (send robot :move-to (make-coords) :world)

  ;; set bat pos
  (send *bat-coords* :orient (deg2rad -80) :z :world)
  (send *bat-coords* :locate (float-vector 0 0 0) :world)
  (send *bat-coords* :rotate (deg2rad -11) :x :world)
  (send *bat-coords* :rotate (deg2rad -25) :y :world)
  (send *bat-coords* :rotate (deg2rad 3) :z :world)
  (send *bat-coords* :locate (float-vector 480 -240 1390) :world)



  ;; fullbody-inverse-kinematics
  (let* ((target-coords
          (list
           *bat-lhand-grip*
           *bat-rhand-grip*
           *lfoot-step*
           *rfoot*
           )))
    (with-move-target-link-list
     (mt ll robot '(:larm :rarm :lleg :rleg))
     (send robot
           :fullbody-inverse-kinematics target-coords
           :root-link-virtual-joint-weight #f(0.1 0.5 0.1 0.1 0.1 0.1) ;; root-link has 6DOF
           :target-centroid-pos (midpoint 0.65 (send *lfoot* :worldpos) (send *rfoot* :worldpos))
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
           :rotation-axis (list t :z t t)
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



(defun load-batting-pose (&key (robot *robot*) (pose 0))
  (send (send robot :larm :end-coords) :dissoc *bat-coords*)
  (let* ((robot-worldcoords nil))
    (cond ((eq pose 1) (pose1) (push (send robot :copy-worldcoords) robot-worldcoords))
          ((eq pose 2) (pose2) (push (send robot :copy-worldcoords) robot-worldcoords))
          ((eq pose 3) (pose3) (push (send robot :copy-worldcoords) robot-worldcoords))
          ((eq pose 4) (pose4) (push (send robot :copy-worldcoords) robot-worldcoords))
          ((eq pose 5) (pose5) (push (send robot :copy-worldcoords) robot-worldcoords))
          (t
           (setq *lp1* (pose1)) (push (send robot :copy-worldcoords) robot-worldcoords)
           (setq *lp2* (pose2)) (push (send robot :copy-worldcoords) robot-worldcoords)
           (setq *lp3* (pose3)) (push (send robot :copy-worldcoords) robot-worldcoords)
           (setq *lp4* (pose4)) (push (send robot :copy-worldcoords) robot-worldcoords)
           (setq *lp5* (pose5)) (push (send robot :copy-worldcoords) robot-worldcoords)
           (setq *lp-list* (list *lp1* *lp2* *lp3* *lp4* *lp5*))
           (setq robot-worldcoords (reverse robot-worldcoords))
           )
          )
    (send (send robot :larm :end-coords) :assoc *bat-coords*)
    (send robot :copy-worldcoords)
    robot-worldcoords
    )
  )
