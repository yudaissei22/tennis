(require "package://tennis/euslisp/robot-interface.l")

(objects (list *robot* *kick-point*))

;; make coords of robot foots
(setq *rfoot-step* (make-coords :pos #f(250 0 0) :rpy (list (deg2rad 75) 0 0)))
(setq *lfoot* (make-coords :pos #f(-100 240 0) :rpy (list (deg2rad 90) 0 0)))
(setq *rfoot* (make-coords :pos #f(250 480 0) :rpy (list (deg2rad 100) 0 0))))

;; kick motion http://www.aspetar.com/journal/viewarticle.aspx?id=163
(defun pose1 (&key (robot *robot*) (base-time 5000) (real nil))
  (send robot :angle-vector
        #f(
           0.0 0.0 -20.0 40.0 -20.0 0.0 ;; rleg_joint0~5 (<legs> crotch-y,r,p / knee-p / ankle-p, r)
           0.0 0.0 -20.0 40.0 -20.0 0.0 ;; lleg_joint0~5
           0 5 10 ;; chest_joint0~2 (<torso> waist-r,p,y)
           10 30.0 ;; head_joint0~1 (<head> neck-y,p)
           0.0 40.0 -25.0 -5.0 -50.0 0.0 0.0 -20.0 ;; rarm_joint0~6 (<arms> collar-y shoulder-p,r,y elbow-p wrist-y,r,p)
           0.0 40.0 30.0 5.0 -50.0 0.0 0.0 -20.0 ;; larm_joint0~6
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
  (send robot :orient (deg2rad 80) :z :world)
  (send robot :locate (float-vector 0 0 -100) :world)
  ;; (send robot :move-to (make-coords) :world)


  ;; fullbody-inverse-kinematics
  (let* ((target-coords
          (list
           *lfoot*
           *rfoot-step*
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
  (send *kick-coords* :move-to (send robot :rleg :end-coords :copy-worldcoords) :world)
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
           25 -10 40 50 20 0
           55.8173 4.53064 -33.0165 48.1424 -15.3197 -6.02988
           0.0 0.0 30.0 22.4 30.0
           0.0 40.0 -20.0 -5.0 -50.0 0.0 0.0 -20.0
           -50.0 60.0 60.0 5.0 -40.0 0.0 0.0 -20.0))

  ;; rotate robot in order to use waist
  (send robot :orient (deg2rad 30) :z :world)
  (send robot :locate (float-vector 0 -30 -100) :world)
  ;; (send robot :move-to (make-coords) :world)


  ;; fullbody-inverse-kinematics
  (let* ((target-coords
          (list
           *lfoot*
           )))
    (with-move-target-link-list
     (mt ll robot '(:lleg))
     (send robot
           :fullbody-inverse-kinematics target-coords
           :root-link-virtual-joint-weight #f(0.5 0.5 0.1 0.1 0.1 0.1) ;; root-link has 6DOF
           :target-centroid-pos (send *lfoot* :worldpos)
           :centroid-thre 30
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
           :rotation-axis (list t)
           :debug-view nil) ;;:no-message) ;;t)
     ))
  (send *kick-coords* :move-to (send robot :rleg :end-coords :copy-worldcoords) :world)
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

        ;; angle-vector of pose1
        #f(
           30 -30 -40 60 30 0
           0.8173 4.53064 -33.0165 48.1424 -15.3197 -6.02988
           0.0 0.0 30.0 0 30.0
           0.0 50.0 -40.0 -5.0 -70.0 0.0 0.0 -30.0
           -20.0 30.0 30.0 5.0 -80.0 0.0 0.0 -30.0))

  ;; rotate robot in order to use waist
  (send robot :orient (deg2rad 60) :z :world)
  ;; (send robot :fix-leg-to-coords *lfoot* :lleg)
  (send robot :locate (float-vector 0 0 -100) :world)
  ;; (send robot :move-to (make-coords) :world)

  (send *kick-coords* :locate #f(370 200 190) :world)

  ;; fullbody-inverse-kinematics
  (let* ((target-coords
          (list
           *lfoot*
           *kick-coords*
           )))
    (with-move-target-link-list
     (mt ll robot '(:lleg :rleg))
     (send robot
           :fullbody-inverse-kinematics target-coords
           :root-link-virtual-joint-weight #f(0.5 0.5 0.1 0.1 0.1 0.1) ;; root-link has 6DOF
           :target-centroid-pos (send *lfoot* :worldpos)
           :centroid-thre 30
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
           :rotation-axis (list t nil)
           :debug-view nil) ;;:no-message) ;;t)
     ))
  (send *kick-coords* :move-to (send robot :rleg :end-coords :copy-worldcoords) :world)
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

        ;; angle-vector of pose1
        #f(
           30 -30 -50 10 30 0
           0.8173 4.53064 -33.0165 48.1424 -15.3197 -6.02988
           0.0 0.0 30.0 0 30.0
           0.0 50.0 -40.0 -5.0 -50.0 0.0 0.0 -20.0
           -40.0 30.0 20.0 5.0 -30.0 0.0 0.0 -20.0))

  ;; rotate robot in order to use waist
  (send robot :orient (deg2rad 60) :z :world)
  (send robot :locate (float-vector 0 0 -100) :world)
  ;; (send robot :move-to (make-coords) :world)

  ;; fullbody-inverse-kinematics
  (let* ((target-coords
          (list
           *lfoot*
           )))
    (with-move-target-link-list
     (mt ll robot '(:lleg))
     (send robot
           :fullbody-inverse-kinematics target-coords
           :root-link-virtual-joint-weight #f(0.5 0.5 0.1 0.1 0.1 0.1) ;; root-link has 6DOF
           :target-centroid-pos (send *lfoot* :worldpos)
           :centroid-thre 30
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
           :rotation-axis (list t)
           :debug-view nil) ;;:no-message) ;;t)
     ))
  (send *kick-coords* :move-to (send robot :rleg :end-coords :copy-worldcoords) :world)
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
           0.0 0.0 -20.0 40.0 -20.0 0.0 ;; rleg_joint0~5 (<legs> crotch-y,r,p / knee-p / ankle-p, r)
           0.0 0.0 -20.0 40.0 -20.0 0.0 ;; lleg_joint0~5
           0 5 10 ;; chest_joint0~2 (<torso> waist-r,p,y)
           -10 30.0 ;; head_joint0~1 (<head> neck-y,p)
           0.0 40.0 -25.0 -5.0 -50.0 0.0 0.0 -20.0 ;; rarm_joint0~6 (<arms> collar-y shoulder-p,r,y elbow-p wrist-y,r,p)
           0.0 40.0 30.0 5.0 -50.0 0.0 0.0 -20.0 ;; larm_joint0~6
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
  (send robot :orient (deg2rad 95) :z :world)
  (send robot :locate (float-vector 0 0 -100) :world)
  ;; (send robot :move-to (make-coords) :world)


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
  (send *kick-coords* :move-to (send robot :rleg :end-coords :copy-worldcoords) :world)
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




(defun load-kick-pose (&key (robot *robot) (pose 0))
  (send (send robot :rleg :end-coords) :dissoc *kick-coords*)
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
    (send (send robot :rleg :end-coords) :assoc *kick-coords*)
    (send robot :copy-worldcoords)
    robot-worldcoords
    )
  )


