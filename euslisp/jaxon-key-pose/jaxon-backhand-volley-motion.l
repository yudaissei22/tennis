(require "package://tennis/euslisp/robot-interface.l")
(objects (list *robot* *racket-grip* *racket-model* *sweet-spot*))

;; make coords of robot foots
(setq *lfoot-step* (make-coords :pos #f(300 380 0) :rpy (list (deg2rad 30) 0 0)))
(setq *lfoot* (make-coords :pos #f(0 280 0) :rpy (list (deg2rad 30) 0 0)))
(setq *rfoot* (make-coords :pos #f(0 -280 0) :rpy (list (deg2rad -30) 0 0)))
(setq *rfoot-step* (make-coords :pos #f(300 -280 0) :rpy (list (deg2rad -30) 0 0)))
(setq *look-at-target* #f(10000 0 1200))

(defun joint-limit-with-margin (&key (robot *robot*) (margin (+ 0.5 1e-6)))
  (dolist (j (send robot :joint-list))
    (send j :max-angle (- (send j :max-angle) margin))
    (send j :min-angle (+ (send j :min-angle) margin))
    )
  )

(defun joint-limit-without-margin (&key (robot *robot*) (margin (+ 0.5 1e-6)))
  (dolist (j (send robot :joint-list))
    (send j :max-angle (+ (send j :max-angle) margin))
    (send j :min-angle (- (send j :min-angle) margin))
    )
  (format t "head-neck-p: ~A < ~A < ~A~%" (send (send robot :head-neck-p) :min-angle) (send (send robot :head-neck-p) :joint-angle) (send (send robot :head-neck-p) :max-angle))
  (format t "head-neck-y: ~A < ~A < ~A~%" (send (send robot :head-neck-y) :min-angle) (send (send robot :head-neck-y) :joint-angle) (send (send robot :head-neck-y) :max-angle))
  (format t "torso-waist-r: ~A < ~A < ~A~%" (send (send robot :torso-waist-r) :min-angle) (send (send robot :torso-waist-r) :joint-angle) (send (send robot :torso-waist-r) :max-angle))
  )

(defun util (&key (flush t))
	(progn
	 (objects (list *racket-model*))
	 (send (send *robot* :rarm :end-coords) :draw-on :flush flush :size 300)
	 (send *larm-coords* :draw-on :flush flush :size 300)
	 (send *racket-grip* :draw-on :flush flush :size 300)
	 (send *racket-coords* :draw-on :flush flush :size 300)
	 ))

;; forehand swing http://www.ikeyon.com/jtc/image/stforhandonecrt.gif
(defun pose1 (&key (robot *robot*) (base-time 5000) (real nil))
  (joint-limit-with-margin :robot robot)
  (send (send robot :rarm :end-coords) :dissoc *racket-coords*)
  (send robot :angle-vector
        #f(
           -0.469077 -14.4589 -9.97825 18.9388 -10.8094 14.4694 ;; rleg_joint0~5 (<legs> crotch-y,r,p / knee-p / ankle-p, r)
           0.455322 14.4907 -9.94546 18.8684 -10.7721 -14.4944 ;; lleg_joint0~5
           0 0 0 ;; chest_joint0~2 (<torso> waist-r,p,y)
           0 -5.17925 ;; head_joint0~1 (<head> neck-y,p)
           ;; -0.328496 -59.2828 -50.1455 -0.49446 -90 -69.5632 -19.1267 46.2037 ;; rarm_joint0~7 (<arms> collar-y shoulder-p,r,y elbow-p wrist-y,r,p)
           9.04974 4.35166 -17.8251 40.471 -93.8492 -20.4049 -48.7172 20.7466 ;; rarm_join.t0~7 (<arms> collar-y shoulder-p,r,y elbow-p wrist-y,r,p)
           -9.07398 4.35839 17.8251 -40.4259 -93.8305 20.4995 48.6094 20.67)) ;; larm_joint0~7
           ;; 8 0 80 0 -40 45 0 0 ;; before Humanoids2016


  ;; rotate robot in order to use waist
  ;; (send robot :orient (deg2rad -6) :z :world)
  ;; (send robot :locate (float-vector 0 -10 -130) :world)
  (send robot :move-to (make-coords) :world)

  (send *racket-coords* :orient (deg2rad 0) :z :world)
  (send *racket-coords* :locate (float-vector 0 0 0) :world)
  (send *racket-coords* :rotate (deg2rad -5) :x :world)
  (send *racket-coords* :rotate (deg2rad 30) :y :world)
  (send *racket-coords* :locate (float-vector 400 -100 900) :world)
;; mirror racket-coords
  (setq *larm-coords* (make-coords))
  (send *larm-coords* :orient (deg2rad 0) :z :world)
  (send *larm-coords* :locate (float-vector 0 0 0) :world)
  (send *larm-coords* :rotate (deg2rad 5) :x :world)
  (send *larm-coords* :rotate (deg2rad 30) :y :world)
  (send *larm-coords* :locate (float-vector 400 100 900) :world)
;; end-effector <-> racket-grip transformation
  (setq end-effector->racket-grip (make-coords :pos #f(0 0 0) :rpy (list (deg2rad -45) (deg2rad -45) 0)))
  (setq *larm-coords* (send *larm-coords* :transform end-effector->racket-grip))

  ;; fullbody-inverse-kinematics
  (let* ((target-coords
          (list
           *larm-coords*
           *racket-grip*
           *lfoot*
           *rfoot*)))

    (with-move-target-link-list
     (mt ll robot '(:larm :rarm :lleg :rleg))
     (send robot
           :fullbody-inverse-kinematics target-coords
           :root-link-virtual-joint-weight #f(0.1 0.5 0.1 0.1 0.1 0.1) ;; root-link has 6DOF
           :target-centroid-pos (midpoint 0.5 (send *lfoot* :worldpos) (send *rfoot* :worldpos))
           :centroid-thre 20
           :move-target mt
           ; :link-list ll
           :link-list (list (send robot :link-list (send (send robot :larm :end-coords) :parent))
                            (remove (send (send robot :rarm-collar-y) :child-link) (send robot :link-list (send (send robot :rarm :end-coords) :parent)) :count 1)
                            (send robot :link-list (send (send robot :lleg :end-coords) :parent))
                            (send robot :link-list (send (send robot :rleg :end-coords) :parent)))
           :avoid-collision-distance 60
           :avoid-collision-null-gain 5.0
           :avoid-collision-joint-gain 0.8
           :collision-avoidance-link-pair
           (list
            (list (cadr (send robot :torso :links))
                  (send robot :larm_link2_lk))
            (list (cadr (send robot :torso :links))
                  (send robot :rarm_link2_lk)))

           :additional-weight-list (list (list (send robot :head-neck-p :child-link) 0)
                                         (list (send robot :head-neck-y :child-link) 0))
           :rotation-axis (list t t t t)
           :debug-view nil))) ;;(:no-message t)

  (send *robot* :look-at-target *look-at-target*)
  ;; (send *robot* :move-centroid-on-foot (make-coords)) ;; これ必要じゃない?

  (send *irtviewer* :draw-objects)

  (if real
      (progn
        (send *ri* :angle-vector (send robot :angle-vector) base-time)
        (send *ri* :wait-interpolation)))

  (joint-limit-without-margin :robot robot)


  (list :angle-vector (send robot :angle-vector)
        :root-coords (send (car (send robot :links)) :copy-worldcoords) ;; (send robot :copy-worldcoords)
        :worldcoords (send robot :copy-worldcoords)
        :time base-time))





(defun pose2 (&key (robot *robot*) (base-time 5000) (real nil))
  (joint-limit-with-margin :robot robot)
  (send (send robot :rarm :end-coords) :dissoc *racket-coords*)
  (send robot :angle-vector
        #f(
           -0.469077 -14.4589 -9.97825 18.9388 -10.8094 14.4694 ;; rleg_joint0~5 (<legs> crotch-y,r,p / knee-p / ankle-p, r)
           0.455322 14.4907 -9.94546 18.8684 -10.7721 -14.4944 ;; lleg_joint0~5
           0 0 0 ;; chest_joint0~2 (<torso> waist-r,p,y)
           0 -5.17925 ;; head_joint0~1 (<head> neck-y,p)
           ;; -0.328496 -59.2828 -50.1455 -0.49446 -90 -69.5632 -19.1267 46.2037 ;; rarm_joint0~7 (<arms> collar-y shoulder-p,r,y elbow-p wrist-y,r,p)
           9.04974 4.35166 -17.8251 40.471 -93.8492 -20.4049 -48.7172 20.7466 ;; rarm_join.t0~7 (<arms> collar-y shoulder-p,r,y elbow-p wrist-y,r,p)
           -9.07398 4.35839 17.8251 -40.4259 -93.8305 20.4995 48.6094 20.67)) ;; larm_joint0~7
           ;; 8 0 80 0 -40 45 0 0 ;; before Humanoids2016


  ;; rotate robot in order to use waist
  ;; (send robot :orient (deg2rad -6) :z :world)
  ;; (send robot :locate (float-vector 0 -10 -130) :world)
  (send robot :move-to (make-coords) :world)

  (send *racket-coords* :orient (deg2rad 0) :z :world)
  (send *racket-coords* :locate (float-vector 0 0 0) :world)
  (send *racket-coords* :rotate (deg2rad -5) :x :world)
  (send *racket-coords* :rotate (deg2rad 30) :y :world)
  (send *racket-coords* :locate (float-vector 400 -100 900) :world)
;; mirror racket-coords
  (setq *larm-coords* (make-coords))
  (send *larm-coords* :orient (deg2rad 0) :z :world)
  (send *larm-coords* :locate (float-vector 0 0 0) :world)
  (send *larm-coords* :rotate (deg2rad 5) :x :world)
  (send *larm-coords* :rotate (deg2rad 30) :y :world)
  (send *larm-coords* :locate (float-vector 400 100 900) :world)
;; end-effector <-> racket-grip transformation
  (setq end-effector->racket-grip (make-coords :pos #f(0 0 0) :rpy (list (deg2rad -45) (deg2rad -45) 0)))
  (setq *larm-coords* (send *larm-coords* :transform end-effector->racket-grip))

  ;; fullbody-inverse-kinematics
  (let* ((target-coords
          (list
           *larm-coords*
           *racket-grip*
           *lfoot*
           *rfoot*)))

    (with-move-target-link-list
     (mt ll robot '(:larm :rarm :lleg :rleg))
     (send robot
           :fullbody-inverse-kinematics target-coords
           :root-link-virtual-joint-weight #f(0.1 0.5 0.1 0.1 0.1 0.1) ;; root-link has 6DOF
           :target-centroid-pos (midpoint 0.5 (send *lfoot* :worldpos) (send *rfoot* :worldpos))
           :centroid-thre 20
           :move-target mt
           ; :link-list ll
           :link-list (list (send robot :link-list (send (send robot :larm :end-coords) :parent))
                            (remove (send (send robot :rarm-collar-y) :child-link) (send robot :link-list (send (send robot :rarm :end-coords) :parent)) :count 1)
                            (send robot :link-list (send (send robot :lleg :end-coords) :parent))
                            (send robot :link-list (send (send robot :rleg :end-coords) :parent)))
           :avoid-collision-distance 60
           :avoid-collision-null-gain 5.0
           :avoid-collision-joint-gain 0.8
           :collision-avoidance-link-pair
           (list
            (list (cadr (send robot :torso :links))
                  (send robot :larm_link2_lk))
            (list (cadr (send robot :torso :links))
                  (send robot :rarm_link2_lk)))

           :additional-weight-list (list (list (send robot :head-neck-p :child-link) 0)
                                         (list (send robot :head-neck-y :child-link) 0))
           :rotation-axis (list t t t t)
           :debug-view nil))) ;;(:no-message t)

  (send *robot* :look-at-target *look-at-target*)
  ;; (send *robot* :move-centroid-on-foot (make-coords)) ;; これ必要じゃない?

  (send *irtviewer* :draw-objects)

  (if real
      (progn
        (send *ri* :angle-vector (send robot :angle-vector) base-time)
        (send *ri* :wait-interpolation)))

  (joint-limit-without-margin :robot robot)


  (list :angle-vector (send robot :angle-vector)
        :root-coords (send (car (send robot :links)) :copy-worldcoords) ;; (send robot :copy-worldcoords)
        :worldcoords (send robot :copy-worldcoords)
        :time base-time))




(defun pose3 (&key (robot *robot*) (base-time 5000) (real nil))
  (joint-limit-with-margin :robot robot)
  (send robot :angle-vector
        ;; rleg_joint0~5 (<legs> crotch-y,r,p / knee-p / ankle-p, r)
        ;; lleg_joint0~5
        ;; chest_joint0~2 (<torso> waist-r,p,y) head_joint0~1 (<head> neck-y,p)
        ;; rarm_joint0~7 (<arms> collar-y shoulder-p,r,y elbow-p wrist-y,r,p)
        ;; larm_joint0~7
        #f(-0.851399 -25.3628 -43.6886 81.0988 -37.0053 23.9976 43.4412 23.4412 -52.1147 73.8623 -20.4123 -24.1527
           4.69939 0 0 10 0.0
           ;; 13.9379 -1.29912 -50.4969 -82.3214 -80 -71.956 -85.2512 16.7357
           0.0 -1.29912 -50.4969 -82.3214 -80 -71.956 -85.2512 16.7357
           5.0 20.0 50.0 0.0 -100.0 45.0 0.0 0.0))
        ;; #f(-0.851399 -25.3628 -43.6886 81.0988 -37.0053 23.9976 43.4412 23.4412 -52.1147 73.8623 -20.4123 -24.1527 4.69939 17.9085 19.7456 22.4 0.0 13.9379 -1.29912 -50.4969 -82.3214 -108.234 -71.956 -85.2512 16.7357 8.0 0.0 80.0 0.0 -40.0 45.0 0.0 0.0) ;; before Humanoids2016

  (send robot :reset-manip-pose)
  ;; rotate robot in order to use waist
  ;; (send robot :orient (deg2rad 10) :z :world)
  ;; (send robot :locate (float-vector 0 -10 -240) :world)
  (send robot :move-to (make-coords) :world)

  ;; そういえばバックハンドは打面逆だけどまだ対処してない
  (send *racket-coords* :orient (deg2rad 0) :z :world)
  (send *racket-coords* :locate (float-vector 0 0 0) :world)
  (send *racket-coords* :rotate (deg2rad -90) :x :world)
  (send *racket-coords* :rotate (deg2rad 70) :y :world)
  (send *racket-coords* :rotate (deg2rad -30) :z :world)
  (send *racket-coords* :locate (float-vector 600 -100 1000) :world)
  ;; (send *racket-coords* :locate (float-vector 550 300 805) :world) ;; before Humanoids2016

  ;; fullbody-inverse-kinematics
  (let* ((target-coords
          (list
           *racket-grip*
           *lfoot*
           *rfoot-step*)))

    (with-move-target-link-list
     (mt ll robot '(:rarm :lleg :rleg))
     (send robot
           :fullbody-inverse-kinematics target-coords
           :root-link-virtual-joint-weight #f(0.1 0.5 0.1 0.1 0.1 0.1) ;; root-link has 6DOF
           :target-centroid-pos (midpoint 0.5 (send *lfoot-step* :worldpos) (send *rfoot* :worldpos))
           ; :target-centroid-pos (midpoint 0.5 (send *lfoot* :worldpos) (send *rfoot* :worldpos))
           :centroid-thre 20
           :move-target mt
           ; :link-list ll
           :link-list (list (remove (send (send robot :rarm-collar-y) :child-link) (send robot :link-list (send (send robot :rarm :end-coords) :parent)) :count 1)
                                     (send robot :link-list (send (send robot :lleg :end-coords) :parent))
                                     (send robot :link-list (send (send robot :rleg :end-coords) :parent)))
           :avoid-collision-distance 60
           :avoid-collision-null-gain 5.0
           :avoid-collision-joint-gain 0.8
           :collision-avoidance-link-pair
           (list
            (list (cadr (send robot :torso :links))
                  (send robot :larm_link2_lk))
            (list (cadr (send robot :torso :links))
                  (send robot :rarm_link2_lk)))

           :additional-weight-list (list (list (send robot :head-neck-p :child-link) 0)
                                         (list (send robot :head-neck-y :child-link) 0))
           :rotation-axis (list t t t)
           :debug-view nil))) ;;(:no-message t)

  (send *robot* :look-at-target *look-at-target*)

  (send *irtviewer* :draw-objects)

  (if real
      (progn
        (send *ri* :angle-vector (send robot :angle-vector) base-time)
        (send *ri* :wait-interpolation)))

  (joint-limit-without-margin :robot robot)


  (list :angle-vector (send robot :angle-vector)
        :root-coords (send (car (send robot :links)) :copy-worldcoords) ;; (send robot :copy-worldcoords)
        :worldcoords (send robot :copy-worldcoords)
        :time base-time))

(defun pose3-test (&key (robot *robot*) (base-time 5000) (real nil))
  (joint-limit-with-margin :robot robot)
  (send robot :angle-vector
        ;; rleg_joint0~5 (<legs> crotch-y,r,p / knee-p / ankle-p, r)
        ;; lleg_joint0~5
        ;; chest_joint0~2 (<torso> waist-r,p,y) head_joint0~1 (<head> neck-y,p)
        ;; rarm_joint0~7 (<arms> collar-y shoulder-p,r,y elbow-p wrist-y,r,p)
        ;; larm_joint0~7
        #f(-0.851399 -25.3628 -43.6886 81.0988 -37.0053 23.9976 43.4412 23.4412 -52.1147 73.8623 -20.4123 -24.1527
           4.69939 17.9085 19.7456 10 0.0
           ;; 13.9379 -1.29912 -50.4969 -82.3214 -80 -71.956 -85.2512 16.7357
           0.0 -1.29912 -50.4969 -82.3214 -80 -71.956 -85.2512 16.7357
           5.0 20.0 50.0 0.0 -100.0 45.0 0.0 0.0))
        ;; #f(-0.851399 -25.3628 -43.6886 81.0988 -37.0053 23.9976 43.4412 23.4412 -52.1147 73.8623 -20.4123 -24.1527 4.69939 17.9085 19.7456 22.4 0.0 13.9379 -1.29912 -50.4969 -82.3214 -108.234 -71.956 -85.2512 16.7357 8.0 0.0 80.0 0.0 -40.0 45.0 0.0 0.0) ;; before Humanoids2016

  ;; rotate robot in order to use waist
  ;; (send robot :orient (deg2rad 10) :z :world)
  ;; (send robot :locate (float-vector 0 -10 -240) :world)
  (send robot :move-to (make-coords) :world)

  (send *racket-coords* :orient (deg2rad 0) :z :world)
  (send *racket-coords* :locate (float-vector 0 0 0) :world)
  (send *racket-coords* :rotate (deg2rad 90) :x :world)
  (send *racket-coords* :rotate (deg2rad 70) :y :world)
  (send *racket-coords* :locate (float-vector 550 -900 1200) :world)
  ;; (send *racket-coords* :locate (float-vector 550 300 805) :world) ;; before Humanoids2016

  ;; fullbody-inverse-kinematics
  (let* ((target-coords
          (list
           *racket-grip*
           ; *lfoot-step*
           *lfoot*
           *rfoot*)))

    (with-move-target-link-list
     (mt ll robot '(:rarm :lleg :rleg))
     (send robot
           :fullbody-inverse-kinematics target-coords
           :root-link-virtual-joint-weight #f(0.1 0.5 0.1 0.1 0.1 0.1) ;; root-link has 6DOF
           ; :target-centroid-pos (midpoint 0.5 (send *lfoot-step* :worldpos) (send *rfoot* :worldpos))
           :target-centroid-pos (midpoint 0.5 (send *lfoot* :worldpos) (send *rfoot* :worldpos))
           :centroid-thre 20
           :move-target mt
           ; :link-list ll
           :link-list (list (remove (send (send robot :rarm-collar-y) :child-link) (send robot :link-list (send (send robot :rarm :end-coords) :parent)) :count 1)
                                     (send robot :link-list (send (send robot :lleg :end-coords) :parent))
                                     (send robot :link-list (send (send robot :rleg :end-coords) :parent)))
           :avoid-collision-distance 60
           :avoid-collision-null-gain 5.0
           :avoid-collision-joint-gain 0.8
           :collision-avoidance-link-pair
           (list
            (list (cadr (send robot :torso :links))
                  (send robot :larm_link2_lk))
            (list (cadr (send robot :torso :links))
                  (send robot :rarm_link2_lk)))

           :additional-weight-list (list (list (send robot :head-neck-p :child-link) 0)
                                         (list (send robot :head-neck-y :child-link) 0))
           :rotation-axis (list t t t)
           :debug-view nil))) ;;:no-message) ;;t)

  (send *robot* :look-at-target *look-at-target*)

  (send *irtviewer* :draw-objects)

  (if real
      (progn
        (send *ri* :angle-vector (send robot :angle-vector) base-time)
        (send *ri* :wait-interpolation)))


  (joint-limit-without-margin :robot robot)

  (list :angle-vector (send robot :angle-vector)
        :root-coords (send (car (send robot :links)) :copy-worldcoords) ;; (send robot :copy-worldcoords)
        :worldcoords (send robot :copy-worldcoords)
        :time base-time))



(defun pose4 (&key (robot *robot*) (base-time 5000) (real nil))
  (joint-limit-with-margin :robot robot)
  (send (send robot :rarm :end-coords) :dissoc *racket-coords*)
  (send robot :angle-vector
        #f(
           -0.469077 -14.4589 -9.97825 18.9388 -10.8094 14.4694 ;; rleg_joint0~5 (<legs> crotch-y,r,p / knee-p / ankle-p, r)
           0.455322 14.4907 -9.94546 18.8684 -10.7721 -14.4944 ;; lleg_joint0~5
           0 0 0 ;; chest_joint0~2 (<torso> waist-r,p,y)
           0 -5.17925 ;; head_joint0~1 (<head> neck-y,p)
           ;; -0.328496 -59.2828 -50.1455 -0.49446 -90 -69.5632 -19.1267 46.2037 ;; rarm_joint0~7 (<arms> collar-y shoulder-p,r,y elbow-p wrist-y,r,p)
           9.04974 4.35166 -17.8251 40.471 -93.8492 -20.4049 -48.7172 20.7466 ;; rarm_join.t0~7 (<arms> collar-y shoulder-p,r,y elbow-p wrist-y,r,p)
           -9.07398 4.35839 17.8251 -40.4259 -93.8305 20.4995 48.6094 20.67)) ;; larm_joint0~7
           ;; 8 0 80 0 -40 45 0 0 ;; before Humanoids2016


  ;; rotate robot in order to use waist
  ;; (send robot :orient (deg2rad -6) :z :world)
  ;; (send robot :locate (float-vector 0 -10 -130) :world)
  (send robot :move-to (make-coords) :world)

  (send *racket-coords* :orient (deg2rad 0) :z :world)
  (send *racket-coords* :locate (float-vector 0 0 0) :world)
  (send *racket-coords* :rotate (deg2rad -5) :x :world)
  (send *racket-coords* :rotate (deg2rad 30) :y :world)
  (send *racket-coords* :locate (float-vector 500 -100 900) :world)
;; mirror racket-coords
  (setq *larm-coords* (make-coords))
  (send *larm-coords* :orient (deg2rad 0) :z :world)
  (send *larm-coords* :locate (float-vector 0 0 0) :world)
  (send *larm-coords* :rotate (deg2rad 5) :x :world)
  (send *larm-coords* :rotate (deg2rad 30) :y :world)
  (send *larm-coords* :locate (float-vector 500 100 900) :world)
;; end-effector <-> racket-grip transformation
  (setq end-effector->racket-grip (make-coords :pos #f(0 0 0) :rpy (list (deg2rad -45) (deg2rad -45) 0)))
  (setq *larm-coords* (send *larm-coords* :transform end-effector->racket-grip))

  ;; fullbody-inverse-kinematics
  (let* ((target-coords
          (list
           *larm-coords*
           *racket-grip*
           *lfoot*
           *rfoot-step*)))

    (with-move-target-link-list
     (mt ll robot '(:larm :rarm :lleg :rleg))
     (send robot
           :fullbody-inverse-kinematics target-coords
           :root-link-virtual-joint-weight #f(0.1 0.5 0.1 0.1 0.1 0.1) ;; root-link has 6DOF
           :target-centroid-pos (midpoint 0.5 (send *lfoot-step* :worldpos) (send *rfoot* :worldpos))
           :centroid-thre 20
           :move-target mt
           ; :link-list ll
           :link-list (list (send robot :link-list (send (send robot :larm :end-coords) :parent))
                            (remove (send (send robot :rarm-collar-y) :child-link) (send robot :link-list (send (send robot :rarm :end-coords) :parent)) :count 1)
                            (send robot :link-list (send (send robot :lleg :end-coords) :parent))
                            (send robot :link-list (send (send robot :rleg :end-coords) :parent)))
           :avoid-collision-distance 60
           :avoid-collision-null-gain 5.0
           :avoid-collision-joint-gain 0.8
           :collision-avoidance-link-pair
           (list
            (list (cadr (send robot :torso :links))
                  (send robot :larm_link2_lk))
            (list (cadr (send robot :torso :links))
                  (send robot :rarm_link2_lk)))

           :additional-weight-list (list (list (send robot :head-neck-p :child-link) 0)
                                         (list (send robot :head-neck-y :child-link) 0))
           :rotation-axis (list t t t t)
           :debug-view nil))) ;;(:no-message t)

  (send *robot* :look-at-target *look-at-target*)
  ;; (send *robot* :move-centroid-on-foot (make-coords)) ;; これ必要じゃない?

  (send *irtviewer* :draw-objects)

  (if real
      (progn
        (send *ri* :angle-vector (send robot :angle-vector) base-time)
        (send *ri* :wait-interpolation)))

  (joint-limit-without-margin :robot robot)


  (list :angle-vector (send robot :angle-vector)
        :root-coords (send (car (send robot :links)) :copy-worldcoords) ;; (send robot :copy-worldcoords)
        :worldcoords (send robot :copy-worldcoords)
        :time base-time))





(defun pose5 (&key (robot *robot*) (base-time 5000) (real nil))
  (joint-limit-with-margin :robot robot)
  (send (send robot :rarm :end-coords) :dissoc *racket-coords*)
  (send robot :angle-vector
        #f(
           -0.469077 -14.4589 -9.97825 18.9388 -10.8094 14.4694 ;; rleg_joint0~5 (<legs> crotch-y,r,p / knee-p / ankle-p, r)
           0.455322 14.4907 -9.94546 18.8684 -10.7721 -14.4944 ;; lleg_joint0~5
           0 0 0 ;; chest_joint0~2 (<torso> waist-r,p,y)
           0 -5.17925 ;; head_joint0~1 (<head> neck-y,p)
           ;; -0.328496 -59.2828 -50.1455 -0.49446 -90 -69.5632 -19.1267 46.2037 ;; rarm_joint0~7 (<arms> collar-y shoulder-p,r,y elbow-p wrist-y,r,p)
           9.04974 4.35166 -17.8251 40.471 -93.8492 -20.4049 -48.7172 20.7466 ;; rarm_join.t0~7 (<arms> collar-y shoulder-p,r,y elbow-p wrist-y,r,p)
           -9.07398 4.35839 17.8251 -40.4259 -93.8305 20.4995 48.6094 20.67)) ;; larm_joint0~7
           ;; 8 0 80 0 -40 45 0 0 ;; before Humanoids2016


  ;; rotate robot in order to use waist
  ;; (send robot :orient (deg2rad -6) :z :world)
  ;; (send robot :locate (float-vector 0 -10 -130) :world)
  (send robot :move-to (make-coords) :world)

  (send *racket-coords* :orient (deg2rad 0) :z :world)
  (send *racket-coords* :locate (float-vector 0 0 0) :world)
  (send *racket-coords* :rotate (deg2rad -5) :x :world)
  (send *racket-coords* :rotate (deg2rad 30) :y :world)
  (send *racket-coords* :locate (float-vector 500 -100 900) :world)
;; mirror racket-coords
  (setq *larm-coords* (make-coords))
  (send *larm-coords* :orient (deg2rad 0) :z :world)
  (send *larm-coords* :locate (float-vector 0 0 0) :world)
  (send *larm-coords* :rotate (deg2rad 5) :x :world)
  (send *larm-coords* :rotate (deg2rad 30) :y :world)
  (send *larm-coords* :locate (float-vector 500 100 900) :world)
;; end-effector <-> racket-grip transformation
  (setq end-effector->racket-grip (make-coords :pos #f(0 0 0) :rpy (list (deg2rad -45) (deg2rad -45) 0)))
  (setq *larm-coords* (send *larm-coords* :transform end-effector->racket-grip))

  ;; fullbody-inverse-kinematics
  (let* ((target-coords
          (list
           *larm-coords*
           *racket-grip*
           *lfoot*
           *rfoot-step*)))

    (with-move-target-link-list
     (mt ll robot '(:larm :rarm :lleg :rleg))
     (send robot
           :fullbody-inverse-kinematics target-coords
           :root-link-virtual-joint-weight #f(0.1 0.5 0.1 0.1 0.1 0.1) ;; root-link has 6DOF
           :target-centroid-pos (midpoint 0.5 (send *lfoot-step* :worldpos) (send *rfoot* :worldpos))
           :centroid-thre 20
           :move-target mt
           ; :link-list ll
           :link-list (list (send robot :link-list (send (send robot :larm :end-coords) :parent))
                            (remove (send (send robot :rarm-collar-y) :child-link) (send robot :link-list (send (send robot :rarm :end-coords) :parent)) :count 1)
                            (send robot :link-list (send (send robot :lleg :end-coords) :parent))
                            (send robot :link-list (send (send robot :rleg :end-coords) :parent)))
           :avoid-collision-distance 60
           :avoid-collision-null-gain 5.0
           :avoid-collision-joint-gain 0.8
           :collision-avoidance-link-pair
           (list
            (list (cadr (send robot :torso :links))
                  (send robot :larm_link2_lk))
            (list (cadr (send robot :torso :links))
                  (send robot :rarm_link2_lk)))

           :additional-weight-list (list (list (send robot :head-neck-p :child-link) 0)
                                         (list (send robot :head-neck-y :child-link) 0))
           :rotation-axis (list t t t t)
           :debug-view nil))) ;;(:no-message t)

  (send *robot* :look-at-target *look-at-target*)
  ;; (send *robot* :move-centroid-on-foot (make-coords)) ;; これ必要じゃない?

  (send *irtviewer* :draw-objects)

  (if real
      (progn
        (send *ri* :angle-vector (send robot :angle-vector) base-time)
        (send *ri* :wait-interpolation)))

  (joint-limit-without-margin :robot robot)


  (list :angle-vector (send robot :angle-vector)
        :root-coords (send (car (send robot :links)) :copy-worldcoords) ;; (send robot :copy-worldcoords)
        :worldcoords (send robot :copy-worldcoords)
        :time base-time))




;; pose番号を指定すると
(defun load-backhand-volley-pose (&key (robot *robot*) (pose 0))
  (send (send robot :rarm :end-coords) :dissoc *racket-coords*)
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
           ;; (push a lst) will make list (a lst), so robot-worldcoord is like pose (5, 4, 3, 2, 1). So it has to be reversed.
           (setq robot-worldcoords (reverse robot-worldcoords))))


    (send (send robot :rarm :end-coords) :assoc *racket-coords*)
    (send robot :copy-worldcoords)
    robot-worldcoords))


