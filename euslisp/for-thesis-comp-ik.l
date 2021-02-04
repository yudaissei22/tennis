(setq *lfoot-step* (make-coords :pos #f(300 380 0) :rpy (list (deg2rad 30) 0 0)))
(setq *lfoot* (make-coords :pos #f(0 280 0) :rpy (list (deg2rad 30) 0 0)))
(setq *rfoot* (make-coords :pos #f(0 -280 0) :rpy (list (deg2rad -30) 0 0)))
(setq *look-at-target* #f(10000 0 2000))
(require "package://tennis/euslisp/robot-interface.l")
;; (objects (list *robot* *racket-grip* *racket-model* *sweet-spot*))


(send (send *robot* :rarm :end-coords) :dissoc *racket-coords*)
(send *robot* :angle-vector
      #f(
         -0.469077 -14.4589 -9.97825 18.9388 -10.8094 14.4694 ;; rleg_joint0~5 (<legs> crotch-y,r,p / knee-p / ankle-p, r)
         0.455322 14.4907 -9.94546 18.8684 -10.7721 -14.4944 ;; lleg_joint0~5
         0 0 0 ;; chest_joint0~2 (<torso> waist-r,p,y)
         0 -5.17925 ;; head_joint0~1 (<head> neck-y,p)
         ;; -0.328496 -59.2828 -50.1455 -0.49446 -90 -69.5632 -19.1267 46.2037 ;; rarm_joint0~7 (<arms> collar-y shoulder-p,r,y elbow-p wrist-y,r,p)
         9.04974 4.35166 -15.8251 40.471 -93.8492 -20.4049 -48.7172 20.7466 ;; rarm_join.t0~7 (<arms> collar-y shoulder-p,r,y elbow-p wrist-y,r,p)
         -9.07398 4.35839 15.8251 -40.4259 -93.8305 20.4995 48.6094 20.67)) ;; larm_joint0~7
         ;; 8 0 80 0 -40 45 0 0 ;; before Humanoids2016

(objects (list *robot*))
(setq *obj0-init* (copy-object (send *irtviewer* :objects)))
(print *obj0-init*)

;; rotate *robot* in order to use waist
;; (send *robot* :orient (deg2rad -6) :z :world)
;; (send *robot* :locate (float-vector 0 -10 -130) :world)
(send *robot* :move-to (make-coords) :world)

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
(send *larm-coords* :locate (float-vector 500 100 900) :world)
;;  end-effector <-> racket-grip transformation
(setq end-effector->racket-grip (make-coords :pos #f(0 0 0) :rpy (list (deg2rad -45) (deg2rad -45) 0)))
(setq *larm-coords* (send *larm-coords* :transform end-effector->racket-grip))

;; jskeus/eus/models/arrow-object.l
(require "models/arrow-object.l")
(setq *arrow-larm* (arrow :pos (send (send *larm-coords* :copy-worldcoords) :pos) :rot (send (send *larm-coords* :copy-worldcoords) :rot)))
(setq *arrow-racket* (arrow :pos (send (send *racket-grip* :copy-worldcoords) :pos) :rot (send (send *racket-grip* :copy-worldcoords) :rot)))
(setq *arrow-lfoot* (arrow :pos (send (send *lfoot* :copy-worldcoords) :pos) :rot (send (send *lfoot* :copy-worldcoords) :rot)))
(setq *arrow-rfoot* (arrow :pos (send (send *rfoot* :copy-worldcoords) :pos) :rot (send (send *rfoot* :copy-worldcoords) :rot)))

;; fullbody-inverse-kinematics
(let* ((target-coords
        (list
         *larm-coords*
         *racket-grip*
         *lfoot*
         *rfoot*)))

  (with-move-target-link-list
   (mt ll *robot* '(:larm :rarm :lleg :rleg))
   (send *robot*
         :fullbody-inverse-kinematics target-coords
         :root-link-virtual-joint-weight #f(0.1 0.5 0.1 0.1 0.1 0.1) ;; root-link has 6DOF
         :target-centroid-pos (midpoint 0.5 (send *lfoot* :worldpos) (send *rfoot* :worldpos))
         :centroid-thre 20
         :move-target mt
         ; :link-list ll
         :link-list (list (send *robot* :link-list (send (send *robot* :larm :end-coords) :parent))
                          (remove (send (send *robot* :rarm-collar-y) :child-link) (send *robot* :link-list (send (send *robot* :rarm :end-coords) :parent)) :count 1)
                          (send *robot* :link-list (send (send *robot* :lleg :end-coords) :parent))
                          (send *robot* :link-list (send (send *robot* :rleg :end-coords) :parent)))
         :avoid-collision-distance 60
         :avoid-collision-null-gain 5.0
         :avoid-collision-joint-gain 0.8
         :collision-avoidance-link-pair
         (list
          (list (cadr (send *robot* :torso :links))
                (send *robot* :larm_link2_lk))
          (list (cadr (send *robot* :torso :links))
                (send *robot* :rarm_link2_lk)))

         :additional-weight-list (list (list (send *robot* :head-neck-p :child-link) 0)
                                       (list (send *robot* :head-neck-y :child-link) 0))
         :rotation-axis (list t t t t)
         :debug-view nil))) ;;:no-message) ;;t)

;; (send *robot* :look-at-target *look-at-target*)

(objects (list *robot* *arrow-larm* *arrow-racket* *arrow-lfoot* *arrow-rfoot*))
(setq *obj0* (copy-object (send *irtviewer* :objects)))

(send (send *robot* :rarm :end-coords) :dissoc *racket-coords*)
(send *robot* :angle-vector
      #f(
         -0.469077 -14.4589 -9.97825 18.9388 -10.8094 14.4694 ;; rleg_joint0~5 (<legs> crotch-y,r,p / knee-p / ankle-p, r)
         0.455322 14.4907 -9.94546 18.8684 -10.7721 -14.4944 ;; lleg_joint0~5
         0 0 0 ;; chest_joint0~2 (<torso> waist-r,p,y)
         0 -5.17925 ;; head_joint0~1 (<head> neck-y,p)
         ;; -0.328496 -59.2828 -50.1455 -0.49446 -90 -69.5632 -19.1267 46.2037 ;; rarm_joint0~7 (<arms> collar-y shoulder-p,r,y elbow-p wrist-y,r,p)
         9.04974 4.35166 -15.8251 40.471 -93.8492 -20.4049 -48.7172 20.7466 ;; rarm_join.t0~7 (<arms> collar-y shoulder-p,r,y elbow-p wrist-y,r,p)
         -9.07398 4.35839 15.8251 -40.4259 -93.8305 20.4995 48.6094 20.67)) ;; larm_joint0~7
         ;; 8 0 80 0 -40 45 0 0 ;; before Humanoids2016
(send *robot* :reset-pose)
(objects (list *robot*))
(setq *obj1-init* (copy-object (send *irtviewer* :objects)))



;; rotate *robot* in order to use waist
;; (send *robot* :orient (deg2rad -6) :z :world)
;; (send *robot* :locate (float-vector 0 -10 -130) :world)
(send *robot* :move-to (make-coords) :world)

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
(send *larm-coords* :locate (float-vector 500 100 900) :world)
;;  end-effector <-> racket-grip transformation
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
   (mt ll *robot* '(:larm :rarm :lleg :rleg))
   (send *robot*
         :fullbody-inverse-kinematics target-coords
         :root-link-virtual-joint-weight #f(0.1 0.5 0.1 0.1 0.1 0.1) ;; root-link has 6DOF
         :target-centroid-pos (midpoint 0.5 (send *lfoot* :worldpos) (send *rfoot* :worldpos))
         :centroid-thre 20
         :move-target mt
         ; :link-list ll
         :link-list (list (send *robot* :link-list (send (send *robot* :larm :end-coords) :parent))
                          (remove (send (send *robot* :rarm-collar-y) :child-link) (send *robot* :link-list (send (send *robot* :rarm :end-coords) :parent)) :count 1)
                          (send *robot* :link-list (send (send *robot* :lleg :end-coords) :parent))
                          (send *robot* :link-list (send (send *robot* :rleg :end-coords) :parent)))
         :avoid-collision-distance 60
         :avoid-collision-null-gain 5.0
         :avoid-collision-joint-gain 0.8
         :collision-avoidance-link-pair
         (list
          (list (cadr (send *robot* :torso :links))
                (send *robot* :larm_link2_lk))
          (list (cadr (send *robot* :torso :links))
                (send *robot* :rarm_link2_lk)))

         :additional-weight-list (list (list (send *robot* :head-neck-p :child-link) 0)
                                       (list (send *robot* :head-neck-y :child-link) 0))
         :rotation-axis (list t t t t)
         :debug-view nil))) ;;:no-message) ;;t)

(objects (list *robot* *arrow-larm* *arrow-racket* *arrow-lfoot* *arrow-rfoot*))
(setq *obj1* (copy-object (send *irtviewer* :objects)))


(dolist (obj *obj0-init*)
  (send obj :translate #f(0 0 0) :world)
  )
(dolist (obj *obj0*)
  (send obj :translate #f(0 4000 0) :world)
  )
(dolist (obj *obj1-init*)
  (send obj :translate #f(0 0 2000) :world)
  )
(dolist (obj *obj1*)
  (send obj :translate #f(0 4000 2000) :world)
  )

  ;; jskeus/eus/models/arrow-object.l
(require "models/single-arrow-object.l")
(when (not (boundp '*single-arrow0*))
  (setq *single-arrow0* (single-arrow :pos #f(0 2000 1000))))
(when (not (boundp '*single-arrow1*))
  (setq *single-arrow1* (single-arrow :pos #f(0 2000 3000))))

(objects (flatten (concatenate cons *obj0* *obj0-init* *obj1* *obj1-init* (list *single-arrow0* *single-arrow1*))))
(send *irtviewer* :draw-objects)
