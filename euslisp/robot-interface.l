(when (not (boundp '*robot*))
  (require "package://hrpsys_ros_bridge_tutorials/euslisp/jaxon-interface.l")
  (setq *jaxon* (jaxon))
  (setq *robot* *jaxon*)
  (setq *rfoot-plate* (make-cube 230 150 0.1))
  (send *rfoot-plate* :put :name "rfoot-plate")
  (send *rfoot-plate* :locate #f(0 -100 45.8336) :world)
  (send (send *robot* :rleg :end-coords) :assoc *rfoot-plate*)
  (setq *lfoot-plate* (make-cube 230 10 0.1))
  (send *lfoot-plate* :put :name "lfoot-plate")
  (send *lfoot-plate* :locate #f(0 100 45.8336) :world)
  (send (send *robot* :lleg :end-coords) :assoc *lfoot-plate*))

(print "[Warn] setting lower torque limit")
(send (send *robot* :larm :collar-y) :max-joint-torque 0) ;; defaults to about 220[Nm]
(send (send *robot* :rarm :collar-y) :max-joint-torque 0) ;; defaults to about 220[Nm]
(send (send *robot* :torso-waist-y) :max-joint-torque 100) ;; defaults to about 434[Nm]
(format t "[Warn] set torque limit (send (send *robot* :larm :collar-y) :max-joint-torque) => ~A~%" (send (send *robot* :larm :collar-y) :max-joint-torque))
(format t "[Warn] set torque limit (send (send *robot* :rarm :collar-y) :max-joint-torque) => ~A~%" (send (send *robot* :rarm :collar-y) :max-joint-torque))
(format t "[Warn] set torque limit (send (send *robot* :torso-waist-y) :max-joint-torque) => ~A~%" (send (send *robot* :torso-waist-y) :max-joint-torque))
;; (when (not (boundp '*jaxon_red*))
;;   (require "package://hrpsys_ros_bridge_tutorials/euslisp/jaxon_red-interface.l")
;;   (setq *jaxon_red* (jaxon_red))
;;   (setq *robot* *jaxon_red*)
;;   (setq *rfoot-plate* (make-cube 230 130 0.1))
;;   (send *rfoot-plate* :locate #f(15.5 -110 45.7286) :world)
;;   (send (send *robot* :rleg :end-coords) :assoc *rfoot-plate*)
;;   (setq *lfoot-plate* (make-cube 230 130 0.1))
;;   (send *lfoot-plate* :locate #f(15.5 110 45.7286) :world)
;;   (send (send *robot* :lleg :end-coords) :assoc *lfoot-plate*)
;;   )


;; set :analysis-level to :coords from :body
(send-all (send *robot* :links) :analysis-level :coords)

;; make racket-model
(setq *racket-coords* (make-cascoords :pos #f(0 0 50) :rpy (list 0 0 0)))

;;;; new model
(require "package://baseball/model/racket-model.l")
(setq *racket-model* (instance racket :init :rim 15))
(send *racket-model* :orient (deg2rad 90) :x :world)
(send *racket-model* :locate #f(0 15 520) :world)
(send *racket-coords* :assoc *racket-model*)
(setq *sweet-spot* (make-cascoords :pos #f(0 0 520) :rpy (list (deg2rad 180) 0 (deg2rad 90))))
(send *racket-coords* :assoc *sweet-spot*)
;;;;

;;;; old model
;; (let* ((a 125)
;;        (b 160)
;;        (pl))
;;   (mapc #'(lambda (i) (setq pl (append pl (list (float-vector (* a (cos (- (* (/ pi 25) i) (/ pi 2)))) (* b (sin (- (* (/ pi 25) i) (/ pi 2)))) 0))))) (my-range-2 1 48))
;;   (setq pl (append pl (list (float-vector (* a (cos (- (/ (* 48 pi) 25) (/ pi 2)))) -380 0) (float-vector (* a (cos (- (/ pi 25) (/ pi 2)))) -380 0) )))
;;   (setq *racket-model* (make-prism pl 30))
;;   )
;; (send *racket-model* :orient (deg2rad 90) :x)
;; (send *racket-model* :locate #f(0 15 380) :world
;; (send *racket-coords* :assoc *racket-model*)
;; (setq *sweet-spot* (make-cascoords :pos #f(0 0 380) :rpy (list (deg2rad 180) 0 (deg2rad 90))))
;; (send *racket-coords* :assoc *sweet-spot*)
;;;;

(setq *racket-grip* (make-cascoords :pos #f(0 0 50) :rpy (list (deg2rad 45) (deg2rad -45) 0)))
(send *racket-coords* :assoc *racket-grip*)

;; make punch target coodinates
(setq *punch-coords* (make-cascoords :rpy (list (deg2rad 90) (deg2rad -10) (deg2rad -90))))
(setq *punch-point* (make-cascoords :rpy (list (deg2rad -90) (deg2rad -80) 0)))
(send *punch-coords* :assoc *punch-point*)

;; make kick target coordinates
(setq *kick-coords* (make-cascoords))
(setq *kick-point* (make-cascoords :rpy (list -2.506 -0.548 -0.392)))
(send *kick-coords* :assoc *kick-point*)

;; make bat-model and batting target coodinates
;;;; new model
(require "package://baseball/model/bat-model.l")
(setq *bat-model* (instance bat :init))
(send *bat-model* :orient (deg2rad 90) :x)
(send *bat-model* :locate #f(0 15 520) :world)
;;;;

;;;; old model
;; (setq *bat-model* (make-solid-of-revolution '(#f(30 0 750) #f(30 0 450) #f(15 0 250) #f(15 0 21) #f(25 0 18) #f(25 0 0))))
;; (send *bat-model* :set-color #f(0 0.2 0.8))
;;;;

(setq *bat-coords* (make-cascoords :pos (float-vector 0 0 0) :rpy (list 0 0 0))) ;; yaw roll pitch
(send *bat-coords* :assoc *bat-model*)
(setq *bat-lhand-grip* (make-cascoords :pos #f(0 0 50) :rpy (list (deg2rad 30) (deg2rad -30) 0)))
(send *bat-coords* :assoc *bat-lhand-grip*)
(setq *bat-rhand-grip* (make-cascoords :pos (float-vector -30 -60 200) :rpy (list (deg2rad -10) (deg2rad 30) 0)))
(send *bat-coords* :assoc *bat-rhand-grip*)
(setq *hitting-point* (make-cascoords :pos (float-vector 0 0 600) :rpy (list (deg2rad 135) 0 (deg2rad 90))))
(send *bat-coords* :assoc *hitting-point*)

