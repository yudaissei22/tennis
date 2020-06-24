;; racket-model.l
(defun my-range-2 (i1 i2)
    (let ((range-list))
      (dotimes (j (1+ (- i2 i1))) (setq range-list (append range-list (list (+ j i1)))))
      range-list))


(defclass racket
  :super cascaded-link
  :slots ())


(defmethod racket
  (:init (&key (gripper t) (rim 20) (surface t) (move-origin))
   (send-super :init)
   (let ((bds (send self :make-racket :gripper gripper :rim rim :surface surface)))
     ;; move origin
     (when move-origin
       (send (car bds) :orient pi/2 :x)
       (send (car bds) :locate (float-vector 0 15 520) :world))
     (setq l0 (instance bodyset-link :init (make-cascoords)
                        :bodies bds)))
   (setq links (list l0))
   (send self :assoc (elt links 0))
   (send self :init-ending)
   (send self :name "racket"))


  (:make-racket (&key (a 140) (b 180) (thickness 30) (rim) (gripper) (surface))
   (let* ((pl nil)
          (gripper-color (float-vector 0.05 0.05 0.05))
          (racket-color  (float-vector 0.1 0.1 0.5))
          racket pp ps)
     (mapc #'(lambda (i) (setq pl (append pl
                                          (list
                                           (float-vector (* a (cos (- (* (/ pi 25) i) (/ pi 2))))
                                                         (* b (sin (- (* (/ pi 25) i) (/ pi 2))))
                                                         0))))) (my-range-2 5 45))
     (when rim
       (mapc #'(lambda (i) (setq pp (append pp
                                            (list
                                             (float-vector (* (- a rim) (cos (- (* (/ pi 25) i) (/ pi 2))))
                                                           (* (- b rim) (sin (- (* (/ pi 25) i) (/ pi 2))))
                                                           -20))))) (my-range-2 2 48)))
     (cond
      (gripper
       (setq pl
             (append pl
                     (list
                      (float-vector (* a (cos (- (/ (* 46.5 pi) 25) (/ pi 2)))) -180 0)
                      (float-vector (* a (cos (- (/ (* 47.5 pi) 25) (/ pi 2)))) -200 0)
                      (float-vector (* a (cos (- (/ (* 49 pi) 25) (/ pi 2)))) -300 0)
                      ;;(float-vector (* a (cos (- (/ (* 49 pi) 25) (/ pi 2)))) -520 0)
                      ;;(float-vector (* a (cos (- (/ pi 25) (/ pi 2)))) -520 0)
                      (float-vector (* a (cos (- (/ pi 25) (/ pi 2)))) -300 0)
                      (float-vector (* a (cos (- (/ (* 2.5 pi) 25) (/ pi 2)))) -200 0)
                      (float-vector (* a (cos (- (/ (* 3.5 pi) 25) (/ pi 2)))) -180 0)))))


      (t
       (setq pl
             (append pl
                     (list
                      (float-vector (* a (cos (- (/ (* 46.5 pi) 25) (/ pi 2)))) -180 0)
                      (float-vector (* a (cos (- (/ (* 47.5 pi) 25) (/ pi 2)))) -200 0)
                      (float-vector (* a (cos (- (/ (* 49 pi) 25) (/ pi 2)))) -300 0)
                      (float-vector (* a (cos (- (/ (* 49 pi) 25) (/ pi 2)))) -520 0)
                      (float-vector (* a (cos (- (/ pi 25) (/ pi 2)))) -520 0)
                      (float-vector (* a (cos (- (/ pi 25) (/ pi 2)))) -300 0)
                      (float-vector (* a (cos (- (/ (* 2.5 pi) 25) (/ pi 2)))) -200 0)
                      (float-vector (* a (cos (- (/ (* 3.5 pi) 25) (/ pi 2)))) -180 0))))))


     (setq racket (make-prism (nreverse pl) thickness))
     (when gripper
       (setq gripper
             (make-prism (nreverse
                          (list
                           (float-vector (* a (cos (- (/ (* 49 pi) 25) (/ pi 2)))) -300 0)
                           (float-vector (* a (cos (- (/ (* 49 pi) 25) (/ pi 2)))) -520 0)
                           (float-vector (* a (cos (- (/ pi 25) (/ pi 2)))) -520 0)
                           (float-vector (* a (cos (- (/ pi 25) (/ pi 2)))) -300 0)))
                         thickness))
       (send gripper :set-color gripper-color))

     (when rim
       (setq racket (body- racket (make-prism (nreverse pp) (+ thickness 40)))))
     (send racket :set-color racket-color)
     (when (and rim surface)
       (mapc #'(lambda (i) (setq ps (append ps
                                            (list
                                             (float-vector (* (- a (/ rim 2.0)) (cos (- (* (/ pi 25) i) (/ pi 2))))
                                                           (* (- b (/ rim 2.0)) (sin (- (* (/ pi 25) i) (/ pi 2))))
                                                           (* thickness (/ 2.0 5.0)) ))))) (my-range-2 2 48))
       (setq surface (make-prism (nreverse ps) (/ thickness 5.0)))
       (send surface :set-color #f(1 1 1)))

     ;;
     (let ((gripper-coords (make-cascoords)) (target-coords (make-cascoords)))
       ;; move gripper-coords
       (send gripper-coords :translate (float-vector 0 -450 15))
       (send gripper-coords :rotate -pi/2 :x)
       (send gripper-coords :rotate pi/2 :z)
       ;; move target-coords
       (send target-coords :translate (float-vector 0 0 15))
       (send target-coords :rotate -pi/2 :x)
       (send target-coords :rotate pi/2 :z)
       ;;
       (send self :put :gripper gripper-coords)
       (send racket :assoc gripper-coords)
       (send self :put :target target-coords)
       (send racket :assoc target-coords))
     ;;
     (if (and rim surface)
         (progn
           (send racket :assoc gripper)
           (send racket :assoc surface)
           (list racket gripper surface))

       (progn
         (send racket :assoc gripper)
         (list racket gripper))))))





#|
;; If you make racket model (wrl), you may need to comment :assoc command
(setq a (instance racket :init))
(setq gl-bodies (mapcar #'(lambda (x) (gl::make-glvertices-from-faceset x)) (send a :bodies)))
(setq glv (send (car gl-bodies) :append-glvertices (cdr gl-bodies)))
(gl::write-wrl-from-glvertices "racket.wrl" glv :scale 0.001)

;;
(setq cds
      (send
       (send (send (send *jaxon_red* :rarm_link7_lk) :worldcoords)
             :transformation (send *jaxon_red* :rarm :end-coords :worldcoords))
       :transformation
       (send (send *racket-grip* :worldcoords) :transformation (send *racket-model* :worldcoords))))
(setq ret (rotation-angle (send cds :worldrot)))

(progn
  (format t "translation ~12,12F ~12,12F ~12,12F~%" (/ (elt (send cds :worldpos) 0) 1000) (/ (elt (send cds :worldpos) 1) 1000) (/ (elt (send cds :worldpos) 2) 1000))
  (format t "rotation ~12,12F ~12,12F ~12,12F ~12,12F~%" (elt (cadr ret) 0) (elt (cadr ret) 1) (elt (cadr ret) 2) (car ret)))

|#
