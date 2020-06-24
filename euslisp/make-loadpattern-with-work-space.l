(require "package://tennis/euslisp/virtual_interpolator.l")
(require "package://tennis/../../../../ik/qp-inverse-kinematics.l")
(require "package://tennis/euslisp/trajectory-interpolator.l")
(require "package://tennis/euslisp/preview-control-for-impulsive-force.l")
(require "package://tennis/euslisp/jaxon-key-pose/jaxon-forehand-motion.l")

(format t "You should call (jsk)!!!~%")
(format t "(load-tennis-pose)~%(setq *input-tm-list* (list 5 3.5 4.5))~%(make-loadpattern (list *lp1* *lp2* *lp3* *lp4*)~%    :tm-list *input-tm-list* ;;(list 0.5 0.4 0.4)~%    :dt 0.002~%    :thre (list 3 3) ;; lleg rleg~%    :rthre (list (deg2rad 3) (deg2rad 3)) ;; lleg rleg~%    :lp-file \"../config/tennis\"~%    :zmp-midpoint-list (list 0.65 0.575 0.45 0.4) ;; (list 0.6 0.62 0.55 0.4 0.3 0.2 0.4)~%    :av-interpolate-func 'pos-list-interpolation-spline-minjerk~%    :rc-interpolate-func 'pos-list-interpolation-spline-minjerk~%    :zmp-interpolate-func 'pos-list-interpolation-linear~%    :impact-tm (+ (nth 0 *input-tm-list*) (nth 1 *input-tm-list*))~%    :zmp-tm-list *input-tm-list* ;; (list 0.1 0.2 0.3 0.3 0.1 0.1)~%    :hit-point *sweet-spot*~%    :equipment-body *racket-coords*~%    :equipment-grip *racket-grip*~%    :limb :rarm~%    :w-space nil ;; (list 620 720)~%    :select-rpy 0 ;; either 0 or 1~%    :correction-term (list #f(0 0 0) 0)~%    :delay-time-offset 1~%    :use-preview-control nil~%    :use-qp t)~%")
;; fix-leg-to-coords *lfoot* :lleg

;; add data to list and make hrpsys sequence file
(defun make-one-pose-for-loadpattern (robot-av robot-pos robot-log &key (robot *robot*) (zmp (make-coords :pos (midpoint 0.6 (send *lfoot* :worldpos) (send *rfoot* :worldpos)))) (use-preview-control nil) (cog nil) (use-qp nil) (dt 0.002) (thre (list 3 3)) (rthre (list (deg2rad 3) (deg2rad 3))) (hit-point nil) (equipment-body) (equipment-grip) (limb :rarm) (ik-target-coords nil) (use-w-space nil) &allow-other-keys)
  (setq *ik-algorithm* :org)
  (send robot :orient (norm robot-log) robot-log :world)
  (send robot :locate robot-pos :world)
  (send robot :angle-vector robot-av)
  (send robot :rleg :inverse-kinematics *rfoot* :dt dt :revert-if-fail nil)
  (send robot :lleg :inverse-kinematics *lfoot* :dt dt :revert-if-fail nil)

  (if use-w-space
      (progn
        ;; (objects ik-target-coords)
        ;; if tennis motion, equipment-body means *racket-coords*
        (send (send *robot* limb :end-coords) :dissoc equipment-body)
        (send equipment-body :move-to ik-target-coords :world)
        (send equipment-body :transform (send hit-point :transformation equipment-body))
        (send robot limb :angle-vector (car (reverse *lp-limb-av-list*)))
        (if use-preview-control
            (progn
              (when use-qp
                (setq *ik-algorithm* :qp)
                )
              (let* ((target-coords (list *lfoot* *rfoot* equipment-grip))
                     (link-list (list :lleg :rleg limb))
                     (thre-w-space (append thre (list (nth 1 thre))))
                     (rthre-w-space (append rthre (list (nth 1 rthre)))))
                (with-move-target-link-list
                 (mt ll robot link-list)
                 (send robot
                       :fullbody-inverse-kinematics target-coords
                       :dt dt
                       :thre thre-w-space
                       :rthre rthre-w-space
                       :root-link-virtual-joint-weight #f(0.5 0.5 0.5 0.5 0.5 0.5) ;; root-link has 6DOF
                       :target-centroid-pos cog
                       :centroid-thre 5
                       :move-target mt
                       :link-list ll
                       :avoid-collision-distance 60
                       :avoid-collision-null-gain 5.0
                       :avoid-collision-joint-gain 0.8
                       :additional-weight-list (list (list (send robot :head-neck-p :child-link) 0)
                                                     (list (send robot :head-neck-y :child-link) 0))
                       :rotation-axis (list t t t)
                       :revert-if-fail nil
                       :min-loop 2 ;; minimum loop
                       :cog-null-space nil
                       :debug-view nil) ;;:no-message) ;;t)
                 ))
              )
          (progn
            (send robot limb :inverse-kinematics equipment-grip :dt dt :revert-if-fail nil :min-loop 2)
            )
          )
        )
    (progn
      (send equipment-body :move-to (send *robot* limb :end-coords :copy-worldcoords) :world)
      (send equipment-body :transform (send equipment-grip :transformation equipment-body))
      (send (send *robot* limb :end-coords) :assoc equipment-body)
      (when use-preview-control
        (when use-qp
          (setq *ik-algorithm* :qp)
          )
        (let* ((target-coords (list *lfoot* *rfoot*)))
          (with-move-target-link-list
           (mt ll robot '(:lleg :rleg))
           (send robot
                 :fullbody-inverse-kinematics target-coords
                 :dt dt
                 :thre thre
                 :rthre rthre
                 :root-link-virtual-joint-weight #f(0.5 0.5 0.5 0.5 0.5 0.5) ;; root-link has 6DOF
                 :target-centroid-pos cog
                 :centroid-thre 5
                 :move-target mt
                 :link-list ll
                 :avoid-collision-distance 60
                 :avoid-collision-null-gain 5.0
                 :avoid-collision-joint-gain 0.8
                 :additional-weight-list (list (list (send robot :head-neck-p :child-link) 0)
                                               (list (send robot :head-neck-y :child-link) 0))
                 :rotation-axis (list t t)
                 :revert-if-fail nil
                 :min-loop 2 ;; minimum loop
                 :cog-null-space nil
                 :debug-view nil) ;;:no-message) ;;t)
           ))
        )
      )
    )

  ;; check robot pose
  (send *irtviewer* :draw-objects)
  ;; (send *ri* :angle-vector (send robot :angle-vector) 2)

  (setq *lp-av-list* (append *lp-av-list* (list (send robot :angle-vector))))
  (setq *lp-limb-av-list* (append *lp-limb-av-list* (list (send robot limb :angle-vector))))
  (setq *lp-zmp-list* (append *lp-zmp-list* (list (send (send (send (car (send robot :links)) :copy-worldcoords) :transformation zmp) :worldpos))))
  (when hit-point
    (setq *hit-point-list* (append *hit-point-list* (list (send hit-point :copy-worldcoords))))
    )
  )


(defun write-loadpattern-file (file tm-list data-list &key (unit-func #'(lambda (var) var)))
  (with-open-file
    (f file :direction :output)
    ;; (mapc #'(lambda (i) (make-one-pose-for-loadpattern (nth i lp-av-after) (nth i lp-rc-pos-after) (nth i lp-rc-rot-after)) (print i)) (my-range (length *lp-tm-list*)))
    (dotimes (j (length tm-list))
      (progn
        (format f "~a" (nth j tm-list))
        (dotimes (i (length (nth j data-list))) (format f " ~a" (funcall unit-func (elt (nth j data-list) i))))
        (format f "~%")
        )
      )
    )
  )


;; calculate velocity and acceleration from (list x_k x_k-1 x_k-2) (x should be float-vector or list)
;; return (list x_k v_k a_k)
(defun calculate-velocity-and-acceleration (dt x-list)
  (let* ((cons-flag (consp (nth 0 x-list)))
         (x_k (if cons-flag (coerce (nth 0 x-list) float-vector) (nth 0 x-list)))
         (x_k-1 (if cons-flag (coerce (nth 1 x-list) float-vector) (nth 1 x-list)))
         (x_k-2 (if cons-flag (coerce (nth 2 x-list) float-vector) (nth 2 x-list)))
         (v_k (scale (/ 1.0 dt) (v- x_k x_k-1)))
         (v_k-1 (scale (/ 1.0 dt) (v- x_k-1 x_k-2)))
         (a_k (scale (/ 1.0 dt) (v- v_k v_k-1))))
    (setf (nth 1 x-list) (if cons-flag (coerce v_k cons) v_k))
    (setf (nth 2 x-list) (if cons-flag (coerce a_k cons) a_k))
    )
  x-list
  )


(defun make-rpy-continuous (rpy_k rpy_k-1)
  (setq rpy_k (let ((cnt 0))
                (dolist (j (mapcar #'- rpy_k rpy_k-1) rpy_k)
                  (cond
                   ((> j 6.0) (setf (nth cnt rpy_k) (- (nth cnt rpy_k) (* 2 pi))))
                   ((< j -6.0) (setf (nth cnt rpy_k) (+ (nth cnt rpy_k) (* 2 pi))))
                   (t)
                   )
                  (setq cnt (1+ cnt)))))
  )


(defun make-loadpattern
  (mocap-pose-list
   &rest args
   &key (robot *robot*)
        (tm-list (list 4 4 3))
        (dt 0.002)
        (lp-file "../config/loadpattern")
        (zmp-midpoint-list (list 0.6 0.52 0.44 0.4)) ;; (make-coords :pos (midpoint i (send *lfoot* :worldpos) (send *rfoot* :worldpos)))
        (av-interpolate-func 'pos-list-interpolation-spline-minjerk) ;; pos-list-interpolate-delay-trajectory-generator :time-offset 0.01
        (rc-interpolate-func 'pos-list-interpolation-linear)
        (zmp-interpolate-func 'pos-list-interpolation-linear) ;; pos-list-interpolate-spline-minjerk
        (impact-tm (+ (nth 0 tm-list) (nth 1 tm-list)) )
        (zmp-tm-list tm-list)
        (delay-time-offset 0.01)
        (w-space nil) ;; (list t_s t_f) ex) t_s=3000, t_f=3600 at time-list=(list 5.0 3.5 4.5)
        (correction-term (list #f(0 0 0) 0))
        (select-rpy 1) ;; nth 0 (or 1) (rpy-angle (send coords :worldrot))
        &allow-other-keys
        )
  (setq *lp-av-list* '())
  (setq *lp-limb-av-list* '())
  (setq *lp-tm-list* '())
  (setq *lp-zmp-list* '())
  (setq *hit-point-target-ik* '())
  (setq *max-tm* (apply #'+ tm-list)) (setq *impact-tm* impact-tm) ;; for preview-control
  (let* (
         (lp-av-before '()) (lp-av-after '())
         (lp-rc-pos-before '()) (lp-rc-rot-before '())
         (lp-rc-pos-after '()) (lp-rc-rot-after '())
         (lp-zmp-mp-before '()) (lp-zmp-mp-after '())
         (t_s-w-space nil) (t_f-w-space nil)
         (pos_s-w-space) (pos_f-w-space) (tmppos-list-w-space)
         (rpy_s-w-space) (rpy_f-w-space) (tmprpy-list-w-space)
         (av_s-c-space)
         )
    ;; interpolation for angle-vector, root-coords and zmp
    (dolist (pl mocap-pose-list) (setq lp-av-before (append lp-av-before (list (nth 1 pl)))))
    (dolist (cl mocap-pose-list) (setq lp-rc-pos-before (append lp-rc-pos-before (list (send (nth 5 cl) :worldpos)))))
    (dolist (cl mocap-pose-list) (setq lp-rc-rot-before (append lp-rc-rot-before (list (matrix-log (send (nth 5 cl) :worldrot))))))
    ;; (dolist (zl zmp-midpoint-list) (setq lp-zmp-mp-before (append lp-zmp-mp-before (list (float-vector zl))))) ;; midpoint
    (dolist (zl zmp-midpoint-list) (setq lp-zmp-mp-before (append lp-zmp-mp-before (list (midpoint zl (send *lfoot* :worldpos) (send *rfoot* :worldpos)) )))) ;; midpoint float-vector

    (setq temp-list (funcall av-interpolate-func lp-av-before tm-list dt))
    (setq lp-av-after (nth 1 temp-list))
    (setq *lp-tm-list* (nth 3 temp-list))
    (setq lp-rc-pos-after (nth 1 (funcall rc-interpolate-func lp-rc-pos-before tm-list dt :time-offset delay-time-offset)))
    (setq lp-rc-rot-after (nth 1 (funcall rc-interpolate-func lp-rc-rot-before tm-list dt :time-offset delay-time-offset)))
    (setq lp-zmp-mp-after (nth 1 (funcall zmp-interpolate-func lp-zmp-mp-before tm-list dt :time-offset delay-time-offset))) ;; midpoint and midpoint float-vector
    (setq *lp-zmp-mp-after* lp-zmp-mp-after)


    ;; if *hit-point-list* already exists, then you can use work space interpolation when swinging racket or bat
    (when (consp (and (boundp '*hit-point-list*) w-space))
      (let ((tmp-t_s (nth 0 w-space))
            (tmp-t_f (nth 1 w-space)))
        (setq t_s-w-space (if (< tmp-t_s 2) 2 tmp-t_s))
        (setq t_f-w-space (if (> tmp-t_f (length *lp-tm-list*)) (- (length *lp-tm-list*) 1) tmp-t_f))
        )

      ;; starting pos work space interpolation
      (setq pos_s-w-space (list (send (nth t_s-w-space *hit-point-list*) :worldpos) (send (nth (- t_s-w-space 1) *hit-point-list*) :worldpos) (send (nth (- t_s-w-space 2) *hit-point-list*) :worldpos)))
      (calculate-velocity-and-acceleration dt pos_s-w-space)

      ;; starting rpy work space interpolation
      (let ((tmp-rpy_k (nth 0 (rpy-angle (send (nth t_s-w-space *hit-point-list*) :worldrot))))
            (tmp-rpy_k-1 (nth 0 (rpy-angle (send (nth (- t_s-w-space 1) *hit-point-list*) :worldrot))))
            (tmp-rpy_k-2 (nth 0 (rpy-angle (send (nth (- t_s-w-space 2) *hit-point-list*) :worldrot)))))
        (make-rpy-continuous tmp-rpy_k-1 tmp-rpy_k-2)
        (make-rpy-continuous tmp-rpy_k tmp-rpy_k-1)
        (setq rpy_s-w-space (list tmp-rpy_k tmp-rpy_k-1 tmp-rpy_k-2))
        )
      (calculate-velocity-and-acceleration dt rpy_s-w-space)

      ;; ;; (TO DO calculate trajectory length and then total time ?)
      ;; finishing pos work space interpolation
      (setq pos_f-w-space (list (send (nth t_f-w-space *hit-point-list*) :worldpos) (send (nth (- t_f-w-space 1) *hit-point-list*) :worldpos) (send (nth (- t_f-w-space 2) *hit-point-list*) :worldpos)))
      (calculate-velocity-and-acceleration dt pos_f-w-space)

      ;; finishing rpy work space interpolation (rpy-angle)
      (let ((tmp-rpy_k (nth select-rpy (rpy-angle (send (nth t_f-w-space *hit-point-list*) :worldrot))))
            (tmp-rpy_k-1 (nth select-rpy (rpy-angle (send (nth (- t_f-w-space 1) *hit-point-list*) :worldrot))))
            (tmp-rpy_k-2 (nth select-rpy (rpy-angle (send (nth (- t_f-w-space 2) *hit-point-list*) :worldrot)))))
        (make-rpy-continuous tmp-rpy_k-1 tmp-rpy_k-2)
        (make-rpy-continuous tmp-rpy_k tmp-rpy_k-1)
        (setq rpy_f-w-space (list tmp-rpy_k tmp-rpy_k-1 tmp-rpy_k-2))
        )
      (calculate-velocity-and-acceleration dt rpy_f-w-space)

      ;; workspace position minjerk interpolation (correction-term reflects changing hit point)
      (setq tmppos-list-w-space (my-minjerk-interpolator dt (* (- t_f-w-space t_s-w-space) dt) (nth 0 pos_s-w-space) (nth 1 pos_s-w-space) (nth 2 pos_s-w-space) (v+ (nth 0 pos_f-w-space) (nth 0 correction-term)) (nth 1 pos_f-w-space) (nth 2 pos_f-w-space)))
      ;; workspace rpy minjerk interpolation
      (let* ((tmp-rpy_f-w-space-0 (nth 0 rpy_f-w-space)))
        (setf (nth 1 tmp-rpy_f-w-space-0) (+ (nth 1 tmp-rpy_f-w-space-0) (deg2rad (nth 1 correction-term))))
        (setq tmprpy-list-w-space (my-minjerk-interpolator dt (* (- t_f-w-space t_s-w-space) dt) (nth 0 rpy_s-w-space) (nth 1 rpy_s-w-space) (nth 2 rpy_s-w-space) tmp-rpy_f-w-space-0 (nth 1 rpy_f-w-space) (nth 2 rpy_f-w-space)))
        )

      ;; make coords-list from position and rpy list
      (mapc #'(lambda (i)
                (setq *hit-point-target-ik* (append *hit-point-target-ik* (list
                                                                           (make-coords :pos (nth i tmppos-list-w-space) :rpy (nth i tmprpy-list-w-space))
                                                                           ))))
            (my-range (length tmppos-list-w-space)))
      )

    (setq *hit-point-list* '())
    ;; make load-pattern list
    (if w-space
        (progn
          (mapc #'(lambda (i)
                    (cond
                     ((< i t_s-w-space)
                      (apply 'make-one-pose-for-loadpattern (nth i lp-av-after) (nth i lp-rc-pos-after) (nth i lp-rc-rot-after) :zmp (make-coords :pos (nth i lp-zmp-mp-after)) :cog (if (boundp '*cog-list*) (nth i *cog-list*) nil) :use-w-space nil args)
                      )
                     ((< i t_f-w-space)
                      (apply 'make-one-pose-for-loadpattern (nth i lp-av-after) (nth i lp-rc-pos-after) (nth i lp-rc-rot-after) :zmp (make-coords :pos (nth i lp-zmp-mp-after)) :cog (if (boundp '*cog-list*) (nth i *cog-list*) nil) :ik-target-coords (nth (- i t_s-w-space) *hit-point-target-ik*) :use-w-space t args)
                      )
                     ((eq i t_f-w-space)

                      ;; interpolate in C-space using final angle-vector pos vel acc (in final condition) and last key-pose, then append to lp-av-after, replacing old lp-av-after into interpolated angle-vector list
                      (let ((tmp-av_s-list (list (nth (- i 1) *lp-av-list*) (nth (- i 2) *lp-av-list*) (nth (- i 3) *lp-av-list*)))
                            (new-av-list)
                            (angle-zero-vector (coerce (make-list (length (send robot :angle-vector)) :initial-element 0) float-vector)))
                        (calculate-velocity-and-acceleration dt tmp-av_s-list)
                        (setq new-av-list (my-minjerk-interpolator dt (- *max-tm* (* t_f-w-space dt)) (nth 0 tmp-av_s-list) (nth 1 tmp-av_s-list) (nth 2 tmp-av_s-list) (nth (- (length *lp-tm-list*) 1) lp-av-after) angle-zero-vector angle-zero-vector)) ;; finishing vel and acc needs to be zero
                        (setq new-av-list (cdr new-av-list)) ;; cdr is needed because initial angle-vector is already used
                        (setq new-av-list (append new-av-list (list (car (reverse new-av-list)))))
                        (setq lp-av-after (subst new-av-list (nthcdr i lp-av-after) lp-av-after)) ;;replace angle-vector list
                        (apply 'make-one-pose-for-loadpattern (nth i lp-av-after) (nth i lp-rc-pos-after) (nth i lp-rc-rot-after) :zmp (make-coords :pos (nth i lp-zmp-mp-after)) :cog (if (boundp '*cog-list*) (nth i *cog-list*) nil) :use-w-space nil args)
                        )
                      )
                     (t
                      (apply 'make-one-pose-for-loadpattern (nth i lp-av-after) (nth i lp-rc-pos-after) (nth i lp-rc-rot-after) :zmp (make-coords :pos (nth i lp-zmp-mp-after)) :cog (if (boundp '*cog-list*) (nth i *cog-list*) nil) :use-w-space nil args)
                      )
                     )
                    ;;(if (eq (mod i 100) 0) (print i))
                    (print i)
                    )
                 (my-range (length *lp-tm-list*)))
          )
      (progn
        (mapc #'(lambda (i)
                  ;; (make-one-pose-for-loadpattern (nth i lp-av-after) (nth i lp-rc-pos-after) (nth i lp-rc-rot-after) :zmp (make-coords :pos (midpoint (aref (nth i lp-zmp-mp-after) 0) (send *lfoot* :worldpos) (send *rfoot* :worldpos)))) ;; midpoint
                  (apply 'make-one-pose-for-loadpattern (nth i lp-av-after) (nth i lp-rc-pos-after) (nth i lp-rc-rot-after) :zmp (make-coords :pos (nth i lp-zmp-mp-after)) :cog (if (boundp '*cog-list*) (nth i *cog-list*) nil) args) :use-w-space nil ;; midpoint float-vector
                  (if (eq (mod i 100) 0) (print i))
                  )
              (my-range (length *lp-tm-list*)))
        )
      )

    ;; write pos file
    (write-loadpattern-file (concatenate string lp-file ".pos") *lp-tm-list* *lp-av-list* :unit-func 'deg2rad)
    ;; write zmp file
    (write-loadpattern-file (concatenate string lp-file ".zmp") *lp-tm-list* *lp-zmp-list* :unit-func #'(lambda (i) (/ i 1000)))
    (with-open-file
     (f (concatenate string lp-file ".optionaldata") :direction :output)
     (dotimes (i (length *lp-tm-list*)) (format f "~a 1 1 0 0 1 1 1 1~%" (nth i *lp-tm-list*)))
     ;; (dotimes (i (length *lp-tm-list*)) (format f "~a 1 1 0 0 5 5 5~%" (nth i *lp-tm-list*)))
     )
    )
  (setq *ik-algorithm* :org)
  )

