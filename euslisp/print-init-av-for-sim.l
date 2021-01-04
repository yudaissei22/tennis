#!/usr/bin/env roseus

(setq *motion-choice* "forehand-volley-step")
(require "package://tennis/euslisp/motion-choice.l")
(cond ((equal *motion-choice* "forehand") (require "package://tennis/euslisp/jaxon-key-pose/jaxon-forehand-motion"))
      ((equal *motion-choice* "forehand-volley") (require "package://tennis/euslisp/jaxon-key-pose/jaxon-forehand-volley-motion"))
      ((equal *motion-choice* "backhand-volley") (require "package://tennis/euslisp/jaxon-key-pose/jaxon-backhand-volley-motion"))
      ((equal *motion-choice* "forehand-volley-step") (require "package://tennis/euslisp/jaxon-key-pose/jaxon-forehand-volley-motion"))
      ((equal *motion-choice* "backhand-volley-step") (require "package://tennis/euslisp/jaxon-key-pose/jaxon-backhand-volley-motion"))
      (t (print "error loading *motion-choice*")))
;; これ後で置いたほうがいい？
(comp::compile-file-if-src-newer (ros::resolve-ros-path "package://tennis/euslisp/nlopt_bspline_optimization.l") (ros::resolve-ros-path "package://tennis/euslisp/"))
(require "package://tennis/euslisp/nlopt_bspline_optimization.so")

(send *robot* :move-to (make-coords) :world)
(with-append-root-joint
  (link-list-with-robot-6dof *robot* (list (cdr (send *robot* :links))) :joint-class 6dof-joint)
  (progn 
    (load-forehand-volley-pose :pose (nth 0 *key-pose-id*))
    (setq *robot0* (copy-object *robot*))
    )
  )
(print (mapcar #'rad2deg (reverse (car (print (rpy-angle (send *robot* :worldrot)))))))

(nlopt-init :x-max 2.0 :x-hit 1.0 :id-max 14 :recursive-order 5 :use-all-joint t :use-append-root-joint t :interval-num 20 :support-polygon-margin (list 50.0 50.0 0 100 50) :epsilon-c 30 :mu 0.3 :use-final-pose nil :default-switching-list nil :use-6dof-p t)
(send *robot* :move-to (make-coords) :world)
(with-append-root-joint
  (link-list-with-robot-6dof *robot* (list (cdr (send *robot* :links))) :joint-class 6dof-joint)
  (calc-specific-time-force-zmp *p-orig* *x-min* :step-time 0.1 :lfoot-ik nil :rfoot-ik nil :fix-leg-coords nil))
(setq *robot1* (copy-object *robot*))

(send-all (flatten (send-all (send *robot0* :links) :bodies)) :set-color #f(1 0 0))
(send-all (flatten (send-all (send *robot1* :links) :bodies)) :set-color #f(0 0 1))
(objects (list *robot0* *robot1*))

(dolist (i (coerce (send *robot1* :angle-vector) cons))
  (format "~A " i)
  )
(format "~%" i)
(dolist (i (coerce (send *robot1* :worldpos) cons))
  (format "~A " i)
  )
(print (send *robot1* :worldrot))
