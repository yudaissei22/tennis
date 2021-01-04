;; choose the motion
(when (not (boundp '*motion-choice*))
  (print "Warning: *motion-choice* is set to \"forehand-volley-step\" automatically.")
  (setq *motion-choice* "forehand-volley-step"))

;; forehand
;; punch
;; kick
;; smash
;; batting
;; forehand-step
;; forehand-volley-step
;; backhand-volley-step

(defun load-motion (&key (motion-choice *motion-choice*))
 (cond
  ((equal motion-choice "forehand")
   (load "package://tennis/euslisp/jaxon-key-pose/jaxon-forehand-motion.l")
   (setq *limb* :rarm)
   (setq *vel-positive* t)
   (setq *load-pose-func* 'load-forehand-pose)
   (setq *key-pose-id* (list 1 3 4)))

  ((equal motion-choice "forehand-volley")
   (load "package://tennis/euslisp/jaxon-key-pose/jaxon-forehand-volley-motion.l")
   (setq *limb* :rarm)
   (setq *vel-positive* t)
   (setq *load-pose-func* 'load-forehand-volley-pose)
   (setq *key-pose-id* (list 1 3 4)))

  ((equal motion-choice "backhand-volley")
   (load "package://tennis/euslisp/jaxon-key-pose/jaxon-backhand-volley-motion.l")
   (setq *limb* :rarm)
   (setq *vel-positive* nil) ;; 裏拳
   (setq *load-pose-func* 'load-backhand-volley-pose)
   (setq *key-pose-id* (list 1 3 5)))

  ((equal motion-choice "punch")
   (load "package://tennis/euslisp/jaxon-key-pose/jaxon-punch-motion.l")
   (setq *limb* :rarm)
   (setq *vel-positive* t)
   (setq *load-pose-func* 'load-punch-pose)
   (setq *key-pose-id* (list 1 3 2)))

  ((equal motion-choice "kick")
   (load "package://tennis/euslisp/jaxon-key-pose/jaxon-kick-motion.l")
   (setq *limb* :rleg)
   (setq *vel-positive* t)
   (setq *load-pose-func* 'load-kick-pose)
   (setq *key-pose-id* (list 2 3 4)))

  ((equal motion-choice "smash")
   (load "package://tennis/euslisp/jaxon-key-pose/jaxon-smash-motion.l")
   (setq *limb* :rarm)
   (setq *vel-positive* t)
   (setq *key-pose-id* (list 1 3 4))
   (setq *load-pose-func* 'load-smash-pose))

  ((equal motion-choice "batting")
   (load "package://tennis/euslisp/jaxon-key-pose/jaxon-batting-motion.l")
   (setq *limb* :larm)
   (setq *vel-positive* t)
   (setq *load-pose-func* 'load-batting-pose)
   (setq *key-pose-id* (list 1 3 4)))

  ((equal motion-choice "forehand-step")
   (load "package://tennis/euslisp/jaxon-key-pose/jaxon-forehand-motion.l")
   (setq *limb* :rarm)
   (setq *vel-positive* t)
   (setq *load-pose-func* 'load-forehand-pose)
   (setq *key-pose-id* (list 5 3 4)))

  ((equal motion-choice "forehand-volley-step")
   (load "package://tennis/euslisp/jaxon-key-pose/jaxon-forehand-volley-motion.l")
   (setq *limb* :rarm)
   (setq *vel-positive* t)
   (setq *load-pose-func* 'load-forehand-volley-pose)
   (setq *key-pose-id* (list 1 3 5)))

  ((equal motion-choice "backhand-volley-step")
   (load "package://tennis/euslisp/jaxon-key-pose/jaxon-backhand-volley-motion.l")
   (setq *limb* :rarm)
   (setq *vel-positive* nil) ;; 裏拳
   (setq *load-pose-func* 'load-backhand-volley-pose)
   (setq *key-pose-id* (list 1 3 5)))

  ((equal motion-choice "kick-step")
   (load "package://tennis/euslisp/jaxon-key-pose/jaxon-kick-motion.l")
   (setq *limb* :rleg)
   (setq *vel-positive* t)
   (setq *load-pose-func* 'load-kick-pose)
   (setq *key-pose-id* (list 1 3 5)))

  ((equal motion-choice "batting-step")
   (load "package://tennis/euslisp/jaxon-key-pose/jaxon-batting-motion.l")
   (setq *limb* :larm)
   (setq *vel-positive* t)
   (setq *load-pose-func* 'load-batting-pose)
   (setq *key-pose-id* (list 5 3 4)))))

(load-motion :motion-choice *motion-choice*)
