;; choose the motion
(when (not (boundp '*motion-choice*))
  (print "Warning: *motion-choice* is set to \"forehand\" automatically.")
  (setq *motion-choice* "forehand"))

;; forehand
;; punch
;; kick
;; smash
;; batting
;; forehand-step

(defun load-motion (&key (motion-choice *motion-choice*))
 (cond
  ((equal motion-choice "forehand")
   (load "package://baseball/euslisp/jaxon-key-pose/jaxon-forehand-motion.l")
   (setq *limb* :rarm)
   (setq *load-pose-func* 'load-forehand-pose)
   (setq *key-pose-id* (list 1 3 4)))

  ((equal motion-choice "punch")
   (load "package://baseball/euslisp/jaxon-key-pose/jaxon-punch-motion.l")
   (setq *limb* :rarm)
   (setq *load-pose-func* 'load-punch-pose)
   (setq *key-pose-id* (list 1 3 2)))

  ((equal motion-choice "kick")
   (load "package://baseball/euslisp/jaxon-key-pose/jaxon-kick-motion.l")
   (setq *limb* :rleg)
   (setq *load-pose-func* 'load-kick-pose)
   (setq *key-pose-id* (list 2 3 4)))

  ((equal motion-choice "smash")
   (load "package://baseball/euslisp/jaxon-key-pose/jaxon-smash-motion.l")
   (setq *limb* :rarm)
   (setq *key-pose-id* (list 1 3 4))
   (setq *load-pose-func* 'load-smash-pose))

  ((equal motion-choice "batting")
   (load "package://baseball/euslisp/jaxon-key-pose/jaxon-batting-motion.l")
   (setq *limb* :larm)
   (setq *load-pose-func* 'load-batting-pose)
   (setq *key-pose-id* (list 1 3 4)))

  ((equal motion-choice "forehand-step")
   (load "package://baseball/euslisp/jaxon-key-pose/jaxon-forehand-motion.l")
   (setq *limb* :rarm)
   (setq *load-pose-func* 'load-forehand-pose)
   (setq *key-pose-id* (list 5 3 4)))

  ((equal motion-choice "kick-step")
   (load "package://baseball/euslisp/jaxon-key-pose/jaxon-kick-motion.l")
   (setq *limb* :rleg)
   (setq *load-pose-func* 'load-kick-pose)
   (setq *key-pose-id* (list 1 3 5)))

  ((equal motion-choice "batting-step")
   (load "package://baseball/euslisp/jaxon-key-pose/jaxon-batting-motion.l")
   (setq *limb* :larm)
   (setq *load-pose-func* 'load-batting-pose)
   (setq *key-pose-id* (list 5 3 4)))))

(load-motion :motion-choice *motion-choice*)
