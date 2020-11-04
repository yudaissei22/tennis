#!/usr/bin/env roseus

(require "package://tennis/euslisp/motion-choice.l")
(cond ((equal *motion-choice* "forehand") (require "package://tennis/euslisp/jaxon-key-pose/jaxon-forehand-pose"))
      ((equal *motion-choice* "forehand-volley") (require "package://tennis/euslisp/jaxon-key-pose/jaxon-forehand-volley-pose"))
      ((equal *motion-choice* "backhand-volley") (require "package://tennis/euslisp/jaxon-key-pose/jaxon-backhand-volley-pose"))
      (t (print "error loading *motion-choice*")))

(pose1)
(format t "initial pose~%")
(format t "~A~%" (mapcar #'deg2rad (coerce (send *robot* :angle-vector) cons)))
(format t "~A~%" (send (car (send *robot* :links)) :worldrot))