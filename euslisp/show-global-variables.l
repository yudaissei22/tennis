#!/usr/bin/env roseus


(defun show-timing-variables ()
  (format t "x-min: ~A " *x-min*)
  (format t "x-hit: ~A " *default-x-hit*)
  (format t "x-max: ~A " *x-max*)
  (format t "~%")
  )
(defun show-bspline-variables ()
  (format t "id-max: ~A " *id-max*)
  (format t "recursive-order: ~A " *recursive-order*)
  (format t "~%")
  )
