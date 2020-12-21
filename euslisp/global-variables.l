#!/usr/bin/env roseus

;; x-min, x-hit, x-land, x-takeoff, x-max, id-max, recursive-order

;; if these variables are not set in p-orig.l, use these values as default
;; (defvar *x-land* nil)
;; (defvar *x-takeoff* nil)
(defvar *x-min* 0.0)
(defvar *x-hit* 1.0)
(defvar *x-max* 2.0)
(defvar *id-max* 14)
(defvar *recursive-order* 5)

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
