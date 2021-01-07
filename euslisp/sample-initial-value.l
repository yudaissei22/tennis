#!/usr/bin/env roseus
;; (setq *motion-choice* "forehand-volley-step")
(setq *motion-choice* "backhand-volley-step")

(setq *default-x-hit* 1.4)
(setq *x-max-of-p-orig* 2.0)

(comp::compile-file-if-src-newer (ros::resolve-ros-path "package://tennis/euslisp/qp-bspline-optimization.l") (ros::resolve-ros-path "package://tennis/euslisp/"))
(load "qp-bspline-optimization.so")
(qp-motion-optimize  :x-max *x-max-of-p-orig* :x-hit *default-x-hit* :maxvel-weight -1e-3 :unit-matrix-weight 1e-4 :minjerk-weight 1e0 :x-step 0.02 :id-max 14 :recursive-order 5 :use-margin 30 :use-all-joint t :use-append-root-joint t)
(setq *p-orig* (concatenate float-vector *ret* (float-vector 0.4 0.8 1.4)))
(setq *p-orig-file-header* (format nil ";; {{{ initial value: (qp-motion-optimize  :x-max ~A :x-hit ~A :maxvel-weight -1e-3 :unit-matrix-weight 1e-4 :minjerk-weight 1e0 :x-step 0.02 :id-max 14 :recursive-order 5 :use-margin 30 :use-all-joint t :use-append-root-joint t)~%(progn (setq *p-orig* " *x-max-of-p-orig* *default-x-hit* ))
(setq *p-orig-file-footer* (format nil ") (setq *x-max-of-p-orig* ~a))~%;; }}}" *x-max-of-p-orig*))
(with-open-file (f "/userdir/logs/qp-init-p-orig.l" :direction :output :if-exists :new-version) (format f "~a~a~a~%" *p-orig-file-header* *p-orig* *p-orig-file-footer*))
