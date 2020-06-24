(require "package://baseball/../../../../../rbrain/virtual_interpolator.l")
(require "package://baseball/../../../../../jsk/gnuplotlib.l")

(defun pos-list-interpolation-linear
  (pos-list ;; (list pos_1 pos_2 ... pos_N), pos_i is float-vector
   time-list ;; (list dtime_1 dtime_2 ... dtime_{N-1}), dtime_i is time[s] between time at pos_{i+1} - pos_i
   dt ;; dt [s]
   &key (initial-time 0.0)
        (neglect-first)
        (vel-vector-list)
        (acc-vector-list)
        (time-offset))
  (funcall 'pos-list-interpolation pos-list time-list dt :interpolator-class linear-interpolator)
  )

(defun pos-list-interpolation-minjerk
  (pos-list ;; (list pos_1 pos_2 ... pos_N), pos_i is float-vector
   time-list ;; (list dtime_1 dtime_2 ... dtime_{N-1}), dtime_i is time[s] between time at pos_{i+1} - pos_i
   dt ;; dt [s]
   &key (initial-time 0.0)
        (neglect-first)
        (vel-vector-list)
        (acc-vector-list)
        (time-offset))
  (funcall 'pos-list-interpolation pos-list time-list dt :interpolator-class minjerk-interpolator)
  )

(defun pos-list-interpolation-spline
  (pos-list ;; (list pos_1 pos_2 ... pos_N), pos_i is float-vector
   time-list ;; (list dtime_1 dtime_2 ... dtime_{N-1}), dtime_i is time[s] between time at pos_{i+1} - pos_i
   dt ;; dt [s]
   &key (initial-time 0.0)
        (neglect-first)
        (vel-vector-list)
        (acc-vector-list)
        (time-offset))
  (funcall 'pos-list-interpolation pos-list time-list dt :interpolator-class spline-interpolator)
  )

(defun pos-list-interpolation-spline-minjerk
  (pos-list ;; (list pos_1 pos_2 ... pos_N), pos_i is float-vector
   time-list ;; (list dtime_1 dtime_2 ... dtime_{N-1}), dtime_i is time[s] between time at pos_{i+1} - pos_i
   dt ;; dt [s]
   &key (div)
        (initial-time 0.0)
        (time-offset))
  (funcall 'pos-list-interpolate-spline-minjerk pos-list time-list dt)
  )

(defun pos-list-interpolation-delay-trajectory-generator
  (pos-list ;; (list pos_1 pos_2 ... pos_N), pos_i is float-vector
   time-list ;; (list dtime_1 dtime_2 ... dtime_{N-1}), dtime_i is time[s] between time at pos_{i+1} - pos_i
   dt ;; dt [s]
   &key (initial-time 0.0)
        (time-offset 0.2)
        (actual-dt dt))
  (funcall 'pos-list-interpolate-delay-trajectory-generator pos-list time-list dt :time-offset time-offset)
  )

;; you should use d and dt for d/dt as N
(defun my-minjerk-interpolator (dt d xs vs as xf vf af)
  (let* ((cons-flag (consp xs))
         (ret (list xs))
         (x_s (if cons-flag (coerce xs float-vector) xs))
         (v_s (if cons-flag (coerce vs float-vector) vs))
         (a_s (if cons-flag (coerce as float-vector) as))
         (x_f (if cons-flag (coerce xf float-vector) xf))
         (v_f (if cons-flag (coerce vf float-vector) vf))
         (a_f (if cons-flag (coerce af float-vector) af))
         (tau dt)
         (a0 x_s)
         (a1 v_s)
         (a2 (scale 0.5 a_s))
         (a3 (reduce #'v+ (list
                           (scale (/ 0.5 d) a_f)
                           (scale (/ -1.5 d) a_s)
                           (scale (/ -4.0 (expt d 2)) v_f)
                           (scale (/ -6.0 (expt d 2)) v_s)
                           (scale (/ 10.0 (expt d 3)) x_f)
                           (scale (/ -10.0 (expt d 3)) x_s))))
         (a4 (reduce #'v+ (list
                           (scale (/ -1.0 (expt d 2)) a_f)
                           (scale (/ 1.5 (expt d 2)) a_s)
                           (scale (/ 7.0 (expt d 3)) v_f)
                           (scale (/ 8.0 (expt d 3)) v_s)
                           (scale (/ -15.0 (expt d 4)) x_f)
                           (scale (/ 15.0 (expt d 4)) x_s))))
         (a5 (reduce #'v+ (list
                           (scale (/ 0.5 (expt d 3)) a_f)
                           (scale (/ -0.5 (expt d 3)) a_s)
                           (scale (/ -3.0 (expt d 4)) v_f)
                           (scale (/ -3.0 (expt d 4)) v_s)
                           (scale (/ 6.0 (expt d 5)) x_f)
                           (scale (/ -6.0 (expt d 5)) x_s)))))

    (while (not (eps= tau d (/ dt 10)))
      (setq pos (reduce #'v+ (list
                              a0
                              (scale tau a1)
                              (scale (expt tau 2) a2)
                              (scale (expt tau 3) a3)
                              (scale (expt tau 4) a4)
                              (scale (expt tau 5) a5))))
      (setq vel (reduce #'v+ (list
                              a1 (scale (* 2.0 tau) a2)
                              (scale (* 3.0 (expt tau 2)) a3)
                              (scale (* 4.0 (expt tau 3)) a4)
                              (scale (* 5.0 (expt tau 4)) a5))))
      (setq acc (reduce #'v+ (list
                              (scale 2 a2)
                              (scale (* 6.0 tau) a3)
                              (scale (* 12.0 (expt tau 2)) a4)
                              (scale (* 20.0 (expt tau 3)) a5))))
      (setq ret (append ret (list (if cons-flag (coerce pos cons) pos))))
      (setq tau (+ tau dt))
      )
    ret
    )
  )
