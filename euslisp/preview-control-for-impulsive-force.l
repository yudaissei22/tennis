(require "package://euslisp/jskeus/irteus/demo/walk-motion.l")
(require "package://baseball/../../../../../jsk/gnuplotlib.l")

;; tennis ball: about 60g (0.06kg)
;; ball speed: about 100km/h (27.8m/s)
;; impuct time: about 0.004s
;; mv - mv0 = ft
;; external force (y): 835N
(defun preview-control-for-impulsive-force
  (&key (preview-controller-class preview-controller)
        (q 1) (r 1e-6)
        (ext-force-time 0.004) ;; [s]
        (ext-force-height 800) ;; [m]
        (ext-force-y -501)) ;; [N]
  (let* ((dt 0.0020) (max-tm (if (boundp '*max-tm*) *max-tm* 5))
         (tm-list (mapcar #'(lambda (x) (* x dt))
                          (let ((i 0)) (mapcar #'(lambda (x) (incf i)) (make-list (+ (round (/ max-tm dt)) 1))))))
         (impact-tm (if (boundp '*impact-tm*) *impact-tm* (/ max-tm 2)))
         (mg (* 130 9.8))
         (i -1)
         (ref-zmp-list
          (mapcar #'(lambda (tm)
                      (v+
                       (float-vector
                        0 ;; x
                        (cond
                         ((< tm impact-tm) 0)
                         ((< tm (+ ext-force-time impact-tm))
                          (/ (* -1 ext-force-height #|(* 1e-3 ext-force-height)|# ext-force-y) mg)) ;; y, ZMP difference according to disturbance force
                         (t 0))
                        0) ;; z
                       (if (boundp '*lp-zmp-mp-after*) (progn (incf i) (nth i *lp-zmp-mp-after*)) (float-vector 100 100 0))) ;; neutoral zmp
                      )
                  tm-list))
         (pc (instance preview-control-cart-table-cog-trajectory-generator :init dt 800 :delay 1.0 :q q :r r :preview-controller-class preview-controller-class :init-xk (car ref-zmp-list)))
         zmp-list cog-y-list ref-zmp-list2
         ret)
    (setq *cog-list* '())
    (mapcar
     #'(lambda (ret tm)
         ;; x
         ;; (push (elt (caddr ret) 0) zmp-list) ;; zmp ;; this zmp is "zmp as a table-cart model"
         ;; (push (elt (cadr ret) 0) cog-y-list)                   ;; cog
         ;; (push (elt (car ret) 0) ref-zmp-list2) ;; ref zmp ;; ref-zmp-list2 should be equal to ref-zmp-list
         ;; y
         (push (elt (caddr ret) 1) zmp-list) ;; zmp ;; this zmp is "zmp as a table-cart model"
         (push (elt (cadr ret) 1) cog-y-list)                   ;; cog-y
         (push (cadr ret) *cog-list*)                   ;; cog
         (push (elt (car ret) 1) ref-zmp-list2) ;; ref zmp ;; ref-zmp-list2 should be equal to ref-zmp-list
         )
     (send pc :pass-preview-controller ref-zmp-list) tm-list)
    (setq *cog-list* (reverse *cog-list*))
    (with-open-file
     (f "/tmp/preview-control-for-impulsive-force.dat" :direction :output)
     (mapcar #'(lambda (tm zmp cog refzmp)
                 (format f "~A ~A ~A ~A~%" tm zmp cog refzmp))
             tm-list (reverse zmp-list) (reverse cog-y-list) (reverse ref-zmp-list2))
     )
    (unless (or (null x::*display*) (= x::*display* 0))
      (let ((gp-command-list (list
                              "set xlabel 'Time [s]';"
                              "set ylabel 'ZMP Y [mm]';"
                              "plot '/tmp/preview-control-for-impulsive-force.dat' using 1:2 title 'cart zmp' with lines;"
                              "replot '/tmp/preview-control-for-impulsive-force.dat' using 1:3 title 'cog' with lines;"
                              "replot '/tmp/preview-control-for-impulsive-force.dat' using 1:4 title 'refzmp' with lines;"
                              "pause -1;")))
        (unix:system (format nil "gnuplot -e \"~A\""
                             (let ((str "")) (dolist (gpc gp-command-list) (setq str (format nil "~A ~A" str gpc))) str)))))
    (mapc #'(lambda (tm zmp cog refzmp)
              (list :time tm :zmp zmp :cog cog :refzmp refzmp))
          tm-list (reverse zmp-list) (reverse cog-y-list) (reverse ref-zmp-list2))
    )
  )
