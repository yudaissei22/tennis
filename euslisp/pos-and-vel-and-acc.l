(load "/home/future731/prog/euslib/jsk/gnuplotlib.l")
(load "bspline.l")
(load "../config/p-orig.l")

(setq *id-max* 14)
(setq *recursive-order* 5)

(setq a (instance basic-spline :init :id-max *id-max* :recursive-order *recursive-order* :x-min 0.0 :x-max 1.0))

(setq x-min 0)
(setq x-buf x-min)
(setq x-max 1)
(setq interval-num 1000)

(setq tm-list nil)
(setq val-list nil)
(setq dval-list nil)
(setq ddval-list nil)


(setq *rarm-collar-index* 17)
(setq *rarm-collar-coeff* (subseq *p-orig* (* *rarm-collar-index* *id-max*) (+ (* *rarm-collar-index* *id-max*) *id-max*)))
(while (<= x-buf x-max)
       (push x-buf tm-list)
       (push (send a :calc x-buf *rarm-collar-coeff*) val-list)
       (push (send a :calc-delta x-buf *rarm-collar-coeff* :n 1) dval-list)
       (push (send a :calc-delta x-buf *rarm-collar-coeff* :n 2) ddval-list)
       (setq x-buf (+ x-buf (+ (* (- x-max x-min) (/ 1.0 interval-num)) x-min))))
(reverse tm-list)
(reverse val-list)
(reverse dval-list)
(reverse ddval-list)

(graph-view (list val-list) tm-list :title "val")
;; (graph-view (list dval-list) tm-list :title "dval")
;; (graph-view (list ddval-list) tm-list :title "dval")
