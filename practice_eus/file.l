(setq i 0)


(dotimes (id 5)
  (dump-loadable-structure "/yuda/home/~A.l" id)
  ;; (format nil "/yuda/home/~A.l" id)
  (setq i (+ i 1))
  )
