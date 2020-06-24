;; make range function
;; (my-range n) => (0 ... n-1)
(defun my-range (i)
  (let ((range-list))
    (dotimes (j i) (setq range-list (append range-list (list j))))
    range-list))

;; (my-range i j) => (i ... j)
(defun my-range-2 (i1 i2)
  (let ((range-list))
    (dotimes (j (1+ (- i2 i1))) (setq range-list (append range-list (list (+ j i1)))))
    range-list))

