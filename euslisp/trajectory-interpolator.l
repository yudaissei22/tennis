;; assoc end-coords
;; fullbody-ik (move-target should be changed)

;; Hoff and Arbib's interpolation with boundary condition

(require "package://tennis/euslisp/my-range.l")

(defun hoffarbib-workspace-interpolation
  (pos vel acc dt
   tmp-remain-time tmp-goal-acc tmp-goal-vel tmp-goal-pos)
  ;; Hoff and Arbib's interpolation with boundary condition
  (let ((jerk (v+
               (v+ (scale (/ 3.0 tmp-remain-time) (v- tmp-goal-acc (scale 3 acc)))
                   (scale (/ -24.0 (expt tmp-remain-time 2)) (v+ tmp-goal-vel (scale 1.5 vel)))
                   )
               (scale (/ 60.0 (expt tmp-remain-time 3)) (v- tmp-goal-pos pos))
               ))
        )
    (setq acc (v+ acc (scale dt jerk))
          vel (v+ vel (scale dt acc))
          pos (v+ pos (scale dt vel)))
    )
  )

(defun delay-trajectory-interpolation
  (total-time remain-time
   pos vel acc
   goal-pos goal-vel goal-acc
   &key (dt 0.002) (time-offset 0.2))

  )

(defun make-next-swing-trajectory-in-workspace
  (total-time remain-time
   current-coords p-vel p-acc r-vel r-acc goal-coords goal-p-vel
   &key (robot *robot*) (dt 0.002) (time-offset 0.2) (goal-p-acc #f(0 0 0)) (goal-r-vel #f(0 0 0)) (goal-r-acc #f(0 0 0)))

  (let* ((pos (send current-coords :worldpos))
         (goal-pos (send goal-coords :worldpos))
         (rot (matrix-log (send current-coords :worldrot)))
         (goal-rot (matrix-log (send goal-coords :worldrot)))
         (next-pos) (setq next-rot)
         )

    (setq next-pos (delay-trajectory-interpolation total-time remain-time pos p-vel p-acc goal-pos goal-p-vel goal-p-acc :dt 0.002 :time-offset 0.2))
    (setq next-rot (delay-trajectory-interpolation total-time remain-time rot r-vel r-acc goal-rot goal-r-vel goal-r-acc :dt 0.002 :time-offset 0.2))
    (make-coords :pos next-pos :rot (matrix-exponent next-rot))
    )
  ;; (send robot :orient (norm robot-log) robot-log :world)
  ;; (send robot :locate robot-pos :world)
  ;; (send robot :angle-vector robot-av)
  ;; (send robot :rleg :inverse-kinematics *rfoot*)
  ;; (send robot :lleg :inverse-kinematics *lfoot*)
  )



(defun path-generator (pos vel acc goal-pos goal-vel goal-acc tau tm-f)
  (let ((a0) (a1) (a2) (a3) (a4) (a5))
    ;;(print pos)
    ;;(print goal-pos)
    (setq a0 pos
          a1 (scale (* tau tm-f) vel)
          a2 (scale (* (expt (* tau tm-f) 2) 0.5) acc)
          a3 (scale (expt tau 3)
                    (reduce #'v+
                            (list
                             (scale (* 0.5 (expt tm-f 2)) (v- goal-acc (scale 3 acc)))
                             (scale (* -4.0 tm-f) (v+ goal-vel (scale 1.5 vel)))
                             (scale 10.0 (v- goal-pos pos)))))
          a4 (scale (expt tau 4)
                    (reduce #'v+
                            (list
                             (scale (expt tm-f 2) (v- (scale 1.5 acc) goal-acc))
                             (scale (* 8.0 tm-f) (v+ (scale 0.875 goal-vel) vel))
                             (scale -15.0 (v- goal-pos pos)))))
          a5 (scale (expt tau 5)
                    (reduce #'v+
                            (list
                             (scale (* 0.5 (expt tm-f 2)) (v- goal-acc acc))
                             (scale (* -3.0 tm-f) (v+ goal-vel vel))
                             (scale 6.0 (v- goal-pos pos)))))
          )
    (reduce #'v+ (list a0 a1 a2 a3 a4 a5))
    )
  )

#|
;; sign inversion happend in 2834
(let* ((start-time 3000)
       (stop-time 3800)
       (dt 0.002))
  (calc-vel-acc 0.002 *hit-point-list*)
  (print "calculation finished!")
  (test-show-trajectory (* (- stop-time start-time) dt) (nth start-time *hp-pos-list*) (nth start-time *hp-vel-list*) (nth start-time *hp-acc-list*) (nth start-time *hp-rpy-list*) (nth start-time *hp-rpyvel-list*) (nth start-time *hp-rpyacc-list*) (nth stop-time *hp-pos-list*) (nth stop-time *hp-vel-list*) (nth stop-time *hp-acc-list*) (nth stop-time *hp-rpy-list*) (nth stop-time *hp-rpyvel-list*) (nth stop-time *hp-rpyacc-list*))
  ;; (test-show-trajectory (* (- stop-time start-time) dt) (nth start-time *hp-pos-list*) (nth start-time *hp-vel-list*) (nth start-time *hp-acc-list*) (nth start-time *hp-angvel-list*) (nth start-time *hp-angacc-list*) (nth start-time *hp-angjerk-list*) (nth stop-time *hp-pos-list*) (nth stop-time *hp-vel-list*) (nth stop-time *hp-acc-list*) (nth stop-time *hp-angvel-list*) (nth stop-time *hp-angacc-list*) (nth stop-time *hp-angjerk-list*))
  )
|#

(defun test-show-trajectory
  (total-time
   pos p-vel p-acc rot r-vel r-acc goal-pos goal-p-vel goal-p-acc goal-rot goal-r-vel goal-r-acc
   &key (robot *robot*) (dt 0.002) (time-offset 0.2))
  ;; (send *racket-coords* :dissoc *sweet-spot*)
  ;; (send *sweet-spot* :assoc *racket-model*)
  (let ((tau 0.0) (ret) (ret-pos) (ret-rot))
    ;; (while (eps= t-remain 0.0 1e-5)
    (while (< tau 1)
      (setq ret-pos (path-generator pos p-vel p-acc goal-pos goal-p-vel goal-p-acc tau total-time))
      (if (listp rot)
          (progn
            (setq ret-rot (coerce (path-generator (coerce rot float-vector) (coerce r-vel float-vector) (coerce r-acc float-vector) (coerce goal-rot float-vector) (coerce goal-r-vel float-vector) (coerce goal-r-acc float-vector) tau total-time) cons))
            (setq ret (make-coords :pos ret-pos :rpy ret-rot))
            )
        (progn
          (setq ret-rot (path-generator rot r-vel r-acc goal-rot goal-r-vel goal-r-acc tau total-time))
          (setq ret (make-coords :pos ret-pos :rot (matrix-exponent ret-rot)))
          )
        )
      ;; (setq t-remain (- t-remain dt))
      ;; (send *sweet-spot* :move-to ret :world)
      (print ret-rot)
      (setq tau (+ tau 0.002))
      (objects ret)
      (with-open-file
       (f "/tmp/hoge.dat" :direction :output)
       (format f "~a ~a ~a~%" (aref ret-pos 0) (aref ret-pos 1) (aref ret-pos 2))
       )
      )
    )
  ;; (send *sweet-spot* :dissoc *racket-model*)
  ;; (send *racket-coords* :assoc *sweet-spot*)
  )



(defun calc-vel-acc (dt coords-list)
  (setq *hp-pos-list* '())
  (setq *hp-vel-list* '())
  (setq *hp-acc-list* '())
  (setq *hp-angvel-list* '())
  (setq *hp-angacc-list* '())
  (setq *hp-angjerk-list* '())
  (setq *hp-rpy-list* '())
  (setq *hp-rpyvel-list* '())
  (setq *hp-rpyacc-list* '())

  (let* ((tmp-pos)
         (tmp-angvel)
         (tmp-rpy)
         (tmp-vel #f(0 0 0))
         (tmp-angacc #f(0 0 0))
         (tmp-rpyvel (list 0 0 0))
         )
    (mapc #'(lambda (i)
              (setq tmp-pos (send (nth i coords-list) :worldpos))
              (setq tmp-angvel (matrix-log (send (nth i coords-list) :worldrot)))
              (setq tmp-rpy (nth 0 (rpy-angle (send (nth i coords-list) :worldrot))))
              (setq *hp-pos-list* (append *hp-pos-list* (list tmp-pos)))
              ;; (setq *hp-rpy-list* (append *hp-rpy-list* (list (rpy-angle (send (nth i coords-list) :worldrot))) ))

              (let ((k (if (< i 1) 0 (- i 1))))
                (if (eq k 0)
                    (progn
                      (setq *hp-angvel-list* (append *hp-angvel-list* (list tmp-angvel)))
                      (setq *hp-rpy-list* (append *hp-rpy-list* (list tmp-rpy)))
                      )
                  (progn
                    (setq *hp-angvel-list* (append *hp-angvel-list* (list
                                                                     (if (> (norm (v- tmp-angvel (nth k *hp-angvel-list*))) 3.0)
                                                                         (scale -1 tmp-angvel)
                                                                       tmp-angvel)
                                                                     )))
                    (setq tmp-rpy (let ((cnt 0))
                                    (dolist (j (mapcar #'- tmp-rpy (nth k *hp-rpy-list*)) tmp-rpy)
                                      (cond
                                       ((> j 6.0) (setf (nth cnt tmp-rpy) (- (nth cnt tmp-rpy) (* 2 pi))))
                                       ((< j -6.0) (setf (nth cnt tmp-rpy) (+ (nth cnt tmp-rpy) (* 2 pi))))
                                       (t)
                                       )
                                      (setq cnt (1+ cnt))
                                      )))
                    (setq *hp-rpy-list* (append *hp-rpy-list* (list tmp-rpy)))
                  )
                  )
                (setq tmp-vel (scale (/ 1.0 dt) (v- tmp-pos (nth k *hp-pos-list*))))
                (setq *hp-vel-list* (append *hp-vel-list* (list tmp-vel)))
                (setq *hp-acc-list* (append *hp-acc-list* (list (scale (/ 1.0 dt) (v- tmp-vel (nth k *hp-vel-list*)))) ))

                (setq tmp-angacc (scale (/ 1.0 dt) (v- tmp-angvel (nth k *hp-angvel-list*))))
                (setq *hp-angacc-list* (append *hp-angacc-list* (list tmp-angacc)))
                (setq *hp-angjerk-list* (append *hp-angjerk-list* (list (scale (/ 1.0 dt) (v- tmp-angacc (nth k *hp-angacc-list*)))) ))

                (setq tmp-rpyvel (mapcar #'/ (mapcar #'- tmp-rpy (nth k *hp-rpy-list*)) (make-list 3 :initial-element dt)))
                (setq *hp-rpyvel-list* (append *hp-rpyvel-list* (list tmp-rpyvel)))
                (setq *hp-rpyacc-list* (append *hp-rpyacc-list* (list (mapcar #'/ (mapcar #'- tmp-rpyvel (nth k *hp-rpyvel-list*)) (make-list 3 :initial-element dt))) ))
                ;; (if (and (> i 2750) (< i 3600)) (progn (format t "pos=~a  vel=~a~%" tmp-pos tmp-vel)))
                ))
          (my-range (length coords-list))
          )
    )
  )

;; delay trajectory generator
;;   "実機の物理的制約を考慮した即応的脚動作生成手法", 金広ら, Vol. 28, No. 10, pp.1251-1261, 2010, 日本ロボット学会誌, 4章

;; (defclass delay-trajectory-generator
;;   :super propertied-object
;;   :slots (dt total-time time-offset remain-time ;; [s]
;;              pos vel acc ;; [X], [X/s], [X/s^2]
;;              )
;;   )
;; (defmethod delay-trajectory-generator
;;   (:init
;;    (&key ((:dt tdt) 0.1) ((:time-offset to) 0.1))
;;    (setq dt tdt time-offset to))
;;   (:reset
;;    (start &key ((:total-time tt) 1.0))
;;    (setq total-time tt remain-time total-time
;;          pos start vel (instantiate float-vector (length start)) acc (instantiate float-vector (length start))))
;;   (:calculate-goal-pos
;;    (point-list)
;;    (midpoint-from-point-list
;;     (/ (- total-time remain-time) (- total-time time-offset))
;;     point-list))
;;   (:generate-trajectory-point
;;    (point-list &key ((:total-time tt) 1.0) (thre 1e-5))
;;    (if (or (eps= remain-time total-time thre)
;;            (eps= remain-time 0.0 thre))
;;        (send self :reset (car point-list) :total-time tt))
;;    (prog1
;;        (if (> remain-time time-offset)
;;            (send self :hoffarbib-interpolation time-offset (send self :calculate-goal-pos point-list))
;;          (send self :hoffarbib-interpolation remain-time (car (last point-list))))
;;      (setq remain-time (- remain-time dt))))
;;   (:hoffarbib-interpolation
;;    (tmp-remain-time tmp-goal)
;;    (let ((jerk (v+ (v+ (scale (/ -9.0 tmp-remain-time) acc)
;;                        (scale (/ -36.0 (expt tmp-remain-time 2)) vel))
;;                    (scale (/ 60.0 (expt tmp-remain-time 3)) (v- tmp-goal pos)))))
;;      (setq acc (v+ acc (scale dt jerk))
;;            vel (v+ vel (scale dt acc))
;;            pos (v+ pos (scale dt vel)))
;;      ))
;;   )
