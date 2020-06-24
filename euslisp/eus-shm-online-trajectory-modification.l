(load "experiment_utils.l")
(comp::compile-file-if-src-newer "experiment_utils.l" nil)
(load "experiment_utils.so")

(defvar SA_DISABLE 0)
(defvar SA_ENABLE 1)
(defvar EUS_TRIGGER 2)
(defvar EUS_ACCESSIBLE 3)
(defvar EUS_DISABLE 4)
(defvar initialize-shm nil)

;; eus-shm-interface
(let ((lib (load-foreign "online-trajectory-modification/eus_shm_interface.so")))
  (defforeign _initialize-sharedmemory lib "initialize_sharedmemory" () :integer)
  (defforeign _write-ref-angle-shm lib "write_ref_angle_shm" (:integer :integer :string) :integer)
  (defforeign _write-otm-flag-shm lib "write_otm_flag_shm" (:integer) :integer)
  (defforeign _read-otm-flag-shm lib "read_otm_flag_shm" () :integer)
  )

(defun initialize-sharedmemory ()
  (when (not initialize-shm)
    (_initialize-sharedmemory)
    )
  )

(defun write-ref-angle-shm (fv seq)
  (let ((bytestr (dvector2float-bytestring fv)))
    (_write-ref-angle-shm (length fv) seq bytestr));;
  fv)

(defun write-otm-flag-shm (flag)
  (_write-otm-flag-shm flag);;
  flag)

(defun read-otm-flag-shm ()
  (let ((otmflag (_read-otm-flag-shm)));;
    (format t "read-otm-flag-shm: otmflag = ~a~%" otmflag)
    otmflag
    )
  )

;;;
(warn "
(initialize-sharedmemory)
(write-ref-angle-shm (fill (instantiate float-vector 33) 1) 25)
")


(defun experiment-eus-shm-online-trajectory-modification
  (jpos tm pos rpy zmp optional t-max x-hit qp-time
   &key (initial-time 10000) (final-time 5000)
        (log-fname "/tmp/online-trajectory-modification")
        (test-target #f(0 0 -0.0026 0 0 0))
        (real t)
        (debug? t)
        (append nil)
        (lfoot-ik nil)
        (rfoot-ik nil)
        )
  (initialize-sharedmemory)
  (if (eps= (apply #'+ (cdr tm)) t-max)
      (progn
        (setq *p* (copy-seq *p-orig*))
        (setq *qp-ss-list* nil)
        (send *robot* :angle-vector (map float-vector #'rad2deg (car jpos)))
        (if real
            (progn
              (send *ri* :start-log)
              ;;;;(send *ri* :angle-vector (send *robot* :angle-vector) initial-time) ;; initial-timeまでに最初の姿勢にする
              (send *ri* :angle-vector (send *robot* :angle-vector) initial-time) ;; initial-timeまでに最初の姿勢にする
              (send *ri* :wait-interpolation)
              )
          (progn
            (send *robot* :angle-vector)
            (send *robot* :fix-leg-to-coords *lfoot* :lleg)
            (send *irtviewer* :draw-objects)
            )
          )
        (let* ((timer (instance mtimer :init))
               (timer2 (instance mtimer :init))
               (step (car tm))
               (load-flag t)
               (send-flag t)
               (bspline
                (progn
                  (if (send (car (send *robot* :links)) :joint) ;; with-append-root-jointしてあればt
                      (progn
                        (mapcar #'(lambda (k)
                                    (instance basic-spline :init
                                              :id-max *id-max* :recursive-order *recursive-order*
                                              :x-min *x-min* :x-max t-max))
                                (flatten *jlist* (make-list 6)))) ;; root-jointの6個分付け加える
                    (progn
                      (mapcar #'(lambda (k)
                                  (instance basic-spline :init
                                            :id-max *id-max* :recursive-order *recursive-order*
                                            :x-min *x-min* :x-max t-max))
                              *jlist*))
                    )
                  ))

               (exp-jpos-orig jpos)
               (exp-tm-orig tm)
               (exp-pos-orig pos)
               (exp-rpy-orig rpy)
               (exp-zmp-orig zmp)
               (exp-optional-orig optional)
               (exp-vel-orig (make-list (length exp-jpos-orig) :initial-element (instantiate float-vector (length (car exp-jpos-orig)))))
               (exp-torque-orig (make-list (length exp-jpos-orig) :initial-element (instantiate float-vector (length (car exp-jpos-orig)))))
               (exp-acc-orig (make-list (length exp-jpos-orig) :initial-element (instantiate float-vector 3)))
               (exp-wrench-orig (make-list (length exp-jpos-orig) :initial-element (instantiate float-vector (length (car exp-jpos-orig)))))
               ;; ;; ref
               (choreonoid (if (= (length (car exp-jpos-orig)) (length (send *robot* :angle-vector))) nil t))
               (exp-jpos-ref nil)
               (exp-jpos-ref-old (concatenate float-vector (map float-vector #'rad2deg (nth 1 exp-jpos-orig))))

               exp-jpos exp-pos exp-rpy exp-zmp exp-optional exp-tm exp-vel exp-torque exp-acc exp-wrench
               (jvss (instance std_msgs::float64multiarray :init))
               (vels (instance std_msgs::float64multiarray :init))
               (torques (instance std_msgs::float64multiarray :init))
               (poss (instance std_msgs::float64multiarray :init))
               (rpys (instance std_msgs::float64multiarray :init))
               (accs (instance std_msgs::float64multiarray :init))
               (zmps (instance std_msgs::float64multiarray :init))
               (wrenchs (instance std_msgs::float64multiarray :init))
               (optionals (instance std_msgs::float64multiarray :init))
               array-jvs array-tm array-pos array-rpy array-zmp array-optional
               array-vel array-torque array-acc array-wrench

               (tms (coerce exp-tm-orig float-vector))

               j
               )
          ;; jvss
          (setq array-jvs (apply #'matrix exp-jpos-orig))
          (send jvss :layout :dim (list
                                   (instance std_msgs::multiArraydimension :init :label "sequence length" :size (nth 0 (array-dimensions array-jvs)) :stride (* (nth 0 (array-dimensions array-jvs)) (nth 1 (array-dimensions array-jvs))))
                                   (instance std_msgs::multiArraydimension :init :label "jvs length" :size (nth 1 (array-dimensions array-jvs)) :stride (nth 1 (array-dimensions array-jvs)))))
          (send jvss :data (array-entity array-jvs))
          ;; vels
          (setq array-vel (apply #'matrix exp-vel-orig))
          (send vels :layout :dim (list
                                   (instance std_msgs::multiArraydimension :init :label "sequence length" :size (nth 0 (array-dimensions array-vel)) :stride (* (nth 0 (array-dimensions array-vel)) (nth 1 (array-dimensions array-vel))))
                                   (instance std_msgs::multiArraydimension :init :label "vel length" :size (nth 1 (array-dimensions array-vel)) :stride (nth 1 (array-dimensions array-vel)))))
          (send vels :data (array-entity array-vel))
          ;; torques
          (setq array-torque (apply #'matrix exp-torque-orig))
          (send torques :layout :dim (list
                                      (instance std_msgs::multiArraydimension :init :label "sequence length" :size (nth 0 (array-dimensions array-torque)) :stride (* (nth 0 (array-dimensions array-torque)) (nth 1 (array-dimensions array-torque))))
                                      (instance std_msgs::multiArraydimension :init :label "torque length" :size (nth 1 (array-dimensions array-torque)) :stride (nth 1 (array-dimensions array-torque)))))
          (send torques :data (array-entity array-torque))
          ;; poss
          (setq array-pos (apply #'matrix exp-pos-orig))
          (send poss :layout :dim (list
                                   (instance std_msgs::multiArraydimension :init :label "sequence length" :size (nth 0 (array-dimensions array-pos)) :stride (* (nth 0 (array-dimensions array-pos)) (nth 1 (array-dimensions array-pos))))
                                   (instance std_msgs::multiArraydimension :init :label "pos length" :size (nth 1 (array-dimensions array-pos)) :stride (nth 1 (array-dimensions array-pos)))))
          (send poss :data (array-entity array-pos))
          ;; rpys
          (setq array-rpy (apply #'matrix exp-rpy-orig))
          (send rpys :layout :dim (list
                                   (instance std_msgs::multiArraydimension :init :label "sequence length" :size (nth 0 (array-dimensions array-rpy)) :stride (* (nth 0 (array-dimensions array-rpy)) (nth 1 (array-dimensions array-rpy))))
                                   (instance std_msgs::multiArraydimension :init :label "rpy length" :size (nth 1 (array-dimensions array-rpy)) :stride (nth 1 (array-dimensions array-rpy)))))
          (send rpys :data (array-entity array-rpy))
          ;; accs
          (setq array-acc (apply #'matrix exp-acc-orig))
          (send accs :layout :dim (list
                                   (instance std_msgs::multiArraydimension :init :label "sequence length" :size (nth 0 (array-dimensions array-acc)) :stride (* (nth 0 (array-dimensions array-acc)) (nth 1 (array-dimensions array-acc))))
                                   (instance std_msgs::multiArraydimension :init :label "acc length" :size (nth 1 (array-dimensions array-acc)) :stride (nth 1 (array-dimensions array-acc)))))
          (send accs :data (array-entity array-acc))
          ;; zmps
          (setq array-zmp (apply #'matrix exp-zmp-orig))
          (send zmps :layout :dim (list
                                   (instance std_msgs::multiArraydimension :init :label "sequence length" :size (nth 0 (array-dimensions array-zmp)) :stride (* (nth 0 (array-dimensions array-zmp)) (nth 1 (array-dimensions array-zmp))))
                                   (instance std_msgs::multiArraydimension :init :label "zmp length" :size (nth 1 (array-dimensions array-zmp)) :stride (nth 1 (array-dimensions array-zmp)))))
          (send zmps :data (array-entity array-zmp))
          ;; wrenchs
          (setq array-wrench (apply #'matrix exp-wrench-orig))
          (send wrenchs :layout :dim (list
                                      (instance std_msgs::multiArraydimension :init :label "sequence length" :size (nth 0 (array-dimensions array-wrench)) :stride (* (nth 0 (array-dimensions array-wrench)) (nth 1 (array-dimensions array-wrench))))
                                      (instance std_msgs::multiArraydimension :init :label "wrench length" :size (nth 1 (array-dimensions array-wrench)) :stride (nth 1 (array-dimensions array-wrench)))))
          (send wrenchs :data (array-entity array-wrench))
          ;; optionals
          (setq array-optional (apply #'matrix exp-optional-orig))
          (send optionals :layout :dim (list
                                        (instance std_msgs::multiArraydimension :init :label "sequence length" :size (nth 0 (array-dimensions array-optional)) :stride (* (nth 0 (array-dimensions array-optional)) (nth 1 (array-dimensions array-optional))))
                                        (instance std_msgs::multiArraydimension :init :label "optional length" :size (nth 1 (array-dimensions array-optional)) :stride (nth 1 (array-dimensions array-optional)))))
          (send optionals :data (array-entity array-optional))
          ;; tms
          (setq tms (coerce exp-tm-orig float-vector))

          ;; (send timer :start)
          ;; setJointAnglesSequenceFull at first time
          (if real
              (progn
                (send *ri* :sequenceplayerservice_setjointanglessequencefull :jvss jvss :vels vels :torques torques :poss poss :rpys rpys :accs accs :zmps zmps :wrenchs wrenchs :optionals optionals :tms tms)
                (send timer :start)
                ;; shm-flag
                (write-otm-flag-shm SA_ENABLE)
                )
            (progn
              (format t "setFointAnglesSequenceFull at first!!~%")
              (send timer :start)
              )
            )

          (setq *dp-list* nil)
          (dolist (i (my-range (- (round (/ t-max step)) 1)))
            (if real
                (progn
                  (while (not (= (read-otm-flag-shm) EUS_TRIGGER))
                    (unix:usleep 10)
                    )
                  (write-otm-flag-shm EUS_ACCESSIBLE)
                  )
              (unix:usleep (round (* step 1000 1000)))
              )


            (setq j (+ i 1)) ;; j = i + 1
            ;; (format t "timer-start = ~a~%" (send timer :stop))
            (let*
                (dp
                 (target test-target) ;;後でiに応じて変更する
                 )
              (send timer2 :start)
              (setq dp
                    (if (< (send timer :stop) qp-time
                           )
                        (progn
                          (calc-dp-from-dr target (* step j) x-hit :bspline bspline :debug? debug?)
                          )
                      (instantiate float-vector (length *p*))
                      )
                    )
              (send timer2 :stop)
              (format t "timer2 = ~a~%" (send timer2 :stop))
              (push dp *dp-list*)
              ;;(print dp)
              (setq *p* (v+ *p* dp))
              )

            (send *robot* :angle-vector (map float-vector #'rad2deg (nth j jpos))) ;; if you change step time (= delta t), you should also change here.
            (map float-vector
                 #'(lambda (jl bs)
                     (setq id (* *id-max*  (position jl (send *robot* :joint-list) :test #'equal)))
                     (send jl :joint-angle (send bs :calc (* step (+ j 1)) (subseq *p* id (+ id *id-max*))))
                     )
                 (send *robot* *limb* :joint-list) bspline)
            (setq exp-jpos-ref (concatenate float-vector (send *robot* :angle-vector) (if choreonoid #f(-80 80 -80 80) nil))) ;; [deg]

            ;; (print (length exp-jpos-ref))
            (when (not real)
              (print exp-jpos-ref-old)
              (print exp-jpos-ref)
              ;;(send *robot* :move-to (nth j *exp-wc1*))
              (send *irtviewer* :draw-objects)
              )

            ;; (format t "timer = ~a,  step*(i+1) = ~a~%" (send timer :stop) (* step j))
            (if real
                (progn
                  ;; ;; write joint angles to shm
                  (print exp-jpos-ref-old)
                  (print exp-jpos-ref)
                  (send *irtviewer* :draw-objects)
                  ;;(format t "exp-jpos-ref = ~a~%" exp-jpos-ref)
                  (let ((step-num (round (/ step 0.002)))
                        tmp-interpolation-list)
                    (setq tmp-interpolation-list
                          (mapcar #'(lambda (x)
                                      (map float-vector #'deg2rad
                                           (v+ (scale (/ (+ x 1.0) step-num) (v- exp-jpos-ref exp-jpos-ref-old)) exp-jpos-ref-old)
                                           )
                                      )
                                  (my-range step-num)))
                    (setq *interpolation* (apply #'concatenate float-vector tmp-interpolation-list))
                    ;; (setq *interpolation* (fill (instantiate float-vector (* 25 37)) 0))
                    )
                  (write-ref-angle-shm *interpolation* 25)
                  )
              (progn
                ;;(format t "send ~a angle-vector timer = ~a~%" (* step j) (send timer :stop)) ;;;;
                )
              )
            (setq exp-jpos-ref-old exp-jpos-ref)
            ) ;; dolist

          (send timer2 :start)
          (when (consp append)
            (dolist (als append) (append-last-sequence (nth 0 als) :tm (nth 1 als) :append t :choreonoid choreonoid))
            (setq exp-jpos (nthcdr (- (length *exp-jpos-rad1*) (length append)) *exp-jpos-rad1*))
            ;;;; (setq exp-jpos (nthcdr (- (length *exp-jpos-deg1*) (length append)) *exp-jpos-deg1*)) ;;;;
            (setq exp-pos (nthcdr (- (length *exp-pos1*) (length append)) *exp-pos1*))
            (setq exp-rpy (nthcdr (- (length *exp-rpy1*) (length append)) *exp-rpy1*))
            (setq exp-zmp (nthcdr (- (length *exp-zmp-rc1*) (length append)) *exp-zmp-rc1*))
            (setq exp-optional (nthcdr (- (length *exp-optional1*) (length append)) *exp-optional1*))
            (setq exp-tm (nthcdr (- (length *exp-tm-s1*) (length append)) *exp-tm-s1*))
            (setq exp-tm (nthcdr (- (length *exp-tm-ms1*) (length append)) *exp-tm-ms1*)) ;;;;
            (setq exp-vel (make-list (length append) :initial-element (instantiate float-vector (length (car exp-jpos-orig)))))
            (setq exp-torque (make-list (length append) :initial-element (instantiate float-vector (length (car exp-jpos-orig)))))
            (setq exp-acc (make-list (length append) :initial-element (instantiate float-vector 3)))
            (setq exp-wrench (make-list (length append) :initial-element (instantiate float-vector (length (car exp-jpos-orig)))))
            (format t "exp-jpos length = ~a, exp-tm length = ~a~%" (length exp-jpos) (length exp-tm))

            ;; jvss
            (setq array-jvs (apply #'matrix exp-jpos))
            (send jvss :layout :dim (list
                                     (instance std_msgs::multiArraydimension :init :label "sequence length" :size (nth 0 (array-dimensions array-jvs)) :stride (* (nth 0 (array-dimensions array-jvs)) (nth 1 (array-dimensions array-jvs))))
                                     (instance std_msgs::multiArraydimension :init :label "jvs length" :size (nth 1 (array-dimensions array-jvs)) :stride (nth 1 (array-dimensions array-jvs)))))
            (send jvss :data (array-entity array-jvs))
            ;; vels
            (setq array-vel (apply #'matrix exp-vel))
            (send vels :layout :dim (list
                                     (instance std_msgs::multiArraydimension :init :label "sequence length" :size (nth 0 (array-dimensions array-vel)) :stride (* (nth 0 (array-dimensions array-vel)) (nth 1 (array-dimensions array-vel))))
                                     (instance std_msgs::multiArraydimension :init :label "vel length" :size (nth 1 (array-dimensions array-vel)) :stride (nth 1 (array-dimensions array-vel)))))
            (send vels :data (array-entity array-vel))
            ;; torques
            (setq array-torque (apply #'matrix exp-torque))
            (send torques :layout :dim (list
                                        (instance std_msgs::multiArraydimension :init :label "sequence length" :size (nth 0 (array-dimensions array-torque)) :stride (* (nth 0 (array-dimensions array-torque)) (nth 1 (array-dimensions array-torque))))
                                        (instance std_msgs::multiArraydimension :init :label "torque length" :size (nth 1 (array-dimensions array-torque)) :stride (nth 1 (array-dimensions array-torque)))))
            (send torques :data (array-entity array-torque))
            ;; poss
            (setq array-pos (apply #'matrix exp-pos))
            (send poss :layout :dim (list
                                     (instance std_msgs::multiArraydimension :init :label "sequence length" :size (nth 0 (array-dimensions array-pos)) :stride (* (nth 0 (array-dimensions array-pos)) (nth 1 (array-dimensions array-pos))))
                                     (instance std_msgs::multiArraydimension :init :label "pos length" :size (nth 1 (array-dimensions array-pos)) :stride (nth 1 (array-dimensions array-pos)))))
            (send poss :data (array-entity array-pos))
            ;; rpys
            (setq array-rpy (apply #'matrix exp-rpy))
            (send rpys :layout :dim (list
                                     (instance std_msgs::multiArraydimension :init :label "sequence length" :size (nth 0 (array-dimensions array-rpy)) :stride (* (nth 0 (array-dimensions array-rpy)) (nth 1 (array-dimensions array-rpy))))
                                     (instance std_msgs::multiArraydimension :init :label "rpy length" :size (nth 1 (array-dimensions array-rpy)) :stride (nth 1 (array-dimensions array-rpy)))))
            (send rpys :data (array-entity array-rpy))
            ;; accs
            (setq array-acc (apply #'matrix exp-acc))
            (send accs :layout :dim (list
                                     (instance std_msgs::multiArraydimension :init :label "sequence length" :size (nth 0 (array-dimensions array-acc)) :stride (* (nth 0 (array-dimensions array-acc)) (nth 1 (array-dimensions array-acc))))
                                     (instance std_msgs::multiArraydimension :init :label "acc length" :size (nth 1 (array-dimensions array-acc)) :stride (nth 1 (array-dimensions array-acc)))))
            (send accs :data (array-entity array-acc))
            ;; zmps
            (setq array-zmp (apply #'matrix exp-zmp))
            (send zmps :layout :dim (list
                                     (instance std_msgs::multiArraydimension :init :label "sequence length" :size (nth 0 (array-dimensions array-zmp)) :stride (* (nth 0 (array-dimensions array-zmp)) (nth 1 (array-dimensions array-zmp))))
                                     (instance std_msgs::multiArraydimension :init :label "zmp length" :size (nth 1 (array-dimensions array-zmp)) :stride (nth 1 (array-dimensions array-zmp)))))
            (send zmps :data (array-entity array-zmp))
            ;; wrenchs
            (setq array-wrench (apply #'matrix exp-wrench))
            (send wrenchs :layout :dim (list
                                        (instance std_msgs::multiArraydimension :init :label "sequence length" :size (nth 0 (array-dimensions array-wrench)) :stride (* (nth 0 (array-dimensions array-wrench)) (nth 1 (array-dimensions array-wrench))))
                                        (instance std_msgs::multiArraydimension :init :label "wrench length" :size (nth 1 (array-dimensions array-wrench)) :stride (nth 1 (array-dimensions array-wrench)))))
            (send wrenchs :data (array-entity array-wrench))
            ;; optionals
            (setq array-optional (apply #'matrix exp-optional))
            (send optionals :layout :dim (list
                                          (instance std_msgs::multiArraydimension :init :label "sequence length" :size (nth 0 (array-dimensions array-optional)) :stride (* (nth 0 (array-dimensions array-optional)) (nth 1 (array-dimensions array-optional))))
                                          (instance std_msgs::multiArraydimension :init :label "optional length" :size (nth 1 (array-dimensions array-optional)) :stride (nth 1 (array-dimensions array-optional)))))
            (send optionals :data (array-entity array-optional))
            ;; tms
            (setq tms (coerce exp-tm float-vector))

            (setq *exp-jpos-deg1* (subseq *exp-jpos-deg1* 0 (- (length *exp-jpos-deg1*) (length append))))
            (setq *exp-tm-ms1* (subseq *exp-tm-ms1* 0 (- (length *exp-tm-ms1*) (length append))))
            (setq *exp-rc1* (subseq *exp-rc1* 0 (- (length *exp-rc1*) (length append))))
            (setq *exp-zmp-wc1* (subseq *exp-zmp-wc1* 0 (- (length *exp-zmp-wc1*) (length append))))
            (setq *exp-optional1* (subseq *exp-optional1* 0 (- (length *exp-optional1*) (length append))))
            (setq *exp-jpos-rad1* (subseq *exp-jpos-rad1* 0 (- (length *exp-jpos-rad1*) (length append))))
            (setq *exp-tm-s1* (subseq *exp-tm-s1* 0 (- (length *exp-tm-s1*) (length append))))
            (setq *exp-pos1* (subseq *exp-pos1* 0 (- (length *exp-pos1*) (length append))))
            (setq *exp-rpy1* (subseq *exp-rpy1* 0 (- (length *exp-rpy1*) (length append))))
            (setq *exp-zmp-rc1* (subseq *exp-zmp-rc1* 0 (- (length *exp-zmp-rc1*) (length append))))
            (setq *exp-wc1* (subseq *exp-wc1* 0 (- (length *exp-wc1*) (length append))))
            )
          (while (<= (send timer2 :stop) step)
            (unix:usleep 10)
            )
          (write-otm-flag-shm EUS_DISABLE)
          (when (consp append)
            (if real
                (progn
                  (send *ri* :sequenceplayerservice_setjointanglessequencefull :jvss jvss :vels vels :torques torques :poss poss :rpys rpys :accs accs :zmps zmps :wrenchs wrenchs :optionals optionals :tms tms)
                  )
              (progn
                (format t "setJointAnglesSequenceFullFinal timer = ~a~%" t-max (send timer :stop))
                )
              )
            )
          );; end let

        (when real
          (send *ri* :wait-interpolation-seq)
          (unix:usleep (* final-time 1000))
          (send *ri* :save-log log-fname :make-directory t)

          ;; ;; CAUTION !!
          (send *ri* :angle-vector (send *robot* :angle-vector (send *ri* :state :potentio-vector)) 1000)
          (unix:usleep 1000000)
          (write-otm-flag-shm SA_DISABLE)
          )
        (setq *dp-list* (reverse *dp-list*))
        (setq *qp-ss-list* (reverse *qp-ss-list*))
        )
    (format t "Time-list and t-max are not consistent. You should make sure the sequence and time scale!~%")
    )

  nil
  )

#|
(let* ((time-scale 1.0))
  (setq *p* (copy-seq *p-orig*))
  (send *robot* :move-to (make-coords) :world)
  (with-append-root-joint
   (link-list-with-robot-6dof *robot* (list (cdr (send *robot* :links)))
                              :joint-class 6dof-joint)
   (experiment-eus-shm-online-trajectory-modification *exp-jpos-rad1* *exp-tm-s1* *exp-pos1* *exp-rpy1* *exp-zmp-rc1* *exp-optional1* (* *x-max-of-p-orig* time-scale) (* (eval *x-hit*) time-scale) (* (- (eval *x-hit*) (* 3.0 (car *exp-tm-s1*))) time-scale) :initial-time 2000 :final-time 5000 :test-target #f(0 0 -0.02 0 0 0) :real nil :debug? t :append *append-list* :lfoot-ik t :rfoot-ik t)
   (setq *p* (copy-seq *p-orig*))
   nil))

(let* ((t-max *x-max-of-p-orig*) (step 0.05) (seq-num (round (+ 1 (/ t-max step)))))
  (setq *exp-jpos-deg1* (make-list seq-num))
  (setq *exp-tm-ms1* (make-list seq-num))
  (setq *exp-rc1* (make-list seq-num))
  (setq *exp-zmp-wc1* (make-list seq-num))
  (setq *exp-optional1* (make-list seq-num))
  (setq *exp-jpos-rad1* (make-list seq-num))
  (setq *exp-tm-s1* (make-list seq-num))
  (setq *exp-pos1* (make-list seq-num))
  (setq *exp-rpy1* (make-list seq-num))
  (setq *exp-zmp-rc1* (make-list seq-num))
  (setq *exp-wc1* (make-list seq-num))
  (setq *exp-cog-wc1* (make-list seq-num))
  (make-sequence-in-advance
   *exp-jpos-deg1* *exp-tm-ms1* *exp-rc1* *exp-zmp-wc1* *exp-optional1*
   *exp-jpos-rad1* *exp-tm-s1* *exp-pos1* *exp-rpy1* *exp-zmp-rc1* *exp-wc1* *exp-cog-wc1*
   :step step :x-max t-max :choreonoid t :p *p* :lfoot-ik t :rfoot-ik t)
  )
|#
