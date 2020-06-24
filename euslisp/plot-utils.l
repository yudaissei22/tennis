(require "package://baseball/../../../../../irteus_proposals/motion-lib-proposal.l")
(require "package://baseball/../../../../../jsk/gnuplotlib.l")
(require "package://baseball/euslisp/my-range.l")
(require "package://baseball/euslisp/robot-interface.l")

(defun show-jvel (&key (robot *robot*) (limb :rarm) (tm-list *bspline-tm-list*) (av-list *bspline-av-list*))
  (let ((rs-list '()) (rs-list2 '()))
    (mapc #'(lambda (tm av)
              (setq rs-list (append rs-list (list (list :time tm :angle-vector av)))))
          tm-list av-list)
    (setq rs-list2 (convert-to-robot-state-data rs-list))
    (plot-robot-state-data-list rs-list2 :mode :jvel :limb limb :robot robot)
    )
  )

;; {{{ unused now (for debug)
(defun show-jvel-ratio (&key (robot *robot*) (limb :rarm) (tm-list *bspline-tm-list*) (av-list *bspline-av-list*))
  (let ((rs-list '()) (rs-list2 '()))
    (mapc #'(lambda (tm av)
              (setq rs-list (append rs-list (list (list :time tm :angle-vector av)))))
          tm-list av-list)
    (setq rs-list2 (convert-to-robot-state-data rs-list))
    (plot-robot-state-data-list rs-list2 :mode :jvel-ratio :limb limb :robot robot)
    )
  )


(defun show-vel-from-coords-list
  (&key (direction (list t #f(0 0 1))) (tm-list *bspline-tm-list*) (coords-list *bspline-hp-list*) (fname "/tmp/show-vel-from-coords-list.txt")
        (plot t) (title "Linear Velocity") (xlabel "Time [s]") (ylabel "Linear Velocity [m/s]") (keylist (list "linear velocity")))
  "If :direction is (list t float-vector), the linear velocity in a normal dirction of float-vector.
   If :direction is float-vector (dim=3), the velocity in direction axis is output.
   If :direction is nil, the speed (norm of the velocity) is output."
  (let* ((cnt 0)
         x_k x_k+1 tmp-vel v_k t_k t_k+1
         (vel-list (list 0.0)))
    (while (< cnt (- (length coords-list) 1))
      (setq x_k (send (nth cnt coords-list) :worldpos))
      (setq x_k+1 (send (nth (+ cnt 1) coords-list) :worldpos))
      (setq tmp-vel (scale 1e-3 (v- x_k+1 x_k)))
      (setq t_k (nth cnt tm-list))
      (setq t_k+1 (nth (+ cnt 1) tm-list))
      (cond
       ((vectorp direction) (setq v_k (v. tmp-vel (normalize-vector direction))))
       ((listp direction) (setq v_k (v. tmp-vel (v- (send (nth cnt coords-list) :transform-vector (nth 1 direction)) (send (nth cnt coords-list) :worldpos)))))
       (t (setq v_k (norm tmp-vel)))
       )
      (setq v_k (/ v_k (- t_k+1 t_k)))
      (push v_k vel-list)
      (incf cnt)
      )
    (setq vel-list (reverse vel-list))
    (with-open-file
     (f fname :direction :output)
     (dotimes (i (length tm-list))
       ;; (format t "~a ~a~%" (nth i tm-list) (nth i vel-list))
       (format f "~a ~a~%" (nth i tm-list) (nth i vel-list))
       )
     )
    (if plot
        (progn
          (graph-view (list vel-list) tm-list :title title :xlabel xlabel :ylabel ylabel :keylist keylist)
          nil)
      vel-list
      )
    )
  )

(defun show-vels-from-coords-lists
  (&key (direction nil) (tm-list *bspline-tm-list*)
        (coords-lists (list *bspline-ec-list* *bspline-cr-list* *bspline-sh-list* *bspline-el-list* *bspline-wr-list*))
        (title "Kinetic Chain") (xlabel "Time [s]") (ylabel "Linear Velocity [m/s]") (keylist (list "l-crotch" "r-shoulder" "r-elbow" "r-wrist" "r-endcoord")))
  (let* ((vel-lists (make-list (length coords-lists))))
    (dotimes (i (length vel-lists))
      (setf (nth i vel-lists) (funcall #'show-vel-from-coords-list :direction direction :tm-list tm-list :coords-list (nth i coords-lists) :plot nil))
      )
    (graph-view vel-lists tm-list :title title :xlabel xlabel :ylabel ylabel :keylist keylist)
    nil
    )
  )


(defun dump-coords-worldpos-xyz
  (&key (xyz 2) (fname (concatenate string "/tmp/coords-world-" (cond ((= xyz 0) "x.txt") ((= xyz 1) "y.txt") ((= xyz 2) "z.txt")))) (tm-list *bspline-tm-list*) (coords-list *bspline-ss-list*))
  "The function dumps the x-, y-, or z-coordinates of the coords-list."
  (with-open-file
   (f fname :direction :output)
   (dotimes (i (length tm-list))
     ;; (format t "~a ~a~%" (nth i tm-list) (aref (send (nth i coords-list) :worldpos) xyz))
     (format f "~a ~a~%" (nth i tm-list) (aref (send (nth i coords-list) :worldpos) xyz))
     )
   )
  )



(setq *gravity* #f(0 0 9.798))

(defun calc-link-mechanical-energy (link spacial-velocity angular-velocity)
  "This calculates the mechanical-energy of total links."
  (let* ((mass (* 1e-3 (send link :weight))) ;; m
         (inertia-tensor
          (m* (m* (send (send link :copy-worldcoords) :worldrot) (scale-matrix 1e-9 (send link :inertia-tensor)))
              (transpose (send (send link :copy-worldcoords) :worldrot)))) ;; RIR^T
         (link-pos (scale 1e-3 (send (send link :copy-worldcoords) :worldpos))) ;; h
         mechanical-energy
         )
    (setq mechanical-energy
          (+
           (* 0.5 (v. spacial-velocity spacial-velocity)) ;; 1/2 v^T v
           (* 0.5 (/ (v. angular-velocity (transform inertia-tensor angular-velocity)) mass)) ;; 1/2 w^T I w / m
           (v. *gravity* link-pos) ;; gh
           )
          )
    mechanical-energy
    )
  )

(defun calc-link-mechanical-power (link spacial-velocity spacial-acceleration angular-velocity angular-acceleration)
  "This function calculates the mechanical-power of total links."
  (let* ((mass (* 1e-3 (send link :weight))) ;; m
         (inertia-tensor
          (m* (m* (send (send link :copy-worldcoords) :worldrot) (scale-matrix 1e-9 (send link :inertia-tensor)))
              (transpose (send (send link :copy-worldcoords) :worldrot)))) ;; RIR^T
         mechanical-power
         )
    (setq mechanical-power
          (+
           (v. spacial-velocity spacial-acceleration) ;; v^T dv
           (/ (v. angular-velocity (transform inertia-tensor angular-acceleration)) mass) ;;  w^T I dw / m
           (v. *gravity* spacial-velocity) ;; gv
           )
          )
    mechanical-power
    )
  )


(defun show-link-mechanical-EorP
  (&key
   (p-ret *p*) (robot *robot*)
   (link-list (flatten (list (car (send robot :links)) (send robot :torso :links) (send robot :rarm :links))))
   (cnt 30) (x-step (/ (- *x-max* *x-min*) (* 1.0 cnt)))
   (calc-energy t)
   (title "Kinetic Chain") (xlabel "Time[s]")
   (ylabel (if calc-energy "Mechanical Energy[J/kg]" "Mechanical Power[W/kg]")) (keylist (send-all link-list :name))
   )
  "This function is for plot of the mechanical energy or power.
   If :calc-energy is nil, it calculates mechanical power."
  (setq *kinetic-chain-EorP-list* '())
  (setq *kinetic-chain-tm-list* '())
  (let* ((x-buf  *x-min*) (id 0)
         (loop-num 0)
         (tmp-EorP-list (instantiate float-vector (length link-list)))
         tmp-rc ;; root-coords_t
         tmp-rc-1 ;; root-coords_t-1
         tmp-rc-2 ;; root-coords_t-2
         )
    (while (<= x-buf *x-max*)
      (setq id 0)
      (mapc
       #'(lambda (jl bs)
           (list
            (send jl :joint-angle
                  (send bs :calc x-buf (subseq p-ret id (+ id *id-max*))))
            (setq id (+ id *id-max*))
            ))
       *jlist* *bspline*)
      (if (send (car (send robot :links)) :joint)
          (progn
            (send (send (car (send robot :links)) :joint) :joint-angle
                  (coerce
                   (mapcar
                    #'(lambda (bs)
                        (setq each-6dof (send bs :calc x-buf (subseq p-ret id (+ id *id-max*))))
                        (setq id (+ id *id-max*))
                        each-6dof)
                    (subseq *bspline* (length *jlist*) (+ (length *jlist*) 6)))
                   float-vector)
                  )
            )
        (progn
          (send robot :fix-leg-to-coords *lfoot* :lleg)
          )
        )
      (if (= loop-num 0)
          (progn
            (setq tmp-rc-2 (mapcar #'(lambda (l) (send l :copy-worldcoords)) link-list))
            (setq tmp-rc-1 (mapcar #'(lambda (l) (send l :copy-worldcoords)) link-list))
            (setq tmp-rc (mapcar #'(lambda (l) (send l :copy-worldcoords)) link-list))
            )
        (progn
          (setq tmp-rc-2 tmp-rc-1)
          (setq tmp-rc-1 tmp-rc)
          (setq tmp-rc (mapcar #'(lambda (l) (send l :copy-worldcoords)) link-list))
          )
        )
      (setq tmp-EorP-list
            (coerce (mapcar
                     #'(lambda (l rc-2 rc-1 rc)
                         (let* ((dt x-step)
                                (vel-1 (scale (/ 1.0 dt) (scale 1e-3 (v- (send rc-1 :worldpos) (send rc-2 :worldpos))))) ;; v_t-1
                                (vel (scale (/ 1.0 dt) (scale 1e-3 (v- (send rc :worldpos) (send rc-1 :worldpos))))) ;; v_t
                                (acc (scale (/ 1.0 dt) (v- vel vel-1))) ;; a_t
                                (angvel-1 (scale (/ 1.0 dt) (send rc-2 :difference-rotation rc-1))) ;; w_t-1
                                (angvel (scale (/ 1.0 dt) (send rc-1 :difference-rotation rc))) ;; w_t
                                (angacc (scale (/ 1.0 dt) (v- angvel angvel-1))) ;; dw_t
                                )
                           (if calc-energy
                               (calc-link-mechanical-energy l vel angvel)
                             (calc-link-mechanical-power l vel acc angvel angacc)
                             )
                           )
                         )
                     link-list tmp-rc-2 tmp-rc-1 tmp-rc)
                    float-vector))
      (push tmp-EorP-list *kinetic-chain-EorP-list*)
      (push x-buf *kinetic-chain-tm-list*)
      (send *irtviewer* :draw-objects)
      (x::window-main-one)
      (incf loop-num)
      (setq x-buf (+ x-buf x-step))
      ) ;; end while loop

    (setq *kinetic-chain-EorP-list* (reverse *kinetic-chain-EorP-list*))
    (setq *kinetic-chain-tm-list* (reverse *kinetic-chain-tm-list*))
    (setq *kinetic-chain-EorP-matrix* (apply #'matrix *kinetic-chain-EorP-list*))
    (graph-view
     (mapcar #'(lambda (col-id) (matrix-column *kinetic-chain-EorP-matrix* col-id)) (my-range (nth 1 (array-dimensions *kinetic-chain-EorP-matrix*))))
     *kinetic-chain-tm-list* :title title :xlabel xlabel :ylabel ylabel :keylist keylist
     )
    )
  nil
  )


(defun dump-list
  (&key
   (robot *robot*)
   (type "av")
   (fpath "/tmp")
   (fname-auto (concatenate string
                            "/"
                            (subseq (substitute #\- #\: (substitute #\- #\ (unix:asctime (unix:localtime)))) 4 (1- (length (unix:asctime (unix:localtime)))) )
                            "-"))
   (fname "motion")
   (extension (cond
           ((equal type "av") ".avlist")
           ((equal type "tau") ".taulist")
           ((equal type "xyz") ".xyzlist")
           (t "raw.rawlist")
           )
          )
   (tm-list *bspline-tm-list*) (limb nil)
   (final-fname (concatenate string fpath fname-auto fname))
   (xyz-offset #f(0 0 0))
   (plot-list
    (cond
     ((equal type "av") *bspline-av-list*)
     ((equal type "tau") *bspline-tau-list*)
     ((equal type "xyz") ;; (progn (show-vel-from-coords-list :plot nil :fname (concatenate string final-fname ".vellist"))
                         ;;        (format t "dump ~a~%" (concatenate string final-fname ".vellist"))
                         ;;        *bspline-hp-list*))
      *bspline-hp-list*)
     )
    )
   )
  (when (equal type "xyz")
    (show-vel-from-coords-list :plot nil :fname (concatenate string final-fname ".vellist"))
    (format t "dump ~a~%" (concatenate string final-fname ".vellist"))
    )
  "The function dumps the plot-list."

  ;; for xyz list
  (when (consp (car plot-list))
    (print xyz-offset)
    (let* ((newlist nil) newfv (xyz-flag nil))
      (dotimes (i (length (car plot-list)))
        (setq newfv nil)
        (dolist (pl plot-list)
          (if (vectorp (nth i pl))
              (setq newfv (concatenate float-vector newfv (nth i pl)))
            (progn
              (setq newfv (concatenate float-vector newfv (v- (send (nth i pl) :worldpos) xyz-offset))) ;; xyz
              (setq xyz-flag t)
              )
            )
          )
        (when xyz-flag
          (let* (
                 (x-act (aref newfv 0))
                 (y-act (aref newfv 1))
                 (z-act (aref newfv 2))
                 (x-ref (aref newfv 3))
                 (y-ref (aref newfv 4))
                 (z-ref (aref newfv 5))
                 ;; (x-act (- (aref newfv 0) (aref xyz-offset 0)))
                 ;; (y-act (- (aref newfv 1) (aref xyz-offset 1)))
                 ;; (z-act (- (aref newfv 2) (aref xyz-offset 2)))
                 ;; (x-ref (- (aref newfv 3) (aref xyz-offset 0)))
                 ;; (y-ref (- (aref newfv 4) (aref xyz-offset 1)))
                 ;; (z-ref (- (aref newfv 5) (aref xyz-offset 2)))
                 )
            (setq newfv (concatenate float-vector
                                     newfv
                                     (list
                                      (sqrt (+ (* (- x-ref x-act) (- x-ref x-act)) (* (- z-ref z-act) (- z-ref z-act))))
                                      (sqrt (+ (* (- x-ref x-act) (- x-ref x-act)) (* (- y-ref y-act) (- y-ref y-act)) (* (- z-ref z-act) (- z-ref z-act))))
                                      )
                                     ))
            ;;(when (= i 1) (print newfv))
            )
          )
        (push newfv newlist)
        )
      (setq newlist (reverse newlist))
      (setq plot-list newlist)
      )
    )

  (with-open-file
   (f (concatenate string final-fname extension) :direction :output)
   (dotimes (i (length tm-list))
     ;;(print (nth i plot-list))
     (format f "~a " (nth i tm-list))
     (let* ((y
             (cond
              ((equal type "av")
               (if limb
                   (progn
                     (send *robot* :angle-vector (nth i plot-list))
                     (send *robot* limb :angle-vector)
                     )
                 (nth i plot-list)))
              ((equal type "xyz")
               ;;(print (nth i plot-list))
               (if (vectorp (nth i plot-list))
                   (scale 1e-3 (nth i plot-list))
                 (scale 1e-3 (send (nth i plot-list) :worldpos))
                 )
               )
              (t (nth i plot-list))
              )
             ))
       (dotimes (j (length y))
         (format f "~a " (aref y j))
         )
       )
     (format f "~%")
     )
   )
  (format t "dump ~a~%" (concatenate string final-fname extension))
  nil
  )


(defun dump-lists
  (&key
   (robot *robot*)
   (type-list (list "av" "xyz" "tau"))
   (fpath "/tmp")
   (fname-auto (concatenate string
                            "/"
                            (subseq (substitute #\- #\: (substitute #\- #\ (unix:asctime (unix:localtime)))) 4 (1- (length (unix:asctime (unix:localtime)))) )
                            "-"))
   (fname "motion")
   (tm-list *bspline-tm-list*) (limb nil)
   )
  (dolist (tl type-list)
    (dump-list :fpath fpath :fname-auto fname-auto :type tl)
    )
  nil
  )


(defun replay-real-motion
  (avlist
   &key
   (make-list nil)
   (robot *robot*)
   (real-time nil)
   (warning nil)
   (show-pose t)
   (fix-leg-coords (list :lleg *lfoot*))
   (dump t)
   (fpath "/tmp")
   (fname-auto (concatenate string
                            "/"
                            (subseq (substitute #\- #\: (substitute #\- #\ (unix:asctime (unix:localtime)))) 4 (1- (length (unix:asctime (unix:localtime)))) )
                            "-"))
   (fname "motion")
   ;; (planned-avlist *bspline-av-list*)
   (start-time 0.0)
   (xyz-offset #f(0 0 0))
   )

  (setq *experiment-av-list* avlist)
  (setq *experiment-tm-list* (mapcar #'(lambda (x) (* x 0.002) ) (my-range (length avlist))))
  (when make-list
    (setq *experiment-hp-list* '())
    (setq *experiment-tau-list* '())
    (let* (p dp ddp p-1 p-2 dp-1
             (cnt 0)
             )
      (dolist (av avlist)
        (cond
         ((= (length *experiment-hp-list*) 0)
          (setq p (coerce av cons))
          (setq dp (make-list (length av) :initial-element 0.0))
          (setq ddp (make-list (length av) :initial-element 0.0))
          (setq p-1 p)
          )
         ((= (length *experiment-hp-list*) 1)
          (setq p (coerce av cons))
          (setq dp
                (mapcar #'(lambda (av av-1)
                            (/ (- av av-1) 0.002)
                            )
                        p p-1)
                )
          (setq ddp (make-list (length av) :initial-element 0.0))
          (setq p-2 p-1)
          (setq p-1 p)
          (setq dp-1 dp)
          )
         (t
          (setq p (coerce av cons))
          (setq dp
                (mapcar #'(lambda (av av-1)
                            (/ (- av av-1) 0.002)
                            )
                        p p-1)
                )
          (setq ddp
                (mapcar #'(lambda (jv jv-1)
                            (/ (- jv jv-1) 0.002)
                            )
                        dp dp-1)
                )
          (setq p-2 p-1)
          (setq p-1 p)
          (setq dp-1 dp)
          ))

        (send robot :angle-vector (coerce p float-vector))
        (send robot :fix-leg-to-coords (nth 1 fix-leg-coords) (nth 0 fix-leg-coords))
        (push (send-all *jlist* :get :ddp) ddp)
        (push (send-all *jlist* :get :dp) dp)
        (push (send-all *jlist* :get :p) p)
        (send robot :calc-torque-from-vel-acc
              :jvv (map float-vector
                        #'(lambda (j) (deg2rad (or (send j :get :dp) 0)))
                        (cdr (send robot :links)))
              :jav (map float-vector
                        #'(lambda (j) (deg2rad (or (send j :get :ddp) 0)))
                        (cdr (send robot :links))))
        (push (coerce (send-all *jlist* :joint-torque) float-vector) *experiment-tau-list*)
        (push (send *hit-point* :copy-worldcoords) *experiment-hp-list*)
        (send *irtviewer* :draw-objects)
        (x::window-main-one)
        (unix:usleep 1)
        (incf cnt)
        )

      (setq *experiment-tau-list* (reverse *experiment-tau-list*))
      (setq *experiment-hp-list* (reverse *experiment-hp-list*))
      )
    )

  (when dump
    (dump-list :fpath fpath :fname-auto fname-auto :type "av" :plot-list *experiment-av-list* :tm-list *experiment-tm-list*)
    (let* ((hplist nil))
      (setq hplist
            (append
             (make-list (round (/ start-time 0.002)) :initial-element (car *bspline-hp-list*))
             *bspline-hp-list*
             (make-list (- (length *experiment-hp-list*) (round (/ start-time 0.002)) (length *bspline-hp-list*)) :initial-element (car (last *bspline-hp-list*)))
             )
            )
      (dump-list :fpath fpath :fname-auto fname-auto :type "xyz" :plot-list (list *experiment-hp-list* hplist) :tm-list *experiment-tm-list* :xyz-offset xyz-offset)
      )
    (dump-list :fpath fpath :fname-auto fname-auto :type "tau" :plot-list *experiment-tau-list* :tm-list *experiment-tm-list*)
    )

  nil
  )


(defun replay-real-motion2
  (avlist avlist2
   &key
   (robot *robot*)
   (real-time nil)
   (warning nil)
   (show-pose t)
   (fix-leg-coords (list :lleg *lfoot*))
   (dump t)
   (fpath "/tmp")
   (fname-auto (concatenate string
                            "/"
                            (subseq (substitute #\- #\: (substitute #\- #\ (unix:asctime (unix:localtime)))) 4 (1- (length (unix:asctime (unix:localtime)))) )
                            "-"))
   (fname "motion")
   ;; (planned-avlist *bspline-av-list*)
   (start-time 0.0)
   (xyz-offset #f(0 0 0))
   )

  (let* ((experiment-av-list avlist)
         (experiment-av-list2 avlist2)
         (experiment-hp-list '())
         (experiment-hp-list2 '())
         (experiment-tau-list '())
         (experiment-tau-list2 '())
         (experiment-tm-list (mapcar #'(lambda (x) (* x 0.002) ) (my-range (length avlist))))
         )
    ;; avlist
    (let* (p dp ddp p-1 p-2 dp-1
             (cnt 0)
             )
      (dolist (av avlist)
        (cond
         ((= (length experiment-hp-list) 0)
          (setq p (coerce av cons))
          (setq dp (make-list (length av) :initial-element 0.0))
          (setq ddp (make-list (length av) :initial-element 0.0))
          (setq p-1 p)
          )
         ((= (length experiment-hp-list) 1)
          (setq p (coerce av cons))
          (setq dp
                (mapcar #'(lambda (av av-1)
                            (/ (- av av-1) 0.002)
                            )
                        p p-1)
                )
          (setq ddp (make-list (length av) :initial-element 0.0))
          (setq p-2 p-1)
          (setq p-1 p)
          (setq dp-1 dp)
          )
         (t
          (setq p (coerce av cons))
          (setq dp
                (mapcar #'(lambda (av av-1)
                            (/ (- av av-1) 0.002)
                            )
                        p p-1)
                )
          (setq ddp
                (mapcar #'(lambda (jv jv-1)
                            (/ (- jv jv-1) 0.002)
                            )
                        dp dp-1)
                )
          (setq p-2 p-1)
          (setq p-1 p)
          (setq dp-1 dp)
          ))

        (send robot :angle-vector (coerce p float-vector))
        (send robot :fix-leg-to-coords (nth 1 fix-leg-coords) (nth 0 fix-leg-coords))
        (push (send-all *jlist* :get :ddp) ddp)
        (push (send-all *jlist* :get :dp) dp)
        (push (send-all *jlist* :get :p) p)
        (send robot :calc-torque-from-vel-acc
              :jvv (map float-vector
                        #'(lambda (j) (deg2rad (or (send j :get :dp) 0)))
                        (cdr (send robot :links)))
              :jav (map float-vector
                        #'(lambda (j) (deg2rad (or (send j :get :ddp) 0)))
                        (cdr (send robot :links))))
        (push (coerce (send-all *jlist* :joint-torque) float-vector) experiment-tau-list)
        (push (send *hit-point* :copy-worldcoords) experiment-hp-list)
        (send *irtviewer* :draw-objects)
        (x::window-main-one)
        (unix:usleep 1)
        (incf cnt)
        )

      (setq experiment-tau-list (reverse experiment-tau-list))
      (setq experiment-hp-list (reverse experiment-hp-list))
      )

    ;; avlist2
    (let* (p dp ddp p-1 p-2 dp-1
             (cnt 0)
             )
      (dolist (av avlist2)
        (cond
         ((= (length experiment-hp-list2) 0)
          (setq p (coerce av cons))
          (setq dp (make-list (length av) :initial-element 0.0))
          (setq ddp (make-list (length av) :initial-element 0.0))
          (setq p-1 p)
          )
         ((= (length experiment-hp-list2) 1)
          (setq p (coerce av cons))
          (setq dp
                (mapcar #'(lambda (av av-1)
                            (/ (- av av-1) 0.002)
                            )
                        p p-1)
                )
          (setq ddp (make-list (length av) :initial-element 0.0))
          (setq p-2 p-1)
          (setq p-1 p)
          (setq dp-1 dp)
          )
         (t
          (setq p (coerce av cons))
          (setq dp
                (mapcar #'(lambda (av av-1)
                            (/ (- av av-1) 0.002)
                            )
                        p p-1)
                )
          (setq ddp
                (mapcar #'(lambda (jv jv-1)
                            (/ (- jv jv-1) 0.002)
                            )
                        dp dp-1)
                )
          (setq p-2 p-1)
          (setq p-1 p)
          (setq dp-1 dp)
          ))

        (send robot :angle-vector (coerce p float-vector))
        (send robot :fix-leg-to-coords (nth 1 fix-leg-coords) (nth 0 fix-leg-coords))
        (push (send-all *jlist* :get :ddp) ddp)
        (push (send-all *jlist* :get :dp) dp)
        (push (send-all *jlist* :get :p) p)
        (send robot :calc-torque-from-vel-acc
              :jvv (map float-vector
                        #'(lambda (j) (deg2rad (or (send j :get :dp) 0)))
                        (cdr (send robot :links)))
              :jav (map float-vector
                        #'(lambda (j) (deg2rad (or (send j :get :ddp) 0)))
                        (cdr (send robot :links))))
        (push (coerce (send-all *jlist* :joint-torque) float-vector) experiment-tau-list2)
        (push (send *hit-point* :copy-worldcoords) experiment-hp-list2)
        (send *irtviewer* :draw-objects)
        (x::window-main-one)
        (unix:usleep 1)
        (incf cnt)
        )

      (setq experiment-tau-list2 (reverse experiment-tau-list2))
      (setq experiment-hp-list2 (reverse experiment-hp-list2))
      )

    (when dump
      (dump-list :fpath fpath :fname-auto fname-auto :type "av" :plot-list experiment-av-list :tm-list experiment-tm-list)
      (let* ((hplist nil))
        (setq hplist
              (append
               (nthcdr (round (/ start-time 0.002)) experiment-hp-list2)
               (make-list (round (/ start-time 0.002)) :initial-element (car (last experiment-hp-list2)))
               )
              )
        (dump-list :fpath fpath :fname-auto fname-auto :type "xyz" :plot-list (list hplist experiment-hp-list) :tm-list experiment-tm-list :xyz-offset xyz-offset)
        )
      (dump-list :fpath fpath :fname-auto fname-auto :type "tau" :plot-list experiment-tau-list :tm-list experiment-tm-list)
      )
    )
  nil
  )
;; }}}



#|
(progn
  (send *robot* :move-to (make-coords) :world)
  (with-append-root-joint
   (link-list-with-robot-6dof *robot* (list (cdr (send *robot* :links)))
                              :joint-class 6dof-joint)
   ;;(show-link-mechanical-EorP :cnt 100 :calc-energy t)
   ))

;;(show-optimized-motion *p* :x-step 0.002 :x-max *x-max-of-p-orig* :real-time t :fix-leg-coords (list :rleg *rfoot*))
(show-optimized-motion *p* :x-step 0.002 :x-max *x-max-of-p-orig* :real-time t :fix-leg-coords (list :lleg *lfoot*))


;; python jaxon_avlist_from_logref.py ~(log) avlist.l
;; load "avlist.l"

;;(replay-real-motion *real-avlist* :fpath "/home/future731/tmp/" :make-list nil :start-time 0.0 :xyz-offset (progn (show-optimized-motion *p* :specific-time (eval *x-hit*) :fix-leg-coords (list :rleg *rfoot*)) (send *hit-point* :worldpos)))
(replay-real-motion *real-avlist* :fpath "/home/future731/tmp/" :make-list nil :start-time 0.0 :xyz-offset (progn (show-optimized-motion *p* :specific-time (eval *x-hit*) :fix-leg-coords (list :lleg *lfoot*)) (send *hit-point* :worldpos)))

(replay-real-motion2 *real-avlist-sh_q* *real-avlist-st_q* :fpath "/home/future731/tmp/"  :start-time 0.0 :xyz-offset (progn (send *robot* :angle-vector (nth (round (/ (eval *x-hit*) 0.002)) *real-avlist-sh_q*) ) (send *robot* :fix-leg-to-coords *lfoot* :lleg ) (send *hit-point* :worldpos)) :fix-leg-coords (list :lleg *lfoot*))
|#
