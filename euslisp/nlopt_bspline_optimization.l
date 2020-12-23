;; $ roseus
;; $ (compile-file "nlopt_bspline_optimization.l")
;; $ (setq *motion-choice* "forehand") ;; "punch", "kick", "smash"
;; $ (load "nlopt_bspline_optimization.so")
(require "package://eus_nlopt/euslisp/nlopt")
(when (and (not (boundp '*p-orig*)) (not (boundp '*x-max-of-p-orig*)))
    (require "package://tennis/config/p-orig"))


(comp::compile-file-if-src-newer  (ros::resolve-ros-path "package://tennis/euslisp/qp-bspline-optimization.l") (ros::resolve-ros-path "package://tennis/euslisp/"))
(require "package://tennis/euslisp/qp-bspline-optimization")

;; capsules are set for collision check
(cond
 ((or (equal (send *robot* :name) "JAXON") (equal (send *robot* :name) "JAXON_RED"))
  (require "package://tennis/euslisp/jaxon-collision-model.l"))
 ;;((equal (send *robot* :name) "***") (require "package://tennis/euslisp/***-collision-model.l"))
 (t (require "package://tennis/euslisp/jaxon-collision-model.l")))

(defun change-robot-state-specific-time (p time use-append-root-joint)
    (let ((id 0))
        (mapc
            #'(lambda (jl bs)
                  (send jl :joint-angle
                        (send bs :calc time (subseq p id (+ id *id-max*))))
                  (send jl :put :dq (deg2rad (send bs :calc-delta time (subseq p id (+ id *id-max*)) :n 1))) ;; 関節速度の単位は[rad]
                  (setq id (+ id *id-max*)))
         *jlist* *bspline*)
        (when use-append-root-joint
            (send (send (car (send *robot* :links)) :joint) :joint-angle
                (coerce
                    (mapcar
                    #'(lambda (bs)
                        (send bs :calc time (subseq p id (setq id (+ id *id-max*)))))
                     (subseq *bspline* (length *jlist*) (+ (length *jlist*) 6)))
                 float-vector))))
    )

(defun validate-p-orig-and-bspline-dim ()
  (when (or (> (- (length *p-orig*) (* *id-max* (length *bspline*))) 3)
            (< (- (length *p-orig*) (* *id-max* (length *bspline*))) 0))
    (format t "size does not match: ~A~%" (length *p-orig*) (* *id-max* (length *bspline*)))
    (exit)
    )
  )

(defun nlopt-init
    (&key
     (robot *robot*)
     (motion-choice *motion-choice*)
     (init (qp-init))

     (use-all-joint t)
     (use-append-root-joint t)
     (jlist (if use-all-joint
              (send robot :joint-list) ;; length jlist = 33
             (send robot *limb* :joint-list)))

     (id-max nil)
     (recursive-order nil)
     (x-min 0.0)
     (x-max *x-max-of-p-orig*)
     (x-hit nil)
     (use-final-pose nil)
     (use-hit-pose t)
     (use-hit-ee nil) ;; bug is included
     (use-6dof-p nil)
     (default-switching-list nil)
     (support-polygon-margin (list 50 50 0 100 50))
     (epsilon-c 30)
     (mu 0.3)
     (interval-num (round (/ x-max 0.05))))

  ;; jskeus/eus/models/arrow-object.l
  (require "models/arrow-object.l")
  (when (not (boundp '*arrow*))
    (setq *arrow* (arrow))
    (objects *arrow*))

  (when (not (boundp '*limb-end-coords-default*))
    (send robot :reset-pose)
    (send robot :move-to (make-coords) :world)
    (setq *limb-end-coords-default* (send (send robot *limb* :end-coords) :copy-worldcoords)))

  (when (not (boundp '*floor*))
    (setq *floor* (make-cube 1500 1500 0.01))
    (send *floor* :put :name "floor")
    (if (boundp '*jaxon*)
        (send *floor* :locate #f(0 0 -0.06) :world) ;; for jaxon
      (send *floor* :locate #f(0 0 3.86) :world))) ;; for jaxon_red

;; 面倒なので*floor*は*collision-pair*の後ろ側(body1 or body3)にする


  (setq *current-force* #f(0 0 0))
  (setq *current-zmp-moment* #f(0 0 0))
  (setq *current-zmp* #f(0 0 0))
  (return-end-coords)
  (funcall *load-pose-func*)
  (send *irtviewer* :draw-objects)
  (setq *qp-flag* nil)

  (setq *p* (copy-seq *p-orig*))

  (setq *id-max* id-max)
  (setq *recursive-order* recursive-order)
  (setq *x-min* x-min)
  (setq *x-max* x-max)
  (setq *x-hit-id* 1)
  ;; (/ *x-max* *x-max-of-p-orig*)の意味: 初期化の変数として，x-maxを変えると途中の*p*をスケーリングしている
  (setq *x-hit* '(* (aref *p* (- (length *p*) *x-hit-id*)) (/ *x-max* *x-max-of-p-orig*))) ;; *x-hit*を遅延評価
  (when x-hit
    (setq *default-x-hit* x-hit))

  (setq *jlist* jlist) ;; length jlist = 33
  (setq *jvel-list* (flatten (send robot :torso :joint-list) (send robot *limb* :joint-list)))
  (setq *bspline* (if use-append-root-joint
                      (progn
                        (mapcar #'(lambda (k)
                                    (instance basic-spline :init
                                              :id-max id-max :recursive-order recursive-order
                                              :x-min x-min :x-max x-max))
                                (flatten jlist (make-list 6))))
                    (progn
                      (mapcar #'(lambda (k)
                                  (instance basic-spline :init
                                            :id-max id-max :recursive-order recursive-order
                                            :x-min x-min :x-max x-max))
                              jlist))))


  (setq *mu* mu)
  (setq *interval-num* interval-num)
  (setq *sp-front* (nth 0 support-polygon-margin))
  (setq *sp-back* (nth 1 support-polygon-margin))
  (setq *sp-inside* (nth 2 support-polygon-margin))
  (setq *sp-outside* (nth 3 support-polygon-margin))
  (setq *sp-oneleg* (nth 4 support-polygon-margin))
  (setq *epsilon-c* epsilon-c)
  (setq *use-all-joint* use-all-joint)
  (setq *use-append-root-joint* use-append-root-joint)
  (setq *concatenate-root-joint* (instantiate float-vector 18))
  (setq *default-switching-list* nil)
  (setq *use-final-pose* use-final-pose)
  (setq *use-hit-pose* use-hit-pose)
  (setq *use-hit-ee* use-hit-ee)
  (setq *use-6dof-p* use-6dof-p)

  (format t "*************** motion ***************~%*id-max*=~a, *recursive-order*=~a, *x-min*=~a, *x-hit*=~a, *x-max*=~a, *sp-front*=~a, *sp-back*=~a, *sp-inside*=~a, *sp-outside*=~a, *epsilon-c*=~a, *mu*=~a~%~%" *id-max* *recursive-order* *x-min* (eval *x-hit*) *x-max* *sp-front* *sp-back* *sp-inside* *sp-outside* *epsilon-c* *mu*)


  (cond
   ((equal motion-choice "forehand")
    (setq *x-hit-id* 1)
    (setq *hit-point* *sweet-spot*)
    (setq *target-coords* *racket-coords*)
    (setq *collision-pair*
          (list (list (list *rarm-no-upper-collision* *chest/head-collision* '*epsilon-c*)   (list *rarm-no-shoulder-collision* *chest/head-collision* 0))
                (list (list *rarm-no-upper-collision* *body/torso-collision* '*epsilon-c*)   (list (append *rarm-collision* *racket-capsule-list*) *larm-collision* '*epsilon-c*))
                (list (list *larm-no-shoulder-collision* *chest/head-collision* 0)   (list *racket-capsule-list* *chest/head-collision* '*epsilon-c*))
                (list (list *larm-no-upper-collision* *body/torso-collision* '*epsilon-c*)   (list *larm-no-shoulder-collision* *body/torso-collision* 20))))


    ;; forehand
    ;; contact-flagは(x-start とx-hitの間) (x-hitとx-endの間)の2つ
    (setq *contact-flag* (list (list *lfoot* *rfoot* nil nil) (list *lfoot* *rfoot* nil nil)))
    ;; contact flag
    ;; (list (list lleg0 rleg0) (list lleg1 rleg1) ...)
    ;; *p*はこの時点で時刻などを含む最適化初期値が入っている
    (if default-switching-list
        (progn
          (setq *default-switching-list* default-switching-list)
          (setq *default-x-hit* (nth (- (length default-switching-list) *x-hit-id*) default-switching-list)))

      (progn
        (dotimes (i (1- (length *contact-flag*))) ;; x-hit1回分の時刻
          (push (aref *p* (- (length *p*) 1 i)) *default-switching-list*)) ;; (aref *p* (- (length *p*) 1 i))は*p*の最後からi(>=0)番目を取り出している
        ;; ここでは*default-switching-list*に最適化に使うタイミング変数が格納される

        (setq *default-x-hit* (eval *x-hit*))))


    (format t "************* forehand **************~%")
    (format t "(nlopt-init :x-max 2.4 :x-hit nil :id-max 14 :recursive-order 5 :use-all-joint t :use-append-root-joint t :support-polygon-margin (list 50 50 0 100 50) :epsilon-c 30 :mu 0.3 :use-final-pose nil :default-switching-list nil :use-6dof-p t)~%~%")
    (format t "(nlopt-motion-optimize :x-max ~a :x-hit ~a :id-max ~a :recursive-order ~a :max-eval 100000000 :alg SLSQP :delta (deg2rad 0.01) :eqthre 1e-8 :xtol 1e-10 :ftol 1e-15 :use-all-joint t :use-margin 0.5 :use-append-root-joint t :maxvel-weight 1 :minjerk-weight 5e-3 :modify-ec t :p *p* :interval-num ~a :title \"maximize-speed\" :max-time (* 2 24 60 60) :file-path \"/userdir/logs\")~%~%~%"
            *x-max* *default-x-hit* *id-max* *recursive-order* *interval-num*))


   ((equal motion-choice "punch")
    (setq *x-hit-id* 1)
    (setq *hit-point* *punch-point*)
    (setq *target-coords* *punch-coords*)
    (setq *collision-pair*
          (list (list (list *rarm-no-upper-collision* *chest/head-collision* '*epsilon-c*)   (list *rarm-no-shoulder-collision* *chest/head-collision* 0))
                (list (list *rarm-no-shoulder-collision* *body/torso-collision* '*epsilon-c*)   (list *larm-no-shoulder-collision* *body/torso-collision* '*epsilon-c*))))


    (setq *contact-flag* (list (list *lfoot* *rfoot* nil nil) (list *lfoot* *rfoot* nil nil)))
    (if default-switching-list
        (progn
          (setq *default-switching-list* default-switching-list)
          (setq *default-x-hit* (nth (- (length default-switching-list) *x-hit-id*) default-switching-list)))

      (progn
        (dotimes (i (1- (length *contact-flag*)))
          (push (aref *p* (- (length *p*) 1 i)) *default-switching-list*))

        (setq *default-x-hit* (eval *x-hit*))))


    (format t "*************** punch ***************~%")
    (format t "(nlopt-init :x-max 2.0 :x-hit nil :id-max 14 :recursive-order 5 :use-all-joint t :use-append-root-joint t :support-polygon-margin (list 50 50 0 100 50) :epsilon-c 30 :mu 0.3 :use-final-pose nil :default-switching-list nil :use-6dof-p t)~%~%")
    (format t "(nlopt-motion-optimize :x-max ~a :x-hit ~a :id-max ~a :recursive-order ~a :max-eval 100000000 :alg SLSQP :delta (deg2rad 0.01) :eqthre 1e-8 :xtol 1e-10 :ftol 1e-15 :use-all-joint t :use-margin 0.5 :use-append-root-joint t :maxvel-weight 1 :minjerk-weight 5e-3 :modify-ec nil :p *p* :interval-num ~a :title \"maximize-speed\" :max-time (* 2 24 60 60) :file-path \"/userdir/logs\")~%~%~%"
            *x-max* *default-x-hit* *id-max* *recursive-order* *interval-num*))


   ((equal motion-choice "kick")
    (setq *x-hit-id* 1)
    (setq *hit-point* *kick-point*)
    (setq *target-coords* *kick-coords*)
    (setq *collision-pair*
          (list (list (list *rleg-collision* *larm-collision* 0)   (list *rleg-no-groin-collision* *larm-collision* 0))
                (list (list *rleg-no-groin-collision* *body/torso-collision* -30)   (list *rleg-no-groin-collision* *body/torso-collision* 0)) ;; initial poseで既に干渉していることになるので-30にしておく
                (list (list *rarm-no-shoulder-collision* *chest/head-collision* 0)   (list *lleg-no-foot-collision* *larm-no-upper-collision* '*epsilon-c*))
                (list (list *larm-no-shoulder-collision* *chest/head-collision* 0)   (list *body/torso-collision* *larm-no-upper-collision* '*epsilon-c*))))


    (setq *contact-flag* (list (list *lfoot* nil nil nil) (list *lfoot* nil nil nil)))
    (if default-switching-list
        (progn
          (setq *default-switching-list* default-switching-list)
          (setq *default-x-hit* (nth (- (length default-switching-list) *x-hit-id*) default-switching-list)))

      (progn
        (dotimes (i (1- (length *contact-flag*)))
          (push (aref *p* (- (length *p*) 1 i)) *default-switching-list*))

        (setq *default-x-hit* (eval *x-hit*))))


    (format t "*************** kick ****************~%")
    (format t "(nlopt-init :x-max 2.0 :x-hit nil :id-max 14 :recursive-order 5 :use-all-joint t :use-append-root-joint t :support-polygon-margin (list 50 50 0 100 50) :epsilon-c 30 :mu 0.3 :use-final-pose nil :default-switching-list nil :use-6dof-p t)~%~%")
    (format t "(nlopt-motion-optimize :x-max ~a :x-hit ~a :id-max ~a :recursive-order ~a :max-eval 100000000 :alg SLSQP :delta (deg2rad 0.01) :eqthre 1e-8 :xtol 1e-10 :ftol 1e-15 :use-all-joint t :use-margin 0.5 :use-append-root-joint t :maxvel-weight 1 :minjerk-weight 5e-3 :modify-ec nil :p *p* :interval-num ~a :title \"maximize-speed\" :max-time (* 2 24 60 60) :file-path \"/userdir/logs\")~%~%~%"
            *x-max* *default-x-hit* *id-max* *recursive-order* *interval-num*))


   ((equal motion-choice "smash")
    (setq *x-hit-id* 1)
    (setq *hit-point* *sweet-spot*)
    (setq *target-coords* *racket-coords*)
    (setq *collision-pair*
          (list (list (list *rarm-no-shoulder-collision* *body/torso-collision* '*epsilon-c*)   (list *rarm-collision* *body/torso-collision* '*epsilon-c*))
                (list (list *rarm-no-shoulder-collision* *chest/head-collision* 0)   (list *rarm-no-shoulder-collision* *chest/head-collision* 0))
                (list (list *racket-capsule-list* *chest/head-collision* '*epsilon-c*)   (list (append *racket-capsule-list* *rarm-collision*) *lleg-no-foot-collision* '*epsilon-c*))
                (list (list *racket-capsule-list* *body/torso-collision* '*epsilon-c*)   (list *larm-no-upper-collision* *body/torso-collision* '*epsilon-c*))))


    (setq *contact-flag* (list (list *lfoot* *rfoot* nil nil) (list *lfoot* *rfoot* nil nil)))
    (if default-switching-list
        (progn
          (setq *default-switching-list* default-switching-list)
          (setq *default-x-hit* (nth (- (length default-switching-list) *x-hit-id*) default-switching-list)))

      (progn
        (dotimes (i (1- (length *contact-flag*)))
          (push (aref *p* (- (length *p*) 1 i)) *default-switching-list*))

        (setq *default-x-hit* (eval *x-hit*))))


    (format t "*************** smash ***************~%")
    (format t "(nlopt-init :x-max 2.4 :x-hit nil :id-max 14 :recursive-order 5 :use-all-joint t :use-append-root-joint t :support-polygon-margin (list 50 50 0 100 50) :epsilon-c 30 :mu 0.3 :use-final-pose nil :default-switching-list nil :use-6dof-p t)~%~%")
    (format t "(nlopt-motion-optimize :x-max ~a :x-hit ~a :id-max ~a :recursive-order ~a :max-eval 100000000 :alg SLSQP :delta (deg2rad 0.01) :eqthre 1e-8 :xtol 1e-10 :ftol 1e-15 :use-all-joint t :use-margin 0.5 :use-append-root-joint t :maxvel-weight 1 :minjerk-weight 5e-3 :modify-ec t :p *p* :interval-num ~a :title \"maximize-speed\" :max-time (* 2 24 60 60) :file-path \"/userdir/logs\")~%~%"
            *x-max* *default-x-hit* *id-max* *recursive-order* *interval-num*))


   ((equal motion-choice "batting")
    (setq *x-hit-id* 1)
    (setq *hit-point* *hitting-point*)
    (setq *target-coords* *bat-coords*)
    (setq *collision-pair*
          (list (list (list *larm-no-upper-collision* *rarm-no-upper-collision* -20)   (list *larm-no-upper-collision* *rarm-collision* '*epsilon-c*))
                (list (list *rarm-no-upper-collision* *chest/head-collision* '*epsilon-c*) (list *rarm-no-shoulder-collision* *chest/head-collision* -20))
                (list (list *rarm-no-upper-collision* *body/torso-collision* '*epsilon-c*) (list *rarm-collision* *body/torso-collision* '*epsilon-c*))
                (list (list *larm-no-upper-collision* *body/torso-collision* '*epsilon-c*) (list *larm-collision* *body/torso-collision* '*epsilon-c*))))


    (setq *contact-flag* (list (list *lfoot* *rfoot* nil *bat-rhand-grip*) (list *lfoot* *rfoot* nil nil)))
    (if default-switching-list
        (progn
          (setq *default-switching-list* default-switching-list)
          (setq *default-x-hit* (nth (- (length default-switching-list) *x-hit-id*) default-switching-list)))

      (progn
        (dotimes (i (1- (length *contact-flag*)))
          (push (aref *p* (- (length *p*) 1 i)) *default-switching-list*))

        (setq *default-x-hit* (eval *x-hit*))))


    (format t "************** batting **************~%")
    (format t "(nlopt-init :x-max 1.9 :x-hit nil :id-max 14 :recursive-order 5 :use-all-joint t :use-append-root-joint t :support-polygon-margin (list 50 50 0 100 50) :epsilon-c 30 :mu 0.3 :use-final-pose nil :default-switching-list nil :use-6dof-p t)~%~%")
    (format t "(nlopt-motion-optimize :x-max ~a :x-hit ~a :id-max ~a :recursive-order ~a :max-eval 100000000 :alg SLSQP :delta (deg2rad 0.01) :eqthre 1e-8 :xtol 1e-10 :ftol 1e-15 :use-all-joint t :use-margin 0.5 :use-append-root-joint t :maxvel-weight 1 :minjerk-weight 5e-3 :modify-ec t :p *p* :interval-num ~a :title \"maximize-speed\" :max-time (* 2 24 60 60) :file-path \"/userdir/logs\")~%~%"
            *x-max* *default-x-hit* *id-max* *recursive-order* *interval-num*))


   ((equal motion-choice "forehand-step")
    (setq *x-hit-id* 1)
    (setq *hit-point* *sweet-spot*)
    (setq *target-coords* *racket-coords*)
    (setq *collision-pair*
          (list (list (list *rarm-no-upper-collision* *chest/head-collision* '*epsilon-c*)   (list *rarm-no-shoulder-collision* *chest/head-collision* 0))
                (list (list *rarm-no-upper-collision* *body/torso-collision* '*epsilon-c*)   (list *rarm-collision* *larm-collision* '*epsilon-c*))
                (list (list *larm-no-upper-collision* (append *chest/head-collision* *rarm-collision*) '*epsilon-c*)   (list *racket-capsule-list* *chest/head-collision* '*epsilon-c*))
                (list (list *lleg-no-groin-collision* *rleg-collision* 20)   (list *racket-capsule-list* *larm-collision* '*epsilon-c*))
                (list (list (append *lleg-no-foot-collision* (list *lfoot-plate*)) (list *floor*) 0)   (list *larm-no-shoulder-collision* *body/torso-collision* 20))))


    ;; forehand-step
    ;; list (t0, tl, tg, th, te)
    ;; 時刻間の状態
    (setq *contact-flag* (list (list *lfoot-step* *rfoot* nil nil) (list nil *rfoot* nil nil) (list *lfoot* *rfoot* nil nil) (list *lfoot* *rfoot* nil nil)))
    (if default-switching-list
        (progn
          (setq *default-switching-list* default-switching-list)
          (setq *default-x-hit* (nth (- (length default-switching-list) *x-hit-id*) default-switching-list)))

      (progn
        (dotimes (i (1- (length *contact-flag*)))
          (push (aref *p* (- (length *p*) 1 i)) *default-switching-list*))

        (setq *default-x-hit* (eval *x-hit*))))


    (format t "*********** forehand-step ***********~%")
    (format t "(nlopt-init :x-max 3.0 :x-hit nil :id-max 16 :recursive-order 5 :use-all-joint t :use-append-root-joint t :support-polygon-margin (list 50 50 0 100 50) :epsilon-c 30 :mu 0.3 :use-final-pose nil :default-switching-list nil :use-6dof-p t)~%~%")
    (format t "(nlopt-motion-optimize :x-max ~a :x-hit ~a :id-max ~a :recursive-order ~a :max-eval 100000000 :alg SLSQP :delta (deg2rad 0.01) :eqthre 1e-8 :xtol 1e-10 :ftol 1e-15 :use-all-joint t :use-margin 0.5 :use-append-root-joint t :maxvel-weight 1 :minjerk-weight 5e-3 :modify-ec t :p *p* :interval-num ~a :title \"maximize-speed\" :max-time (* 2 24 60 60) :file-path \"/userdir/logs\")~%~%~%"
            *x-max* *default-x-hit* *id-max* *recursive-order* *interval-num*))

   ((equal motion-choice "forehand-volley-step")
    (setq *x-hit-id* 1)
    (setq *hit-point* *sweet-spot*)
    (setq *target-coords* *racket-coords*)
    (setq *collision-pair*
          (list (list (list *rarm-no-upper-collision* *chest/head-collision* '*epsilon-c*)   (list *rarm-no-shoulder-collision* *chest/head-collision* 0))
                (list (list *rarm-no-upper-collision* *body/torso-collision* '*epsilon-c*)   (list *rarm-collision* *larm-collision* '*epsilon-c*))
                (list (list *larm-no-upper-collision* (append *chest/head-collision* *rarm-collision*) '*epsilon-c*)   (list *racket-capsule-list* *chest/head-collision* '*epsilon-c*))
                (list (list *lleg-no-groin-collision* *rleg-collision* 20)   (list *racket-capsule-list* *larm-collision* '*epsilon-c*))
                (list (list (append *lleg-no-foot-collision* (list *lfoot-plate*)) (list *floor*) 0)   (list *larm-no-shoulder-collision* *body/torso-collision* 20))))


    ;; forehand-volley-step
    ;; list (t0, tl, tg, th, te)
    ;; 時刻間の状態
    ;; contact-flagは(t0 とtlの間)... の4つ
    (setq *contact-flag* (list (list *lfoot* *rfoot* nil nil) (list nil *rfoot* nil nil) (list *lfoot-step* *rfoot* nil nil) (list *lfoot-step* *rfoot* nil nil)))
    (if default-switching-list
        (progn
          (setq *default-switching-list* default-switching-list)
          (setq *default-x-hit* (nth (- (length default-switching-list) *x-hit-id*) default-switching-list)))

      (progn
        (dotimes (i (1- (length *contact-flag*)))
          (push (aref *p* (- (length *p*) 1 i)) *default-switching-list*))

        (setq *default-x-hit* (eval *x-hit*))))


    (format t "*********** forehand-volley-step ***********~%")
    (format t "(nlopt-init :x-max 1.4 :x-hit nil :id-max 10 :recursive-order 5 :use-all-joint t :use-append-root-joint t :support-polygon-margin (list 50 50 0 100 50) :epsilon-c 30 :mu 0.3 :use-final-pose nil :default-switching-list nil :use-6dof-p t)~%~%")
    (format t "(nlopt-motion-optimize :x-max ~a :x-hit ~a :id-max ~a :recursive-order ~a :max-eval 100000000 :alg SLSQP :delta (deg2rad 0.01) :eqthre 1e-8 :xtol 1e-10 :ftol 1e-15 :use-all-joint t :use-margin 0.5 :use-append-root-joint t :maxvel-weight 1 :minjerk-weight 5e-3 :modify-ec t :p *p* :interval-num ~a :title \"maximize-speed\" :max-time (* 2 24 60 60) :file-path \"/userdir/logs\")~%~%~%"
            *x-max* *default-x-hit* *id-max* *recursive-order* *interval-num*))

   ((equal motion-choice "backhand-volley-step")
    (setq *x-hit-id* 1)
    (setq *hit-point* *sweet-spot*)
    (setq *target-coords* *racket-coords*)
    (setq *collision-pair*
          (list (list (list *rarm-no-upper-collision* *chest/head-collision* '*epsilon-c*)   (list *rarm-no-shoulder-collision* *chest/head-collision* 0))
                (list (list *rarm-no-upper-collision* *body/torso-collision* '*epsilon-c*)   (list *rarm-collision* *larm-collision* '*epsilon-c*))
                (list (list *larm-no-upper-collision* (append *chest/head-collision* *rarm-collision*) '*epsilon-c*)   (list *racket-capsule-list* *chest/head-collision* '*epsilon-c*))
                (list (list *lleg-no-groin-collision* *rleg-collision* 20)   (list *racket-capsule-list* *larm-collision* '*epsilon-c*))
                (list (list (append *lleg-no-foot-collision* (list *lfoot-plate*)) (list *floor*) 0)   (list *larm-no-shoulder-collision* *body/torso-collision* 20))))


    ;; backhand-volley-step
    ;; list (t0, tl, tg, th, te)
    ;; 時刻間の状態
    ;; contact-flagは(t0 とtlの間)... の4つ
    (setq *contact-flag* (list (list *lfoot* *rfoot* nil nil) (list *lfoot* nil nil nil) (list *lfoot* *rfoot-step* nil nil) (list *lfoot* *rfoot-step* nil nil)))
    (if default-switching-list
        (progn
          (setq *default-switching-list* default-switching-list)
          (setq *default-x-hit* (nth (- (length default-switching-list) *x-hit-id*) default-switching-list)))

      (progn
        (dotimes (i (1- (length *contact-flag*)))
          (push (aref *p* (- (length *p*) 1 i)) *default-switching-list*))

        (setq *default-x-hit* (eval *x-hit*))))


    (format t "*********** backhand-volley-step ***********~%")
    (format t "(nlopt-init :x-max 1.4 :x-hit nil :id-max 10 :recursive-order 5 :use-all-joint t :use-append-root-joint t :support-polygon-margin (list 50 50 0 100 50) :epsilon-c 30 :mu 0.3 :use-final-pose nil :default-switching-list nil :use-6dof-p t)~%~%")
    (format t "(nlopt-motion-optimize :x-max ~a :x-hit ~a :id-max ~a :recursive-order ~a :max-eval 100000000 :alg SLSQP :delta (deg2rad 0.01) :eqthre 1e-8 :xtol 1e-10 :ftol 1e-15 :use-all-joint t :use-margin 0.5 :use-append-root-joint t :maxvel-weight 1 :minjerk-weight 5e-3 :modify-ec t :p *p* :interval-num ~a :title \"maximize-speed\" :max-time (* 2 24 60 60) :file-path \"/userdir/logs\")~%~%~%"
            *x-max* *default-x-hit* *id-max* *recursive-order* *interval-num*))
   ((equal motion-choice "kick-step")
    (setq *x-hit-id* 2)
    (setq *hit-point* *hitting-point*)
    (setq *target-coords* *bat-coords*)
    (setq *collision-pair*
          (list (list (list *rleg-collision* *larm-collision* 0)   (list *rleg-no-groin-collision* *larm-collision* 0))
                (list (list *rleg-no-groin-collision* *body/torso-collision* -30)   (list *rleg-no-groin-collision* *body/torso-collision* 0)) ;; initial poseで既に干渉していることになるので-30にしておく
                (list (list *rarm-no-shoulder-collision* *chest/head-collision* 0)   (list *lleg-no-foot-collision* *larm-no-upper-collision* '*epsilon-c*))
                (list (list *larm-no-shoulder-collision* *chest/head-collision* 0)   (list *body/torso-collision* *larm-no-upper-collision* '*epsilon-c*))
                (list (list (append *rleg-no-foot-collision* (list *rfoot-plate*)) (list *floor*) 0)   (list (append *rleg-no-foot-collision* (list *rfoot-plate*)) (list *floor*) 0))))


    (setq *contact-flag* (list (list *lfoot* *rfoot-step* nil nil) (list *lfoot* nil nil nil) (list *lfoot* nil nil nil) (list *lfoot* *rfoot* nil nil)))
    (if default-switching-list
        (progn
          (setq *default-switching-list* default-switching-list)
          (setq *default-x-hit* (nth (- (length default-switching-list) *x-hit-id*) default-switching-list)))

      (progn
        (dotimes (i (1- (length *contact-flag*)))
          (push (aref *p* (- (length *p*) 1 i)) *default-switching-list*))

        (setq *default-x-hit* (eval *x-hit*))))


    (format t "************* kick-step ************~%")
    (format t "(nlopt-init :x-max 2.6 :x-hit nil :id-max 14 :recursive-order 5 :use-all-joint t :use-append-root-joint t :support-polygon-margin (list 50 50 0 100 50) :epsilon-c 30 :mu 0.3 :use-final-pose nil :default-switching-list nil :use-6dof-p t)~%~%")
    (format t "(nlopt-motion-optimize :x-max ~a :x-hit ~a :id-max ~a :recursive-order ~a :max-eval 100000000 :alg SLSQP :delta (deg2rad 0.01) :eqthre 1e-8 :xtol 1e-10 :ftol 1e-15 :use-all-joint t :use-margin 0.5 :use-append-root-joint t :maxvel-weight 1 :minjerk-weight 5e-3 :modify-ec nil :p *p* :interval-num ~a :title \"maximize-speed\" :max-time (* 2 24 60 60) :file-path \"/userdir/logs\")~%~%~%"
            *x-max* *default-x-hit* *id-max* *recursive-order* *interval-num*))


   ((equal motion-choice "batting-step")
    (setq *x-hit-id* 1)
    (setq *hit-point* *hitting-point*)
    (setq *target-coords* *bat-coords*)
    (setq *collision-pair*
          ;; nil
          (list (list (list *larm-no-upper-collision* *rarm-no-upper-collision* -20)   (list *larm-no-upper-collision* *rarm-collision* '*epsilon-c*))
                (list (list *rarm-no-upper-collision* *chest/head-collision* '*epsilon-c*) (list *rarm-no-shoulder-collision* *chest/head-collision* -20))
                (list (list *rarm-no-upper-collision* *body/torso-collision* '*epsilon-c*) (list *rarm-collision* *body/torso-collision* '*epsilon-c*))
                (list (list *larm-no-upper-collision* *body/torso-collision* '*epsilon-c*) (list *larm-collision* *body/torso-collision* '*epsilon-c*))
                (list (list (append *lleg-no-foot-collision* (list *lfoot-plate*)) (list *floor*) 0)   (list (list *bat-capsule*) *chest/head-collision* '*epsilon-c*))))


    (setq *contact-flag* (list (list *lfoot-step* *rfoot* nil *bat-rhand-grip*) (list nil *rfoot* nil *bat-rhand-grip*) (list *lfoot* *rfoot* nil *bat-rhand-grip*) (list *lfoot* *rfoot* nil nil)))
    (if default-switching-list
        (progn
          (setq *default-switching-list* default-switching-list)
          (setq *default-x-hit* (nth (- (length default-switching-list) *x-hit-id*) default-switching-list)))

      (progn
        (dotimes (i (1- (length *contact-flag*)))
          (push (aref *p* (- (length *p*) 1 i)) *default-switching-list*))

        (setq *default-x-hit* (eval *x-hit*))))


    (format t "************ batting-step ***********~%")
    (format t "(nlopt-init :x-max 2.2 :x-hit nil :id-max 14 :recursive-order 5 :use-all-joint t :use-append-root-joint t :support-polygon-margin (list 50 50 0 100 50) :epsilon-c 30 :mu 0.3 :use-final-pose nil :default-switching-list nil :use-6dof-p t)~%~%")
    (format t "(nlopt-motion-optimize :x-max ~a :x-hit ~a :id-max ~a :recursive-order ~a :max-eval 100000000 :alg SLSQP :delta (deg2rad 0.01) :eqthre 1e-8 :xtol 1e-10 :ftol 1e-15 :use-all-joint t :use-margin 0.5 :use-append-root-joint t :maxvel-weight 1 :minjerk-weight 5e-3 :modify-ec t :p *p* :interval-num ~a :title \"maximize-speed\" :max-time (* 2 24 60 60) :file-path \"/userdir/logs\")~%~%~%"
            *x-max* *default-x-hit* *id-max* *recursive-order* *interval-num*)))


  ;; prepare for the initial state and limit of root joint
  ;; (make-concatenate-rootjoint (funcall *load-pose-func*))

  (setq *dump-nloptlog* nil)
  nil)





;; end-coordsをラケットの*hit-point*に変更
(defun modify-end-coords (&key (robot *robot*))
  (let* ((robot-coords (send robot :copy-worldcoords))
         (robot-angle-vector (send robot :angle-vector)))
    (send robot :reset-pose)
    (send robot :move-to (make-coords) :world)
    (send (send robot *limb* :end-coords) :dissoc *target-coords*)
    (send (send robot *limb* :end-coords) :move-to (send *hit-point* :copy-worldcoords) :world)
    (send (send robot *limb* :end-coords :parent) :assoc *target-coords*)
    (send robot :move-to robot-coords :world)
    (send robot :angle-vector robot-angle-vector)))



;; end-coordsを元の手先に戻す
(defun return-end-coords (&key (robot *robot*))
  (when (boundp '*target-coords*)
    (let* ((robot-coords (send robot :copy-worldcoords))
           (robot-angle-vector (send robot :angle-vector)))
      (send robot :reset-pose)
      (send robot :move-to (make-coords) :world)
      (send (send robot *limb* :end-coords :parent) :dissoc *target-coords*)
      (send (send robot *limb* :end-coords) :move-to (send *limb-end-coords-default* :copy-worldcoords) :world)
      (send (send robot *limb* :end-coords) :assoc *target-coords*)
      (send robot :move-to robot-coords :world)
      (send robot :angle-vector robot-angle-vector))))







;; making root-joint initial state and upper or lower limit
(defun make-concatenate-rootjoint (rootcoords-list &key (concatenate-root-joint *concatenate-root-joint*))
  (let* ((rootjoint-list
          (mapcar #'(lambda (rc)
                      (concatenate float-vector (send rc :worldpos) (mapcar #'rad2deg (reverse (car (rpy-angle (send rc :worldrot)))))))

                  rootcoords-list))


         (mean-rootjoint (vector-mean rootjoint-list))
         (variance-rootjoint
          (vector-mean
           (mapcar #'(lambda (rj)
                       (map float-vector #'(lambda (x) (* x x)) (v- rj mean-rootjoint)))

                   rootjoint-list)))
         (standard-deviation-rootjoint (map float-vector #'sqrt variance-rootjoint))
         (replace-src (concatenate float-vector
                                   mean-rootjoint
                                   (v- mean-rootjoint (scale 3 standard-deviation-rootjoint))
                                   (v+ mean-rootjoint (scale 3 standard-deviation-rootjoint)))))


    (replace concatenate-root-joint replace-src)))






;; transpose
(defun simple-jacobian-transpose
    (v1 v2 func &key (delta 1e-6) (w (/ (length v2) (length v1))))
  (let* ((h (length v1))
         (buf (instantiate float-vector w))
         tmp
         (org (instantiate float-vector w))
         (tmp-jacobian))

    (funcall func v1 org)
    (dotimes (i h)
      (setq tmp (aref v1 i))
      (setf (aref v1 i) (+ (aref v1 i) delta))
      (funcall func v1 buf)
      (dotimes (j w)
        (let* ((tmp-jacob-ij (/ (- (aref buf j) (aref org j)) delta)))
          (setf (aref v2 (+ j (* i w))) tmp-jacob-ij))

        (setf (aref v1 i) tmp)))
    (setq tmp-jacobian (copy-seq v2))
    (dotimes (i w)
      (dotimes (j h)
        (setf (aref v2 (+ j (* i h))) (aref tmp-jacobian (+ i (* j w))))))))








;; {{{ 目的関数 min f(x)
;; v1: 設計変数p ((33+4) * *id-max* + timing vars)
;; v2: 1次元
;; 返り値: 成功なら0?
(defun-c-callable motion-f ((v1 :string) (v2 :string)) :integer
  (when (equal (aref v1 0) (aref #f(nan) 0)) ;; (aref #f(nan) 0)はnanを取り出している
    (dotimes (i (length v1))
      (setf (aref v1 i) (aref *tmp-p* i)))) ;; tmp-pに入力を保存している


  ;; 躍度最小
  (setf (aref v2 0) 0)

  (let* ((integral-minjerk 0) (hitting-maxvel 0))
    (when (not (= *minjerk-weight* 0))
      ;; 躍度の2乗の積分の和
      (let* ((id 0)
             (integral-objective-coeff-matrix (send (car *bspline*) :calc-integral-objective-coeff-matrix :n 3)))
        (mapc
         #'(lambda (jl bs)
             (setq integral-minjerk
                   (+ integral-minjerk
                      (deg2rad (deg2rad ;; すでに2乗されているので足す前に2回deg2rad
                                (v.
                                 (transform (subseq v1 id (+ id *id-max*))
                                            ;;(send bs :calc-integral-objective-coeff-matrix :n 3))
                                            integral-objective-coeff-matrix)
                                 (subseq v1 id (+ id *id-max*)))))))



             (setq id (+ id *id-max*)))

         *jlist* *bspline*) ;; ここでは仮想ジョイントの躍度を考慮しない
        (when (send (car (send *robot* :links)) :joint) ;; virtual jointありのとき
          (mapc
           #'(lambda (bs)
               (setq integral-minjerk
                     (+ integral-minjerk
                        (* 1e-6 ;; すでに2乗されているので足す前に2回mm2m
                           (v.
                            (transform (subseq v1 id (+ id *id-max*))
                                       ;;(send bs :calc-integral-objective-coeff-matrix :n 3))
                                       integral-objective-coeff-matrix)
                            (subseq v1 id (+ id *id-max*))))))



               (setq id (+ id *id-max*)))

           (subseq *bspline* (length *jlist*) (+ (length *jlist*) 3)))
          (mapc
           #'(lambda (bs)
               (setq integral-minjerk
                     (+ integral-minjerk
                        (deg2rad (deg2rad ;; すでに2乗されているので足す前に2回deg2rad
                                  (v.
                                   (transform (subseq v1 id (+ id *id-max*))
                                              ;;(send bs :calc-integral-objective-coeff-matrix :n 3))
                                              integral-objective-coeff-matrix)
                                   (subseq v1 id (+ id *id-max*)))))))



               (setq id (+ id *id-max*)))

           (subseq *bspline* (+ (length *jlist*) 3) (+ (length *jlist*) 6))))))




    ;; hitting-poseにおける速度の最大化
    (when (not (= *maxvel-weight* 0))
      (let* ((id 0) (root-joint-dq nil) hitting-direction hitting-sweet-spot hitting-jacobian hitting-dq hitting-dr)
        ;; ここは最適化に含まれる時間を設定するところ
        ;; 時刻を消したかったら(aref v1 (- (length v1) *x-hit-id*))をx(定数)とすればよい
        ;; こうするとx(定数)における速度が評価関数になる
        (mapc
         #'(lambda (jl bs)
             (send jl :joint-angle ;; 関節ごと
                   ;; (send bs :calc (aref v1 (- (length v1) *x-hit-id*)) (subseq v1 id (+ id *id-max*)))) ;; subseqは関節毎にpを抜き取ってる
                   (send bs :calc *default-x-hit* (subseq v1 id (+ id *id-max*)))) ;; subseqは関節毎にpを抜き取ってる
             ;; send bs :calc 時刻 ベクトル(基底関数の頂点の高さ(*p*と同じなのだが))->関節角がとれる
             ;; send bs :calc-delta 時刻 ベクトル(基底関数の頂点の高さ(*p*と同じなのだが))->関節速度が取れる
             ;; (send jl :put :dq (deg2rad (send bs :calc-delta (aref v1 (- (length v1) *x-hit-id*)) (subseq v1 id (+ id *id-max*)) :n 1))) ;; 関節速度の単位は[rad]
             (send jl :put :dq (deg2rad (send bs :calc-delta *default-x-hit* (subseq v1 id (+ id *id-max*)) :n 1))) ;; 関節速度の単位は[rad]
             (setq id (+ id *id-max*)))

         *jlist* *bspline*)
        ;; virtual jointがあったら
        (if (send (car (send *robot* :links)) :joint)
            (progn
              (send (send (car (send *robot* :links)) :joint) :joint-angle ;; virtual jointのjoint-angleがとれる (xyz yprが格納されている)
                    (coerce
                     (mapcar
                      #'(lambda (bs)
                          ;; (push (send bs :calc-delta (aref v1 (- (length v1) *x-hit-id*)) (subseq v1 id (+ id *id-max*)) :n 1) root-joint-dq) ;; 先にルートリンクの速度を取得
                          (push (send bs :calc-delta *default-x-hit* (subseq v1 id (+ id *id-max*)) :n 1) root-joint-dq) ;; 先にルートリンクの速度を取得
                          ;; (send bs :calc (aref v1 (- (length v1) *x-hit-id*)) (subseq v1 id (setq id (+ id *id-max*)))))
                          (send bs :calc *default-x-hit* (subseq v1 id (setq id (+ id *id-max*)))))
                      (subseq *bspline* (length *jlist*) (+ (length *jlist*) 6))) ;; virtual joint
                     float-vector))

              (setq root-joint-dq (reverse root-joint-dq)) ;; ルートリンクの速度の単位は[m]
              ;; 単位換算
              (let* ((cnt 0))
                (dolist (sdq (subseq root-joint-dq 0 3))
                  (setf (nth cnt root-joint-dq) (* 1e-3 sdq))
                  (incf cnt))
                (dolist (adq (subseq root-joint-dq 3 6))
                  (setf (nth cnt root-joint-dq) (deg2rad adq))
                  (incf cnt))))


            ;; virtual jointなしの場合
          (progn
            (send *robot* :fix-leg-to-coords *lfoot* :lleg)))



        ;; *hit-point*はラケットがあったらラケットの先まで計算される
        (setq hitting-sweet-spot (send *hit-point* :copy-worldcoords)) ;; 打つ姿勢における*hit-point*
        (setq hitting-direction (v- (send hitting-sweet-spot :transform-vector #f(0 0 1)) (send hitting-sweet-spot :worldpos))) ;; *hit-point*のz方向以外の速度を最大化する

        ;; 打つ姿勢におけるヤコビアン（単位系は[rad],[m]）を計算
        (setq hitting-jacobian
              (send *robot* :calc-jacobian-from-link-list
                    ;; virtual joint
                    (if (send (car (send *robot* :links)) :joint)
                        (progn
                          (setq hitting-dq (coerce (flatten root-joint-dq (send-all *jvel-list* :get :dq)) float-vector)) ;; ここで打つ姿勢での関節速度を先に計算
                          (flatten (list (car (send *robot* :links)) (send *robot* :torso :links) (send *robot* *limb* :links)))) ;; BODY + CHEST_LINK0~2 + RARM_LINK0~7

                      (progn
                        (setq hitting-dq (coerce (send-all *jvel-list* :get :dq) float-vector)) ;; ここで打つ姿勢での関節速度を先に計算
                        (flatten (send *robot* :torso :links) (send *robot* *limb* :links)))) ;; CHEST_LINK0~2 + RARM_LINK0~7


                    :move-target (send *robot* *limb* :end-coords)
                    :transform-coords (make-coords)
                    :translation-axis '(t)
                    :rotation-axis '(nil))) ;; 3(or6)行17(= (length (+ 6dof chest-link rarm-link)))列

        ;; ヤコビアンと関節速度から手先速度を計算
        (setq hitting-dr (transform hitting-jacobian hitting-dq))
        ;; 評価値は最大化したい方向の正射影ベクトルの大きさ
        (if *vel-positive*
          (setq hitting-maxvel (* -1e3 (v. hitting-dr hitting-direction)))  ;; 面の正方向の速度を最大化するため面の反対方向の速度の最小化問題にする、スケール合わせのために1000倍する
          (setq hitting-maxvel (* 1e3 (v. hitting-dr hitting-direction)))  ;; バックハンドの場合は面の正方向の速度を最小化して、スケール合わせのために1000倍する
          )
          ))



    ;; 各値の重み付けによって評価関数を定める
    (setf (aref v2 0)
          (+ (* *minjerk-weight* integral-minjerk)
             (* *maxvel-weight* hitting-maxvel))))




  (setq *tmp-p* (copy-seq v1))
  (when *dump-nloptlog*
    ;; motion-dfを使うか
    (if *use-delta-func*
        (progn
          (when (null (eq *eval-cnt-f* *eval-cnt-f-old*))
            (format t "objective functiion: ~a~%" v2)
            (with-open-file
             (f (concatenate string *file-name* "_obj.dat") :direction :output :if-exists :append)
             (dolist (obj-data (coerce v2 cons))
               (format f "~a   " obj-data))

             (format f "~%"))

            (with-open-file
             (f (concatenate string *file-name* "_p.dat") :direction :output :if-exists :append)
             (format f "~a~%" *tmp-p*))

            (setq *eval-cnt-f-old* *eval-cnt-f*)))


      (progn
        (format t "objective functiion: ~a~%" v2)
        (with-open-file
         (f (concatenate string *file-name* "_obj.dat") :direction :output :if-exists :append)
         (dolist (obj-data (coerce v2 cons))
           (format f "~a   " obj-data))

         (format f "~%"))

        (with-open-file
         (f (concatenate string *file-name* "_p.dat") :direction :output :if-exists :append)
         (format f "~a~%" *tmp-p*)))))




  0)
;; }}}

(defun-c-callable motion-df ((v1 :string) (v2 :string)) :integer
  ;; TODO
  ;; warning nloptがtransposeなのか調べる
  (simple-jacobian-transpose v1 v2 (read-from-string "motion-f") :delta *delta*)
  (incf *eval-cnt-f*)
  (show-optimized-motion v1 :x-step 0.1 :real-time nil)
  0)


;; {{{ 等式制約 g(x) = 0
;; v1: 設計変数p ((33+4) * *id-max* + timing vars)
;; v2: 初期姿勢，速度，加速度の2乗誤差の和: 3次元
;;     タスク達成姿勢の2乗誤差の和: 1次元
;;     最終姿勢の速度，加速度の2乗誤差の和: 2次元
;;     足の接地位置の距離の2乗和: 1次元
;; 返り値: 成功したら0?
(defun-c-callable motion-g ((v1 :string) (v2 :string)) :integer
  (when (equal (aref v1 0) (aref #f(nan) 0)) ;; (aref #f(nan) 0)はnanを返す NaNチェック
    (dotimes (i (length v1))
      (setf (aref v1 i) (aref *tmp-p* i)))) ;; ログ用に*tmp-p*をつける


  (let* ((id 0)
         (x-min-pose nil) (x-min-vel nil) (x-min-acc nil)
         (x-hit-pose nil) (x-hit-ee nil)
         (x-max-pose nil) (x-max-vel nil) (x-max-acc nil)
         (contact-tm-cnt 0)
         (g-dim-id 0)
         grounding-id)
    ;; v1(=p)に対する各時刻(x-min, x-hit, x-max)の関節角を計算
    (mapc
     #'(lambda (jl bs)
         (push (send bs :calc *x-min* (subseq v1 id (+ id *id-max*))) x-min-pose) ;; 時刻x-minにおける関節角度
         (push (send bs :calc-delta *x-min* (subseq v1 id (+ id *id-max*)) :n 1) x-min-vel) ;; 時刻x-minにおける関節角速度 (nは微分階数)
         (push (send bs :calc-delta *x-min* (subseq v1 id (+ id *id-max*)) :n 2) x-min-acc) ;; 時刻x-minにおける関節角加速度
         ;; (push (send bs :calc (aref v1 (- (length v1) *x-hit-id*)) (subseq v1 id (+ id *id-max*))) x-hit-pose) ;; 時刻x-hitにおける関節角度
         (when *use-hit-pose*
             (progn
                 (push (send bs :calc *default-x-hit* (subseq v1 id (+ id *id-max*))) x-hit-pose) ;; 時刻x-hitにおける関節角度
             ))


         (when *use-final-pose*
           (push (send bs :calc *x-max* (subseq v1 id (+ id *id-max*))) x-max-pose)) ;; 時刻x-maxにおける関節角度
   ;;(push (send bs :calc (aref *p* (- (length v1) 2)) (subseq v1 id (+ id *id-max*))) x-max-pose) ;; 着地時刻の関節角度

         (push (send bs :calc-delta *x-max* (subseq v1 id (+ id *id-max*)) :n 1) x-max-vel) ;; 時刻x-maxにおける関節角速度
         (push (send bs :calc-delta *x-max* (subseq v1 id (+ id *id-max*)) :n 2) x-max-acc) ;; 時刻x-maxにおける関節角加速度
         (setq id (+ id *id-max*)))

     *jlist* *bspline*);; *bspline*に仮想ジョイント分追加されているので数合わせ用に*jlist*を渡す
    (when (send (car (send *robot* :links)) :joint) ;; virtual joint
      (mapc
       #'(lambda (bs)
           (push (send bs :calc *x-min* (subseq v1 id (+ id *id-max*))) x-min-pose) ;; 時刻x-minにおける関節角度
           (push (send bs :calc-delta *x-min* (subseq v1 id (+ id *id-max*)) :n 1) x-min-vel) ;; 時刻x-minにおける関節角速度
           (push (send bs :calc-delta *x-min* (subseq v1 id (+ id *id-max*)) :n 2) x-min-acc) ;; 時刻x-minにおける関節角加速度
           ;; (push (send bs :calc (aref v1 (- (length v1) *x-hit-id*)) (subseq v1 id (+ id *id-max*))) x-hit-pose) ;; 時刻x-hitにおける関節角度
           (when *use-hit-pose*
               (push (send bs :calc *default-x-hit* (subseq v1 id (+ id *id-max*))) x-hit-pose)) ;; 時刻x-hitにおける関節角度
           (when *use-final-pose*
             (push (send bs :calc *x-max* (subseq v1 id (+ id *id-max*))) x-max-pose)) ;; 時刻x-maxにおける関節角度
     ;; (push (send bs :calc (aref *p* (- (length v1) 2)) (subseq v1 id (+ id *id-max*))) x-max-pose) ;; 着地時刻における関節角度

           (push (send bs :calc-delta *x-max* (subseq v1 id (+ id *id-max*)) :n 1) x-max-vel) ;; 時刻x-maxにおける関節角速度
           (push (send bs :calc-delta *x-max* (subseq v1 id (+ id *id-max*)) :n 2) x-max-acc) ;; 時刻x-maxにおける関節角加速度
           (setq id (+ id *id-max*)))

       (subseq *bspline* (length *jlist*) (+ (length *jlist*) 6)))) ;; 最後のvirtual joint6自由度

     (when *use-hit-ee*
         (progn
             (change-robot-state-specific-time v1 *default-x-hit* (send (car (send *robot* :links)) :joint))
             (setq x-hit-ee-pos (send (send *sweet-spot* :copy-worldcoords) :worldpos))
             (setq x-hit-ee-rpy (nth 0 (rpy-angle (send (send *sweet-spot* :copy-worldcoords) :worldrot))))
             (setq x-hit-ee (concatenate float-vector x-hit-ee-pos (mapcar #'rad2deg (reverse x-hit-ee-rpy))))

             (setq *target-hitting-ee*
                 (let* ((pose-wc (progn (funcall *load-pose-func* :pose (nth 1 *key-pose-id*)) (send *sweet-spot* :copy-worldcoords)))
                        (ee-pos (send pose-wc :worldpos))
                        (ee-rpy (nth 0 (rpy-angle (send pose-wc :worldrot)))))
                       (concatenate float-vector ee-pos (mapcar #'rad2deg (reverse ee-rpy)))
                       ))
                 ))


    (setq x-min-pose (coerce (reverse x-min-pose) float-vector))
    (setq x-min-vel (coerce (reverse x-min-vel) float-vector))
    (setq x-min-acc (coerce (reverse x-min-acc) float-vector))
    (when *use-hit-pose*
      (setq x-hit-pose (coerce (reverse x-hit-pose) float-vector)))
    (when *use-hit-ee*
      (setq x-hit-ee (coerce x-hit-ee float-vector)))
    (when *use-final-pose*
      (setq x-max-pose (coerce (reverse x-max-pose) float-vector)))

    (setq x-max-vel (coerce (reverse x-max-vel) float-vector))
    (setq x-max-acc (coerce (reverse x-max-acc) float-vector))

    ;; 各姿勢の２乗誤差を計算してv2に格納
    (let* ((v2-0 (v- x-min-pose *initial-pose*))
           (v2-1 x-min-vel)
           (v2-2 x-min-acc)
           (v2-3 (if *use-hit-pose* (v- x-hit-pose *hitting-pose*) nil))
           (v2-3.5 (if *use-hit-ee* (v- x-hit-ee *target-hitting-ee*) nil))
           (v2-4 (if *use-final-pose* (v- x-max-pose *final-pose*) nil))
           (v2-5 x-max-vel)
           (v2-6 x-max-acc))

      (setf (aref v2 g-dim-id) (norm2 (map float-vector #'(lambda (x) (deg2rad x)) v2-0) ))  (incf g-dim-id)
      (setf (aref v2 g-dim-id) (norm2 (map float-vector #'(lambda (x) (deg2rad x)) v2-1) ))  (incf g-dim-id)
      (setf (aref v2 g-dim-id) (norm2 (map float-vector #'(lambda (x) (deg2rad x)) v2-2) ))  (incf g-dim-id)
      (when *use-hit-pose*
        (setf (aref v2 g-dim-id) (norm2 (map float-vector #'(lambda (x) (deg2rad x)) v2-3) ))  (incf g-dim-id))
      (when *use-hit-ee*
        (setf (aref v2 g-dim-id) (norm2 (map float-vector #'(lambda (x) (deg2rad x)) v2-3.5) ))  (incf g-dim-id))
      (when *use-final-pose*
        (setf (aref v2 g-dim-id) (norm2 (map float-vector #'(lambda (x) (deg2rad x)) v2-4) ))  (incf g-dim-id))

      (setf (aref v2 g-dim-id) (norm2 (map float-vector #'(lambda (x) (deg2rad x)) v2-5) ))  (incf g-dim-id)
      (setf (aref v2 g-dim-id) (norm2 (map float-vector #'(lambda (x) (deg2rad x)) v2-6) ))  (incf g-dim-id))


    ;; 等式条件側に足の条件を追加
    (when (send (car (send *robot* :links)) :joint)
      ;; 離散化された関節角度における両足end-coordsと*lfoot*, *rfoot*の誤差ゼロ
      (let* (integral-endcoords discretized-endcoords (tmp-endcoords-list nil) current-av current-rc current-lfoot current-rfoot current-larm current-rarm (interval-num (float *interval-num*)))
        (mapc
         #'(lambda (rate)
             (let* ((id 0)
                    (x-buf (+ *x-min* (* rate (- *x-max* *x-min*))))
                    (switching-tm-list (list *x-max*)))

               (dotimes (i (1- (length *contact-flag*)))
                 (push (aref *p-orig* (- (length *p-orig* #|v1|#) 1 i)) switching-tm-list))


               (setq discretized-endcoords nil)
               (mapc
                #'(lambda (bs)
                    (push (send bs :calc x-buf
                                (subseq v1 id (+ id *id-max*)) ) discretized-endcoords)
                    (setq id (+ id *id-max*)))

                *bspline*)
               (setq discretized-endcoords (reverse discretized-endcoords))
               (setq current-av (coerce (subseq discretized-endcoords 0 (length *jlist*)) float-vector))
               (send *robot* :angle-vector current-av)
               (setq current-rc (coerce (subseq discretized-endcoords (length *jlist*) (+ (length *jlist*) 6)) float-vector))
               ;; (send *robot* :move-to (make-coords :pos (coerce (subseq current-rc 0 3) float-vector) :rpy (mapcar #'deg2rad (subseq current-rc 0 3))) :world) ;; root-coordsは[mm],[deg]で計算する
               (send (send (car (send *robot* :links)) :joint) :joint-angle current-rc)
               (setq current-lfoot (send *robot* :lleg :end-coords :copy-worldcoords))
               (setq current-rfoot (send *robot* :rleg :end-coords :copy-worldcoords))
               (setq current-larm (send *robot* :larm :end-coords :copy-worldcoords))
               (setq current-rarm (send *robot* :rarm :end-coords :copy-worldcoords))
               (push (+ (if (nth 0 (nth contact-tm-cnt *contact-flag*)) ;; lleg flag
                            (+ (norm2 (scale 1e-3 (v- (send current-lfoot :worldpos)
                                                      (send (nth 0 (nth contact-tm-cnt *contact-flag*)) :worldpos)))) ;; *lfoot-pos*

                               (norm2 (v- (matrix2quaternion (send current-lfoot :worldrot))
                                          (matrix2quaternion (send (nth 0 (nth contact-tm-cnt *contact-flag*)) :worldrot))))) ;; *lfoot-qtn*


                          0)
                        (if (nth 1 (nth contact-tm-cnt *contact-flag*)) ;; rleg flag
                            (+ (norm2 (scale 1e-3 (v- (send current-rfoot :worldpos)
                                                      (send (nth 1 (nth contact-tm-cnt *contact-flag*)) :worldpos)))) ;; *rfoot-pos*

                               (norm2 (v- (matrix2quaternion (send current-rfoot :worldrot))
                                          (matrix2quaternion (send (nth 1 (nth contact-tm-cnt *contact-flag*)) :worldrot))))) ;; *rfoot-qtn*


                          0)
                        (if (nth 2 (nth contact-tm-cnt *contact-flag*)) ;; larm flag
                            (+ (norm2 (scale 1e-3 (v- (send current-larm :worldpos)
                                                      (send (nth 2 (nth contact-tm-cnt *contact-flag*)) :worldpos)))))

                               ;; (norm2 (v- (matrix2quaternion (send current-larm :worldrot))
                               ;;            (matrix2quaternion (send (nth 2 (nth contact-tm-cnt *contact-flag*)) :worldrot))
                               ;;            ))

                          0)
                        (if (nth 3 (nth contact-tm-cnt *contact-flag*)) ;; rarm flag
                            (+ (norm2 (scale 1e-3 (v- (send current-rarm :worldpos)
                                                      (send (nth 3 (nth contact-tm-cnt *contact-flag*)) :worldpos)))))

                               ;; (norm2 (v- (matrix2quaternion (send current-rarm :worldrot))
                               ;;            (matrix2quaternion (send (nth 3 (nth contact-tm-cnt *contact-flag*)) :worldrot))
                               ;;            ))

                          0))

                     tmp-endcoords-list)

               (when (> x-buf (nth contact-tm-cnt switching-tm-list))
                 (incf contact-tm-cnt))))



         (mapcar #'(lambda (x) (/ x interval-num)) (my-range-2 0 interval-num)))
        ;; '(0.0 0.1 0.2 0.3 0.4 0.5 0.6 0.7 0.8 0.9 1.0)) ;; 33(=(length jlist))*11個の要素をもつfloat-vector
        (setq integral-endcoords (* (apply #'+ tmp-endcoords-list) (/ (- *x-max* *x-min*) interval-num)))
        (setf (aref v2 g-dim-id) (* integral-endcoords 10)) (incf g-dim-id))) ;; 他の等式条件のスケールに適当に合わせる



    ;; 着地時の速度ゼロ
    (setq grounding-id (- (length *contact-flag*) 1))
    (while (< 0 grounding-id)
      (let* ((root-joint-dq nil) grounding-jacobian grounding-dq grounding-dr
             (grounding-tm (aref *p* (- (length v1) grounding-id)))
             (grounding-leg
              (if (and (not (car (nth (- (length *contact-flag*) grounding-id 1) *contact-flag*)))
                       (car (nth (- (length *contact-flag*) grounding-id) *contact-flag*)))
                  :lleg
                (if (and (not (nth 1 (nth (- (length *contact-flag*) grounding-id 1) *contact-flag*)))
                         (nth 1 (nth (- (length *contact-flag*) grounding-id) *contact-flag*)))
                    :rleg
                  nil))))
                 ;; *contact-flag*の着地直前の接触フラグの左足がnilなら:llegを着地させる



        (when grounding-leg
          (setq id 0)
          (mapc
           #'(lambda (jl bs)
               (send jl :joint-angle
                     (send bs :calc grounding-tm (subseq v1 id (+ id *id-max*))))
               (send jl :put :dq (deg2rad (send bs :calc-delta grounding-tm (subseq v1 id (+ id *id-max*)) :n 1))) ;; 関節速度の単位は[rad]
               (setq id (+ id *id-max*)))

           *jlist* *bspline*)
          (if (send (car (send *robot* :links)) :joint)
              (progn
                (send (send (car (send *robot* :links)) :joint) :joint-angle
                      (coerce
                       (mapcar
                        #'(lambda (bs)
                            (push (send bs :calc-delta grounding-tm (subseq v1 id (+ id *id-max*)) :n 1) root-joint-dq) ;; 先にルートリンクの速度を取得
                            (send bs :calc grounding-tm (subseq v1 id (setq id (+ id *id-max*)))))
                        (subseq *bspline* (length *jlist*) (+ (length *jlist*) 6)))
                       float-vector))

                (setq root-joint-dq (reverse root-joint-dq)) ;; ルートリンクの速度の単位は[m]
                (let* ((cnt 0))
                  (dolist (sdq (subseq root-joint-dq 0 3))
                    (setf (nth cnt root-joint-dq) (* 1e-3 sdq))
                    (incf cnt))
                  (dolist (adq (subseq root-joint-dq 3 6))
                    (setf (nth cnt root-joint-dq) (deg2rad adq))
                    (incf cnt))))


            (progn
              (send *robot* :fix-leg-to-coords *lfoot* :lleg)))



          ;; 着地姿勢におけるヤコビアン（単位系は[rad],[m]）を計算
          (setq grounding-jacobian
                (send *robot* :calc-jacobian-from-link-list
                      (if (send (car (send *robot* :links)) :joint)
                          (progn
                            (setq grounding-dq (coerce (flatten root-joint-dq (send-all (send *robot* grounding-leg :joint-list) :get :dq)) float-vector)) ;; ここで着地姿勢での関節速度を先に計算
                            (flatten (list (car (send *robot* :links)) (send *robot* grounding-leg :links)))) ;; BODY + LEG_LINK0~5

                        (progn
                          (setq grounding-dq (coerce (send-all (send *robot* grounding-leg :joint-list) :get :dq) float-vector)) ;; ここで着地姿勢での関節速度を先に計算
                          (flatten (send *robot* grounding-leg :links)))) ;; LEG_LINK0~5


                      :move-target (send *robot* *limb* :end-coords)
                      :transform-coords (make-coords)
                      :translation-axis '(t)
                      :rotation-axis '(t))) ;; 6

          ;; ヤコビアンと関節速度から足先速度を計算
          (setq grounding-dr (transform grounding-jacobian grounding-dq))
          (setf (aref v2 g-dim-id) (* (norm2 grounding-dr) 10))))


      (setq grounding-id (1- grounding-id))))



  (when *dump-nloptlog*
    (if *use-delta-func*
        (progn
          (when (null (eq *eval-cnt-g* *eval-cnt-g-old*))
            (format t "equality condition: ~a~%" v2)
            (with-open-file
             (f (concatenate string *file-name* "_eq.dat") :direction :output :if-exists :append)
             (dolist (eq-data (coerce v2 cons))
               (format f "~a   " eq-data))

             (format f "~%"))

            (setq *eval-cnt-g-old* *eval-cnt-g*)))


      (progn
        (format t "equality condition: ~a~%" v2)
        (with-open-file
         (f (concatenate string *file-name* "_eq.dat") :direction :output :if-exists :append)
         (dolist (eq-data (coerce v2 cons))
           (format f "~a   " eq-data))

         (format f "~%")))))




  0)
;; }}}

(defun-c-callable motion-dg ((v1 :string) (v2 :string)) :integer
  (simple-jacobian-transpose v1 v2 (read-from-string "motion-g") :delta *delta*)
  (incf *eval-cnt-g*)
  0)

;; {{{ 不等式制約 h(x) <= 0
;; v1: 設計変数p ((33+4) * *id-max* + timing vars)
;; v2: 関節速度制限 *jvel-list* (chest3, *limb*(rarmなら8)) 11 * (*id-max* - 1) = 143
;;     支持多角形からZMPまでの最短: interval-num + 1 = 21
;;     並進摩擦: interval-num + 1 = 21
;;     回転摩擦: interval-num + 1 = 21
;;     コリジョン: (length *collision-pair*) * (0-10 + 11-20) = 4 * 21 = 84
;;     トルク制限: *jvel-list* * (interval-num + 1) = 11 * 21 = 231
;;     時間制約 0
;; 返り値: 成功したら0?
(defun-c-callable motion-h ((v1 :string) (v2 :string)) :integer
  (when (equal (aref v1 0) (aref #f(nan) 0))
    (dotimes (i (length v1))
      (setf (aref v1 i) (aref *tmp-p* i))))


  ;; for calculate jvel limit
  (let* ((max-jvel-vector ;; [rad/s]
          (coerce (flatten
                   (mapcar
                    #'(lambda (j) (make-list (- *id-max* 1) :initial-element (send j :max-joint-velocity))) ;; 速度の条件は(- *id-max* 1)個
                    *jvel-list*))
                  float-vector))
         (id 0)
         h-dim-id
         ;; for calculate jvel weight condition
         (dp nil)
         max-ieq-cond
         (discretized-zmp #f(0 0 0))
         inside-zmp
         (discretized-force #f(0 0 0))
         (discretized-zmp-moment #f(0 0 0))
         (interval-num (float *interval-num*))
         (contact-tm-cnt 0))

    (mapc
     #'(lambda (jvl bs)
         (setq id (* (position jvl *jlist*) *id-max*)) ;;chest0~2, rarm0~7を抽出
         (let* ((tmp-dp (transform (subseq v1 id (+ id *id-max*)) (send bs :calc-delta-matrix :n 1))))
           (setq tmp-dp (subseq tmp-dp 0 (- (length tmp-dp) 1))) ;; 速度の条件は(- *id-max* 1)個
           (setq dp (concatenate float-vector dp tmp-dp)))) ;; dq = d(p^T b) = dp^T bとしたときのdp [deg/s]


     *jvel-list* *bspline*)
    (setq dp (map float-vector #'(lambda (dpi) (deg2rad (abs dpi))) dp)) ;; |dp| < max-jvel-vectorで事足りる

    (setq max-ieq-cond (v- dp max-jvel-vector))


    ;; v2の各要素にmax-ieq-cond
    (setq h-dim-id 0)
    (map float-vector
         #'(lambda (max-ieq)
             (setf (aref v2 h-dim-id) max-ieq)
             (incf h-dim-id))

         max-ieq-cond)

    ;; 離散化されたzmpと支持多角形の縁までの距離
    ;; collision check
    (setq id (length dp))
    (mapc
     #'(lambda (rate) ;; *x-min*と*x-max*の内分時間0-1 interval-numは20でやることが多そうだから0-20で21個
         (let* ((x-buf (+ *x-min* (* rate (- *x-max* *x-min*))))
                (switching-tm-list (list *x-max*)))

           (dotimes (i (1- (length *contact-flag*)))
             (push (aref *p-orig* (- (length *p-orig* #|v1|#) 1 i)) switching-tm-list))


           ;; (calc-specific-time-force-zmp v1 x-buf :zmp-at-specific-time discretized-zmp :force-at-specific-time discretized-force :zmp-moment-at-specific-time discretized-zmp-moment :contact-flag (nth contact-tm-cnt *contact-flag*) :step-time (/ (- *x-max* *x-min*) interval-num))
           (if (< x-buf (* 3 (/ (- *x-max* *x-min*) interval-num)))
               (calc-specific-time-force-zmp v1 x-buf :zmp-at-specific-time discretized-zmp :force-at-specific-time discretized-force :zmp-moment-at-specific-time discretized-zmp-moment :contact-flag (nth contact-tm-cnt *contact-flag*) :step-time (/ (- *x-max* *x-min*) interval-num))
             (let ((jid 0))
               (mapc
                #'(lambda (jl bs)
                    (list
                     (send jl :joint-angle
                           (send bs :calc x-buf (subseq v1 jid (setq jid (+ jid *id-max*)))))))

                *jlist* *bspline*)
               ;; root-joint
               (if (send (car (send *robot* :links)) :joint)
                   (progn
                     (send (send (car (send *robot* :links)) :joint) :joint-angle
                           (coerce
                            (mapcar
                             #'(lambda (bs)
                                 (send bs :calc x-buf (subseq v1 jid (setq jid (+ jid *id-max*)))))

                             (subseq *bspline* (length *jlist*) (+ (length *jlist*) 6)))
                            float-vector))

                     (when (nth 0 (nth contact-tm-cnt *contact-flag*))
                       (send *robot* :lleg :inverse-kinematics (nth 0 (nth contact-tm-cnt *contact-flag*)) :rotation-axis t :revert-if-fail nil))

                     (when (nth 1 (nth contact-tm-cnt *contact-flag*))
                       (send *robot* :rleg :inverse-kinematics (nth 1 (nth contact-tm-cnt *contact-flag*)) :rotation-axis t :revert-if-fail nil)))


                 (progn
                   (send *robot* :fix-leg-to-coords *lfoot* :lleg)))


               (setq discretized-zmp (send *robot* :calc-zmp (send *robot* :angle-vector) (send (car (send *robot* :links)) :copy-worldcoords) :dt (/ (- *x-max* *x-min*) interval-num)))
               (setq discretized-force (send (car (send *robot* :links)) :force))
               (setq discretized-zmp-moment (send *robot* :get :zmp-moment))))


           (make-support-face :front *sp-front* :back *sp-back* :inside *sp-inside* :outside *sp-outside* :oneleg *sp-oneleg* :contact-flag (nth contact-tm-cnt *contact-flag*))
           ;; (send *arrow* :locate discretized-zmp :world) (send *irtviewer* :draw-objects)

           (setq inside-zmp
                 (* 1e-3
                    (apply #'min
                           (mapcar
                            #'(lambda (edge)
                                (send edge :distance discretized-zmp)) ;; zmpから*support-face*の各辺までの距離

                            (send *support-face* :edges)))))
                            ;; zmpまでの距離の最小値[m]

           (when (eq (send *support-face* :insidep discretized-zmp) :inside)
             (setq inside-zmp (- inside-zmp))) ;; 負であればよい(:inside)

           ;; zmpの条件
           (setf (aref v2 id) inside-zmp)

           ;; 摩擦の条件
           (let* ((fx (aref discretized-force 0)) (fy (aref discretized-force 1)) (fz (aref discretized-force 2))
                  (tz (aref discretized-zmp-moment 2))
                  (current-lfoot-pos (send (send *robot* :lleg :end-coords :copy-worldcoords) :worldpos))
                  (current-rfoot-pos (send (send *robot* :rleg :end-coords :copy-worldcoords) :worldpos))
                  (lfoot->zmp (v- discretized-zmp current-lfoot-pos)) ;; [mm]
                  (rfoot->zmp (v- discretized-zmp current-rfoot-pos)) ;; [mm]
                  (lfoot->rfoot (v- current-rfoot-pos current-lfoot-pos))
                  (distribution-ratio (if (nth 0 (nth contact-tm-cnt *contact-flag*))
                                          (if (nth 1 (nth contact-tm-cnt *contact-flag*))
                                              (/ (v. lfoot->rfoot lfoot->zmp) (norm2 lfoot->rfoot)) ;; *lfoot* --- r --- zmp -- (1-r) -- *rfoot*
                                            0)
                                        1))
                  (fzl (* (- 1 distribution-ratio) fz)) ;; fzl = (1-r) fz
                  (fzr (* distribution-ratio fz))) ;; fzr = r fz

             ;; 並進滑り摩擦
             (setf (aref v2 (round (+ id (* 1 (+ interval-num 1)))))
                   (- (sqrt (+ (* fx fx) (* fy fy))) (* *mu* fz))) ;; sqrt(fx^2 + fy^2) < mu fz
             ;; 回転滑り摩擦
             (setf (aref v2 (round (+ id (* 2 (+ interval-num 1)))))
                   (- (abs tz) (* *mu* 1e-3 (norm lfoot->zmp) fzl) (* *mu* 1e-3 (norm rfoot->zmp) fzr)))) ;; |tzmp| < mu |rl-rzmp| fzl + mu |rr-rzmp| fzr


           ;; collision check
           ;; (if (<= x-buf (aref v1 (- (length v1) *x-hit-id*)))
           (if (<= x-buf *default-x-hit*)
               (let* ((collision-id 3)
                      (collision-distance nil)
                      body0/1-distance)
                 (dolist (pair-list *collision-pair*)
                   ;; make the list about the combination of the collision distance
                   (dolist (body0 (nth 0 (nth 0 pair-list)))
                     (dolist (body1 (nth 1 (nth 0 pair-list)))
                       (if (and (derivedp body0 capsule) (derivedp body1 capsule))
                           (let ((body0-line (instance line :init :pvertex (send body0 :worldpos) :nvertex (send body0 :transform-vector (float-vector 0 0 (send body0 :get :height)))))
                                 (body1-line (instance line :init :pvertex (send body1 :worldpos) :nvertex (send body1 :transform-vector (float-vector 0 0 (send body1 :get :height))))))

                             (setq body0/1-distance (- (send body0-line :distance body1-line)
                                                       (send body0 :get :radius)
                                                       (send body1 :get :radius))))


                         (progn
                           (if (equal body1 *floor*)
                               (let ((swing-leg (if (nth 0 (nth contact-tm-cnt *contact-flag*))
                                                    (if (nth 1 (nth contact-tm-cnt *contact-flag*))
                                                        nil
                                                      :rleg)

                                                  :lleg)))

                                 (setq body0/1-distance 1e5) ;; 1e5 is equivalent to infinity
                                 (when (or (equal body0 *lfoot-plate*) (equal body0 *rfoot-plate*))
                                   (if swing-leg
                                       (progn
                                         (setq body0/1-distance (- (apply #'min (mapcar #'(lambda (vts) (aref vts 2)) (send (send *robot* :support-polygon swing-leg) :vertices)))
                                                                   (aref (send body1 :worldpos) 2))))
                                                                    ;; この段階では地面より上にあったらプラス

                                     (setq body0/1-distance (- 20 (abs (- (aref (send body0 :worldpos) 2) (aref (send body1 :worldpos) 2)))))))) ;; 本来は等式条件を満たさなければいけないところなので床との差がプラマイ20mm以内にしておく



                             (setq body0/1-distance (car (pqp-collision-distance body0 body1))))))



                       (push body0/1-distance collision-distance)))
                       ;; (when (< body0/1-distance 1e-5)
                       ;;   (format t "body0:~a body1:~a distance:~a~%" (send body0 :get :name) (send body1 :get :name) body0/1-distance)
                       ;;   )


                   (setf (aref v2 (round (+ id (* collision-id (+ interval-num 1)))))
                         ;; (* 1e-3 (- (+ (send (nth 0 pair-list) :get :radius) (send (nth 1 pair-list) :get :radius)) (norm (v- (send (nth 0 pair-list) :worldpos) (send (nth 1 pair-list) :worldpos))))))
                         (- (eval (nth 2 (nth 0 pair-list)))
                            (apply #'min collision-distance))) ;; minimum collision distance


                   (setq collision-distance nil)
                   (incf collision-id)))


             (let* ((collision-id 3)
                    (collision-distance nil)
                    body2/3-distance)
               (dolist (pair-list *collision-pair*)
                 ;; make the list about the combination of the collision distance
                 (dolist (body2 (nth 0 (nth 1 pair-list)))
                   (dolist (body3 (nth 1 (nth 1 pair-list)))
                     (if (and (derivedp body2 capsule) (derivedp body3 capsule))
                         (let ((body2-line (instance line :init :pvertex (send body2 :worldpos) :nvertex (send body2 :transform-vector (float-vector 0 0 (send body2 :get :height)))))
                               (body3-line (instance line :init :pvertex (send body3 :worldpos) :nvertex (send body3 :transform-vector (float-vector 0 0 (send body3 :get :height))))))

                           (setq body2/3-distance (- (send body2-line :distance body3-line)
                                                     (send body2 :get :radius)
                                                     (send body3 :get :radius))))


                       (progn
                         (if (equal body3 *floor*)
                             (let ((swing-leg (if (nth 0 (nth contact-tm-cnt *contact-flag*))
                                                  (if (nth 1 (nth contact-tm-cnt *contact-flag*))
                                                      nil
                                                    :rleg)

                                                :lleg)))

                               (setq body2/3-distance 1e5) ;; 1e5 is equivalent to infinity
                               (when (or (equal body2 *lfoot-plate*) (equal body2 *rfoot-plate*))
                                 (if swing-leg
                                     (progn
                                       (setq body2/3-distance (- (apply #'min (mapcar #'(lambda (vts) (aref vts 2)) (send (send *robot* :support-polygon swing-leg) :vertices)))
                                                                 (aref (send body3 :worldpos) 2))))
                                                                  ;; この段階では地面より上にあったらプラス

                                   (setq body2/3-distance (- 20 (abs (- (aref (send body2 :worldpos) 2) (aref (send body3 :worldpos) 2)))))))) ;; 本来は等式条件を満たさなければいけないところなので床との差がプラマイ20mm以内にしておく



                           (setq body2/3-distance (car (pqp-collision-distance body2 body3))))))



                     (push body2/3-distance collision-distance)))
                     ;; (when (< body2/3-distance 1e-5)
                     ;;   (format t "body2:~a body3:~a distance:~a~%" (send body2 :get :name) (send body3 :get :name) body2/3-distance)
                     ;;   )


                 (setf (aref v2 (round (+ id (* collision-id (+ interval-num 1)))))
                       ;; (* 1e-3 (- (+ (send (nth 2 pair-list) :get :radius) (send (nth 3 pair-list) :get :radius)) (norm (v- (send (nth 2 pair-list) :worldpos) (send (nth 3 pair-list) :worldpos))))))
                       (- (eval (nth 2 (nth 1 pair-list)))
                          (apply #'min collision-distance))) ;; minimum collision distance


                 (setq h-dim-id (round (+ id (* collision-id (+ interval-num 1)))))
                 (setq collision-distance nil)
                 (incf collision-id))))




           (when (> x-buf (nth contact-tm-cnt switching-tm-list))
             (incf contact-tm-cnt))

           ;; トルク制限
           (let ((jid 0)
                 (av (instantiate float-vector (length (send *robot* :angle-vector)))) ;; angle-vector
                 (jvv (instantiate float-vector (length (send *robot* :angle-vector)))) ;; joint-velocity-vector
                 (jav (instantiate float-vector (length (send *robot* :angle-vector)))) ;; joint-acceleration-vector
                 (root-spacial-velocity (instantiate float-vector 3))
                 (root-angular-velocity (instantiate float-vector 3))
                 (root-spacial-acceleration (instantiate float-vector 3))
                 (root-angular-acceleration (instantiate float-vector 3))
                 (tqv (instantiate float-vector (length (send *robot* :angle-vector)))) ;; torque-vector
                 )
             ;; 現在時刻における関節角度，速度，加速度
             (mapc
              #'(lambda (jl bs)
                  (setf (aref av jid) (send bs :calc x-buf (subseq v1 (* jid *id-max*) (* (+ jid 1) *id-max*))))
                  (setf (aref jvv jid) (send bs :calc-delta x-buf (subseq v1 (* jid *id-max*) (* (+ jid 1) *id-max*)) :n 1))
                  (setf (aref jav jid) (send bs :calc-delta x-buf (subseq v1 (* jid *id-max*) (* (+ jid 1) *id-max*)) :n 2))
                  (setq jid (+ jid 1))
                  )
              *jlist* *bspline*)
             ;; root-joint
             ;; TODO ルートリンク角加速度が本当にこれでいいのか検証
             (if (send (car (send *robot* :links)) :joint)
                 (let ((rid 0)
                       (root-link-pos (instantiate float-vector 6))
                       (root-link-vel (instantiate float-vector 6))
                       (root-link-acc (instantiate float-vector 6)))
                   (map float-vector
                     #'(lambda (bs)
                         (setf (aref root-link-pos rid) (send bs :calc x-buf (subseq v1 (* rid *id-max*) (* (+ rid 1) *id-max*))))
                         (setf (aref root-link-vel rid) (send bs :calc-delta x-buf (subseq v1 (* rid *id-max*) (* (+ rid 1) *id-max*)) :n 1))
                         (setf (aref root-link-acc rid) (send bs :calc-delta x-buf (subseq v1 (* rid *id-max*) (* (+ rid 1) *id-max*)) :n 2))
                         (setq rid (+ rid 1)))
                     (subseq *bspline* (length *jlist*) (+ (length *jlist*) 6)))
                   (setq root-spacial-velocity (subseq root-link-vel 0 3))
                   (setq root-angular-velocity (subseq root-link-vel 3 6))
                   (setq root-spacial-acceleration (subseq root-link-acc 0 3))
                   (setq root-angular-acceleration (subseq root-link-acc 3 6)))
                 (progn
                   (send *robot* :fix-leg-to-coords *lfoot* :lleg)))

             (setq tqv (send *robot* :calc-torque-from-vel-acc
                             :root-spacial-velocity root-spacial-velocity
                             :root-angular-velocity root-angular-velocity
                             :root-spacial-acceleration root-spacial-acceleration
                             :root-angular-acceleration root-angular-acceleration
                             :jvv (map float-vector #'(lambda (a) (deg2rad a)) jvv)
                             :jav (map float-vector #'(lambda (a) (deg2rad a)) jav)))
             (let ((rjid 0))
               (mapc
                 #'(lambda (jvl bs)
                     (setq h-dim-id (round (+ id (* (+ 3 (length *collision-pair*) rjid) (+ interval-num 1)))))
                     (setf (aref v2 h-dim-id) (- (send jvl :max-joint-torque) (abs (elt tqv (position jvl *jlist*)))))
                     (setq rjid (+ rjid 1)))
                 *jvel-list* *bspline*)))
           (incf id)))


     (mapcar #'(lambda (x) (/ x interval-num)) (my-range-2 0 interval-num)))
    ;; '(0.0 0.1 0.2 0.3 0.4 0.5 0.6 0.7 0.8 0.9 1.0)) ;; 33(=(length jlist))*11個の要素をもつfloat-vector
    ;;(send *irtviewer* :draw-objects)

    ;; 着地時間などの不等式制約
    (dotimes (i (- (length *contact-flag*) 2))
      (incf h-dim-id)
      (setf (aref v2 h-dim-id) (- (aref #|*p-orig*|# v1 (- (length #|*p-orig*|# v1) i 2)) (aref #|*p-orig*|# v1 (- (length #|*p-orig*|# v1) i 1))))))



  (when *dump-nloptlog*
    (if *use-delta-func*
        (progn
          (when (null (eq *eval-cnt-h* *eval-cnt-h-old*))
            (format t "inequality function: ~a~%~%" v2)
            (with-open-file
             (f (concatenate string *file-name* "_ieq.dat") :direction :output :if-exists :append)
             (dolist (ieq-data (coerce v2 cons))
               (format f "~a   " ieq-data))

             (format f "~%"))

            (setq *eval-cnt-h-old* *eval-cnt-h*)))


      (progn
        (format t "inequality function: ~a~%~%" v2)
        (with-open-file
         (f (concatenate string *file-name* "_ieq.dat") :direction :output :if-exists :append)
         (dolist (ieq-data (coerce v2 cons))
           (format f "~a   " ieq-data))

         (format f "~%")))))




  0)
;; }}}

(defun-c-callable motion-dh ((v1 :string) (v2 :string)) :integer
  (simple-jacobian-transpose v1 v2 (read-from-string "motion-h") :delta *delta*)
  (incf *eval-cnt-h*)
  0)




;; optimization
(defun nlopt-motion-optimize
    (&key
     (file-path "/userdir/logs")
     (title "nlopt-maximize-velocity")
     (alg SLSQP)
     (max-time (* 10 60 60))
     (max-eval 10000)
     (eqthre 1e-8)
     (ftol 1e-10)
     (xtol 1e-10)
     (delta 1e-6)
     (use-all-joint *use-all-joint*)
     (use-margin 0.5)
     (use-append-root-joint *use-append-root-joint*)
     (maxvel-weight 1)
     (minjerk-weight 1e-3)
     (interval-num 52.0)
     ;; (support-polygon-margin (list *sp-front* *sp-back* *sp-inside* *sp-outside*))
     (modify-ec t)
     (robot *robot*)
     (motion-choice *motion-choice*)

     (jlist (if use-all-joint
                (send robot :joint-list) ;; length jlist = 33
              (send robot *limb* :joint-list)))

     (jvel-list (flatten
                 (send robot :torso :joint-list)
                 (send robot *limb* :joint-list)))



     (id-max *id-max*) ;; タスク変数より大きくする（今回は位置姿勢の6か？）
     (recursive-order *recursive-order*)

     (x-min *x-min*)
     (x-max *x-max*)
     (x-hit *default-x-hit*)

     (bspline
      (if use-append-root-joint
          (progn
            (mapcar #'(lambda (k)
                        (instance basic-spline :init
                                  :id-max id-max :recursive-order recursive-order
                                  :x-min x-min :x-max x-max))
                    (flatten jlist (my-range 6))))
        (progn
          (mapcar #'(lambda (k)
                      (instance basic-spline :init
                                :id-max id-max :recursive-order recursive-order
                                :x-min x-min :x-max x-max))
                  jlist))))



     (state-min-vector
      (coerce (flatten
               (mapcar
                #'(lambda (j) (make-list id-max :initial-element (+ (send j :min-angle) (if use-margin use-margin 0))))
                jlist))
              float-vector))


     (state-max-vector
      (coerce (flatten
               (mapcar
                #'(lambda (j) (make-list id-max :initial-element
                                         (if (or (equal j (send robot :rarm-elbow-p)) (equal j (send robot :larm-elbow-p)))
                                             (- 0 (if use-margin use-margin 0)) ;; for jaxon
                                           (- (send j :max-angle) (if use-margin use-margin 0)))))

                jlist))
              float-vector))


     (initial-pose (if use-all-joint
                       (progn
                         (return-end-coords)
                         (funcall *load-pose-func* :pose (nth 0 *key-pose-id*))
                         (send robot :angle-vector))
                         ;; (send-all jlist :joint-angle)
                          ;; 初期姿勢(float-vector)
                     (progn
                       (nth (nth 0 *key-pose-id*) *limb-av-pose-list*))))
                        ;; 初期姿勢(float-vector)



     (hitting-pose (if use-all-joint
                       (progn
                         (return-end-coords)
                         (funcall *load-pose-func* :pose (nth 1 *key-pose-id*))
                         (send robot :angle-vector))
                         ;; (send-all jlist :joint-angle)
                          ;; 打撃姿勢(float-vector)
                     (progn
                       (nth (nth 1 *key-pose-id*) *limb-av-pose-list*))))
                        ;; 打撃姿勢(float-vector)


     (final-pose (if use-all-joint
                     (progn
                       (return-end-coords)
                       (funcall *load-pose-func* :pose (nth 2 *key-pose-id*))
                       (send robot :angle-vector))
                       ;; (send-all jlist :joint-angle)
                        ;; 終端姿勢(float-vector)
                   (progn
                     (nth (nth 2 *key-pose-id*) *limb-av-pose-list*))))
                      ;; 終端姿勢(float-vector)


     ;; q_k(t)=\bm{p}_k^{T} \bm{b}(t)
     ;; B-splineの重みベクトル\bm{p}=[\bm{p}_1 ... \bm{p}_K]（ (*id-max (length bspline))個の要素を持つベクトル）
     (p
      (progn
        (qp-motion-optimize :x-max x-max :x-hit x-hit :x-step 0.1 :maxvel-weight 0 :minjerk-weight 1 :id-max id-max :recursive-order recursive-order :use-all-joint use-all-joint :use-margin use-margin)
        (setq *qp-flag* t)
        (setq *q-orig* (copy-seq *ret*))))) ;; qpのminjerkで解いたときのパラメータ




  ;; initial process
  (return-end-coords)
  ;; ディレクトリを作ってfileの名前を自動生成
  (let (file-directory)
    (setq file-directory
          (concatenate string
                       (format nil
                               "~a/~a_~a_~a_~a_~ah_M-~a_N-~a_x-max-~a_x-hit-~f_maxvel-~a_minjerk-~a_delta-~e_eqthre-~e_ftol-~e_xtol-~e_interval-~a_sp-~a-~a-~a-~a-~a"
                               file-path
                               (subseq (substitute #\- #\: (substitute #\- #\ (unix:asctime (unix:localtime)))) 4 (1- (length (unix:asctime (unix:localtime)))))
                               motion-choice
                               title
                               (cond ((eq alg DIRECT) "DIRECT") ((eq alg G_DIRECT) "G-DIRECT") ((eq alg DIRECT_L) "DIRECT-L") ((eq alg G_DIRECT_L) "G-DIRECT-L") ((eq alg CRS) "CRS") ((eq alg STOGO) "STOGO") ((eq alg ISRES) "ISRES") ((eq alg CCSA) "CCSA") ((eq alg SLSQP) "SLSQP") ((eq alg L_BFGS) "L-BFGS") ((eq alg TN) "TN") ((eq alg SL_VM) "SL-VM") ((eq alg COBYLA) "COBYLA") ((eq alg BOBYQA) "BOBYQA") ((eq alg NEWUOA) "NEWUOA") ((eq alg PRAXIS) "PRAXIS") ((eq alg NelderMeadSimplex) "NelderMeadSimplex")  ((eq alg Sbplx) "Sbplx")  ((eq alg NUM_ALGORITHMS) "NUM-ALGORITHMS"))
                               (/ max-time 60.0 60.0)
                               id-max
                               recursive-order
                               x-max
                               x-hit
                               maxvel-weight
                               minjerk-weight
                               delta
                               eqthre
                               ftol
                               xtol
                               (round interval-num)
                               *sp-front* *sp-back* *sp-inside* *sp-outside* *sp-oneleg*)

                       (if use-append-root-joint "_root-joint" "")
                       (if use-all-joint "" "_limb-joint")
                       (if modify-ec "_modify-ec" "")))


    (unix:mkdir file-directory)
    (setq *file-name* (concatenate string file-directory "/" title)))

  ;; 最初に空のdatファイルを作っておく
  (file-init (concatenate string *file-name* "_obj.dat"))
  (file-init (concatenate string *file-name* "_eq.dat"))
  (file-init (concatenate string *file-name* "_ieq.dat"))
  (file-init (concatenate string *file-name* "_p.dat"))

  (when use-append-root-joint
    (setq initial-pose (concatenate float-vector
                                    initial-pose
                                    (let* ((pose-wc (progn (funcall *load-pose-func* :pose (nth 0 *key-pose-id*)) (send robot :copy-worldcoords)))
                                           (robot-pos (send pose-wc :worldpos))
                                           (robot-rpy (nth 0 (rpy-angle (send pose-wc :worldrot)))))
                                      (concatenate float-vector robot-pos (mapcar #'rad2deg (reverse robot-rpy))))))


    (setq hitting-pose (concatenate float-vector
                                    hitting-pose
                                    (let* ((pose-wc (progn (funcall *load-pose-func* :pose (nth 1 *key-pose-id*)) (send robot :copy-worldcoords)))
                                           (robot-pos (send pose-wc :worldpos))
                                           (robot-rpy (nth 0 (rpy-angle (send pose-wc :worldrot)))))
                                      (concatenate float-vector robot-pos (mapcar #'rad2deg (reverse robot-rpy))))))


    (setq final-pose (concatenate float-vector
                                  final-pose
                                  (let* ((pose-wc (progn (funcall *load-pose-func* :pose (nth 2 *key-pose-id*)) (send robot :copy-worldcoords)))
                                         (robot-pos (send pose-wc :worldpos))
                                         (robot-rpy (nth 0 (rpy-angle (send pose-wc :worldrot)))))
                                    (concatenate float-vector robot-pos (mapcar #'rad2deg (reverse robot-rpy))))))



    (when *qp-flag*
      (setq p (concatenate float-vector
                           p
                           (if *use-6dof-p*
                               (subseq *p-orig* (* (length (send robot :joint-list)) *id-max*) (* (+ (length (send robot :joint-list)) 6) *id-max*))
                             (concatenate float-vector
                                          (make-list id-max :initial-element (aref *concatenate-root-joint* 0)) ;; x [mm]
                                          (make-list id-max :initial-element (aref *concatenate-root-joint* 1)) ;; y [mm]
                                          (make-list id-max :initial-element (aref *concatenate-root-joint* 2)) ;; z [mm]
                                          (make-list id-max :initial-element (aref *concatenate-root-joint* 3)) ;; roll [deg]
                                          (make-list id-max :initial-element (aref *concatenate-root-joint* 4)) ;; pitch [deg]
                                          (make-list id-max :initial-element (aref *concatenate-root-joint* 5))))))) ;; yaw [deg]





    (setq state-min-vector (concatenate float-vector
                                        state-min-vector
                                        (make-list id-max :initial-element (aref *concatenate-root-joint* 6)) ;; x [mm]
                                        (make-list id-max :initial-element (aref *concatenate-root-joint* 7)) ;; y [mm]
                                        (make-list id-max :initial-element (aref *concatenate-root-joint* 8)) ;; z [mm]
                                        (make-list id-max :initial-element (aref *concatenate-root-joint* 9)) ;; roll [deg]
                                        (make-list id-max :initial-element (aref *concatenate-root-joint* 10)) ;; pitch [deg]
                                        (make-list id-max :initial-element (aref *concatenate-root-joint* 11)))) ;; yaw [deg]

    (setq state-max-vector (concatenate float-vector
                                        state-max-vector
                                        (make-list id-max :initial-element (aref *concatenate-root-joint* 12)) ;; x [mm]
                                        (make-list id-max :initial-element (aref *concatenate-root-joint* 13)) ;; y [mm]
                                        (make-list id-max :initial-element (aref *concatenate-root-joint* 14)) ;; z [mm]
                                        (make-list id-max :initial-element (aref *concatenate-root-joint* 15)) ;; roll [deg]
                                        (make-list id-max :initial-element (aref *concatenate-root-joint* 16)) ;; pitch [deg]
                                        (make-list id-max :initial-element (aref *concatenate-root-joint* 17)))) ;; yaw [deg]

    (send robot :move-to (make-coords) :world))


  ;; hitting-time optimization
  (when *qp-flag*
    (setq p (concatenate float-vector p ;; (cond
                                        ;;  ((= (/ (length *contact-flag*) 2) 1) (float-vector x-hit)) ;; hitting-time [s]
                                        ;;  ((= (/ (length *contact-flag*) 2) 2) (float-vector 0.3 (- x-hit 0.5) x-hit)) ;; lift-up-time grounding-time hitting-time [s]
                                        ;;  (t (cdr (mapcar #'(lambda (x) (* (/ x-hit (- (length *contact-flag*) 1)) x)) (my-range (length *contact-flag*))))) ;; TODO x-hit以外のタイミング変数もあればそれを使いたければ初期値を使う
                                        ;;  )
                         *default-switching-list*))

    (setq *qp-flag* nil))

  (setq state-min-vector (concatenate float-vector state-min-vector (make-list (- (length *contact-flag*) 1) :initial-element 0.0)))
  (setq state-max-vector (concatenate float-vector state-max-vector (make-list (- (length *contact-flag*) 1) :initial-element x-max)))

  (setq *eval-cnt-f* 0)
  (setq *eval-cnt-f-old* 0)
  (setq *eval-cnt-g* 0)
  (setq *eval-cnt-g-old* 0)
  (setq *eval-cnt-h* 0)
  (setq *eval-cnt-h-old* 0)

  ;; setq pose is needed for choosing full-joint or rarm(only)
  (setq *initial-pose* initial-pose)
  (setq *hitting-pose* hitting-pose)
  (setq *final-pose* final-pose)
  (setq *id-max* id-max)
  (setq *recursive-order* recursive-order)
  (setq *x-min* x-min)
  (setq *x-max* x-max)
  (setq *jlist* jlist)
  (setq *jvel-list* jvel-list)
  (setq *bspline* bspline)
  (setq *delta* delta)
  (setq *use-delta-func*
        ;; derivative-free algorithms are commented out
        (cond ((or (eq alg DIRECT)
                   (eq alg G_DIRECT)
                   (eq alg DIRECT_L)
                   (eq alg G_DIRECT_L)
                   (eq alg CRS)
                   (eq alg STOGO)
                   (eq alg ISRES)
                   (eq alg CCSA)
                   (eq alg SLSQP)
                   (eq alg L_BFGS)
                   (eq alg TN)
                   (eq alg SL_VM)
                   ;; (eq alg COBYLA)
                   ;; (eq alg BOBYQA)
                   ;; (eq alg NEWUOA)
                   ;; (eq alg PRAXIS)
                   (eq alg NelderMeadSimplex)
                   (eq alg Sbplx)
                   ) t)
              (t nil)))


  (setq *minjerk-weight* minjerk-weight)
  (setq *maxvel-weight* maxvel-weight)
  (setq *interval-num* interval-num)
  (setq *state-min-vector* state-min-vector)
  (setq *state-max-vector* state-max-vector)


  ;; もし初期パラメータがstate-min-vectorかstate-max-vectorの制限を超えていたら範囲に入れる
  (let ((cnt 0))
    (map float-vector
         #'(lambda (pn max min)
             (when (< (- max pn) 0.0) (setf (aref p cnt) (- max 1e-6)))
             (when (< (- pn min) 0.0) (setf (aref p cnt) (+ min 1e-6)))
             (incf cnt))

         p state-max-vector state-min-vector))
  (setq *p-pre* (copy-seq p))

  (when modify-ec
    (modify-end-coords))


  (setq *dump-nloptlog* t)

  (if use-append-root-joint
      (progn
        (with-append-root-joint
         (link-list-with-robot-6dof *robot* (list (cdr (send *robot* :links)))
                                    :joint-class 6dof-joint)
         (print (nlopt-optimize
          :initial-state p
          :state-min-vector state-min-vector
          :state-max-vector state-max-vector
          :state-dimension (length p)
          :evaluation-function (read-from-string "motion-f")
          :evaluation-function-gradient (read-from-string "motion-df")
          :equality-function (read-from-string "motion-g")
          :equality-function-gradient (read-from-string "motion-dg")
          :inequality-function (read-from-string "motion-h")
          :inequality-function-gradient (read-from-string "motion-dh")
          :equality-dimension (+ 6 (- (/ (length *contact-flag*) 2) 1) (if *use-hit-pose* 1 0) (if *use-hit-ee* 1 0) (if *use-final-pose* 1 0))
          :inequality-dimension (round (+ (* (- id-max 1) (length jvel-list)) (* (+ interval-num 1) (+ 3 (length jvel-list) (length *collision-pair*))) (- (length *contact-flag*) 2))) ;; (* (- id-max 1) (length jvel-list) 2)
          :alg alg ;; SLSQP ;; COBYLA ;; DIRECT
          :max-time max-time ;;(* 10 60 60)
          :max-eval max-eval ;; 10000
          :eqthre eqthre
          :ftol ftol ;; 1e-10
          :xtol xtol)) ;; 1e-10

         (setq *p* (copy-seq p))
         (print *p*)
         (show-optimized-motion p :cnt 50 :real-time t)))
          ;; with-append-root-joint

    (progn
      (nlopt-optimize
       :initial-state p
       :state-min-vector state-min-vector
       :state-max-vector state-max-vector
       :state-dimension (length p)
       :evaluation-function (read-from-string "motion-f")
       :evaluation-function-gradient (read-from-string "motion-df")
       :equality-function (read-from-string "motion-g")
       :equality-function-gradient (read-from-string "motion-dg")
       :inequality-function (read-from-string "motion-h")
       :inequality-function-gradient (read-from-string "motion-dh")
       :equality-dimension (+ 5 (- (/ (length *contact-flag*) 2) 1) (if *use-hit-pose* 1 0) (if *use-hit-ee* 1 0) (if *use-final-pose* 1 0))
       :inequality-dimension (round (+ (* (- id-max 1) (length jvel-list)) (* (+ interval-num 1) (+ 3 (length jvel-list) (length *collision-pair*))) (- (length *contact-flag*) 2)))
       :alg alg ;; SLSQP ;; COBYLA ;; DIRECT
       :max-time max-time ;;(* 10 60 60)
       :max-eval max-eval ;; 10000
       :eqthre eqthre
       :ftol ftol ;; 1e-10
       :xtol xtol) ;; 1e-10

      (setq *p* (copy-seq p))
      (print *p*)
      (show-optimized-motion p :cnt 50 :real-time t)))


  (when modify-ec
    (return-end-coords))

  nil)




(defun show-optimized-motion
    (p-ret
     &key
     (robot *robot*)
     (cnt 30)
     (x-max *x-max*)
     (make-loadpattern nil) ;; "/userdir/logs/motion"
     (make-sequence nil)
     (choreonoid nil)
     (specific-time nil)
     (x-step (if (or make-loadpattern specific-time) 0.002 (/ (- x-max *x-min*) (* 1.0 cnt))))
     (real-time t)
     (warning t)
     (show-pose t)
     ;; (modify-zmp nil) ;; (send (midcoords 0.5 *rfoot* *lfoot*) :worldpos)
     (lfoot-ik nil)
     (rfoot-ik nil)
     (zmp-offset #f(0 0 0))
     (fix-leg-coords (list :lleg *lfoot*)))


  (when (not specific-time)
    (setq *bspline-av-list* '()) ;; for load-pattern .pos, angle-vector-sequence-full jpos and showing jvel or jvel-ratio
    (setq *bspline-tm-list* '()) ;; for load-pattern time-stamp, angle-vector-sequence-full tm and showing jvel or jvel-ratio
    (setq *bspline-zmp-rc-list* '()) ;; for load-pattern .zmp
    (setq *bspline-zmp-wc-list* '()) ;; for angle-vector-sequence-full :zmp
    (setq *bspline-cog-wc-list* '()) ;; for angle-vector-sequence-full :zmp
    (setq *bspline-rc-list* '()) ;; for load-pattern .waist and angle-vector-sequence-full :root-coords
    (setq *bspline-wc-list* '())
    (setq *bspline-optional-list* '()) ;; for load-pattern .optionaldata and angle-vector-sequence-full :optional
    (setq *bspline-ec-list* '())
    (setq *bspline-tau-list* '())
    (setq *bspline-sweet-spot-list* '()) ;; not for load-pattern just for calc-plan
    (setq *bspline-hp-list* '()))
    (setq *bspline-jvv-list* '()) ;; for jvv list


  (let* ((x-buf (if specific-time specific-time *x-min*))
         (retl (list (cons :gain p-ret)))  ;; (cons :gain p-ret) ==> (:gain . p-ret)
         p dp ddp id tau
         (show-loop 0)
		 (image-idx 0)
         (bspline
          (if specific-time
              *bspline* ;; 特定の時刻を表示するときは今の*bspline*を用いる（現状はnloptの計算速度を考慮してこうする）
            (progn ;; ひと通り表示するときはx-maxを変えて表示ができるようにここで新たにbsplineのinstanceを生成
              (if (send (car (send robot :links)) :joint) ;; with-append-root-jointしてあればt
                  (progn
                    (mapcar #'(lambda (k)
                                (instance basic-spline :init
                                          :id-max *id-max* :recursive-order *recursive-order*
                                          :x-min *x-min* :x-max x-max))
                            (flatten *jlist* (make-list 6)))) ;; root-jointの6個分付け加える
                (progn
                  (mapcar #'(lambda (k)
                              (instance basic-spline :init
                                        :id-max *id-max* :recursive-order *recursive-order*
                                        :x-min *x-min* :x-max x-max))
                          *jlist*))))))




         (contact-tm-cnt 0)
         (switching-tm-list (list x-max))
         (tm (instance mtimer :init))) ;; 実際の時間で表示するためのタイマー

    (dotimes (i (1- (length *contact-flag*)))
      (push (* (aref p-ret (- (length p-ret) 1 i)) (/ x-max *x-max*)) switching-tm-list))


    (while (<= x-buf x-max)
      (setq id 0)
      (mapc
       #'(lambda (jl bs)
            (send jl :joint-angle
                  (send bs :calc x-buf (subseq p-ret id (setq id (+ id *id-max*))))))
       *jlist* bspline)

      (let ((jid 0)
            (av (instantiate float-vector (length (send *robot* :angle-vector)))) ;; angle-vector
            (jvv (instantiate float-vector (length (send *robot* :angle-vector)))) ;; joint-velocity-vector
            (jav (instantiate float-vector (length (send *robot* :angle-vector)))) ;; joint-acceleration-vector
            (root-spacial-velocity (instantiate float-vector 3))
            (root-angular-velocity (instantiate float-vector 3))
            (root-spacial-acceleration (instantiate float-vector 3))
            (root-angular-acceleration (instantiate float-vector 3))
            (tqv (instantiate float-vector (length (send *robot* :angle-vector)))) ;; torque-vector
            )
        ;; 現在時刻における関節角度，速度，加速度
        (mapc
         #'(lambda (jl bs)
             (setf (aref av jid) (send bs :calc x-buf (subseq p-ret (* jid *id-max*) (* (+ jid 1) *id-max*))))
             (setf (aref jvv jid) (send bs :calc-delta x-buf (subseq p-ret (* jid *id-max*) (* (+ jid 1) *id-max*)) :n 1))
             (setf (aref jav jid) (send bs :calc-delta x-buf (subseq p-ret (* jid *id-max*) (* (+ jid 1) *id-max*)) :n 2))
             (setq jid (+ jid 1))
             )
         *jlist* *bspline*)
        (push (coerce av cons) p)
        (push (coerce jvv cons) dp)
        (push (coerce jvv cons) ddp)
        ;; root-joint
        ;; TODO ルートリンク角加速度が本当にこれでいいのか検証
        (if (send (car (send *robot* :links)) :joint)
            (let ((rid 0)
                  (root-link-pos (instantiate float-vector 6))
                  (root-link-vel (instantiate float-vector 6))
                  (root-link-acc (instantiate float-vector 6)))
              (map float-vector
                #'(lambda (bs)
                    (setf (aref root-link-pos rid) (send bs :calc x-buf (subseq p-ret (* rid *id-max*) (* (+ rid 1) *id-max*))))
                    (setf (aref root-link-vel rid) (send bs :calc-delta x-buf (subseq p-ret (* rid *id-max*) (* (+ rid 1) *id-max*)) :n 1))
                    (setf (aref root-link-acc rid) (send bs :calc-delta x-buf (subseq p-ret (* rid *id-max*) (* (+ rid 1) *id-max*)) :n 2))
                    (setq rid (+ rid 1)))
                (subseq *bspline* (length *jlist*) (+ (length *jlist*) 6)))
              (setq root-spacial-velocity (scale 1e-3 (subseq root-link-vel 0 3)))
              (setq root-angular-velocity (mapcar #'deg2rad (subseq root-link-vel 3 6))))
              (setq root-spacial-acceleration (scale 1e-3 (subseq root-link-acc 0 3)))
              (setq root-angular-acceleration (mapcar #'deg2rad (subseq root-link-acc 3 6)))
            (progn
              (send *robot* :fix-leg-to-coords *lfoot* :lleg)))

        (setq tqv (send *robot* :calc-torque-from-vel-acc
                        :root-spacial-velocity root-spacial-velocity
                        :root-angular-velocity root-angular-velocity
                        :root-spacial-acceleration root-spacial-acceleration
                        :root-angular-acceleration root-angular-acceleration
                        :jvv (map float-vector #'deg2rad jvv)
                        :jav (map float-vector #'deg2rad jav)))
        (push tqv *bspline-tau-list*)
        (push jvv *bspline-jvv-list*)
        (push (coerce tqv cons) tau)
        )

      (let* (each-6dof current-zmp-tf current-zmp-with-offset)
        (if (send (car (send robot :links)) :joint)
            ;(print
              (progn
                (send (send (car (send robot :links)) :joint) :joint-angle
                    (coerce
                     (mapcar
                      #'(lambda (bs)
                          (setq each-6dof (send bs :calc x-buf (subseq p-ret id (+ id *id-max*))))
                          ;; (send bs :calc-delta x-buf (subseq p-ret id (+ id *id-max*)) :n 1)
                          ;; (send bs :calc-delta x-buf (subseq p-ret id (+ id *id-max*)) :n 2)
                          (setq id (+ id *id-max*))
                          each-6dof)
                      (subseq bspline (length *jlist*) (+ (length *jlist*) 6)))
                     float-vector)))
              ;)
          (progn
            (send robot :fix-leg-to-coords (nth 1 fix-leg-coords) (nth 0 fix-leg-coords))))



        ;; 支持多角形の表示
        (calc-specific-time-force-zmp p-ret x-buf :bspline bspline :zmp-at-specific-time *current-zmp* :force-at-specific-time *current-force* :zmp-moment-at-specific-time *current-zmp-moment* :contact-flag (nth contact-tm-cnt *contact-flag*) :lfoot-ik lfoot-ik :rfoot-ik rfoot-ik :step-time x-step)
        (send *support-face* :draw-on :flush t :color #f(0.5 0.5 0) :width 2.5)

        (setq current-zmp-with-offset (v+ *current-zmp* zmp-offset))
        (send *arrow* :locate current-zmp-with-offset :world) ;; zmpを表示

        (when (and (eq (send *support-face* :insidep current-zmp-with-offset) :outside) warning)
          (format t "ZMP is on the outside of support polygon at ~a!!~%" x-buf))

        (when specific-time
          (when show-pose
            (send *irtviewer* :draw-objects)
            (x::window-main-one))

          (return-from nil 1))

        (push current-zmp-with-offset *bspline-zmp-wc-list*)
        (setq current-zmp-tf (send (send (car (send robot :links)) :copy-worldcoords) :inverse-transform-vector current-zmp-with-offset)) ;; root-coords相対の座標変換
        (push current-zmp-tf  *bspline-zmp-rc-list*))


      (when (and lfoot-ik (nth 0 (nth contact-tm-cnt *contact-flag*)))
        (let* ((new-lfoot (copy-object (nth 0 (nth contact-tm-cnt *contact-flag*)))))
          (when (vectorp lfoot-ik)
            (send new-lfoot :translate  (subseq lfoot-ik 0 3) :world)
            (when (= (length lfoot-ik) 4)
              (send new-lfoot :rotate (deg2rad (aref lfoot-ik 3)) :z)))


          (send robot :lleg :inverse-kinematics new-lfoot :rotation-axis t)))


      (when (and rfoot-ik (nth 1 (nth contact-tm-cnt *contact-flag*)))
        (let* ((new-rfoot (copy-object (nth 1 (nth contact-tm-cnt *contact-flag*)))))
          (when (vectorp rfoot-ik)
            (send new-rfoot :translate  (subseq rfoot-ik 0 3) :world)
            (when (= (length rfoot-ik) 4)
              (send new-rfoot :rotate (deg2rad (aref rfoot-ik 3)) :z)))


          (send robot :rleg :inverse-kinematics new-rfoot :rotation-axis t)))



      (push (send robot :centroid) *bspline-cog-wc-list*)

      ;;(send *robot* :fix-leg-to-coords *lfoot* :lleg)
      (push (send robot :angle-vector) *bspline-av-list*)
      (push (send *hit-point* :worldpos) *bspline-sweet-spot-list*)
      (push x-buf *bspline-tm-list*)
      (push (send (car (send robot :links)) :copy-worldcoords) *bspline-rc-list*)
      (push (send robot :copy-worldcoords) *bspline-wc-list*)
      (let ((right-remain-tm (- x-max x-buf))
            (left-remain-tm (- x-max x-buf))
            (max-contact-tm-cnt (- (length *contact-flag*) 1)))

        (dotimes (i (- max-contact-tm-cnt contact-tm-cnt))
          (when (or (and (not (nth 1 (nth contact-tm-cnt *contact-flag*)))
                         (nth 1 (nth (min (+ contact-tm-cnt i) max-contact-tm-cnt) *contact-flag*)))
                    (and (nth 1 (nth contact-tm-cnt *contact-flag*))
                         (not (nth 1 (nth (min (+ contact-tm-cnt i) max-contact-tm-cnt) *contact-flag*)))))
                     ;; xor (exclusive-or)
            (setq right-remain-tm (- (nth (+ contact-tm-cnt i -1) switching-tm-list) x-buf)) ;; のこぎり (iは必ず1以上)
            (return nil)))


        (dotimes (i (- max-contact-tm-cnt contact-tm-cnt))
          (when (or (and (not (nth 0 (nth contact-tm-cnt *contact-flag*)))
                         (nth 0 (nth (min (+ contact-tm-cnt i) max-contact-tm-cnt) *contact-flag*)))
                    (and (nth 0 (nth contact-tm-cnt *contact-flag*))
                         (not (nth 0 (nth (min (+ contact-tm-cnt i) max-contact-tm-cnt) *contact-flag*)))))
                     ;; xor (exclusive-or)
            (setq left-remain-tm (- (nth (+ contact-tm-cnt i -1) switching-tm-list) x-buf)) ;; のこぎり (iは必ず1以上)
            (return nil)))


        (push (float-vector
               (if (nth 1 (nth contact-tm-cnt *contact-flag*)) 1 0)
               (if (nth 0 (nth contact-tm-cnt *contact-flag*)) 1 0)
               0 0
               right-remain-tm
               left-remain-tm
               1 1)

              *bspline-optional-list*))

      (push (send (send robot *limb* :end-coords) :copy-worldcoords) *bspline-ec-list*)
      (push (send *hit-point* :copy-worldcoords) *bspline-hp-list*)
      (send *irtviewer* :draw-objects)
      (send *viewer* :viewsurface :write-to-image-file (format nil "/userdir/logs/~A.jpg" image-idx))
      (incf image-idx)
      (x::window-main-one)

      (if real-time
          (while (< (send tm :stop) (* x-step show-loop)))
         ;; (unix:usleep 1)
         (unix::usleep 100000)
          (print "sleep")
        )

      (setq x-buf (+ x-buf x-step))

      (when (> x-buf (nth contact-tm-cnt switching-tm-list))
        (incf contact-tm-cnt))

      (incf show-loop))
       ;; end while loop


    (when (not specific-time)
      (push (cons :p (reverse p)) retl)
      (push (cons :dp (reverse dp)) retl)
      (push (cons :ddp (reverse ddp)) retl)
      (push (cons :tau (reverse tau)) retl)
      (setq *bspline-av-list* (reverse *bspline-av-list*))
      (setq *bspline-tm-list* (reverse *bspline-tm-list*))
      (setq *bspline-zmp-rc-list* (reverse *bspline-zmp-rc-list*))
      (setq *bspline-zmp-wc-list* (reverse *bspline-zmp-wc-list*))
      (setq *bspline-cog-wc-list* (reverse *bspline-cog-wc-list*))
      (setq *bspline-rc-list* (reverse *bspline-rc-list*))
      (setq *bspline-wc-list* (reverse *bspline-wc-list*))
      (setq *bspline-tau-list* (reverse *bspline-tau-list*))
      (setq *bspline-jvv-list* (reverse *bspline-jvv-list*))
      (progn
        (setf (aref (car *bspline-optional-list*) 4) 1)
        (setf (aref (car *bspline-optional-list*) 5) 1) ;; 最後のoptionaldataの時間だけベタ書きで1にしておく
        (setq *bspline-optional-list* (reverse *bspline-optional-list*)))

      (setq *bspline-ec-list* (reverse *bspline-ec-list*))
      (setq *bspline-hp-list* (reverse *bspline-hp-list*))

      (when make-loadpattern
        (file-init (concatenate string make-loadpattern ".pos"))
        (file-init (concatenate string make-loadpattern ".zmp"))
        (file-init (concatenate string make-loadpattern ".waist"))
        (file-init (concatenate string make-loadpattern ".optionaldata"))

        ;; write load-pattern file
        (let* ((i 0) bspline-pos bspline-zmp tmp-rc bspline-waist bspline-optional)
          (dolist (tm *bspline-tm-list*)
            (setq bspline-pos (map float-vector #'deg2rad (nth i *bspline-av-list*)))
            (setq bspline-zmp (scale 1e-3 (nth i *bspline-zmp-rc-list*)))
            (setq tmp-rc (nth i *bspline-rc-list*))
            (setq bspline-waist (concatenate float-vector
                                             (scale 1e-3 (send tmp-rc :worldpos)) ;; x y z [m]
                                             (nth 0 (rpy-angle (send tmp-rc :worldrot))))) ;; yaw pitch roll [rad]

            (setq bspline-optional (nth i *bspline-optional-list*))
            (with-open-file
             (f (concatenate string make-loadpattern ".pos") :direction :output :if-exists :append)
             (format f "~a" tm)
             (dotimes (j (length bspline-pos))
               (format f " ~a" (aref bspline-pos j)))

             (if choreonoid
                 (format f " -1.39626 1.39626 -1.39626 1.39626~%")
               (format f "~%")))


            (with-open-file
             (f (concatenate string make-loadpattern ".zmp") :direction :output :if-exists :append)
             (format f "~a ~a ~a ~a~%" tm (aref bspline-zmp 0) (aref bspline-zmp 1) (aref bspline-zmp 2))) ;; ２個目のzmpの位置は上で補正している

            (with-open-file
             (f (concatenate string make-loadpattern ".waist") :direction :output :if-exists :append)
             (format f "~a ~a ~a ~a ~a ~a ~a~%" tm (aref bspline-waist 0) (aref bspline-waist 1) (aref bspline-waist 2) (aref bspline-waist 5) (aref bspline-waist 4) (aref bspline-waist 3))) ;; x y z roll pitch yaw

            (with-open-file
             (f (concatenate string make-loadpattern ".optionaldata") :direction :output :if-exists :append)
             (format f "~a ~a ~a ~a ~a ~a ~a ~a ~a~%" tm (aref bspline-optional 0) (aref bspline-optional 1) (aref bspline-optional 2) (aref bspline-optional 3) (aref bspline-optional 4) (aref bspline-optional 5) (aref bspline-optional 6) (aref bspline-optional 7)))

            (incf i))))
             ;; end dolist



      (when make-sequence
        ;; for :angle-vector-sequence-full
        ;; jpos
        (if choreonoid
            (setq *bspline-avsf-jpos-deg*
                  (mapcar
                   #'(lambda (av)
                       (setq av (concatenate float-vector av #f(-80 80 -80 80)))) ;; add head and hand for choreonoid

                   *bspline-av-list*))
          (setq *bspline-avsf-jpos-deg* *bspline-av-list*))
           ;; [deg]
        (setq *bspline-avsf-jpos-rad*
              (mapcar
               #'(lambda (av)
                   (map float-vector #'deg2rad av))

               *bspline-avsf-jpos-deg*)) ;; [rad]

        ;; tm
        (setq *bspline-avsf-tm-ms* (make-list (length *bspline-tm-list*) :initial-element (* 1e3 x-step))) ;; [ms]
        (setq *bspline-avsf-tm-s* (make-list (length *bspline-tm-list*) :initial-element x-step)) ;; [s]

        ;; pos
        (setq *bspline-avsf-pos*
              (mapcar
               #'(lambda (rc)
                   (scale 1e-3 (send rc :worldpos))) ;; x y z [m]

               *bspline-rc-list*))
        ;; rpy
        (setq *bspline-avsf-rpy*
              (mapcar
               #'(lambda (rc)
                   (reverse (nth 0 (rpy-angle (send rc :worldrot))))) ;; roll pitch yaw [rad]

               *bspline-rc-list*))
        ;; root coords
        (setq *bspline-avsf-rc* *bspline-rc-list*)

        ;; robot world coords
        (setq *bspline-avsf-wc* *bspline-wc-list*)

        ;; zmp
        (setq *bspline-avsf-zmp-rc*
              (mapcar
               #'(lambda (zmp)
                   (scale 1e-3 zmp)) ;; x y z [m]

               *bspline-zmp-rc-list*)) ;; [m] relative to root coords
        (setq *bspline-avsf-zmp-wc* *bspline-zmp-wc-list*) ;; [mm] relative to world coords

        ;; optional
        (setq *bspline-avsf-optional* *bspline-optional-list*)

        ;; centroid position
        (setq *bspline-avsf-cog-wc* *bspline-cog-wc-list*)))



    nil))

(defun show-optimized-motion-util ()
    (progn
      (send *robot* :move-to (make-coords) :world)
      (with-append-root-joint
       (link-list-with-robot-6dof *robot* (list (cdr (send *robot* :links)))
                                  :joint-class 6dof-joint)
       ;; (show-optimized-motion *p* :cnt 20 :x-max *x-max-of-p-orig* :real-time t :lfoot-ik t :rfoot-ik t)
       (show-optimized-motion *p* :cnt 20 :x-max *x-max-of-p-orig* :real-time nil :lfoot-ik t :rfoot-ik t)
       ;;(show-optimized-motion *p* :x-max *x-max-of-p-orig* :real-time nil :make-loadpattern "/home/terasawa/bspline_1.5_M-14_N-5" :zmp-offset #f(0 0 0) :choreonoid nil)
       ;;(show-optimized-motion *p* :specific-time *x-max-of-p-orig*)
       ;;(show-optimized-motion *p* :x-step 0.01 :x-max *x-max-of-p-orig* :real-time nil :make-sequence t :zmp-offset #f(0 0 0) :choreonoid t)
       ))
  )



;; 特定の時刻におけるzmpとforce, momentを計算
(defun calc-specific-time-force-zmp
    (p-ret specific-time
     &key
     (robot *robot*)
     (bspline *bspline*)
     (zmp-at-specific-time *current-zmp*)
     (force-at-specific-time *current-force*)
     (zmp-moment-at-specific-time *current-zmp-moment*)
     (contact-flag (list *lfoot* *rfoot* nil nil))
     (step-time 0.002)
     (lfoot-ik t)
     (rfoot-ik t)
     (fix-leg-coords (list :lleg *lfoot*)))


  (let* (id
         ;; calc-zmpではロボットの関節角度列とルートリンクの姿勢列のみを与え，逆動力学を解くことができる
         ;; しかし，このままでは加速度項がうまく出ないため，現在の周期だけでなくその前も含めた3回calc-zmpを解くことで加速度項まで計算してもらう
         (specific-time-2 (if (>= specific-time (* step-time 2))
                              (- specific-time (* step-time 2))
                            0.0))

         (specific-time-1 (if (>= specific-time (* step-time 2))
                              (- specific-time step-time)
                            (/ specific-time 2.0)))
         (dt (if (or (>= specific-time (* step-time 2)) (= specific-time 0.0)) step-time specific-time-1)))


    ;; specific-time-2
    (setq id 0)
    ;; joint-angle
    (mapc
     #'(lambda (jl bs)
         (list
          (send jl :joint-angle
                (send bs :calc specific-time-2 (subseq p-ret id (setq id  (+ id *id-max*)))))))

     *jlist* bspline)
    ;; root-joint
    (if (send (car (send robot :links)) :joint)
        (progn
          (send (send (car (send robot :links)) :joint) :joint-angle
                (coerce
                 (mapcar
                  #'(lambda (bs)
                      (send bs :calc specific-time-2 (subseq p-ret id (setq id (+ id *id-max*)))))

                  (subseq bspline (length *jlist*) (+ (length *jlist*) 6)))
                 float-vector))

          (when (and lfoot-ik (nth 0 contact-flag))
            (let* ((new-lfoot (copy-object (nth 0 contact-flag))))
              (when (vectorp lfoot-ik)
                (send new-lfoot :translate  (subseq lfoot-ik 0 3) :world)
                (when (= (length lfoot-ik) 4)
                  (send new-lfoot :rotate (deg2rad (aref lfoot-ik 3)) :z)))


              (send robot :lleg :inverse-kinematics new-lfoot :rotation-axis t :revert-if-fail nil)))


          (when (and rfoot-ik (nth 1 contact-flag))
            (let* ((new-rfoot (copy-object (nth 1 contact-flag))))
              (when (vectorp rfoot-ik)
                (send new-rfoot :translate  (subseq rfoot-ik 0 3) :world)
                (when (= (length rfoot-ik) 4)
                  (send new-rfoot :rotate (deg2rad (aref rfoot-ik 3)) :z)))


              (send robot :rleg :inverse-kinematics new-rfoot :rotation-axis t :revert-if-fail nil))))



      (progn
        (send robot :fix-leg-to-coords (nth 1 fix-leg-coords) (nth 0 fix-leg-coords))))


    ;; calc-zmp
    (send robot :calc-zmp (send robot :angle-vector) (send (car (send robot :links)) :copy-worldcoords) :dt dt)

    ;; specific-time-1
    (setq id 0)
    ;; joint-angle
    (mapc
     #'(lambda (jl bs)
         (list
          (send jl :joint-angle
                (send bs :calc specific-time-1 (subseq p-ret id (setq id (+ id *id-max*)))))))

     *jlist* bspline)
    ;; root-joint
    (if (send (car (send robot :links)) :joint)
        (progn
          (send (send (car (send robot :links)) :joint) :joint-angle
                (coerce
                 (mapcar
                  #'(lambda (bs)
                      (send bs :calc specific-time-1 (subseq p-ret id (setq id (+ id *id-max*)))))

                  (subseq bspline (length *jlist*) (+ (length *jlist*) 6)))
                 float-vector))

          (when (and lfoot-ik (nth 0 contact-flag))
            (let* ((new-lfoot (copy-object (nth 0 contact-flag))))
              (when (vectorp lfoot-ik)
                (send new-lfoot :translate  (subseq lfoot-ik 0 3) :world)
                (when (= (length lfoot-ik) 4)
                  (send new-lfoot :rotate (deg2rad (aref lfoot-ik 3)) :z)))


              (send robot :lleg :inverse-kinematics new-lfoot :rotation-axis t :revert-if-fail nil)))


          (when (and rfoot-ik (nth 1 contact-flag))
            (let* ((new-rfoot (copy-object (nth 1 contact-flag))))
              (when (vectorp rfoot-ik)
                (send new-rfoot :translate  (subseq rfoot-ik 0 3) :world)
                (when (= (length rfoot-ik) 4)
                  (send new-rfoot :rotate (deg2rad (aref rfoot-ik 3)) :z)))


              (send robot :rleg :inverse-kinematics new-rfoot :rotation-axis t :revert-if-fail nil))))



      (progn
        (send robot :fix-leg-to-coords (nth 1 fix-leg-coords) (nth 0 fix-leg-coords))))


    ;; calc-zmp
    (send robot :calc-zmp (send robot :angle-vector) (send (car (send robot :links)) :copy-worldcoords) :dt dt)

    ;; specific-time
    (setq id 0)
    ;; joint-angle
    (mapc
     #'(lambda (jl bs)
         (list
          (send jl :joint-angle
                (send bs :calc specific-time (subseq p-ret id (setq id (+ id *id-max*)))))))

     *jlist* bspline)
    ;; root-joint
    (if (send (car (send robot :links)) :joint)
        (progn
          (send (send (car (send robot :links)) :joint) :joint-angle
                (coerce
                 (mapcar
                  #'(lambda (bs)
                      (send bs :calc specific-time (subseq p-ret id (setq id (+ id *id-max*)))))

                  (subseq bspline (length *jlist*) (+ (length *jlist*) 6)))
                 float-vector))

          (when (and lfoot-ik (nth 0 contact-flag))
            (let* ((new-lfoot (copy-object (nth 0 contact-flag))))
              (when (vectorp lfoot-ik)
                (send new-lfoot :translate  (subseq lfoot-ik 0 3) :world)
                (when (= (length lfoot-ik) 4)
                  (send new-lfoot :rotate (deg2rad (aref lfoot-ik 3)) :z)))


              (send robot :lleg :inverse-kinematics new-lfoot :rotation-axis t :revert-if-fail nil)))


          (when (and rfoot-ik (nth 1 contact-flag))
            (let* ((new-rfoot (copy-object (nth 1 contact-flag))))
              (when (vectorp rfoot-ik)
                (send new-rfoot :translate  (subseq rfoot-ik 0 3) :world)
                (when (= (length rfoot-ik) 4)
                  (send new-rfoot :rotate (deg2rad (aref rfoot-ik 3)) :z)))


              (send robot :rleg :inverse-kinematics new-rfoot :rotation-axis t :revert-if-fail nil))))



      (progn
        (send robot :fix-leg-to-coords (nth 1 fix-leg-coords) (nth 0 fix-leg-coords))))



    ;; 支持多角形の表示
    (make-support-face :front *sp-front* :back *sp-back* :inside *sp-inside* :outside *sp-outside* :oneleg *sp-oneleg* :contact-flag contact-flag)

    ;; zmpを計算
    (replace zmp-at-specific-time (send robot :calc-zmp (send robot :angle-vector) (send (car (send robot :links)) :copy-worldcoords) :dt dt))
    ;; force, momentも計算(:calc-zmp内部でinverse-dynamicsを呼んでいるのですでに計算されている)
    (replace force-at-specific-time (send (car (send robot :links)) :force))
    (replace zmp-moment-at-specific-time (send robot :get :zmp-moment)))


  nil)




;; 支持多角形*support-face*の計算
(defun make-support-face (&key (robot *robot*) (front 20) (back 20) (inside 20) (outside 20) (oneleg 20) (contact-flag (list *lfoot* *rfoot* nil nil)))
  (let* ((rleg-vall (send (send robot :support-polygon :rleg) :vertices))
         (lleg-vall (send (send robot :support-polygon :lleg) :vertices))
         (rleg-v0 (nth 0 rleg-vall))
         (rleg-v1 (nth 1 rleg-vall))      ;;    足の順番            x
         (rleg-v2 (nth 2 rleg-vall))      ;;  lleg    rleg          ^
         (rleg-v3 (nth 3 rleg-vall))      ;; 0___3   3___0          |
         (lleg-v0 (nth 3 lleg-vall))      ;; |   |   |   |          |
         (lleg-v1 (nth 2 lleg-vall))      ;; |   |   |   |    y<-----
         (lleg-v2 (nth 1 lleg-vall))      ;; 1---2   2---1
         (lleg-v3 (nth 0 lleg-vall))
         (rleg-x (v- (send (send robot :rleg :end-coords :copy-worldcoords) :transform-vector #f(1 0 0)) (send (send robot :rleg :end-coords) :worldpos)))
         (rleg-y (v- (send (send robot :rleg :end-coords :copy-worldcoords) :transform-vector #f(0 1 0)) (send (send robot :rleg :end-coords) :worldpos)))
         (lleg-x (v- (send (send robot :lleg :end-coords :copy-worldcoords) :transform-vector #f(1 0 0)) (send (send robot :lleg :end-coords) :worldpos)))
         (lleg-y (v- (send (send robot :lleg :end-coords :copy-worldcoords) :transform-vector #f(0 1 0)) (send (send robot :lleg :end-coords) :worldpos))))

    (when (not (and (nth 0 contact-flag) (nth 1 contact-flag)))
      (setq front oneleg
            back oneleg
            inside oneleg
            outside oneleg)) ;; 全てonelegのマージンに書き換える

    (setq rleg-v0 (reduce #'v+ (list rleg-v0 (scale outside rleg-y) (scale (- front) rleg-x))))
    (setq rleg-v1 (reduce #'v+ (list rleg-v1 (scale outside rleg-y) (scale back rleg-x))))
    (setq rleg-v2 (reduce #'v+ (list rleg-v2 (scale (- inside) rleg-y) (scale back rleg-x))))
    (setq rleg-v3 (reduce #'v+ (list rleg-v3 (scale (- inside) rleg-y) (scale (- front) rleg-x))))
    (setq lleg-v0 (reduce #'v+ (list lleg-v0 (scale (- outside) lleg-y) (scale (- front) lleg-x))))
    (setq lleg-v1 (reduce #'v+ (list lleg-v1 (scale (- outside) lleg-y) (scale back lleg-x))))
    (setq lleg-v2 (reduce #'v+ (list lleg-v2 (scale inside lleg-y) (scale back lleg-x))))
    (setq lleg-v3 (reduce #'v+ (list lleg-v3 (scale inside lleg-y) (scale (- front) lleg-x))))

    (setq *support-face*
          (instance face :init
                    :vertices
                    (quickhull
                     (mapcar
                      #'(lambda (v) (float-vector (aref v 0) (aref v 1) 0))
                      (append
                       (if (nth 0 contact-flag) (list lleg-v0 lleg-v1 lleg-v2 lleg-v3) nil)
                       (if (nth 1 contact-flag) (list rleg-v0 rleg-v1 rleg-v2 rleg-v3) nil))))))))





;; ファイルの初期化
(defun file-init (file-name)
  (with-open-file
   (f file-name :direction :output :if-exists :new-version)))



;(when (not (boundp '*bspline*))
;  (if (not (boundp '*id-max*))
;      (nlopt-init)
;    (nlopt-init :id-max *id-max*)))




#|
(nlopt-motion-optimize :x-max 2.6 :x-hit 1.7 :cnt 50 :id-max 14 :recursive-order 5 :max-eval 100000000 :max-time (* 24 60 60) :alg SLSQP :delta 1e-6 :eqthre 1e-2 :xtol 1e-10 :ftol 1e-10 :file-name "/userdir/logs/2016*_nlopt_SLSQP_*_M-*_N-*_delta-*_eqthre-1e-*_xtol-1e-*_ftol-1e-*_1day" :use-all-joint t :use-margin 0.5 :use-append-root-joint t)

(show-jvel :limb *limb*)

(progn
  (send *robot* :move-to (make-coords) :world)
  (with-append-root-joint
   (link-list-with-robot-6dof *robot* (list (cdr (send *robot* :links)))
                              :joint-class 6dof-joint)))
   ;;(show-optimized-motion *p* :cnt 20 :x-max *x-max-of-p-orig* :real-time t :lfoot-ik nil :rfoot-ik nil)
   ;;(show-optimized-motion *p* :x-max *x-max-of-p-orig* :real-time nil :make-loadpattern "/home/future731/bspline_1.5_M-14_N-5" :zmp-offset #f(0 0 0) :choreonoid nil)
   ;;(show-optimized-motion *p* :specific-time *x-max-of-p-orig*)
   ;;(show-optimized-motion *p* :x-step 0.01 :x-max *x-max-of-p-orig* :real-time nil :make-sequence t :zmp-offset #f(0 0 0) :choreonoid t)

|#
