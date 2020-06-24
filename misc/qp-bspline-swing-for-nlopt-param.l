(require "package://tennis/euslisp/bspline.l")
(require "package://tennis/euslisp/jaxon-tennis-motion.l")

(require "package://eus_qpoases/euslisp/eus-qpoases.l")

(setq *rarm-av-pose1* #f(-3.20674 -51.6272 -49.1086 1.72789 -119.195 -30.3532 -31.7841 3.82424))
(setq *rarm-av-pose2* #f(14.5493 -4.96018 -46.1575 -79.8115 -109.248 -71.5355 -84.855 16.5296))
(setq *rarm-av-pose3* #f(47.8365 2.28994 -34.7166 -28.2919 -105.731 -104.431 -55.4491 -8.10341))
(setq *rarm-av-pose4* #f(56.2899 -56.8759 -53.1873 33.4169 -62.5403 -75.4065 13.6978 25.7323))

(defun qp-init (&key (robot *robot*))
  (load-tennis-pose :pose 3)
  (send robot :reset-pose)
  (send robot :move-to (make-coords) :world)
  (setq *rarm-end-coords-default* (send (send robot :rarm :end-coords) :copy-worldcoords))
  (send robot :rarm :angle-vector *rarm-av-pose3*)
  (setq *hit-point* (send *sweet-spot* :copy-worldcoords))
  (send robot :reset-pose)
  (send *irtviewer* :draw-objects)
  (format t ";;(qp-swing-optimize :x-max 2.6 :x-hit 1.7 :maxvel-weight 1e3 :minjerk-weight 1e0 :x-step 0.02)~%")
  )

(defun modify-end-coords (&key (robot *robot*))
  (send robot :reset-pose)
  (send robot :move-to (make-coords) :world)
  (send (send robot :rarm :end-coords) :dissoc *racket-coords*)
  (send (send robot :rarm :end-coords) :move-to (send *sweet-spot* :copy-worldcoords) :world)
  (send (send robot :rarm :end-coords :parent) :assoc *racket-coords*)
  )

(defun return-end-coords (&key (robot *robot*))
  (send robot :reset-pose)
  (send robot :move-to (make-coords) :world)
  (send (send robot :rarm :end-coords :parent) :dissoc *racket-coords*)
  (send (send robot :rarm :end-coords) :move-to (send *rarm-end-coords-default* :copy-worldcoords) :world)
  (send (send robot :rarm :end-coords) :assoc *racket-coords*)
  )

;; (when (not (boundp '*jaxon_red*))
;;   (load "package://hrpsys_ros_bridge_tutorials/euslisp/jaxon-interface.l")
;;   (jaxon-init)
;;   (setq *robot* *jaxon_red*)
;;   (objects (list *robot*))
;;   )


;; (defun plot-bspline
;;   (&rest
;;    args
;;    &key
;;    ((:id-max M) 10) ;; タスク空間のパラメータより多くする
;;    ((:recursive-order N) 4) ;; 変化させると山の裾の広がり具合が変わる、多いほどなめらかになる
;;    ((:recursive-cnt _n) N) ;; 実装上の都合（recursive-orderは変化させないでおく）
;;    (x-min 0.0)
;;    (x-max 1.0)
;;    (x x-min)
;;    (plot-cnt 100)
;;    (bspline
;;     (instance basic-spline :init
;;               :recursive-order N
;;               :recursive-cnt _n
;;               :id-max M
;;               :x-min x-min :x-max x-max))
;;    (pos-func :calc-coeff-vector) ;; :calc-coeff-vector (x) はbsplineのxにおける出力を表す
;;    (graph
;;     (progn
;;       (require "../../util/graph-sample.lisp")
;;       (setq *bspline-coeff-matrix-hash* (make-hash-table))
;;       (create-graph
;;        (send bspline :to-string)
;;        :size '(640 320)
;;        :range
;;        (list (float-vector x-min 0)
;;              (float-vector x-max 1.))
;;        :name-list
;;        (mapcar
;;         #'(lambda (id) (format nil "id=~A" id))
;;         (send-all (send bspline :bspline-element-list) :id))
;;        :data-list
;;        (let* ((tm (instance mtimer :init)) ;; mtimterは秒の計測
;;               (cnt x-min)
;;               (id -1)
;;               dlist xlist)
;;          (dotimes (i plot-cnt) ;; グラフ描画時に100点に分割してプロット
;;            (push (+ x-min (/ (* i 1.0 (- x-max x-min))
;;                              plot-cnt)) xlist)
;;            (push (send bspline pos-func (car xlist)) ;; send bspline :calc-coeff-vector (car xlist)
;;                  dlist))
;;          (format t "TIME: ~A~%" (send tm :stop))
;;          (reverse
;;           (mapcar
;;            #'(lambda (hh)
;;                (incf id) ;; -1スタートで0
;;                (map cons ;; mapcarの上位互換
;;                     #'(lambda (dl x)
;;                         (float-vector x (aref dl id)))
;;                     dlist xlist))
;;            (make-list M))
;;           ))
;;        )))
;;    &allow-other-keys)
;;   (if graph
;;       (send graph :fit-draw)) ;; :fit-drawはgraph-panel.lispにある
;;   graph)

(defun qp-swing-optimize
  (&key
   (robot
    (cond
     ((and (boundp '*robot*) *robot*) *robot*) ;; *robot*があってnilではなかったら*robot*を使う
     (t ;;
      (require "package://euslisp/jskeus/irteus/demo/sample-robot-model.l") ;; そうでなかったらsample-robotを使う
      (setq *robot* (instance sample-robot :init))
      ;;(require "package://euslisp/jskeus/irteus/demo/sample-robot-model.lisp")
      ;;(setq *robot* (instance sample-robot :init))
      (objects (list *robot*))
      *robot*)))

   (init (progn
           (require "package://eus_qp/euslisp/eiquadprog.lisp")
           (send robot :reset-pose)
           (load-tennis-pose :pose 1)
           )) ;; ここでstart-avの姿勢を作る

   (jlist (send robot :joint-list))
   ;;(send robot :joint-list))
   ;;(flatten (send-all (send robot :rarm :links) :joint)))

   (start-av (send-all jlist :joint-angle)) ;; start-av = (coerce *rarm-av-pose1* cons)

   (hit-av (progn (load-tennis-pose :pose 3) (coerce (send robot :angle-vector) cons)))
   (hit-jacobian (progn
                   (send robot :rarm :angle-vector (coerce hit-av float-vector))
                   (send robot :calc-jacobian-from-link-list
                         (send robot :rarm :links)
                         :move-target (send robot :rarm :end-coords)
                         :transform-coords (make-coords)
                         :translation-axis '(t)
                         :rotation-axis '(nil)) ;; 3(or6)行jlist(=8)列
                   ))

   (end-av
    (progn (load-tennis-pose :pose 4) (coerce (send robot :angle-vector) cons))
    ;; (mapcar #'(lambda (j)
    ;;             (+ (send j :min-angle)
    ;;                (random (- (send j :max-angle) (send j :min-angle)))))
    ;;         jlist) ;; 目標関節角度列をランダムに生成
    )

   (id-max 9) ;; タスク変数（位置姿勢6）より大きくする

   (recursive-order 4)

   (x-min 0.0)
   (x-max 2.6)
   (x-hit (* x-max 0.65))

   (bspline
    (mapcar #'(lambda (k)
                (instance basic-spline :init
                          :id-max id-max :recursive-order recursive-order
                          :x-min x-min :x-max x-max))
            jlist)) ;; 各関節（*jaxon_red*なら8個）にそれぞれid-max(=9)個のbsplineを作る、bsplineには(#<basic-spline #X8f25708> #<basic-spline #Xe85b650> #<basic-spline #Xfbd6448> #<basic-spline #X11555750> #<basic-spline #X12afff10> #<basic-spline #X218ad928> #<basic-spline #X250322c8> #<basic-spline #Xf8f82d8>)のように関節数分のbsplineのインスタンスのリストが入る

   (initial-state
    (instantiate float-vector (* id-max (length bspline))))
   ;; (length bspline) = (length jlist)
   ;; *jaxon_red*の場合(length initial-state)=9*8=72

   (state-min-vector
    (apply #'concatenate
           (cons float-vector
                 (mapcar
                  #'(lambda (j) (make-list id-max :initial-element (send j :min-angle)))
                  jlist))))
   ;; (#<vectorclass #X1ff49c8 float-vector> (-17.5812 -17.5812 -17.5812 -17.5812 -17.5812 -17.5812 -17.5812 -17.5812 -17.5812) (-180.0 -180.0 -180.0 -180.0 -180.0 -180.0 -180.0 -180.0 -180.0) (-180.0 -180.0 -180.0 -180.0 -180.0 -180.0 -180.0 -180.0 -180.0) (-180.0 -180.0 -180.0 -180.0 -180.0 -180.0 -180.0 -180.0 -180.0) (-125.481 -125.481 -125.481 -125.481 -125.481 -125.481 -125.481 -125.481 -125.481) (-180.0 -180.0 -180.0 -180.0 -180.0 -180.0 -180.0 -180.0 -180.0) (-90.0002 -90.0002 -90.0002 -90.0002 -90.0002 -90.0002 -90.0002 -90.0002 -90.0002) (-81.0 -81.0 -81.0 -81.0 -81.0 -81.0 -81.0 -81.0 -81.0))
   ;;       ↓
   ;; #f(-17.5812 -17.5812 -17.5812 -17.5812 -17.5812 -17.5812 -17.5812 -17.5812 -17.5812 -180.0 -180.0 -180.0 -180.0 -180.0 -180.0 -180.0 -180.0 -180.0 -180.0 -180.0 -180.0 -180.0 -180.0 -180.0 -180.0 -180.0 -180.0 -180.0 -180.0 -180.0 -180.0 -180.0 -180.0 -180.0 -180.0 -180.0 -125.481 -125.481 -125.481 -125.481 -125.481 -125.481 -125.481 -125.481 -125.481 -180.0 -180.0 -180.0 -180.0 -180.0 -180.0 -180.0 -180.0 -180.0 -90.0002 -90.0002 -90.0002 -90.0002 -90.0002 -90.0002 -90.0002 -90.0002 -90.0002 -81.0 -81.0 -81.0 -81.0 -81.0 -81.0 -81.0 -81.0 -81.0)

   (state-max-vector
    (apply #'concatenate
           (cons float-vector
                 (mapcar
                  #'(lambda (j) (make-list id-max :initial-element (send j :max-angle)))
                  jlist))))

   (equality-matrix-for-start/end-pos
    (matrix-append ;; s-nodaさんが作った関数で、'(1 0) で行（下）方向に、'(0 1)で列（横）方向に、'(1 1)で斜め方向につなげる ex)(matrix-append (list (matrix (list 1 2 3) (list 4 5 6)) (matrix (list 7 8) (list 9 10))) '(1 1)) = #2f((1.0 2.0 3.0 0.0 0.0) (4.0 5.0 6.0 0.0 0.0) (0.0 0.0 0.0 7.0 8.0) (0.0 0.0 0.0 9.0 10.0))
     (map cons
          #'(lambda (bs st ed)
              (send bs :calc-gain-vector-coeff-matrix-from-via-x-list (list 0.0 x-max x-hit)))
          ;; ex) (setq via-x-list (list 0.5)) (send (nth 0 bspline) :calc-gain-vector-coeff-matrix-from-via-x-list via-x-list) = #2f((0.0 0.0 0.002604 0.197917 0.598958 0.197917 0.002604 0.0 0.0))
          ;; (length via-x-list)行id-max(=9)列
          bspline start-av end-av)
     '(1 1))) ;; 24(start-av+end-av+hit-av)*72(jlist*id-max)行列

   (equality-coeff-for-start/end-pos
    (concatenate float-vector
                 (flatten (map cons #'list start-av end-av hit-av)))) ;; #f(start-av end-av hit-av)

   (equality-matrix-for-start/end-vel ;; 初期・終端条件（速度ゼロ）
    (matrix-append
     (map cons
          #'(lambda (bs st ed)
              (make-matrix
               2 id-max ;;2 行id-max列
               (mapcar
                #'(lambda (x) (send bs :calc-delta-coeff-vector x :n 1))
                (list 0.0 x-max))))
          bspline start-av end-av)
     '(1 1)))

   (equality-coeff-for-start/end-vel
    (scale 0 (concatenate float-vector
                          (flatten (map cons #'list start-av end-av)))))

   (equality-matrix-for-start/end-acc ;; 初期・終端条件（加速度ゼロ）
    (matrix-append
     (map cons
          #'(lambda (bs st ed)
              (make-matrix
               2 id-max
               (mapcar
                #'(lambda (x) (send bs :calc-delta-coeff-vector x :n 2))
                (list 0.0 x-max))))
          bspline start-av end-av)
     '(1 1)))

   (equality-coeff-for-start/end-acc
    (scale 0 (concatenate float-vector
                          (flatten (map cons #'list start-av end-av)))))

   (minjerk-weight 1e-6)
   (eval-weight-matrix ;; 72*72
    ;; minimize integreted jerk squared
    (let* ((mat
            (matrix-append
             (mapcar
              #'(lambda (rate)
                  (matrix-append
                   (mapcar
                    #'(lambda (bs)
                        (make-matrix ;; 1行id-max(=9)列の行列を作る
                         1 id-max
                         (list
                          (send bs :calc-delta-coeff-vector ;; :calc-delta-coeff-vector (x &key (n 1)) bsplineのn階微分したときのxにおける値
                                (+ x-min (* rate (- x-max x-min))) ;; x-minからx-maxの間で離散化
                                :n 3)
                          )))
                    bspline) ;; 1行id-max(=9)列の行列の関節数(=8)個のlist
                   '(1 1)))
              '(0 0.1 0.2 0.3 0.4 0.5 0.6 0.7 0.8 0.9 1.0)) ;; 8行72列の行列の離散化した分(=11)のリスト
             '(1 0)))) ;; 88行72列の行列（上のリストを縦につなげる）
      (m* (transpose mat) mat)) ;; M^T*M jerkの二乗
    )

   (maxvel-weight 1)
   (eigen-weight 1)
   (vel-dir (normalize-vector #f(0.546619 0.80954 0.21413)))
   (eval-weight-matrix2 (progn
    ;; maxmize joint velocity at a time point x-hit
    (let* ((mat
            (matrix-append
             (mapcar
              #'(lambda (bs)
                  (make-matrix ;; 1行id-max(=9)列の行列を作る
                   1 id-max
                   (list
                    (send bs :calc-delta-coeff-vector x-hit :n 1) ;; x-hitにおけるbsplineの1階微分（＝速度）の値
                    )))
              bspline) ;; 1行id-max(=9)列の行列の関節数(=8)個のlist
             '(1 1))))
      (scale-matrix -1 (m* (transpose mat) mat))) ;; 速度を二乗後、maximizeのため符号反転

    ;; ;; maximize end-effector velocity at a time point x-hit
    ;; (let* ((mat
    ;;         (matrix-append
    ;;          (mapcar
    ;;           #'(lambda (bs)
    ;;               (make-matrix ;; 1行id-max(=9)列の行列を作る
    ;;                1 id-max
    ;;                (list
    ;;                 (send bs :calc-delta-coeff-vector x-hit :n 1) ;; x-hitにおけるbsplineの1階微分（＝速度）の値
    ;;                 )))
    ;;           bspline) ;; 1行id-max(=9)列の行列の関節数(=8)個のlist
    ;;          '(1 1)))
    ;;        (Jb^T (m* hit-jacobian mat))
    ;;        )
    ;;   (scale-matrix -1 (m* (transpose Jb^T) Jb^T))) ;; 速度を二乗後、maximizeのため符号反転
    ))

   (cnt 30)
   (x-step ;; x-stepごとに結果を出力
    ;;0.002
    (/ (- x-max x-min) (* 1.0 cnt))
    )

   (x-buf x-min)
   (ret (solve-eiquadprog
         ;; solve-qpoases-qp
         ;; :debug? t
         :initial-state initial-state
         :eval-weight-matrix (setq *mat* (m+ (scale-matrix minjerk-weight eval-weight-matrix)
                                             (scale-matrix eigen-weight
                                                           (unit-matrix (send eval-weight-matrix :get-val 'dim0)))
                                             (scale-matrix maxvel-weight eval-weight-matrix2)))
         :state-min-vector state-min-vector
         :state-max-vector state-max-vector
         :equality-vector
         (concatenate float-vector
                      equality-coeff-for-start/end-pos
                      equality-coeff-for-start/end-vel
                      equality-coeff-for-start/end-acc)
         :equality-matrix
         (matrix-append
          (list equality-matrix-for-start/end-pos
                equality-matrix-for-start/end-vel
                equality-matrix-for-start/end-acc)
          '(1 0))
         ))
   )
  (setq *E1* eval-weight-matrix)
  (setq *E2* eval-weight-matrix2)
  (setq *ret* ret)
  (setq *bspline* bspline)

  (setq *bspline-av-list* '())
  (setq *bspline-tm-list* '())
  (setq *bspline-endcoords-list* '())

  (if (null ret) (setq ret initial-state))
  (format t "   --- ~A x ~A = ~A variables~%" id-max (length start-av) (length initial-state))
  (let* ((retl (list (cons :gain ret))) p dp ddp (id) tau) ;; (cons :gain ret) ==> (:gain . ret)
    (setq x-buf x-min)
    (while (<= x-buf x-max)
      (when (eps= x-buf x-hit 1e-3)
        (print x-hit)
        )
      (setq id 0)
      (mapcar
       #'(lambda (bs js)
           (list
            (send js :joint-angle
                  (send bs :calc x-buf (subseq ret id (+ id id-max))))
            (send js :put :p (send bs :calc x-buf (subseq ret id (+ id id-max))))
            (send js :put :dp (send bs :calc-delta x-buf (subseq ret id (+ id id-max)) :n 1))
            (send js :put :ddp (send bs :calc-delta x-buf (subseq ret id (setq id (+ id id-max))) :n 2))
            ))
       bspline jlist)
      (push (send-all jlist :get :ddp) ddp)
      (push (send-all jlist :get :dp) dp)
      (push (send-all jlist :get :p) p)
      (push (send robot :angle-vector) *bspline-av-list*)
      (push x-buf *bspline-tm-list*)
      (push (send (send robot :rarm :end-coords) :copy-worldcoords) *bspline-endcoords-list*)
      (send *robot* :calc-torque-from-vel-acc
            :jvv (map float-vector
                      #'(lambda (j) (deg2rad (or (send j :get :dp) 0)))
                      (cdr (send robot :links)))
            :jav (map float-vector
                      #'(lambda (j) (deg2rad (or (send j :get :ddp) 0)))
                      (cdr (send robot :links))))
      (push (send-all jlist :joint-torque) tau)
      (setq x-buf (+ x-buf x-step))
      (send *viewer* :draw-objects)
      (x::window-main-one)
      (unix:usleep (round (* 0.01 1000 1000))))
    (push (cons :p (reverse p)) retl)
    (push (cons :dp (reverse dp)) retl)
    (push (cons :ddp (reverse ddp)) retl)
    (push (cons :tau (reverse tau)) retl)
    (setq *bspline-av-list* (reverse *bspline-av-list*))
    (setq *bspline-tm-list* (reverse *bspline-tm-list*))
    (setq *bspline-endcoords-list* (reverse *bspline-endcoords-list*))
    ;; (format t "  [dif] |~A| = ~A~%"
    ;;         (map float-vector #'- end-av (send-all jlist :joint-angle))
    ;;         (norm (map float-vector #'- end-av (send-all jlist :joint-angle))))
    ;; retl
    nil
    )
  )

(defun show-bspline-jvel (&key (limb :rarm))
  (funcall 'show-jvel :limb limb :tm-list *bspline-tm-list* :av-list *bspline-av-list*)
  )

(defun show-bspline-jvel-ratio (&key (limb :rarm))
  (funcall 'show-jvel-ratio :limb limb :tm-list *bspline-tm-list* :av-list *bspline-av-list*)
  )

;; (dolist (arm (send *robot* :rarm :links)) (dolist (b (append (send *robot* :head :links) (send *robot* :torso :links))) (print (pqp-collision-distance arm b))))
