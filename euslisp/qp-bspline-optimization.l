(require "package://tennis/euslisp/motion-choice") ;; for *load-pose-func*
(comp::compile-file-if-src-newer  (ros::resolve-ros-path "package://tennis/euslisp/bspline.l") (ros::resolve-ros-path "package://tennis/euslisp/"))
(require "package://tennis/euslisp/bspline")
(require "package://eus_qpoases/euslisp/eus-qpoases")

(defun qp-init (&key (robot *robot*))
  ;; set pose parameter according to the designated motion
  (funcall *load-pose-func*)
  ;; motion-choice内で指定された*limb*の関節角を取得
  (setq *limb-av-pose-list*
        ;; lpはlist(:angle-vector [angle-vector] :root-coords [worldcoords] :worldcoords [worldcoords] :time base-time)という形．
        ;; 要は抽出してlimbにある関節角を取得しているだけだが，以下にこのように動作する理由を示す．
        ;; (setq a (list :test 1 :test2 2))
        ;; (memq :test a)とすると(:test 1 :test 2)が返ってくるので，cadr(car cdr)を使ってリストの0から数えて1番目を抽出すると:testの後にある1が返ってくるようなイメージ
        (mapcar #'(lambda (lp)
                    (send robot :angle-vector (cadr (memq :angle-vector lp)))
                    (send robot *limb* :angle-vector))
                *lp-list*))

  (push nil *limb-av-pose-list*)
  ;; この時点で*limb-av-pose-list*にはlp(nil, 1, 2, 3, 4)のlimbのangle-vectorが入っている
  (send robot :reset-pose)
  (send robot :move-to (make-coords) :world)
  (setq *limb-end-coords-default* (send (send robot *limb* :end-coords) :copy-worldcoords))
  ;; *key-pose-id*の要素は1から5まであって，:poseを指定するとその番号のポーズをする関数
  ;; *key-pose-id*には，初期ポーズ，タスク達成ポーズ，最終ポーズの3つがある
  (funcall *load-pose-func* :pose (nth 0 *key-pose-id*)) ;; init-pose
  (send *irtviewer* :draw-objects))
  ;;(format t ";;(qp-motion-optimize :x-max 2.6 :x-hit 1.7 :maxvel-weight 1e3 :minjerk-weight 1e0 :x-step 0.02)~%")


;; optimization by qp
;; if use-all-joint t --> optimize motion using all joints
;; if use-all-joint nil --> optimize motion using *limb*(mainly rarm) joints
(defun qp-motion-optimize
  ;; {{{ 引数
  (&key
   (use-all-joint nil)
   (use-append-root-joint nil)
   (use-margin nil)
   (robot *robot*)
   ;; dummy variable to initialize
   (init (progn
           (qp-init)
           ;; (send robot :reset-pose)
           ;; ここでinitial-poseの姿勢を作る
           (if use-all-joint
             (funcall *load-pose-func* :pose (nth 0 *key-pose-id*))
             (send robot *limb* :angle-vector (nth (nth 0 *key-pose-id*) *limb-av-pose-list*)))))

   (jlist (if use-all-joint
            (send robot :joint-list)
            (send robot *limb* :joint-list)))


   ;; 開始時のjlistのangle-vectorをリスト化したもの
   (initial-pose (if use-append-root-joint
                   (let* ((pose-wc (progn (funcall *load-pose-func* :pose (nth 0 *key-pose-id*))
                                          (send robot :copy-worldcoords)))
                          (robot-pos (send pose-wc :worldpos))
                          (robot-rpy (nth 0 (rpy-angle (send pose-wc :worldrot)))))
                    (flatten (list (send-all jlist :joint-angle) (coerce robot-pos cons) (mapcar #'rad2deg (reverse robot-rpy)))))
                   (send-all jlist :joint-angle))) ;; initial-pose = (flatten coerce *limb-av-pose1* cons)

   ;; タスク達成時の*robot*または*limb*のangle-vectorをリスト化したもの
   (hitting-pose (if use-all-joint
                   (if use-append-root-joint
                     (let* ((pose-wc (progn (funcall *load-pose-func* :pose (nth 1 *key-pose-id*))
                                            (send robot :copy-worldcoords)))
                            (robot-pos (send pose-wc :worldpos))
                            (robot-rpy (nth 0 (rpy-angle (send pose-wc :worldrot)))))
                       (flatten (list (send-all jlist :joint-angle) (coerce robot-pos cons) (mapcar #'rad2deg (reverse robot-rpy)))))
                     (progn (funcall *load-pose-func* :pose (nth 1 *key-pose-id*)) (coerce (send robot :angle-vector) cons)))
                   (coerce (nth (nth 1 *key-pose-id*) *limb-av-pose-list*) cons)))

   ;; 動作終了時の*limb*のangle-vector
   (final-pose (if use-all-joint
                 (if use-append-root-joint
                   (let* ((pose-wc (progn (funcall *load-pose-func* :pose (nth 2 *key-pose-id*))
                                          (send robot :copy-worldcoords)))
                          (robot-pos (send pose-wc :worldpos))
                          (robot-rpy (nth 0 (rpy-angle (send pose-wc :worldrot)))))
                     (flatten (list (send-all jlist :joint-angle) (coerce robot-pos cons) (mapcar #'rad2deg (reverse robot-rpy)))))
                   (progn (funcall *load-pose-func* :pose (nth 2 *key-pose-id*)) (coerce (send robot :angle-vector) cons)))
                 (coerce (nth (nth 2 *key-pose-id*) *limb-av-pose-list*) cons)))

   (id-max 14) ;; タスク変数(位置姿勢6)より大きくする
   (recursive-order 5)

   (x-min 0.0)
   (x-max 2.6)
   (x-hit (* x-max 0.65))

   (bspline
    (mapcar #'(lambda (k)
                (instance basic-spline :init
                          :id-max id-max :recursive-order recursive-order
                          :x-min x-min :x-max x-max))
            (if use-append-root-joint (flatten jlist (make-list 6)) jlist))) ;; 各関節(*jaxon_red*なら33個)にそれぞれid-max個のbsplineを作る、bsplineには(#<basic-spline #X8f25708> #<basic-spline #Xe85b650> #<basic-spline #Xfbd6448> #<basic-spline #X11555750>... #<basic-spline #X12afff10> #<basic-spline #X218ad928> #<basic-spline #X250322c8> #<basic-spline #Xf8f82d8>)のように関節数分のbsplineのインスタンスのリストが入る

   (initial-state
    (instantiate float-vector (* id-max (length bspline))))
   ;; (length bspline) = (length jlist)
   ;; *jaxon_red*の場合(length initial-state)=id-max*33

   ;; 関節角の最小角度を並べたリストのリスト(33*id-max)にrootlinkの最小角度を並べたリストのリストを足して(39*id-max)を1次元に並べてfloat-vectorにしたもの
   (state-min-vector
    ;; concatenate float-vector a bでa, bを連結してfloat-vectorにして返す．bが可変長なのでconsとapplyを合わせ技している
    ;; cons float-vector (list (list 1) (list 2)) => #f(1.0 2.0)
    ;; (#<vectorclass #X1ff49c8 float-vector> (-17.5812 -17.5812 -17.5812 -17.5812 -17.5812 -17.5812 -17.5812 -17.5812 -17.5812) (-180.0 -180.0 -180.0 -180.0 -180.0 -180.0 -180.0 -180.0 -180.0) (-180.0 -180.0 -180.0 -180.0 -180.0 -180.0 -180.0 -180.0 -180.0) (-180.0 -180.0 -180.0 -180.0 -180.0 -180.0 -180.0 -180.0 -180.0) (-125.481 -125.481 -125.481 -125.481 -125.481 -125.481 -125.481 -125.481 -125.481) (-180.0 -180.0 -180.0 -180.0 -180.0 -180.0 -180.0 -180.0 -180.0) (-90.0002 -90.0002 -90.0002 -90.0002 -90.0002 -90.0002 -90.0002 -90.0002 -90.0002) (-81.0 -81.0 -81.0 -81.0 -81.0 -81.0 -81.0 -81.0 -81.0))
    ;;       ↓
    ;; #f(-17.5812 -17.5812 -17.5812 -17.5812 -17.5812 -17.5812 -17.5812 -17.5812 -17.5812 -180.0 -180.0 -180.0 -180.0 -180.0 -180.0 -180.0 -180.0 -180.0 -180.0 -180.0 -180.0 -180.0 -180.0 -180.0 -180.0 -180.0 -180.0 -180.0 -180.0 -180.0 -180.0 -180.0 -180.0 -180.0 -180.0 -180.0 -125.481 -125.481 -125.481 -125.481 -125.481 -125.481 -125.481 -125.481 -125.481 -180.0 -180.0 -180.0 -180.0 -180.0 -180.0 -180.0 -180.0 -180.0 -90.0002 -90.0002 -90.0002 -90.0002 -90.0002 -90.0002 -90.0002 -90.0002 -90.0002 -81.0 -81.0 -81.0 -81.0 -81.0 -81.0 -81.0 -81.0 -81.0)
    (if use-append-root-joint
      (apply #'concatenate
             (cons float-vector
                   (append
                     (mapcar
                       #'(lambda (j) (make-list id-max
                                                :initial-element (if use-margin (+ (send j :min-angle) use-margin) (send j :min-angle))))
                       jlist)
                     (list (make-list id-max :initial-element (elt #f(-inf) 0))
                           (make-list id-max :initial-element (elt #f(-inf) 0))
                           (make-list id-max :initial-element 0.0)
                           (make-list id-max :initial-element -pi/2)
                           (make-list id-max :initial-element -pi/2)
                           (make-list id-max :initial-element -2pi)))))
      (apply #'concatenate
             (cons float-vector
                 (mapcar
                   #'(lambda (j) (make-list id-max
                                              :initial-element (if use-margin (+ (send j :min-angle) use-margin) (send j :min-angle))))

                  jlist)))))

   ;; 関節角の最大角度を並べたfloat-vector
   (state-max-vector
    (if use-append-root-joint
      (apply #'concatenate
             (cons float-vector
                   (append
                     (mapcar
                       #'(lambda (j) (make-list id-max
                                                :initial-element (if use-margin (- (send j :max-angle) use-margin) (send j :max-angle))))
                       jlist)
                     (list (make-list id-max :initial-element (elt #f(inf) 0))
                           (make-list id-max :initial-element (elt #f(inf) 0))
                           (make-list id-max :initial-element (elt #f(inf) 0))
                           (make-list id-max :initial-element pi/2)
                           (make-list id-max :initial-element pi/2)
                           (make-list id-max :initial-element 2pi)))))

      (apply #'concatenate
             (cons float-vector
                 (mapcar
                   #'(lambda (j) (make-list id-max
                                           :initial-element (if use-margin (- (send j :max-angle) use-margin) (send j :max-angle))))

                  jlist)))))

   ;; min_x xAx + Bx, Cx + D = 0
   (equality-matrix-for-start/end-pos
    (matrix-append ;; s-nodaさんが作った関数で、'(1 0) で行(下)方向に、'(0 1)で列(横)方向に、'(1 1)で斜め方向につなげる ex)(matrix-append (list (matrix (list 1 2 3) (list 4 5 6)) (matrix (list 7 8) (list 9 10))) '(1 1)) = #2f((1.0 2.0 3.0 0.0 0.0) (4.0 5.0 6.0 0.0 0.0) (0.0 0.0 0.0 7.0 8.0) (0.0 0.0 0.0 9.0 10.0))
     ;; map [返り値の型T] [(ai bi ci...) := Tなる関数] [A] [B] [C] [...]
     ;; A, B, Cの長さが異なる場合はiは長さ最小のものまでイテレーションされる
     ;; 返り値は長さ33, 要素#2f(3x(length id-max))のlist型
     (map cons
          #'(lambda (bs st ed)
              ;; 3 x 長さ14float-vectorの#2f型が返ってくる
              (send bs :calc-gain-vector-coeff-matrix-from-via-x-list (list 0.0 x-max x-hit)))
          ;; ex) (setq via-x-list (list 0.5)) (send (nth 0 bspline) :calc-gain-vector-coeff-matrix-from-via-x-list via-x-list) = #2f((0.0 0.0 0.002604 0.197917 0.598958 0.197917 0.002604 0.0 0.0))
          ;; (bspline, initial-pose, final-poseは長さ33)
          bspline initial-pose final-pose)
     '(1 1))) ;; #2f (33x3) x (33x(length id-max))行列

   (equality-coeff-for-start/end-pos
    (concatenate float-vector
                 (flatten (map cons #'list initial-pose final-pose hitting-pose)))) ;; #f(initial-pose final-pose hitting-pose)

   (equality-matrix-for-start/end-vel ;; 初期・終端条件(速度ゼロ)
    (matrix-append
     (map cons
          #'(lambda (bs st ed)
              (make-matrix
               2 id-max ;;2 行id-max列
               (mapcar
                #'(lambda (x) (send bs :calc-delta-coeff-vector x :n 1))
                (list 0.0 x-max))))
          bspline initial-pose final-pose)
     '(1 1)))

   (equality-coeff-for-start/end-vel
    (scale 0 (concatenate float-vector
                          (flatten (map cons #'list initial-pose final-pose)))))

   (equality-matrix-for-start/end-acc ;; 初期・終端条件(加速度ゼロ)
    (matrix-append
     (map cons
          #'(lambda (bs st ed)
              (make-matrix
               2 id-max
               (mapcar
                #'(lambda (x) (send bs :calc-delta-coeff-vector x :n 2))
                (list 0.0 x-max))))
          bspline initial-pose final-pose)
     '(1 1)))

   (equality-coeff-for-start/end-acc
    (scale 0 (concatenate float-vector
                          (flatten (map cons #'list initial-pose final-pose)))))

   ;; これを小さくするとinit-pose(stateが0になる状態)になるような解
   (minjerk-weight 1e-3)
   ;; (minjerk-weight 1)
   (eval-weight-matrix ;; (jlist*id-max)^2の正方行列
    ;; minimize integreted jerk squared
    (let* ((mat
            (matrix-append
             (mapcar
              #'(lambda (rate)
                  (m* (let ((l (make-list 39 :initial-element 1))) (setf (elt l 35) 1000) (diagonal l))
                      (matrix-append
                      (mapcar
                        #'(lambda (bs)
                            (make-matrix ;; 1行id-max列の行列を作る
                              1 id-max
                              (list
                                (send bs :calc-delta-coeff-vector ;; :calc-delta-coeff-vector (x &key (n 1)) bsplineのn階微分したときのxにおける値
                                      (+ x-min (* rate (- x-max x-min))) ;; x-minからx-maxの間で離散化
                                      :n 3))))
                        bspline) ;; 1行id-max(=9)列の行列の関節数(=8)個のlist
                   '(1 1))))
              '(0 0.1 0.2 0.3 0.4 0.5 0.6 0.7 0.8 0.9 1.0)) ;; 8行72列の行列の離散化した分(=11)のリスト
             '(1 0)))) ;; 88行72列の行列(上のリストを縦につなげる)
      (m* (transpose mat) mat))) ;; M^T*M jerkの二乗


   (unit-matrix-weight 1e-3)
   ;; (eigen-weight 1) ;; for compile
   (maxvel-weight 1)
   (eval-weight-matrix2 (progn
    ;; maximize joint velocity at a time point x-hit
                         (let* ((mat
                                 (matrix-append
                                  (mapcar
                                   #'(lambda (bs)
                                       (make-matrix ;; 1行id-max(=9)列の行列を作る
                                        1 id-max
                                        (list
                                         (send bs :calc-delta-coeff-vector x-hit :n 1)))) ;; x-hitにおけるbsplineの1階微分（＝速度）の値

                                   bspline) ;; 1行id-max(=9)列の行列の関節数(=8)個のlist
                                  '(1 1))))
                           (scale-matrix -1 (m* (transpose mat) mat))))) ;; 速度を二乗後、maximizeのため符号反転


   (cnt 30)
   (x-step ;; x-stepごとに結果を出力
    ;;0.002
    (/ (- x-max x-min) (* 1.0 cnt)))


   (ret (solve-eiquadprog
         ;; solve-qpoases-qp
         ;; :debug? t
         :initial-state initial-state
         :eval-weight-matrix (setq *mat* (m+ (scale-matrix minjerk-weight eval-weight-matrix)
                                             ;; 正定値性を保とうとして足しこんだ(ここは小さくする)
                                             ;; (scale-matrix eigen-weight
                                             (scale-matrix unit-matrix-weight (unit-matrix (send eval-weight-matrix :get-val 'dim0)))
                                             ;; )
                                             ;; 速度の評価用に使う行列
                                             (scale-matrix maxvel-weight eval-weight-matrix2)))
         ;; 長さ33 x 14
         :state-min-vector state-min-vector
         :state-max-vector state-max-vector
         ;; 位置3種類(start hit end), 速度2種類(start end), 加速度2種類(start end)
         ;; 長さ7 x 33
         :equality-vector
         (setq *evec*
         (concatenate float-vector
                      equality-coeff-for-start/end-pos
                      equality-coeff-for-start/end-vel
                      equality-coeff-for-start/end-acc))
         ;; 位置3種類(start hit end), 速度2種類(start end), 加速度2種類(start end)
         :equality-matrix
         (setq *emat*
         (matrix-append
          (list equality-matrix-for-start/end-pos
                equality-matrix-for-start/end-vel
                equality-matrix-for-start/end-acc)
          '(1 0)))))
   ;; これ以上keyを追加するとkey数が32個を超えてコンパイル時に失敗する
   )
  ;; }}}

  (format t "(qp-motion-optimize :x-max ~a :x-hit ~a :maxvel-weight ~a :minjerk-weight ~a :x-step ~a :use-all-joint ~a :use-margin ~a)~%" x-max x-hit maxvel-weight minjerk-weight x-step use-all-joint use-margin)

  ;; 462  = 33 x 14
  (setq *ret* (copy-object (if ret ret initial-state)))
  ;; (setq *bspline* bspline)

  (setq *bspline-av-list* '())
  (setq *bspline-tm-list* '())
  (setq *bspline-ec-list* '())

  (if (null ret) (setq ret initial-state))
  (format t "   --- ~A x ~A = ~A variables~%" id-max (length initial-pose) (length initial-state))
  (let* ((retl (list (cons :gain ret))) (x-buf x-min) p dp ddp (id) tau) ;; (cons :gain ret) ==> (:gain . ret)
    (setq x-buf x-min)
    (while (<= x-buf x-max)
      (setq id 0)
      (mapcar
       #'(lambda (bs js)
           (list
            (send js :joint-angle
                  (send bs :calc x-buf (subseq ret id (+ id id-max))))
            (send js :put :p (send bs :calc x-buf (subseq ret id (+ id id-max))))
            (send js :put :dp (send bs :calc-delta x-buf (subseq ret id (+ id id-max)) :n 1))
            (send js :put :ddp (send bs :calc-delta x-buf (subseq ret id (setq id (+ id id-max))) :n 2))))

       (subseq bspline 0 (length jlist)) jlist)
      (push (send-all jlist :get :ddp) ddp)
      (push (send-all jlist :get :dp) dp)
      (push (send-all jlist :get :p) p)
      (push (send robot :angle-vector) *bspline-av-list*)
      (push x-buf *bspline-tm-list*)
      (push (send (send robot *limb* :end-coords) :copy-worldcoords) *bspline-ec-list*)
      (send robot :calc-torque-from-vel-acc
            :jvv (map float-vector
                      #'(lambda (j) (deg2rad (or (send j :get :dp) 0)))
                      (cdr (send robot :links)))
            :jav (map float-vector
                      #'(lambda (j) (deg2rad (or (send j :get :ddp) 0)))
                      (cdr (send robot :links))))
      (push (send-all jlist :joint-torque) tau)
      ;; (send robot :fix-leg-to-coords *lfoot* :lleg)
      (setq x-buf (+ x-buf x-step))
      (send *irtviewer* :draw-objects)
      (x::window-main-one)
      (unix:usleep (round (* 0.01 1000 1000))))
    (push (cons :p (reverse p)) retl)
    (push (cons :dp (reverse dp)) retl)
    (push (cons :ddp (reverse ddp)) retl)
    (push (cons :tau (reverse tau)) retl)
    (setq *bspline-av-list* (reverse *bspline-av-list*))
    (setq *bspline-tm-list* (reverse *bspline-tm-list*))
    (setq *bspline-ec-list* (reverse *bspline-ec-list*))
    ;; (dotimes (i (+ (if use-append-root-joint 6 0) (length jlist)))
    ;;   (print (subseq *ret* (* id-max i) (* id-max (+ i 1)))))
    (setq id 0)
    (mapcar
     #'(lambda (bs js)
         (list
          (send js :joint-angle
                (send bs :calc x-min (subseq ret id (+ id id-max))))
          (send js :put :p (send bs :calc x-min (subseq ret id (+ id id-max))))
          (send js :put :dp (send bs :calc-delta x-min (subseq ret id (+ id id-max)) :n 1))
          (send js :put :ddp (send bs :calc-delta x-min (subseq ret id (setq id (+ id id-max))) :n 2))))

     (subseq bspline 0 (length jlist)) jlist)
    (print initial-pose)
    (format t "  [dif] |~A| = ~A~%"
            (map float-vector #'- initial-pose (send-all jlist :joint-angle))
            (norm (map float-vector #'- initial-pose (send-all jlist :joint-angle))))
    (print ret)
    (setq id 0)
    (mapcar
     #'(lambda (bs js)
         (list
          (send js :joint-angle
                (send bs :calc x-hit (subseq ret id (+ id id-max))))
          (send js :put :p (send bs :calc x-hit (subseq ret id (+ id id-max))))
          (send js :put :dp (send bs :calc-delta x-hit (subseq ret id (+ id id-max)) :n 1))
          (send js :put :ddp (send bs :calc-delta x-hit (subseq ret id (setq id (+ id id-max))) :n 2))))

     (subseq bspline 0 (length jlist)) jlist)
    (format t "  [dif] |~A| = ~A~%"
            (map float-vector #'- hitting-pose (send-all jlist :joint-angle))
            (norm (map float-vector #'- hitting-pose (send-all jlist :joint-angle))))
    (setq id 0)
    (mapcar
     #'(lambda (bs js)
         (list
          (send js :joint-angle
                (send bs :calc x-max (subseq ret id (+ id id-max))))
          (send js :put :p (send bs :calc x-max (subseq ret id (+ id id-max))))
          (send js :put :dp (send bs :calc-delta x-max (subseq ret id (+ id id-max)) :n 1))
          (send js :put :ddp (send bs :calc-delta x-max (subseq ret id (setq id (+ id id-max))) :n 2))))

     (subseq bspline 0 (length jlist)) jlist)
    (format t "  [dif] |~A| = ~A~%"
            (map float-vector #'- final-pose (send-all jlist :joint-angle))
            (norm (map float-vector #'- final-pose (send-all jlist :joint-angle))))
    ;; retl
    nil))

;; (dolist (arm (send *robot* *limb* :links)) (dolist (b (append (send *robot* :head :links) (send *robot* :torso :links))) (print (pqp-collision-distance arm b))))
