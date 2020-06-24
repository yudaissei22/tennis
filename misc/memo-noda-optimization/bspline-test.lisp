(require "bspline.lisp")

(defun plot-bspline
  (&rest
   args
   &key
   ((:id-max M) 10) ;;;;;;;;;;;;;;;;;;;;;;; この数はどのように決めるものか（何を基準に基底関数の個数を決めればよいか）？？=====>タスク空間のパラメータより多くする
   ((:recursive-order N) 4) ;;;;;;;;;;;;;;;;;;;;;;;;; 変化させると山の裾の広がり具合が変わるようだが、この数も何を基準に決めれば良いのかわからない？？=====>多いほどなめらかになる
   ((:recursive-cnt _n) N) ;;;;;;;;;;;;;;;;;;;;;;;;;;; これは何を意味するのでしょうか？？
   (x-min 0.0)
   (x-max 1.0)
   (x x-min)
   (plot-cnt 100)
   (bspline
    (instance basic-spline :init
              :recursive-order N
              :recursive-cnt _n
              :id-max M
              :x-min x-min :x-max x-max))
   (pos-func :calc-coeff-vector) ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;; これの使い方がわからない pos-funcという変数に:calc-coeff-vectorが入っているという理解で良さげ！！
   (graph
    (progn
      (require "../../util/graph-sample.lisp")
      (setq *bspline-coeff-matrix-hash* (make-hash-table)) ;;;;;;;;;;;;;;;;;;;;;;;; hash tableってどうやって使うのですかー？？？？？？
      (create-graph
       (send bspline :to-string)
       :size '(640 320)
       :range
       (list (float-vector x-min 0)
             (float-vector x-max 1.))
       :name-list
       (mapcar
        #'(lambda (id) (format nil "id=~A" id))
        (send-all (send bspline :bspline-element-list) :id)) ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;; bspline-element-list＝slotsで、send-all ~ :idとは(idもスロット変数らしい)？？結局何をやっている？？
       :data-list
       (let* ((tm (instance mtimer :init)) ;;;;;;;;;;;;;;;;;;;;; mtimerってなんやー、多分ミリ秒？？:initでスタートして:stopで止まる？？
              (cnt x-min)
              (id -1)
              dlist xlist) ;;;;;;;;;;;;;こういう書き方の場合はdlistとxlistは別々？？xlistってnilってことか！！
         (dotimes (i plot-cnt) ;;;;;;;;;;;;;;;;;;;;;plot-cntは100。。。なにこれ=>100個プロットするのか！！
           (push (+ x-min (/ (* i 1.0 (- x-max x-min))
                             plot-cnt)) xlist)
           (push (send bspline pos-func (car xlist));;;;;;;;;;;;;;;;;;;;;;;send bspline :calc-coeff-vector (car xlist)！！
                 dlist)) ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;多分だけど、dlistはx=？？(x-min<=x<=x-max)に対するbsplineの出力か！！
         ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;M個分のxlistとdlistはどこで作っている？？
         (format t "TIME: ~A~%" (send tm :stop))
         (reverse
          (mapcar
           #'(lambda (hh)
               (incf id) ;;;;;;;;;;;;;;;;;;;;-1スタートだから0から！！
               (map cons ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;map consとは？？==========================>mapcarの上位互換
                    #'(lambda (dl x)
                        (float-vector x (aref dl id)))
                    dlist xlist))
           (make-list M))
          ))
       )))
   &allow-other-keys)
  (if graph
      (send graph :fit-draw));;;;;;;;;;;;;;;;;;;;;;;;;;; fit-drawはgraph-panel.lispにいた！！
  graph)

(defun demo-bspline-interpole
  ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;ところでなぜこれはkey変数にしているのか？他の条件をkeyで変えられる？？本当か？？
  (&key
   (robot
    (cond
     ((and (boundp '*robot*) *robot*) *robot*) ;;;;;;;;;;;;;;;;;;;;;; *robot*があってnilではなかったら*robot*を使う！！
     (t ;;
      (require "package://euslisp/jskeus/irteus/demo/sample-robot-model.l");;;;;;;;;;;;;;;;;;;;;;;;そうでなかったらsample-robotを使う！！
      (setq *robot* (instance sample-robot :init))
      ;;(require "package://euslisp/jskeus/irteus/demo/sample-robot-model.lisp")
      ;;(setq *robot* (instance sample-robot :init))
      (objects (list *robot*))
      *robot*)))
   (init (progn
           (require "package://eus_qp/euslisp/eiquadprog.lisp") ;;;;;;;;;;;;;;;;;;;;このdefforeignのコメントの大まかな説明が知りたい？？
           (send robot :reset-pose)))
   (jlist (send robot :rarm :joint-list))
   ;;(send robot :joint-list))
   ;;(flatten (send-all (send robot :rarm :links) :joint)))
   (start-av (send-all jlist :joint-angle)) ;;;;;;;;;;;;;;;;;;;;;;;;;(-133.001 45.0004 48.0005 -137.001 40.0004 47.0005 77.0005) (sample-robot)多分現在の関節角度列
   (end-av (mapcar #'(lambda (j)
                       (+ (send j :min-angle)
                          (random (- (send j :max-angle) (send j :min-angle)))))
                   jlist));;;;;;;;;;;;;;;;;;;;;;;;目標関節角度列（ここでランダムでつくっているらしい）
   (id-max 8) ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;なぜ8？ (:rarmの関節数は7/条件はなんだっけ？==========>最後の位置姿勢の6だから！！)
   (recursive-order 4)
   (x-min 0.0)
   (x-max 1.0) ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;このx-min=0とx-max=1にはどのような意味が存在するのか？？基本０から１にするものなのか？？
   (bspline
    (mapcar #'(lambda (k)
                (instance basic-spline :init
                          :id-max id-max :recursive-order recursive-order
                          :x-min x-min :x-max x-max))
            jlist)) ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;あれ、各関節ごとにbsplineを作るんでしたっけ？
   (initial-state
    (instantiate float-vector (* id-max (length bspline)))) ;;;;;;;;;;;;;;;;;;;;;;;;;; (length bspline)はjlistと一緒になる？？===========> basic-splineには(#<basic-spline #X63b5820> #<basic-spline #X6f44e98> #<basic-spline #X69aa010> #<basic-spline #X5ec2080> #<basic-spline #X60b74a8> #<basic-spline #X614a5b8> #<basic-spline #X61c2120>)的な情報が入っていた？？
   (state-min-vector
    (apply #'concatenate ;;;;;;;;;;;;;;;;;;;;;;(#<vectorclass #X10629c8 float-vector> (-150 -150 -150 -150 -150 -150 -150 -150) (-30 -30 -30 -30 -30 -30 -30 -30) (-90 -90 -90 -90 -90 -90 -90 -90) (-180 -180 -180 -180 -180 -180 -180 -180) (-90 -90 -90 -90 -90 -90 -90 -90) (-90 -90 -90 -90 -90 -90 -90 -90) (-90 -90 -90 -90 -90 -90 -90 -90))=======>#f(-150.0 -150.0 -150.0 -150.0 -150.0 -150.0 -150.0 -150.0 -30.0 -30.0 -30.0 -30.0 -30.0 -30.0 -30.0 -30.0 -90.0 -90.0 -90.0 -90.0 -90.0 -90.0 -90.0 -90.0 -180.0 -180.0 -180.0 -180.0 -180.0 -180.0 -180.0 -180.0 -90.0 -90.0 -90.0 -90.0 -90.0 -90.0 -90.0 -90.0 -90.0 -90.0 -90.0 -90.0 -90.0 -90.0 -90.0 -90.0 -90.0 -90.0 -90.0 -90.0 -90.0 -90.0 -90.0 -90.0)
           (cons float-vector
                 (mapcar
                  #'(lambda (j) (make-list id-max :initial-element (send j :min-angle)))
                  jlist))));;;;;;;;;;;;;;;;;;;;;;;;;;;;これはなぜ8(id-max)*7(jlist)=56も必要なの？？実行する限り1*56の
   (state-max-vector
    (apply #'concatenate
           (cons float-vector
                 (mapcar
                  #'(lambda (j) (make-list id-max :initial-element (send j :max-angle)))
                  jlist))))
   (equality-matrix-for-start/end-pos
    (matrix-append ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;; matrix-apppendとは？？===========>どうやら野田さんが定義したものらしくて、行列をある方向につなげる感じ
     (map cons
          #'(lambda (bs st ed)
              (send bs :calc-gain-vector-coeff-matrix-from-via-x-list (list 0.0 0.99))) ;;;;;;;;;;;;;;;;;このvia-x-listは何を意味しているか？
          bspline start-av end-av) ;;;;;;;;;;;;;;;;;;;stとedは何をやっているのか
     '(1 1))) ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;斜めにつなげる
   (equality-coeff-for-start/end-pos
    (concatenate float-vector
                 (flatten (map cons #'list start-av end-av)))) ;;;;;;;;;;;;;;;;;;;;;;;;;;;;; #f(-133.001 -104.0 45.0004 150.0 48.0005 73.0 -137.001 -80.0 40.0004 29.0 47.0005 -31.0 77.0005 21.0)
   ;;
   (equality-matrix-for-start/end-vel ;;;;;;;;;;;;;;;;;;;;速度の条件？？特に速度が関係しているようにはみえないが？？
    (matrix-append
     (map cons
          #'(lambda (bs st ed)
              (make-matrix
               2 id-max ;;;;;;;;;;;;;;;;;;;;;;;;2行id-max列
               (mapcar
                #'(lambda (x) (send bs :calc-delta-coeff-vector x :n 1))
                (list 0.0 0.99))))
          bspline start-av end-av)
     '(1 1)))
   (equality-coeff-for-start/end-vel ;;;;;;;;;;;;;;;;;;;;;;;;;これもそうだけどequality-matrix-~~~とequality-coeff-~~~の違いは？？
    (scale 0 (concatenate float-vector
                          (flatten (map cons #'list start-av end-av))))) ;;;;;;;;;;;;;;;;;;;;;;; scale 0にするんだったらmake-listとかinstantiateでもよさげなきがするけどなぜでしょうか？？scaleが0以外になる可能性があった？？
   ;;
   (equality-matrix-for-start/end-acc
    (matrix-append
     (map cons
          #'(lambda (bs st ed)
              (make-matrix
               2 id-max
               (mapcar
                #'(lambda (x) (send bs :calc-delta-coeff-vector x :n 2))
                (list 0.0 0.99))))
          bspline start-av end-av)
     '(1 1)))
   (equality-coeff-for-start/end-acc
    (scale 0 (concatenate float-vector
                          (flatten (map cons #'list start-av end-av)))))
   ;;
   (eval-weight-matrix
    (let* ((mat
            (matrix-append
             (mapcar
              #'(lambda (rate)
                  (matrix-append
                   (mapcar
                    #'(lambda (bs)
                        (make-matrix
                         1 id-max
                         (list
                          (scale
                           1e-3
                           (send bs :calc-delta-coeff-vector ;;;;;;;;;;;;;;; bsはbasic-splineのインスタンス？？
                                 (+ x-min (* rate (- x-max x-min))) ;;;;;;;;;;;つまり何をやりたいんでしょうか？？
                                 :n 3)))))
                    bspline)
                   '(1 1)))
              '(0 0.1 0.2 0.3 0.4 0.5 0.6 0.7 0.8 0.9 1.0)) ;;;;;;;;;;;;;;;;;;;何の0.1刻みでしょうか？
             '(1 0))))
      (m* (transpose mat) mat))) ;;;;;;;;;;;;;;mat=MとするとM^T*M
   (cnt 30) ;;;;;;;;;;;;;;;;何のカウントでしょう？？動作を30分割したとか？
   (x-step (/ (- x-max x-min) (* 1.0 cnt)))
   (x-buf x-min)
   (ret (solve-eiquadprog ;;;;;;;;;;;;;;;eus_qpに書いてある。ここは使うだけだからなんとなく何を与えるべきかを知れば大丈夫そう？？
         :debug? t
         :initial-state initial-state
         :eval-weight-matrix eval-weight-matrix
         :state-min-vector state-min-vector
         :state-max-vector state-max-vector
         :equality-vector
         (concatenate float-vector
                      equality-coeff-for-start/end-pos
                      equality-coeff-for-start/end-vel
                      equality-coeff-for-start/end-acc);;;;;;;;;;;;;;;;;;;;全部つなげて与える必要がある？？
         :equality-matrix
         (matrix-append
          (list equality-matrix-for-start/end-pos
                equality-matrix-for-start/end-vel
                equality-matrix-for-start/end-acc)
          '(1 0))
         ))
   )
  (if (null ret) (setq ret initial-state))
  (format t "   --- ~A x ~A = ~A variables~%" id-max (length start-av) (length initial-state))
  (let* ((retl (list (cons :gain ret))) p dp ddp (id) tau) ;;;;;;;;;;;;;;; (cons :gain ret) ==> (:gain . ret)
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
            (send js :put :ddp (send bs :calc-delta x-buf (subseq ret id (setq id (+ id id-max))) :n 2))
            )) ;;;;;;;;;;;;;;;;;;;; :putは一般的なメソッド？？何をしている？？
       bspline jlist)
      (push (send-all jlist :get :ddp) ddp)
      (push (send-all jlist :get :dp) dp)
      (push (send-all jlist :get :p) p)
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
    (format t "  [dif] |~A| = ~A~%"
            (map float-vector #'- end-av (send-all jlist :joint-angle))
            (norm (map float-vector #'- end-av (send-all jlist :joint-angle))))
    retl
    )
  )

;; (dolist (arm (send *robot* :rarm :links)) (dolist (b (append (send *robot* :head :links) (send *robot* :torso :links))) (print (pqp-collision-distance arm b))))
