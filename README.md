# 初期軌道生成
1.
```
$ roscd tennis/euslisp
$ roseus
$ (qp-init)
$ (qp-motion-optimize :use-all-joint t :use-append-root-joint t :use-margin 30 :id-max 14 :recursive-order 5 :x-min 0.0 :x-max 1.0 :x-hit 0.5)
$ (print *ret*) ;; 最後にx-hitを入れると最適化変数初期値*p-orig*になる
```

# 最適化計算
1. euslispディレクトリにおいてroseusを立ち上げ，変数\*motion-choice\*に"forehand"， "batting"， "punch"， "smash"， "forehand-step" などのいずれかの文字列を設定して最適化の動作を選択する．
```
$ roscd tennis/euslisp ;; tennisをビルドしておく
$ roseus
$ (setq *motion-choice* "forehand") ;; forehandで最適化をするときの例
```
2. nlopt_bspline_optimization.lをloadする．そのままloadしても良いが，nlopt_bspline_optimization.lをコンパイルしてから.soをloadする方が計算速度が速くなる．
```
$ (load "nlopt_bspline_optimization.l")
   or
$ (compile-file "nlopt_bspline_optimization.l")
$ (load "nlopt_bspline_optimization.so")
```
3. loadしたらコメントとして表示される例をもとに関数nlopt-initを実行する．
```
$ (nlopt-init :x-max 2.4 :x-hit nil :id-max 14 :recursive-order 5 :use-all-joint t :use-append-root-joint t :support-polygon-margin (list 50 50 0 100 50) :epsilon-c 30 :mu 0.3 :use-final-pose nil :default-switching-list nil :use-6dof-p t)
```
記号は2016年度卒のterasawaの修論を参照
* x-max: 動作全体の時間(終端時刻)t_f [s]
* x-hit: タスク実行時刻 t_e [s] ;; (これはただの初期設定で，最適化の設計変数に含まれるものなのでnilでもよい(自動的に何らかの値が設定されるため))
* id-max: B-spline曲線の既定関数の数 m
* recursive-order: B-spline曲線の次数 n
* use-all-joint: 最適化に全関節を使うか，それとも右腕など一部の関節のみを使うか (t or nil)
* use-append-root-joint: ルートリンクの６自由度を最適化に含めるか (t or nil)
* support-polygon-margin: 支持多角形のマージンepsilon_b [mm] ex.(list 50 50 0 100 50) 順番に　足の前・後・内側・外側・片足時の周り
* epsilon-c: 干渉のマージン epsilon_c [mm]
* mu: 動摩擦係数 mu
* use-final-pose: キーポーズの３つ目を使うかどうか (t or nil)
* default-switching-list: 片足支持期が存在するときのデフォルトの離地時刻 t_l や着地時刻 t_g [s]
* use-6dof-p: \*p\*(最適化変数)に6dofが含まれているかどうか (t or nil)

ひとまず試すだけなら，use-all-joint以下は特に変更せずにコメントとして表示される例をそのまま使って良い．

4. nlopt-initをしたあとに表示されるコメントの例をもとに関数nlopt-motion-optimizeを実行する＝＞最適化計算の結果は変数\*p\*に勝手に入るが，前に計算した値を使いたいときなどはそのfloat-vectorの変数名を\*p\*としておくとよい．
```
(nlopt-motion-optimize :x-max 2.4 :x-hit 1.46104 :id-max 14 :recursive-order 5 :max-eval 100000000 :alg SLSQP :delta (deg2rad 0.01) :eqthre 1e-8 :xtol 1e-10 :ftol 1e-15 :use-all-joint t :use-margin 0.5 :use-append-root-joint t :maxvel-weight 1 :minjerk-weight 5e-3 :modify-ec t :p *p* :interval-num 48 :title "maximize-speed" :max-time (* 2 24 60 60) :file-path "/tmp")
```
* x-max, x-hit, id-max, recursive-order, use-all-joint, use-append-root-jointは3.を参照(nlopt-initで設定した値がここに表示されているはず)
* max-eval: 評価の回数(大きめの値にして時間(:max-time)で切るのが良いかも)
* alg: NLoptで使うアルゴリズム(eus_nloptを参照)
* delta: 最適化計算時に勾配をどれだけ減らすか
* eqthre: 等式条件の閾値
* xtol: 最適化の設計変数の変化量の閾値(下回ったら停止)
* ftol: 最適化の評価関数の変化量の閾値(　　　〃　　　)
* use-margin: 安全のため設計変数のうちの関節角度の探索範囲をやや狭める [deg]
* maxvel-weight: 速度最大化の重み
* minjerk-weight: 躍度最小の重み
* modify-ec: 最適化計算時に:end-coordsをハンドからラケットやバットに移すかどうか (t or nil)
* p: デフォルトの設計変数を与える (与えない場合は何も表記しない(:p nilも書いてはダメ！)と初期値にQPの躍度最小をとりあえず用いて最適化を開始する)
* interval-num: 離散化する必要があるバランスや滑りなどの制約の離散化の数(この分だけ不等式成約が増えるのであまり多くすると計算が重くなる)
* title: 最適化の計算のログを吐き出すときのファイル名のタイトル部分の指定(開始した時の時刻や最適化の条件などは自動でファイル名につく)
* max-time: 最適化計算をする時間 [s]
* file-path: ログの保存場所

5.最適化計算の結果をeusのirtviewerで確認する場合は関数show-optimized-motionを用いる．
* nlopt_bspline_optimization.lの一番最後のコメントにある４つのshow-optimized-motionのどれかをコメントインして実行するとよい．
```
;; 通しでみたい場合は４つのうち一番最初のコメントを外す

* もとのversionのだが、エラーが出てくる。
(progn
  (send *robot* :move-to (make-coords) :world)
  (with-append-root-joint
   (link-list-with-robot-6dof *robot* (list (cdr (send *robot* :links)))
                              :joint-class 6dof-joint)
   (show-optimized-motion *p* :x-max *x-max-of-p-orig* :real-time t :lfoot-ik nil :rfoot-ik nil)
   ;;(show-optimized-motion *p* :x-max *x-max-of-p-orig* :real-time nil :make-loadpattern "/home/terasawa/bspline_1.5_M-14_N-5" :zmp-offset #f(0 0 0) :choreonoid nil)
   ;;(show-optimized-motion *p* :specific-time *x-max-of-p-orig*)
   ;;(show-optimized-motion *p* :x-step 0.01 :x-max *x-max-of-p-orig* :real-time nil :make-sequence t :zmp-offset #f(0 0 0) :choreonoid t)                                 
   ))

* こっちだと再生できる。
(progn  ;; このブロックはnlopt_bspline_optimization.lの一番下のコメント部分にある(progn〜と一緒で，ここでは 4つの;;(show-optimized-motion 〜)については一番下をコメントインしたものと同じです）
  (send *robot* :move-to (make-coords) :world)
  (with-append-root-joint
   (link-list-with-robot-6dof *robot* (list (cdr (send *robot* :links)))
                              :joint-class 6dof-joint)
   (show-optimized-motion *p* :x-step 0.01 :x-max *x-max-of-p-orig* :real-time nil :make-sequence t :zmp-offset #f(0 0 0) :choreonoid nil)  ;; x-stepのオプションでavlistの時間間隔を設定できます．それ以外は多分パラメータは変えなくてよいです
   ))

```


# 最適化した動作の実行手順
1. choreonoidを用いる場合、choreonoidを起動する
* cd ../scripts/ && bash ./choreonoid_settings.sh
* rtmlaunch hrpsys_choreonoid_tutorials jaxon_red_choreonoid.launch TASK:=FOREHAND
2. experiment_utils.lをloadする
```
$ (load experiment_utils.l)
```
* 動作をforehand以外にしたいときは「最適化計算」1.を参考に\*motion-choice\*に違う変数をセットして一度関数nlopt-initを呼んでから進めると良い．
* 用いる設計変数\*p\*を変えたい場合は，tennis/config/p-orig.lを変更すると良い．(デフォルトではここに記された変数\*p-orig\*を\*p\*にセットしている)
* p-orig.lでは\*x-max-of-p-orig\*という変数もセットしている．これは最適化時のx-maxの値に相当するため，最適化計算と等倍の時間を用いたいときは\*x-max-of-p-orig\*を使うと良い
3. 関数experiment-initで\*ri\*を作るなどの初期設定をする
```
$ (experiment-init)
```
* ここで補間モードを*linear*にしているので要注意!!
4. 関数make-sequence-in-advanceで\*exp-~1\*という変数にangle-vector-sequence-fullで用いるシーケンスのデータをセットする(experiment_util.lの一番下のコメントを参照)
```
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
```
* t-max: 最適化で設定した時刻 t_f をt-maxまで等倍で動作を引き伸ばす[s](たとえば，t-maxを30.0にすると，2.4秒などで作った動作を30秒に引き伸ばして実行できるので，実機で確認するときなどはまず30.0などで作ってみると良い)
* step: angle-vector-sequence-fullの時間間隔(0.002にしたらSequencePlayerの補間を使わない)
* choreonoid: choreonoidがtだと\*exp-jpos-〜\*の後ろ４桁にTHKハンドの関節角度を追加している (t or nil)
* lfoot-ik: 最適化で作った動作で完全に接地した状態にいするのは難しいので，tなら動作を作るさいにIKをつかう．#f(10 0 0)などにするとその方向に足を動かしてIKを解きにいく．rfoot-ikも同様．
5. experiment_utils.lの最後にあるコメントを参考に，種々の関数を実行して実機を動かす
* experiment-angle-vector-sequence-fullで一回スイングする．
```
(experiment-angle-vector-sequence-full *exp-jpos-deg1* *exp-tm-ms1* *exp-rc1* *exp-zmp-wc1* *exp-optional1* :initial-time 10000 :final-time 5000 :log-fname "/tmp/angle-vector-sequence-full") 
```
* initial-time: シーケンスの最初の姿勢に移るまでの時間 [ms]．この例だと10秒で最初の姿勢まで移り，そこからangle-vector-sequence-fullでシーケンスを実行する
* final-time: シーケンスの実行が終わってからこの時間の間はsleepする．この直後，save-logでhrpsysのlogを書き出す
* log-fname: hrpsysのlogの名前

* ただし，実機を使う場合は１度
```
(experiment-angle-vector-sequence-full (list (car *exp-jpos-deg1*)) (list (car *exp-tm-ms1*)) (list (car *exp-rc1*)) (list (car *exp-zmp-wc1*)) (list (car *exp-optional1*)) :initial-time 2000 :final-time 0 :log-fname "/tmp/init")
```
で最初の姿勢まで動かしてから，STを入れるとよい．(これをしないとoptional_dataに何もデータが入ってない状態でSTを入れることになってしまい倒れる)
* 現状AutoBalancerは入れないほうが良い．フィードフォワードで重心位置が補正されてしまうので適切でない．


# テニス環境
```bash
rtmlaunch tennis tennis.launch
```
