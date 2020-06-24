(comp::compile-file-if-src-newer (ros::resolve-ros-path "package://tennis/euslisp/nlopt_bspline_optimization.l") nil)
(load "package://tennis/euslisp/nlopt_bspline_optimization")

;; calc dp from dr [m(), rad(RPY)] using qp (minimize dp^T dp)
(defun calc-dp-from-dr
  (dr t-current x-hit
   &key
   (robot *robot*)
   (bspline *bspline*)
   (debug? t)
   )
  ;; p = [p_0 p_1 ... p_M]
  ;; b(t_e) = [ b_0 b_1 ... b_M ]^T
  ;; \theta(t_e) = p^T b(t_e) = p_0 b_0 + p_1 b_1 + ... + p_M b_M
  ;; t_{m}で\Delta r が与えられたとする。h(=\frac{x_{max} - x_{min}}{M - N})に対して, x \geq t_{m} + \frac{N+1}{2} h 以降のB-splineの値を[b_K ... b_M] ^Tとすると,
  ;; \Delta \theta = (b_K + ... + b_M) \Delta p [17, 1]
  ;; \Delta r = J(\theta) \Delta \theta [6, 1]
  ;; \Delta p = J(\theta)^+ \Delta r [17, 1] <== 結局IKを解いてしまった

  ;; 打つ姿勢におけるヤコビアン（単位系は[rad],[m]）を計算
  (let* ((id 0)
         ;; (position val (list val val a b c) :test-not #'=) => 2 初めてvalじゃない数が現れるのを0から数えたもの
         (online-modified-min-id (position 0.0 (send (car bspline) :calc-coeff-vector t-current) :test-not #'=))
         (online-modified-max-id (- *id-max* (position 0.0 (reverse (send (car bspline) :calc-coeff-vector x-hit)) :test-not #'=))) ;; これ微妙．実際はさらに-1したものが正しいidだが後のソースで吸収している
         (C (- online-modified-max-id online-modified-min-id)) ;; C = 2(N+1)-c
         (online-modified-links ;;root-jointを入れるとよくわからなくなるのでとりあえず除く
          ;;(if (send (car (send *robot* :links)) :joint)
          ;;(flatten (list (car (send robot :links)) (send robot :torso :links) (send robot *limb* :links)))
          ;; (flatten (list (send robot :torso :links) (send robot *limb* :links)))
          (send robot *limb* :links)
          )
         (online-modified-jlist
          (send-all online-modified-links :joint)
          )
         (K (length online-modified-jlist))

         ;; idでdqを求める
         (current-pose
          ;; x-hitにおける関節角度の計算
          (progn
            (mapc
             #'(lambda (jl bs)
                 (send jl :joint-angle
                       (send bs :calc x-hit (subseq *p* id (+ id *id-max*))))
                 (setq id (+ id *id-max*))
                 )
             *jlist* bspline)
            (if (send (car (send robot :links)) :joint)
                (progn
                  (send (send (car (send robot :links)) :joint) :joint-angle
                        (coerce
                         (mapcar
                          #'(lambda (bs)
                              (send bs :calc x-hit (subseq *p* id (setq id (+ id *id-max*)))))
                          (subseq bspline (length *jlist*) (+ (length *jlist*) 6)))
                         float-vector)
                        )
                  )
              (progn
                (send robot :fix-leg-to-coords *lfoot* :lleg)
                )
              )

            (map float-vector #'(lambda (jl) (send jl :joint-angle)) online-modified-jlist)
            ))
         (dq
          (progn
            (modify-end-coords)
            ;; 結局ikを解くのであればそのうちtargetを引数にしたほうがよいかもしれない
            (let* ((target (send robot *limb* :end-coords :copy-worldcoords)) after-ik before-ik)
              ;; (format t "target=~a~%" target)
              ;; (format t "dr=~a~%" (subseq dr 0 3))
              (send target :translate (scale 1e3 (subseq dr 0 3)) :world) ;; dr[m] に注意
              ;; (format t "target=~a~%" target)
              (send target :rotate (aref dr 5) :z :world)
              (send target :rotate (aref dr 3) :x :world)
              (send target :rotate (aref dr 4) :y :world)
              (setq before-ik (send robot :angle-vector))
              (send robot *limb* :inverse-kinematics target :rotation-axis :z :link-list online-modified-links :revert-if-fail nil
                    :debug-veiw nil) ;; もしかしたら:revert-if-fail nilだと危ないかもしれない
              (setq after-ik (send robot :angle-vector))
              (when debug?
                (format t "after-ik - before-ik = ~a~%"  (v- after-ik before-ik))
                )
              (return-end-coords)
              )
            (map float-vector #'(lambda (jl cja) (- (send jl :joint-angle) cja)) online-modified-jlist current-pose)
            ))
         (dp (instantiate float-vector (length *p*)))

         (dp-modified
          ;; qpで解くことによりdpを求める
          (let* ((bspline-element (car bspline)) ;; bsplineの中身が全部同じことを仮定してcarをとっている
                 (initial-state ;; dp C次 (ただしCはb(t-current)とb(t_hit)の共通する部分の個数)
                  (instantiate float-vector C) ;; 初期値は使われていないらしい
                  )
                 (b=0-element ;; 2行 C列
                  (matrix-append
                   (list (matrix (subseq (send bspline-element :calc-coeff-vector t-current) online-modified-min-id online-modified-max-id)) ;; b(t_current)のdpに関係する部分 C次
                         (matrix (subseq (send bspline-element :calc-delta-coeff-vector t-current :n 1) online-modified-min-id online-modified-max-id)) ;; db(t_current)のdpに関係する部分 C次
                         ;;(matrix (subseq (send bspline-element :calc-delta-coeff-vector x-hit :n 1) online-modified-min-id online-modified-max-id)) ;; db(t_hit)のdpに関係する部分 C次
                         )
                   '(1 0))
                  )
                 (b=dq-element ;; 1行 C列
                  (matrix (subseq (send bspline-element :calc-coeff-vector x-hit) online-modified-min-id online-modified-max-id))) ;; b(t_hit)のdpに関係する部分 C次

                 (equality-matrix ;; 2行 C列 ;; 3行C列では
                  (matrix-append
                   (list b=0-element b=dq-element)
                   '(1 0))
                  )

                 (inequality-matrix ;; (M-1)行 C列 (速度の条件は(- *id-max* 1)個)
                  (let* ((tmp-matrix (make-matrix (- *id-max* 1) C) ))
                    (mapc
                     #'(lambda (i j)
                         (setf (matrix-column tmp-matrix j)
                               (map float-vector #'deg2rad (subseq (matrix-column (send (car bspline) :calc-delta-matrix :n 1) i) 0 (- *id-max* 1))))
                         )
                     (my-range-2 online-modified-min-id (- online-modified-max-id 1)) (my-range online-modified-max-id)
                     )
                    tmp-matrix
                    )
                  )
                 (eval-weight-matrix ;; C行 C列
                  ;;(unit-matrix (length initial-state))
                  (progn
                    (diagonal (map float-vector #'(lambda (x) (+ (* 1 x) 1)) (my-range (length initial-state) ))) ;; 後ろの重みが大きい==>後ろが小さくなる
                    )
                  )
                 equality-coeff r inequality-min-vector inequality-max-vector
                 (tmp-dp-modified nil)
                 )
            (mapcar #'(lambda (j_k_id)
                        (setq equality-coeff
                              ;;(float-vector 0 0 0 (aref dq j_k_id))
                              ;;(float-vector 0 0 (aref dq j_k_id))
                              (float-vector 0 0 (aref dq j_k_id))
                              ) ;; 3次
                        (setq id (* (position (nth j_k_id online-modified-jlist) *jlist*) *id-max*))
                        (setq r (map float-vector #'deg2rad (subseq (transform (subseq *p* id (+ id *id-max*)) (send bspline-element :calc-delta-matrix :n 1)) 0 (- *id-max* 1)))) ;; (M-1)次
                        (setq inequality-min-vector ;; (M-1) 次
                              (v-
                               (coerce (make-list (- *id-max* 1) :initial-element (- (send (nth j_k_id online-modified-jlist) :max-joint-velocity))) float-vector)
                               r
                               ))
                        (setq inequality-max-vector ;; (M-1) 次
                              (v-
                               (coerce (make-list (- *id-max* 1) :initial-element (send (nth j_k_id online-modified-jlist) :max-joint-velocity)) float-vector)
                               r
                               ))


                        (setq tmp-dp-modified
                              (concatenate float-vector tmp-dp-modified
                                           (let ((ret
                                                  (solve-eiquadprog
                                                   ;;solve-qpoases-qp
                                                   :debug? debug?
                                                   :initial-state initial-state
                                                   :eval-weight-matrix eval-weight-matrix
                                                   ;;:state-min-vector (coerce (make-list (length initial-state) :initial-element -1.0) float-vector);;state-min-vector
                                                   ;;:state-max-vector (coerce (make-list (length initial-state) :initial-element 1.0) float-vector);;state-max-vector
                                                   :equality-vector equality-coeff
                                                   :equality-matrix equality-matrix
                                                   :inequality-matrix inequality-matrix
                                                   :inequality-min-vector inequality-min-vector
                                                   :inequality-max-vector inequality-max-vector
                                                   :eqthre 1e-5
                                                   )
                                                  ))
                                             (if ret
                                                 ret
                                               (instantiate float-vector (length initial-state))
                                               ))
                                           ))
                        )
                    (my-range K))
            tmp-dp-modified
            )
          );; end dp-modified

         )

    (when (= (length dp-modified) (* C K))
      (mapc
       #'(lambda (jvl dp-modified-id)
           (setq id (* (position jvl *jlist*) *id-max*))
           (replace dp dp-modified
                    :start1 (+ id online-modified-min-id) :end1 (+ id online-modified-max-id)
                    :start2 dp-modified-id :end2 (+ dp-modified-id C)
                    )
           )
       online-modified-jlist (mapcar #'(lambda (x) (* x C)) (my-range (length online-modified-jlist))) )
      );; end when
    dp
    )
  )

;; unused now
(defun evaluate-modify-qp (dp-list scale)
  (setq *modified-ss-list* nil)
  (let* ((seq-len (length dp-list))
         (p (copy-seq *p-orig*)))
    (send *robot* :move-to (make-coords) :world)
    (with-append-root-joint
     (link-list-with-robot-6dof *robot* (list (cdr (send *robot* :links)))
                                :joint-class 6dof-joint)
     (dotimes (i seq-len)
       (format t "p=~a~%" p)
       (format t "dp=~a~%" (nth i dp-list))
       (setq p (v+ p (nth i dp-list)))
       (show-optimized-motion p :specific-time (* scale 1.596))
       (push (send (send *hit-point* :copy-worldcoords) :worldpos) *modified-ss-list*)
       )
     )
    (setq *modified-ss-list* (reverse *modified-ss-list*))
    )
  )







#|
(progn
  (setq *p* (copy-seq *p-orig*))
  (send *robot* :move-to (make-coords) :world)
  (with-append-root-joint
   (link-list-with-robot-6dof *robot* (list (cdr (send *robot* :links)))
                              :joint-class 6dof-joint)
   (setq *dp* (calc-dp-from-dr #f(0 0 -0.1 0 0 0) 0.0 1.596))
   (show-optimized-motion *p* :specific-time 1.596)
   (setq a (send (send *hit-point* :copy-worldcoords) :worldpos))
   (show-optimized-motion (v+ *p* *dp*) :specific-time 1.596)
   (setq b (send (send *hit-point* :copy-worldcoords) :worldpos))
   (v- b a)
  ))

(setq *p-list* (list (copy-seq *p-orig*)))
(mapc #'(lambda (x) (setq *p-list* (append *p-list* (list (v+ (car (last *p-list*)) x)))) ) *dp-list*)
|#
