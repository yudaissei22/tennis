(defun fullbody-ik-jaxon
  (&key (stop 100)
        (debug-view :no-message)
        (centroid (send (send *robot* :foot-midcoords) :worldpos))
        (centroid-thre 20.0);;
        (rarm-target (send *robot* :rarm :end-coords :copy-worldcoords))
        (lleg-target (send *robot* :lleg :end-coords :copy-worldcoords))
        (rleg-target (send *robot* :rleg :end-coords :copy-worldcoords))
        (translation-axis (list t t t)) ;; rleg, lleg, rarm
        (rotation-axis    (list t t t)) ;; rleg, lleg, rarm
        (thre  (list 0.1 0.1 0.1))    ;; rleg, lleg, rarm
        (rthre (list 0.0017453 0.0017453 0.0017453)) ;; rleg, lleg, rarm
        (root-link-virtual-joint-weight ;; root movement weight
         (float-vector 0.1 0.1 0.1 0.05 0.2 0.2)) ;; x,y,z,roll,pitch,yaw
        (min (float-vector -500 -500 -500 -20 -20 -10)) ;; root movement limit, x,y,z,roll,pitch,yaw
        (max (float-vector  500  500  500  20  20  10)) ;; root movement limit, x,y,z,roll,pitch,yaw
        (use-collision-avoidance nil)
        (avoid-collision-null-gain 40)
        (avoid-collision-joint-gain 0.1)
        (collision-avoidance-pair
         (list
          (list (send *robot* :larm_link2_lk)
                (send *robot* :chest_link2_lk))
          (list (send *robot* :larm_link2_lk)
                (send *robot* :chest_link1_lk))
          (list (send *robot* :rarm_link2_lk)
                (send *robot* :chest_link2_lk))
          (list (send *robot* :rarm_link2_lk)
                (send *robot* :chest_link1_lk))))
        (use-waist-r nil)
        (use-waist-y nil)
        (revert-if-fail t)
        (move-target-legs
         (list (send *robot* :rleg :end-coords)
               (send *robot* :lleg :end-coords)))
        (move-target-arms
         (list (send *robot* :rarm :end-coords))))

  (let* ((move-target (append move-target-legs move-target-arms))
         (target-coords
          (list
           rleg-target lleg-target
           rarm-target))

         (additional-weight-list ;; jaxon specific
          (list
           (if (not use-waist-r)
               (list (send *robot* :torso :waist-r :child-link) 0)
             (if (numberp use-waist-r)
                 (list (send *robot* :torso :waist-r :child-link) use-waist-r)))
           (if (not use-waist-y)
               (list (send *robot* :torso :waist-y :child-link) 0)
             (if (numberp use-waist-y)
                 (list (send *robot* :torso :waist-y :child-link) use-waist-y)))))

         (link-list))

    (setq additional-weight-list (remove nil additional-weight-list));;

    ;;link-list
    (setq link-list
          (mapcar #'(lambda (limb)
                      (send *robot* :link-list (send limb :parent))) move-target))

    (let (args)
      (when use-collision-avoidance
        (push :avoid-collision-null-gain args)
        (push avoid-collision-null-gain args)
        (push :avoid-collision-joint-gain args)
        (push avoid-collision-joint-gain args)
        (push :collision-avoidance-link-pair args)
        (push collision-avoidance-pair args)
        (setq args (nreverse args)))

      (send* *robot* :fullbody-inverse-kinematics target-coords
             :move-target move-target ;; move-targetをtarget-coordsに移動する
             :link-list link-list ;; listのlist型
             :translation-axis translation-axis
             :rotation-axis    rotation-axis
             :target-centroid-pos centroid ;; *robot*の目標重心位置
             :centroid-thre centroid-thre ;; 収束
             :thre thre ;; end-coordsのpos収束
             :rthre rthre ;; 姿勢収束
             :min min
             :max max
             :additional-weight-list additional-weight-list
             :root-link-virtual-joint-weight root-link-virtual-joint-weight
             :debug-view debug-view ;;:no-message
             :stop stop ;; 計算回数
             :revert-if-fail revert-if-fail
             :dump-command nil
             :warnp nil
             args))))
(defun fullbody-ik-jaxon-with-target (target)
  (fullbody-ik-jaxon
   :rarm-target target ;; ikのtarget-coords (ラケットの打点をどの姿勢に持っていくか)
   :rotation-axis (list t t :x) ;; x軸周りの回転自由度を無視する
   :translation-axis (list t t t)
   :move-target-arms (list (send *robot* :get :racket-target)) ;; ラケットの打点(姿勢考慮)オブジェクト(座標ではなくどの部分を動かしてikを解くか)
                           ;; (send *robot* :larm :end-coords))
   :use-waist-r nil
   :use-waist-y t
   :root-link-virtual-joint-weight ;; root movement weight
   (float-vector 0.1 0.1 0.03 0.0 0.1 0.0))) ;; x,y,z,roll,pitch,yaw
