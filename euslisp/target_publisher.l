#!/usr/bin/env roseus

(ros::roseus-add-msgs "ball_state_msgs")
(defun target_publisher ()
  (ros::roseus "target_publisher")
  (ros::advertise "/pointgrey/estimated_ball_state"
                  ball_state_msgs::PosAndVelWithCovarianceStamped)

  (let (x y z vx vy vz rate msg)

   (setq x 6.0)
   (setq y 0.0)
   (setq z 0.0)
   (setq vx -10.0)
   (setq vy -0.3)
   (setq vz 5.3)
   (setq rate 100.0)
   (ros::rate rate)
   (while (>= z 0)

     (ros::spin-once)

     (setq x (+ x (/ vx rate)))
     (setq y (+ y (/ vy rate)))
     (setq z (+ z (/ vz rate) (/ (/ (/ -9.8 rate) rate) 2.0)))
     (setq vz (+ vz (/ -9.8 rate)))

     (setq msg (instance ball_state_msgs::PosAndVelWithCovarianceStamped :init))
     (send msg :header :frame_id "map")
     (send msg :header :stamp (ros::time-now))
     (send msg :point (instance geometry_msgs::Point :init :x x :y y :z z))
     (send msg :velocity (instance geometry_msgs::Vector3 :init :x vx :y vy :z vz))
     (send msg :pos_and_vel_covariance
           #f(1 0 0 0 0 0
              0 1 0 0 0 0
              0 0 1 0 0 0
              0 0 0 1 0 0
              0 0 0 0 1 0
              0 0 0 0 0 1))

     (print "target")
     (format t "x: ~A   y: ~A   z: ~A~%" x y z)

     (ros::publish "/pointgrey/estimated_ball_state" msg)

     (ros::sleep))));; 10ms


(target_publisher)
