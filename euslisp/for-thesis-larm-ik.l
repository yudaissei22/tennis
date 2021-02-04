#!/usr/bin/env roseus

(require "package://hrpsys_ros_bridge_tutorials/euslisp/jaxon_red-interface.l")
(require "models/arrow-object.l")

(jaxon_red)
(setq *robot* *jaxon_red*)
(setq *robot-cp* (copy-object *robot*))
(setq *arrow* (arrow :pos (send (send *robot* :larm :end-coords) :worldpos) :rot (send (send *robot* :larm :end-coords) :worldrot)))

(setq *arrow-cp* (copy-object *arrow*))

(send *robot* :larm :inverse-kinematics (make-coords :pos #f(300 300 1200)))
(setq *arrow* (arrow :pos (send (send *robot* :larm :end-coords) :worldpos) :rot (send (send *robot* :larm :end-coords) :worldrot)))

(objects (list *robot-cp* *arrow-cp* *robot* *arrow*))

(read-line)

(send *robot* :reset-pose)

(send *robot* :larm-shoulder-r :joint-angle 90)
(send *robot* :lleg-crotch-r :joint-angle 40)
(objects (list *robot-cp* *robot*))
