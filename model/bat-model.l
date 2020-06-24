;; bat-model.l
(defclass bat
  :super cascaded-link
  :slots ()
  )

(defmethod bat
  (:init ()
   (send-super :init)
   (let ((bds (send self :make-bat)))
     (setq l0 (instance bodyset-link :init (make-cascoords)
                        :bodies bds)))
   (setq links (list l0))
   (send self :assoc (elt links 0))
   (send self :init-ending)
   (send self :name "bat")
   )

  (:make-bat ()
     (let (bat)
       (setq bat (make-solid-of-revolution (list #f(30 0 750) #f(30 0 450) #f(15 0 250) #f(15 0 21) #f(25 0 18) #f(25 0 0))))
       (send bat :rotate (deg2rad -90) :x)
       (send bat :locate #f(0 -520 15) :world)
       (send bat :set-color #f(0.753 0.753 0.753))
       (list bat)
       )
     )
  )

#|
(setq a (instance bat :init))
(setq gl-bodies (mapcar #'(lambda (x) (gl::make-glvertices-from-faceset x)) (send a :bodies)))
(gl::write-wrl-from-glvertices "bat.wrl" (car gl-bodies) :scale 0.001)

;;
(setq cds
      (send
       (send (send (send *jaxon_red* :larm_link7_lk) :worldcoords)
             :transformation (send *jaxon_red* :larm :end-coords :worldcoords))
       :transformation
       (send (send *bat-lhand-grip* :worldcoords) :transformation (send *bat-model* :worldcoords))))
(setq ret (rotation-angle (send cds :worldrot)))

(progn
  (format t "translation ~12,12F ~12,12F ~12,12F~%" (/ (elt (send cds :worldpos) 0) 1000) (/ (elt (send cds :worldpos) 1) 1000) (/ (elt (send cds :worldpos) 2) 1000))
  (format t "rotation ~12,12F ~12,12F ~12,12F ~12,12F~%" (elt (cadr ret) 0) (elt (cadr ret) 1) (elt (cadr ret) 2) (car ret))
  )
|#
