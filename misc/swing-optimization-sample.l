 ;; $ roscd nlopt
 ;; $ make
 ;; $ roscd
 ;; $ cd lib
 ;; $ ldd libnlopt_wrapper.so
 ;; $ ln -s libnlopt_cxx.so libnlopt_cxx.so.0

(require "package://eus_nlopt/euslisp/nlopt.l")

(require "package://hrpsys_ros_bridge_tutorials/models/samplerobot.l")

(defvar *robot* (samplerobot))

(objects (list *robot*))

(mapcar
 #'(lambda (j)
     (if (< (send j :min-angle) -180) (send j :min-angle -180))
     (if (> (send j :max-angle) +180) (send j :max-angle +180)))
 (send *robot* :joint-list))

;; リファレンス姿勢
(defvar *ref-postures*
  (apply
   'concatenate
   (cons
    float-vector
    (mapcar
     #'(lambda (id)
         (map float-vector
              '(lambda (j) (+ (send j :min-angle)
                              (* (random 1.0)
                                 (- (send j :max-angle)
                                    (send j :min-angle)))))
              (send *robot* :joint-list)))
     '(0 1 2)))))

;; 探索で出てくる答え次元
(defvar *output-postures*
   (instantiate float-vector
                (* 5 (length (send *robot* :joint-list)))))
(defvar *output-postures-min*
  (apply
   'concatenate
   (cons
    float-vector
    (mapcar #'(lambda (id) (send-all (send *robot* :joint-list) :min-angle))
            (make-list 5)))))
(defvar *output-postures-max*
   (apply
    'concatenate
    (cons
     float-vector
     (mapcar #'(lambda (id) (send-all (send *robot* :joint-list) :max-angle))
             (make-list 5)))))

;; リファレンス姿勢が *output-postures* の何番目か?
(defvar *ref-id* '(0 2 4))

(defun calc-velocity
  (v1 &optional (v2 (copy-seq v1)))
  (dotimes (i (- (/ (length v1) (length (send *robot* :joint-list))) 1))
    (dotimes (j (length (send *robot* :joint-list)))
      (setf (aref v2 (+ j (* i (length (send *robot* :joint-list)))))
            (- (aref v1 (+ j (* (+ 1 i) (length (send *robot* :joint-list)))))
               (aref v1 (+ j (* (+ 0 i) (length (send *robot* :joint-list)))))))))
  v2)

(defvar *target-maximize-velocity-posture-id* 2)
(defvar *v2-buf*)

;; swing
(defun-c-callable swing-f ((v1 :string) (v2 :string)) :integer
  (if (null *v2-buf*) (setq *v2-buf* (copy-seq v1)))
  (calc-velocity v1 *v2-buf*)
  (setf (aref v2 0) 0)
  (dotimes (i (length (send *robot* :joint-list)))
    (let* ((vel (aref *v2-buf* (+ i (* *target-maximize-velocity-posture-id*
                                       (length (send *robot* :joint-list)))))))
      (setf (aref v2 0)
            (+ (aref v2 0) (* vel vel)))))
  (setf (aref v2 0) (* -1 (aref v2 0)))
  0)

(defun-c-callable swing-df ((v1 :string) (v2 :string)) :integer
  ;; impl
  (simple-jacobian v1 v2 'swing-f)
  0)

;; *ref-id* の姿勢が *ref-postures* に等しい．
(defun-c-callable swing-g ((v1 :string) (v2 :string)) :integer
  (setf (aref v2 0) 0)
  (dolist (id *ref-id*)
    (dotimes (i (length (send *robot* :joint-list)))
      (let* (;; (pos (+ i (* id (length (send *robot* :joint-list)))))
             (q1 (aref v1 (+ i (* id (length (send *robot* :joint-list))))))
             (q2 (aref *ref-postures*
                       (+ i (* (position id *ref-id*)
                               (length (send *robot* :joint-list)))))))
        (setf (aref v2 0)
              (+ (aref v2 0) (expt (- q1 q2) 2))))))
  0)

(defun-c-callable swing-dg ((v1 :string) (v2 :string)) :integer
  ;; impl
  (simple-jacobian v1 v2 'swing-g)
  0)

(defun-c-callable swing-h ((v1 :string) (v2 :string)) :integer
  ;; impl
  0)

(defun-c-callable swing-dh ((v1 :string) (v2 :string)) :integer
  ;; impl
  0)

(defun swing-optimize
  nil
  (nlopt-optimize
   :initial-state *output-postures*
   :state-min-vector *output-postures-min*
   :state-max-vector *output-postures-max*
   :state-dimension (length *output-postures*)
   :evaluation-function 'swing-f
   :evaluation-function-gradient 'swing-df
   :equality-function 'swing-g
   :equality-function-gradient 'swing-dg
   :inequality-function 'swing-h
   :inequality-function-gradient 'swing-dh
   :equality-dimension 1
   :inequality-dimension 0
   ;;
   :alg SLSQP ;; COBYLA ;; DIRECT
   :max-time (* 10 60 60)
   :max-eval 10000
   :eqthre 1
   :ftol 1e-10
   :xtol 1e-10
   )
  )

(defun show-postures
  (v)
  (dotimes (i (/ (length v) (length (send *robot* :joint-list))))
    (map cons
     #'(lambda (j av) (send j :joint-angle av))
     (send *robot* :joint-list)
     (subseq v (* i (length (send *robot* :joint-list)))
             (* (+ i 1) (length (send *robot* :joint-list)))))
    (send-all (send *robot* :links) :worldcoords)
    (send *viewer* :draw-objects)
    (read-line)))

;; もう少しoutputを増やしてこのあとラグランジュ補間などで適当に補間する