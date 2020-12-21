(require "package://tennis/euslisp/motion-choice.l")
(defun choose-motion (&key (motion-choice *motion-choice*))
 (cond
  ((equal motion-choice "forehand")
   (load "package://tennis/config/opt-log/forehand.l")
   )

  ;; {{{ punch motion
  ((equal motion-choice "punch")
   (load "package://tennis/config/opt-log/punch.l")
   )
   ;; }}}

  ;; {{{ kick motion
  ((equal motion-choice "kick")
   (load "package://tennis/config/opt-log/kick.l")
   )
  ;; }}}

  ;; {{{ smash motion

  ((equal motion-choice "smash")
   (load "package://tennis/config/opt-log/smash.l")
   )
  ;; }}}

  ;; {{{ batting motion
  ((equal motion-choice "batting")
   (load "package://tennis/config/opt-log/batting.l")
   )
  ;; }}}

  ;; {{{ forehand-step motion
  ((equal motion-choice "forehand-step")
   (load "package://tennis/config/opt-log/forehand-step.l")
  )
  ;; }}}

  ;; {{{ forehand-volley-step motion
  ((equal motion-choice "forehand-volley-step")
   (load "package://tennis/config/opt-log/forehand-volley-step.l")
  )
  ;; }}}

  ;; {{{ backhand-volley-step motion
  ((equal motion-choice "backhand-volley-step")
   (load "package://tennis/config/opt-log/backhand-volley-step.l")
  )
  ;; }}}

  ;; {{{ kick-step motion
  ((equal motion-choice "kick-step")
   (load "package://tennis/config/opt-log/kick-step.l")
   )
  ;; }}}

  ;; {{{ batting-step motion
  ((equal motion-choice "batting-step")
   (load "package://tennis/config/opt-log/batting-step.l")
   )
  ;; }}}

  ))

(choose-motion :motion-choice *motion-choice*)
