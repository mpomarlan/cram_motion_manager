;;; Copyright (c) 2016, Mihai Pomarlan <blandc@informatik.uni-bremen.de>
;;; All rights reserved.
;;; 
;;; Redistribution and use in source and binary forms, with or without
;;; modification, are permitted provided that the following conditions are met:
;;; 
;;;     * Redistributions of source code must retain the above copyright
;;;       notice, this list of conditions and the following disclaimer.
;;;     * Redistributions in binary form must reproduce the above copyright
;;;       notice, this list of conditions and the following disclaimer in the
;;;       documentation and/or other materials provided with the distribution.
;;;     * Neither the name of the Intelligent Autonomous Systems Group/
;;;       Universitaet Bremen nor the names of its contributors may be used to
;;;       endorse or promote products derived from this software without
;;;       specific prior written permission.
;;; 
;;; THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
;;; AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
;;; IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
;;; ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
;;; LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
;;; CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
;;; SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
;;; INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
;;; CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
;;; ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
;;; POSSIBILITY OF SUCH DAMAGE.

(in-package :mot-man)

(cut:define-hook cram-language::on-prepare-move-arm (side pose-stamped planning-group ignore-collisions))
(cut:define-hook cram-language::on-finish-move-arm (log-id success))

(defclass arm-goal-specification ()
  ((side
     :initform nil
     :initarg :side
     :accessor side
     :type (or null keyword)
     :documentation "Which arm this refers to. Typically one of :left or :right")
   (arm-link
     :initform nil
     :initarg :arm-link
     :accessor arm-link
     :type (string)
     :documentation "Which arm link the goal refers to. Usually can be left blank as there is a sensible default (eef link); set to some other link when needed.")
   (poses
     :initform nil
     :initarg :poses
     :accessor poses
     :type (list cl-transforms-stamped:pose-stamped)
     :documentation "List of waypoints (stamped poses) to bring the arm-link to.")))

(defclass manipulation-goal-specification ()
  ((arm-pose-goals
      :initform nil
      :initarg :arm-pose-goals
      :accessor arm-pose-goals
      :type (or null list)
      :documentation "A list of objects representing poses that the robot's arms should reach")
   (plan-only
      :initform nil
      :initarg :plan-only
      :accessor plan-only
      :type (boolean)
      :documentation "Indicates whether the manipulation should only be planned and not executed. Default is nil (which executes the motion).")
   (keys
      :initform nil
      :initarg :keys
      :accessor keys
      :type (or null list)
      :documentation "An alist. Plonk any extra data, such as constraints or parameter hints, here. TODO: get this more structured, maybe.")
   (fallback-converter
      :initform nil
      :initarg :fallback-converter
      :accessor fallback-converter
      :type (or null (list (function (manipulation-goal-specification) manipulation-goal-specification)))
      :documentation "List of functions to obtain the goal specification (and any other key parameters) for a fallback controller, in case the one selected by this goal specification fails.")))

(defun list->arm-goal-spec (list-arm-goals)
  (let* ((llen (length list-arm-goals))
         (side (first list-arm-goals))
         (arm-link (or (if (< 2 llen)
                         (second list-arm-goals)
                         nil)
                       (eef-link-name side)))
         (poses (if (< 2 llen)
                  (third list-arm-goals)
                  (second list-arm-goals)))
         (poses (if (typep poses 'list)
                  poses
                  (list poses))))
    (make-instance 'arm-goal-specification
                   :side side
                   :arm-link arm-link
                   :poses poses)))

(defun arm-goal-cleanup (arm-goal)
  (let* ((side (side arm-goal))
         (arm-link (or (arm-link arm-goal)
                       (eef-link-name side)))
         (poses (poses arm-goal))
         (poses (if (typep poses 'list)
                  poses
                  (list poses))))
    (make-instance 'arm-goal-specification
                   :side side
                   :arm-link arm-link
                   :poses poses)))

(defmethod initialize-instance :after ((goal-spec manipulation-goal-specification) &key)
  (let* ((proper-arm-pose-goals (mapcar (lambda (arm-goal)
                                          (if (eql (type-of arm-goal) 'arm-goal-specification)
                                            (arm-goal-cleanup arm-goal)
                                            (list->arm-goal-spec arm-goal)))
                                        (arm-pose-goals goal-spec))))
    (setf (arm-pose-goals goal-spec)
          proper-arm-pose-goals)))

(defclass manipulation-result ()
  ((all-ok
     :initform nil
     :initarg :all-ok
     :reader all-ok
     :documentation "A convenience boolean flag to test if the manipulation goal succeeded.")
   (trajectories
     :initform nil
     :initarg :trajectories
     :reader trajectories
     :documentation "A list of trajectory objects.")
   (error-object
     :initform nil
     :initarg :error-object
     :reader error-object
     :documentation "Error data. Put whatever you think relevant here, in a format easy for the MACHINE to read.")
   (error-message
     :initform nil
     :initarg :error-message
     :reader error-message
     :documentation "Error data. Put whatever you think relevant here, in a format easy for YOU to read.")))

(defun copy-goal-specification (goal-specification &optional (new-type nil))
  (make-instance (or new-type (type-of goal-specification))
                 :arm-pose-goals (arm-pose-goals goal-specification)
                 :plan-only (plan-only goal-specification)
                 :keys (keys goal-specification)
                 :fallback-converter (if (not new-type)
                                       (fallback-converter goal-specification)
                                       (cdr (fallback-converter goal-specification)))))

(defun add-goal-spec-keys (goal-spec keys)
  "Updates the keys slot of goal-spec. Overwrites values associated to keys already present in the goal spec.
   Special handling for :plan-only which is put in its own separate slot."
  (mapcar (lambda (key-val)
            (let* ((key (car key-val))
                   (val (cdr key-val)))
              (if (equal key :plan-only)
                (setf (plan-only goal-spec) (car val))
                (if (assoc key (keys goal-spec))
                  (rplacd (assoc key (keys goal-spec))
                          val)
                  (setf (keys goal-spec) 
                        (cons key-val
                              (keys goal-spec)))))))
          keys))

(defun add-goal-spec-arm-goals (goal-spec arm-pose-goals)
  "Updates the arm-pose-goals slot of goal-spec. Overwrites goals already present for an arm.
   arm-pose-goals is expected to be a list of arm pose goals, which are lists having one of the following forms:
     (arm pose)
     (arm pose-list)
     (arm link-name pose)
     (arm link-name pose-list)
   After this function is done, arm-pose-goals inside the goal-spec will always contain arm pose goals of the form
     (arm link-name pose-list)"
  (let* ((arm-pose-goals (mapcar (lambda (arm-pose-goal)
                                   (let* ((arm (first arm-pose-goal))
                                          (link-name (if (third arm-pose-goal)
                                                       (second arm-pose-goal)
                                                       (eef-link-name arm)))
                                          (pose-list (if (third arm-pose-goal)
                                                       (third arm-pose-goal)
                                                       (second arm-pose-goal)))
                                          (pose-list (if (listp pose-list)
                                                       pose-list
                                                       (list pose-list))))
                                     (list (list arm link-name) pose-list)))
                                 arm-pose-goals))
         (gs-pose-goals (arm-pose-goals goal-spec))
         (gs-pose-goals (mapcar (lambda (arm-pose-goal)
                                  (let* ((arm (side arm-pose-goal))
                                         (link-name (if (arm-link arm-pose-goal)
                                                        (arm-link arm-pose-goal)
                                                        (eef-link-name arm)))
                                         (pose-list (poses arm-pose-goal)))
                                    (list (list arm link-name) pose-list)))
                                gs-pose-goals))
         (dummy (loop for new-arm-goal in arm-pose-goals do
                  (let* ((key (first new-arm-goal))
                         (val (cdr new-arm-goal)))
                    (if (assoc key gs-pose-goals :test #'equal)
                      (rplacd (assoc key gs-pose-goals :test #'equal)
                              val)
                      (setf gs-pose-goals
                            (cons new-arm-goal
                                  gs-pose-goals))))))
         (gs-pose-goals (mapcar (lambda (arm-pose-goal)
                                  (make-instance 'arm-goal-specification
                                                 :side (first (first arm-pose-goal))
                                                 :arm-link (second (first arm-pose-goal))
                                                 :poses (second arm-pose-goal)))
                                gs-pose-goals)))
    (declare (ignore dummy))
    (setf (arm-pose-goals goal-spec)
          gs-pose-goals)))

(defun enriched-goal-specification (goal-specification &key keys arm-pose-goals)
  (let* ((new-goal-specification (copy-goal-specification goal-specification)))
    (add-goal-spec-keys new-goal-specification keys)
    (add-goal-spec-arm-goals new-goal-specification arm-pose-goals)
    new-goal-specification))

(defgeneric execute-arm-action (goal-specification)
  (:documentation "Select a motion controller and send the goal to it."))

(defgeneric make-goal-specification (type &rest args)
  (:documentation "Convenience function to make a goal-spec."))

(defgeneric make-fallback-converter (type)
  (:documentation "Convenience function to make a function object representing a fallback converter: takes a goal-spec and converts its type to TYPE."))

(defmethod make-fallback-converter ((type t))
  (cpl-impl:fail 'cram-plan-failures:manipulation-failed
                 :format-control "Manipulation failed: requested creation of unrecognized fallback-converter type"))

(defmethod make-fallback-converter ((type (eql :manipulation-goal-specification)))
  (cpl-impl:fail 'cram-plan-failures:manipulation-failed
                 :format-control "Manipulation failed: requested creation of generic fallback-converter to MANIPULATION-GOAL-SPECIFICATION is not allowed."))

(defmethod make-goal-specification ((type t) &rest args)
  (declare (ignore args))
  (cpl-impl:fail 'cram-plan-failures:manipulation-failed
                 :format-control "Manipulation failed: requested creation of unrecognized goal-spec type."))

(defmethod execute-arm-action ((goal-specification t))
  (declare (ignore goal-specification))
  (cpl-impl:fail 'cram-plan-failures:manipulation-failed
                 :format-control "Manipulation failed: motion-manager received something that is not a goal-spec."))

(defmethod execute-arm-action ((goal-specification manipulation-goal-specification))
  (declare (ignore goal-specification))
  (cpl-impl:fail 'cram-plan-failures:manipulation-failed
                 :format-control "Manipulation failed: motion-manager received a generic goal-spec; make sure you have motion generators (e.g. moveit or giskard) loaded, and send an appropriate goal-spec."))

(defun call-fallback (goal-specification)
  (if (car (fallback-converter goal-specification))
    (execute-arm-action (funcall (car (fallback-converter goal-specification)) goal-specification))
    (cpl-impl:fail 'cram-plan-failures:manipulation-failure
                   :format-control "Fallbacks exhausted.")))

