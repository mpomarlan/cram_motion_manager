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

(defclass ik-check-goal-specification (manipulation-goal-specification)
  ())

(defmethod make-goal-specification ((type (eql :ik-check-goal-specification)) &rest args)
  (apply #'make-instance (cons 'ik-check-goal-specification args)))

(defmethod make-goal-specification ((type (eql 'ik-check-goal-specification)) &rest args)
  (apply #'make-instance (cons 'ik-check-goal-specification args)))

(defun ik-checker (arm-goal allowed-collision-objects highlight-links)
  (let* ((side (side arm-goal))
         (link-name (or (arm-link arm-goal) (eef-link-name side)))
         (planning-group (planning-group-name side))
         (poses-to-check (poses arm-goal))
         (touch-links (arm-link-names side))
         ;; TODO: mpomarlan: might be nicer to know which pose fails
         (all-ok (moveit:plan-link-movements link-name planning-group poses-to-check :destination-validity-only t 
                                             :allowed-collision-objects allowed-collision-objects
                                             :highlight-links highlight-links
                                             :touch-links touch-links))
         (error-object nil)
         (error-message (if all-ok nil "One of the poses is out of reach.")))
    (make-instance 'manipulation-result
                   :all-ok all-ok
                   :error-object error-object
                   :error-message error-message)))

(defun combine-ik-checker-results (results arm-goals)
  ;; TODO: mpomarlan: might be nicer to know which pose, for which arm, fails
  (declare (ignore arm-goals))
  (let* ((all-ok (and (mapcar #'all-ok results)))
         (error-object nil)
         (error-message (if all-ok nil "One of the poses is out of reach.")))
    (make-instance 'manipulation-result
                   :all-ok all-ok
                   :error-object error-object
                   :error-message error-message)))

(defmethod execute-arm-action ((goal-specification ik-check-goal-specification))
  ;; Always in plan-only mode, so will not actually move the arms.
  ;; Never falls-back on someone else; either a pose is reachable, or it's not.
  ;; Does not generate a trajectory output yet. TODO: fix this.
  (let* ((arm-goals (arm-pose-goals goal-specification))
         (keys (keys goal-specification))
         (allowed-collision-objects (cadr (assoc :allowed-collision-objects keys)))
         (highlight-links (cadr (assoc :highlight-links keys)))
         (results (mapcar (lambda (arm-goal)
                            (ik-checker arm-goal allowed-collision-objects highlight-links))
                          arm-goals)))
    (combine-ik-checker-results results arm-goals)))

