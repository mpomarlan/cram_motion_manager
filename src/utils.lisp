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

(defun eef-link-name (side)
  (cut:var-value '?link
                 (first (prolog:prolog `(and (cram-robot-interfaces:robot ?robot)
                                             (cram-robot-interfaces:end-effector-link ?robot ,side ?link))))))

(defun planning-group-name (side)
  (cut:var-value '?group
                 (first (prolog:prolog `(and (cram-robot-interfaces:robot ?robot)
                                             (cram-robot-interfaces:planning-group ?robot ,side ?group))))))

(defun arm-link-names (side)
  (cut:var-value '?links
                 (first (prolog:prolog
                          `(and (cram-robot-interfaces:robot ?robot)
                                (cram-robot-interfaces:arm-links ?robot ,side ?links))))))

(defun hand-link-names (side)
  (cut:var-value '?links
                 (first (prolog:prolog
                          `(and (cram-robot-interfaces:robot ?robot)
                                (cram-robot-interfaces:hand-links ?robot ,side ?links))))))

(defun object-names-in-hand (side)
  (let* ((objects-in-hand
           (cut:lazy-mapcar (lambda (bdgs)
                              (cut:with-vars-bound (?o) bdgs
                                    (desig:current-desig ?o)))
                            (prolog:prolog `(cram-plan-occasions-events:object-in-hand
                                               ?o ,side))))
        (object-names-in-hand
          (cut:force-ll
                (cut:lazy-mapcar (lambda (object)
                                   (string-upcase (desig:desig-prop-value object :name)))
                                 objects-in-hand))))
    object-names-in-hand))

