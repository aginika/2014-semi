#!/usr/bin/env roseus
;;
(ros::roseus "robot_main")

(ros::load-ros-manifest "jsk_recognition_msgs")
(load "package://jsk_2014_picking_challenge/euslisp/utils.l")
(require :state-machine "package://roseus_smach/src/state-machine.l")
(require :state-machine-ros "package://roseus_smach/src/state-machine-ros.l")
(require :state-mcahine-utils "package://roseus_smach/src/state-machine-utils.l")
(require :state-machine-actionlib "package://roseus_smach/src/state-machine-actionlib.l")

(ros::load-ros-manifest "jsk_recognition_msgs")
(ros::load-ros-manifest "jsk_2014_picking_challenge")


;;Global Variables

;; end of Global Variables

;; (setq client (instance ros::simple-action-client :init
;; 		  "/picking_server/rarm" jsk_2014_picking_challenge::EusCommandAction))
;; (ros::ros-info "waiting ~A available" "picking_server/rarm")
;; (send client :wait-for-server)

;; (setq goal (instance jsk_2014_picking_challenge::EusCommandActionGoal :init))
;; (send goal :header :stamp (ros::time-now))
;; (send goal :goal_id :stamp (ros::time-now))
;; (send goal :goal :command  "move_to_target")
;; (send goal :goal :target_bin "c")
;; (send client :send-goal goal))

;; (send client :wait-for-result)

(defun create-euscommand-goal(command target-bin)
  (let ((goal (instance jsk_2014_picking_challenge::EusCommandActionGoal :init)))
    (send goal :header :stamp (ros::time-now))
    (send goal :goal_id :stamp (ros::time-now))
    (send goal :goal :command command)
    (send goal :goal :target_bin target-bin)
    goal
    ))

(defun  init-pose(target-object)
  (create-euscommand-goal "init_pose" "")
  )

(defun  move-to-target(target-object)
  (create-euscommand-goal "move_to_target" target-object)
  )

(defun  pick-1-object(target-object)
  (create-euscommand-goal "pick_object" target-object)
  )

(defun  wait-place-object(target-object)
  (create-euscommand-goal "wait_place_object" "")
  )

(defun  place-object(target-object)
  (create-euscommand-goal "place_object" target-object)
  )

(defun  pick-object-from-multi(target-object)
  (create-euscommand-goal "pick_object" target-object)
  )

(defun  wait-verification(target-object)
  (create-euscommand-goal "wait_verification" target-object)
  )

(defun  move-to-verification(target-object)
  (create-euscommand-goal "move_to_verification" target-object)
  )

(defun  return-object(target-object)
  (create-euscommand-goal "return_object" target-object)
  )


(defclass robot-manip-controller
 :super propertied-object
 :slots (target-arm opposite-arm target-list target-object action-client))

(defmethod robot-manip-controller
  (:init (arm target-list-given)
	 (setq target-arm arm)
	 (setq opposite-arm (if (equal arm :larm) :rarm :larm))
	 (setq target-list target-list-given)
	 (warn (format nil "Arm is ~A" arm))
	 (setq action-client (instance ros::simple-action-client :init
				(format nil "/picking_server/~A" (symbol-to-str arm)) jsk_2014_picking_challenge::EusCommandAction))
	 (send action-client :wait-for-server)
	 )
  (:apc-setup ()
	      (ros::ros-info "APC-INIT")
	      t)
  (:check-target-list (&optional userdata)
		      (ros::ros-info "CHECK-TARGET-LIST")
		      (if (> (length target-list) 0)
			  (progn
			    (setq target-object (pop target-list))
			    (set-alist :target-object target-object userdata)
			    t)
			:end-picking
			))
  )

(defun make-parallel-state-machine ()
  ;; (setq *robot-arm-smach* (instance robot-manip-controller :init))
  (let ((rarm-client (instance ros::simple-action-client :init
			       "/picking_server/rarm" jsk_2014_picking_challenge::EusCommandAction))
	(larm-client (instance ros::simple-action-client :init
                               "/picking_server/larm" jsk_2014_picking_challenge::EusCommandAction))
	async-clients)
    
    (setq *robot-rarm-smach* (instance robot-manip-controller :init :rarm (list "a" "b")))
    (setq *robot-larm-smach* (instance robot-manip-controller :init :larm (list "g" "h")))
    (setq *sm*
	  (make-state-machine
	   '((:apc-setup -> (:rarm-check-target-list :larm-check-target-list))
	     (:rarm-move-to-target -> :rarm-pick-object-from-multi)
	     (:rarm-move-for-verification -> :rarm-wait-place-object)
	     (:rarm-wait-place-object -> :rarm-place-object)
	     (:rarm-return-object -> :rarm-pick-object-from-multi)
	     (:rarm-place-object -> :rarm-check-target-list)
	     (:rarm-pick-1-object -> :rarm-wait-place-object)
	     (:rarm-wait-verification -> :rarm-move-for-verification)

	     (:larm-move-to-target -> :larm-pick-object-from-multi)
	     (:larm-move-for-verification -> :larm-wait-place-object)
	     (:larm-wait-place-object -> :larm-place-object)
	     (:larm-return-object -> :larm-pick-object-from-multi)
	     (:larm-place-object -> :larm-check-target-list)
	     (:larm-pick-1-object -> :larm-wait-place-object)
	     (:larm-wait-verification -> :larm-move-for-verification)
	     )
	   '((:apc-setup '(lambda (&optional args) (send *robot-rarm-smach* :apc-setup)))
	     (:rarm-check-target-list '(lambda (&optional args) (send *robot-rarm-smach* :check-target-list args)))
	     (:larm-check-target-list '(lambda (&optional args) (send *robot-larm-smach* :check-target-list args)))
	     )
	   '(:apc-setup)
	   '(:success)
	   :exec-result :succeeded
	   )
	  )

    ;; NODES ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
    (send *sm* :add-node (actionlib-client-state
			  :rarm-init-pose rarm-client :timeout 300 :retry t
			  :key 'init-pose))
    (send *sm* :add-node (actionlib-client-state
			  :rarm-move-to-target rarm-client :async t :timeout 300 :retry t
			  :key 'move-to-target))
    (send *sm* :add-node (actionlib-client-state
			  :rarm-pick-object-from-multi rarm-client :async t :timeout 300 :retry t
			  :key 'pick-object-from-multi))
    (send *sm* :add-node (actionlib-client-state
			  :rarm-move-for-verification rarm-client :async t :timeout 300 :retry t
			  :key 'move-for-verification))
    (send *sm* :add-node (actionlib-client-state
			  :rarm-return-object rarm-client :async t :timeout 300 :retry t
			  :key 'return-object))
    (send *sm* :add-node (actionlib-client-state
			  :rarm-place-object rarm-client :async t :timeout 300 :retry t
			  :key 'place-object))
    (send *sm* :add-node (actionlib-client-state
			  :rarm-wait-verification rarm-client :async t :timeout 300 :retry t
			  :key 'wait-verification))
    (send *sm* :add-node (actionlib-client-state
			  :rarm-wait-place-object rarm-client :async t :timeout 300 :retry t
			  :key 'wait-place-object))
    (send *sm* :add-node (actionlib-client-state
			  :rarm-pick-1-object rarm-client :async t :timeout 300 :retry t
			  :key 'pick-1-object))


    (send *sm* :add-node (actionlib-client-state
			  :larm-init-pose larm-client :timeout 300 :retry t
			  :key 'init-pose))
    (send *sm* :add-node (actionlib-client-state
			  :larm-move-to-target larm-client :async t :timeout 300 :retry t
			  :key 'move-to-target))
    (send *sm* :add-node (actionlib-client-state
			  :larm-pick-object-from-multi larm-client :async t :timeout 300 :retry t
			  :key 'pick-object-from-multi))
    (send *sm* :add-node (actionlib-client-state
			  :larm-move-for-verification larm-client :async t :timeout 300 :retry t
			  :key 'move-for-verification))
    (send *sm* :add-node (actionlib-client-state
			  :larm-return-object larm-client :async t :timeout 300 :retry t
			  :key 'return-object))
    (send *sm* :add-node (actionlib-client-state
			  :larm-place-object larm-client :async t :timeout 300 :retry t
			  :key 'place-object))
    (send *sm* :add-node (actionlib-client-state
			  :larm-wait-verification larm-client :async t :timeout 300 :retry t
			  :key 'wait-verification))
    (send *sm* :add-node (actionlib-client-state
			  :larm-wait-place-object larm-client :async t :timeout 300 :retry t
			  :key 'wait-place-object))
    (send *sm* :add-node (actionlib-client-state
			  :larm-pick-1-object larm-client :async t :timeout 300 :retry t
			  :key 'pick-1-object))


    (send *sm* :add-node
          (instance async-join-state :init
                    :join :timeout 30
                    :remap-list '((:async . async-clients))))
    (ros::ros-info "ADD NODE DONE")


    ;;remap-list series
    (send (send *sm* :node :rarm-check-target-list) :remap-list '((:target-object . target-object)))
    (send (send *sm* :node :larm-check-target-list) :remap-list '((:target-object . target-object)))

    (send (send *sm* :node :rarm-init-pose) :remap-list '((:goal . target-object)(:async . async-clients)))
    (send (send *sm* :node :rarm-move-to-target) :remap-list '((:goal . target-object)(:async . async-clients)))
    (send (send *sm* :node :rarm-pick-object-from-multi) :remap-list '((:goal . target-object)(:async . async-clients)))
    (send (send *sm* :node :rarm-move-for-verification) :remap-list '((:goal . target-object)(:async . async-clients)))
    (send (send *sm* :node :rarm-return-object) :remap-list '((:goal . target-object)(:async . async-clients)))
    (send (send *sm* :node :rarm-place-object) :remap-list '((:goal . target-object)(:async . async-clients)))
    (send (send *sm* :node :rarm-wait-verification) :remap-list '((:goal . target-object)(:async . async-clients)))
    (send (send *sm* :node :rarm-wait-place-object) :remap-list '((:goal . target-object)(:async . async-clients)))
    (send (send *sm* :node :rarm-pick-1-object) :remap-list '((:goal . target-object)(:async . async-clients)))

    (send (send *sm* :node :larm-init-pose) :remap-list '((:goal . target-object)(:async . async-clients)))
    (send (send *sm* :node :larm-move-to-target) :remap-list '((:goal . target-object)(:async . async-clients)))
    (send (send *sm* :node :larm-pick-object-from-multi) :remap-list '((:goal . target-object)(:async . async-clients)))
    (send (send *sm* :node :larm-move-for-verification) :remap-list '((:goal . target-object)(:async . async-clients)))
    (send (send *sm* :node :larm-return-object) :remap-list '((:goal . target-object)(:async . async-clients)))
    (send (send *sm* :node :larm-place-object) :remap-list '((:goal . target-object)(:async . async-clients)))
    (send (send *sm* :node :larm-wait-verification) :remap-list '((:goal . target-object)(:async . async-clients)))
    (send (send *sm* :node :larm-wait-place-object) :remap-list '((:goal . target-object)(:async . async-clients)))
    (send (send *sm* :node :larm-pick-1-object) :remap-list '((:goal . target-object)(:async . async-clients)))
    (ros::ros-info "ADD REMAP LIST")
    ;; NODES END ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


    ;; from and to nodes are selected by name or symbol
    (send *sm* :add-transition :rarm-check-target-list :rarm-init-pose t)
    (send *sm* :add-transition :larm-check-target-list :larm-init-pose t)

    (send *sm* :add-transition :rarm-init-pose :rarm-move-to-target :succeeded)
    (send *sm* :add-transition :larm-init-pose :larm-move-to-target :succeeded)

    (send *sm* :add-transition :rarm-pick-object-from-multi :rarm-pick-object-from-multi nil)
    (send *sm* :add-transition :larm-pick-object-from-multi :larm-pick-object-from-multi nil)

    (send *sm* :add-transition :rarm-move-for-verification :rarm-return-object nil)
    (send *sm* :add-transition :larm-move-for-verification :larm-return-object nil)

    (send *sm* :add-transition :rarm-pick-object-from-multi :rarm-wait-verification t)
    (send *sm* :add-transition :larm-pick-object-from-multi :larm-wait-verification t)

    (send *sm* :add-transition :rarm-wait-verification :rarm-wait-verification :wait)
    (send *sm* :add-transition :larm-wait-verification :larm-wait-verification :wait)

    (send *sm* :add-transition :rarm-wait-place-object :rarm-wait-place-object nil)
    (send *sm* :add-transition :larm-wait-place-object :larm-wait-place-object nil)
    
    (send *sm* :add-transition :rarm-check-target-list :join :end-picking)
    (send *sm* :add-transition :larm-check-target-list :join :end-picking)

    (send *sm* :add-transition :join :success t)
    
    (send *sm* :add-transition :rarm-move-to-target :rarm-pick-1-object nil)
    (send *sm* :add-transition :larm-move-to-target :larm-pick-1-object nil)
    (ros::ros-info "ADD TRANSITION")

    (send *sm* :arg-keys 'async-clients 'target-object)
    ))

  (defun init ()
    (ros::roseus "arm_state_machine_master")
    (make-parallel-state-machine)
    (ros::ros-info "created state machine ~A" *sm*))

  (warning-message 3 ";; (init)~%")

  (defun demo ()
    (when (or (not (boundp '*sm*)) (not (boundp '*robot-rarm-smach*) ) (not (boundp '*robot-larm-smach*)))
      (init))
    (exec-smach-with-spin *sm*))

  (warning-message 3 ";; (demo)~%")

  (provide :paralell-state-machine)



