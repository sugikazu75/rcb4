(require :robot-interface "package://pr2eus/robot-interface.l")

(ros::load-ros-manifest "control_msgs")
(ros::load-ros-manifest "kxr_controller")


(defun load-robot-model (&key (port 8000) (namespace nil)
                           (server-ip "localhost"))
  (let* ((hash-param-name (if namespace (format nil "~A~A" namespace "/eusmodel_hash") "/eusmodel_hash"))
         (robot-name-param-name (if namespace (format nil "~A~A" namespace "/eus_robot_name") "/eus_robot_name"))
         hash robot-name outpath server-url)
    (setq port (truncate port))
    (while (null hash)
      (ros::ros-info (format nil "Waiting rosparam ~A" hash-param-name))
      (setq hash (ros::get-param hash-param-name nil))
      (unix::usleep (* 1000 1000)))
    (ros::ros-info (format nil "Get rosparam ~A" hash-param-name))

    (while (null robot-name)
      (ros::ros-info (format nil "Waiting rosparam ~A" robot-name-param-name))
      (setq robot-name (ros::get-param robot-name-param-name nil))
      (unix::usleep (* 1000 1000)))
    (ros::ros-info (format nil "Get rosparam ~A" robot-name-param-name))

    (setq outpath (ros::resolve-ros-path (format nil "package://kxreus/models/cache/~A.l" hash)))
    (setq server-url (format nil "http://~A:~A/euslisp/~A.l" server-ip port hash))

    (when (not (probe-file outpath))
      (while (not (= (unix:system (format nil "wget -O ~A ~A" outpath server-url)) 0))
        (ros::ros-info (format nil "Waiting model file from server url ~A" server-url))
        (unix::usleep (* 1000 1000))))
    (load outpath)
    (funcall (read-from-string robot-name))))


(defclass kxr-interface
  :super robot-interface
  :slots (joint-names servo-on-off-client))


(defmethod kxr-interface
  (:init (robot &rest args &key (type :default-controller) &allow-other-keys)
   (let* ((namespace (cadr (memq :namespace args)))
          (joint-param (if namespace (format nil "~A/~A" namespace "/kxr_fullbody_controller/joints")
                           "/kxr_fullbody_controller/joints")))
     (setq joint-names (ros::get-param joint-param nil))
     (while (and (ros::ok) (null joint-names))
       (ros::ros-info (format nil "Waiting ~A rosparam set" joint-param))
       (setq joint-names (ros::get-param joint-param nil))
       (unix::usleep (* 1000 1000))))

   (send-super* :init :robot robot :type type
                :groupname "kxr_interface"
                args)

   (setq servo-on-off-client (instance ros::simple-action-client :init
                                       (if namespace (format nil "~A/~A" namespace "/kxr_fullbody_controller/servo_on_off") "/kxr_fullbody_controller/servo_on_off")
                                       kxr_controller::ServoOnOffAction
                                       :groupname groupname))
   (dolist (action (list servo-on-off-client))
     (unless (and joint-action-enable (send action :wait-for-server 3))
       (setq joint-action-enable nil)
       (ros::ros-warn "~A is not respond, kxr-interface is disabled" action)
       (return)))
   t)
  (:default-controller
   ()
   (send self :fullbody-controller))
  (:fullbody-controller
   ()
   (list
    (list
     (cons :controller-action "kxr_fullbody_controller/follow_joint_trajectory")
     (cons :controller-state "kxr_fullbody_controller/state")
     (cons :action-type control_msgs::FollowJointTrajectoryAction)
     (cons :joint-names joint-names))))
  (:servo-on
   (&key (names nil))
   (unless joint-action-enable
     (if viewer (send self :draw-objects))
     (return-from :servo-on t))
   (when (null names)
     (setq names joint-names))
   (let* (goal result)
     ;; send current angle-vector
     (send self :angle-vector (send self :state :potentio-vector) 0.1)
     (send self :wait-interpolation)

     (setq goal (instance kxr_controller::ServoOnOffGoal :init))
     (send goal :joint_names names)
     (send goal :servo_on_states (make-array (length names) :initial-element t))
     (send servo-on-off-client :send-goal goal)))
  (:servo-off
   (&key (names nil))
   (unless joint-action-enable
     (if viewer (send self :draw-objects))
     (return-from :servo-off t))
   (when (null names)
     (setq names joint-names))
   (let* (goal result)
     (setq goal (instance kxr_controller::ServoOnOffGoal :init))
     (send goal :joint_names names)
     (send goal :servo_on_states (make-array (length names) :initial-element nil))
     (send servo-on-off-client :send-goal goal))))


(defun kxr-init (&key
                   (namespace nil)
                   (create-viewer t))
  (unless (boundp '*robot*)
    (setq *robot* (load-robot-model :namespace namespace
                                    :server-ip (ros::get-param (if namespace (format nil "~A/model_server_ip" namespace) "/model_server_ip") "localhost")
                                    :port (ros::get-param (if namespace (format nil "~A/model_server_port" namespace) "/model_server_port") 8123))))
  (unless (ros::ok) (ros::roseus "kxr_eus_interface"))
  (unless (boundp '*ri*)
    (setq *ri* (instance kxr-interface :init *robot* :namespace namespace)))

  (ros::spin-once)
  (send *ri* :spin-once)
  (send *robot* :angle-vector (send *ri* :state :potentio-vector))
  (when create-viewer (objects (list *robot*))))
