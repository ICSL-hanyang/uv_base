Firmware/src/modules/rover_pos_control/RoverPositionControl.cpp  
파일에 control_velocity 함수를 아래 코드로 변경하면   
angular.z와 linear.x값으로 속도제어가능  
펌웨어 버전 v1.11.0-beta1

```c++
void
RoverPositionControl::control_velocity(const matrix::Vector3f &current_velocity,
				       const position_setpoint_triplet_s &pos_sp_triplet)
{
	if (pos_sp_triplet.current.velocity_frame == position_setpoint_s::VELOCITY_FRAME_BODY_NED) {
		// float dt = 0.01; // Using non zero value to a avoid division by zero

		const float mission_throttle = 0.7f;
		// const float mission_throttle = _param_throttle_cruise.get();
		// const matrix::Vector3f desired_velocity{pos_sp_triplet.current.vx, pos_sp_triplet.current.vy, pos_sp_triplet.current.vz};
		// const float desired_speed = desired_velocity.norm();

		const matrix::Vector3f linear_x{pos_sp_triplet.current.vx, pos_sp_triplet.current.vx, pos_sp_triplet.current.vx};
		// const matrix::Vector3f linear_x{pos_sp_triplet.current.vy, pos_sp_triplet.current.vy, pos_sp_triplet.current.vy};
		const matrix::Vector3f angular_z{pos_sp_triplet.current.yawspeed, pos_sp_triplet.current.yawspeed, pos_sp_triplet.current.yawspeed};

		const float linear_x_speed = linear_x.min(); //linear x
		const float angular_z_speed = angular_z.min(); //angular z

		// angular z
		// const Quatf qe = Quatf(att.q).inversed() * Quatf(att_sp.q_d);
		// const Eulerf euler_sp = qe;
		// angular_z.print();
		printf("vx : %f\nvy : %f\nvz : %f\n",(double)pos_sp_triplet.current.vx,(double)pos_sp_triplet.current.vy,(double)pos_sp_triplet.current.vz );

		// linear_x.print();
		if ( (linear_x_speed > 0.01f || linear_x_speed < -0.01f) && (angular_z_speed > 0.01f || angular_z_speed < -0.01f) )
		{ //throttle 속도가 있고, 조향도 있을때.
			const float control_throttle = linear_x_speed;
			float control_effort = angular_z_speed / _param_max_turn_angle.get();
			// printf("control_effort : %f",(double)control_effort);
			control_effort = math::constrain(control_effort, -1.0f, 1.0f);
			// printf("constrain : %f",(double)control_effort);

			_act_controls.control[actuator_controls_s::INDEX_THROTTLE] = math::constrain(control_throttle, -mission_throttle, mission_throttle);
			_act_controls.control[actuator_controls_s::INDEX_YAW] = control_effort;
		}
		else if ( (linear_x_speed > 0.01f || linear_x_speed < -0.01f) && (angular_z_speed <= 0.01f && angular_z_speed >= -0.01f) )
		{ //속도있고 조향 없고
			const float control_throttle = linear_x_speed;
			_act_controls.control[actuator_controls_s::INDEX_THROTTLE] = math::constrain(control_throttle, -mission_throttle, mission_throttle);
			_act_controls.control[actuator_controls_s::INDEX_YAW] = 0.0f;
		}
		else if ( (linear_x_speed <= 0.01f && linear_x_speed >= -0.01f) && (angular_z_speed > 0.01f || angular_z_speed < -0.01f) )
		{ //속도없고 조향 있고
			float control_effort = angular_z_speed / _param_max_turn_angle.get();
			control_effort = math::constrain(control_effort, -1.0f, 1.0f);
			_act_controls.control[actuator_controls_s::INDEX_THROTTLE] = 0.0f;
			_act_controls.control[actuator_controls_s::INDEX_YAW] = control_effort;
		}
		else
		{ //속도없고 조향 없고
			_act_controls.control[actuator_controls_s::INDEX_THROTTLE] = 0.0f;
			_act_controls.control[actuator_controls_s::INDEX_YAW] = 0.0f;
		}
		// printf("THR : %f\nYAW : %f\n",(double)_act_controls.control[actuator_controls_s::INDEX_THROTTLE], (double)_act_controls.control[actuator_controls_s::INDEX_YAW] );
	}
	// if (desired_speed > 0.01f) {

	// 	const Dcmf R_to_body(Quatf(_vehicle_att.q).inversed());
	// 	const Vector3f vel = R_to_body * Vector3f(current_velocity(0), current_velocity(1), current_velocity(2));

	// 	const float x_vel = vel(0);
	// 	const float x_acc = _vehicle_acceleration_sub.get().xyz[0];

	// 	const float control_throttle = pid_calculate(&_speed_ctrl, desired_speed, x_vel, x_acc, dt);

	// 	//Constrain maximum throttle to mission throttle
	// 	_act_controls.control[actuator_controls_s::INDEX_THROTTLE] = math::constrain(control_throttle, 0.0f, mission_throttle);

	// 	Vector3f desired_body_velocity;

	// 	if (pos_sp_triplet.current.velocity_frame == position_setpoint_s::VELOCITY_FRAME_BODY_NED) {
	// 		desired_body_velocity = desired_velocity;

	// 	} else {
	// 		// If the frame of the velocity setpoint is unknown, assume it is in local frame
	// 		desired_body_velocity = R_to_body * desired_velocity;

	// 	}

	// 	const float desired_theta = atan2f(desired_body_velocity(1), desired_body_velocity(0));
	// 	float control_effort = desired_theta / _param_max_turn_angle.get();
	// 	control_effort = math::constrain(control_effort, -1.0f, 1.0f);

	// 	_act_controls.control[actuator_controls_s::INDEX_YAW] = control_effort;

	// } else {

	// 	_act_controls.control[actuator_controls_s::INDEX_THROTTLE] = 0.0f;
	// 	_act_controls.control[actuator_controls_s::INDEX_YAW] = 0.0f;

	// }
}
```
