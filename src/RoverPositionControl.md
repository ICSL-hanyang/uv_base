Firmware/src/modules/rover_pos_control/RoverPositionControl.cpp  
파일에 control_velocity 함수를 아래 코드로 변경하면   
angular.z와 linear.x값으로 속도제어가능  
펌웨어 버전 v1.11.0-beta1

```c++
void
RoverPositionControl::control_velocity(const matrix::Vector3f &current_velocity,
				       const position_setpoint_triplet_s &pos_sp_triplet)
{
	float dt = 0.01; // Using non zero value to a avoid division by zero

	const float mission_throttle = _param_throttle_cruise.get();
	const float desired_speed = pos_sp_triplet.current.vx;

	if (desired_speed > 0.01f || desired_speed < -0.01f) {
		const Dcmf R_to_body(Quatf(_vehicle_att.q).inversed());
		// current_velocity는 body Frame이 아닌 local Frame 이니 body Frame으로 변경해주는 작업 필요!
		const Vector3f vel = R_to_body * Vector3f(current_velocity(0), current_velocity(1), current_velocity(2));

		const float x_vel = vel(0);
		const float x_acc = _vehicle_acceleration_sub.get().xyz[0];
		const float control_throttle = pid_calculate(&_speed_ctrl, desired_speed, x_vel, x_acc, dt);

		//Constrain maximum throttle to mission throttle
		_act_controls.control[actuator_controls_s::INDEX_THROTTLE] = math::constrain(control_throttle, -mission_throttle, mission_throttle);

		const float angular_z_speed = pos_sp_triplet.current.yawspeed; //angular z

		if (angular_z_speed > 0.01f || angular_z_speed < -0.01f)
		{ //조향이 있을때
			const float wheel_base = _param_wheel_base.get();
			float radius = abs(desired_speed) / angular_z_speed;
			const float desired_theta = atanf(wheel_base / radius);
			float control_effort = desired_theta / _param_max_turn_angle.get();
			control_effort = math::constrain(control_effort, -1.0f, 1.0f);

			_act_controls.control[actuator_controls_s::INDEX_YAW] = control_effort;
		}
		else {
			_act_controls.control[actuator_controls_s::INDEX_YAW] = 0.0f;
		}
	}
	else {
		_act_controls.control[actuator_controls_s::INDEX_THROTTLE] = 0.0f;

	}
}
```
