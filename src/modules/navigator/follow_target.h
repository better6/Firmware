/***************************************************************************
 *
 *   Copyright (c) 2016 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/
/**
 * @file followme.cpp
 *
 * Helper class to track and follow a given position
 *
 * @author Jimmy Johnson <catch22@fastmail.net>
 */

#pragma once

#include "navigator_mode.h"
#include "mission_block.h"

#include <mathlib/mathlib.h>
#include <matrix/math.hpp>

#include <px4_module_params.h>
#include <uORB/topics/follow_target.h>
#include <uORB/topics/formation_type.h>
#include <uORB/topics/vehicle_gps_position.h>

#include <systemlib/mavlink_log.h>

class FollowTarget : public MissionBlock, public ModuleParams
{

public:
	FollowTarget(Navigator *navigator);
	~FollowTarget() = default;

	void on_inactive() override;
	void on_activation() override;
	void on_active() override;

private:

	static constexpr int TARGET_TIMEOUT_MS = 2500;
	static constexpr int TARGET_ACCEPTANCE_RADIUS_M = 5;
	static constexpr int INTERPOLATION_PNTS = 20;
	static constexpr float FF_K = .25F;
	static constexpr float OFFSET_M = 8;

	enum FollowTargetState {
		TRACK_POSITION,
		TRACK_VELOCITY,
		SET_WAIT_FOR_TARGET_POSITION,
		WAIT_FOR_TARGET_POSITION
	};

	enum {
		FOLLOW_FRONT,
		FOLLOW_BEHIDE,
		FOLLOW_LEFT_BEHIDE,
		FOLLOW_RIGHT_BEHIND,
		FOLLOW_RIGHT,
		FOLLOW_LEFT
	};

	enum {
		START=1,
		FORM=2,
		END=3
	};

	enum {
		TRIANGLE=1,//三角形
		HORIZONTAL=2,//横向一字
		VERTICAL=3 //纵向一字
	};

	//下面是绕z轴旋转
	//cos  -sin 0
	//sin  cos  0
	//0     0   1
	static constexpr float _follow_position_matricies[8][9] = {     //以下这些方位没做,不要随意改动因为代码有用这些顺序 _param_follow_side在用,以主机速度方向为0度 顺时针为正
		{ 1.0F,  0.0F, 0.0F,  0.0F,  1.0F, 0.0F, 0.0F, 0.0F, 1.0F},    // 0度   前方   follow front
		{-1.0F,  0.0F, 0.0F,  0.0F, -1.0F, 0.0F, 0.0F, 0.0F, 1.0F},    // 180度 后方   follow behind
		{-1.0F,  1.0F, 0.0F, -1.0F, -1.0F, 0.0F, 0.0F, 0.0F, 1.0F},    //-225度 左后侧  behind*right 
		{-1.0F, -1.0F, 0.0F,  1.0F, -1.0F, 0.0F, 0.0F, 0.0F, 1.0F},    // 135度 右后侧  left*behind
		{ 0.0F, -1.0F, 0.0F,  1.0F,  0.0F, 0.0F, 0.0F, 0.0F, 1.0F},    // 90都  右侧   follow right	
		{ 0.0F,  1.0F, 0.0F, -1.0F,  0.0F, 0.0F, 0.0F, 0.0F, 1.0F},    //-90度  左侧   follow left sides
		{ 1.0F, -1.0F, 0.0F,  1.0F,  1.0F, 0.0F, 0.0F, 0.0F, 1.0F},    // 45都  右前方  
		{ 1.0F,  1.0F, 0.0F, -1.0F,  1.0F, 0.0F, 0.0F, 0.0F, 1.0F}     //-45度  左前方  
		                                                            

	};

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::NAV_MIN_FT_HT>)	_param_min_alt,
		(ParamFloat<px4::params::NAV_FT_DST>) _param_tracking_dist,
		(ParamInt<px4::params::NAV_FT_FS>) _param_tracking_side,
		(ParamFloat<px4::params::FT_POS_FLITER>) _param_tracking_resp,
		(ParamFloat<px4::params::FT_VEL_FILTER>) _param_vel_resp,
		(ParamFloat<px4::params::FT_POS_FF>) _param_pos_delay,
		(ParamInt<px4::params::MAV_SYS_ID>) _param_vehicle_id,
		(ParamInt<px4::params::FT_ENTER_SPD>) _enter_speed		
	)
	
	FollowTargetState _follow_target_state{SET_WAIT_FOR_TARGET_POSITION};
	int _param_follow_side{FOLLOW_BEHIDE};
	float _param_follow_alt{8.0f};
	float _param_follow_dis{OFFSET_M};
	int _vehicle_id{0};
	int _param_vel_dis{5};

	int _follow_target_sub{-1};
	int _formation_type_sub{-1};
	int  _slave_gps_sub{-1};
	orb_advert_t	_mavlink_log_pub{nullptr};
	float _step_time_in_ms{0.0f};

	formation_type_s _formation{0};
	vehicle_gps_position_s _slave_gps{0};

	uint8_t _curr_shape{1};
	uint8_t _mid_shape{1};
	uint8_t _mav_shape{1};

	uint64_t _target_updates{0};
	uint64_t _last_update_time{0};

	matrix::Vector3f _current_vel;
	matrix::Vector3f _step_vel;
	matrix::Vector3f _est_target_vel;
	matrix::Vector3f _master_vel;
	matrix::Vector3f _vel_pre;
	matrix::Vector3f _slave_master_dis;
	matrix::Vector3f _target_position_offset;
	matrix::Vector3f _delay_offset;
	matrix::Vector3f _target_position_delta;
	matrix::Vector3f _filtered_target_position_delta;

	follow_target_s _curr_master{};
	follow_target_s _previous_target_motion{};

	float _yaw_rate{0.0f};
	float _param_pos_filter{0.2f};
	float _param_vel_filter{0.2f};
	float _param_delay{1.5f};
	float _yaw_angle{0.0f};

	// Mavlink defined motion reporting capabilities
	enum {
		POS = 0,
		VEL = 1,
		ACCEL = 2,
		ATT_RATES = 3
	};

	matrix::Dcmf _rot_matrix;
	matrix::Dcmf _rot_delay;

	void track_target_position();
	void track_target_velocity();
	bool target_velocity_valid();
	bool target_position_valid();
	void reset_target_validity();
	void formation_pre();
	void update_position_sp(bool velocity_valid, bool position_valid, float yaw_rate);
	void update_target_motion();
	void update_target_velocity();

	/**
	 * Set follow_target item
	 */
	void set_follow_target_item(struct mission_item_s *item, float min_clearance, follow_target_s &target, float yaw);
};
