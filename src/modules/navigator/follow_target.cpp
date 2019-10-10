/****************************************************************************
 *
 *   Copyright (c) 2013-2016 PX4 Development Team. All rights reserved.
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

#include "follow_target.h"

#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <fcntl.h>

#include <systemlib/err.h>

#include <systemlib/mavlink_log.h>

#include <uORB/uORB.h>
#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/follow_target.h>
#include <lib/ecl/geo/geo.h>
#include <lib/mathlib/math/Limits.hpp>

#include "navigator.h"

using matrix::wrap_pi;

constexpr float FollowTarget::_follow_position_matricies[6][9];

FollowTarget::FollowTarget(Navigator *navigator) :
	MissionBlock(navigator),
	ModuleParams(navigator)
{
	_current_vel.zero();
	_step_vel.zero();
	_est_target_vel.zero();
	_slave_master_dis.zero();
	_target_position_offset.zero();
	_target_position_delta.zero();
}

void FollowTarget::on_inactive()
{
	reset_target_validity();
}

//1. 初始化切换模式时执行一次：参数参数
void FollowTarget::on_activation()
{
	//跟随目标的水平距离
	_param_follow_dis = _param_tracking_dist.get() < 3.0F ? 3.0F : _param_tracking_dist.get();

	//对接收到的主机位置进行一阶滤波
	_param_pos_filter = math::constrain((float) _param_tracking_resp.get(), .1F, 1.0F);

	//跟随方位，这个旋转的矩阵的计算类似绕偏航旋转
	_param_follow_side = _param_tracking_side.get();

	//飞机的编号
	_vehicle_id=_param_vehicle_id.get();


	if ((_param_follow_side > FOLLOW_LEFT) || (_param_follow_side < FOLLOW_FRONT)) {
		_param_follow_side = FOLLOW_BEHIDE;
	}

	_rot_matrix = (_follow_position_matricies[_param_follow_side]);

	if (_follow_target_sub < 0) {
		_follow_target_sub = orb_subscribe(ORB_ID(follow_target));
	}

	if (_formation_type_sub < 0) {
		_formation_type_sub = orb_subscribe(ORB_ID(formation_type));
	}
}


//可全局搜索Follow_TARGET 五
//这个函数里主要做什么，就是根据目标位置以及距离参数 计算从机真正应该飞往的经度纬度高度
//在从机跟随主机过程中还设计了一个状态机，刚进模式的时候等到位置目标位置数据，目标数据有效后进入位置跟随，当距离目标很近的时候计入速度跟随。
//根据主机位置、参数设置 从机应该的位置和速度都有了，下面通过两个函数发送给位置控制，如下两个函数：
//set_follow_target_item函数 把位置数据target_motion_with_offset复制给mission_item
//update_position_sp函数中_mission_item赋值给pos_sp_triplet->current，并且还赋值几个重要的标志位 航点类型SETPOINT_TYPE_FOLLOW_TARGET position_valid  velocity_valid
//这个主题和这三个标志位都会传递到位置控制中使用实现具体的控制，怎么使用的可以去看mc_pso_control.cpp，看看位置跟随 速度跟随在哪里具体怎么实现的

//主机target位置传递的过程
//vehicle_global_position 到 mavlink消息FOLLOW_TARGET发送出去 到接收到解析为主题消息follow_target 到封装为pos_sp_triple->cucurrent传递给位置控制 位置控制会将经度纬度转换为local本地坐标系
//中间涉及本身的位置解算global gps  local   

//强调一句，主机有速度从机才会跟随，跟随的是位置或者速度
void FollowTarget::on_active()
{
	struct map_projection_reference_s target_ref;
	follow_target_s slave_target_pos = {};
	uint64_t current_time = hrt_absolute_time();
	bool _radius_entered = false;
	bool _radius_exited = false;
	bool updated = false;
	float dt_ms = 0;

	orb_check(_follow_target_sub, &updated);

//2. 对主机位置指令进行滤波 得到主机的滤波后的位置_curr_master_pos
	if (updated) {

		//mavlink_log_info(&_mavlink_log_pub, "wanggen----------"); 
		//编队实现
		formation_pre();

		follow_target_s target_motion;

		_target_updates++; //follow_target主题有更新 说明主机位置、速度有效

		// save last known motion topic

		_previous_target_motion = _curr_master_pos;

		//可全局搜索Follow_TARGET 四，目标的位置信息经度纬度高度转存到target_motion中

		orb_copy(ORB_ID(follow_target), _follow_target_sub, &target_motion);

		if (_curr_master_pos.timestamp == 0) {
			//第一次进来，以后称为目标位置指令
			_curr_master_pos = target_motion;
		}

		//这是一个对目标期望位置的滤波，避免目标的位置太过剧烈。滤波的过程取之前的目标位置指令权重+现在目标位置指令权重，避免目标位置变换剧烈，由此不用担心目标的位置剧烈变换
		_curr_master_pos.timestamp = target_motion.timestamp;
		_curr_master_pos.lat = (_curr_master_pos.lat * (double)_param_pos_filter) + target_motion.lat * (double)(
						     1 - _param_pos_filter);
		_curr_master_pos.lon = (_curr_master_pos.lon * (double)_param_pos_filter) + target_motion.lon * (double)(
						     1 - _param_pos_filter);

	} 
	//如果好久没收到主机位置信息 则置位所有，重新来过。有此代码在此守候，不用担心万一接收不到主机数据怎么办，如果没接收到此模块标志位无效 大部分程序不运行，从机保持悬停等待主机位置数据
	else if (((current_time - _curr_master_pos.timestamp) / 1000) > TARGET_TIMEOUT_MS && target_velocity_valid()) {
		reset_target_validity();
	}

	// update distance to target
	//更新到目标的距离

//2.计算当前从机距离主机的水平距离xy
	if (target_position_valid()) {//follow_target主题更新一次 目标位置指令有更新，如果一段时间没有收到 140行有置位不会进入这里

		// get distance to target
		//得到距离目标的距离

		//飞机现在的位置映射为NED原点（0，0），再把现在位置指令映射成（x，y），则（x，y）就是现在距离目标的距离

		map_projection_init(&target_ref, _navigator->get_global_position()->lat, _navigator->get_global_position()->lon);
		map_projection_project(&target_ref, _curr_master_pos.lat, _curr_master_pos.lon, &_slave_master_dis(0),
				       &_slave_master_dis(1));

	}

	// update target velocity
	//更新目标的速度 所有的目标是值跟随目标
//3.计算主机水平速度_est_target_vel，没算z轴速度
	if (target_velocity_valid() && updated) {//follow_target主题更新两次 可以求目标速度了

		//计算出两次更新的时间间隔，这里用于求速度,也代表主机位置更新的频率
		dt_ms = ((_curr_master_pos.timestamp - _previous_target_motion.timestamp) / 1000);

		// ignore a small dt
		if (dt_ms > 10.0F) { //100hz
			// get last gps known reference for target
			//把上一次的主机位置指令 映射成NED原点（0,0），下面再把现在的位置指令映射出来 就可以得到目标两次位置的变换 即可算出目标的速度
			map_projection_init(&target_ref, _previous_target_motion.lat, _previous_target_motion.lon);

			//计算目标移动的距离
			map_projection_project(&target_ref, _curr_master_pos.lat, _curr_master_pos.lon,
					       &(_target_position_delta(0)), &(_target_position_delta(1)));

			//根据上面目标两次位置的变换 计算出速度_est表示估算出来的目标速度，注意这是水平面的速度 没有z轴速度
			_est_target_vel = _target_position_delta / (dt_ms / 1000.0f);

//4.当主机移动时 从机开始找方位 保持水平距离进行跟随

			// if the target is moving add an offset and rotation
			//如果目标有速度正在移动，则添加偏移和旋转，这个偏移和旋转指什么，要解决什么问题，前馈目标的位置？？5
			if (_est_target_vel.length() > 1.5F) { //实测特别有效 主机停下时  从机也会稳定停下，不会乱飘，只有在检测在主机速度时 才会继续跟。从机的最大速度可以大于主机。
				//如果目标（目标）有移动，则目标在移动过程中 从机也要找到位置进行跟随，从机只是从目标那里获取经度纬度高度数据
				//目标在转向的时候 并且有速度，这时候从机也会转向保持跟在目标后面，主要是速度原因，而非航向
				//如果不是转向 目标在直走，ok啊 从机距离主句还是要有一个offset，即保持距离
				//这一句主要就是实现从机对目标保持一定距离
				_target_position_offset = _rot_matrix * _est_target_vel.normalized() * _param_follow_dis;//实测 没有旋转矩阵 无法保持方位？？？？为什么
			}

			// are we within the target acceptance radius?
			// give a buffer to exit/enter the radius to give the velocity controller a chance to catch up
			//我们是否在目标接受半径范围内？
			//给出一个缓冲区来退出/输入半径，让速度控制器有机会赶上

			//当前飞机距离目标的距离+要和目标保持的参数距离<5米，已经“近身了”,放大范围看看跟速度是什么样的效果？？？现在5米以内才跟速度呢，注意我们的设置的跟随距离6米 是一直在跟随位置 不会进入跟随速度
			_radius_exited = ((_target_position_offset + _slave_master_dis).length() > (float) TARGET_ACCEPTANCE_RADIUS_M * 1.5f);
			_radius_entered = ((_target_position_offset + _slave_master_dis).length() < (float) TARGET_ACCEPTANCE_RADIUS_M);

			// to keep the velocity increase/decrease smooth      保持速度的平滑
			// calculate how many velocity increments/decrements  计算速度增量
			// it will take to reach the targets velocity         它将达到目标速度】
			//
			// with the given amount of steps also add a feed forward input that adjusts the
			// velocity as the position gap increases since
			// just traveling at the exact velocity of the target will not
			// get any closer or farther from the target

			_step_vel = (_est_target_vel - _current_vel) + (_target_position_offset + _slave_master_dis) * FF_K;
			_step_vel /= (dt_ms / 1000.0F * (float) INTERPOLATION_PNTS);
			_step_time_in_ms = (dt_ms / (float) INTERPOLATION_PNTS);

			// if we are less than 1 meter from the target don't worry about trying to yaw
			// lock the yaw until we are at a distance that makes sense
			//计算航向差得出偏航角速度 应该是希望从机机头指向主机 实际效果特别差从机航向乱转，因为是因为主从位置不准的原因导致的。最后不使用。
			if ((_slave_master_dis).length() > 0.8F) { 

				// yaw rate smoothing

				// this really needs to control the yaw rate directly in the attitude pid controller
				// but seems to work ok for now since the yaw rate cannot be controlled directly in auto mode

				_yaw_angle = get_bearing_to_next_waypoint(_navigator->get_global_position()->lat,
						_navigator->get_global_position()->lon,
						_curr_master_pos.lat,
						_curr_master_pos.lon);

				_yaw_rate = wrap_pi((_yaw_angle - _navigator->get_global_position()->yaw) / (dt_ms / 1000.0f));

			} else {
				_yaw_angle = _yaw_rate = NAN;
			}
		}


	}
//5.根据距离 方位等参数，计算从机跟随应该飞往真正的位置：经度纬度没有高度。
	
	if (target_position_valid()) {//如果目标位置有效

		// get the target position using the calculated offset

		//根据目标的位置指令 以及从机跟随的参数 计算出从机真实应该飞往的目标位置
		map_projection_init(&target_ref,  _curr_master_pos.lat, _curr_master_pos.lon);
		map_projection_reproject(&target_ref, _target_position_offset(0), _target_position_offset(1),
					 &slave_target_pos.lat, &slave_target_pos.lon);
	}

	// clamp yaw rate smoothing if we are with in
	// 3 degrees of facing target

	if (PX4_ISFINITE(_yaw_rate)) {
		if (fabsf(fabsf(_yaw_angle) - fabsf(_navigator->get_global_position()->yaw)) < math::radians(3.0F)) {
			_yaw_rate = NAN;
		}
	}

//6.从机至始至终保持航向不变，无头模式进行跟随，避免因航向晃动导致姿态晃动。
	/////////////////////////////////////////////////////
	_yaw_rate = 0;//这一句很重要，没有了角速度的“跟随”，避免从机跟着主机时乱动航向，航向乱动 俯仰横滚跟着动，整个跟随效果太别差劲！从机特别不稳 乱晃！实测的。。
	              //取消航向角速度 ，就是飞机跟随过程中航向一直保持不变，可以看做是无头模式！已经实测，跟随的稳定性极好！无头模式航向保持不变 位置跟随 从机姿态很平稳。
	/////////////////////////////////////////////////////

	// update state machine
//7. 进入最后的从机跟随的状态机，从机保持悬停、还是跟随位置、还是跟随速度，跟随的实现需要跟踪到位置控制里。
	switch (_follow_target_state) { //初始化为SET_WAIT_FOR_TARGET_POSITION 等待主机位置

	//在目标位置有效时 进入跟随目标位置
	case TRACK_POSITION: {

			if (_radius_entered == true) { //如果距离目标已经很近了，5米，进入速度跟随
				_follow_target_state = TRACK_VELOCITY;

			} 
			//距离目标还比较远，且目标位置有效，进行位置跟随
			else if (target_velocity_valid()) {

				////////开始跟随目标位置。跟随的位置是带有“距离参数”的位置target_motion_with_offset
				set_follow_target_item(&_mission_item, _param_follow_alt, slave_target_pos, _yaw_angle);
				// keep the current velocity updated with the target velocity for when it's needed
				_current_vel = _est_target_vel;
				
				update_position_sp(true, true, _yaw_rate);

			} 
			//跟的过程中接收不到主机位置了，当前位置保持悬停
			else {
				_follow_target_state = SET_WAIT_FOR_TARGET_POSITION;
			}

			break;
		}
	//这是跟随目标位置很近的时候了
	case TRACK_VELOCITY: {

			//在跟随速度的时候 如果距离目标又变远了，进入位置跟随
			if (_radius_exited == true) {
				_follow_target_state = TRACK_POSITION;

			} 
			//近距离进入速度跟随
			else if (target_velocity_valid()) {

				if ((float)(current_time - _last_update_time) / 1000.0f >= _step_time_in_ms) {
					_current_vel += _step_vel;
					_last_update_time = current_time;
				}

				set_follow_target_item(&_mission_item, _param_follow_alt, slave_target_pos, _yaw_angle);
				//使用速度 位置无效
				update_position_sp(true, false, _yaw_rate);

			} 
			//接收不到主机位置数据 保持悬停重新进入状态机
			else {
				_follow_target_state = SET_WAIT_FOR_TARGET_POSITION;
			}

			break;
		}

	//以当前位置 爬升到参数最小高度（最小8），保持悬停 等待主机位置。
	case SET_WAIT_FOR_TARGET_POSITION: {

			//爬升到最低高度，并等待目标位置

			follow_target_s target = {};

			// for now set the target at the minimum height above the uav

			target.lat = _navigator->get_global_position()->lat;
			target.lon = _navigator->get_global_position()->lon;
			target.alt = 0.0F;

			//以当前飞机位置 爬升到参数最小高度，这里写死了最小8米，悬停 等待主机位置。
			//所以就是如果切换到follow_target模式 没有目标位置，飞机就会一直在这里保持悬停
			set_follow_target_item(&_mission_item, _param_follow_alt, target, _yaw_angle);

			//赋值给pos_sp_triplet->current传递给位置控制进行位置实现，注意这里设置了航点类型pos_sp_triplet->current.type= SETPOINT_TYPE_FOLLOW_TARGET
			update_position_sp(false, false, _yaw_rate);

			//在当前位置保持悬停，进入下一个状态机，等待目标位置
			_follow_target_state = WAIT_FOR_TARGET_POSITION;
		}

	
	//从机保持悬停等待中，如果主机位子信息有效了 则进入位置跟踪
	case WAIT_FOR_TARGET_POSITION: {

			////////////////////////
			//这里有一个bug，不知道算不算bug，有时候从机切folow不跟随 经过跟踪是因为is_mission_item_reached()无效，里面nav_cmd=0(NAV_CMD_IDLE)
			//有时候切follow有效，知道原因在这里 可以的时候是怎样的情况 还没跟踪
			//if (is_mission_item_reached() && target_velocity_valid()) {
			if (target_position_valid() && target_velocity_valid()) {
				_target_position_offset(0) = _param_follow_dis; //记录下跟随距离的参数
				_follow_target_state = TRACK_POSITION; //目标位置有效了，进入跟随位置，如果位置没有效 代码保持在这里 从机保持悬停
			}

			break;
		}
	}
}

void FollowTarget::update_position_sp(bool use_velocity, bool use_position, float yaw_rate)
{
	// convert mission item to current setpoint

	struct position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();

	// activate line following in pos control if position is valid

	pos_sp_triplet->previous.valid = use_position;
	pos_sp_triplet->previous = pos_sp_triplet->current;
	mission_apply_limitation(_mission_item);
	mission_item_to_position_setpoint(_mission_item, &pos_sp_triplet->current);
	pos_sp_triplet->current.type = position_setpoint_s::SETPOINT_TYPE_FOLLOW_TARGET;
	pos_sp_triplet->current.position_valid = use_position;
	pos_sp_triplet->current.velocity_valid = use_velocity;
	pos_sp_triplet->current.vx = _current_vel(0);
	pos_sp_triplet->current.vy = _current_vel(1);
	pos_sp_triplet->next.valid = false;
	pos_sp_triplet->current.yawspeed_valid = PX4_ISFINITE(yaw_rate);
	pos_sp_triplet->current.yawspeed = yaw_rate;
	_navigator->set_position_setpoint_triplet_updated();
}

void FollowTarget::reset_target_validity()
{
	_yaw_rate = NAN;
	_previous_target_motion = {};
	_curr_master_pos = {};
	_target_updates = 0;
	_current_vel.zero();
	_step_vel.zero();
	_est_target_vel.zero();
	_slave_master_dis.zero();
	_target_position_offset.zero();
	reset_mission_item_reached();
	_follow_target_state = SET_WAIT_FOR_TARGET_POSITION;
}



//下面根据飞机ID、方位、高度、距离四个参数开发队形和队形变换的代码。
//跟随方位_param_tracking_side： 1 后面  2左后侧  3右后侧  4右面  5左面
//跟随距离：_param_follow_dis;  从机高度：_param_min_alt ;  飞机ID：_vehicle_id

//编队思路：先获取mavlink编队信息，编写编队状态机，再确认各飞机上面的编队参数。

void FollowTarget::formation_pre()
{

	//获取地面站发送过来的mavlink消息FORMATION_TYPE -> msg消息formation_type -> 变量_formation
	bool updated=false;
	orb_check(_formation_type_sub, &updated);

	if(updated){
		orb_copy(ORB_ID(formation_type), _formation_type_sub, &_formation);
		
		if(_formation.start_end==2)//队形中
		{	
			_curr_shape=_formation.formation_type; //记录期望的队形
		}

	}

	// if(_curr_shape!=_mav_shape){ //如果现在的队形不等于期望的队形
	// 	switch(_mav_shape){
	// 		case TRIANGLE://期望三角队形
	// 			_curr_shape=TRIANGLE;
	// 			break;
			
	// 		case HORIZONTAL://想切横向一字，直接切
	// 			_curr_shape=HORIZONTAL;
	// 			break;

	// 		case VERTICAL://想切横纵向一字
	// 			_curr_shape=VERTICAL;
	// 			break;

	// 		default:
	// 			_curr_shape = TRIANGLE;
	// 			break;

	// 	}

	// }
	

	//重新读取跟随距离
	_param_follow_dis = _param_tracking_dist.get() < 3.0F ? 3.0F : _param_tracking_dist.get();
	//重新读取高度
	_param_follow_alt = _param_min_alt.get();

	if(_curr_shape==TRIANGLE){//实现三角队形
		
		if(2==_vehicle_id)     {  _param_follow_side=2;  } //2号飞机左侧
		else if(3==_vehicle_id){  _param_follow_side=3;  } //3号飞机飞右侧
		else                   {  _param_follow_side=1;  }
	}
	else if(_curr_shape==HORIZONTAL){
		if(2==_vehicle_id)     {  _param_follow_side=5;  } //2号飞机左侧
		else if(3==_vehicle_id){  _param_follow_side=4;  } //3号飞机飞右侧
		else                   {  _param_follow_side=1;  }
	}
	else if(_curr_shape==VERTICAL){
		
		if(2==_vehicle_id)     {  _param_follow_side=1; _param_follow_dis=5.0f; } //2号飞机飞后侧 
		else if(3==_vehicle_id){  _param_follow_side=1; _param_follow_dis=10.0f; } //3号飞机飞后侧
		else                   {  _param_follow_side=1;  }

	}
	else{

	}
	//_param_follow_side=_param_tracking_side.get(); //重新获取方位参数
	//重新赋值旋转矩阵
	_rot_matrix = (_follow_position_matricies[_param_follow_side]);

}



bool FollowTarget::target_velocity_valid()
{
	// need at least 2 continuous data points for velocity estimate
	return (_target_updates >= 2);
}

bool FollowTarget::target_position_valid()
{
	// need at least 1 continuous data points for position estimate
	return (_target_updates >= 1);
}

void
FollowTarget::set_follow_target_item(struct mission_item_s *item, float min_clearance, follow_target_s &target,
				     float yaw)
{
	if (_navigator->get_land_detected()->landed) {
		/* landed, don't takeoff, but switch to IDLE mode */
		item->nav_cmd = NAV_CMD_IDLE;

	} else {

		item->nav_cmd = NAV_CMD_DO_FOLLOW_REPOSITION;

		/* use current target position */
		item->lat = target.lat;
		item->lon = target.lon;
		item->altitude = _navigator->get_home_position()->alt;//这是home高度
		//这是跟踪的最小高度写死了为8
		if (min_clearance > 8.0f) {
			item->altitude += min_clearance;

		} else {
			item->altitude += 8.0f; // if min clearance is bad set it to 8.0 meters (well above the average height of a person)
		}
	}

	item->altitude_is_relative = false;
	item->yaw = yaw;
	item->loiter_radius = _navigator->get_loiter_radius();
	item->acceptance_radius = _navigator->get_acceptance_radius();
	item->time_inside = 0.0f;
	item->autocontinue = false;
	item->origin = ORIGIN_ONBOARD;
}
