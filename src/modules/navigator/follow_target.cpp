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
#include <lib/ecl/geo/geo.h>                //很重要的库，代码中很多地理运算都是基于这个库函数实现的。这里面有很多实用的地理运算
#include <lib/mathlib/math/Limits.hpp>

#include "navigator.h"

using matrix::wrap_pi;

constexpr float FollowTarget::_follow_position_matricies[8][9];

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

//1. 初始化切换模式时执行一次：获取参数
void FollowTarget::on_activation()
{
	//跟随目标的水平距离
	_param_follow_dis = _param_tracking_dist.get() < 3.0F ? 3.0F : _param_tracking_dist.get();

	//对主机的位置和速度进行绿币的参数,防止主机位置速度骤变
	_param_pos_filter = math::constrain((float) _param_tracking_resp.get(), .1F, 1.0F);

	_param_vel_filter  = math::constrain((float) _param_vel_resp.get(), .1F, 1.0F);

	_param_delay  = math::constrain((float) _param_pos_delay.get(), .1F, 10.0F);

	//跟随方位参数,计算跟随主机的方位,这是计算的是相对主机的一个点(从机跟随的目标点),背后的原理就是旋转矩阵的计算
	_param_follow_side = _param_tracking_side.get();

	_param_vel_dis = _enter_speed.get();

	//获取飞机编号,区分主机从机
	_vehicle_id=_param_vehicle_id.get();


	if ((_param_follow_side > 7) || (_param_follow_side < 0)) {
		_param_follow_side = FOLLOW_BEHIDE;
	}

	_rot_matrix = (_follow_position_matricies[_param_follow_side]);

	_rot_delay  = (_follow_position_matricies[0]); //前方

	if (_follow_target_sub < 0) {
		_follow_target_sub = orb_subscribe(ORB_ID(follow_target));
	}

	if (_formation_type_sub < 0) {
		_formation_type_sub = orb_subscribe(ORB_ID(formation_type));
	}

	//订阅gps获取当前utc时间计算延时
	if (_slave_gps_sub < 0) {
		_slave_gps_sub = orb_subscribe(ORB_ID(vehicle_gps_position));
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

//强调一句，主机有速度从机才会跟随，以主机方向为准,跟随的是位置或者速度
void FollowTarget::on_active()
{
	struct map_projection_reference_s target_ref;
	follow_target_s slave_target_pos = {};
	uint64_t current_time = hrt_absolute_time();
	bool	 _radius_entered = false;
	bool 	 _radius_exited = false;
	bool 	 updated = false;
	float	 dt_ms = 0;


	orb_check(_follow_target_sub, &updated);

//1、 主机位置有更新follow才有效,否则大部分代码不会执行,从机保持悬停
//	对主机位置速度进行滤波 防止主机的抖动 引起从机的不稳定
	if (updated) {

		follow_target_s get_master;

		_target_updates++; //根据这个变量：更新的次数，判断主机位置和速度的有效，如果长时间未获取主机位置 此标志位会被重置为0

		_previous_target_motion = _curr_master;

		//可全局搜索Follow_TARGET 四，主机的位置信息经度纬度高度转存到target_motion中

		orb_copy(ORB_ID(follow_target), _follow_target_sub, &get_master);

		if (_curr_master.timestamp == 0) { //第一次获取主机信息
			_curr_master = get_master;  //主机位置
			_master_vel(0) = get_master.vx; //主机速度
			_master_vel(1) = get_master.vy;
			_vel_pre       = _master_vel;
		}

		//对主机位置速度进行滤波，避免目标位置变换剧烈影响从机跟随的稳定性
		_curr_master.timestamp = get_master.timestamp;
		_curr_master.lat = (_curr_master.lat * (double)_param_pos_filter) + get_master.lat * (double)( 1 - _param_pos_filter);
		_curr_master.lon = (_curr_master.lon * (double)_param_pos_filter) + get_master.lon * (double)( 1 - _param_pos_filter);

		//对主机速度进行滤波
		_master_vel(0) = ( _master_vel(0) * _param_vel_filter ) + get_master.vx *( 1 - _param_vel_filter);
		_master_vel(1) = ( _master_vel(1) * _param_vel_filter ) + get_master.vy *( 1 - _param_vel_filter);
		_master_vel(2) = 0;


		_curr_shape=get_master.form_type;   //可全局搜索主机把编队类型再次传给从机三
		//预处理：取所有参数、包括队形参数、主机速度方向改变从机走位的判断
		formation_pre();

		
		//从机延时参数是使用主机统一设置的参数 还是使用从机自己的参数
		if(get_master.slave_delay > 5 ) //主机传递过来的统一设置的从机的延时参数，参数是FT_SLAVE_DELAY地面站设置是小数 传递的时候放大了10倍作为整数传递的
		{
			delay_s=  get_master.slave_delay/10.0f; //主机参数>0.5 有效 统一使用主机设置的参数FT_SLAVE_DELAY
		}
		else{
			delay_s=_param_delay; //主机参数<0.5无效，使用从机自己设置的延时参数FT_POS_FF
		}





		//计算的延时有问题:获取不到hrt和utc时间,这个地方可以好好深究下
		// _curr_master.master_utc = get_master.master_utc;//主机的utc时间
		// hrt_abstime slave_utc_now = _slave_gps.time_utc_usec + (hrt_absolute_time()-_slave_gps.timestamp);
		// delay_s   =  (slave_utc_now - get_master.master_utc ) * 1e-6f;
		
		
	} 
	//如果好久没收到主机位置信息 则置位所有，重新来过。有此代码在此守候，不用担心万一接收不到主机数据怎么办，如果没接收到此模块标志位无效 大部分程序不运行，从机保持悬停等待主机位置数据
	else if (((current_time - _curr_master.timestamp) / 1000) > TARGET_TIMEOUT_MS && target_velocity_valid()) {
		reset_target_validity();
	}



//2.计算当前从机距离主机的水平距离xy
//  注意这两个函数可以直接实现Global坐标系和NED坐标系之间的想换转换。注意是NED坐标系，不是机体坐标系，这里区分清楚 方便后面旋转方位的理解
	if (target_position_valid()) {//主机位置有更新

		// get distance to target
		//得到距离目标的距离

		//从机现在的global位置映射为NED原点（0，0），然后把主机的位置映射成NED下的（x，y），则（x，y）不就是现在主从之间的距离嘛
		//注意再强调一边这三个映射函数，可以直接实现global经度纬度和NED xy的想换转换。
		//从机需要的航点是经度纬度数据，而NED可以很好的计算的飞机间的相对距离、飞机的速度

		map_projection_init(&target_ref, _navigator->get_global_position()->lat, _navigator->get_global_position()->lon);
		map_projection_project(&target_ref, _curr_master.lat, _curr_master.lon, &_slave_master_dis(0),
				       &_slave_master_dis(1));

	}



//3. 更新主机的速度，为什么这里要自己估算速度 而不是直接获取主机的速度，这里的速度不包含z轴 而且是NED坐标系下的vx vy，因为这是给人看的 所以NED坐标系很正常

	if (target_velocity_valid() && updated) {//follow_target主题更新两次 可以求目标速度了

		//计算出两次更新的时间间隔，这里用于求速度,也可以求出主机位置更新的频率
		dt_ms = ((_curr_master.timestamp - _previous_target_motion.timestamp) / 1000);
		//mavlink_log_info(&_mavlink_log_pub, "主机频率=%2.1f",1/dt_ms);

		// ignore a small dt
		if (dt_ms > 10.0F) { //主机更新最快允许100hz
			// get last gps known reference for target
			//把上一次的主机位置 映射成NED原点（0,0），下面再把现在的位置映射出来 就可以得到目标两次位置的变换 即可算出目标的速度
			map_projection_init(&target_ref, _previous_target_motion.lat, _previous_target_motion.lon);

			//计算目标移动的距离，这里求出是NED坐标系的x y，所以下面求出的速度也是NED下的vx vy
			map_projection_project(&target_ref, _curr_master.lat, _curr_master.lon, &(_target_position_delta(0)), &(_target_position_delta(1))); 

			//估算出来的主机速度，因为位置有滤波 此速度低于主机真实的速度，这是NED下的速度 而且没有z轴
			//_est_target_vel = _target_position_delta / (dt_ms / 1000.0f); 

			_est_target_vel = _master_vel;//不用估算主机的速度 直接使用主机传递过来的速度，但是目前来看主机更新的频率个很低 会不会这里有影响，
			
//4.上面估算出主机的速度，当主机移动时 从机开始找方位 保持水平距离进行跟随
//  如果主机没有移动，从机会怎么样呢

			//如果主机正在移动， if the target is moving add an offset and rotation
			if (_est_target_vel.length() > 0.8F) { //实测有效 主机停下时 从机也会稳定停下，不会乱飘，只有在检测在主机速度时 才会继续跟。从机的最大速度可以大于主机。
				
				//现在知道主机的经度纬度和NED速度，如何求从机飞往的位置（经度纬度），不需要从机的速度，只给从机期望的位置，速度由从机自行计算。
				//1.前面的计算 根据主机速度方向（单位向量）旋转一个角度，（可以参考旋转向量的计算过程）
				//2.然后在旋转后的方向上求长度（主从保持的距离），最终得到NED坐标系主从之间的相对距离（这个距离是个向量 就是NED下x轴y轴的之间的相对距离），方位在主机速度的某个角度。
				//3.有了NED下的相对距离 就可以根据主机的经度纬度 利用映射函数 求出期望的经度纬度。
				//这种计算方式可能不是很好理解，对于我们观众是在NED下看相对位置，或者采用林哥固定翼编队的做法：先求机体坐标系下的相对距离，再利用速度方向转换到NED坐标系，最后转到global坐标系下求经度纬度
				_target_position_offset = _rot_matrix * _est_target_vel.normalized() * _param_follow_dis;//实测 没有旋转矩阵 无法保持方位

				//这是我加的延时补偿，本质上也是弥补到了NED x轴y轴的相对距离上
				//已经实测 这种主机前方位置的补偿有效
				_delay_offset = _rot_delay * _master_vel.normalized() * _master_vel.length()* delay_s;

				//已经实测 这种主机前方位置的补偿有效
				_target_position_offset=_target_position_offset  +  _delay_offset;
						
			}
			else{
				//如果主机没有移动 静止呢
				//主机没有动，就不会新定义从机期望的位置，主从的之间的位置也是保持不变，这时候如果跟位置 位置都没有动从机应该保持静止 如果是跟速度从机速度为0 主机也应该静止
				//就是主机不动的时候 从机也应该保持静止，但是从机会静止在哪个位置呢
				mavlink_log_info(&_mavlink_log_pub, "主静");
			}
	
			// are we within the target acceptance radius?
			// give a buffer to exit/enter the radius to give the velocity controller a chance to catch up

			//这是两个向量相加 最后还是一个向量，最后的和表示从机现在的位置 到从机期望的位置（向量）,已经实测验证。
			//所以下面求向量的长度，即从机距离期望的位置很近的时候，那就是队形上差不多和主机保持一致的时候，如横向一字 就是快到和主机位置水平的时候，为了保持队形 应该和主机的速度保持相对静止
			//远距离跟位置 近距离跟速度 这种方式测试跟随效果不错跟随稳定，就是主机速度大时 主从位置有落差。


			_radius_exited = ((_target_position_offset + _slave_master_dis).length() > (float) _param_vel_dis * 1.5f);
			_radius_entered = ((_target_position_offset + _slave_master_dis).length() < (float) _param_vel_dis);
			mavlink_log_info(&_mavlink_log_pub, "dis=%2.1f",(double)(_target_position_offset + _slave_master_dis).length());


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
			if ((_slave_master_dis).length() > 1.0F) { 

				// yaw rate smoothing

				// this really needs to control the yaw rate directly in the attitude pid controller
				// but seems to work ok for now since the yaw rate cannot be controlled directly in auto mode

				_yaw_angle = get_bearing_to_next_waypoint(_navigator->get_global_position()->lat,
						_navigator->get_global_position()->lon,
						_curr_master.lat,
						_curr_master.lon);

				_yaw_rate = wrap_pi((_yaw_angle - _navigator->get_global_position()->yaw) / (dt_ms / 1000.0f));

			} else {
				_yaw_angle = _yaw_rate = NAN;
			}
		}


	}


//5. 计算从机真实的目标位置，给从机位置指令（或者近距离时也会给速度指令）
//  根据距离 方位等参数，计算从机跟随应该飞往真正的位置：经度纬度没有高度。
	
	if (target_position_valid()) {//如果主机位置有效

		// get the target position using the calculated offset

		//根据主机的位置  以及从机跟随的参数 计算出从机真实应该飞往的目标位置
		map_projection_init(&target_ref,  _curr_master.lat, _curr_master.lon);
		map_projection_reproject(&target_ref, _target_position_offset(0), _target_position_offset(1),  //找到了为什么从机先于主机完成起飞后会向主机飞去，都飞向主机看起来比较危险。原因是从机完成起飞，主机还在起飞过程中 没有速度 就没有偏移量_target_position_offset的计算，这里代入后就是飞往主机所在的位置
					 &slave_target_pos.lat, &slave_target_pos.lon);

		//计算从机到目标位置的距离
		// float dis_target = get_distance_to_next_waypoint((double)_navigator->get_global_position()->lat, (double)_navigator->get_global_position()->lon,
		// 												 (double)slave_target_pos.lat, (double)slave_target_pos.lon);
		
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


//7. 进入最后的从机跟随的状态机，从机保持悬停、还是跟随位置、还是跟随速度，
//   这是只是状态机，跟随的具体代码实现需要跟踪到多旋翼位置控制里去查看。
	switch (_follow_target_state) { //初始化为SET_WAIT_FOR_TARGET_POSITION爬升一定高度等待主机位置更新，如果一直接收不到主机位置就在此处悬停

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
				mavlink_log_info(&_mavlink_log_pub, "POS");
				
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
				mavlink_log_info(&_mavlink_log_pub, "VEL");

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

			//有时候从机切folow不跟随 经过分析是因为is_mission_item_reached()无效，里面nav_cmd=0(NAV_CMD_IDLE)，但有的时候是可以跟踪，再深入的没分析
			//if (is_mission_item_reached() && target_velocity_valid()) {
			
			//修改判断为高度判断,飞机达到一定高度后才会开始follow，避免从机还在向上起飞过程中 如果接到了主机更新的位置信息 就会斜着去跟
			if ( (_navigator->get_global_position()->alt > _navigator->get_home_position()->alt + _param_follow_alt - 2.0f) && target_velocity_valid() && (_est_target_vel.length() > 0.8F) ) { //从机先于主机完成起飞后 不会再奔向主机

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
	_curr_master = {};
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

	bool updated =false;
	
	orb_check(_slave_gps_sub, &updated);

	if(updated){
		orb_copy(ORB_ID(vehicle_gps_position), _slave_gps_sub, &_slave_gps);
	}

	//获取地面站发送过来的mavlink消息FORMATION_TYPE -> msg消息formation_type -> 变量_formation
	orb_check(_formation_type_sub, &updated);

	if(updated){
		orb_copy(ORB_ID(formation_type), _formation_type_sub, &_formation);
		
		if(_formation.start_end==2)//队形中
		{	
			_curr_shape=_formation.formation_type; //记录期望的队形
		}

	}


	_param_pos_filter = math::constrain((float) _param_tracking_resp.get(), .1F, 1.0F);

	_param_vel_filter  = math::constrain((float) _param_vel_resp.get(), .1F, 1.0F);

	_param_delay  = math::constrain((float) _param_pos_delay.get(), .1F, 10.0F);

	//进入速度跟随的距离
	_param_vel_dis = _enter_speed.get();

	//重新读取跟随距离
	_param_follow_dis = _param_tracking_dist.get() < 3.0F ? 3.0F : _param_tracking_dist.get();

	//重新读取高度
	_param_follow_alt = _param_min_alt.get();

	if(2==_vehicle_id) {
		_param_follow_alt=8;
	}
	else if(3==_vehicle_id){
		_param_follow_alt=10;
	}
	else{

	}

	//cos120 = cos-120 = -0.5
	//cos90  = cos-90  = 0
	//从机跟随的是主机的速度方向 根据主机前后两次的速度方向，调整从机跟随的方位，主要目的是防止主机方向变化太大时 从机交叉互换位置有相撞的可能性。
	if(_master_vel.length() > 0.8F){//主机有速度了，才会进行比较
		
		float cos = _vel_pre.normalized() * _master_vel.normalized();
		if(cos < -0.5f){ //主机速度方向改变很大，超出+—120度了，从机交叉有相撞风险,那么不交叉走位了
			if(_vehicle_id==2)      { _vehicle_id=3; }
			else if(_vehicle_id==3) { _vehicle_id=2; }
			else                    {                }
		}
		//其他 主机小角度转弯 从机正常旋转跟随走位。

		_vel_pre=_master_vel;

	}




	if(_curr_shape==TRIANGLE){//实现三角队形
		
		if(2==_vehicle_id)     {  _param_follow_side=2;  } //2号飞机左后侧
		else if(3==_vehicle_id){  _param_follow_side=3;  } //3号飞机飞右后侧
		else                   {  _param_follow_side=1;  }
	}
	else if(_curr_shape==HORIZONTAL){
		// if(_est_target_vel.length() > 0.8F){ //这个是从机飞左右前方45度角 用来补偿延时的
		// 	if(2==_vehicle_id)     {  _param_follow_side=7;  } //2号飞机左前侧
		// 	else if(3==_vehicle_id){  _param_follow_side=6;  } //3号飞机飞右前侧
		// 	else                   {  _param_follow_side=1;  }
		// }
		// else{
			if(2==_vehicle_id)     {  _param_follow_side=5;  } //2号飞机左侧
			else if(3==_vehicle_id){  _param_follow_side=4;  } //3号飞机飞右侧
			else                   {  _param_follow_side=1;  }
		// }

	}
	else if(_curr_shape==VERTICAL){
		
		if(2==_vehicle_id)     {  _param_follow_side=1; _param_follow_dis=6.0f; } //2号飞机飞后侧 
		else if(3==_vehicle_id){  _param_follow_side=1; _param_follow_dis=12.0f; } //3号飞机飞后侧
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
