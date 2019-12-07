/****************************************************************************
 *
 *   Copyright (c) 2013-2017 PX4 Development Team. All rights reserved.
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
 * @file navigator_rtl.cpp
 * Helper class to access RTL
 * @author Julian Oes <julian@oes.ch>
 * @author Anton Babushkin <anton.babushkin@me.com>
 */

//总结返航模式的执行过程，其实主要是返航执行过程中阶段的划分（状态机）
//总的状态机下面有我这里只是描述一遍：1.先上升到安全高度  2.飞往home点上方  3.下降到悬停高度  4.是否保持悬停  5.继续下降落地  6.已经落地
//然后这里rtl.cpp就是根据不同的状态机设置不同的期望位置信息，一个状态到位后继续下一阶段的执行
//重点的函数只有一个set_rtl_item();但是并不难。


#include "rtl.h"
#include "navigator.h"

#include <cfloat>

#include <mathlib/mathlib.h>
#include <systemlib/mavlink_log.h>

using math::max;
using math::min;

static constexpr float DELAY_SIGMA = 0.01f;
uint8_t h0_enable=0;
int32_t h1_lat=0;
int32_t h1_lon=0;
int32_t h1_alt=0; 
int32_t h2_lat=0;
int32_t h2_lon=0;
int32_t h2_alt=0;
int32_t h3_lat=0;
int32_t h3_lon=0;
int32_t h3_alt=0;
int32_t h4_lat=0;
int32_t h4_lon=0;
int32_t h4_alt=0;
int32_t h5_lat=0;
int32_t h5_lon=0;
int32_t h5_alt=0;
float dist1=0;
float dist2=0;
float dist3=0;
float dist4=0;
float dist5=0;

RTL::RTL(Navigator *navigator) :
	MissionBlock(navigator),
	ModuleParams(navigator)
{
}

void
RTL::on_inactive()
{
	// reset RTL state
	_rtl_state = RTL_STATE_NONE;
}

int
RTL::rtl_type() const
{
	return _param_rtl_type.get();
}


//第一次切换到返航模式的 初始化函数

void
RTL::on_activation()
{
	if (_navigator->get_land_detected()->landed) {
		// for safety reasons don't go into RTL if landed
		//飞机已经落地 不会在进入rtl模式，注意这是一个状态机，跟踪下看看有几个状态 怎么交替的
		_rtl_state = RTL_STATE_LANDED;

	} else if ((rtl_type() == RTL_LAND) && _navigator->on_mission_landing()) {
		// RTL straight to RETURN state, but mission will takeover for landing

	} else if ((_navigator->get_global_position()->alt < _navigator->get_home_position()->alt + _param_return_alt.get())
		   || _rtl_alt_min) {

		// if lower than return altitude, climb up first
		// if rtl_alt_min is true then forcing altitude change even if above
		_rtl_state = RTL_STATE_CLIMB;

	} else {
		// otherwise go straight to return
		_rtl_state = RTL_STATE_RETURN;
	}


	h0_enable=_param_h0_enable.get();
	
	//注意下面这些参数只是获取了一次 没有实时获取，太占用内存资源 ，没有必要
	h1_lat   = _param_h1_lat.get();
	h1_lon   = _param_h1_lon.get();
	h1_alt   = _param_h1_alt.get();
	// double h1d_lat = h1_lat*1e-7;
	// double h1d_lon = h1_lon*1e-7;
	// float  h1d_alt = h1_alt;
	//warnx("lat=%3.7f  lon=%3.7f  alt=%2.7f ",h1_lat*1e-7,h1_lon*1e-7,(double)h1_alt);


	h2_lat   = _param_h2_lat.get();
	h2_lon   = _param_h2_lon.get();
	h2_alt   = _param_h2_alt.get();

	h3_lat   = _param_h3_lat.get();
	h3_lon   = _param_h3_lon.get();
	h3_alt   = _param_h3_alt.get();

	h4_lat   = _param_h4_lat.get();
	h4_lon   = _param_h4_lon.get();
	h4_alt   = _param_h4_alt.get();

	h5_lat   = _param_h5_lat.get();
	h5_lon   = _param_h5_lon.get();
	h5_alt   = _param_h5_alt.get();

	//根据不同的状态机 初始化设置不同的期望航点信息
	set_rtl_item();
}


//周期执行的函数
//重点的函数就这一个set_rtl_item，根据不同阶段的不同状态机设置期望的位置信息
void
RTL::on_active()
{
	//这个参数实时获取
	h0_enable=_param_h0_enable.get();

	if (_rtl_state != RTL_STATE_LANDED && is_mission_item_reached()) {
		//当rtl还没有落地RTL_STATE_LANDED（还没有结束的时候），当前状态达到后就继续下一阶段的执行
		advance_rtl();
		set_rtl_item();
	}
}

void
RTL::set_return_alt_min(bool min)
{
	_rtl_alt_min = min;
}



//根据返航不同阶段 不同的状态机
//确定当前期望的航点信息

void
RTL::set_rtl_item()
{
	// RTL_TYPE: mission landing
	// landing using planned mission landing, fly to DO_LAND_START instead of returning HOME
	// do nothing, let navigator takeover with mission landing
	if (rtl_type() == RTL_LAND) {
		if (_rtl_state > RTL_STATE_CLIMB) {
			if (_navigator->start_mission_landing()) {
				mavlink_and_console_log_info(_navigator->get_mavlink_log_pub(), "RTL: using mission landing");
				return;

			} else {
				// otherwise use regular RTL
				mavlink_log_critical(_navigator->get_mavlink_log_pub(), "RTL: unable to use mission landing");
			}
		}
	}

	_navigator->set_can_loiter_at_sp(false);

	//获取home点的位置
	//可全局搜索HOME点来源一 返航主要返航到HOME点，home点来源与哪
	 home_position_s &home = *_navigator->get_home_position();
	//获取gps坐标
	const vehicle_global_position_s &gpos = *_navigator->get_global_position();

	//开启了备选降落点
	if(h0_enable==1){
		//如果开启了备降点，会不会开启了备降点 但是正常的切换rtl 电压还没到呢，会所以在这里还要检测电压
		//如果开启了备降点判断当前位置距离哪个近
		dist1=get_distance_to_next_waypoint(gpos.lat, gpos.lon, h1_lat*1e-7, h1_lon*1e-7);
		dist2=get_distance_to_next_waypoint(gpos.lat, gpos.lon, h2_lat*1e-7, h2_lon*1e-7);
		dist3=get_distance_to_next_waypoint(gpos.lat, gpos.lon, h3_lat*1e-7, h3_lon*1e-7);
		dist4=get_distance_to_next_waypoint(gpos.lat, gpos.lon, h4_lat*1e-7, h4_lon*1e-7);
		dist5=get_distance_to_next_waypoint(gpos.lat, gpos.lon, h5_lat*1e-7, h5_lon*1e-7);
		warnx("dist1 = %6.6f",(double)dist1);
		warnx("dist2 = %6.6f",(double)dist2);
		warnx("dist3 = %6.6f",(double)dist3);
		warnx("dist4 = %6.6f",(double)dist4);
		warnx("dist5 = %6.6f",(double)dist5);
	
		float range=dist1;
		uint8_t point=0;

		if(range > dist2){  range=dist2; point=2; home.lat=(double)(h2_lat*1e-7);  home.lon=(double)(h2_lon*1e-7);  home.alt=(float)(h2_alt);}
		else             {  range=dist1; point=1; home.lat=h1_lat*1e-7;  home.lon=h1_lon*1e-7;  home.alt=h1_alt;}
		if(range > dist3){  range=dist3; point=3; home.lat=h3_lat*1e-7;  home.lon=h3_lon*1e-7;  home.alt=h3_alt;}
		if(range > dist4){  range=dist4; point=4; home.lat=h4_lat*1e-7;  home.lon=h4_lon*1e-7;  home.alt=h4_alt;}
		if(range > dist5){  range=dist5; point=5; home.lat=h5_lat*1e-7;  home.lon=h5_lon*1e-7;  home.alt=h5_alt;}
		warnx("point=%d",point);
		mavlink_and_console_log_info(_navigator->get_mavlink_log_pub(), "#低电压返航降落到%d号备降点",point);
	
	}
	//获取航点信息
	position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();

	// check if we are pretty close to home already
	//检查现在的位置是不是已经距离home很近了，很近了不需要上升安全高度可以直接降落
	const float home_dist = get_distance_to_next_waypoint(home.lat, home.lon, gpos.lat, gpos.lon);

	// compute the return altitude
	//如果当前已经大于返航高度就以当前高度返航，如果当前高度不高 那就上升到安全高度再返航！
	float return_alt = max(home.alt + _param_return_alt.get(), gpos.alt);

	// we are close to home, limit climb to min
	//距离home很近的时候 飞机不会再上升到安全高度RTL_RETURN_ALT进行返航，而是相对升高点RTL_DESCEND_ALT进行返航
	if (home_dist < _param_rtl_min_dist.get()) {
		return_alt = home.alt + _param_descend_alt.get();
	}

	// compute the loiter altitude
	//盘旋高度
	const float loiter_altitude = min(home.alt + _param_descend_alt.get(), gpos.alt);


		


	


//	 get_distance_to_next_waypoint(gpos.lat, gpos.lon, _mission_item.lat, _mission_item.lon);


	




	//上面都是在进行初始化取参数，
	//下面开始根据状态机进行动作
	// //	enum RTLState {
	// // 	RTL_STATE_NONE = 0,         //正常降落的阶段流程
	// // 	RTL_STATE_CLIMB,            //1. 先CLIMB爬升到安全高度
	// // 	RTL_STATE_RETURN,	        //2. RETURN直接飞往home点上方
	// // 	RTL_STATE_TRANSITION_TO_MC, 
	// // 	RTL_STATE_DESCEND,          //3. 达到home点上方后开始降落，这段DESCEND是指RTL_RETURN_ALT到到悬停高度RTL_DESCEND_ALT这一段
	// // 	RTL_STATE_LOITER,			//4 在悬停高度处 RTL_DESCEND_ALT是否悬停延时
	// // 	RTL_STATE_LAND,             //5 降落（落地）
	// // 	RTL_STATE_LANDED,           //6 已经落地（加锁）
	// // } _rtl_state{RTL_STATE_NONE};

	switch (_rtl_state) {
	//1. 爬升到安全高度阶段
	case RTL_STATE_CLIMB: {

			_mission_item.nav_cmd = NAV_CMD_WAYPOINT; //普通的waypoint
			_mission_item.lat = gpos.lat;             //当前的经度纬度
			_mission_item.lon = gpos.lon;
			_mission_item.altitude = return_alt;      //爬升到“安全高度”，这个在上面有分三种情况讨论
			_mission_item.altitude_is_relative = false;//使用的不是相对高度，是绝对海拔高度
			_mission_item.yaw = NAN;                   //上升保持航向
			_mission_item.acceptance_radius = _navigator->get_acceptance_radius();//达到判断：可接受的达到半径
			_mission_item.time_inside = 0.0f; 		   //达到这个点不悬停延时
			_mission_item.autocontinue = true;         //执行完这个点 继续执行下一个点
			_mission_item.origin = ORIGIN_ONBOARD;     //航点来源onboard飞控内部产生的

			mavlink_and_console_log_info(_navigator->get_mavlink_log_pub(), "RTL: climb to %d m (%d m above home)",
						     (int)ceilf(return_alt), (int)ceilf(return_alt - _navigator->get_home_position()->alt));
			break;
		}
	
	//2. 爬升到安全高度后 就可以执行返航了RETURN,直线飞往home点上方
	case RTL_STATE_RETURN: {

			// don't change altitude
			_mission_item.nav_cmd = NAV_CMD_WAYPOINT; //普通航点
			_mission_item.lat = home.lat;             //飞往home上方 
			_mission_item.lon = home.lon;
			_mission_item.altitude = return_alt;
			_mission_item.altitude_is_relative = false;

			// use home yaw if close to home
			/* check if we are pretty close to home already */
			if (home_dist < _param_rtl_min_dist.get()) {
				_mission_item.yaw = home.yaw;

			} else {
				// use current heading to home  //机头指向home点
				_mission_item.yaw = get_bearing_to_next_waypoint(gpos.lat, gpos.lon, home.lat, home.lon);
			}

			_mission_item.acceptance_radius = _navigator->get_acceptance_radius();
			_mission_item.time_inside = 0.0f;
			_mission_item.autocontinue = true;
			_mission_item.origin = ORIGIN_ONBOARD;

			mavlink_and_console_log_info(_navigator->get_mavlink_log_pub(), "RTL: return at %d m (%d m above home)",
						     (int)ceilf(_mission_item.altitude), (int)ceilf(_mission_item.altitude - home.alt));

			break;
		}
	//
	case RTL_STATE_TRANSITION_TO_MC: {
			set_vtol_transition_item(&_mission_item, vtol_vehicle_status_s::VEHICLE_VTOL_STATE_MC);
			break;
		}

	//3 到达home上方以后开始进行降落，这一段DESCEND是只降落到悬停高度RTL_DESCEND_ALT
	case RTL_STATE_DESCEND: {
			_mission_item.nav_cmd = NAV_CMD_WAYPOINT;
			_mission_item.lat = home.lat;
			_mission_item.lon = home.lon;
			_mission_item.altitude = loiter_altitude;
			_mission_item.altitude_is_relative = false;

			// except for vtol which might be still off here and should point towards this location
			const float d_current = get_distance_to_next_waypoint(gpos.lat, gpos.lon, _mission_item.lat, _mission_item.lon);

			if (_navigator->get_vstatus()->is_vtol && (d_current > _navigator->get_acceptance_radius())) {
				_mission_item.yaw = get_bearing_to_next_waypoint(gpos.lat, gpos.lon, _mission_item.lat, _mission_item.lon);

			} else {
				_mission_item.yaw = home.yaw;
			}

			_mission_item.acceptance_radius = _navigator->get_acceptance_radius();
			_mission_item.time_inside = 0.0f;
			_mission_item.autocontinue = true;
			_mission_item.origin = ORIGIN_ONBOARD;

			/* disable previous setpoint to prevent drift */
			pos_sp_triplet->previous.valid = false;

			mavlink_and_console_log_info(_navigator->get_mavlink_log_pub(), "RTL: descend to %d m (%d m above home)",
						     (int)ceilf(_mission_item.altitude), (int)ceilf(_mission_item.altitude - home.alt));
			break;
		}

	//4 降落到悬停高度 判断是否悬停，悬停多久（参数RTL_LAND_DELAY在此处悬停多久）
	case RTL_STATE_LOITER: {
			const bool autoland = (_param_land_delay.get() > FLT_EPSILON);

			// don't change altitude
			_mission_item.lat = home.lat;
			_mission_item.lon = home.lon;
			_mission_item.altitude = loiter_altitude;
			_mission_item.altitude_is_relative = false;
			_mission_item.yaw = home.yaw;
			_mission_item.loiter_radius = _navigator->get_loiter_radius();
			_mission_item.acceptance_radius = _navigator->get_acceptance_radius();
			_mission_item.time_inside = max(_param_land_delay.get(), 0.0f);
			_mission_item.autocontinue = autoland;
			_mission_item.origin = ORIGIN_ONBOARD;

			_navigator->set_can_loiter_at_sp(true);

			if (autoland && (get_time_inside(_mission_item) > FLT_EPSILON)) {
				_mission_item.nav_cmd = NAV_CMD_LOITER_TIME_LIMIT;
				mavlink_and_console_log_info(_navigator->get_mavlink_log_pub(), "RTL: loiter %.1fs",
							     (double)get_time_inside(_mission_item));

			} else {
				_mission_item.nav_cmd = NAV_CMD_LOITER_UNLIMITED;
				mavlink_and_console_log_info(_navigator->get_mavlink_log_pub(), "RTL: completed, loitering");
			}

			break;
		}
	
	//5 进行落地
	case RTL_STATE_LAND: {
			// land at home position
			_mission_item.nav_cmd = NAV_CMD_LAND;
			_mission_item.lat = home.lat;
			_mission_item.lon = home.lon;
			_mission_item.yaw = home.yaw;
			_mission_item.altitude = home.alt;
			_mission_item.altitude_is_relative = false;
			_mission_item.acceptance_radius = _navigator->get_acceptance_radius();
			_mission_item.time_inside = 0.0f;
			_mission_item.autocontinue = true;
			_mission_item.origin = ORIGIN_ONBOARD;

			mavlink_and_console_log_info(_navigator->get_mavlink_log_pub(), "RTL: land at home");
			break;
		}

	//6 已经落地
	case RTL_STATE_LANDED: {
			set_idle_item(&_mission_item);
			set_return_alt_min(false);
			break;
		}

	default:
		break;
	}

	reset_mission_item_reached();

	/* execute command if set. This is required for commands like VTOL transition */
	if (!item_contains_position(_mission_item)) {
		issue_command(_mission_item);
	}

	/* convert mission item to current position setpoint and make it valid */
	mission_apply_limitation(_mission_item);

	if (mission_item_to_position_setpoint(_mission_item, &pos_sp_triplet->current)) {
		_navigator->set_position_setpoint_triplet_updated();
	}
}

void
RTL::advance_rtl()
{
	switch (_rtl_state) {
	case RTL_STATE_CLIMB:
		_rtl_state = RTL_STATE_RETURN;
		break;

	case RTL_STATE_RETURN:
		_rtl_state = RTL_STATE_DESCEND;

		if (_navigator->get_vstatus()->is_vtol && !_navigator->get_vstatus()->is_rotary_wing) {
			_rtl_state = RTL_STATE_TRANSITION_TO_MC;
		}

		break;

	case RTL_STATE_TRANSITION_TO_MC:
		_rtl_state = RTL_STATE_RETURN;
		break;

	case RTL_STATE_DESCEND:

		/* only go to land if autoland is enabled */
		if (_param_land_delay.get() < -DELAY_SIGMA || _param_land_delay.get() > DELAY_SIGMA) {
			_rtl_state = RTL_STATE_LOITER;

		} else {
			_rtl_state = RTL_STATE_LAND;
		}

		break;

	case RTL_STATE_LOITER:
		_rtl_state = RTL_STATE_LAND;
		break;

	case RTL_STATE_LAND:
		_rtl_state = RTL_STATE_LANDED;
		break;

	default:
		break;
	}
}
