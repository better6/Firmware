/****************************************************************************
 *
 *   Copyright (c) 2013-2014 PX4 Development Team. All rights reserved.
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
 * @file Takeoff.cpp
 *
 * Helper class to Takeoff
 *
 * @author Lorenz Meier <lorenz@px4.io
 */

#include "takeoff.h"
#include "navigator.h"

Takeoff::Takeoff(Navigator *navigator) :
	MissionBlock(navigator)
{
}

//1 第一次进来执行的初始化函数 设置takeoff点 以下简称起飞点
void
Takeoff::on_activation()
{
	set_takeoff_position();
}

//2 周期循环的函数：设置默认起飞点、检查起飞有没有完成、没有完成就等着位置执行完成
void
Takeoff::on_active()
{
	struct position_setpoint_triplet_s *rep = _navigator->get_takeoff_triplet();

	//3 初始化起飞点不会进来，在on_activation()已经初始化完成 并在函数的最后清空结构体，这个可以看做是地面站设置的起飞点位置或者其他外部设置的起飞点位置。
	if (rep->current.valid) {
		// reset the position
		set_takeoff_position();
		//mavlink_log_info(_navigator->get_mavlink_log_pub(), "1 set_takeoff_position()");

	}
	//4 检查起飞点有没有达到 完成后原地悬停
	else if (is_mission_item_reached() && !_navigator->get_mission_result()->finished) {
		_navigator->get_mission_result()->finished = true;
		_navigator->set_mission_result_updated();

		// set loiter item so position controllers stop doing takeoff logic
		set_loiter_item(&_mission_item);
		struct position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();
		mission_apply_limitation(_mission_item);
		mission_item_to_position_setpoint(_mission_item, &pos_sp_triplet->current);
		_navigator->set_position_setpoint_triplet_updated();

		//mavlink_log_info(_navigator->get_mavlink_log_pub(), "3 起飞完成进入盘旋");
	}
	//5 如果没有完成 什么也不做，等到位置控制执行完成 进入悬停
	//mavlink_log_info(_navigator->get_mavlink_log_pub(), "2 正在起飞中");
}


//6 切换到takeoff模式时 设置初始化的起飞点
void
Takeoff::set_takeoff_position()
{

	//整个函数主要功能是设置起飞点位置经度纬度高度，过程大概可以分成四步
	//第一步 设置起飞高度，这个高度有三种：外部传参进来的如地面站设置的起飞点，切换模式是默认高度，还有已经飞很高时切换模式时保留当前高度
	//第二步 把航点信息打包到_mission_item结构体中
	//第三步 再把_mission_item结构体 复制到pos_sp_triplet->current，通过这个topic传递给位置控制进行实现起飞点
	//第四部 通知位置控制pos_sp_triplet更新了，你可以控制实现了


	//下面高度逻辑，如果外部发来高度，好啊，但是不能太低。如果没人设置就用飞控自己默认的高度，但是飞机都已经飞得很高了再切takeoff，那就hold吧
	//abs_altitude就是最终确定的起飞高度。

	//rep表示外部传参给的航点信息，如是offboard则在navigator_main.cpp 482行源码里已经给_takeoff_triplet航点赋值，下面的rep拿到已经是填充好了offboard参数的航点，否则拿到的就是一片空闲空间

	struct position_setpoint_triplet_s *rep = _navigator->get_takeoff_triplet();

	float abs_altitude = 0.0f;

	float min_abs_altitude;

	if (_navigator->home_position_valid()) { //only use home position if it is valid
		min_abs_altitude = _navigator->get_global_position()->alt + _navigator->get_takeoff_min_alt();

	} else { //e.g. flow
		min_abs_altitude = _navigator->get_takeoff_min_alt();
	}

	// Use altitude if it has been set. If home position is invalid use min_abs_altitude
	if (rep->current.valid && PX4_ISFINITE(rep->current.alt) && _navigator->home_position_valid()) {
		abs_altitude = rep->current.alt;

		// If the altitude suggestion is lower than home + minimum clearance, raise it and complain.
		if (abs_altitude < min_abs_altitude) {
			if (abs_altitude < min_abs_altitude - 0.1f) { // don't complain if difference is smaller than 10cm
				mavlink_log_critical(_navigator->get_mavlink_log_pub(),
						     "Using minimum takeoff altitude: %.2f m", (double)_navigator->get_takeoff_min_alt());
			}

			abs_altitude = min_abs_altitude;
		}

	} else {
		// Use home + minimum clearance but only notify.
		abs_altitude = min_abs_altitude;
		mavlink_log_info(_navigator->get_mavlink_log_pub(),
				 "Using minimum takeoff altitude: %.2f m", (double)_navigator->get_takeoff_min_alt());
	}


	if (abs_altitude < _navigator->get_global_position()->alt) {
		// If the suggestion is lower than our current alt, let's not go down.
		abs_altitude = _navigator->get_global_position()->alt;
		mavlink_log_critical(_navigator->get_mavlink_log_pub(), "Already higher than takeoff altitude");
	}

	//abs_altitude就是最终确定的起飞高度。高度已经确定，下面应该定经度 维度等其余起飞相关信息。

	//打包航点到_mission_item
	// set current mission item to takeoff
	set_takeoff_item(&_mission_item, abs_altitude);     //取当前航向 经度 维度打包成航点，nav_cmd=NAV_CMD_TAKEOFF
	_navigator->get_mission_result()->finished = false; //任务没有完成
	_navigator->set_mission_result_updated();           //航点已经更新
	reset_mission_item_reached();                       //航点没有达到

	// convert mission item to current setpoint
	struct position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();
	//对航点高度做个限制
	mission_apply_limitation(_mission_item);
	//把航点赋值给pos_sp_triplet->current传递到位置控制进行起飞实现
	mission_item_to_position_setpoint(_mission_item, &pos_sp_triplet->current);
	pos_sp_triplet->previous.valid = false;
	pos_sp_triplet->current.yaw_valid = true;
	pos_sp_triplet->next.valid = false;

	//如果外部传参有效 直接使用外部参数，但是外部设置使用后就会清零
	if (rep->current.valid) {

		// Go on and check which changes had been requested
		if (PX4_ISFINITE(rep->current.yaw)) {
			pos_sp_triplet->current.yaw = rep->current.yaw;
		}

		if (PX4_ISFINITE(rep->current.lat) && PX4_ISFINITE(rep->current.lon)) {
			pos_sp_triplet->current.lat = rep->current.lat;
			pos_sp_triplet->current.lon = rep->current.lon;
		}

		// mark this as done
		memset(rep, 0, sizeof(*rep));
	}
	//完成后可以悬停
	_navigator->set_can_loiter_at_sp(true);
	//通知位置控制已经更新 可以进行控制
	_navigator->set_position_setpoint_triplet_updated();
}
