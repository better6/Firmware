/***************************************************************************
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
 * @file navigator_rtl.h
 * Helper class for RTL
 *
 * @author Julian Oes <julian@oes.ch>
 * @author Anton Babushkin <anton.babushkin@me.com>
 */

#pragma once

#include <px4_module_params.h>

#include "navigator_mode.h"
#include "mission_block.h"

class Navigator;

class RTL : public MissionBlock, public ModuleParams
{
public:
	enum RTLType {
		RTL_HOME = 0,
		RTL_LAND,
		RTL_MISSION,
	};

	RTL(Navigator *navigator);

	~RTL() = default;

	void on_inactive() override;
	void on_activation() override;
	void on_active() override;

	void set_return_alt_min(bool min);

	int rtl_type() const;

private:
	/**
	 * Set the RTL item
	 */
	void		set_rtl_item();

	/**
	 * Move to next RTL item
	 */
	void		advance_rtl();

	enum RTLState {
		RTL_STATE_NONE = 0,         //正常降落的阶段流程
		RTL_STATE_CLIMB,            //1. 先CLIMB爬升到安全高度
		RTL_STATE_RETURN,	        //2. RETURN直接飞往home点上方
		RTL_STATE_TRANSITION_TO_MC, 
		RTL_STATE_DESCEND,          //3. 达到home点上方后开始降落，这段DESCEND是指RTL_RETURN_ALT到到悬停高度RTL_DESCEND_ALT这一段
		RTL_STATE_LOITER,			//4 在悬停高度处 RTL_DESCEND_ALT是否悬停延时
		RTL_STATE_LAND,             //5 降落（落地）
		RTL_STATE_LANDED,           //6 已经落地（加锁）
	} _rtl_state{RTL_STATE_NONE};

	bool _rtl_alt_min{false};

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::RTL_RETURN_ALT>) _param_return_alt,
		(ParamFloat<px4::params::RTL_DESCEND_ALT>) _param_descend_alt,
		(ParamFloat<px4::params::RTL_LAND_DELAY>) _param_land_delay,
		(ParamFloat<px4::params::RTL_MIN_DIST>) _param_rtl_min_dist,
		(ParamInt<px4::params::RTL_TYPE>) _param_rtl_type,
		(ParamInt<px4::params::H0_ENABLE>) _param_h0_enable,
		(ParamInt<px4::params::H1_LAT>) _param_h1_lat,
		(ParamInt<px4::params::H1_LON>) _param_h1_lon,
		(ParamInt<px4::params::H1_ALT>) _param_h1_alt,
		(ParamInt<px4::params::H2_LAT>) _param_h2_lat,
		(ParamInt<px4::params::H2_LON>) _param_h2_lon,
		(ParamInt<px4::params::H2_ALT>) _param_h2_alt,
		(ParamInt<px4::params::H3_LAT>) _param_h3_lat,
		(ParamInt<px4::params::H3_LON>) _param_h3_lon,
		(ParamInt<px4::params::H3_ALT>) _param_h3_alt,
		(ParamInt<px4::params::H4_LAT>) _param_h4_lat,
		(ParamInt<px4::params::H4_LON>) _param_h4_lon,
		(ParamInt<px4::params::H4_ALT>) _param_h4_alt,
		(ParamInt<px4::params::H5_LAT>) _param_h5_lat,
		(ParamInt<px4::params::H5_LON>) _param_h5_lon,
		(ParamInt<px4::params::H5_ALT>) _param_h5_alt
	)
};
