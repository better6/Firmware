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
 * FOR A PBRTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
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

#include "FixedwingPositionControl.hpp"

extern "C" __EXPORT int fw_pos_control_l1_main(int argc, char *argv[]);

FixedwingPositionControl::FixedwingPositionControl() :
    ModuleParams(nullptr),
    _sub_airspeed(ORB_ID(airspeed)),
    _sub_sensors(ORB_ID(sensor_bias)),
    _loop_perf(perf_alloc(PC_ELAPSED, "fw l1 control")),
    _launchDetector(this),
    _runway_takeoff(this)
{
    _parameter_handles.l1_period = param_find("FW_L1_PERIOD");
    _parameter_handles.l1_damping = param_find("FW_L1_DAMPING");

    _parameter_handles.airspeed_min = param_find("FW_AIRSPD_MIN");
    _parameter_handles.airspeed_trim = param_find("FW_AIRSPD_TRIM");
    _parameter_handles.airspeed_max = param_find("FW_AIRSPD_MAX");
    _parameter_handles.airspeed_disabled = param_find("FW_ARSP_MODE");

    _parameter_handles.pitch_limit_min = param_find("FW_P_LIM_MIN");
    _parameter_handles.pitch_limit_max = param_find("FW_P_LIM_MAX");
    _parameter_handles.roll_limit = param_find("FW_R_LIM");
    _parameter_handles.throttle_min = param_find("FW_THR_MIN");
    _parameter_handles.throttle_max = param_find("FW_THR_MAX");
    _parameter_handles.throttle_idle = param_find("FW_THR_IDLE");
    _parameter_handles.throttle_slew_max = param_find("FW_THR_SLEW_MAX");
    _parameter_handles.throttle_cruise = param_find("FW_THR_CRUISE");
    _parameter_handles.throttle_alt_scale = param_find("FW_THR_ALT_SCL");
    _parameter_handles.throttle_land_max = param_find("FW_THR_LND_MAX");
    _parameter_handles.man_roll_max_deg = param_find("FW_MAN_R_MAX");
    _parameter_handles.man_pitch_max_deg = param_find("FW_MAN_P_MAX");
    _parameter_handles.rollsp_offset_deg = param_find("FW_RSP_OFF");
    _parameter_handles.pitchsp_offset_deg = param_find("FW_PSP_OFF");

    _parameter_handles.land_slope_angle = param_find("FW_LND_ANG");
    _parameter_handles.land_slope_angle1 = param_find("FORMA_PARAM1");
    _parameter_handles.land_slope_angle2 = param_find("FORMA_PARAM2");
    _parameter_handles.land_slope_angle3 = param_find("FORMA_PARAM3");
    _parameter_handles.land_H1_virt = param_find("FW_LND_HVIRT");
    _parameter_handles.land_flare_alt_relative = param_find("FW_LND_FLALT");
    _parameter_handles.land_flare_pitch_min_deg = param_find("FW_LND_FL_PMIN");
    _parameter_handles.land_flare_pitch_max_deg = param_find("FW_LND_FL_PMAX");
    _parameter_handles.land_thrust_lim_alt_relative = param_find("FW_LND_TLALT");
    _parameter_handles.land_heading_hold_horizontal_distance = param_find("FW_LND_HHDIST");
    _parameter_handles.land_use_terrain_estimate = param_find("FW_LND_USETER");
    _parameter_handles.land_airspeed_scale = param_find("FW_LND_AIRSPD_SC");

    _parameter_handles.time_const = param_find("FW_T_TIME_CONST");
    _parameter_handles.time_const_throt = param_find("FW_T_THRO_CONST");
    _parameter_handles.min_sink_rate = param_find("FW_T_SINK_MIN");
    _parameter_handles.max_sink_rate = param_find("FW_T_SINK_MAX");
    _parameter_handles.max_climb_rate = param_find("FW_T_CLMB_MAX");
    _parameter_handles.climbout_diff = param_find("FW_CLMBOUT_DIFF");
    _parameter_handles.throttle_damp = param_find("FW_T_THR_DAMP");
    _parameter_handles.integrator_gain = param_find("FW_T_INTEG_GAIN");
    _parameter_handles.vertical_accel_limit = param_find("FW_T_VERT_ACC");
    _parameter_handles.height_comp_filter_omega = param_find("FW_T_HGT_OMEGA");
    _parameter_handles.speed_comp_filter_omega = param_find("FW_T_SPD_OMEGA");
    _parameter_handles.roll_throttle_compensation = param_find("FW_T_RLL2THR");
    _parameter_handles.speed_weight = param_find("FW_T_SPDWEIGHT");
    _parameter_handles.pitch_damping = param_find("FW_T_PTCH_DAMP");
    _parameter_handles.heightrate_p = param_find("FW_T_HRATE_P");
    _parameter_handles.heightrate_ff = param_find("FW_T_HRATE_FF");
    _parameter_handles.speedrate_p = param_find("FW_T_SRATE_P");

    // if vehicle is vtol these handles will be set when we get the vehicle status
    _parameter_handles.airspeed_trans = PARAM_INVALID;
    _parameter_handles.vtol_type = PARAM_INVALID;

    // initialize to invalid vtol type
    _parameters.vtol_type = -1;

    /* fetch initial parameter values */
    parameters_update();
}

FixedwingPositionControl::~FixedwingPositionControl()
{
    perf_free(_loop_perf);
}

int
FixedwingPositionControl::parameters_update()
{
    updateParams();

    param_get(_parameter_handles.airspeed_min, &(_parameters.airspeed_min));
    param_get(_parameter_handles.airspeed_trim, &(_parameters.airspeed_trim));
    param_get(_parameter_handles.airspeed_max, &(_parameters.airspeed_max));
    param_get(_parameter_handles.airspeed_disabled, &(_parameters.airspeed_disabled));

    param_get(_parameter_handles.pitch_limit_min, &(_parameters.pitch_limit_min));
    param_get(_parameter_handles.pitch_limit_max, &(_parameters.pitch_limit_max));
    param_get(_parameter_handles.throttle_min, &(_parameters.throttle_min));
    param_get(_parameter_handles.throttle_max, &(_parameters.throttle_max));
    param_get(_parameter_handles.throttle_idle, &(_parameters.throttle_idle));
    param_get(_parameter_handles.throttle_cruise, &(_parameters.throttle_cruise));
    param_get(_parameter_handles.throttle_alt_scale, &(_parameters.throttle_alt_scale));
    param_get(_parameter_handles.throttle_land_max, &(_parameters.throttle_land_max));

    param_get(_parameter_handles.man_roll_max_deg, &_parameters.man_roll_max_rad);
    _parameters.man_roll_max_rad = radians(_parameters.man_roll_max_rad);
    param_get(_parameter_handles.man_pitch_max_deg, &_parameters.man_pitch_max_rad);
    _parameters.man_pitch_max_rad = radians(_parameters.man_pitch_max_rad);

    param_get(_parameter_handles.rollsp_offset_deg, &_parameters.rollsp_offset_rad);
    _parameters.rollsp_offset_rad = radians(_parameters.rollsp_offset_rad);
    param_get(_parameter_handles.pitchsp_offset_deg, &_parameters.pitchsp_offset_rad);
    _parameters.pitchsp_offset_rad = radians(_parameters.pitchsp_offset_rad);

    param_get(_parameter_handles.climbout_diff, &(_parameters.climbout_diff));

    param_get(_parameter_handles.land_heading_hold_horizontal_distance,
              &(_parameters.land_heading_hold_horizontal_distance));
    param_get(_parameter_handles.land_flare_pitch_min_deg, &(_parameters.land_flare_pitch_min_deg));
    param_get(_parameter_handles.land_flare_pitch_max_deg, &(_parameters.land_flare_pitch_max_deg));
    param_get(_parameter_handles.land_use_terrain_estimate, &(_parameters.land_use_terrain_estimate));
    param_get(_parameter_handles.land_airspeed_scale, &(_parameters.land_airspeed_scale));

    // VTOL parameter VTOL_TYPE
    if (_parameter_handles.vtol_type != PARAM_INVALID) {
        param_get(_parameter_handles.vtol_type, &_parameters.vtol_type);
    }

    // VTOL parameter VT_ARSP_TRANS
    if (_parameter_handles.airspeed_trans != PARAM_INVALID) {
        param_get(_parameter_handles.airspeed_trans, &_parameters.airspeed_trans);
    }


    float v = 0.0f;


    // L1 control parameters

    if (param_get(_parameter_handles.l1_damping, &v) == PX4_OK) {
        _l1_control.set_l1_damping(v);
    }

    if (param_get(_parameter_handles.l1_period, &v) == PX4_OK) {
        _l1_control.set_l1_period(v);
    }

    if (param_get(_parameter_handles.roll_limit, &v) == PX4_OK) {
        _l1_control.set_l1_roll_limit(radians(v));
    }


    // TECS parameters

    param_get(_parameter_handles.max_climb_rate, &(_parameters.max_climb_rate));
    _tecs.set_max_climb_rate(_parameters.max_climb_rate);

    param_get(_parameter_handles.max_sink_rate, &(_parameters.max_sink_rate));
    _tecs.set_max_sink_rate(_parameters.max_sink_rate);

    param_get(_parameter_handles.speed_weight, &(_parameters.speed_weight));
    _tecs.set_speed_weight(_parameters.speed_weight);

    _tecs.set_indicated_airspeed_min(_parameters.airspeed_min);
    _tecs.set_indicated_airspeed_max(_parameters.airspeed_max);

    if (param_get(_parameter_handles.time_const, &v) == PX4_OK) {
        _tecs.set_time_const(v);
    }

    if (param_get(_parameter_handles.time_const_throt, &v) == PX4_OK) {
        _tecs.set_time_const_throt(v);
    }

    if (param_get(_parameter_handles.min_sink_rate, &v) == PX4_OK) {
        _tecs.set_min_sink_rate(v);
    }

    if (param_get(_parameter_handles.throttle_damp, &v) == PX4_OK) {
        _tecs.set_throttle_damp(v);
    }

    if (param_get(_parameter_handles.integrator_gain, &v) == PX4_OK) {
        _tecs.set_integrator_gain(v);
    }

    if (param_get(_parameter_handles.throttle_slew_max, &v) == PX4_OK) {
        _tecs.set_throttle_slewrate(v);
    }

    if (param_get(_parameter_handles.vertical_accel_limit, &v) == PX4_OK) {
        _tecs.set_vertical_accel_limit(v);
    }

    if (param_get(_parameter_handles.height_comp_filter_omega, &v) == PX4_OK) {
        _tecs.set_height_comp_filter_omega(v);
    }

    if (param_get(_parameter_handles.speed_comp_filter_omega, &v) == PX4_OK) {
        _tecs.set_speed_comp_filter_omega(v);
    }

    if (param_get(_parameter_handles.roll_throttle_compensation, &v) == PX4_OK) {
        _tecs.set_roll_throttle_compensation(v);
    }

    if (param_get(_parameter_handles.pitch_damping, &v) == PX4_OK) {
        _tecs.set_pitch_damping(v);
    }

    if (param_get(_parameter_handles.heightrate_p, &v) == PX4_OK) {
        _tecs.set_heightrate_p(v);
    }

    if (param_get(_parameter_handles.heightrate_ff, &v) == PX4_OK) {
        _tecs.set_heightrate_ff(v);
    }

    if (param_get(_parameter_handles.speedrate_p, &v) == PX4_OK) {
        _tecs.set_speedrate_p(v);
    }


    // Landing slope

    float land_slope_angle = 0.0f;
    param_get(_parameter_handles.land_slope_angle, &land_slope_angle);
    

        float land_slope_angle1 = 0.0f;
    param_get(_parameter_handles.land_slope_angle1, &land_slope_angle1);
    //warnx("---angle1=%2.1f",(double)land_slope_angle1);

        float land_slope_angle2 = 0.0f;
    param_get(_parameter_handles.land_slope_angle2, &land_slope_angle2);
   // warnx("----angle2=%2.1f",(double)land_slope_angle2);

        float land_slope_angle3 = 0.0f;
    param_get(_parameter_handles.land_slope_angle3, &land_slope_angle3);
   // warnx("---angle3=%2.1f",(double)land_slope_angle3);

    float land_flare_alt_relative = 0.0f;
    param_get(_parameter_handles.land_flare_alt_relative, &land_flare_alt_relative);

    float land_thrust_lim_alt_relative = 0.0f;
    param_get(_parameter_handles.land_thrust_lim_alt_relative, &land_thrust_lim_alt_relative);

    float land_H1_virt = 0.0f;
    param_get(_parameter_handles.land_H1_virt, &land_H1_virt);

    /* check if negative value for 2/3 of flare altitude is set for throttle cut */
    if (land_thrust_lim_alt_relative < 0.0f) {
        land_thrust_lim_alt_relative = 0.66f * land_flare_alt_relative;
    }

    _landingslope.update(radians(land_slope_angle), land_flare_alt_relative, land_thrust_lim_alt_relative, land_H1_virt);

   // mavlink_log_info(&_mavlink_log_pub,"0=%2.1f  1=%2.1f 1=%2.1f 1=%2.1f  ",(double)land_slope_angle,(double)land_slope_angle1,(double)land_slope_angle2,(double)land_slope_angle3);



    // Update and publish the navigation capabilities
    _fw_pos_ctrl_status.landing_slope_angle_rad = _landingslope.landing_slope_angle_rad();
    _fw_pos_ctrl_status.landing_horizontal_slope_displacement = _landingslope.horizontal_slope_displacement();
    _fw_pos_ctrl_status.landing_flare_length = _landingslope.flare_length();
    fw_pos_ctrl_status_publish();


    // sanity check parameters
    if ((_parameters.airspeed_max < _parameters.airspeed_min) ||
            (_parameters.airspeed_max < 5.0f) ||
            (_parameters.airspeed_min > 100.0f) ||
            (_parameters.airspeed_trim < _parameters.airspeed_min) ||
            (_parameters.airspeed_trim > _parameters.airspeed_max)) {

        mavlink_log_critical(&_mavlink_log_pub, "Airspeed parameters invalid");

        return PX4_ERROR;
    }

    return PX4_OK;
}

void
FixedwingPositionControl::vehicle_control_mode_poll()
{
    bool updated;

    orb_check(_control_mode_sub, &updated);

    if (updated) {
        orb_copy(ORB_ID(vehicle_control_mode), _control_mode_sub, &_control_mode);
    }
}

void
FixedwingPositionControl::vehicle_command_poll()
{
    bool updated;

    orb_check(_vehicle_command_sub, &updated);

    if (updated) {
        orb_copy(ORB_ID(vehicle_command), _vehicle_command_sub, &_vehicle_command);
        handle_command();
    }
}

void
FixedwingPositionControl::home_position_update(bool force)
{
    bool updated = false;
    orb_check(_home_pos_sub, &updated);

    if (updated || force) {
        orb_copy(ORB_ID(home_position), _home_pos_sub, &_home_pos);
    }
}


void
FixedwingPositionControl::vehicle_status_poll()
{
    bool updated;

    orb_check(_vehicle_status_sub, &updated);

    if (updated) {
        orb_copy(ORB_ID(vehicle_status), _vehicle_status_sub, &_vehicle_status);

        /* set correct uORB ID, depending on if vehicle is VTOL or not */
        if (_attitude_setpoint_id == nullptr) {
            if (_vehicle_status.is_vtol) {
                _attitude_setpoint_id = ORB_ID(fw_virtual_attitude_setpoint);

                _parameter_handles.airspeed_trans = param_find("VT_ARSP_TRANS");
                _parameter_handles.vtol_type = param_find("VT_TYPE");

                parameters_update();

            } else {
                _attitude_setpoint_id = ORB_ID(vehicle_attitude_setpoint);
            }
        }
    }
}

void
FixedwingPositionControl::vehicle_land_detected_poll()
{
    bool updated;

    orb_check(_vehicle_land_detected_sub, &updated);

    if (updated) {
        orb_copy(ORB_ID(vehicle_land_detected), _vehicle_land_detected_sub, &_vehicle_land_detected);
    }
}

void
FixedwingPositionControl::manual_control_setpoint_poll()
{
    bool manual_updated;

    /* Check if manual setpoint has changed */
    orb_check(_manual_control_sub, &manual_updated);

    if (manual_updated) {
        orb_copy(ORB_ID(manual_control_setpoint), _manual_control_sub, &_manual);
    }
}
void
FixedwingPositionControl::airspeed_poll()
{
    if (!_parameters.airspeed_disabled && _sub_airspeed.updated()) {
        _sub_airspeed.update();
        _airspeed_valid = PX4_ISFINITE(_sub_airspeed.get().indicated_airspeed_m_s)
                && PX4_ISFINITE(_sub_airspeed.get().true_airspeed_m_s);
        _airspeed_last_received = hrt_absolute_time();
        _airspeed = _sub_airspeed.get().indicated_airspeed_m_s;

        if (_sub_airspeed.get().indicated_airspeed_m_s > 0.0f
                && _sub_airspeed.get().true_airspeed_m_s > _sub_airspeed.get().indicated_airspeed_m_s) {
            _eas2tas = max(_sub_airspeed.get().true_airspeed_m_s / _sub_airspeed.get().indicated_airspeed_m_s, 1.0f);

        } else {
            _eas2tas = 1.0f;
        }

    } else {
        /* no airspeed updates for one second */
        if (_airspeed_valid && (hrt_absolute_time() - _airspeed_last_received) > 1e6) {
            _airspeed_valid = false;
        }
    }

    /* update TECS state */
    _tecs.enable_airspeed(_airspeed_valid);
}

void
FixedwingPositionControl::vehicle_attitude_poll()
{
    /* check if there is a new position */
    bool updated;
    orb_check(_vehicle_attitude_sub, &updated);

    if (updated) {
        orb_copy(ORB_ID(vehicle_attitude), _vehicle_attitude_sub, &_att);
    }

    /* set rotation matrix and euler angles */
    _R_nb = Quatf(_att.q);

    // if the vehicle is a tailsitter we have to rotate the attitude by the pitch offset
    // between multirotor and fixed wing flight
    if (_parameters.vtol_type == vtol_type::TAILSITTER && _vehicle_status.is_vtol) {
        Dcmf R_offset = Eulerf(0, M_PI_2_F, 0);
        _R_nb = _R_nb * R_offset;
    }

    Eulerf euler_angles(_R_nb);
    _roll    = euler_angles(0);
    _pitch   = euler_angles(1);
    _yaw     = euler_angles(2);
}

void
FixedwingPositionControl::position_setpoint_triplet_poll()
{
    /* check if there is a new setpoint */
    bool pos_sp_triplet_updated;
    orb_check(_pos_sp_triplet_sub, &pos_sp_triplet_updated);

    if (pos_sp_triplet_updated) {
        orb_copy(ORB_ID(position_setpoint_triplet), _pos_sp_triplet_sub, &_pos_sp_triplet);
    }
}

float
FixedwingPositionControl::get_demanded_airspeed()
{
    float altctrl_airspeed = 0;

    // neutral throttle corresponds to trim airspeed
    if (_manual.z < 0.5f) {
        // lower half of throttle is min to trim airspeed
        altctrl_airspeed = _parameters.airspeed_min +
                (_parameters.airspeed_trim - _parameters.airspeed_min) *
                _manual.z * 2;

    } else {
        // upper half of throttle is trim to max airspeed
        altctrl_airspeed = _parameters.airspeed_trim +
                (_parameters.airspeed_max - _parameters.airspeed_trim) *
                (_manual.z * 2 - 1);
    }

    return altctrl_airspeed;
}

float
FixedwingPositionControl::calculate_target_airspeed(float airspeed_demand)
{
    /*
     * Calculate accelerated stall airspeed factor from commanded bank angle and use it to increase minimum airspeed.
     *
     *  We don't know the stall speed of the aircraft, but assuming user defined
     *  minimum airspeed (FW_AIRSPD_MIN) is slightly larger than stall speed
     *  this is close enough.
     *
     * increase lift vector to balance additional weight in bank
     *  cos(bank angle) = W/L = 1/n
     *   n is the load factor
     *
     * lift is proportional to airspeed^2 so the increase in stall speed is
     *  Vsacc = Vs * sqrt(n)
     *
     */
    float adjusted_min_airspeed = _parameters.airspeed_min;

    if (_airspeed_valid && PX4_ISFINITE(_att_sp.roll_body)) {

        adjusted_min_airspeed = constrain(_parameters.airspeed_min / sqrtf(cosf(_att_sp.roll_body)), _parameters.airspeed_min,
                                          _parameters.airspeed_max);
    }

    // add minimum ground speed undershoot (only non-zero in presence of sufficient wind)
    // sanity check: limit to range
    return constrain(airspeed_demand + _groundspeed_undershoot, adjusted_min_airspeed, _parameters.airspeed_max);
}

void
FixedwingPositionControl::calculate_gndspeed_undershoot(const Vector2f &curr_pos,
                                                        const Vector2f &ground_speed,
                                                        const position_setpoint_s &pos_sp_prev, const position_setpoint_s &pos_sp_curr)
{
    if (pos_sp_curr.valid && !_l1_control.circle_mode()) {
        /* rotate ground speed vector with current attitude */
        Vector2f yaw_vector(_R_nb(0, 0), _R_nb(1, 0));
        yaw_vector.normalize();
        float ground_speed_body = yaw_vector * ground_speed;

        /* The minimum desired ground speed is the minimum airspeed projected on to the ground using the altitude and horizontal difference between the waypoints if available*/
        float distance = 0.0f;
        float delta_altitude = 0.0f;

        if (pos_sp_prev.valid) {
            distance = get_distance_to_next_waypoint(pos_sp_prev.lat, pos_sp_prev.lon, pos_sp_curr.lat, pos_sp_curr.lon);
            delta_altitude = pos_sp_curr.alt - pos_sp_prev.alt;

        } else {
            distance = get_distance_to_next_waypoint(curr_pos(0), curr_pos(1), pos_sp_curr.lat, pos_sp_curr.lon);
            delta_altitude = pos_sp_curr.alt - _global_pos.alt;
        }

        float ground_speed_desired = _parameters.airspeed_min * cosf(atan2f(delta_altitude, distance));

        /*
         * Ground speed undershoot is the amount of ground velocity not reached
         * by the plane. Consequently it is zero if airspeed is >= min ground speed
         * and positive if airspeed < min ground speed.
         *
         * This error value ensures that a plane (as long as its throttle capability is
         * not exceeded) travels towards a waypoint (and is not pushed more and more away
         * by wind). Not countering this would lead to a fly-away.
         */
        _groundspeed_undershoot = max(ground_speed_desired - ground_speed_body, 0.0f);

    } else {
        _groundspeed_undershoot = 0.0f;
    }
}

void
FixedwingPositionControl::fw_pos_ctrl_status_publish()
{
    _fw_pos_ctrl_status.timestamp = hrt_absolute_time();

    if (_fw_pos_ctrl_status_pub != nullptr) {
        orb_publish(ORB_ID(fw_pos_ctrl_status), _fw_pos_ctrl_status_pub, &_fw_pos_ctrl_status);

    } else {
        _fw_pos_ctrl_status_pub = orb_advertise(ORB_ID(fw_pos_ctrl_status), &_fw_pos_ctrl_status);
    }
}

void
FixedwingPositionControl::get_waypoint_heading_distance(float heading, position_setpoint_s &waypoint_prev,
                                                        position_setpoint_s &waypoint_next, bool flag_init)
{
    position_setpoint_s temp_prev = waypoint_prev;
    position_setpoint_s temp_next = waypoint_next;

    if (flag_init) {
        // previous waypoint: HDG_HOLD_SET_BACK_DIST meters behind us
        waypoint_from_heading_and_distance(_global_pos.lat, _global_pos.lon, heading + radians(180.0f),
                                           HDG_HOLD_SET_BACK_DIST, &temp_prev.lat, &temp_prev.lon);

        // next waypoint: HDG_HOLD_DIST_NEXT meters in front of us
        waypoint_from_heading_and_distance(_global_pos.lat, _global_pos.lon, heading,
                                           HDG_HOLD_DIST_NEXT, &temp_next.lat, &temp_next.lon);

    } else {
        // use the existing flight path from prev to next

        // previous waypoint: shifted HDG_HOLD_REACHED_DIST + HDG_HOLD_SET_BACK_DIST
        create_waypoint_from_line_and_dist(waypoint_next.lat, waypoint_next.lon, waypoint_prev.lat, waypoint_prev.lon,
                                           HDG_HOLD_REACHED_DIST + HDG_HOLD_SET_BACK_DIST, &temp_prev.lat, &temp_prev.lon);

        // next waypoint: shifted -(HDG_HOLD_DIST_NEXT + HDG_HOLD_REACHED_DIST)
        create_waypoint_from_line_and_dist(waypoint_next.lat, waypoint_next.lon, waypoint_prev.lat, waypoint_prev.lon,
                                           -(HDG_HOLD_REACHED_DIST + HDG_HOLD_DIST_NEXT), &temp_next.lat, &temp_next.lon);
    }

    waypoint_prev = temp_prev;
    waypoint_prev.alt = _hold_alt;
    waypoint_prev.valid = true;

    waypoint_next = temp_next;
    waypoint_next.alt = _hold_alt;
    waypoint_next.valid = true;
}

float
FixedwingPositionControl::get_terrain_altitude_takeoff(float takeoff_alt,
                                                       const vehicle_global_position_s &global_pos)
{
    if (PX4_ISFINITE(global_pos.terrain_alt) && global_pos.terrain_alt_valid) {
        return global_pos.terrain_alt;
    }

    return takeoff_alt;
}

bool
FixedwingPositionControl::update_desired_altitude(float dt)
{
    /*
     * The complete range is -1..+1, so this is 6%
     * of the up or down range or 3% of the total range.
     */
    const float deadBand = 0.06f;

    /*
     * The correct scaling of the complete range needs
     * to account for the missing part of the slope
     * due to the deadband
     */
    const float factor = 1.0f - deadBand;

    /* Climbout mode sets maximum throttle and pitch up */
    bool climbout_mode = false;

    /*
     * Reset the hold altitude to the current altitude if the uncertainty
     * changes significantly.
     * This is to guard against uncommanded altitude changes
     * when the altitude certainty increases or decreases.
     */

    if (fabsf(_althold_epv - _global_pos.epv) > ALTHOLD_EPV_RESET_THRESH) {
        _hold_alt = _global_pos.alt;
        _althold_epv = _global_pos.epv;
    }

    /*
     * Manual control has as convention the rotation around
     * an axis. Positive X means to rotate positively around
     * the X axis in NED frame, which is pitching down
     */
    if (_manual.x > deadBand) {
        /* pitching down */
        float pitch = -(_manual.x - deadBand) / factor;
        _hold_alt += (_parameters.max_sink_rate * dt) * pitch;
        _was_in_deadband = false;

    } else if (_manual.x < - deadBand) {
        /* pitching up */
        float pitch = -(_manual.x + deadBand) / factor;
        _hold_alt += (_parameters.max_climb_rate * dt) * pitch;
        _was_in_deadband = false;
        climbout_mode = (pitch > MANUAL_THROTTLE_CLIMBOUT_THRESH);

    } else if (!_was_in_deadband) {
        /* store altitude at which manual.x was inside deadBand
         * The aircraft should immediately try to fly at this altitude
         * as this is what the pilot expects when he moves the stick to the center */
        _hold_alt = _global_pos.alt;
        _althold_epv = _global_pos.epv;
        _was_in_deadband = true;
    }

    if (_vehicle_status.is_vtol) {
        if (_vehicle_status.is_rotary_wing || _vehicle_status.in_transition_mode) {
            _hold_alt = _global_pos.alt;
        }
    }

    return climbout_mode;
}

bool
FixedwingPositionControl::in_takeoff_situation()
{
    // in air for < 10s
    const hrt_abstime delta_takeoff = 10000000;

    return (hrt_elapsed_time(&_time_went_in_air) < delta_takeoff)
            && (_global_pos.alt <= _takeoff_ground_alt + _parameters.climbout_diff);
}

void
FixedwingPositionControl::do_takeoff_help(float *hold_altitude, float *pitch_limit_min)
{
    /* demand "climbout_diff" m above ground if user switched into this mode during takeoff */
    if (in_takeoff_situation()) {
        *hold_altitude = _takeoff_ground_alt + _parameters.climbout_diff;
        *pitch_limit_min = radians(10.0f);

    } else {
        *pitch_limit_min = _parameters.pitch_limit_min;
    }
}

bool
FixedwingPositionControl::control_position(const Vector2f &curr_pos, const Vector2f &ground_speed,
                                           const position_setpoint_s &pos_sp_prev, const position_setpoint_s &pos_sp_curr)
{
//    if(INFO_enable_1s) PX4_INFO("control_position 程序正在运行 %.1f秒",double(hrt_absolute_time()/1000/1000));
    float dt = 0.01f;

    if (_control_position_last_called > 0) {
        dt = hrt_elapsed_time(&_control_position_last_called) * 1e-6f;
    }

    _control_position_last_called = hrt_absolute_time();

    /* only run position controller in fixed-wing mode and during transitions for VTOL */
    if (_vehicle_status.is_rotary_wing && !_vehicle_status.in_transition_mode) {
        _control_mode_current = FW_POSCTRL_MODE_OTHER;
        return false;
    }

    bool setpoint = true;

    _att_sp.fw_control_yaw = false;		// by default we don't want yaw to be contoller directly with rudder
    _att_sp.apply_flaps = false;		// by default we don't use flaps

    calculate_gndspeed_undershoot(curr_pos, ground_speed, pos_sp_prev, pos_sp_curr);

    // l1 navigation logic breaks down when wind speed exceeds max airspeed
    // compute 2D groundspeed from airspeed-heading projection
    Vector2f air_speed_2d{_airspeed * cosf(_yaw), _airspeed * sinf(_yaw)};




    Vector2f nav_speed_2d{0.0f, 0.0f};

    // angle between air_speed_2d and ground_speed
    float air_gnd_angle = acosf((air_speed_2d * ground_speed) / (air_speed_2d.length() * ground_speed.length()));

    // if angle > 90 degrees or groundspeed is less than threshold, replace groundspeed with airspeed projection
    if ((fabsf(air_gnd_angle) > M_PI_2_F) || (ground_speed.length() < 3.0f)) {
        nav_speed_2d = air_speed_2d;

    } else {
        nav_speed_2d = ground_speed;
    }

    /* no throttle limit as default */
    float throttle_max = 1.0f;

    /* save time when airplane is in air */
    if (!_was_in_air && !_vehicle_land_detected.landed) {
        _was_in_air = true;
        _time_went_in_air = hrt_absolute_time();
        _takeoff_ground_alt = _global_pos.alt;
    }

    /* reset flag when airplane landed */
    if (_vehicle_land_detected.landed) {
        _was_in_air = false;
    }

    /* Reset integrators if switching to this mode from a other mode in which posctl was not active */
    if (_control_mode_current == FW_POSCTRL_MODE_OTHER) {
        /* reset integrators */
        _tecs.reset_state();
    }
    if (_control_mode.flag_control_auto_enabled && pos_sp_curr.valid) {
        /* AUTONOMOUS FLIGHT */

        _control_mode_current = FW_POSCTRL_MODE_AUTO;

        /* reset hold altitude */
        _hold_alt = _global_pos.alt;

        /* reset hold yaw */
        _hdg_hold_yaw = _yaw;

        /* get circle mode */
        bool was_circle_mode = _l1_control.circle_mode();

        /* restore speed weight, in case changed intermittently (e.g. in landing handling) */
        _tecs.set_speed_weight(_parameters.speed_weight);

        /* current waypoint (the one currently heading for) */
        Vector2f curr_wp((float)pos_sp_curr.lat, (float)pos_sp_curr.lon);

        /* Initialize attitude controller integrator reset flags to 0 */
        _att_sp.roll_reset_integral = false;
        _att_sp.pitch_reset_integral = false;
        _att_sp.yaw_reset_integral = false;

        /* previous waypoint */
        Vector2f prev_wp{0.0f, 0.0f};

        if (pos_sp_prev.valid) {
            prev_wp(0) = (float)pos_sp_prev.lat;
            prev_wp(1) = (float)pos_sp_prev.lon;

        } else {
            /*
             * No valid previous waypoint, go for the current wp.
             * This is automatically handled by the L1 library.
             */
            prev_wp(0) = (float)pos_sp_curr.lat;
            prev_wp(1) = (float)pos_sp_curr.lon;
        }

        float mission_airspeed = _parameters.airspeed_trim;

        if (PX4_ISFINITE(pos_sp_curr.cruising_speed) &&
                pos_sp_curr.cruising_speed > 0.1f) {

            mission_airspeed = pos_sp_curr.cruising_speed;
        }

        //待办:这里的throttle_cruise巡航油门需要所有飞机统一设置.
        float mission_throttle = _parameters.throttle_cruise;
        mission_throttle = 0.5;//这里暂定设置所有飞机的默认巡航油门为0.5;

        if (PX4_ISFINITE(pos_sp_curr.cruising_throttle) &&
                pos_sp_curr.cruising_throttle > 0.01f) {

            mission_throttle = pos_sp_curr.cruising_throttle;
        }

//        PX4_INFO("pos_sp_curr.type = %.1f",1.0 * pos_sp_curr.type);

        if (pos_sp_curr.type == position_setpoint_s::SETPOINT_TYPE_IDLE) {

            if(INFO_enable_1s) PX4_INFO("SETPOINT_TYPE_IDLE !");

            _att_sp.thrust = 0.0f;
            _att_sp.roll_body = 0.0f;
            _att_sp.pitch_body = 0.0f;

        } else if (pos_sp_curr.type == position_setpoint_s::SETPOINT_TYPE_POSITION) {
            if(INFO_enable_1s) PX4_INFO("SETPOINT_TYPE_POSITION !");
            /* waypoint is a plain navigation waypoint */
            _l1_control.navigate_waypoints(prev_wp, curr_wp, curr_pos, nav_speed_2d);
            _att_sp.roll_body = _l1_control.nav_roll();
            _att_sp.yaw_body = _l1_control.nav_bearing();

            tecs_update_pitch_throttle(pos_sp_curr.alt,
                                       calculate_target_airspeed(mission_airspeed),
                                       radians(_parameters.pitch_limit_min) - _parameters.pitchsp_offset_rad,
                                       radians(_parameters.pitch_limit_max) - _parameters.pitchsp_offset_rad,
                                       _parameters.throttle_min,
                                       _parameters.throttle_max,
                                       mission_throttle,
                                       false,
                                       radians(_parameters.pitch_limit_min));


        } else if (pos_sp_curr.type == position_setpoint_s::SETPOINT_TYPE_LOITER) {

            /* waypoint is a loiter waypoint */
            _l1_control.navigate_loiter(curr_wp, curr_pos, pos_sp_curr.loiter_radius,
                                        pos_sp_curr.loiter_direction, nav_speed_2d);
            _att_sp.roll_body = _l1_control.nav_roll();
            _att_sp.yaw_body = _l1_control.nav_bearing();

            float alt_sp = pos_sp_curr.alt;

            if (in_takeoff_situation()) {
                alt_sp = max(alt_sp, _takeoff_ground_alt + _parameters.climbout_diff);
                _att_sp.roll_body = constrain(_att_sp.roll_body, radians(-5.0f), radians(5.0f));
            }

            if (_fw_pos_ctrl_status.abort_landing) {
                if (pos_sp_curr.alt - _global_pos.alt  < _parameters.climbout_diff) {
                    // aborted landing complete, normal loiter over landing point
                    _fw_pos_ctrl_status.abort_landing = false;

                } else {
                    // continue straight until vehicle has sufficient altitude
                    _att_sp.roll_body = 0.0f;
                }
            }

            tecs_update_pitch_throttle(alt_sp,
                                       calculate_target_airspeed(mission_airspeed),
                                       radians(_parameters.pitch_limit_min) - _parameters.pitchsp_offset_rad,
                                       radians(_parameters.pitch_limit_max) - _parameters.pitchsp_offset_rad,
                                       _parameters.throttle_min,
                                       _parameters.throttle_max,
                                       _parameters.throttle_cruise,
                                       false,
                                       radians(_parameters.pitch_limit_min));

        } else if (pos_sp_curr.type == position_setpoint_s::SETPOINT_TYPE_FOLLOW_TARGET) {  //这里是自定义的语句,增加的follow_target模式

            control_follow_target(nav_speed_2d,
                                  air_speed_2d,mission_throttle,
                                  pos_sp_prev,pos_sp_curr);

        } else if (pos_sp_curr.type == position_setpoint_s::SETPOINT_TYPE_LAND) {
            control_landing(curr_pos, ground_speed, pos_sp_prev, pos_sp_curr);

        } else if (pos_sp_curr.type == position_setpoint_s::SETPOINT_TYPE_TAKEOFF) {
            control_takeoff(curr_pos, ground_speed, pos_sp_prev, pos_sp_curr);
        }

        /* reset landing state */
        if (pos_sp_curr.type != position_setpoint_s::SETPOINT_TYPE_LAND) {
            reset_landing_state();
        }

        /* reset takeoff/launch state */
        if (pos_sp_curr.type != position_setpoint_s::SETPOINT_TYPE_TAKEOFF) {
            reset_takeoff_state();
        }

        if (was_circle_mode && !_l1_control.circle_mode()) {
            /* just kicked out of loiter, reset roll integrals */
            _att_sp.roll_reset_integral = true;
        }

    } else if (_control_mode.flag_control_velocity_enabled &&
               _control_mode.flag_control_altitude_enabled) {
        /* POSITION CONTROL: pitch stick moves altitude setpoint, throttle stick sets airspeed,
           heading is set to a distant waypoint */

        if (_control_mode_current != FW_POSCTRL_MODE_POSITION) {
            /* Need to init because last loop iteration was in a different mode */
            _hold_alt = _global_pos.alt;
            _hdg_hold_yaw = _yaw;
            _hdg_hold_enabled = false; // this makes sure the waypoints are reset below
            _yaw_lock_engaged = false;

            /* reset setpoints from other modes (auto) otherwise we won't
             * level out without new manual input */
            _att_sp.roll_body = _manual.y * _parameters.man_roll_max_rad;
            _att_sp.yaw_body = 0;
        }

        _control_mode_current = FW_POSCTRL_MODE_POSITION;

        float altctrl_airspeed = get_demanded_airspeed();

        /* update desired altitude based on user pitch stick input */
        bool climbout_requested = update_desired_altitude(dt);

        /* if we assume that user is taking off then help by demanding altitude setpoint well above ground
        * and set limit to pitch angle to prevent stearing into ground
        */
        float pitch_limit_min{0.0f};
        do_takeoff_help(&_hold_alt, &pitch_limit_min);

        /* throttle limiting */
        throttle_max = _parameters.throttle_max;

        if (_vehicle_land_detected.landed && (fabsf(_manual.z) < THROTTLE_THRESH)) {
            throttle_max = 0.0f;
        }

        tecs_update_pitch_throttle(_hold_alt,
                                   altctrl_airspeed,
                                   radians(_parameters.pitch_limit_min),
                                   radians(_parameters.pitch_limit_max),
                                   _parameters.throttle_min,
                                   throttle_max,
                                   _parameters.throttle_cruise,
                                   climbout_requested,
                                   climbout_requested ? radians(10.0f) : pitch_limit_min,
                                   tecs_status_s::TECS_MODE_NORMAL);

        /* heading control */
        if (fabsf(_manual.y) < HDG_HOLD_MAN_INPUT_THRESH &&
                fabsf(_manual.r) < HDG_HOLD_MAN_INPUT_THRESH) {

            /* heading / roll is zero, lock onto current heading */
            if (fabsf(_att.yawspeed) < HDG_HOLD_YAWRATE_THRESH && !_yaw_lock_engaged) {
                // little yaw movement, lock to current heading
                _yaw_lock_engaged = true;

            }

            /* user tries to do a takeoff in heading hold mode, reset the yaw setpoint on every iteration
              to make sure the plane does not start rolling
            */
            if (in_takeoff_situation()) {
                _hdg_hold_enabled = false;
                _yaw_lock_engaged = true;
            }

            if (_yaw_lock_engaged) {

                /* just switched back from non heading-hold to heading hold */
                if (!_hdg_hold_enabled) {
                    _hdg_hold_enabled = true;
                    _hdg_hold_yaw = _yaw;

                    get_waypoint_heading_distance(_hdg_hold_yaw, _hdg_hold_prev_wp, _hdg_hold_curr_wp, true);
                }

                /* we have a valid heading hold position, are we too close? */
                float dist = get_distance_to_next_waypoint(_global_pos.lat, _global_pos.lon, _hdg_hold_curr_wp.lat,
                                                           _hdg_hold_curr_wp.lon);

                if (dist < HDG_HOLD_REACHED_DIST) {
                    get_waypoint_heading_distance(_hdg_hold_yaw, _hdg_hold_prev_wp, _hdg_hold_curr_wp, false);
                }

                Vector2f prev_wp{(float)_hdg_hold_prev_wp.lat, (float)_hdg_hold_prev_wp.lon};
                Vector2f curr_wp{(float)_hdg_hold_curr_wp.lat, (float)_hdg_hold_curr_wp.lon};

                /* populate l1 control setpoint */
                _l1_control.navigate_waypoints(prev_wp, curr_wp, curr_pos, ground_speed);

                _att_sp.roll_body = _l1_control.nav_roll();
                _att_sp.yaw_body = _l1_control.nav_bearing();

                if (in_takeoff_situation()) {
                    /* limit roll motion to ensure enough lift */
                    _att_sp.roll_body = constrain(_att_sp.roll_body, radians(-15.0f), radians(15.0f));
                }
            }
        }

        if (!_yaw_lock_engaged || fabsf(_manual.y) >= HDG_HOLD_MAN_INPUT_THRESH ||
                fabsf(_manual.r) >= HDG_HOLD_MAN_INPUT_THRESH) {

            _hdg_hold_enabled = false;
            _yaw_lock_engaged = false;
            _att_sp.roll_body = _manual.y * _parameters.man_roll_max_rad;
            _att_sp.yaw_body = 0;
        }

    } else if (_control_mode.flag_control_altitude_enabled) {
        /* ALTITUDE CONTROL: pitch stick moves altitude setpoint, throttle stick sets airspeed */

        if (_control_mode_current != FW_POSCTRL_MODE_POSITION && _control_mode_current != FW_POSCTRL_MODE_ALTITUDE) {
            /* Need to init because last loop iteration was in a different mode */
            _hold_alt = _global_pos.alt;
        }

        _control_mode_current = FW_POSCTRL_MODE_ALTITUDE;

        /* Get demanded airspeed */
        float altctrl_airspeed = get_demanded_airspeed();

        /* update desired altitude based on user pitch stick input */
        bool climbout_requested = update_desired_altitude(dt);

        /* if we assume that user is taking off then help by demanding altitude setpoint well above ground
        * and set limit to pitch angle to prevent stearing into ground
        */
        float pitch_limit_min{0.0f};
        do_takeoff_help(&_hold_alt, &pitch_limit_min);

        /* throttle limiting */
        throttle_max = _parameters.throttle_max;

        if (_vehicle_land_detected.landed && (fabsf(_manual.z) < THROTTLE_THRESH)) {
            throttle_max = 0.0f;
        }

        tecs_update_pitch_throttle(_hold_alt,
                                   altctrl_airspeed,
                                   radians(_parameters.pitch_limit_min),
                                   radians(_parameters.pitch_limit_max),
                                   _parameters.throttle_min,
                                   throttle_max,
                                   _parameters.throttle_cruise,
                                   climbout_requested,
                                   climbout_requested ? radians(10.0f) : pitch_limit_min,
                                   tecs_status_s::TECS_MODE_NORMAL);

        _att_sp.roll_body = _manual.y * _parameters.man_roll_max_rad;
        _att_sp.yaw_body = 0;

    } else {
        _control_mode_current = FW_POSCTRL_MODE_OTHER;

        /* do not publish the setpoint */
        setpoint = false;

        // reset hold altitude
        _hold_alt = _global_pos.alt;

        /* reset landing and takeoff state */
        if (!_last_manual) {
            reset_landing_state();
            reset_takeoff_state();
        }
    }

    /* Copy thrust output for publication */
    if (_control_mode_current == FW_POSCTRL_MODE_AUTO && // launchdetector only available in auto
            pos_sp_curr.type == position_setpoint_s::SETPOINT_TYPE_TAKEOFF &&
            _launch_detection_state != LAUNCHDETECTION_RES_DETECTED_ENABLEMOTORS &&
            !_runway_takeoff.runwayTakeoffEnabled()) {

        /* making sure again that the correct thrust is used,
         * without depending on library calls for safety reasons.
           the pre-takeoff throttle and the idle throttle normally map to the same parameter. */
        _att_sp.thrust = _parameters.throttle_idle;

    } else if (_control_mode_current == FW_POSCTRL_MODE_AUTO &&
               pos_sp_curr.type == position_setpoint_s::SETPOINT_TYPE_TAKEOFF &&
               _runway_takeoff.runwayTakeoffEnabled()) {

        _att_sp.thrust = _runway_takeoff.getThrottle(min(get_tecs_thrust(), throttle_max));

    } else if (_control_mode_current == FW_POSCTRL_MODE_AUTO &&
               pos_sp_curr.type == position_setpoint_s::SETPOINT_TYPE_IDLE) {

        _att_sp.thrust = 0.0f;

    } else if (_control_mode_current == FW_POSCTRL_MODE_OTHER) {
        _att_sp.thrust = min(_att_sp.thrust, _parameters.throttle_max);

    } else {
        /* Copy thrust and pitch values from tecs */
        if (_vehicle_land_detected.landed) {
            // when we are landed state we want the motor to spin at idle speed
            _att_sp.thrust = min(_parameters.throttle_idle, throttle_max);

        } else {
            _att_sp.thrust = min(get_tecs_thrust(), throttle_max);
        }
    }

    // decide when to use pitch setpoint from TECS because in some cases pitch
    // setpoint is generated by other means
    bool use_tecs_pitch = true;

    // auto runway takeoff
    use_tecs_pitch &= !(_control_mode_current == FW_POSCTRL_MODE_AUTO &&
                        pos_sp_curr.type == position_setpoint_s::SETPOINT_TYPE_TAKEOFF &&
                        (_launch_detection_state == LAUNCHDETECTION_RES_NONE || _runway_takeoff.runwayTakeoffEnabled()));

    // flaring during landing
    use_tecs_pitch &= !(pos_sp_curr.type == position_setpoint_s::SETPOINT_TYPE_LAND && _land_noreturn_vertical);

    // manual attitude control
    use_tecs_pitch &= !(_control_mode_current == FW_POSCTRL_MODE_OTHER);

    if (use_tecs_pitch) {
        _att_sp.pitch_body = get_tecs_pitch();
    }

    if (_control_mode.flag_control_position_enabled) {
        _last_manual = false;

    } else {
        _last_manual = true;
    }

    return setpoint;
}



matrix::Vector2f FixedwingPositionControl::bodytoNED(matrix::Vector2f L_body,matrix::Vector2f speed_ned, float yaw)
{
    float cos_MP_yaw = speed_ned(0) / speed_ned.length();
    float sin_MP_yaw = speed_ned(1) / speed_ned.length();
    if(speed_ned.length()<3.0f){   //当地速很小时,地速方向不稳定,此时使用飞机机头指向
        //待办:这部分需要在飞行时确认飞机的地速方向与机头指向偏差不大,务必注意根据试验情况确定
        cos_MP_yaw = cos(double(yaw));
        sin_MP_yaw = sin(double(yaw));
    }

    // 坐标转换,从飞机体轴系转换到地轴系ned
    matrix::Vector2f L_ned;
    L_ned(0) = L_body(0) * cos_MP_yaw - L_body(1) * sin_MP_yaw;
    L_ned(1) = L_body(1) * cos_MP_yaw + L_body(0) * sin_MP_yaw;
    return L_ned;

}


void
FixedwingPositionControl::INFO_enable_1s_TS(){


}

void
FixedwingPositionControl::INFO_enable1s_TS(){

    //控制输出频率
    static uint64_t prevsend2time = 0;
    float dt_send2time = hrt_elapsed_time(&prevsend2time) * 1e-6f;
    prevsend2time = hrt_absolute_time();
    float send2HZ = 1.0f / dt_send2time;
    static uint64_t previnfo2time{0};
    if((hrt_elapsed_time(&previnfo2time) * 1e-6f) > 1.0f){
        previnfo2time = prevsend2time;
        PX4_INFO("\n\n control_follow_target周期:%6.3fs 频率:%3.1fHz",double(dt_send2time),double(send2HZ));
        INFO_enable1s = true;
    }


}



//待办,遥控器切换验证程序

void
FixedwingPositionControl::control_follow_target(const Vector2f &nav_speed_2d,
                                                const Vector2f &air_speed_2d,const float mission_throttle,
                                                const position_setpoint_s &pos_sp_prev, const position_setpoint_s &pos_sp_curr)
{

    //控制输出频率
    static uint64_t prevsend2time = 0;
    float dt_send2time = hrt_elapsed_time(&prevsend2time) * 1e-6f;
    prevsend2time = hrt_absolute_time();
    float send2HZ = 1.0f / dt_send2time;
    static uint64_t previnfo2time{0};
    if((hrt_elapsed_time(&previnfo2time) * 1e-6f) > 1.0f){
        previnfo2time = prevsend2time;
        PX4_INFO("\n\n control_follow_target周期:%6.3fs 频率:%3.1fHz",double(dt_send2time),double(send2HZ));
//        mavlink_log_info(&_mavlink_log_pub,"#跟随位置控制");
        INFO_enable_1s = true;
    }



//INFO_enable_1s_TS();
//INFO_enable1s_TS();


//    if(INFO_enable_1s) PX4_INFO(">>>>>>>>>>>>>>>> 运行 位置控制程序中的FOLLOW_TARGET  <<<<<<<<<<<<<<<<");


    //获得主机位置信息
    bool follow_target_updated = false;
    if (_follow_target_sub < 0) {
        _follow_target_sub = orb_subscribe(ORB_ID(follow_target));
    }
    static follow_target_s MP_position{};
    orb_check(_follow_target_sub, &follow_target_updated);
    if (follow_target_updated) {  //如果获得了目标更新
        orb_copy(ORB_ID(follow_target), _follow_target_sub, &MP_position);
    }


    //对主机位置进行滤波
    static float _responsiveness = 0.01f;
    static follow_target_s MP_position_filter{};
    if(MP_position_filter.timestamp == 0){
        MP_position_filter = MP_position;
    }
    MP_position_filter.timestamp = MP_position.timestamp;
    MP_position_filter.lat = MP_position_filter.lat * (double)_responsiveness + MP_position.lat * (double)(1 - _responsiveness);
    MP_position_filter.lon = MP_position_filter.lon * (double)_responsiveness + MP_position.lon * (double)(1 - _responsiveness);
    MP_position_filter.alt = MP_position_filter.alt *         _responsiveness + MP_position.alt *         (1 - _responsiveness); //附加一个对高度的滤波
    MP_position_filter.yaw = MP_position_filter.yaw *         _responsiveness + MP_position.yaw *         (1 - _responsiveness); //附加一个对高度的滤波

    //这一段用来求平均速度
    static follow_target_s MP_position_prev{};
    float dt_ms = (MP_position_filter.timestamp - MP_position_prev.timestamp) *1e-3f;
    if (dt_ms >= 100.0f) {
        // get last gps known reference for target
        static matrix::Vector3f MP_position_delta3D{};
        get_vector_to_next_waypoint( MP_position_prev.lat,MP_position_prev.lon,MP_position_filter.lat,MP_position_filter.lon,
                                     &MP_position_delta3D(0),&MP_position_delta3D(1));
        MP_position_delta3D(2) = MP_position_filter.alt - MP_position_prev.alt;
        // update the average velocity of the target based on the position
        matrix::Vector3f MP_velocity_average3D = MP_position_delta3D / (dt_ms * 1e-3f);
        MP_position_filter.vx = MP_velocity_average3D(0);
        MP_position_filter.vy = MP_velocity_average3D(1);
        MP_position_filter.vz = MP_velocity_average3D(2);
        MP_position_prev = MP_position_filter;  //每一段计算时间之后,重新复位MP_position_prev
    }

    Vector2f MP_speed = {MP_position_filter.vx,MP_position_filter.vy};    //收到的主机的速度.单位 m/s
    Vector2f MP_gndspd_ned = {MP_position_filter.vx,MP_position_filter.vy};    //收到的主机的速度.单位 m/s


    static uint8_t _form_shape_current =  MP_position.FORMSHAPE_RHOMBUS4;
    _form_shape_current =  MP_position.formshape_id;

    /*这部分进行编队队形计算,计算出从机在编队中的相对位置*/
    //输入:编队队形编号
    //输出:从机在某一时刻相对于主机的距离向量 L_MPtoSP{L_along,L_cross}

//   static bool info1 = true;
    static uint8_t _form_shape_last = 0;

    static int FORMATION_rhombus4_axis[4][2] = {      //4机菱形编队的坐标集合,{主机地轴航向前后位置,左右位置}向前为正,向右为正
                                                      { 0, 0}, // 1号机位置(主机) 坐标原点
                                                      {-1, 1}, // 2号机位置,主机右边,后面
                                                      {-1,-1}, // 3号机位置,主机左边,后面
                                                      {-2, 0}  // 4号机位置,主机后面
                                               };

    float L_space(20.0f);  //编队飞机之间的间距

    matrix::Vector2f L_MPtoSP = {L_space,L_space}; //从机相对主机的偏移距离向量,主机地轴航向作为x轴正方向,主机右侧是y正方向
    uint8_t sys_id = _vehicle_status.system_id;
    switch (_form_shape_current) {
    case MP_position.FORMSHAPE_HORIZON1 :
        if(_form_shape_last != _form_shape_current){
            mavlink_log_info(&_mavlink_log_pub,"#转换横向1字编队");
        }

        L_MPtoSP = {0.0f, L_space * (sys_id-1)};


        break;

    case MP_position.FORMSHAPE_HORIZON1_and_VERTIAL1 :
        if(_form_shape_last != _form_shape_current){
            mavlink_log_info(&_mavlink_log_pub,"#转换倾斜1字编队");
        }
        L_MPtoSP = {-1.0f * L_space * (sys_id-1), L_space * (sys_id-1)};

        break;
    case MP_position.FORMSHAPE_VERTIAL1  :
        if(_form_shape_last != _form_shape_current){
            mavlink_log_info(&_mavlink_log_pub,"#转换纵向1字编队");
        }

        L_MPtoSP = {-1.0f * L_space * (sys_id-1) ,0.0f};

        break;
    case MP_position.FORMSHAPE_VERTIAL1_and_RHOMBUS4  :
        if(_form_shape_last != _form_shape_current){
            mavlink_log_info(&_mavlink_log_pub,"#菱形 纵向 过渡编队");
        }

        L_MPtoSP(0) = -1.0f * L_space * (sys_id-1);                     //纵向坐标
        L_MPtoSP(1) = FORMATION_rhombus4_axis[sys_id][1] * L_space;  //横向坐标


        break;
    case MP_position.FORMSHAPE_RHOMBUS4  :
        if(_form_shape_last != _form_shape_current){
            mavlink_log_info(&_mavlink_log_pub,"#转换菱形4机编队");
        }
        L_MPtoSP(0) = FORMATION_rhombus4_axis[sys_id][0] * L_space;//纵向坐标
        L_MPtoSP(1) = FORMATION_rhombus4_axis[sys_id][1] * L_space;//横向坐标

        break;
    }
    _form_shape_last = _form_shape_current; //记录上面状态机的值

    //把两架飞机之间的水平距离转换为地理坐标系下的差距，下一步好根据这个差距计算从机的位置指令
    matrix::Vector2f L_MPtoSP_ned = bodytoNED(L_MPtoSP,MP_gndspd_ned,MP_position_filter.yaw);

    float L_spacePB{30.0f};//2.0f * _navigator->get_acceptance_radius());  // B点到从机目标位置的距离,默认大于L1距离
    float L_spacePA{0.0f};  //A点到从机目标位置的距离,这个距离不宜太大
    matrix::Vector2f offset_PB_ned = bodytoNED({L_spacePB         , 0.0f},MP_gndspd_ned,MP_position_filter.yaw);  //PB方向为正
    matrix::Vector2f offset_PA_ned = bodytoNED({-1.0f * L_spacePA , 0.0f},MP_gndspd_ned,MP_position_filter.yaw);  //PA方向为负

    //获得从机GPS位置信息
    bool vehicle_gps_position_updated;
    static vehicle_gps_position_s SP_gps_pos;
    orb_check(_vehicle_gps_position_sub, &vehicle_gps_position_updated);
    if (vehicle_gps_position_updated) {
        orb_copy(ORB_ID(vehicle_gps_position), _vehicle_gps_position_sub, &SP_gps_pos);
    }
    //获得从机global位置信息
    bool vehicle_global_position_updated;
    static int _vehicle_global_position_sub = orb_subscribe(ORB_ID(vehicle_global_position));
    static vehicle_global_position_s SP_global_pos{};
    orb_check(_vehicle_global_position_sub, &vehicle_global_position_updated);
    if (vehicle_global_position_updated) {
        orb_copy(ORB_ID(vehicle_global_position), _vehicle_global_position_sub, &SP_global_pos);
    }
    Vector2f curr_pos = {float(SP_global_pos.lat),float(SP_global_pos.lon)};  //初始化从机位置向量


    hrt_abstime now_utc_time0 = SP_gps_pos.time_utc_usec + hrt_elapsed_time(&SP_gps_pos.timestamp);
    float dt_utc_s0 = (now_utc_time0-MP_position.timestamp) * 1e-6f;  //单位 秒 计算从机本地时间到主机时间戳的时间差
    float dt_utc_s = dt_utc_s0;

    static int times_mavlink = 9;
    if(dt_utc_s>0.3f){//说明此时主机数据传输有较大延时,此数据不能作为预测使用
        if(times_mavlink % 10 == 0){
           if(INFO_enable_1s) mavlink_and_console_log_info(&_mavlink_log_pub, "#%d超时%.0f",_vehicle_status.system_id,double(dt_utc_s * 1000.0f));
//           if(INFO_enable1s) PX4_INFO("超时dt_utc_s: %.2f pos_sp_curr.timestamp=%.0f",double(dt_utc_s),double(pos_sp_curr.timestamp))  ;
        }
        times_mavlink ++;
        dt_utc_s = 0.1f;//避免延时问题,更改为平均值
    } else {
        times_mavlink = 9;
    }
    //这一段计算在时间差内的位移,并且应用到主机位置上

    Vector2f MP_deltaL = dt_utc_s * MP_speed;  //主机在传输时间差内的位移 单位m


    hrt_abstime now_utc_time1 = SP_gps_pos.time_utc_usec + hrt_elapsed_time(&SP_gps_pos.timestamp);
    //根据从机相对主机的距离,计算出从机的目标位置
    map_projection_reference_s target_ref;
    static follow_target_s SP_position_sp;
    static follow_target_s PB_position_sp;
    static follow_target_s PA_position_sp;

    //下面这段代码主要是NED坐标系和全球坐标系之间的转换，map_projection_init函数的意思就是当前这个经度纬度视为（0,0）原点。
    //map_projection_reproject函数在原点基础上 偏移一个（x,y）后的经度纬度是多少，这样就可以根据两个点之间地理坐标系的差距 计算出另外一个点的全球坐标系
    //当然与之对应的 还有一个是已知B点的全球坐标系 可以映射求出B点的地理坐标系
    //
    //编队目的，下面已知主机和从机的地理坐标系上编队的差距，知道主机的位置 如何求从机的位置经度纬度？
    //那就把主机位置映射成原点，加上地理坐标系上的偏移后，求出从机的经度纬度。函数重要重要是地理坐标系和全球坐标系之间的转换，主要注意的是初始化谁是原点（0,0）

    //初始化主机位置
    map_projection_init(&target_ref,  MP_position_filter.lat, MP_position_filter.lon);
    //计算主机时移位置
    static follow_target_s MP_position_filter_dL{};
    map_projection_reproject(&target_ref, MP_deltaL(0), MP_deltaL(1),&MP_position_filter_dL.lat, &MP_position_filter_dL.lon);
    //初始化主机时移位置
    map_projection_init(&target_ref,  MP_position_filter_dL.lat, MP_position_filter_dL.lon);
    //计算从机时移位置
    map_projection_reproject(&target_ref, L_MPtoSP_ned(0), L_MPtoSP_ned(1),
                             &SP_position_sp.lat, &SP_position_sp.lon);
    //计算A点时移位置
    map_projection_reproject(&target_ref, L_MPtoSP_ned(0)+offset_PB_ned(0), L_MPtoSP_ned(1)+offset_PB_ned(1),
                             &PB_position_sp.lat, &PB_position_sp.lon); //计算A点坐标,A点位置为主机地速方向前方一定距离
    //计算B点时移位置
    map_projection_reproject(&target_ref, L_MPtoSP_ned(0)+offset_PA_ned(0), L_MPtoSP_ned(1)+offset_PA_ned(1),
                             &PA_position_sp.lat, &PA_position_sp.lon); //计算B点坐标,B点位置为从机地速方向后方一定距离













    hrt_abstime now_utc_time2 = SP_gps_pos.time_utc_usec + hrt_elapsed_time(&SP_gps_pos.timestamp);




    /******************************************* 这部分进行纵向控制 **************************************************************************/

    //计算位置差
    static Vector2f PtoPsp_distance{}; //从机实际位置P到从机目标位置Psp之间的距离向量
    get_vector_to_next_waypoint(SP_global_pos.lat,SP_global_pos.lon,SP_position_sp.lat,SP_position_sp.lon,
                                &PtoPsp_distance(0),&PtoPsp_distance(1));
    //计算速度差

    Vector2f SP_gndspd_ned = {SP_global_pos.vel_n,SP_global_pos.vel_e};
    Vector2f MPminusSP_speed = MP_gndspd_ned - SP_gndspd_ned;  //主机地速减去从机地速

    //
    float bear_P2Psp_P1v = math::degrees(atan2f(PtoPsp_distance % MP_gndspd_ned,PtoPsp_distance * MP_gndspd_ned));



    Vector2f yaw_norm = {float(cos(double(MP_position_filter.yaw))),float(sin(double(MP_position_filter.yaw)))};

    Vector2f MP_gndspd_ned_norm = MP_gndspd_ned.length()>2.0f ? MP_gndspd_ned.normalized() : yaw_norm;    //调试,这个阈值是地速大小,注意切换时数据有无跳变

    //待办:在飞行或高速运动的时候务必确认以下以上两个向量的角度,先飞主机看看
    //待办:在地面上调好飞机的距离响应和反馈,务必确认好速度的反馈

    //待办:这部分速度投影的处理还需要再优化,后期可以根据从机到编队的距离来判断,当距离非常大时不使用投影,距离很小时使用投影.或者根据从机到主机的方位来判断.
    //计算在主机速度上的投影
    float dL_PtoPsp_project(math::constrain(PtoPsp_distance * MP_gndspd_ned_norm,-50.0f, 50.0f)); //注意将距离差向量投影到主机速度向量上 ,加限幅是为了防止SP_gndspd_ned溢出
    float dV_MPtoSP_project(math::constrain(MPminusSP_speed * MP_gndspd_ned_norm,-30.0f, 30.0f)); //将速度差向量投影到主机速度向量上 ,加限幅是为了防止SP_gndspd_ned溢出


    //这里要注意差值向量的正负

    float K_P(1.5f); //距离差量的增益值  待办,这个参数要做成地面站可调的,注意,参数为2时3号机也能飞,不建议再继续增大了,下次调试改成1.5试试
    float K_D(1.2f); //速度差量的增益值  待办,这个参数要做成地面站可调的,注意,这个值先保持1.2,目前问题是当通信频率不高时这个值是否有效
    Vector2f SP_gndspd_ned_sp = MP_gndspd_ned + MP_gndspd_ned.normalized() * (K_P * dL_PtoPsp_project + K_D * dV_MPtoSP_project); //从机目标地速向量于主机地速向量平行

    //根据地速与空速数据,计算环境风速.当空速有效时起效
    static Vector2f wind_speed_ned{};
    if(_airspeed_valid){
        wind_speed_ned = SP_gndspd_ned - air_speed_2d;
    } else {
        wind_speed_ned = {0.0,0.0};  //当空速数据无效时,风速归零
    }
    //计算目标空速
    Vector2f SP_airspd_ned_sp = SP_gndspd_ned_sp-wind_speed_ned; //目标风速矢量 = 目标地速 - 风速
    float airspeed_follow_sp = math::max(SP_airspd_ned_sp.length(), _parameters.airspeed_min);

    //待办:主机设置的最小空速要大于从机的最小空速4m/s.暂时在地面站里面设置,之后要写在代码里面


    float throttle_follow_refer = mission_throttle;



    //注意:在这里设置TECS的油门参考值,通过参考值的设定更迅速调整飞机的速度
    float Control_thr_L = 3.0f;//油门比例控制范围
    float THR_outofrange = 0.95f;//超出比例控制范围后的油门
    if(PtoPsp_distance.length() < 12.0f){
        if(dL_PtoPsp_project < 0.0f){ //当飞机超前的时候务必减速
            throttle_follow_refer = 0.01f;
            airspeed_follow_sp = 0.01f;
        } else if(dL_PtoPsp_project < Control_thr_L){
            throttle_follow_refer = constrain(dL_PtoPsp_project * THR_outofrange / Control_thr_L, 0.01f, THR_outofrange);
        }
    }

    //从机超前时稍微提高目标高度
    float chaosu_L = 0.0f;
    if((PtoPsp_distance.length() < 30.0f && dL_PtoPsp_project < -7.0f)){ //这里给了一个很宽的作用范围,防止飞机对头飞行时相撞
        chaosu_L = 5.0f;
    }





    float dL_PtoPsp_across = PtoPsp_distance % MP_gndspd_ned_norm;



    /******************************************* 这部分进行横向控制和修正 **************************************************************************/
    //开始横航向计算
//提取A点和B点坐标
Vector2f currB_sp = {float(PB_position_sp.lat), float(PB_position_sp.lon)};
Vector2f prevA_sp = {float(PA_position_sp.lat), float(PA_position_sp.lon)};
    //放在这里主要是为了使用上面计算出来的侧偏距
    _l1_control.navigate_followme(prevA_sp, currB_sp, curr_pos, nav_speed_2d);//使用目标航点的位置作为跟踪位置
    _att_sp.roll_body = _l1_control.nav_roll();
    _att_sp.yaw_body = _l1_control.nav_bearing();

    //如果飞机的侧偏距在一定范围内(需要同时满足以下条件),就启用2倍纠偏权限
    if(dL_PtoPsp_project < 10.0f && dL_PtoPsp_project > -4.0f && fabs(double(dL_PtoPsp_across)) < 10.0){ //条件1

        //以下两种模式,等测试
        static int8_t last_check_aux2_SW_enable = 0;
        int8_t now_check_aux2_SW_enable = check_aux2_SW_enable();

        //待办,给飞机增加高度处理程序,现在飞机在天上高度严重不一致.
        //待办,给飞机增加获得定位时报高度的程序,方便外场操作
        //待办,当某个飞机超出控制精度,需要降高度避险时,需要地面站发出声音

        if(true){
            if(last_check_aux2_SW_enable != now_check_aux2_SW_enable){
                mavlink_log_info(&_mavlink_log_pub,"#第一状态") //注意:这个控制模式追踪的最好
            }

            const float rectify_L_range = 1.0f;  //超过这个距离值,就会启用强制纠偏算法
            if(float(fabs(double(dL_PtoPsp_across))) > rectify_L_range){ //条件2 //注意:这个值是1的时候是上次正常状态
                _att_sp.roll_body = float(fabs(double(dL_PtoPsp_across * 1.0f/rectify_L_range))) * _att_sp.roll_body; //注意:这个值是5的时候是上次正常状态
                //上次调试,飞机侧偏控制容易超调,想办法减缓,试试对侧偏距开根号
            }
        } else {
            if(last_check_aux2_SW_enable != now_check_aux2_SW_enable){
                mavlink_log_info(&_mavlink_log_pub,"#第二状态")
            }
            if(0){  //待办,注意这里的测试
                _att_sp.roll_body = _att_sp.roll_body * float(fabs(double(_att_sp.roll_body)));
            }else{
                _att_sp.roll_body = math::radians(0.5f * math::degrees(_att_sp.roll_body) * float(fabs(double(math::degrees(_att_sp.roll_body)))));

//                float rectify_ROLL_range = 5.0f;  //超过范围值,就会启用强制纠偏算法
//                if(_att_sp.roll_body>0.0f){
//                    _att_sp.roll_body = float(fmax(double(_att_sp.roll_body),double(1.0f/rectify_ROLL_range * _att_sp.roll_body * float(fabs(double(_att_sp.roll_body))))));
//                }else{
//                    _att_sp.roll_body = float(fmin(double(_att_sp.roll_body),double(1.0f/rectify_ROLL_range * _att_sp.roll_body * float(fabs(double(_att_sp.roll_body))))));
//                }
            }
        }
        last_check_aux2_SW_enable = now_check_aux2_SW_enable;

        _att_sp.roll_body = constrain(_att_sp.roll_body, radians(-50.0f), radians(50.0f));  //限制范围


    }


    //此功能是飞机在目标范围内时,加入编队高度层,注意,编队无高度差
    //这一段是使用水平距离判断是否需要进行降高度保护
    float juli_L = 4.0f * float(sys_id-1);  //根据各机编号确定安全间隔
    if((dL_PtoPsp_project < 8.0f && dL_PtoPsp_project > -4.0f) && (fabs(double(dL_PtoPsp_across)) < 8.0)){
        juli_L = 1.0f * float(sys_id-1);//加入编队,也有一定的安全间隔
    }

    //    待办:这里可以加一个对高度的处理,当飞行器很接近目标位置时提高高度进入编队
    //  home_alt_valid() ? _home_pos.alt + 40.0f : MP_position_filter.alt -10.0f,  //调试,注意这里是用了home高度 待办:注意所有飞机起飞前应在同一高度解锁
    float follow_alt_sp = max(MP_position_filter.alt - juli_L + chaosu_L, pos_sp_curr.home_alt + 70.0f);//_home_pos.alt + 100.0f;

    if(INFO_enable_1s) mavlink_log_info(&_mavlink_log_pub,"纵%.0f 横%.0f 速差%.0f",double(dL_PtoPsp_project),double(PtoPsp_distance % MP_gndspd_ned_norm),double(dV_MPtoSP_project));

    //待办,注意这里使用的home的高度可能不对
    //待办,注意限制主机在编队时的转弯半径,目前是通过限制主机滚转角小于20度的方式限制,此时2号机能跟随,但是其他从机不确定能否正常跟随.20190622
    //待办,为了提高从机的响应速度,是否要给飞机增加姿态环的信息传输

    tecs_update_pitch_throttle(follow_alt_sp,
                               calculate_target_airspeed(airspeed_follow_sp),
                               radians(_parameters.pitch_limit_min) - _parameters.pitchsp_offset_rad,
                               radians(_parameters.pitch_limit_max) - _parameters.pitchsp_offset_rad,
                               _parameters.throttle_min,
                               _parameters.throttle_max,
                               throttle_follow_refer,
                               false,
                               radians(_parameters.pitch_limit_min));



    hrt_abstime now_utc_time3 = SP_gps_pos.time_utc_usec + hrt_elapsed_time(&SP_gps_pos.timestamp);




    //待办,飞机的降落监测程序要换成固定翼版本


    if(0){

        //待办:打印输出非常占用时间,调试好就取消
        //待办,改成用遥控器控制,这一段输出要人为控制

        //    mavlink_log_info(&_mavlink_log_pub, "#%d号传输超时",_vehicle_status.system_id);


        if(INFO_enable_1s) PX4_INFO("MP_position          .alt:\t%4.2f lat:\t%8.5f lon:\t%8.5f vx:\t%4.2f vy:\t%4.2f ",double(MP_position.alt),MP_position.lat,MP_position.lon,double(MP_position.vx),double(MP_position.vy));
        if(INFO_enable_1s) PX4_INFO("MP_position_filter   .alt:\t%4.2f lat:\t%8.5f lon:\t%8.5f vx:\t%4.2f vy:\t%4.2f ",double(MP_position_filter.alt),MP_position_filter.lat,MP_position_filter.lon,double(MP_position_filter.vx),double(MP_position_filter.vy));
        if(INFO_enable_1s) PX4_INFO("MP_position_filter_dL.alt:\t%4.2f lat:\t%8.5f lon:\t%8.5f vx:\t%4.2f vy:\t%4.2f ",double(MP_position_filter_dL.alt),MP_position_filter_dL.lat,MP_position_filter_dL.lon,double(MP_position_filter_dL.vx),double(MP_position_filter_dL.vy));
        if(INFO_enable_1s) PX4_INFO("SP_position_sp       .alt:\t%4.2f lat:\t%8.5f lon:\t%8.5f vx:\t%4.2f vy:\t%4.2f ",double(SP_position_sp.alt),SP_position_sp.lat,SP_position_sp.lon,double(SP_position_sp.vx),double(SP_position_sp.vy));
        if(INFO_enable_1s) PX4_INFO("SP_global_pos        .alt:\t%4.2f lat:\t%8.5f lon:\t%8.5f vx:\t%4.2f vy:\t%4.2f ",double(SP_global_pos.alt),SP_global_pos.lat,SP_global_pos.lon,double(SP_global_pos.vel_n),double(SP_global_pos.vel_e));
        if(INFO_enable_1s) PX4_INFO("PB_position_sp       .alt:\t%4.2f lat:\t%8.5f lon:\t%8.5f vx:\t%4.2f vy:\t%4.2f ",double(PB_position_sp.alt),PB_position_sp.lat,PB_position_sp.lon,double(PB_position_sp.vx),double(PB_position_sp.vy));
        if(INFO_enable_1s) PX4_INFO("PA_position_sp       .alt:\t%4.2f lat:\t%8.5f lon:\t%8.5f vx:\t%4.2f vy:\t%4.2f ",double(PA_position_sp.alt),PA_position_sp.lat,PA_position_sp.lon,double(PA_position_sp.vx),double(PA_position_sp.vy));



        if(INFO_enable_1s) PX4_INFO("                      currB_sp    lat:\t%8.5f lon:\t%8.5f",double(currB_sp(0)),double(currB_sp(1)));
        if(INFO_enable_1s) PX4_INFO("                      prevA_sp    lat:\t%8.5f lon:\t%8.5f",double(prevA_sp(0)),double(prevA_sp(1)));
        if(INFO_enable_1s) PX4_INFO("                      curr_pos    lat:\t%8.5f lon:\t%8.5f",double(curr_pos(0)),double(curr_pos(1)));
        if(INFO_enable_1s) PX4_INFO("                                             nav_speed_2d vx:\t%4.2f vy:\t%4.2f",double(nav_speed_2d(0)),double(nav_speed_2d(1)));

        if(INFO_enable_1s) PX4_INFO("   MP_position_filter.alt:\t%4.2f airspeed_follow_sp:\t%2.4f",double(MP_position_filter.alt),double(airspeed_follow_sp));






        if(INFO_enable_1s) PX4_INFO("bear_P2Psp_P1v = %.2f",double(bear_P2Psp_P1v));



        if(INFO_enable_1s) PX4_INFO("设置空速m/s:%.1f 距离差m:%.1f 速度差m/s:%.1f",double(airspeed_follow_sp),double(dL_PtoPsp_project),double(dV_MPtoSP_project));



        if(INFO_enable_1s) PX4_INFO(" MP_speed(0)= %.3f  MP_speed(1)= %.3f ",double(MP_speed(0)),double(MP_speed(1)))  ;
        if(INFO_enable_1s) PX4_INFO("MP_deltaL(0)= %.3f MP_deltaL(1)= %.3f ",double(MP_deltaL(0)),double(MP_deltaL(1)))  ;


        hrt_abstime now_utc_time4 = SP_gps_pos.time_utc_usec + hrt_elapsed_time(&SP_gps_pos.timestamp);

        if(INFO_enable_1s) PX4_INFO("使用延时s:%.2f",double(dt_utc_s));


        float dt_utc_s1 = (now_utc_time1-MP_position.timestamp) * 1e-6f;  //单位 秒 计算从机本地时间到主机时间戳的时间差
        float dt_utc_s2 = (now_utc_time2-MP_position.timestamp) * 1e-6f;  //单位 秒 计算从机本地时间到主机时间戳的时间差
        float dt_utc_s3 = (now_utc_time3-MP_position.timestamp) * 1e-6f;  //单位 秒 计算从机本地时间到主机时间戳的时间差
        float dt_utc_s4 = (now_utc_time4-MP_position.timestamp) * 1e-6f;  //单位 秒 计算从机本地时间到主机时间戳的时间差


        //总延时=传输时+程序时A    ,程序时B是这个程序段的运行时间
        if(INFO_enable_1s) PX4_INFO("第0延时s:%.3f 第1延时s:%.3f 第2延时s:%.3f 第3延时s:%.3f 第4延时s:%.3f",double(dt_utc_s0),double(dt_utc_s1),double(dt_utc_s2),double(dt_utc_s3),double(dt_utc_s4));



    }







}

//用aux2开关控制是否进入 followme模式,使用时需要将某个通道映射到aux2上.这里用个3段开关,位置在中间时不启用,在上下时表示两个测试状态,正.负
int8_t
FixedwingPositionControl::check_aux2_SW_enable()
{
    manual_control_setpoint_poll();
    if (_manual.aux2 <= 1.2f && _manual.aux2 >= -1.2f) {  //开关范围检测
        if (_manual.aux2 >= 0.4f){
            return 1;
        } else{
            return 0;
        }
    } else {
        return 0;
    }
}


void
FixedwingPositionControl::control_takeoff(const Vector2f &curr_pos, const Vector2f &ground_speed,
                                          const position_setpoint_s &pos_sp_prev, const position_setpoint_s &pos_sp_curr)
{
    /* current waypoint (the one currently heading for) */
    Vector2f curr_wp((float)pos_sp_curr.lat, (float)pos_sp_curr.lon);
    Vector2f prev_wp{0.0f, 0.0f}; /* previous waypoint */

    if (pos_sp_prev.valid) {
        prev_wp(0) = (float)pos_sp_prev.lat;
        prev_wp(1) = (float)pos_sp_prev.lon;

    } else {
        /*
         * No valid previous waypoint, go for the current wp.
         * This is automatically handled by the L1 library.
         */
        prev_wp(0) = (float)pos_sp_curr.lat;
        prev_wp(1) = (float)pos_sp_curr.lon;
    }

    // continuously reset launch detection and runway takeoff until armed
    if (!_control_mode.flag_armed) {
        _launchDetector.reset();
        _launch_detection_state = LAUNCHDETECTION_RES_NONE;
        _launch_detection_notify = 0;
    }

    if (_runway_takeoff.runwayTakeoffEnabled()) {
        if (!_runway_takeoff.isInitialized()) {
            Eulerf euler(Quatf(_att.q));
            _runway_takeoff.init(euler.psi(), _global_pos.lat, _global_pos.lon);

            /* need this already before takeoff is detected
             * doesn't matter if it gets reset when takeoff is detected eventually */
            _takeoff_ground_alt = _global_pos.alt;

            mavlink_log_info(&_mavlink_log_pub, "Takeoff on runway");
        }

        float terrain_alt = get_terrain_altitude_takeoff(_takeoff_ground_alt, _global_pos);

        // update runway takeoff helper
        _runway_takeoff.update(_airspeed, _global_pos.alt - terrain_alt,
                               _global_pos.lat, _global_pos.lon, &_mavlink_log_pub);

        /*
         * Update navigation: _runway_takeoff returns the start WP according to mode and phase.
         * If we use the navigator heading or not is decided later.
         */
        _l1_control.navigate_waypoints(_runway_takeoff.getStartWP(), curr_wp, curr_pos, ground_speed);

        // update tecs
        const float takeoff_pitch_max_deg = _runway_takeoff.getMaxPitch(_parameters.pitch_limit_max);

        tecs_update_pitch_throttle(pos_sp_curr.alt,
                                   calculate_target_airspeed(_runway_takeoff.getMinAirspeedScaling() * _parameters.airspeed_min),
                                   radians(_parameters.pitch_limit_min),
                                   radians(takeoff_pitch_max_deg),
                                   _parameters.throttle_min,
                                   _parameters.throttle_max, // XXX should we also set runway_takeoff_throttle here?
                                   _parameters.throttle_cruise,
                                   _runway_takeoff.climbout(),
                                   radians(_runway_takeoff.getMinPitch(pos_sp_curr.pitch_min, 10.0f, _parameters.pitch_limit_min)),
                                   tecs_status_s::TECS_MODE_TAKEOFF);

        // assign values
        _att_sp.roll_body = _runway_takeoff.getRoll(_l1_control.nav_roll());
        _att_sp.yaw_body = _runway_takeoff.getYaw(_l1_control.nav_bearing());
        _att_sp.fw_control_yaw = _runway_takeoff.controlYaw();
        _att_sp.pitch_body = _runway_takeoff.getPitch(get_tecs_pitch());

        // reset integrals except yaw (which also counts for the wheel controller)
        _att_sp.roll_reset_integral = _runway_takeoff.resetIntegrators();
        _att_sp.pitch_reset_integral = _runway_takeoff.resetIntegrators();

    } else {
        /* Perform launch detection */
        if (_launchDetector.launchDetectionEnabled() &&
                _launch_detection_state != LAUNCHDETECTION_RES_DETECTED_ENABLEMOTORS) {

            if (_control_mode.flag_armed) {
                /* Perform launch detection */

                /* Inform user that launchdetection is running every 4s */
                if (hrt_elapsed_time(&_launch_detection_notify) > 4e6) {
                    mavlink_log_critical(&_mavlink_log_pub, "Launch detection running");
                    _launch_detection_notify = hrt_absolute_time();
                }

                /* Detect launch using body X (forward) acceleration */
                _launchDetector.update(_sub_sensors.get().accel_x);

                /* update our copy of the launch detection state */
                _launch_detection_state = _launchDetector.getLaunchDetected();
            }

        } else	{
            /* no takeoff detection --> fly */
            _launch_detection_state = LAUNCHDETECTION_RES_DETECTED_ENABLEMOTORS;
        }

        /* Set control values depending on the detection state */
        if (_launch_detection_state != LAUNCHDETECTION_RES_NONE) {
            /* Launch has been detected, hence we have to control the plane. */

            _l1_control.navigate_waypoints(prev_wp, curr_wp, curr_pos, ground_speed);
            _att_sp.roll_body = _l1_control.nav_roll();
            _att_sp.yaw_body = _l1_control.nav_bearing();

            /* Select throttle: only in LAUNCHDETECTION_RES_DETECTED_ENABLEMOTORS we want to use
             * full throttle, otherwise we use idle throttle */
            float takeoff_throttle = _parameters.throttle_max;

            if (_launch_detection_state != LAUNCHDETECTION_RES_DETECTED_ENABLEMOTORS) {
                takeoff_throttle = _parameters.throttle_idle;
            }

            /* select maximum pitch: the launchdetector may impose another limit for the pitch
             * depending on the state of the launch */
            const float takeoff_pitch_max_deg = _launchDetector.getPitchMax(_parameters.pitch_limit_max);
            const float altitude_error = pos_sp_curr.alt - _global_pos.alt;

            /* apply minimum pitch and limit roll if target altitude is not within climbout_diff meters */
            if (_parameters.climbout_diff > 0.0f && altitude_error > _parameters.climbout_diff) {
                /* enforce a minimum of 10 degrees pitch up on takeoff, or take parameter */
                tecs_update_pitch_throttle(pos_sp_curr.alt,
                                           _parameters.airspeed_trim,
                                           radians(_parameters.pitch_limit_min),
                                           radians(takeoff_pitch_max_deg),
                                           _parameters.throttle_min,
                                           takeoff_throttle,
                                           _parameters.throttle_cruise,
                                           true,
                                           max(radians(pos_sp_curr.pitch_min), radians(10.0f)),
                                           tecs_status_s::TECS_MODE_TAKEOFF);

                /* limit roll motion to ensure enough lift */
                _att_sp.roll_body = constrain(_att_sp.roll_body, radians(-15.0f), radians(15.0f));

            } else {
                tecs_update_pitch_throttle(pos_sp_curr.alt,
                                           calculate_target_airspeed(_parameters.airspeed_trim),
                                           radians(_parameters.pitch_limit_min),
                                           radians(_parameters.pitch_limit_max),
                                           _parameters.throttle_min,
                                           takeoff_throttle,
                                           _parameters.throttle_cruise,
                                           false,
                                           radians(_parameters.pitch_limit_min));
            }

        } else {
            /* Tell the attitude controller to stop integrating while we are waiting
             * for the launch */
            _att_sp.roll_reset_integral = true;
            _att_sp.pitch_reset_integral = true;
            _att_sp.yaw_reset_integral = true;

            /* Set default roll and pitch setpoints during detection phase */
            _att_sp.roll_body = 0.0f;
            _att_sp.pitch_body = max(radians(pos_sp_curr.pitch_min), radians(10.0f));
        }
    }
}

void
FixedwingPositionControl::control_landing(const Vector2f &curr_pos, const Vector2f &ground_speed,
                                          const position_setpoint_s &pos_sp_prev, const position_setpoint_s &pos_sp_curr)
{
    /* current waypoint (the one currently heading for) */
    Vector2f curr_wp((float)pos_sp_curr.lat, (float)pos_sp_curr.lon);
    Vector2f prev_wp{0.0f, 0.0f}; /* previous waypoint */

    if (pos_sp_prev.valid) {
        prev_wp(0) = (float)pos_sp_prev.lat;
        prev_wp(1) = (float)pos_sp_prev.lon;

    } else {
        /*
         * No valid previous waypoint, go for the current wp.
         * This is automatically handled by the L1 library.
         */
        prev_wp(0) = (float)pos_sp_curr.lat;
        prev_wp(1) = (float)pos_sp_curr.lon;
    }

    // apply full flaps for landings. this flag will also trigger the use of flaperons
    // if they have been enabled using the corresponding parameter
    _att_sp.apply_flaps = true;

    // save time at which we started landing and reset abort_landing
    if (_time_started_landing == 0) {
        _time_started_landing = hrt_absolute_time();

        _fw_pos_ctrl_status.abort_landing = false;
    }

    const float bearing_airplane_currwp = get_bearing_to_next_waypoint(curr_pos(0), curr_pos(1), curr_wp(0), curr_wp(1));

    float bearing_lastwp_currwp = bearing_airplane_currwp;

    if (pos_sp_prev.valid) {
        bearing_lastwp_currwp = get_bearing_to_next_waypoint(prev_wp(0), prev_wp(1), curr_wp(0), curr_wp(1));
    }

    /* Horizontal landing control */
    /* switch to heading hold for the last meters, continue heading hold after */
    float wp_distance = get_distance_to_next_waypoint(curr_pos(0), curr_pos(1), curr_wp(0), curr_wp(1));

    /* calculate a waypoint distance value which is 0 when the aircraft is behind the waypoint */
    float wp_distance_save = wp_distance;

    if (fabsf(bearing_airplane_currwp - bearing_lastwp_currwp) >= radians(90.0f)) {
        wp_distance_save = 0.0f;
    }

    // create virtual waypoint which is on the desired flight path but
    // some distance behind landing waypoint. This will make sure that the plane
    // will always follow the desired flight path even if we get close or past
    // the landing waypoint
    if (pos_sp_prev.valid) {
        double lat = pos_sp_curr.lat;
        double lon = pos_sp_curr.lon;

        create_waypoint_from_line_and_dist(pos_sp_curr.lat, pos_sp_curr.lon,
                                           pos_sp_prev.lat, pos_sp_prev.lon, -1000.0f, &lat, &lon);

        curr_wp(0) = (float)lat;
        curr_wp(1) = (float)lon;
    }

    // we want the plane to keep tracking the desired flight path until we start flaring
    // if we go into heading hold mode earlier then we risk to be pushed away from the runway by cross winds
    if ((_parameters.land_heading_hold_horizontal_distance > 0.0f) && !_land_noreturn_horizontal &&
            ((wp_distance < _parameters.land_heading_hold_horizontal_distance) || _land_noreturn_vertical)) {

        if (pos_sp_prev.valid) {
            /* heading hold, along the line connecting this and the last waypoint */
            _target_bearing = bearing_lastwp_currwp;

        } else {
            _target_bearing = _yaw;
        }

        _land_noreturn_horizontal = true;
        mavlink_log_info(&_mavlink_log_pub, "Landing, heading hold");
    }

    if (_land_noreturn_horizontal) {
        // heading hold
        _l1_control.navigate_heading(_target_bearing, _yaw, ground_speed);

    } else {
        // normal navigation
        _l1_control.navigate_waypoints(prev_wp, curr_wp, curr_pos, ground_speed);
    }

    _att_sp.roll_body = _l1_control.nav_roll();
    _att_sp.yaw_body = _l1_control.nav_bearing();

    if (_land_noreturn_horizontal) {
        /* limit roll motion to prevent wings from touching the ground first */
        _att_sp.roll_body = constrain(_att_sp.roll_body, radians(-10.0f), radians(10.0f));
    }

    /* Vertical landing control */
    /* apply minimum pitch (flare) and limit roll if close to touch down, altitude error is negative (going down) */

    // default to no terrain estimation, just use landing waypoint altitude
    float terrain_alt = pos_sp_curr.alt;

    if (_parameters.land_use_terrain_estimate == 1) {
        if (_global_pos.terrain_alt_valid) {
            // all good, have valid terrain altitude
            terrain_alt = _global_pos.terrain_alt;
            _t_alt_prev_valid = terrain_alt;
            _time_last_t_alt = hrt_absolute_time();

        } else if (_time_last_t_alt == 0) {
            // we have started landing phase but don't have valid terrain
            // wait for some time, maybe we will soon get a valid estimate
            // until then just use the altitude of the landing waypoint
            if (hrt_elapsed_time(&_time_started_landing) < 10 * 1000 * 1000) {
                terrain_alt = pos_sp_curr.alt;

            } else {
                // still no valid terrain, abort landing
                terrain_alt = pos_sp_curr.alt;
                _fw_pos_ctrl_status.abort_landing = true;
            }

        } else if ((!_global_pos.terrain_alt_valid && hrt_elapsed_time(&_time_last_t_alt) < T_ALT_TIMEOUT * 1000 * 1000)
                   || _land_noreturn_vertical) {
            // use previous terrain estimate for some time and hope to recover
            // if we are already flaring (land_noreturn_vertical) then just
            //  go with the old estimate
            terrain_alt = _t_alt_prev_valid;

        } else {
            // terrain alt was not valid for long time, abort landing
            terrain_alt = _t_alt_prev_valid;
            _fw_pos_ctrl_status.abort_landing = true;
        }
    }

    /* Check if we should start flaring with a vertical and a
     * horizontal limit (with some tolerance)
     * The horizontal limit is only applied when we are in front of the wp
     */
    if (((_global_pos.alt < terrain_alt + _landingslope.flare_relative_alt()) &&
         (wp_distance_save < _landingslope.flare_length() + 5.0f)) ||
            _land_noreturn_vertical) {  //checking for land_noreturn to avoid unwanted climb out

        /* land with minimal speed */

        /* force TECS to only control speed with pitch, altitude is only implicitly controlled now */
        // _tecs.set_speed_weight(2.0f);

        /* kill the throttle if param requests it */
        float throttle_max = _parameters.throttle_max;

        /* enable direct yaw control using rudder/wheel */
        if (_land_noreturn_horizontal) {
            _att_sp.yaw_body = _target_bearing;
            _att_sp.fw_control_yaw = true;
        }

        if (_global_pos.alt < terrain_alt + _landingslope.motor_lim_relative_alt() || _land_motor_lim) {
            throttle_max = min(throttle_max, _parameters.throttle_land_max);

            if (!_land_motor_lim) {
                _land_motor_lim  = true;
                mavlink_log_info(&_mavlink_log_pub, "Landing, limiting throttle");
            }
        }

        float flare_curve_alt_rel = _landingslope.getFlareCurveRelativeAltitudeSave(wp_distance, bearing_lastwp_currwp,
                                                                                    bearing_airplane_currwp);

        /* avoid climbout */
        if ((_flare_curve_alt_rel_last < flare_curve_alt_rel && _land_noreturn_vertical) || _land_stayonground) {
            flare_curve_alt_rel = 0.0f; // stay on ground
            _land_stayonground = true;
        }

        const float airspeed_land = _parameters.land_airspeed_scale * _parameters.airspeed_min;
        const float throttle_land = _parameters.throttle_min + (_parameters.throttle_max - _parameters.throttle_min) * 0.1f;

        tecs_update_pitch_throttle(terrain_alt + flare_curve_alt_rel,
                                   calculate_target_airspeed(airspeed_land),
                                   radians(_parameters.land_flare_pitch_min_deg),
                                   radians(_parameters.land_flare_pitch_max_deg),
                                   0.0f,
                                   throttle_max,
                                   throttle_land,
                                   false,
                                   _land_motor_lim ? radians(_parameters.land_flare_pitch_min_deg) : radians(_parameters.pitch_limit_min),
                                   _land_motor_lim ? tecs_status_s::TECS_MODE_LAND_THROTTLELIM : tecs_status_s::TECS_MODE_LAND);

        if (!_land_noreturn_vertical) {
            // just started with the flaring phase
            _flare_pitch_sp = 0.0f;
            _flare_height = _global_pos.alt - terrain_alt;
            mavlink_log_info(&_mavlink_log_pub, "Landing, flaring");
            _land_noreturn_vertical = true;

        } else {
            if (_global_pos.vel_d > 0.1f) {
                _flare_pitch_sp = radians(_parameters.land_flare_pitch_min_deg) *
                        constrain((_flare_height - (_global_pos.alt - terrain_alt)) / _flare_height, 0.0f, 1.0f);
            }

            // otherwise continue using previous _flare_pitch_sp
        }

        _att_sp.pitch_body = _flare_pitch_sp;
        _flare_curve_alt_rel_last = flare_curve_alt_rel;

    } else {

        /* intersect glide slope:
         * minimize speed to approach speed
         * if current position is higher than the slope follow the glide slope (sink to the
         * glide slope)
         * also if the system captures the slope it should stay
         * on the slope (bool land_onslope)
         * if current position is below the slope continue at previous wp altitude
         * until the intersection with slope
         * */

        float altitude_desired = terrain_alt;

        const float landing_slope_alt_rel_desired = _landingslope.getLandingSlopeRelativeAltitudeSave(wp_distance,
                                                                                                      bearing_lastwp_currwp, bearing_airplane_currwp);

        if (_global_pos.alt > terrain_alt + landing_slope_alt_rel_desired || _land_onslope) {
            /* stay on slope */
            altitude_desired = terrain_alt + landing_slope_alt_rel_desired;

            if (!_land_onslope) {
                mavlink_log_info(&_mavlink_log_pub, "Landing, on slope");
                _land_onslope = true;
            }

        } else {
            /* continue horizontally */
            if (pos_sp_prev.valid) {
                altitude_desired = pos_sp_prev.alt;

            } else {
                altitude_desired = _global_pos.alt;
            }
        }

        const float airspeed_approach = _parameters.land_airspeed_scale * _parameters.airspeed_min;

        tecs_update_pitch_throttle(altitude_desired,
                                   calculate_target_airspeed(airspeed_approach),
                                   radians(_parameters.pitch_limit_min),
                                   radians(_parameters.pitch_limit_max),
                                   _parameters.throttle_min,
                                   _parameters.throttle_max,
                                   _parameters.throttle_cruise,
                                   false,
                                   radians(_parameters.pitch_limit_min));
    }
}

float
FixedwingPositionControl::get_tecs_pitch()
{
    if (_is_tecs_running) {
        return _tecs.get_pitch_setpoint();
    }

    // return 0 to prevent stale tecs state when it's not running
    return 0.0f;
}

float
FixedwingPositionControl::get_tecs_thrust()
{
    if (_is_tecs_running) {
        return _tecs.get_throttle_setpoint();
    }

    // return 0 to prevent stale tecs state when it's not running
    return 0.0f;
}

void
FixedwingPositionControl::handle_command()
{
    if (_vehicle_command.command == vehicle_command_s::VEHICLE_CMD_DO_GO_AROUND) {
        // only abort landing before point of no return (horizontal and vertical)
        if (_control_mode.flag_control_auto_enabled &&
                _pos_sp_triplet.current.valid &&
                _pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_LAND) {

            _fw_pos_ctrl_status.abort_landing = true;
            mavlink_log_critical(&_mavlink_log_pub, "Landing aborted");
        }
    }
}

void
FixedwingPositionControl::run()
{
    /*
     * do subscriptions
     */
    _global_pos_sub = orb_subscribe(ORB_ID(vehicle_global_position));
    _local_pos_sub = orb_subscribe(ORB_ID(vehicle_local_position));
    _pos_sp_triplet_sub = orb_subscribe(ORB_ID(position_setpoint_triplet));
    _control_mode_sub = orb_subscribe(ORB_ID(vehicle_control_mode));
    _vehicle_attitude_sub = orb_subscribe(ORB_ID(vehicle_attitude));
    _vehicle_command_sub = orb_subscribe(ORB_ID(vehicle_command));
    _vehicle_status_sub = orb_subscribe(ORB_ID(vehicle_status));
    _vehicle_land_detected_sub = orb_subscribe(ORB_ID(vehicle_land_detected));
    _params_sub = orb_subscribe(ORB_ID(parameter_update));
    _manual_control_sub = orb_subscribe(ORB_ID(manual_control_setpoint));
    _sensor_baro_sub = orb_subscribe(ORB_ID(sensor_baro));
    _vehicle_gps_position_sub = orb_subscribe(ORB_ID(vehicle_gps_position));

    /* rate limit control mode updates to 5Hz */
    orb_set_interval(_control_mode_sub, 200);

    /* rate limit vehicle status updates to 5Hz */
    orb_set_interval(_vehicle_status_sub, 200);

    /* rate limit vehicle land detected updates to 5Hz */
    orb_set_interval(_vehicle_land_detected_sub, 200);

    /* rate limit position updates to 50 Hz */
    orb_set_interval(_global_pos_sub, 20);

    /* rate limit barometer updates to 1 Hz */
    orb_set_interval(_sensor_baro_sub, 1000);

    /* abort on a nonzero return value from the parameter init */
    if (parameters_update() != PX4_OK) {
        /* parameter setup went wrong, abort */
        PX4_ERR("aborting startup due to errors.");
    }

    /* wakeup source(s) */
    px4_pollfd_struct_t fds[1];

    /* Setup of loop */
    fds[0].fd = _global_pos_sub;
    fds[0].events = POLLIN;



    while (!should_exit()) {

        /* wait for up to 500ms for data */
        int pret = px4_poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), 100);

        /* timed out - periodic check for _task_should_exit, etc. */
        if (pret == 0) {
            continue;
        }

        /* this is undesirable but not much we can do - might want to flag unhappy status */
        if (pret < 0) {
            PX4_WARN("poll error %d, %d", pret, errno);
            continue;
        }

        /* only update parameters if they changed */
        bool params_updated = false;
        orb_check(_params_sub, &params_updated);

        if (params_updated) {
            /* read from param to clear updated flag */
            parameter_update_s update;
            orb_copy(ORB_ID(parameter_update), _params_sub, &update);

            /* update parameters from storage */
            parameters_update();
        }

        /* only run controller if position changed */
        if ((fds[0].revents & POLLIN) != 0) {
            perf_begin(_loop_perf);

            /* load local copies */
            orb_copy(ORB_ID(vehicle_global_position), _global_pos_sub, &_global_pos);
            orb_copy(ORB_ID(vehicle_local_position), _local_pos_sub, &_local_pos);

            // handle estimator reset events. we only adjust setpoins for manual modes
            if (_control_mode.flag_control_manual_enabled) {
                if (_control_mode.flag_control_altitude_enabled && _global_pos.alt_reset_counter != _alt_reset_counter) {
                    _hold_alt += _global_pos.delta_alt;
                    // make TECS accept step in altitude and demanded altitude
                    _tecs.handle_alt_step(_global_pos.delta_alt, _global_pos.alt);
                }

                // adjust navigation waypoints in position control mode
                if (_control_mode.flag_control_altitude_enabled && _control_mode.flag_control_velocity_enabled
                        && _global_pos.lat_lon_reset_counter != _pos_reset_counter) {

                    // reset heading hold flag, which will re-initialise position control
                    _hdg_hold_enabled = false;
                }
            }

            // update the reset counters in any case
            _alt_reset_counter = _global_pos.alt_reset_counter;
            _pos_reset_counter = _global_pos.lat_lon_reset_counter;

            _sub_sensors.update();
            airspeed_poll();
            manual_control_setpoint_poll();
            position_setpoint_triplet_poll();
            vehicle_attitude_poll();
            vehicle_command_poll();
            vehicle_control_mode_poll();
            vehicle_land_detected_poll();
            vehicle_status_poll();

            Vector2f curr_pos((float)_global_pos.lat, (float)_global_pos.lon);
            Vector2f ground_speed(_global_pos.vel_n, _global_pos.vel_e);

            //

            /*
             * Attempt to control position, on success (= sensors present and not in manual mode),
             * publish setpoint.
             */

            //            if(INFO_enable_1s) PX4_INFO("_pos_sp_triplet.current.vx = %.1f",double(_pos_sp_triplet.current.vx));


            if (control_position(curr_pos, ground_speed, _pos_sp_triplet.previous, _pos_sp_triplet.current)) {
                _att_sp.timestamp = hrt_absolute_time();

                // add attitude setpoint offsets
                _att_sp.roll_body += _parameters.rollsp_offset_rad;
                _att_sp.pitch_body += _parameters.pitchsp_offset_rad;

                if (_control_mode.flag_control_manual_enabled) {
                    _att_sp.roll_body = constrain(_att_sp.roll_body, -_parameters.man_roll_max_rad, _parameters.man_roll_max_rad);
                    _att_sp.pitch_body = constrain(_att_sp.pitch_body, -_parameters.man_pitch_max_rad, _parameters.man_pitch_max_rad);
                }

                Quatf q(Eulerf(_att_sp.roll_body, _att_sp.pitch_body, _att_sp.yaw_body));
                q.copyTo(_att_sp.q_d);
                _att_sp.q_d_valid = true;

                if (!_control_mode.flag_control_offboard_enabled ||
                        _control_mode.flag_control_position_enabled ||
                        _control_mode.flag_control_velocity_enabled ||
                        _control_mode.flag_control_acceleration_enabled) {

                    /* lazily publish the setpoint only once available */
                    if (_attitude_sp_pub != nullptr) {
                        /* publish the attitude setpoint */
                        orb_publish(_attitude_setpoint_id, _attitude_sp_pub, &_att_sp);

                    } else if (_attitude_setpoint_id != nullptr) {
                        /* advertise and publish */
                        _attitude_sp_pub = orb_advertise(_attitude_setpoint_id, &_att_sp);
                    }
                }

                /* XXX check if radius makes sense here */
                float turn_distance = _l1_control.switch_distance(500.0f);

                /* lazily publish navigation capabilities */
                if ((hrt_elapsed_time(&_fw_pos_ctrl_status.timestamp) > 1000000)
                        || (fabsf(turn_distance - _fw_pos_ctrl_status.turn_distance) > FLT_EPSILON
                            && turn_distance > 0)) {

                    /* set new turn distance */
                    _fw_pos_ctrl_status.turn_distance = turn_distance;

                    _fw_pos_ctrl_status.nav_roll = _l1_control.nav_roll();
                    _fw_pos_ctrl_status.nav_pitch = get_tecs_pitch();
                    _fw_pos_ctrl_status.nav_bearing = _l1_control.nav_bearing();

                    _fw_pos_ctrl_status.target_bearing = _l1_control.target_bearing();
                    _fw_pos_ctrl_status.xtrack_error = _l1_control.crosstrack_error();

                    Vector2f curr_wp((float)_pos_sp_triplet.current.lat, (float)_pos_sp_triplet.current.lon);

                    _fw_pos_ctrl_status.wp_dist = get_distance_to_next_waypoint(curr_pos(0), curr_pos(1), curr_wp(0), curr_wp(1));

                    fw_pos_ctrl_status_publish();
                }
            }

            perf_end(_loop_perf);
        }



        INFO_enable_1s = false;
        INFO_enable1s = false;

    }
}

void
FixedwingPositionControl::reset_takeoff_state()
{
    // only reset takeoff if !armed or just landed
    if (!_control_mode.flag_armed || (_was_in_air && _vehicle_land_detected.landed)) {

        _runway_takeoff.reset();

        _launchDetector.reset();
        _launch_detection_state = LAUNCHDETECTION_RES_NONE;
        _launch_detection_notify = 0;

    } else {
        _launch_detection_state = LAUNCHDETECTION_RES_DETECTED_ENABLEMOTORS;
    }
}

void
FixedwingPositionControl::reset_landing_state()
{
    _time_started_landing = 0;

    // reset terrain estimation relevant values
    _time_last_t_alt = 0;

    _land_noreturn_horizontal = false;
    _land_noreturn_vertical = false;
    _land_stayonground = false;
    _land_motor_lim = false;
    _land_onslope = false;

    // reset abort land, unless loitering after an abort
    if (_fw_pos_ctrl_status.abort_landing
            && _pos_sp_triplet.current.type != position_setpoint_s::SETPOINT_TYPE_LOITER) {

        _fw_pos_ctrl_status.abort_landing = false;
    }
}

void
FixedwingPositionControl::tecs_update_pitch_throttle(float alt_sp, float airspeed_sp,
                                                     float pitch_min_rad, float pitch_max_rad,
                                                     float throttle_min, float throttle_max, float throttle_cruise,
                                                     bool climbout_mode, float climbout_pitch_min_rad,
                                                     uint8_t mode)
{
    float dt = 0.01f; // prevent division with 0

    if (_last_tecs_update > 0) {
        dt = hrt_elapsed_time(&_last_tecs_update) * 1e-6;
    }

    _last_tecs_update = hrt_absolute_time();

    // do not run TECS if we are not in air
    bool run_tecs = !_vehicle_land_detected.landed;

    // do not run TECS if vehicle is a VTOL and we are in rotary wing mode or in transition
    // (it should also not run during VTOL blending because airspeed is too low still)
    if (_vehicle_status.is_vtol) {
        if (_vehicle_status.is_rotary_wing || _vehicle_status.in_transition_mode) {
            run_tecs = false;
        }

        if (_vehicle_status.in_transition_mode) {
            // we're in transition
            _was_in_transition = true;

            // set this to transition airspeed to init tecs correctly
            if (_parameters.airspeed_disabled) {
                // some vtols fly without airspeed sensor
                _asp_after_transition = _parameters.airspeed_trans;

            } else {
                _asp_after_transition = _airspeed;
            }

            _asp_after_transition = constrain(_asp_after_transition, _parameters.airspeed_min, _parameters.airspeed_max);

        } else if (_was_in_transition) {
            // after transition we ramp up desired airspeed from the speed we had coming out of the transition
            _asp_after_transition += dt * 2; // increase 2m/s

            if (_asp_after_transition < airspeed_sp && _airspeed < airspeed_sp) {
                airspeed_sp = max(_asp_after_transition, _airspeed);

            } else {
                _was_in_transition = false;
                _asp_after_transition = 0;
            }
        }
    }

    _is_tecs_running = run_tecs;

    if (!run_tecs) {
        // next time we run TECS we should reinitialize states
        _reinitialize_tecs = true;
        return;
    }

    if (_reinitialize_tecs) {
        _tecs.reset_state();
        _reinitialize_tecs = false;
    }

    if (_vehicle_status.engine_failure) {
        /* Force the slow downwards spiral */
        pitch_min_rad = M_DEG_TO_RAD_F * -1.0f;
        pitch_max_rad = M_DEG_TO_RAD_F * 5.0f;
    }

    /* No underspeed protection in landing mode */
    _tecs.set_detect_underspeed_enabled(!(mode == tecs_status_s::TECS_MODE_LAND
                                          || mode == tecs_status_s::TECS_MODE_LAND_THROTTLELIM));

    /* Using tecs library */
    float pitch_for_tecs = _pitch - _parameters.pitchsp_offset_rad;

    /* filter speed and altitude for controller */
    Vector3f accel_body(_sub_sensors.get().accel_x, _sub_sensors.get().accel_y, _sub_sensors.get().accel_z);

    // tailsitters use the multicopter frame as reference, in fixed wing
    // we need to use the fixed wing frame
    if (_parameters.vtol_type == vtol_type::TAILSITTER && _vehicle_status.is_vtol) {
        float tmp = accel_body(0);
        accel_body(0) = -accel_body(2);
        accel_body(2) = tmp;
    }

    /* tell TECS to update its state, but let it know when it cannot actually control the plane */
    bool in_air_alt_control = (!_vehicle_land_detected.landed &&
                               (_control_mode.flag_control_auto_enabled ||
                                _control_mode.flag_control_velocity_enabled ||
                                _control_mode.flag_control_altitude_enabled));

    /* update TECS vehicle state estimates */
    _tecs.update_vehicle_state_estimates(_airspeed, _R_nb,
                                         accel_body, (_global_pos.timestamp > 0), in_air_alt_control,
                                         _global_pos.alt, _local_pos.v_z_valid, _local_pos.vz, _local_pos.az);

    /* scale throttle cruise by baro pressure */
    if (_parameters.throttle_alt_scale > FLT_EPSILON) {

        bool baro_updated = false;
        orb_check(_sensor_baro_sub, &baro_updated);

        sensor_baro_s baro;

        if (orb_copy(ORB_ID(sensor_baro), _sensor_baro_sub, &baro) == PX4_OK) {
            if (PX4_ISFINITE(baro.pressure) && PX4_ISFINITE(_parameters.throttle_alt_scale)) {
                // scale throttle as a function of sqrt(p0/p) (~ EAS -> TAS at low speeds and altitudes ignoring temperature)
                const float eas2tas = sqrtf(CONSTANTS_STD_PRESSURE_MBAR / baro.pressure);
                const float scale = constrain((eas2tas - 1.0f) * _parameters.throttle_alt_scale + 1.0f, 1.0f, 2.0f);

                throttle_max = constrain(throttle_max * scale, throttle_min, 1.0f);
                throttle_cruise = constrain(throttle_cruise * scale, throttle_min + 0.01f, throttle_max - 0.01f);
            }
        }
    }

    _tecs.update_pitch_throttle(_R_nb, pitch_for_tecs,
                                _global_pos.alt, alt_sp,
                                airspeed_sp, _airspeed, _eas2tas,
                                climbout_mode, climbout_pitch_min_rad,
                                throttle_min, throttle_max, throttle_cruise,
                                pitch_min_rad, pitch_max_rad);

    tecs_status_s t;
    t.timestamp = _tecs.timestamp();

    switch (_tecs.tecs_mode()) {
    case TECS::ECL_TECS_MODE_NORMAL:
        t.mode = tecs_status_s::TECS_MODE_NORMAL;
        break;

    case TECS::ECL_TECS_MODE_UNDERSPEED:
        t.mode = tecs_status_s::TECS_MODE_UNDERSPEED;
        break;

    case TECS::ECL_TECS_MODE_BAD_DESCENT:
        t.mode = tecs_status_s::TECS_MODE_BAD_DESCENT;
        break;

    case TECS::ECL_TECS_MODE_CLIMBOUT:
        t.mode = tecs_status_s::TECS_MODE_CLIMBOUT;
        break;
    }

    t.altitudeSp			= _tecs.hgt_setpoint_adj();
    t.altitude_filtered		= _tecs.vert_pos_state();
    t.airspeedSp			= _tecs.TAS_setpoint_adj();
    t.airspeed_filtered 	= _tecs.tas_state();

    t.flightPathAngleSp		= _tecs.hgt_rate_setpoint();
    t.flightPathAngle		= _tecs.vert_vel_state();

    t.airspeedDerivativeSp	= _tecs.TAS_rate_setpoint();
    t.airspeedDerivative	= _tecs.speed_derivative();

    t.totalEnergyError				= _tecs.STE_error();
    t.totalEnergyRateError			= _tecs.STE_rate_error();
    t.energyDistributionError		= _tecs.SEB_error();
    t.energyDistributionRateError	= _tecs.SEB_rate_error();

    t.throttle_integ	= _tecs.throttle_integ_state();
    t.pitch_integ		= _tecs.pitch_integ_state();

    if (_tecs_status_pub != nullptr) {
        orb_publish(ORB_ID(tecs_status), _tecs_status_pub, &t);

    } else {
        _tecs_status_pub = orb_advertise(ORB_ID(tecs_status), &t);
    }
}

FixedwingPositionControl *FixedwingPositionControl::instantiate(int argc, char *argv[])
{
    return new FixedwingPositionControl();
}

int FixedwingPositionControl::task_spawn(int argc, char *argv[])
{
    _task_id = px4_task_spawn_cmd("fw_pos_control_l1",
                                  SCHED_DEFAULT,
                                  SCHED_PRIORITY_POSITION_CONTROL,
                                  3000,
                                  (px4_main_t)&run_trampoline,
                                  (char *const *)argv);

    if (_task_id < 0) {
        _task_id = -1;
        return -errno;
    }

    return 0;
}

int FixedwingPositionControl::custom_command(int argc, char *argv[])
{
    return print_usage("unknown command");
}

int FixedwingPositionControl::print_usage(const char *reason)
{
    if (reason) {
        PX4_WARN("%s\n", reason);
    }

    PRINT_MODULE_DESCRIPTION(
                R"DESCR_STR(
                ### Description
                fw_pos_control_l1 is the fixed wing position controller.

                )DESCR_STR");


            PRINT_MODULE_USAGE_NAME("fw_pos_control_l1", "controller");
    PRINT_MODULE_USAGE_COMMAND("start");
    PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

    return 0;
}

int FixedwingPositionControl::print_status()
{
    PX4_INFO("Running");

    return 0;
}

int fw_pos_control_l1_main(int argc, char *argv[])
{
    return FixedwingPositionControl::main(argc, argv);
}
