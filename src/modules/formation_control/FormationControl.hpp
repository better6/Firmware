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

#include <px4_module.h>
//#include <drivers/drv_hrt.h>


#include <px4_config.h>
#include <px4_defines.h>
#include <px4_posix.h>
#include <px4_tasks.h>
#include <mathlib/mathlib.h>
#include <matrix/math.hpp>
#include <lib/ecl/geo/geo.h>
#include <lib/mathlib/math/Limits.hpp>
#include <parameters/param.h>
#include <perf/perf_counter.h>
#include <uORB/Subscription.hpp>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/airspeed.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/rate_ctrl_status.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_gps_position.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/vehicle_status.h>

#include <uORB/topics/formationx.h>
#include <uORB/topics/follow_target.h>
#include <uORB/topics/sensor_baro.h>
#include <uORB/topics/fw_pos_ctrl_status.h>
//#include <uORB/topics/offboard_control_mode.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/follow_target.h>



#include <vtol_att_control/vtol_type.h>
#include <systemlib/mavlink_log.h>

#include <uORB/topics/sensor_combined.h>

//#include "mavlink_orb_subscription.h"


using matrix::Vector2f;



#define mainplaneID 1  //暂定主机为1号机


using uORB::Subscription;

class FormationControl final : public ModuleBase<FormationControl>
{
public:
        FormationControl();
        ~FormationControl() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
        static FormationControl *instantiate(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase::run() */
	void run() override;

	/** @see ModuleBase::print_status() */
	int print_status() override;

private:

    int     _att_sub{-1};
    int     _follow_target_sub{-1};
    int		_params_sub{-1};			/**< notification of parameter updates */
    int		_set_offboard_sub{-1};
    int		_vehicle_status_sub{-1};
    int	 _vehicle_global_position_sub{-1};
    int	 _vehicle_gps_position_sub{-1};
//    int	 _follow_target_sub{-1};

    int     _manual_control_setpoint_sub{-1};

    bool run_test{false};

    bool INFO_enable{false};



    int mav_sysid{-1};
    int mav_compid{-1};


//定义编队队形:1字形,菱形,等等
    enum FORMATION_SHAPE {
        FORMATION_horizon1,
        FORMATION_vertical1,
        FORMATION_rhombus4
    } _form_shape_current{FORMATION_vertical1};  //默认队形是纵向1字形



    int FORMATION_rhombus4_axis[4][2] = {      //4机菱形编队的坐标集合,{主机地轴航向前后位置,左右位置}向前为正,向右为正
        { 0, 0}, // 1号机位置(主机) 坐标原点
        {-1, 1}, // 2号机位置,主机右边,后面
        {-2, 0}, // 3号机位置,主机后面
        {-1,-1}  // 4号机位置,主机左边,后面
    };







    orb_advert_t    P1_send_pub{nullptr};			/**<  */
    orb_advert_t	_mavlink_log_pub{nullptr};
//    orb_advert_t    _offboard_control_mode_pub{nullptr};
    orb_advert_t    _vehicle_command_pub{nullptr};

    orb_advert_t    _send_follow_target_pub{nullptr};


    orb_id_t P1_send_id{nullptr};

    formationx_s         P1_send{}; /*自定义的编队控制结构体 */
    follow_target_s         P1_received{}; /*从机收到的主机在之前某一时刻的位姿状态 */
     follow_target_s         P1_send_target{}; /*从机收到的主机在之前某一时刻的位姿状态 */
    vehicle_gps_position_s P1_gps_pos{};





    vehicle_command_s           _set_offboard{}; /*自定义的编队控制结构体 */
    //    offboard_control_mode_s     _offboard_control_mode{};
    vehicle_command_s           _command{};

    follow_target_s             _send_follow_target{}; /*自定义的编队控制结构体 */

    uint8_t sys_id{0};
    uint8_t comp_id{0};
    uint8_t nav_status{0};
    bool status_valid{false};
    bool pub{true};

    manual_control_setpoint_s   _manual_sp{};

    vehicle_attitude_s   _mainuav_att{};

	perf_counter_t	_loop_perf;			/**< loop performance counter */
	perf_counter_t	_nonfinite_input_perf;		/**< performance counter for non finite input */
	perf_counter_t	_nonfinite_output_perf;		/**< performance counter for non finite output */

    bool			_idparam_initialized;
    param_t			_param_system_id;
    param_t			_param_component_id;

    void        follow_target_sp_poll();

    void        vehicle_gps_position_poll();
    void        status_poll();

    void        test_data_program(bool enable_test);


    void        attitude_poll();
    void        formationx_sp_publish();
    void        hold_offboard_status();
    void        enable_offboard_mode(bool offboard_sw);
    void        param_update_system();
    void        send_follow_target_publish();
    void        enable_follow_target_mode(bool follow_target_enabled);
    void        manual_control_setpoint_poll();

    bool        check_aux1_enable_follow();
    bool        check_aux3_enable_follow();

};
