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

#include "FormationControl.hpp"

/**
 * Fixedwing attitude control app start / stop handling function
 *
 * @ingroup apps
 */
extern "C" __EXPORT int formation_control_main(int argc, char *argv[]);

FormationControl::FormationControl():
    _idparam_initialized(false),
  _param_system_id(PARAM_INVALID),
  _param_component_id(PARAM_INVALID)
{

}

FormationControl::~FormationControl()
{

}




void
FormationControl::manual_control_setpoint_poll()
{
    /* check if there is a new setpoint */
    bool _updated;
    orb_check(_manual_control_setpoint_sub, &_updated);

    if (_updated) {
        orb_copy(ORB_ID(manual_control_setpoint), _manual_control_setpoint_sub, &_manual_sp);
    }
}

//测试数据发送程序,一直发送假的GPS位置点和地速
void
FormationControl::test_data_program(bool enable_test)
{

    if(enable_test == false){ //程序执行的开关
        return;
    }

    run_test = enable_test;


}


void
FormationControl::follow_target_sp_poll()
{
    /* check if there is a new setpoint */
    bool follow_target_updated;
    orb_check(_follow_target_sub, &follow_target_updated);

    if (follow_target_updated) {
        orb_copy(ORB_ID(follow_target), _follow_target_sub, &P1_received);


        //printf("P1_received.alt :\t%8.4f  \n",double(P1_received.alt));

    }






}

void
FormationControl::vehicle_gps_position_poll()
{
    /* check if there is a new setpoint */
    bool vehicle_gps_position_updated;
    orb_check(_vehicle_gps_position_sub, &vehicle_gps_position_updated);

    if (vehicle_gps_position_updated) {
        orb_copy(ORB_ID(vehicle_gps_position), _vehicle_gps_position_sub, &P1_gps_pos);

    }

}





void
FormationControl::status_poll()
{
    bool _updated;
    static vehicle_status_s            status{};
    orb_check(_vehicle_status_sub, &_updated);
    if (_updated) {
          orb_copy(ORB_ID(vehicle_status), _vehicle_status_sub, &status);  //从飞行器状态中获得sysid

          sys_id = status.system_id;
          comp_id = status.component_id;
          status_valid = (status.timestamp > 0);
          nav_status = status.nav_state;

    }
}

void
FormationControl::attitude_poll()
{
    bool _updated;
    orb_check(_att_sub, &_updated);
    if (_updated) {
          orb_copy(ORB_ID(vehicle_attitude), _att_sub, &_mainuav_att);  //从飞行器状态中获得sysid
    }
}









void
FormationControl::formationx_sp_publish()
{

    if (P1_send_pub != nullptr) {
        /* publish the attitude rates setpoint */
        orb_publish(ORB_ID(formationx), P1_send_pub, &P1_send);
//            if(INFO_enable) PX4_INFO("推送formationx_sp_publish");

    } else {
        /* advertise the attitude rates setpoint */
        P1_send_pub = orb_advertise(ORB_ID(formationx), &P1_send);
    }
}



//从机向本机uorb总线发送follow_target的目标位置和速度
void
FormationControl::send_follow_target_publish()
{

    if (_send_follow_target_pub != nullptr) {
        /* publish the attitude rates setpoint */
        orb_publish(ORB_ID(follow_target), _send_follow_target_pub, &_send_follow_target);
//        if(INFO_enable) PX4_INFO("当前是从机,推送ORB_ID(follow_target)");

    } else {
        /* advertise the attitude rates setpoint */
        _send_follow_target_pub = orb_advertise(ORB_ID(follow_target), &_send_follow_target);
    }
}




//void
//FormationControl::hold_offboard_status()
//{
//    _offboard_control_mode.ignore_position = true;
//    _offboard_control_mode.ignore_velocity = true;
//    _offboard_control_mode.ignore_acceleration_force = true;

//    _offboard_control_mode.timestamp = hrt_absolute_time();


//    if (_offboard_control_mode_pub == nullptr) {
//        _offboard_control_mode_pub = orb_advertise(ORB_ID(offboard_control_mode), &_offboard_control_mode);
//    } else {
//        orb_publish(ORB_ID(offboard_control_mode), _offboard_control_mode_pub, &_offboard_control_mode);
////         if(INFO_enable) PX4_INFO("推送offboard_control_mode_publish");
//    }
//}

//void
//FormationControl::enable_offboard_mode(bool offboard_sw)
//{
//    orb_copy(ORB_ID(vehicle_status), _vehicle_status_sub, &status);
//    _command.command            = vehicle_command_s::VEHICLE_CMD_NAV_GUIDED_ENABLE;//设置模式命令id
//    _command.param1             = offboard_sw;
//    _command.target_system      = status.system_id;
//    _command.target_component   = status.component_id;

//    if (_vehicle_command_pub != nullptr) {
//        orb_publish(ORB_ID(vehicle_command), _vehicle_command_pub, &_command);//发布这个命令
//    } else {
//        _vehicle_command_pub = orb_advertise(ORB_ID(vehicle_command), &_command);
//    }

//}

//用aux1开关控制是否进入 followme模式,使用时需要将某个通道映射到aux1上.
bool
FormationControl::check_aux1_enable_follow()
{
//  return true;  //调试语句,注意删除,有遥控器时不用这句
    manual_control_setpoint_poll();

//    printf("_manual_sp.aux1 : %.2f \n",(double)_manual_sp.aux1);
    if (_manual_sp.aux1 <= 1.2f && _manual_sp.aux1 >= -1.2f) {
        if (_manual_sp.aux1 >= 0.5f){
            return true;
        } else {
            return false;
        }
    } else
        return false;
}

//用aux3开关控制是否切换编队队形
bool
FormationControl::check_aux3_enable_follow()
{
//  return true;  //调试语句,注意删除,有遥控器时不用这句
    manual_control_setpoint_poll();

//    printf("_manual_sp.aux3 : %.2f \n",(double)_manual_sp.aux3);
    if (_manual_sp.aux3 <= 1.2f && _manual_sp.aux3 >= -1.2f) {
        if (_manual_sp.aux3 >= 0.5f){
            return true;
        } else {
            return false;
        }
    } else
        return false;
}





void
FormationControl::enable_follow_target_mode(bool follow_target_enabled)
{
    if (follow_target_enabled) {

        //status_poll();  //获取飞机状态


        //        printf("启用followme模式 %d \n",follow_target_enabled);
        //

        //if(nav_status != vehicle_status_s::NAVIGATION_STATE_AUTO_FOLLOW_TARGET){

            _command.command            = vehicle_command_s::VEHICLE_CMD_DO_SET_MODE;//设置模式命令id
            _command.param1             = 1.0f;
            _command.param2             = 4.0f;
            _command.param3             = 8.0f;
            _command.target_system      = sys_id;
            _command.target_component   = comp_id;

            if (_vehicle_command_pub != nullptr) {
                orb_publish(ORB_ID(vehicle_command), _vehicle_command_pub, &_command);//发布这个命令
                pub=false;
            } else {
                _vehicle_command_pub = orb_advertise(ORB_ID(vehicle_command), &_command);
            }
       // }

    }
}



//void
//FormationControl::heartbeat_poll()
//{
//    /* check if there is a new setpoint */
//    bool heartbeat_updated;
//    orb_check(_heartbeat_sub, &heartbeat_updated);

//    if (heartbeat_updated) {
//        orb_copy(ORB_ID(heartbeat), _heartbeat_sub, &_heartbeat);
//    }
//}


//从参数设置中获得sysid
void FormationControl::param_update_system()
{
    _param_system_id = param_find("MAV_SYS_ID");
    _param_component_id = param_find("MAV_COMP_ID");

    /* update system and component id */
    int32_t system_id;
    param_get(_param_system_id, &system_id);

    int32_t component_id;
    param_get(_param_component_id, &component_id);

    if (system_id > 0 && system_id < 255) {
        mav_sysid = system_id;
    }

    if (component_id > 0 && component_id < 255) {
        mav_compid = component_id;
    }

}





void FormationControl::run()
{
    sleep(4); //这是为了在启动程序之前正常启动飞控,让飞控正常进入followme模式,如果不延时有可能进不了模式
    PX4_INFO("formation_control 执行!! ");

    //    int fw_pos_ctrl_status_sub_fd = orb_subscribe(ORB_ID(fw_pos_ctrl_status));
    //提取主机数据
    //    int vehicle_attitude_setpoint_sub_fd = orb_subscribe(ORB_ID(vehicle_attitude_setpoint));

    //提取从机接收到的数据

    _att_sub                        = orb_subscribe(ORB_ID(vehicle_attitude));
    //    _follow_target_sub               = orb_subscribe(ORB_ID(follow_target));


    //    _set_offboard_sub               = orb_subscribe(ORB_ID(vehicle_command));
    _vehicle_status_sub             = orb_subscribe(ORB_ID(vehicle_status));
    _vehicle_global_position_sub    = orb_subscribe(ORB_ID(vehicle_global_position));
    _vehicle_gps_position_sub    = orb_subscribe(ORB_ID(vehicle_gps_position));

    _follow_target_sub              = orb_subscribe(ORB_ID(follow_target));
    _manual_control_setpoint_sub    = orb_subscribe(ORB_ID(manual_control_setpoint));
    _params_sub                     = orb_subscribe(ORB_ID(parameter_update));


    status_poll();  //在飞机循环之前,预先获得飞机状态
    follow_target_sp_poll();

    /* 1.公告attitude主题 */
    //    struct vehicle_attitude_setpoint_s att;
    //    memset(&att, 0, sizeof(att));

    /* 可以在此等待多个主题 */

    /* wakeup source */
    px4_pollfd_struct_t fds[1];
//    px4_pollfd_struct_t fds2[1];

    /* Setup of loop */
    fds[0].fd = _vehicle_global_position_sub;
    fds[0].events = POLLIN;

//    fds2[0].fd = _follow_target_sub;
//    fds2[0].events = POLLIN;

    int error_counter = 0;

    orb_set_interval(fds[0].fd, 50);//限制更新频率为20 Hz
    //    orb_set_interval(fds2[0].fd, 50);//限制更新频率为20 Hz



//    PX4_INFO("formation_control should_exit?  %d",should_exit());




    while (!should_exit()) {

        status_poll();  //获取飞机状态

        //根据飞机ID判断当前飞机的属性和位置,然后执行对应操作
        //ID 1 是主机,执行位置/导航姿态/速度/速度设置点发送等
        //ID 其他值是从机,从机接收主机的数据,根据编队队形计算相应的控制指令,然后进入offboard模式执行

        if((sys_id != mainplaneID)){//如果是从机
             status_poll();  //获取飞行模式和飞机ID
             if(nav_status == vehicle_status_s::NAVIGATION_STATE_AUTO_FOLLOW_TARGET) //切到了follow_target模式
             {  
                 enable_follow_target_mode(pub);//切换从机进入follow主机模式
             }
             else{
                 pub=true;
             }
            
            enable_follow_target_mode(check_aux1_enable_follow()); //这一段预留:使从机进入follow_target模式  //试验时,程序和遥控器共同实现

        }
        //待办:注意,这部分仅处理编队命令

        int poll_sp = px4_poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), 100); /* 等待???ms获取数据 */

        static bool debug_enable = true;

        /* 处理结果*/
        if (poll_sp == 0) {  //未得到数据
            //                PX4_ERR("主机:在0.1秒内没有获得uorb vehicle_global_position 数据! ");
            if(status_valid){
                mavlink_and_console_log_info(&_mavlink_log_pub, "#%d号无定位",sys_id);
                debug_enable = true;
            }
            sleep(5);
            continue;
        }
        if (poll_sp < 0) {  //严重错误
            if (error_counter < 10 || error_counter % 50 == 0) {
                /* use a counter to prevent flooding (and slowing us down) */
                PX4_ERR("ERROR return value from poll(): %d", poll_sp);
            }
            error_counter++;
            continue;
        }

        if ((fds[0].revents & POLLIN) != 0) {

            if(debug_enable){
                mavlink_and_console_log_info(&_mavlink_log_pub, "#%d号有定位",sys_id);
                debug_enable = false;
            }




            if (sys_id == mainplaneID) { //说明当前飞机是主机,执行主机相应操作


                //                struct vehicle_global_position_s P1_global_pos;
                //                orb_copy(ORB_ID(vehicle_global_position), _vehicle_global_position_sub, &P1_global_pos);

                //                vehicle_gps_position_poll();


                //待办:在这里增加编队队形控制的命令语句

                if(check_aux3_enable_follow()){
                    P1_send.formshape_id = P1_send_target.FORMSHAPE_RHOMBUS4;
                    //                 mavlink_log_info(&_mavlink_log_pub,"#主机菱形4机编队");
                }else{
                    P1_send.formshape_id = P1_send_target.FORMSHAPE_VERTIAL1;
                    //                    mavlink_log_info(&_mavlink_log_pub,"#主机竖直1字编队");
                }

                formationx_sp_publish();  //将主机数据推送到主机的uorb总线上,然后由mavlink发送出去
                usleep(10000);//每一句都要延时,防止不能上传航线

            } else if((sys_id <= 4 && sys_id >= 1)){ //说明当前飞机是从机,并且遥控器上的编队开关启用了,此时要根据从机编号做相应操作

                usleep(10000);//每一句都要延时,防止不能上传航线

            } else {
                if(status_valid) { //当程序poll到的status数据有效时,才根据这个数据执行程序,避免初始值对程序造成不必要影响
                    PX4_ERR("sys_id = %d 从机status.system_id编号错误,不能继续执行,程序目前最多支持4机编队,请修改status.system_id为1~4之间的值!",sys_id);
                }
                usleep(10000);//每一句都要延时,防止不能上传航线
            }
        }




    }
    PX4_INFO("exiting");
}




FormationControl *FormationControl::instantiate(int argc, char *argv[])
{
        return new FormationControl();
}

int FormationControl::task_spawn(int argc, char *argv[])
{
        _task_id = px4_task_spawn_cmd("formation_control",
				      SCHED_DEFAULT,
                      SCHED_PRIORITY_DEFAULT,
				      1500,
				      (px4_main_t)&run_trampoline,
				      (char *const *)argv);

	if (_task_id < 0) {
		_task_id = -1;
		return -errno;
	}

	return 0;
}

int FormationControl::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int FormationControl::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
formation_control is the ........

)DESCR_STR");

	PRINT_MODULE_USAGE_COMMAND("start");

        PRINT_MODULE_USAGE_NAME("formation_control", "controller");

	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int FormationControl::print_status()
{
	PX4_INFO("Running");

	// perf?

	return 0;
}

int formation_control_main(int argc, char *argv[])
{
        return FormationControl::main(argc, argv);
}
