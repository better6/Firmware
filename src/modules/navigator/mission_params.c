

/****************************************************************************
 *
 *   Copyright (c) 2014-2016 PX4 Development Team. All rights reserved.
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
 * @file mission_params.c
 *
 * Parameters for mission.
 *
 * @author Julian Oes <julian@oes.ch>
 */

/*
 * Mission parameters, accessible via MAVLink
 */

/**
 * Take-off altitude
 *
 * This is the minimum altitude the system will take off to.
 *
 * @unit m
 * @min 0
 * @max 80
 * @decimal 1
 * @increment 0.5
 * @group Mission
 */
PARAM_DEFINE_FLOAT(MIS_TAKEOFF_ALT, 2.5f); //takeoff起飞的高度

/**
 * Minimum Loiter altitude
 *
 * This is the minimum altitude the system will always obey. The intent is to stay out of ground effect.
 * set to -1, if there shouldn't be a minimum loiter altitude
 *
 * @unit m
 * @min -1
 * @max 80
 * @decimal 1
 * @increment 0.5
 * @group Mission
 */
PARAM_DEFINE_FLOAT(MIS_LTRMIN_ALT, -1.0f); //最小的悬停高度，不使用

/**
 * Maximal horizontal distance from home to first waypoint
 *
 * Failsafe check to prevent running mission stored from previous flight at a new takeoff location.
 * Set a value of zero or less to disable. The mission will not be started if the current
 * waypoint is more distant than MIS_DIS_1WP from the home position.
 *
 * @unit m
 * @min 0
 * @max 10000
 * @decimal 1
 * @increment 100
 * @group Mission
 */
PARAM_DEFINE_FLOAT(MIS_DIST_1WP, 500);//航线保护 要求第一个航点距离home点的距离不能大于这个值，避免换了个测试场地 保留的还是之前的航线，那样misison会比较可怕

/**
 * Maximal horizontal distance between waypoint
 *
 * Failsafe check to prevent running missions which are way too big.
 * Set a value of zero or less to disable. The mission will not be started if any distance between
 * two subsequent waypoints is greater than MIS_DIST_WPS.
 *
 * @unit m
 * @min 0
 * @max 10000
 * @decimal 1
 * @increment 100
 * @group Mission
 */
PARAM_DEFINE_FLOAT(MIS_DIST_WPS, 1000); //航线保护功能 任意连个航点之间的最大距离，此参数限制了飞机mission的最远距离，现在给放大一些

/**
 * Altitude setpoint mode
 *
 * 0: the system will follow a zero order hold altitude setpoint
 * 1: the system will follow a first order hold altitude setpoint
 * values follow the definition in enum mission_altitude_mode
 *
 * @min 0
 * @max 1
 * @value 0 Zero Order Hold
 * @value 1 First Order Hold
 * @group Mission
 */
PARAM_DEFINE_INT32(MIS_ALTMODE, 1); //高度的一阶滤波

/**
 * Multirotor only. Yaw setpoint mode.
 *
 * The values are defined in the enum mission_altitude_mode
 *
 * @min 0
 * @max 3
 * @value 0 Heading as set by waypoint
 * @value 1 Heading towards waypoint
 * @value 2 Heading towards home
 * @value 3 Heading away from home
 * @group Mission
 */
PARAM_DEFINE_INT32(MIS_YAWMODE, 3); //  mission模式下 航向的保持方式，指向航点 还是指向home  还是背离home

/**
* Enable yaw control of the mount. (Only affects multicopters and ROI mission items)
*
* If enabled, yaw commands will be sent to the mount and the vehicle will follow its heading mode as specified by MIS_YAWMODE.
* If disabled, the vehicle will yaw towards the ROI.
*
* @value 0 Disable
* @value 1 Enable
* @min 0
* @max 1
* @group Mission
*/
PARAM_DEFINE_INT32(MIS_MNT_YAW_CTL, 0);

/**
 * Time in seconds we wait on reaching target heading at a waypoint if it is forced.
 *
 * If set > 0 it will ignore the target heading for normal waypoint acceptance. If the
 * waypoint forces the heading the timeout will matter. For example on VTOL forwards transition.
 * Mainly useful for VTOLs that have less yaw authority and might not reach target
 * yaw in wind. Disabled by default.
 *
 * @unit s
 * @min -1
 * @max 20
 * @decimal 1
 * @increment 1
 * @group Mission
 */
PARAM_DEFINE_FLOAT(MIS_YAW_TMT, -1.0f); //设置航点时可以设置航速和航向，这是设置航向达到的最大超时时间

/**
 * Max yaw error in degrees needed for waypoint heading acceptance.
 *
 * @unit deg
 * @min 0
 * @max 90
 * @decimal 1
 * @increment 1
 * @group Mission
 */
PARAM_DEFINE_FLOAT(MIS_YAW_ERR, 12.0f);//航向能够接受的最大航向误差


