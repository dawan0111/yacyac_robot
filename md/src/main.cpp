#include <geometry_msgs/msg/twist.hpp>
#include <yacyac_interface/msg/pose.hpp>

#include "com.cpp"
#include "md/global.hpp"

#include <rclcpp/rclcpp.hpp>

yacyac_interface::msg::Pose robot_pose;
rclcpp::Publisher<yacyac_interface::msg::Pose>::SharedPtr robot_pose_pub;

ROBOT_PARAMETER_t robotParamData;

SETTINNG_PARAM_STEP_t byCntInitStep;
volatile uint16_t appTick;
volatile uint16_t req_pid_pnd_main_data_count;
uint16_t byCntComStep;
uint32_t velCmdUpdateCount;
uint32_t velCmdRcvCount;

static uint8_t byCntCmdVel;
static uint8_t fgSendCmdVel;

static uint8_t byCntCase[10];
static uint8_t byFglIO;
INIT_SETTING_STATE_t fgInitsetting;

double goal_cmd_speed;     // m/sec
double goal_cmd_ang_speed; // radian/sec
bool reset_odom_flag;
bool reset_alarm_flag;

// extern PID_PNT_MAIN_DATA_t curr_pid_pnt_main_data;
// extern PID_ROBOT_MONITOR2_t curr_pid_robot_monitor2;
// extern PID_PNT_IO_MONITOR_t curr_pid_pnt_io_monitor;
// extern PID_ROBOT_MONITOR_t curr_pid_robot_monitor;

// extern int InitSerialComm(void);
// extern int16_t * RobotSpeedToRPMSpeed(double linear, double angular);
// extern void ResetOdom(void);

using namespace std::chrono_literals;

void AppTickTimerCallback()
{
    appTick++;
}

void VelCmdRcvTimeoutCallback()
{
    static uint32_t old_velCmdRcvCount;

    if (velCmdRcvCount == velCmdRcvCount) {
        goal_cmd_speed = 0;
        goal_cmd_ang_speed = 0;
    }

    old_velCmdRcvCount = velCmdRcvCount;
}

void Req_PID_PNT_MAIN_DATA_Callback()
{
    req_pid_pnd_main_data_count++;
}

void InitMotorParameter(void)
{
    switch (byCntInitStep) {
    case SETTING_PARAM_STEP_PID_PNT_VEL_CMD: {
        PID_PNT_VEL_CMD_t cmd_data, *p;

        p = &cmd_data;
        p->enable_id1 = 1;
        p->rpm_id1 = 0;
        p->enable_id2 = 1;
        p->rpm_id2 = 0;
        p->req_monitor_id = REQUEST_PNT_MAIN_DATA;
        PutMdData(PID_PNT_VEL_CMD, robotParamData.nRMID, (const uint8_t*)p, sizeof(cmd_data)); // 207

        byCntInitStep = SETTING_PARAM_STEP_PID_ROBOT_PARAM;
        break;
    }

    case SETTING_PARAM_STEP_PID_ROBOT_PARAM: {
        if (robotParamData.use_MDUI == 1) { // If using MDUI
            if (robotParamData.nRMID == robotParamData.nIDMDUI) {
                PID_ROBOT_PARAM_t cmd_data, *p;

                p = &cmd_data;
                p->nDiameter = (uint16_t)robotParamData.nDiameter;
                p->nWheelLength = (uint16_t)robotParamData.nWheelLength * 1000; // m unit --> mm unit
                p->nGearRatio = (uint16_t)robotParamData.nGearRatio;
                PutMdData(PID_ROBOT_PARAM, MID_MDUI, (const uint8_t*)p, sizeof(cmd_data)); // 247

                byCntInitStep = SETTING_PARAM_STEP_PID_POSI_RESET;
            }
            else {
                RCLCPP_INFO(rclcpp::get_logger("SETTING_PARAM_STEP_PID_ROBOT_PARAM"), "err.mismatch ID(RMID(%d), MDUI(%d))", robotParamData.nRMID, robotParamData.nIDMDUI);
                fgInitsetting = INIT_SETTING_STATE_ERROR;
            }
        }
        else {
            byCntInitStep = SETTING_PARAM_STEP_PID_POSI_RESET;
        }
        break;
    }

    case SETTING_PARAM_STEP_PID_POSI_RESET: {
        PutMdData(PID_POSI_RESET, robotParamData.nRMID, NULL, 0);

        ResetOdom();

        byCntInitStep = SETTING_PARAM_STEP_PID_SLOW_START;
        break;
    }

    case SETTING_PARAM_STEP_PID_SLOW_START: {
        PID_SLOW_START_t cmd_data, *p;

        p = &cmd_data;
        p->value = robotParamData.nSlowstart;

        PutMdData(PID_SLOW_START, robotParamData.nRMID, (const uint8_t*)p, sizeof(cmd_data));

        byCntInitStep = SETTING_PARAM_STEP_PID_SLOW_DOWN;
        break;
    }

    case SETTING_PARAM_STEP_PID_SLOW_DOWN: {
        PID_SLOW_DOWN_t cmd_data, *p;

        p = &cmd_data;
        p->value = robotParamData.nSlowdown;

        PutMdData(PID_SLOW_DOWN, robotParamData.nRMID, (const uint8_t*)p, sizeof(cmd_data));

        byCntInitStep = SETTING_PARAM_STEP_PID_GAIN;
        break;
    }

    case SETTING_PARAM_STEP_PID_GAIN: {
        PID_GAIN_t cmd_data, *p;

        p = &cmd_data;
        p->position_proportion_gain = robotParamData.position_proportion_gain;
        p->speed_proportion_gain = robotParamData.speed_proportion_gain;
        p->integral_gain = robotParamData.integral_gain;

        PutMdData(PID_GAIN, robotParamData.nRMID, (const uint8_t*)p, sizeof(cmd_data));

        byCntInitStep = SETTING_PARAM_STEP_PID_INV_SIGH_CMD;
        break;
    }

    case SETTING_PARAM_STEP_PID_INV_SIGH_CMD: // Left motor
    {
        uint8_t cmd_data;

#if 1
        RCLCPP_INFO(rclcpp::get_logger("SETTING_PARAM_STEP_PID_INV_SIGH_CMD"), "PID_INV_SIGN_CMD(%d)", PID_INV_SIGN_CMD);
#endif

        if (robotParamData.reverse_direction == 0) {
            cmd_data = 1;
        }
        else {
            cmd_data = 0;
        }

        PutMdData(PID_INV_SIGN_CMD, robotParamData.nRMID, (const uint8_t*)&cmd_data, 1);

        byCntInitStep = SETTING_PARAM_STEP_PID_INV_SIGH_CMD2;
        break;
    }

    case SETTING_PARAM_STEP_PID_INV_SIGH_CMD2: // Right motor
    {
        uint8_t cmd_data;

#if 1
        RCLCPP_INFO(rclcpp::get_logger("SETTING_PARAM_STEP_PID_INV_SIGH_CMD2"), "PID_INV_SIGN_CMD2(%d)", PID_INV_SIGN_CMD2);
#endif

        if (robotParamData.reverse_direction == 0) {
            cmd_data = 0;
        }
        else {
            cmd_data = 1;
        }

        PutMdData(PID_INV_SIGN_CMD2, robotParamData.nRMID, (const uint8_t*)&cmd_data, 1);

        byCntInitStep = SETTING_PARAM_STEP_PID_USE_EPOSI;
        break;
    }

    case SETTING_PARAM_STEP_PID_USE_EPOSI: {
        uint8_t cmd_data;

#if 1
        RCLCPP_INFO(rclcpp::get_logger("SETTING_PARAM_STEP_PID_USE_EPOSI"), "PID_USE_POSI(%d)", PID_USE_POSI);
#endif

        if (robotParamData.motor_position_type == 0) {
            cmd_data = 0; // hall sensor
        }
        else {
            cmd_data = 1; // encoder
        }

        PutMdData(PID_USE_POSI, robotParamData.nRMID, (const uint8_t*)&cmd_data, 1);

        byCntInitStep = SETTING_PARAM_STEP_PID_PPR;
        break;
    }

    case SETTING_PARAM_STEP_PID_PPR: {
        PID_PPR_t cmd_data, *p;

#if 1
        RCLCPP_INFO(rclcpp::get_logger("SETTING_PARAM_STEP_PID_PPR"), "PID_PPR(%d)", PID_PPR);
#endif
        p = &cmd_data;

        p->PPR = robotParamData.encoder_PPR;

        PutMdData(PID_PPR, robotParamData.nRMID, (const uint8_t*)&cmd_data, sizeof(PID_PPR_t));

        byCntInitStep = SETTING_PARAM_STEP_DONE;

        fgInitsetting = INIT_SETTING_STATE_OK;
        break;
    }

    default: break;
    }
}

void RequestRobotStatusTask(void)
{
    int nArray[5];
    uint8_t req_pid;

    switch (byCntComStep) {
    case 0: {
        req_pid = PID_PNT_MAIN_DATA; // PID 210
        PutMdData(PID_REQ_PID_DATA, robotParamData.nRMID, (const uint8_t*)&req_pid, 1);

        if (robotParamData.use_MDUI == 1) { // If using MDUI
            byCntComStep = 1;
        }
        else {
            byCntComStep = 3;
        }
        break;
    }

    case 1: {
        if (robotParamData.use_MDUI == 1) { // If using MDUI
            req_pid = PID_ROBOT_MONITOR2;   // PID 224, Only MDUI
            PutMdData(PID_REQ_PID_DATA, MID_MDUI, (const uint8_t*)&req_pid, 1);
        }

        byCntComStep = 3;
        break;
    }

#if 0
        case 2:
        {
            if(robotParamData.use_MDUI == 1) {  // If using MDUI
                req_pid = PID_PNT_IO_MONITOR;               // PID 241, Only MDUI, but not using
                PutMdData(PID_REQ_PID_DATA, MID_MDUI, (const uint8_t *)&req_pid, 1);
            }

            byCntComStep = 3;
            break;
        }
#endif

    case 3: {
        if (curr_pid_robot_monitor2.byPlatStatus.bits.bEmerSW == 1) {
            PID_PNT_TQ_OFF_t pid_pnt_tq_off, *p;

            fgSendCmdVel = 0;
            pid_pnt_tq_off.enable_id1 = 1;
            pid_pnt_tq_off.enable_id2 = 1;
            pid_pnt_tq_off.req_monitor_id = REQUEST_PNT_MAIN_DATA;
            PutMdData(PID_PNT_TQ_OFF, robotParamData.nRMID, (const uint8_t*)&pid_pnt_tq_off, sizeof(pid_pnt_tq_off));
        }
        else if (reset_odom_flag == true) {
            reset_odom_flag = false;

            PutMdData(PID_POSI_RESET, robotParamData.nRMID, NULL, 0);
            ResetOdom();
        }
        else if (reset_alarm_flag == true) {
            uint8_t cmd_pid;

            reset_alarm_flag = false;

            cmd_pid = CMD_ALARM_RESET;
            PutMdData(PID_COMMAND, robotParamData.nRMID, (const uint8_t*)&cmd_pid, 1);
        }

        byCntComStep = 0;
        break;
    }

    case 4:
        req_pid = PID_GAIN; // PID: 203
        PutMdData(PID_REQ_PID_DATA, robotParamData.nIDMDT, (const uint8_t*)&req_pid, 1);

        byCntComStep = 0;
        break;

    default: byCntComStep = 0; break;
    }
}

void cmdVelCallBack(const geometry_msgs::msg::Twist::SharedPtr keyVel) // from turtlebot3_teleop_key node
{
    static geometry_msgs::msg::Twist old_cmd;

    if (fgInitsetting == INIT_SETTING_STATE_OK) {
        velCmdUpdateCount++;

        goal_cmd_speed = keyVel->linear.x;
        goal_cmd_ang_speed = keyVel->angular.z;
    }
    return;
}

void resetOdomCallBack(const std_msgs::msg::Bool::SharedPtr reset_odom_msg)
{
    if (reset_odom_msg->data == 1) {
        RCLCPP_INFO(rclcpp::get_logger("resetOdomCallBack"), "Reset Odom");
        reset_odom_flag = true;
    }
}

void resetAlarmCallBack(const std_msgs::msg::Bool::SharedPtr reset_alarm_msg)
{
    if (reset_alarm_msg->data == 1) {
        RCLCPP_INFO(rclcpp::get_logger("resetAlarmCallBack"), "Reset Alarm");
        reset_alarm_flag = true;
    }
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("md");
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));
    auto keyboard_sub = node->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", qos_profile, cmdVelCallBack);
    auto reset_odom_sub = node->create_subscription<std_msgs::msg::Bool>("reset_odom", qos_profile, resetOdomCallBack);
    auto reset_alarm_sub = node->create_subscription<std_msgs::msg::Bool>("reset_alarm", qos_profile, resetAlarmCallBack);

    robot_pose_pub = node->create_publisher<yacyac_interface::msg::Pose>("robot_pose", qos_profile);

    int16_t* pGoalRPMSpeed;

    reset_odom_flag = false;
    reset_alarm_flag = false;

    fgSendCmdVel = 1;
    fgInitsetting = INIT_SETTING_STATE_NONE;

    node->declare_parameter("use_MDUI", 0);
    node->declare_parameter("wheel_radius", 0.065);
    node->declare_parameter("wheel_length", 0.454);
    node->declare_parameter("motor_pole", 20);
    node->declare_parameter("reduction", 1);
    node->declare_parameter("reverse_direction", 0);
    node->declare_parameter("maxrpm", 400);
    node->declare_parameter("motor_posi", 0);
    node->declare_parameter("encoder_PPR", 4096);
    node->declare_parameter("position_proportion_gain", 20);
    node->declare_parameter("speed_proportion_gain", 50);
    node->declare_parameter("integral_gain", 1800);
    node->declare_parameter("slow_start", 300);
    node->declare_parameter("slow_down", 300);

    robotParamData.use_MDUI = node->get_parameter("use_MDUI").get_value<int>();
    robotParamData.wheel_radius = node->get_parameter("wheel_radius").get_value<float>();
    robotParamData.nWheelLength = node->get_parameter("wheel_length").get_value<float>();
    robotParamData.nGearRatio = node->get_parameter("reduction").get_value<int>();
    robotParamData.motor_pole = node->get_parameter("motor_pole").get_value<int>();
    robotParamData.reverse_direction = node->get_parameter("reverse_direction").get_value<int>();
    robotParamData.nMaxRPM = node->get_parameter("maxrpm").get_value<int>();
    robotParamData.motor_position_type = node->get_parameter("motor_posi").get_value<int>();
    robotParamData.encoder_PPR = node->get_parameter("encoder_PPR").get_value<int>();
    robotParamData.position_proportion_gain = node->get_parameter("position_proportion_gain").get_value<int>();
    robotParamData.speed_proportion_gain = node->get_parameter("speed_proportion_gain").get_value<int>();
    robotParamData.integral_gain = node->get_parameter("integral_gain").get_value<int>();
    robotParamData.nSlowstart = node->get_parameter("slow_start").get_value<int>();
    robotParamData.nSlowdown = node->get_parameter("slow_down").get_value<int>();
    robotParamData.nBaudrate = 19200; // fixed

    robotParamData.nIDPC = MID_PC;     // Platform mini-PC ID
    robotParamData.nIDMDUI = MID_MDUI; // MDUI ID
    robotParamData.nIDMDT = MID_MDT;   // MD750T, MD400T, MD200T ID

    if (robotParamData.use_MDUI == 1) { // If using MDUI
        robotParamData.nRMID = robotParamData.nIDMDUI;
    }
    else {
        robotParamData.nRMID = robotParamData.nIDMDT;
    }

    RCLCPP_INFO(node->get_logger(), "PC ID          : %d", robotParamData.nIDPC);
    RCLCPP_INFO(node->get_logger(), "MDUI ID        : %d", robotParamData.nIDMDUI);
    RCLCPP_INFO(node->get_logger(), "MDT ID         : %d", robotParamData.nIDMDT);
    RCLCPP_INFO(node->get_logger(), "Receving ID    : %d", robotParamData.nRMID);
    RCLCPP_INFO(node->get_logger(), "baudrate       : %d", robotParamData.nBaudrate);
    RCLCPP_INFO(node->get_logger(), "Wheel Radius(m): %f", robotParamData.wheel_radius);
    RCLCPP_INFO(node->get_logger(), "WheelLength(m) : %f", robotParamData.nWheelLength);
    RCLCPP_INFO(node->get_logger(), "Reduction rate : %d", robotParamData.nGearRatio);
    RCLCPP_INFO(node->get_logger(), "Motor pole     : %d", robotParamData.motor_pole);

    if (robotParamData.motor_position_type == 0) {
        RCLCPP_INFO(node->get_logger(), "motor position detection: hall sensor");
    }
    else {
        RCLCPP_INFO(node->get_logger(), "motor position detection: encoder");
    }

    RCLCPP_INFO(node->get_logger(), "PPR: %d", robotParamData.encoder_PPR);

    if (robotParamData.motor_position_type == 0) {
        RCLCPP_INFO(node->get_logger(), "Robot direction: Forward");
    }
    else {
        RCLCPP_INFO(node->get_logger(), "Robot direction: Reverse");
    }
    RCLCPP_INFO(node->get_logger(), "Max RPM        : %d", robotParamData.nMaxRPM);
    RCLCPP_INFO(node->get_logger(), "Position proportion gain: %d", robotParamData.position_proportion_gain);
    RCLCPP_INFO(node->get_logger(), "Speed proportion gain   : %d", robotParamData.speed_proportion_gain);
    RCLCPP_INFO(node->get_logger(), "Integral gain  : %d", robotParamData.integral_gain);
    RCLCPP_INFO(node->get_logger(), "Slow start     : %d", robotParamData.nSlowstart);
    RCLCPP_INFO(node->get_logger(), "Slow down      : %d\r\n", robotParamData.nSlowdown);

    if (robotParamData.motor_position_type == 0) {
        robotParamData.motor_count = robotParamData.motor_pole * 3 * robotParamData.nGearRatio;
    }
    else {
        robotParamData.motor_count = robotParamData.encoder_PPR * 4 * robotParamData.nGearRatio;
    }

    robotParamData.motor_count_per_degree = (double)(360.0 / (double)robotParamData.motor_count);
    RCLCPP_INFO(node->get_logger(), "count per degree: %f", robotParamData.motor_count_per_degree);

    robotParamData.nDiameter = (int)(robotParamData.wheel_radius * 2.0 * 1000.0); // nDiameter is (mm) unit

    InitSerialComm(); // communication initialization in com.cpp

    rclcpp::Rate r(1000); // Set the loop period -> 1ms.
    rclcpp::Time start_time = node->get_clock()->now();
    rclcpp::Duration start_delay(1.5);
    double start_delay_sec = node->get_clock()->now().seconds();

    start_delay_sec += start_delay.seconds();

    //---------------------------------------------------------------------------------------------------------
    // Start delay: 1.5sec
    //---------------------------------------------------------------------------------------------------------
    while (rclcpp::ok()) {
        if (node->get_clock()->now().seconds() >= start_delay_sec) {
            break;
        }

        ReceiveDataFromController(); // com.cpp

        rclcpp::spin_some(node);
        r.sleep();
    }

    appTick = 0;
    byCntInitStep = SETTING_PARAM_STEP_PID_PNT_VEL_CMD;
    auto app_tick_timer = node->create_wall_timer(std::chrono::milliseconds(500), AppTickTimerCallback);
    app_tick_timer->reset();

    while (rclcpp::ok()) {
        ReceiveDataFromController(); // com.cpp

        if (appTick > 0) {
            appTick = 0;

            InitMotorParameter(); // com.cpp

            if (fgInitsetting != INIT_SETTING_STATE_NONE) {
                break;
            }
        }

        rclcpp::spin_some(node);
        r.sleep();
    }

    app_tick_timer->cancel();

    if (fgInitsetting != INIT_SETTING_STATE_OK) {
        RCLCPP_INFO(node->get_logger(), "error.init ROBOT");
    }
    else {
        RCLCPP_INFO(node->get_logger(), "Init done");
    }
    byCntComStep = 0;
    appTick = 0;
    app_tick_timer = node->create_wall_timer(std::chrono::milliseconds(20), AppTickTimerCallback);
    app_tick_timer->reset();

    auto vel_cmd_rcv_timeout = node->create_wall_timer(std::chrono::milliseconds(1000), VelCmdRcvTimeoutCallback);
    vel_cmd_rcv_timeout->reset();

    auto req_PID_PNT_MAIN_DATA_timer = node->create_wall_timer(std::chrono::milliseconds(100), Req_PID_PNT_MAIN_DATA_Callback);

    while (rclcpp::ok()) {
        ReceiveDataFromController(); // com.cpp

        if (appTick > 0) {
            appTick = 0;

            RequestRobotStatusTask(); // com.cpp
        }

        if (velCmdUpdateCount > 0) {
            velCmdUpdateCount = 0;

            PID_PNT_VEL_CMD_t pid_pnt_vel_cmd, *p;

            pGoalRPMSpeed = RobotSpeedToRPMSpeed(goal_cmd_speed, goal_cmd_ang_speed); // robot.cpp

            p = &pid_pnt_vel_cmd;
            p->enable_id1 = 1;
            p->rpm_id1 = pGoalRPMSpeed[0];
            p->enable_id2 = 1;
            p->rpm_id2 = pGoalRPMSpeed[1];
            p->req_monitor_id = REQUEST_PNT_MAIN_DATA;

            PutMdData(PID_PNT_VEL_CMD, robotParamData.nRMID, (const uint8_t*)&pid_pnt_vel_cmd, sizeof(pid_pnt_vel_cmd));
        }

        rclcpp::spin_some(node);
        r.sleep();
    }
    return 0;
}

void PubRobotPose(void)
{
    if (robotParamData.reverse_direction == 0) {
        robot_pose.us1 = curr_pid_robot_monitor2.byUS1;
        robot_pose.us2 = curr_pid_robot_monitor2.byUS2;
        robot_pose.us3 = curr_pid_robot_monitor2.byUS3;
        robot_pose.us4 = curr_pid_robot_monitor2.byUS4;
        robot_pose.platform_state = curr_pid_robot_monitor2.byPlatStatus.val;
    }
    else {
        PLATFORM_STATE_2_t temp;

        robot_pose.us1 = curr_pid_robot_monitor2.byUS4;
        robot_pose.us2 = curr_pid_robot_monitor2.byUS3;
        robot_pose.us3 = curr_pid_robot_monitor2.byUS2;
        robot_pose.us4 = curr_pid_robot_monitor2.byUS1;

        temp.val = curr_pid_robot_monitor2.byPlatStatus.val;
        temp.bits.bBumper1 = curr_pid_robot_monitor2.byPlatStatus.bits.bBumper4;
        temp.bits.bBumper2 = curr_pid_robot_monitor2.byPlatStatus.bits.bBumper3;
        temp.bits.bBumper3 = curr_pid_robot_monitor2.byPlatStatus.bits.bBumper2;
        temp.bits.bBumper4 = curr_pid_robot_monitor2.byPlatStatus.bits.bBumper1;

        robot_pose.platform_state = temp.val;
    }

    // robot_pose.battery_voltage = curr_pid_robot_monitor2.sVoltIn;
    // robot_pose.left_mot_current = curr_pid_pnt_main_data.current_id1;
    // robot_pose.right_mot_current = curr_pid_pnt_main_data.current_id2;
    // robot_pose.left_motor_ctrl_state = curr_pid_pnt_io_monitor.motor_1.val;
    // robot_pose.right_motor_ctrl_state = curr_pid_pnt_io_monitor.motor_2.val;

    robot_pose.left_motor_state = curr_pid_pnt_main_data.mtr_state_id1.val;
    robot_pose.right_motor_state = curr_pid_pnt_main_data.mtr_state_id2.val;

    robot_pose_pub->publish(robot_pose);
}