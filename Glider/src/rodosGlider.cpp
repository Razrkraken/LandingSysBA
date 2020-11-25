//
// Created by alejandro on 24.09.20.
//

#include <GazeboTopic.h>
#include <rodos.h>
#include <stdio.h>

#define TODEG 180/3.1415926
#define TORAD 3.1415926/180

static Application landingSysAPP("landingSysApplication");

/**
 * Commbuffers
 * */
static CommBuffer<gazebo::msgs::Vector3d> real_position_buff; // Only DesHeightPublisher has access
static CommBuffer<gazebo::msgs::IMU> imu_data_buff;
static CommBuffer<double> error_height_buff;
static CommBuffer<double> des_pitch_buff;
static CommBuffer<double> des_pitch_rate_buff;
static CommBuffer<double> des_elev_pos_buff;
static CommBuffer<gazebo::msgs::Vector3d> vel_buff;
static CommBuffer<gazebo::msgs::Vector3d> elevPosition_buff;

/**
 * Semaphores
 * */
Semaphore realPose_sem;
Semaphore imu_sem;
Semaphore error_height_sem;
Semaphore des_pitch_sem;
Semaphore des_pitch_rate_sem;
Semaphore des_elev_pos_sem;
Semaphore vel_sem;
Semaphore elev_sem;

/**
 * Gazebotopics (shared topics between RODOS and Gazebo)
 * */
//The topics that will be shared  between Gazebo and RODOS.
GazeboTopic<gazebo::msgs::IMU> imuTopic(10, "~/SUAV_glider/SUAV_glider/fuselage/IMU_Fuselage/imu");
GazeboTopic<gazebo::msgs::Vector3d> realStateTopic(12, "~/SUAV_glider/my_pose");
GazeboTopic<gazebo::msgs::Vector3d> velStateTopic(14, "~/SUAV_glider/my_vel");
GazeboTopic<gazebo::msgs::Vector3d> elevStateTopic(18, "~/SUAV_glider/my_elev");
GazeboTopic<gazebo::msgs::Vector2d> rudderCmdTopic(11, "~/SUAV_glider/rudder_angle");
GazeboTopic<gazebo::msgs::Vector2d> elevatorCmdTopic(13, "~/SUAV_glider/elevator_angle");
GazeboTopic<gazebo::msgs::Vector2d> leftAileronCmdTopic(15, "~/SUAV_glider/left_aileron_angle");
GazeboTopic<gazebo::msgs::Vector2d> rightAileronCmdTopic(17, "~/SUAV_glider/right_aileron_angle");

/**
 * Subscribers
 * */
static Subscriber RealStateSub(realStateTopic, real_position_buff, "RealStateSubscriber");
static Subscriber IMUSub(imuTopic, imu_data_buff, "IMUSubscriber");
static Subscriber VelSub(velStateTopic, vel_buff, "VelSubscriber");
static Subscriber ElevSub(elevStateTopic, elevPosition_buff, "ElevSubscriber");

/**
 * StaticThreads
 * */
class RudderControlThread : public StaticThread<> {
public:
    RudderControlThread() : StaticThread("RudderControlThread") {}

    [[noreturn]] void run() override {
        gazebo::msgs::Vector2d cmd;
        while (true) {
            cmd.set_x(0);
            cmd.set_y(0);
            rudderCmdTopic.publish(cmd);
            suspendCallerUntil(NOW() + SECONDS);
        }
    }
} rudderControlThread;

class ElevatorControlThread : public StaticThread<> {
public:
    ElevatorControlThread() : StaticThread("ElevatorControlThread") {}

    [[noreturn]] void run() override {
        gazebo::msgs::Vector2d cmd;
        double pos;
        while (true) {
            des_elev_pos_buff.get(pos);
            bool sign = pos >= 0;
            pos = (abs(pos) < 0.349066) ? pos : (0.349066 * (sign ? 1 : -1));  // limits command
            cmd.set_x(pos);
            cmd.set_y(0);
            elevatorCmdTopic.publish(cmd);
            suspendCallerUntil(NOW() + 0.01 * SECONDS);
        }
    }
} elevatorControlThread;

class LeftAileronControlThread : public StaticThread<> {
public:
    LeftAileronControlThread() : StaticThread("LeftAileronControlThread") {}

    [[noreturn]] void run() override {
        gazebo::msgs::Vector2d cmd;
        while (true) {
            cmd.set_x(0.5);
            cmd.set_y(0);
            leftAileronCmdTopic.publish(cmd);
            suspendCallerUntil(NOW() + SECONDS);
        }
    }
} leftAileronControlThread;

class RightAileronControlThread : public StaticThread<> {
public:
    RightAileronControlThread() : StaticThread("RightAileronControlThread") {}

    [[noreturn]] void run() override {
        gazebo::msgs::Vector2d cmd;
        while (true) {
            cmd.set_x(-0.5);
            cmd.set_y(0);
            rightAileronCmdTopic.publish(cmd);
            suspendCallerUntil(NOW() + SECONDS);
        }
    }
} rightAileronControlThread;

/**
 * Height Error Loop
 *
 * This Thread calculates and publishes the error height. The desire height has to be estimates to calculate the error.
 * The desire height depends on the distance to the threshold of the runway and the angle of the glide slope.
 *
 * Input:  Current pose (respect to world frame in X, Y and Z)
 * Output: Error Height
 *
 * */
class HeightErrorLoopThread : public StaticThread<> {
public:

    //constructor
    HeightErrorLoopThread() : StaticThread("HeightErrorLoopThread") {}

    [[noreturn]] void run() override {


        gazebo::msgs::Vector3d curr_position;

        while (true) {

            realPose_sem.enter();
            real_position_buff.get(curr_position);
            realPose_sem.leave();

            double x = curr_position.x();
            double des_height = (x < 0) ? tan(5.5 * TORAD) * -1 * x : 0;

            error_height_sem.enter();
            error_height_buff.put(des_height - curr_position.z());
            error_height_sem.leave();

            suspendCallerUntil(NOW() + 0.3 * SECONDS);
        }
    }
} HeightErrorLoopThread;

/**
 * Pitch cmd loop.
 *
 *
 *
 * Input: Error Height
 * Output: Desire Pitch
 * */
class DesEulerPitchLoopThread : public StaticThread<> {

public:
    DesEulerPitchLoopThread() : StaticThread("DesEulerPitchLoopThread") {}

    [[noreturn]] void run() override {
        double error_height;
        gazebo::msgs::Vector3d curr_pose;
        double K_heightP = 0.03; // 0.03
        double K_heightD = 5; // 5
        double K_flareP = 0.2; // 0.001
        double K_flareD = 0.8; // 5
        double des_euler_pitch;
        double des_vertical_vel = -1;
        gazebo::msgs::Vector3d position;
        gazebo::msgs::Vector3d velocity;
        double lastcmd = 0;
        double prevError = 0;
        int64_t lastTime = NOW();
        while (true) {
//          Get position
            realPose_sem.enter();
            real_position_buff.get(position);
            realPose_sem.leave();
            if (position.x() >= 0) {
                if (position.z() < 3) { // Do flare manoeuvre
                    vel_sem.enter();
                    vel_buff.get(velocity);
                    vel_sem.leave();
                    double error_vel = des_vertical_vel - velocity.z();
                    int64_t timeThisIteration = NOW();
                    double dt = (timeThisIteration/MILLISECONDS - lastTime/MILLISECONDS);
                    des_euler_pitch = (error_vel) * K_flareP + ((error_vel-prevError)/dt) * K_flareD;
                    prevError = error_vel;
                    lastTime = timeThisIteration;
                }else{
                    prevError = 0;
                    des_euler_pitch = lastcmd; //Lock attitude
                }
            } else { //Connect height control loop
                //gets error height
                error_height_sem.enter();
                error_height_buff.get(error_height);
                error_height_sem.leave();
                int64_t timeThisIteration = NOW();
                double dt = (timeThisIteration/MILLISECONDS - lastTime/MILLISECONDS);
                //calculates des euler pitch
                des_euler_pitch = (error_height) * K_heightP   +  ((error_height-prevError)/dt) * K_heightD ;
                lastcmd = des_euler_pitch;
                prevError = error_height;
                lastTime = timeThisIteration;
            }
            //publishes des. euler pitch
            des_pitch_sem.enter();
            des_pitch_buff.put(des_euler_pitch);
            des_pitch_sem.leave();
            suspendCallerUntil(NOW() + 0.05 * SECONDS);
        }
    }
} DesEulerPitchLoopThread;

/**
 * Pitch rate cmd loop.
 *
 * Input: Desire Pitch + Current Pitch
 * Output: Desire Pitch rate
 * */
class DesPitchRateLoopThread : public StaticThread<> {

public:
    DesPitchRateLoopThread() : StaticThread("DesPitchRateLoopThread") {}

    [[noreturn]] void run() override {

        double des_euler_pitch;
        gazebo::msgs::IMU imu_data;
        ignition::math::Quaterniond curr_orientiation;
        double K_euler_pitchP = 3; //3
        double K_euler_pitchD = 0.01; //0.01
        double des_pitch_rate;
        double curr_euler_pitch;

        double prevError = 0;
        int64_t lastTime = NOW();

        while (true) {
            //Gets desire Euler Pitch
            des_pitch_sem.enter();
            des_pitch_buff.get(des_euler_pitch);
            des_pitch_sem.leave();

            //Gets current Position
            //Enters Semaphore
            imu_sem.enter();
            imu_data_buff.get(imu_data);
            imu_sem.leave();
            //Leaves Semaphore

            curr_orientiation = gazebo::msgs::ConvertIgn(imu_data.orientation());
            curr_euler_pitch = -1 * curr_orientiation.Pitch();

            double des_pitch_error = des_euler_pitch - curr_euler_pitch;
            int64_t timeThisIteration = NOW();
            double dt = (timeThisIteration/MILLISECONDS - lastTime/MILLISECONDS);


            des_pitch_rate = (des_pitch_error) * K_euler_pitchP + ((des_pitch_error-prevError)/dt) * K_euler_pitchD ;

            //Publishes the desire pitch rate on buffer
            des_pitch_rate_sem.enter();
            des_pitch_rate_buff.put(des_pitch_rate);
            des_pitch_rate_sem.leave();

            prevError = des_pitch_error;
            lastTime = timeThisIteration;


            suspendCallerUntil(NOW() + 0.01 * SECONDS);
        }
    }
} DesPitchRateLoopThread;

/**
 * Elevator pos cmd loop.
 *
 * Input - Desire Pitch rate + Current Pitch rate
 * Output - Desire elevator position
 * */
class DesElevatorPosLoopThread : public StaticThread<> {
public:
    DesElevatorPosLoopThread() : StaticThread("DesElevatorPosLoopThread") {}

    [[noreturn]] void run() override {
        double des_pitch_rate;
        gazebo::msgs::IMU imu_data;
        double K_pitch_rate = 0.3; //0.3
        double des_elev_pos;
        double pitch_rate;
        gazebo::msgs::Vector3d position;

        while (true) {

            //Gets desire Pitch rate
            des_pitch_rate_sem.enter();
            des_pitch_rate_buff.get(des_pitch_rate);
            des_pitch_rate_sem.leave();

            //Gets current imu data
            imu_sem.enter();
            imu_data_buff.get(imu_data);
            imu_sem.leave();

            pitch_rate = -1 * imu_data.angular_velocity().y(); //in rad/s

            des_elev_pos = (des_pitch_rate - pitch_rate) * K_pitch_rate;

            des_elev_pos_sem.enter();
            des_elev_pos_buff.put(des_elev_pos);
            des_elev_pos_sem.leave();

            suspendCallerUntil(NOW() + 0.005 * SECONDS);
        }
    }
} DesElevatorPosLoopThread;

/**
 * Thread to log glider data
 *
 * Logged data:
 * - True Height  c
 * - Error Height  c
 * - True Pitch c
 * - Desire Pitch c
 * - True Pitch Rate c
 * - Desire Pitch Rate c
 * - True Elevator pos c
 * - Cmd Elevator pos  c
 * - velocity X c
 * - velocity Z c
 * - velocity along flight path c
 * */
class LoggerThread : public StaticThread<> {
private:

    FILE *p_logFile;
    int64_t start_time;

public:

    LoggerThread() : StaticThread("LoggerThread", 50) {}

    [[noreturn]] void run() override {

        start_time = NOW() / MILLISECONDS; // Time in milliseconds
        gazebo::msgs::Vector3d truePosition;
        double errorHeight;
        gazebo::msgs::IMU imuData;
        ignition::math::Quaterniond orientation;
        double desirePitch;
        double desirePitchRate;
        double desElevatorPos;
        gazebo::msgs::Vector3d elevPosition;
//      double trueElevatorPos;
        int64_t dt;
        gazebo::msgs::Vector3d velocity;
        ignition::math::Vector3<double> velocityIgn;

        p_logFile = fopen("logs/dataLog.csv", "w");
        fprintf(p_logFile,
                "Time,True Height,Error Height,True Pitch,Desire Pitch,True Pitch Rate,Desire Pitch Rate,True Elevator Pos,Desire Elevator Pos,Horizontal Vel,Vertical Vel,Flight Path Vel,Distance to Runway Start\n");
        fclose(p_logFile);

        while (1) {

            //open log file
            p_logFile = fopen("logs/dataLog.csv", "a");

            //Get true position
            realPose_sem.enter();
            real_position_buff.get(truePosition);
            realPose_sem.leave();

            //Get error height
            error_height_sem.enter();
            error_height_buff.get(errorHeight);
            error_height_sem.leave();

            dt = (NOW() / MILLISECONDS) - start_time;

            //Print to file: Time, True Height, Error Height.
            fprintf(p_logFile, "%"
                               PRId64
                               ",%f,%f,", dt, truePosition.z(), errorHeight);

            //Get true pitch
            imu_sem.enter();
            imu_data_buff.get(imuData);
            imu_sem.leave();
            orientation = gazebo::msgs::ConvertIgn(imuData.orientation());

            //Get desire pitch
            des_pitch_sem.enter();
            des_pitch_buff.get(desirePitch);
            des_pitch_sem.leave();

            //Print to file: True Pitch, desire Pitch.
            fprintf(p_logFile, "%f,%f,", -1 * orientation.Pitch(), desirePitch);

            //True Pitch rate in imuData

            //Get desire pitch rate
            des_pitch_rate_sem.enter();
            des_pitch_rate_buff.get(desirePitchRate);
            des_pitch_rate_sem.leave();

            //Print to file: True Pitch, desire Pitch.
            fprintf(p_logFile, "%f,%f,", -1 * imuData.angular_velocity().y(), desirePitchRate);

            //Get elevator cmd
            des_elev_pos_sem.enter();
            des_elev_pos_buff.get(desElevatorPos);
            des_elev_pos_sem.leave();

            //Get elevator state
            elev_sem.enter();
            elevPosition_buff.get(elevPosition);
            elev_sem.leave();

            //Print to file: desire Elevator pos
            fprintf(p_logFile, "%f,%f,", elevPosition.x(), desElevatorPos);

            //Get velocity vector
            vel_sem.enter();
            vel_buff.get(velocity);
            vel_sem.leave();

            velocityIgn = gazebo::msgs::ConvertIgn(velocity);

            //Print to file: Velocity x, Velocity z, flight path vel
            fprintf(p_logFile, "%f,%f,%f,", velocityIgn.X(), velocityIgn.Z(), velocityIgn.Length());


            //Print to file: Distance to Runway Start
            fprintf(p_logFile, "%f\n", truePosition.x());

            fclose(p_logFile);

            suspendCallerUntil(NOW() + 0.3 * SECONDS);
        }
    }

} LoggerThread;


