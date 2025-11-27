#ifndef __PYRO_HYBRID_CHASSIS_H__
#define __PYRO_HYBRID_CHASSIS_H__

#include "pyro_algo_pid.h"
#include "pyro_chassis_base.h"
#include "pyro_kin_hybrid.h"
#include "pyro_motor_base.h"

namespace pyro
{

/**
 * @brief Hybrid Chassis (Tracks + Mecanum + Active Legs) Implementation
 */
class hybrid_chassis_t final : public chassis_base_t
{
  public:
    /**
     * @brief Extended command structure for hybrid chassis
     */
    struct cmd_hybrid_t final : cmd_base_t
    {
        // Use the mode enum defined in the kinematics class
        hybrid_kin_t::drive_mode_t drive_mode;
        
        // Target Pitch angle for active balancing (degrees), default 0.0 (horizontal)
        float target_pitch_deg; 

        cmd_hybrid_t() 
            : cmd_base_t(type_t::WHEEL_LEG), // Using WHEEL_LEG as closest type
              drive_mode(hybrid_kin_t::drive_mode_t::CRUISING),
              target_pitch_deg(0.0f)
        {
        }
    };

    hybrid_chassis_t();
    ~hybrid_chassis_t() override;

    // --- Interface Implementation ---
    void init() override;
    void set_command(const cmd_base_t &cmd) override;
    void update_feedback() override;
    void kinematics_solve() override;
    void chassis_control() override;
    void power_control() override;
    void send_motor_command() override;

    /**
     * @brief Inject IMU data (Must be called externally before thread update)
     * @param pitch_deg Current chassis pitch angle in degrees
     */
    void update_imu_data(float pitch_deg);

  private:
    // --- Commands & State ---
    cmd_hybrid_t _cmd_hybrid;
    hybrid_kin_t *_kinematics{};
    float _current_pitch_deg = 0.0f; // From IMU

    // --- Motor Outputs (Targets) ---
    hybrid_kin_t::hybrid_speeds_t _target_speeds_mps{}; // Linear speeds (m/s)
    
    // Final output to motors (Torque/Current)
    float _mecanum_output[4]{}; // FL, FR, BL, BR
    float _track_output[2]{};   // Left, Right
    float _leg_output[2]{};     // Left, Right (Torque or Position speed)

    // --- Hardware Handles ---
    // Mecanum Wheels: FL, FR, BL, BR
    motor_base_t *_mecanum_motor[4]{}; 
    // Tracks: Left, Right
    motor_base_t *_track_motor[2]{};   
    // Rear Legs Joints: Left, Right
    motor_base_t *_leg_motor[2]{};     

    // --- Controllers ---
    pid_t *_mecanum_speed_pid[4]{}; // Speed -> Torque
    pid_t *_track_speed_pid[2]{};   // Speed -> Torque
    
    // Leg Control
    pid_t *_leg_position_pid[2]{};  // Position -> Speed/Torque
    pid_t *_balance_pid{};          // Pitch Error -> Leg Position Offset

    // --- Constants & Config ---
    // You need to measure and fill these based on your CAD/Physical robot
    static constexpr float MEC_WHEEL_RADIUS_M = 0.076f; // e.g., 76mm wheel
    static constexpr float TRACK_WHEEL_RADIUS_M = 0.05f; // Track sprocket radius
    static constexpr float LEG_RETRACT_POS = 0.0f;   // Encoder value for retracted
    static constexpr float LEG_EXTEND_POS = 3.14f;   // Encoder value for extended
    
    // Helper to convert m/s to motor RPM/Rad/s
    static float mps_to_rpm(float mps, float radius) ;
};

} // namespace pyro

#endif