package frc.robot.constants;

import java.util.List;
import java.util.Optional;

import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.firecontrol.ShotLUT;
import frc.firecontrol.ShotParameters;
import frc.lib.team9410.configs.CancoderConfig;
import frc.lib.team9410.configs.LeadMotorConfig;
import frc.lib.team9410.configs.MotionMagicConfig;
import frc.lib.team9410.configs.MotorConfig;
import frc.lib.team9410.configs.PositionSubsystemConfig;
import frc.lib.team9410.math.LinearInterpolator;

public class TurretConstants {
    public static final int MOTOR_ID = 60;
    public static final int ENCODER_ID = 61;

    public static final double TURRET_KP = 57;
    public static final double TURRET_KI = 0;
    public static final double TURRET_KD = 0;
    public static final double TURRET_KG = 0;

    public static final double TURRET_MIN = -0.481;
    public static final double TURRET_MAX = 0.46;
    public static final double TURRET_DEFAULT = 0.0;

    public static final double TURRET_SENSOR_TO_MECHANISM_RATIO = -1;
    public static final double TURRET_ROTOR_TO_SENSOR_RATIO = 51 * (8.5 / 9);

    public static final double TURRET_MM_CRUISE_VELOCITY = 3; // 5
    public static final double TURRET_MM_ACCELERATION = 60; // 60
    public static final double TURRET_MAGNET_OFFSET_ROTATIONS = 0.1255;
    public static final double TURRET_DISCONTINUITY_POINT_ROTATIONS = 0.5;

    public static final List<MotorConfig> TURRET_MOTOR_CONFIGS = List.of(
            MotorConfig.leader(MOTOR_ID, NeutralModeValue.Brake, true));

    public static final LeadMotorConfig TURRET_LEAD_CONFIG = new LeadMotorConfig(
            TURRET_KP, TURRET_KI, TURRET_KD, TURRET_KG, Optional.empty(), Optional.empty(), Optional.empty(),
            TURRET_SENSOR_TO_MECHANISM_RATIO, TURRET_ROTOR_TO_SENSOR_RATIO);

    public static final CancoderConfig TURRET_CANCODER_CONFIG = new CancoderConfig(
            ENCODER_ID, TURRET_MAGNET_OFFSET_ROTATIONS, TURRET_DISCONTINUITY_POINT_ROTATIONS);

    public static final MotionMagicConfig TURRET_MOTION_MAGIC_CONFIG = new MotionMagicConfig(
            TURRET_MM_CRUISE_VELOCITY, TURRET_MM_ACCELERATION);

    public static final PositionSubsystemConfig TURRET_CONFIG = new PositionSubsystemConfig(
            TURRET_MOTOR_CONFIGS, TURRET_LEAD_CONFIG, TURRET_CANCODER_CONFIG, TURRET_MOTION_MAGIC_CONFIG,
            "Turret", "degrees", Optional.of(TURRET_DEFAULT));



        public static final double[][] SHOOTER_SPEEDS = {
                { 2.5, 41.6992 },
                { 3.0, 44.6289 },
                { 3.5, 47.2656 },
                { 4.1, 50.4883 },
                { 4.9, 54.5898 },
        };

        public static final double[][] HOOD_ANGLES = {
                { 2.5, 0.14667 },  // 52.8 deg
                { 3.0, 0.13833 },  // 49.8 deg
                { 3.5, 0.13833 },  // 49.8 deg
                { 4.1, 0.13278 },  // 47.8 deg
                { 4.9, 0.13000 },  // 46.8 deg
        };

        public static final double[][] SHOOTER_TOF = {
                { 2.5, 0.717 },
                { 3.0, 0.758 },
                { 3.5, 0.841 },
                { 4.1, 0.892 },
                { 4.9, 0.979 },
        };

//     public static final double[][] HOOD_ANGLES = {
//             { 4.9, 0.09 },
//             { 4.1, 0.08 },
//             { 3.5, 0.07 },
//             { 3.0, 0.065 },
//             { 2.5, 0.055 }
//     };

//     public static final double[][] SHOOTER_SPEEDS = {
//              { 4.9, 37 },
//              { 4.1, 34 },
//              { 3.5, 32 },
//              { 3.0, 29.5 }, // 30
//              { 2.5, 28.5 } // 29
//     };

//     public static final double[][] FEEDER_SPEEDS = {
//         { 4.9, 80 },
//         { 4.1, 85 },
//         { 3.5, 85 },
//         { 3.0, 85 },
//         { 2.5, 85 },
//     };

    public static final double TURRET_DIST_FROM_ROBOT_CENTER = -0.15875;
    public static final double TURRET_RADIUS = 0.11596; //.127
    public static final double TURRET_LIMELIGHT_STATIC_OFFSET = TURRET_DIST_FROM_ROBOT_CENTER + TURRET_RADIUS;

    /** Max turret angle error (degrees) from target before allowing shoot (must be within this to fire). */
    public static final double TURRET_SHOOT_ANGLE_TOLERANCE_DEG = 2.0;

    /** Max drivetrain linear speed (m/s) to allow shooting; above this we brake and do not shoot. */
    public static final double SHOOT_MAX_DRIVETRAIN_SPEED_MPS = 0.5;

    public static final LinearInterpolator HOOD_ANGLE_INTERPOLATOR = new LinearInterpolator(HOOD_ANGLES);
    public static final LinearInterpolator SHOOTER_VELOCITY_INTERPOLATOR = new LinearInterpolator(SHOOTER_SPEEDS);

    public static final double TURRET_CAMERA_Y_OFFSET = 0.0;


    /*
     * Combined lookup table (distance → rps, hood angle, TOF) for ShotCalculator.
     * Merges SHOOTER_SPEEDS, HOOD_ANGLES, and SHOOTER_TOF at each known distance.
     * Hood angle is in degrees (converted from the rotations scale used by the hood motor:
     * rotations * 360). The ShotCalculator will query getHoodAngle() in degrees and the
     * caller converts back to rotations before commanding the motor.
     */
    public static final ShotLUT SHOT_LUT;
    static {
        SHOT_LUT = new ShotLUT();
        for (int i = 0; i < SHOOTER_SPEEDS.length; i++) {
            double dist    = SHOOTER_SPEEDS[i][0];
            double rps     = SHOOTER_SPEEDS[i][1];
            // Lookup matching hood angle (by distance key, same order)
            double hoodRot = HOOD_ANGLES[i][1]; // rotations
            double angleDeg = hoodRot * 360.0;  // convert to degrees for ShotParameters
            double tof     = SHOOTER_TOF[i][1];
            SHOT_LUT.put(dist, new ShotParameters(rps, angleDeg, tof));
        }
    }
}
