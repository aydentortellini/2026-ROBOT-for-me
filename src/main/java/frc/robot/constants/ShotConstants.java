package frc.robot.constants;

import frc.firecontrol.ShotLUT;
import frc.firecontrol.ShotParameters;
import frc.lib.team9410.math.LinearInterpolator;

// All shooting-related constants and lookup tables.
// Distance keys are in meters; all arrays are sorted ascending by distance.
public class ShotConstants {

    // Shooter Tables
    public static final double[][] SHOOTER_SPEEDS = {
        { 2.5, 28.5 },
        { 3.0, 29.5 },
        { 3.5, 32.0 },
        { 4.1, 34.0 },
        { 4.9, 37.0 },
    };

    public static final double[][] HOOD_ANGLES = {
        { 2.5, 0.055 },
        { 3.0, 0.065 },
        { 3.5, 0.070 },
        { 4.1, 0.080 },
        { 4.9, 0.090 },
    };

    public static final double[][] SHOOTER_TOF = {
        { 2.5, 0.682 },
        { 3.0, 0.784 },
        { 3.5, 0.833 },
        { 4.1, 0.889 },
        { 4.9, 0.982 },
    };

    public static final double[][] FEEDER_SPEEDS = {
        { 2.5, 85.0 },
        { 3.0, 85.0 },
        { 3.5, 85.0 },
        { 4.1, 85.0 },
        { 4.9, 80.0 },
    };

    // Interpolators
    public static final LinearInterpolator SHOOTER_VELOCITY_INTERPOLATOR =
        new LinearInterpolator(SHOOTER_SPEEDS);

    public static final LinearInterpolator HOOD_ANGLE_INTERPOLATOR =
        new LinearInterpolator(HOOD_ANGLES);

    public static final LinearInterpolator FEEDER_VELOCITY_INTERPOLATOR =
        new LinearInterpolator(FEEDER_SPEEDS);

    // SOTM fire-control config
    // Shooter offset from robot center along X (negative = rear-mounted).
    public static final double LAUNCHER_OFFSET_X = -0.20;

    // Shooter faces the rear of the robot.
    public static final double SHOOTER_ANGLE_OFFSET_RAD = Math.PI;

    // Max robot speed (m/s) for SOTM solver; above this returns INVALID.
    public static final double MAX_SOTM_SPEED = 3.0;

    // Min/max scoring distance (m).
    public static final double MIN_SCORING_DISTANCE = 0.5;
    public static final double MAX_SCORING_DISTANCE = 5.0;

    // Robot heading must be within this many degrees of aim angle to fire.
    public static final double HEADING_TOLERANCE_DEG = 10.0;

    // Combined ShotLUT
    public static final ShotLUT SHOT_LUT;
    static {
        SHOT_LUT = new ShotLUT();
        for (int i = 0; i < SHOOTER_SPEEDS.length; i++) {
            double dist     = SHOOTER_SPEEDS[i][0];
            double rps      = SHOOTER_SPEEDS[i][1];
            double hoodDeg  = HOOD_ANGLES[i][1] * 360.0; // rotations → degrees for ShotParameters
            double tof      = SHOOTER_TOF[i][1];
            SHOT_LUT.put(dist, new ShotParameters(rps, hoodDeg, tof));
        }
    }
}
