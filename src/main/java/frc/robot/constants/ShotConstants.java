package frc.robot.constants;

import frc.firecontrol.ShotLUT;
import frc.firecontrol.ShotParameters;
import frc.lib.team9410.math.LinearInterpolator;

// All shooting-related constants and lookup tables.
// Distance keys are in meters; all arrays are sorted ascending by distance.
public class ShotConstants {

    // Original tables (kept for reference — descending order, mismatched TOF size)
    // public static final double[][] SHOOTER_SPEEDS = {
    //     { 5.5, 89}, { 4.85, 84}, { 4.6, 82}, { 4.05, 79}, { 3.75, 77},
    //     { 3.29, 74}, { 2.75, 71}, { 2.38, 67}, { 1.97, 64}, { 1.68, 62}
    // };
    // public static final double[][] HOOD_ANGLES = {
    //     { 5.5, 0.135 }, { 4.85, 0.105 }, { 4.6, 0.09 }, { 4.05, 0.08 }, { 3.75, 0.07 },
    //     { 3.29, 0.06 }, { 2.75, 0.05 }, { 2.38, 0.04 }, { 1.97, 0.03 }, { 1.68, 0.02 }
    // };
    // public static final double[][] SHOOTER_TOF = {
    //     { 2.5, 0.682 }, { 3.0, 0.784 }, { 3.5, 0.833 }, { 4.1, 0.889 }, { 4.9, 0.982 },
    // };
    // public static final double[][] FEEDER_SPEEDS = {
    //     { 5.5, 72}, { 4.85, 68}, { 4.6, 67}, { 4.05, 66}, { 3.75, 65},
    //     { 3.29, 65}, { 2.75, 63}, { 2.38, 61}, { 1.97, 59}, { 1.68, 58}
    // };

    // Shooter Tables — sorted ascending by distance (required for interpolators)
    // TOF at 1.68, 1.97, 2.38, 5.5 are extrapolated from real data — tune on robot
    public static final double[][] SHOOTER_SPEEDS = {
        { 1.68, 62 },
        { 1.97, 64 },
        { 2.38, 67 },
        { 2.75, 71 },
        { 3.29, 74 },
        { 3.75, 77 },
        { 4.05, 79 },
        { 4.60, 82 },
        { 4.85, 84 },
        { 5.50, 89 },
    };

    public static final double[][] HOOD_ANGLES = {
        { 1.68, 0.020 },
        { 1.97, 0.030 },
        { 2.38, 0.040 },
        { 2.75, 0.050 },
        { 3.29, 0.060 },
        { 3.75, 0.070 },
        { 4.05, 0.080 },
        { 4.60, 0.090 },
        { 4.85, 0.105 },
        { 5.50, 0.135 },
    };

    // TOF interpolated/extrapolated from real measurements at 2.5, 3.0, 3.5, 4.1, 4.9 m
    public static final double[][] SHOOTER_TOF = {
        { 1.68, 0.515 },
        { 1.97, 0.574 },
        { 2.38, 0.658 },
        { 2.75, 0.733 },
        { 3.29, 0.812 },
        { 3.75, 0.856 },
        { 4.05, 0.884 },
        { 4.60, 0.947 },
        { 4.85, 0.976 },
        { 5.50, 1.052 },
    };

    public static final double[][] FEEDER_SPEEDS = {
        { 1.68, 58 },
        { 1.97, 59 },
        { 2.38, 61 },
        { 2.75, 63 },
        { 3.29, 65 },
        { 3.75, 65 },
        { 4.05, 66 },
        { 4.60, 67 },
        { 4.85, 68 },
        { 5.50, 72 },
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
