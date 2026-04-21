package frc.robot.utils;

import static edu.wpi.first.units.Units.Rotation;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.Kinematics;
import edu.wpi.first.units.AngleUnit;
import frc.lib.team9410.PowerRobotContainer;
import frc.robot.Constants;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.TurretConstants;
import frc.robot.subsystems.StateMachine;
import frc.robot.utils.FieldUtils.GameZone;

public class TurretHelpers {
    /**
     * Gets the position of the limelight on the turret relative to center of robot.
     * 
     * @param turret_dir In degrees
     * @return
     */
    public static Pose2d turretCamPosRelative(double turretDegrees) {
        double turretRadians = Math.toRadians(turretDegrees);

        return new Pose2d(
                Math.sin(turretRadians) * Constants.Turret.TURRET_RADIUS,
                Constants.Turret.TURRET_DIST_FROM_ROBOT_CENTER
                        - Math.cos(turretRadians) * Constants.Turret.TURRET_RADIUS,
                Rotation2d.fromRadians(turretRadians));
    }

    /// DOES NOT PREDICT FOR VELO
    public static double getRotationToTargetDirect() {
        return 0.0;
    }

    public static double calculateHoodAngle(double distance) {
        double hoodAngleSetpoint = TurretConstants.HOOD_ANGLE_INTERPOLATOR.getInterpolatedValue(distance);
        return hoodAngleSetpoint;
    }

    public static double calculateShooterVelocity(double distance) {
        double shooterVelocitySetpoint = TurretConstants.SHOOTER_VELOCITY_INTERPOLATOR.getInterpolatedValue(distance);
        return shooterVelocitySetpoint;
    }

    /// DOES NOT PREDICT FOR VELO
    public static double getElevationToTarget() {
        return 0.0;
    }

    /// sets prc.data("turretTarget") as Translation3d
    public static void setTarget(Translation3d pos) {
        PowerRobotContainer.setData("turretTarget", pos);
    }

    /// does nothing as of now
    public static double predictRotationToTarget() {
        return 0.0;
    }

    public static Translation2d rotatePoint(Translation2d point, double radians) {
        double xn = point.getX() + TurretConstants.TURRET_DIST_FROM_ROBOT_CENTER * Math.cos(radians);
        double yn = point.getY() + TurretConstants.TURRET_DIST_FROM_ROBOT_CENTER * Math.sin(radians);

        // double xn = x * Math.cos(radians) - y * Math.sin(radians);
        // double yn = x * Math.sin(radians) + y * Math.cos(radians);

        return new Translation2d(xn, yn);
    }

    public static double getTurretRotationsWithoutLead(StateMachine stateMachine) {
        Translation2d hub = FieldUtils.getZone(stateMachine.drivetrain.getState().Pose) == GameZone.BLUE_ALLIANCE
                ? FieldConstants.HOPPER_BLUE
                : FieldConstants.HOPPER_RED;

        Translation2d relative = stateMachine.drivetrain.getState().Pose.getTranslation()
                .minus(hub)
                .plus(new Translation2d(0, TurretConstants.TURRET_DIST_FROM_ROBOT_CENTER).rotateBy(stateMachine.drivetrain.getState().Pose.getRotation()));

        // System.out.println("x: " + relative.getX());
        // System.out.println("Y: " + relative.getY());
        

        return Math.atan2(relative.getY(), relative.getX());
    }

    public static double getTurretRotationsWithoutLead(StateMachine stateMachine, Translation2d point) {
        // Translation2d hub = point;

        // Translation2d relative = stateMachine.drivetrain.getState().Pose.getTranslation()
        //         .minus(hub)
        //         .plus(new Translation2d(0, TurretConstants.TURRET_DIST_FROM_ROBOT_CENTER).rotateBy(stateMachine.drivetrain.getState().Pose.getRotation()));

        // // System.out.println("x: " + relative.getX());
        // // System.out.println("Y: " + relative.getY());

        // return Math.atan2(relative.getY(), relative.getX());

        // return getTurretAngleRadiansFromBack(stateMachine.drivetrain.getState().Pose, point);
        return getOffsetTurretAngleRadiansFromBack(stateMachine.drivetrain.getState().Pose, point);
    }

    public static double getRadiansToPoint (Pose2d robotPose, Translation2d target) {
        // Get the robot's current position on the field
        Translation2d robotPosition = robotPose.getTranslation();

        // Find the vector from the robot to the target
        double deltaX = target.getX() - robotPosition.getX();
        double deltaY = target.getY() - robotPosition.getY();

        // Find the angle from the robot to the target in field coordinates
        double targetAngleFieldRelative = Math.atan2(deltaY, deltaX);

        return targetAngleFieldRelative;
    }

    public static double getTurretAngleRadiansFromBack(
    Pose2d robotPose,
    Translation2d targetPoint
    ) {
        // Get the robot's current position on the field
        Translation2d robotPosition = robotPose.getTranslation();

        // Find the vector from the robot to the target
        double deltaX = targetPoint.getX() - robotPosition.getX();
        double deltaY = targetPoint.getY() - robotPosition.getY();

        // Find the angle from the robot to the target in field coordinates
        double targetAngleFieldRelative = Math.atan2(deltaY, deltaX);

        // Get the robot's heading
        double robotHeadingRadians = robotPose.getRotation().getRadians();

        // Convert the field-relative target angle into a robot-relative angle
        // In this convention:
        // 0 = front, +pi/2 = left, -pi/2 = right, pi = back
        double targetAngleRobotRelative = targetAngleFieldRelative - robotHeadingRadians;
        targetAngleRobotRelative = MathUtil.angleModulus(targetAngleRobotRelative);

        // Shift the angle so that "back" becomes zero
        double turretAngleFromBack = targetAngleRobotRelative - Math.PI;

        // Normalize again so the result stays in [-pi, pi]
        turretAngleFromBack = MathUtil.angleModulus(turretAngleFromBack);

        return turretAngleFromBack;
    }

    public static double getOffsetTurretAngleRadiansFromBack(
        Pose2d robotPose,
        Translation2d targetPoint
    ) {
        // Robot center on the field
        Translation2d robotCenter = robotPose.getTranslation();

        // Turret pivot offset in robot coordinates.
        // Positive Y means "toward the back" in your current convention.
        Translation2d turretOffsetFromRobotCenter =
            new Translation2d(0.0, TurretConstants.TURRET_DIST_FROM_ROBOT_CENTER);

        // Rotate that offset by the robot heading so it becomes field-relative
        Translation2d turretOffsetFieldRelative =
            turretOffsetFromRobotCenter.rotateBy(robotPose.getRotation());

        // Actual turret pivot position on the field
        Translation2d turretPivotPosition =
            robotCenter.plus(turretOffsetFieldRelative);

        // Vector from turret pivot to target
        Translation2d turretToTarget =
            targetPoint.minus(turretPivotPosition);

        // Field-relative angle from turret pivot to target
        double targetAngleFieldRelative =
            Math.atan2(turretToTarget.getY(), turretToTarget.getX());

        // Robot heading on the field
        double robotHeadingRadians = robotPose.getRotation().getRadians();

        // Convert to robot-relative angle
        double targetAngleRobotRelative =
            MathUtil.angleModulus(targetAngleFieldRelative - robotHeadingRadians);

        // Shift so "back of robot" becomes zero
        double turretAngleFromBack =
            MathUtil.angleModulus(targetAngleRobotRelative - Math.PI);

        return turretAngleFromBack;
    }

    public static double getDistance(Pose2d RobotPosition, Translation2d hopperPosition) {
        Translation2d difference = hopperPosition.minus(RobotPosition.getTranslation());
        double distance = Math.sqrt(difference.getX() * difference.getX() + difference.getY() * difference.getY()); // it
                                                                                                                    // is
                                                                                                                    // written
                                                                                                                    // like
                                                                                                                    // this
                                                                                                                    // because
                                                                                                                    // it
                                                                                                                    // makes
                                                                                                                    // Caden
                                                                                                                    // mad
                                                                                                                    // :)
        return distance;
    }

    // todo this
    public static double getTangentalSpeed(ChassisSpeeds speeds, Pose2d robotPos) {
        double vx = speeds.vxMetersPerSecond;
        double vy = speeds.vyMetersPerSecond;
        double relativeX = Constants.Field.HOPPER_BLUE.getX() - robotPos.getX();
        double relativeY = Constants.Field.HOPPER_BLUE.getY() - robotPos.getY();

        double relativeMagPos = Math.sqrt(relativeX * relativeX + relativeY * relativeY);
        double magVelocity = Math.sqrt(vx * vx + vy * vy);
        double normalizedX = relativeX / relativeMagPos;
        double normalizedY = relativeY / relativeMagPos;

        return Math.sqrt(magVelocity * magVelocity - Math.pow(vx * normalizedX + vy * normalizedY, 2));
    }

    public static double getRadialSpeed(ChassisSpeeds speeds, Pose2d robotPos) {
        double vx = speeds.vxMetersPerSecond;
        double vy = speeds.vyMetersPerSecond;
        double relativeX = Constants.Field.HOPPER_BLUE.getX() - robotPos.getX();
        double relativeY = Constants.Field.HOPPER_BLUE.getY() - robotPos.getY();

        double relativeMagPos = Math.sqrt(relativeX * relativeX + relativeY * relativeY);
        double normalizedX = relativeX / relativeMagPos;
        double normalizedY = relativeY / relativeMagPos;

        return vx * normalizedX + vy * normalizedY;
    }

    public static double dot(Translation2d a, Translation2d b) {
        return a.getX() * b.getX() + a.getY() * b.getY();
    }

    public static Translation2d project(Translation2d a, Translation2d onto) {
        return onto.times(dot(a, onto) / dot(onto, onto));
    }

    public static Translation2d reject(Translation2d a, Translation2d onto) {
        return a.minus(project(a, onto));
    }

    public static Translation2d elementMult(Translation2d a, Translation2d b) {
        return new Translation2d(a.getX() * b.getX(), a.getY() * b.getY());
    }

    public static double getTurretAngle(double turretPosRotation) {
        if (turretPosRotation >= 0) {
            return ((turretPosRotation * 360) - 180); // if encoder and cam rot are backwards, mult by -1
        } else {
            return ((turretPosRotation * 360) + 180);
        }
    }
}
