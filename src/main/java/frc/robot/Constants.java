// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.Interpolatable;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.slicelibs.configs.CTREConfigs;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final Mode ADVANTAGE_KIT_MODE = Mode.REAL;
    public static final CTREConfigs CTRE_CONFIGS = new CTREConfigs();

    public static enum Mode {
        REAL,
        SIM,
        REPLAY
    }

    public final class AlignTargets {
        public static final Translation2d BLUE_HUB = new Translation2d(4.625, 4.034);
        public static final Translation2d RED_HUB = new Translation2d(11.915, 4.034);

        public static final Translation2d BLUE_PASS_LEFT = new Translation2d(3.0, 6.5);
        public static final Translation2d BLUE_PASS_RIGHT = new Translation2d(3.0, 1.5);
        public static final Translation2d RED_PASS_LEFT = new Translation2d(13.5, 6.5);
        public static final Translation2d RED_PASS_RIGHT = new Translation2d(13.5, 1.5);

        public static final double HEADING_KP = 6.0;
        public static final double HEADING_KI = 0.0;
        public static final double HEADING_KD = 0.3;
        public static final double HEADING_TOLERANCE_DEG = 2.0;
    }

    public static class IndexerConstants {
        /* Indexer Motor IDs */
        public static final int STAGE_ONE_MOTOR_ID = 18;
        public static final int STAGE_TWO_MOTOR_ID = 1;

        public static final double STAGE_ONE_INTAKE_SPEED = 0.5;
        public static final double STAGE_TWO_INTAKE_SPEED = 0.5;

        public static final int INDEXER_STATOR_CURRENT_LIMIT = 40;
        public static final int INDEXER_SUPPLY_CURRENT_LIMIT = 30;
    }

    public static class DriveConstants {

        /* Swerve Physics */
        public static final double TRACK_WIDTH = Units.inchesToMeters(22.0);
        public static final double WHEEL_BASE = Units.inchesToMeters(23.1782);
        public static final double DRIVE_BASE_RADIUS = Math.hypot(WHEEL_BASE / 2, TRACK_WIDTH / 2);
        public static final double WHEEL_DIAMETER = Units.inchesToMeters(3.95);
        public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;
        public static final double MASS = 65; // TODO: Find mass (kg)
        public static final double MOMENT_OF_INERTIA = 6; // TODO: Find MOI (kg*m^2)
        public static final double WHEEL_COEFFICIENT_OF_FRICTION = 1.5;

        public static final SwerveDriveKinematics kSwerveKinematics = new SwerveDriveKinematics(
                new Translation2d(WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0), // Front left
                new Translation2d(WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0), // Front right
                new Translation2d(-WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0), // Back right
                new Translation2d(-WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0)); // Back left

        /* Motor IDs */
        public static final int FRONT_LEFT_DRIVE_ID = 17;
        public static final int FRONT_RIGHT_DRIVE_ID = 13;
        public static final int BACK_LEFT_DRIVE_ID = 15;
        public static final int BACK_RIGHT_DRIVE_ID = 11;

        public static final int FRONT_LEFT_TURN_ID = 16;
        public static final int FRONT_RIGHT_TURN_ID = 12;
        public static final int BACK_LEFT_TURN_ID = 14;
        public static final int BACK_RIGHT_TURN_ID = 10;

        /* CANcoder IDs */
        public static final int FRONT_LEFT_ENCODER_ID = 23;
        public static final int FRONT_RIGHT_ENCODER_ID = 21;
        public static final int BACK_LEFT_ENCODER_ID = 20;
        public static final int BACK_RIGHT_ENCODER_ID = 22;

        public static final int GYRO_ID = 24;

        /* Gear Ratios */
        public static final double DRIVE_GEAR_RATIO = (5.79 / 1.0); // 5.79:1
        public static final double ANGLE_GEAR_RATIO = (25.0 / 1.0); // 25:1

        /* Drive Motor PID / FF */
        public static final double DRIVE_KP = 0.05;
        public static final double DRIVE_KI = 0.0;
        public static final double DRIVE_KD = 0.0;
        public static final double DRIVE_KS = 0.0;
        public static final double DRIVE_KV = 0.12;

        /* Turn Motor PID */
        public static final double TURN_KP = 55.0;
        public static final double TURN_KI = 0.0;
        public static final double TURN_KD = 0.0;

        /* Current Limits */
        public static final double DRIVE_STATOR_CURRENT_LIMIT = 80;
        public static final double DRIVE_SUPPLY_CURRENT_LIMIT = 40;
        public static final double TURN_STATOR_CURRENT_LIMIT = 40;
        public static final double TURN_SUPPLY_CURRENT_LIMIT = 30;

        /* Motor Inverts */
        public static final boolean DRIVE_MOTOR_INVERT = false;
        public static final boolean TURN_MOTOR_INVERT = true;

        /* CANcoder direction */
        public static final com.ctre.phoenix6.signals.SensorDirectionValue ABSOLUTE_ENCODER_INVERT = com.ctre.phoenix6.signals.SensorDirectionValue.CounterClockwise_Positive;

        /* Velocity limits */
        public static final double MAX_LINEAR_VELOCITY = 4.5; // m/s
        public static final double MAX_ANGULAR_VELOCITY = 5.279; // rad/s

        /*
         * //TODO remake PathPlanner constants
         * 
         * // PathPlanner
         * public static final com.pathplanner.lib.path.PathConstraints PATH_CONSTRAINTS
         * =
         * new com.pathplanner.lib.path.PathConstraints(3.5, 2.5, Math.PI * 2, Math.PI *
         * 2);
         * public static final double TRANSLATION_KP = 4.5;
         * public static final double ROTATION_KP = 1.0;
         * 
         * //TODO I don't think these swerve module constants are necessary anymore, but
         * I could indeed be wrong
         * 
         * // Per-module constants (drive ID, turn ID, CANcoder ID, angle offset)
         * public static final frc.slicelibs.configs.SwerveModuleConstants
         * FRONT_LEFT_MODULE =
         * new frc.slicelibs.configs.SwerveModuleConstants(
         * FRONT_LEFT_DRIVE_ID, FRONT_LEFT_TURN_ID, FRONT_LEFT_ENCODER_ID,
         * Rotation2d.fromRotations(0.516602));
         * 
         * public static final frc.slicelibs.configs.SwerveModuleConstants
         * FRONT_RIGHT_MODULE =
         * new frc.slicelibs.configs.SwerveModuleConstants(
         * FRONT_RIGHT_DRIVE_ID, FRONT_RIGHT_TURN_ID, FRONT_RIGHT_ENCODER_ID,
         * Rotation2d.fromRotations(0.479248));
         * 
         * public static final frc.slicelibs.configs.SwerveModuleConstants
         * BACK_RIGHT_MODULE =
         * new frc.slicelibs.configs.SwerveModuleConstants(
         * BACK_RIGHT_DRIVE_ID, BACK_RIGHT_TURN_ID, BACK_RIGHT_ENCODER_ID,
         * Rotation2d.fromRotations(0.817139));
         * 
         * public static final frc.slicelibs.configs.SwerveModuleConstants
         * BACK_LEFT_MODULE =
         * new frc.slicelibs.configs.SwerveModuleConstants(
         * BACK_LEFT_DRIVE_ID, BACK_LEFT_TURN_ID, BACK_LEFT_ENCODER_ID,
         * Rotation2d.fromRotations(0.519043));
         * 
         */
    }

    public static final class OIConstants {
        public static final int kDriverControllerPort = 0;
        public static final int kOperatorControllerPort = 1;
        public static final double kDriveDeadband = 0.025;
    }

    public static class IntakeConstants {

        public static final int ROTATION_MOTOR_ID = 6;
        public static final int EXTENDER_MOTOR_ID = 5;

        // Positional subsystem constants
        public static final double EXTENDER_KP = 0.2;
        public static final double EXTENDER_KI = 0.0;
        public static final double EXTENDER_KD = 0.0;
        public static final double EXTENDER_KG = 0.0; // FF for gravity, most likely don't need this
        public static final double EXTENDER_RATIO = 50.0 / 9.0; // 5.55 repeating
        public static final int EXTENDER_STATOR_CURRENT_LIMIT = 60;
        public static final int EXTENDER_SUPPLY_CURRENT_LIMIT = 40;
        public static final double POSITION_CONVERSION_FACTOR = (0.0254 * Math.PI) * EXTENDER_RATIO; // (pitch diameter
                                                                                                     // of pinion * pi)
                                                                                                     // * ratio
        public static final double VELOCITY_CONVERSION_FACTOR = POSITION_CONVERSION_FACTOR; // meters per second

        public static final double STOWED_POSITION = 0.0; // Meters
        public static final double DEPLOYED_POSITION = 0.2921; // Meters
        public static final double OSCILLATION_AMOUNT = .02; // Meters; how far it goes out
        public static final double OSCILLATION_DIFF = .01; // Meters; how much further it goes in
        // Roller motor constants
        public static final double ROLLER_SPEED = 0.8;
        public static final double ROLLER_RETRACT_SPEED = 0.0;
        public static final double ROLLER_GEAR_RATIO = 2.0;
        public static final int ROLLER_STATOR_CURRENT_LIMIT = 40;
        public static final int ROLLER_SUPPLY_CURRENT_LIMIT = 30;
    }

    public static class ShooterConstants {

        public static final int PIVOT_MOTOR_ID = 4;
        public static final int LEFT_SHOOTER_MOTOR_ID = 3;
        public static final int RIGHT_SHOOTER_MOTOR_ID = 2;

        // TODO: tune PIDs
        public static final double FLYWHEEL_KP = 0.35;
        public static final double FLYWHEEL_KI = 0.0;
        public static final double FLYWHEEL_KD = 0.0;
        public static final double FLYWHEEL_KS = 0.0;
        public static final double FLYWHEEL_KV = 0.12;
        public static final int FLYWHEEL_STATOR_CURRENT_LIMIT = 80;
        public static final int FLYWHEEL_SUPPLY_CURRENT_LIMIT = 60;

        public static final double FLYWHEEL_GEAR_RATIO = 1.4;
        public static final double PIVOT_GEAR_RATIO = 4.75 * 16.5;
        public static final double POSITION_CONVERSION_FACTOR = 360; // Degrees
        public static final double VELOCITY_CONVERSION_FACTOR = POSITION_CONVERSION_FACTOR; // Degrees per Second

        public static final double AIM_KP = 0.6;
        public static final double AIM_KI = 0.0;
        public static final double AIM_KD = 0.0;
        public static final double AIM_KG = 0.01; // gravity feedforward
        public static final int PIVOT_STATOR_CURRENT_LIMIT = 60;
        public static final int PIVOT_SUPPLY_CURRENT_LIMIT = 40;

        public static final double SHOOTER_STOW = 12.0; // The angle at which the shooter is considered stowed
        public static final double FLYWHEEL_RPM_ACCEPTABLE_ERROR = 10.0; // rpm
        public static final double VERTICAL_AIM_ACCEPTABLE_ERROR = .1; // degrees
        // TODO: Populate table with real data (placeholder example data right now)
        public static final InterpolatingTreeMap<Double, FullShooterParams> SHOOTER_MAP = new InterpolatingTreeMap<>(
                MathUtil::inverseInterpolate, FullShooterParams::interpolate);
        static {
            SHOOTER_MAP.put(0.5, new FullShooterParams(1240.0, 12.0, 1.04));
            SHOOTER_MAP.put(1.02, new FullShooterParams(1300.0, 12.0, 1.24));
            SHOOTER_MAP.put(1.58, new FullShooterParams(1450.0, 13.0, 1.11));
            SHOOTER_MAP.put(1.98, new FullShooterParams(1550.0, 17.0, 1.14));
            SHOOTER_MAP.put(2.74, new FullShooterParams(1600.0, 20.0, 1.11));
            SHOOTER_MAP.put(3.65, new FullShooterParams(1675.0, 23.0, 1.08));
            SHOOTER_MAP.put(4.79, new FullShooterParams(1779.0, 26.4, 1.26));

        }

        public record FullShooterParams(double rpm, double hoodAngle, double tof)
                implements Interpolatable<FullShooterParams> {
            @Override
            public FullShooterParams interpolate(FullShooterParams endValue, double t) {
                return new FullShooterParams(
                        rpm + (endValue.rpm - rpm) * t,
                        hoodAngle + (endValue.hoodAngle - hoodAngle) * t,
                        tof + (endValue.tof - tof) * t);
            }
        }

        // Robot dimensions
        public static final double SHOOTER_HEIGHT = 1.7891; // Feet
        public static final double FLYWHEEL_RADIUS = 0.1667; // Feet
        public static final double LIMELIGHT_ANGLE = 30.29; // Degrees
        public static final double LIMELIGHT_HEIGHT = 0.52; // Metres
    }

    public static class FieldConstants {
        /*
         * public static final Translation2d BLUE_HUB = new Translation2d(4.62534,
         * 4.03479); // TODO: Check the hubs (in metres)
         * public static final Translation2d RED_HUB = new Translation2d(11.91514,
         * 4.03479);
         */
        public static final double GRAVITY = 32.185; // Feet per second per second

        // Height and length of the hub
        public static final double HUB_HEIGHT = 6.15; // Feet
        public static final double HUB_HALF_LENGTH = 1.958335; // Feet
        public static final double HUB_APRILTAG_HEIGHT = 1.124; // Metres

        // How long to speed up shooter before hub active
        public static final double SPEED_SHOOTER_AT = 4; // Seconds

    }

}