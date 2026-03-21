package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import java.util.Optional;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.config.PIDConstants;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;

/**
 * CTRE-generated CommandSwerveDrivetrain extended with shooter-support helpers:
 * pose, field-relative speeds, distance/heading math, and heading PID.
 *
 * @see <a href="https://api.ctr-electronics.com/phoenix6/release/java/">Phoenix
 *      6 API</a>
 * @see <a href="https://github.wpilib.org/allwpilib/docs/release/java/">WPILib
 *      API</a>
 */
@SuppressWarnings("unused")
public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.004;
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
    private boolean m_hasAppliedOperatorPerspective = false;

    private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
    private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
    private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();

    private final SwerveRequest.ApplyRobotSpeeds autoRequest = new SwerveRequest.ApplyRobotSpeeds();
    

    private final SysIdRoutine m_sysIdRoutineTranslation = new SysIdRoutine(
            new SysIdRoutine.Config(null, Volts.of(4), null,
                    state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())),
            new SysIdRoutine.Mechanism(
                    output -> setControl(m_translationCharacterization.withVolts(output)), null, this));

    private final SysIdRoutine m_sysIdRoutineSteer = new SysIdRoutine(
            new SysIdRoutine.Config(null, Volts.of(7), null,
                    state -> SignalLogger.writeString("SysIdSteer_State", state.toString())),
            new SysIdRoutine.Mechanism(
                    volts -> setControl(m_steerCharacterization.withVolts(volts)), null, this));

    private final SysIdRoutine m_sysIdRoutineRotation = new SysIdRoutine(
            new SysIdRoutine.Config(
                    Volts.of(Math.PI / 6).per(Second), Volts.of(Math.PI), null,
                    state -> SignalLogger.writeString("SysIdRotation_State", state.toString())),
            new SysIdRoutine.Mechanism(output -> {
                setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
                SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
            }, null, this));

    private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineTranslation;

    /** Heading PID used by AlignAndShoot for auto-rotation toward a target. */
    private final PIDController headingPID = new PIDController(
            Constants.AlignTargets.HEADING_KP,
            Constants.AlignTargets.HEADING_KI,
            Constants.AlignTargets.HEADING_KD);

    
    /* ShuffleBoard Stuffs */
    ShuffleboardTab driverTab;
    public Field2d m_Field;

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants drivetrainConstants,
            SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, modules);
        //configureAutoBuilder();
        configHeadingPID();
        if (Utils.isSimulation())
            startSimThread();

        m_Field = new Field2d();
        driverTab = Shuffleboard.getTab("Driver");
    }

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants drivetrainConstants, double odometryUpdateFrequency,
            SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, odometryUpdateFrequency, modules);
        //configureAutoBuilder();
        configHeadingPID();
        if (Utils.isSimulation())
            startSimThread();
        
        m_Field = new Field2d();
        //driverTab = Shuffleboard.getTab("Driver"); TODO i dont think we need this but we'll see
    }

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants drivetrainConstants, double odometryUpdateFrequency,
            Matrix<N3, N1> odometryStandardDeviation,
            Matrix<N3, N1> visionStandardDeviation, SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, odometryUpdateFrequency, odometryStandardDeviation, visionStandardDeviation,
                modules);
        //configureAutoBuilder();
        configHeadingPID();
        if (Utils.isSimulation())
            startSimThread();

        m_Field = new Field2d();
        //driverTab = Shuffleboard.getTab("Driver");
    }

    private void configHeadingPID() {
        headingPID.enableContinuousInput(-Math.PI, Math.PI);
        headingPID.setTolerance(Math.toRadians(Constants.AlignTargets.HEADING_TOLERANCE_DEG));
    }

    private void configureAutoBuilder() {
            //field = new Field2d();
            try {
                var config = RobotConfig.fromGUISettings();
                AutoBuilder.configure(
                    () -> getPose(),   // Supplier of current robot pose
                    this::resetPose,         // Consumer for seeding pose against auto
                    () -> getChassisSpeeds(), // Supplier of current robot speeds
                    // Consumer of ChassisSpeeds and feedforwards to drive the robot
                    (speeds, feedforwards) -> setControl(
                        autoRequest.withSpeeds(ChassisSpeeds.discretize(speeds, 0.020))
                            .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                            .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())
                    ),
                    new PPHolonomicDriveController(
                        // PID constants for translation
                        new PIDConstants(10, 0, 0),
                        // PID constants for rotation
                        new PIDConstants(7, 0, 0)
                    ),
                    config,
                    // Assume the path needs to be flipped for Red vs Blue, this is normally the case
                    () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
                    this // Subsystem for requirements
                );
            } catch (Exception ex) {
                DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder", ex.getStackTrace());
            }
        }

    // -------------------------------------------------------------------------
    // Core drivetrain commands
    // -------------------------------------------------------------------------

    /**
     * Returns a command that applies the specified {@link SwerveRequest} to this
     * drivetrain.
     *
     */
    public Command applyRequest(Supplier<SwerveRequest> request) {
        return run(() -> this.setControl(request.get()));
    }

    /**
     * @param direction SysId direction
     * @return quasistatic characterization command
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.quasistatic(direction);
    }

    /**
     * @param direction SysId direction
     * @return dynamic characterization command
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.dynamic(direction);
    }

    // Pose / odometry helpers

    /**
     * Returns the current estimated robot pose from the CTRE pose estimator.
     *
     */
    public Pose2d getPose() {
        return getState().Pose;
    }

    /**
     * Returns the robot-relative {@link ChassisSpeeds} from the current drive
     * state.
     *
     */
    public ChassisSpeeds getChassisSpeeds() {
        return getState().Speeds;
    }

    /**
     * Returns field-relative {@link ChassisSpeeds} by rotating robot-relative
     * speeds by the current heading.
     *
     */
    public ChassisSpeeds getFieldRelativeSpeeds() {
        return ChassisSpeeds.fromRobotRelativeSpeeds(getChassisSpeeds(), getPose().getRotation());
    }

    /**
     * Straight-line distance from the current robot pose to a field target.
     *
     * @param target field-relative target position in meters
     */
    public double getDistanceTo(Translation2d target) {
        return getPose().getTranslation().getDistance(target);
    }

    /**
     * Returns a lead-corrected target position based on current robot velocity and
     * the shot time-of-flight from the shooter map.
     *
     * @param realTarget the actual field target
     * @return corrected target accounting for robot motion during TOF
     */
    public Translation2d getCompensatedTarget(Translation2d realTarget) {
        double dist = getDistanceTo(realTarget);
        double tof = Constants.ShooterConstants.SHOOTER_MAP.get(dist).tof();
        ChassisSpeeds fieldSpeeds = getFieldRelativeSpeeds();
        //return realTarget;
        return new Translation2d(
                realTarget.getX() - fieldSpeeds.vxMetersPerSecond * tof, //TODO multiply by factor possibly
                realTarget.getY() - fieldSpeeds.vyMetersPerSecond * tof);
        
    }

    // Heading PID helpers (used by AlignAndShoot)
    /**
     * Returns the desired robot heading to face the given field target.
     */
    public Rotation2d getTargetHeading(Translation2d target) {
        Translation2d robotPos = getPose().getTranslation();
        return Rotation2d.fromRadians(0.5 * Math.atan2(
                target.getY() - robotPos.getY(),
                target.getX() - robotPos.getX()));
    }

    /**
     * Returns heading PID output (rad/s) toward the given target.
     */
    public double getHeadingPIDOutput(Translation2d target) {
        return headingPID.calculate(
                getPose().getRotation().getRadians(),
                getTargetHeading(target).getRadians());
    }

    /** @return true when the heading PID is within tolerance */
    public boolean atTargetHeading() {
        return headingPID.atSetpoint();
    }

    // Vision measurement overrides
    @Override
    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds));
    }

    @Override
    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds,
            Matrix<N3, N1> visionMeasurementStdDevs) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds),
                visionMeasurementStdDevs);
    }

    @Override
    public Optional<Pose2d> samplePoseAt(double timestampSeconds) {
        return super.samplePoseAt(Utils.fpgaToCurrentTime(timestampSeconds));
    }

    // Simulation
    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    public void resetPose(Pose2d pose) {
        super.resetPose(pose);
    }

    @Override
    public void periodic() {
        if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                setOperatorPerspectiveForward(
                    allianceColor == Alliance.Red
                        ? kRedAlliancePerspectiveRotation
                        : kBlueAlliancePerspectiveRotation
                );
                m_hasAppliedOperatorPerspective = true;
            });
        }
        m_Field.setRobotPose(getState().Pose);
        SmartDashboard.putData("Pose", m_Field);
        

        // Vision update with MegaTag2 if tags visible
        
        if (DriverStation.getAlliance().equals(Alliance.Red)) {
            var limelightPose = LimelightHelpers.getBotPoseEstimate_wpiRed_MegaTag2("limelight-shooter");
            if (limelightPose != null && limelightPose.tagCount > 0) {
            addVisionMeasurement(limelightPose.pose, limelightPose.timestampSeconds);
        }
        } else if (DriverStation.getAlliance().equals(Alliance.Blue)) {
            var limelightPose = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-shooter");
            if (limelightPose != null && limelightPose.tagCount > 0) {
            addVisionMeasurement(limelightPose.pose, limelightPose.timestampSeconds);
        }
        }
        
        
        
    }
}
