package frc.robot.subsystems;

import java.util.Optional;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants.FullShooterParams;
import frc.robot.LimelightHelpers;
import frc.slicelibs.TalonFXPositionalSubsystem;

public class Shooter extends TalonFXPositionalSubsystem {

    private final CommandSwerveDrivetrain m_drivetrain;
    private TalonFX leftShooterMotor, rightShooterMotor;
    private StrictFollower rightFollowerRequest;
    private final VelocityVoltage flywheelVelocityRequest = new VelocityVoltage(0).withEnableFOC(true);

    private boolean tuningMode = false;
    private double tunedRPM = 3000.0;
    private double tunedHoodAngle = 20.0;

    private double targetSpeed, targetPosition;

    /**
     * @param drivetrain required for field-relative velocity in SWIM calculations
     */
    public Shooter(CommandSwerveDrivetrain drivetrain) {
        super(new int[] { Constants.ShooterConstants.PIVOT_MOTOR_ID },
                new boolean[] { true },
                Constants.ShooterConstants.AIM_KP,
                Constants.ShooterConstants.AIM_KI,
                Constants.ShooterConstants.AIM_KD,
                Constants.ShooterConstants.AIM_KG,
                Constants.ShooterConstants.PIVOT_GEAR_RATIO,
                GravityTypeValue.Arm_Cosine,
                Constants.ShooterConstants.POSITION_CONVERSION_FACTOR,
                Constants.ShooterConstants.VELOCITY_CONVERSION_FACTOR,
                Constants.CTRE_CONFIGS.pivotConfigs);
        setEncoderPosition(Constants.ShooterConstants.SHOOTER_STOW);

        SmartDashboard.putBoolean("Shooter/TuningMode", false);
        SmartDashboard.putNumber("Shooter/TunedRPM", 3000.0);
        SmartDashboard.putNumber("Shooter/TunedHoodAngle", 20.0);

        leftShooterMotor = new TalonFX(Constants.ShooterConstants.LEFT_SHOOTER_MOTOR_ID);
        rightShooterMotor = new TalonFX(Constants.ShooterConstants.RIGHT_SHOOTER_MOTOR_ID);

        leftShooterMotor.getConfigurator().apply(Constants.CTRE_CONFIGS.shooterConfigs);
        rightShooterMotor.getConfigurator().apply(Constants.CTRE_CONFIGS.shooterFollowerConfigs);
        rightFollowerRequest = new StrictFollower(leftShooterMotor.getDeviceID());
        m_drivetrain = drivetrain;
    }

    public void spinFlywheels(double targetRPM) {
        leftShooterMotor.setControl(flywheelVelocityRequest.withVelocity(targetRPM /*  / 60.0*/));
        rightShooterMotor.setControl(rightFollowerRequest);
    }
    
    public void pivotShooter(double angle) {
        setPosition(angle);
    }

    /**
     * Projects the robot's field-relative velocity onto the robot-to-target vector
     * to compute
     * the required horizontal ball velocity (velocity needed to reach target directly), accounting for robot movement (SWIM
     * compensation).
     *
     * @param distance current distance to target in meters
     * @param target   field-relative target position
     * @return required horizontal ball velocity in m/s
     * @see <a href=
     *      "https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/math/kinematics/ChassisSpeeds.html">ChassisSpeeds</a>
     */
    public double getHorizontalVelocity(double distance, Translation2d target) {
        FullShooterParams params = Constants.ShooterConstants.SHOOTER_MAP.get(distance);
        double baselineHorizVel = distance / params.tof();

        ChassisSpeeds fieldSpeeds = m_drivetrain.getFieldRelativeSpeeds();
        Translation2d toTarget = target.minus(m_drivetrain.getPose().getTranslation()).getNorm() > 0
                ? target.minus(m_drivetrain.getPose().getTranslation())
                : new Translation2d(1, 0);
        Translation2d unitVec = toTarget.div(toTarget.getNorm());
        double robotVelAlongTarget = fieldSpeeds.vxMetersPerSecond * unitVec.getX()
                + fieldSpeeds.vyMetersPerSecond * unitVec.getY();

        return baselineHorizVel - robotVelAlongTarget;
    }

    public void calculateShot(double distance, double requiredVelocity) {
        FullShooterParams baseline = Constants.ShooterConstants.SHOOTER_MAP.get(distance);
        double baselineVelocity = distance / baseline.tof();

        // Flywheel velocity
        double velocityRatio = MathUtil.clamp(requiredVelocity / baselineVelocity, 0.5, 2.0);
        targetSpeed = baseline.rpm() * velocityRatio;

        // Hood angle
        double totalVelocity = baselineVelocity / Math.cos(Math.toRadians(baseline.hoodAngle()));
        double targetHoriz = MathUtil.clamp(requiredVelocity, 0.0, totalVelocity);
        targetPosition = Math.toDegrees(Math.acos(targetHoriz / totalVelocity));
    }

    public void windDownFlywheels() {
        leftShooterMotor.stopMotor();
        rightShooterMotor.stopMotor();
    }

    public double distanceFromHub() {
        double distance = -1;
        if (!LimelightHelpers.getTV("limelight-shooter")) {
            return distance; // Invalid distance
        }
        double offsetAngleVertical = LimelightHelpers.getTY("limelight-shooter");
        double angleToGoal = Math.toRadians(Constants.ShooterConstants.LIMELIGHT_ANGLE + offsetAngleVertical);

        distance = (Constants.FieldConstants.HUB_APRILTAG_HEIGHT - Constants.ShooterConstants.LIMELIGHT_HEIGHT)
                / Math.tan(angleToGoal);

        return Math.abs(distance);
    }

    public double getFlywheelSpeed() {
        return (leftShooterMotor.getVelocity().getValueAsDouble() + rightShooterMotor.getVelocity().getValueAsDouble())
                / 2 * 60.0;
    }

    public double getPivotPosition() {
        return getPositions()[0];
    }

    public double getTargetVelocity() {
        return targetSpeed;
    }

    public double getTargetPosition() {
        return targetPosition;
    }

    public boolean atTargetSpeed() {
        return Math.abs(targetSpeed - getFlywheelSpeed()) <= Constants.ShooterConstants.FLYWHEEL_RPM_ACCEPTABLE_ERROR;
    }

    public boolean atTargetPosition() {
        return Math.abs(targetPosition
                - getPivotPosition()) <= (Constants.ShooterConstants.VERTICAL_AIM_ACCEPTABLE_ERROR * (Math.PI / 180));
    }

    public boolean isTuningMode() {
        return tuningMode;
    }

    public boolean isHubActive() {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isEmpty())
            return false;
        if (DriverStation.isAutonomousEnabled())
            return true;
        if (!DriverStation.isTeleopEnabled())
            return false;

        double matchTime = DriverStation.getMatchTime();
        String gameData = DriverStation.getGameSpecificMessage();
        if (gameData.isEmpty())
            return true;

        boolean redInactiveFirst = false;
        switch (gameData.charAt(0)) {
            case 'R' -> redInactiveFirst = true;
            case 'B' -> redInactiveFirst = false;
            default -> {
                return true;
            }
        }

        boolean shift1Active = switch (alliance.get()) {
            case Red -> !redInactiveFirst;
            case Blue -> redInactiveFirst;
        };

        if (matchTime > 130)
            return true;
        else if (matchTime > 105)
            return shift1Active;
        else if (matchTime > 80)
            return !shift1Active;
        else if (matchTime > 55)
            return shift1Active;
        else if (matchTime > 30)
            return !shift1Active;
        else
            return true;
    }

    public boolean isHubAlmostActive() {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isEmpty())
            return false;
        if (DriverStation.isAutonomousEnabled())
            return true;
        if (!DriverStation.isTeleopEnabled())
            return false;

        double matchTime = DriverStation.getMatchTime();
        String gameData = DriverStation.getGameSpecificMessage();
        if (gameData.isEmpty())
            return true;

        boolean redInactiveFirst = false;
        switch (gameData.charAt(0)) {
            case 'R' -> redInactiveFirst = true;
            case 'B' -> redInactiveFirst = false;
            default -> {
                return true;
            }
        }

        boolean shift1Active = switch (alliance.get()) {
            case Red -> !redInactiveFirst;
            case Blue -> redInactiveFirst;
        };

        double s = Constants.FieldConstants.SPEED_SHOOTER_AT;
        if (matchTime > (130 - s))
            return true;
        else if (matchTime > (105 - s))
            return shift1Active;
        else if (matchTime > (80 - s))
            return !shift1Active;
        else if (matchTime > (55 - s))
            return shift1Active;
        else if (matchTime > (30 - s))
            return !shift1Active;
        else
            return true;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shooter/ActualRPM", getFlywheelSpeed());
        SmartDashboard.putNumber("Shooter/TargetRPM", targetSpeed);
        SmartDashboard.putNumber("Shooter/TargetHoodAngle", targetPosition);
        SmartDashboard.putNumber("Shooter/ActualHoodAngle", getPivotPosition());
        SmartDashboard.putBoolean("Shooter/AtSpeed", atTargetSpeed());
        SmartDashboard.putBoolean("Shooter/AtPosition", atTargetPosition());

        tuningMode = SmartDashboard.getBoolean("Shooter/TuningMode", false);
        SmartDashboard.putBoolean("Shooter/TuningMode", tuningMode);

        SmartDashboard.putNumber("Shooter velocity: ", getFlywheelSpeed());
        SmartDashboard.putNumber("Shooter supposed velocity: ", targetSpeed);
        SmartDashboard.putNumber("Shooter angle: ", getPivotPosition());
        SmartDashboard.putNumber("Shooter supposed angle: ", targetPosition);

        if (tuningMode) {
            tunedRPM = SmartDashboard.getNumber("Shooter/TunedRPM", tunedRPM);
            tunedHoodAngle = SmartDashboard.getNumber("Shooter/TunedHoodAngle", tunedHoodAngle);
            SmartDashboard.putNumber("Shooter/TunedRPM", tunedRPM);
            SmartDashboard.putNumber("Shooter/TunedHoodAngle", tunedHoodAngle);
            targetSpeed = tunedRPM;
            targetPosition = tunedHoodAngle;
        }
    }
}
