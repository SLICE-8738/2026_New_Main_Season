package frc.robot.subsystems;

import java.util.Optional;

import javax.xml.crypto.dsig.spec.C14NMethodParameterSpec;

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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants.FullShooterParams;
import frc.robot.LimelightHelpers;
import frc.slicelibs.TalonFXPositionalSubsystem;

public class Shooter extends SubsystemBase {

    private final CommandSwerveDrivetrain m_drivetrain;
    private TalonFX bottomLeftShooterMotor, bottomRightShooterMotor, topLeftShooterMotor, topRightShooterMotor;
    private StrictFollower bottomRightFollowerRequest;
    private StrictFollower topLeftFollowerRequest;
    private StrictFollower topRightFollowerRequest;
    private final VelocityVoltage flywheelVelocityRequest = new VelocityVoltage(0).withEnableFOC(true);

    private boolean tuningMode = false;
    private double tunedRPM = 3000.0;

    private static double goodSpeed = 1200.0;

    private double targetSpeed;

    /**
     * @param drivetrain required for field-relative velocity in SWIM calculations
     */
    public Shooter(CommandSwerveDrivetrain drivetrain) {
        //SmartDashboard.putBoolean("Shooter/TuningMode", false);
        //SmartDashboard.putNumber("Shooter/TunedRPM", 3000.0);

        bottomLeftShooterMotor = new TalonFX(Constants.ShooterConstants.BOTTOM_LEFT_SHOOTER_MOTOR_ID);
        bottomRightShooterMotor = new TalonFX(Constants.ShooterConstants.BOTTOM_RIGHT_SHOOTER_MOTOR_ID);
        topLeftShooterMotor = new TalonFX(Constants.ShooterConstants.TOP_LEFT_SHOOTER_MOTOR_ID);
        topRightShooterMotor = new TalonFX(Constants.ShooterConstants.TOP_RIGHT_SHOOTER_MOTOR_ID);

        bottomLeftShooterMotor.getConfigurator().apply(Constants.CTRE_CONFIGS.shooterConfigs);
        bottomRightShooterMotor.getConfigurator().apply(Constants.CTRE_CONFIGS.shooterFollowerConfigs);
        topRightShooterMotor.getConfigurator().apply(Constants.CTRE_CONFIGS.shooterFollowerConfigs);
        topLeftShooterMotor.getConfigurator().apply(Constants.CTRE_CONFIGS.shooterConfigs);

        bottomRightFollowerRequest = new StrictFollower(bottomLeftShooterMotor.getDeviceID());
        topLeftFollowerRequest = new StrictFollower(bottomLeftShooterMotor.getDeviceID());
        topRightFollowerRequest = new StrictFollower(bottomLeftShooterMotor.getDeviceID());
        m_drivetrain = drivetrain;
    }

    public void spinFlywheels(double targetRPM) {
        bottomLeftShooterMotor.setControl(flywheelVelocityRequest.withVelocity(targetRPM   / 60.0));
        bottomRightShooterMotor.setControl(bottomRightFollowerRequest);
        topLeftShooterMotor.setControl(topLeftFollowerRequest);
        topRightShooterMotor.setControl(topRightFollowerRequest);
    }

    public double getGoodSpeed(){
        return goodSpeed;
    }

    public void setGoodSpeed(double speed){
        goodSpeed = speed;
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
    /*public double getHorizontalVelocity(double distance, Translation2d target) {
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
    }*/

    public void windDownFlywheels() {
        bottomLeftShooterMotor.stopMotor();
        bottomRightShooterMotor.stopMotor();
        topLeftShooterMotor.stopMotor();
        topRightShooterMotor.stopMotor();
    }

    public double distanceFromTrench(){
        boolean isBlue = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue;
        double distance = -1;
        // get distance to each trench
        if (isBlue){
            Translation2d leftTarget = Constants.AlignTargets.BLUE_TRENCH_LEFT;
            Translation2d rightTarget = Constants.AlignTargets.BLUE_TRENCH_RIGHT;
            double leftDistance = m_drivetrain.getDistanceTo(leftTarget);
            double rightDistance = m_drivetrain.getDistanceTo(rightTarget);
            distance = (leftDistance < rightDistance) ? leftDistance : rightDistance;
        } else {
            Translation2d leftTarget = Constants.AlignTargets.RED_TRENCH_LEFT;
            Translation2d rightTarget = Constants.AlignTargets.RED_TRENCH_RIGHT;
            double leftDistance = m_drivetrain.getDistanceTo(leftTarget);
            double rightDistance = m_drivetrain.getDistanceTo(rightTarget);
            distance = (leftDistance < rightDistance) ? leftDistance : rightDistance;
        }

        return distance;
    }

    public double distanceFromHub() {
        boolean isBlue = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue;
        Translation2d targetPosition = isBlue ? Constants.AlignTargets.BLUE_HUB : Constants.AlignTargets.RED_HUB;
        double dist = m_drivetrain.getDistanceTo(targetPosition);
        return dist;

        /*double distance = -1;
        if (!LimelightHelpers.getTV("limelight-hub")) {
            return distance; // Invalid distance
        }
        double offsetAngleVertical = LimelightHelpers.getTY("limelight-hub");
        double angleToGoal = Math.toRadians(Constants.ShooterConstants.LIMELIGHT_ANGLE + offsetAngleVertical);

        distance = (Constants.FieldConstants.HUB_APRILTAG_HEIGHT - Constants.ShooterConstants.LIMELIGHT_HEIGHT)
                / Math.tan(angleToGoal);

        return Math.abs(distance);*/
    }

    public double getFlywheelSpeed() {
        return (bottomLeftShooterMotor.getVelocity().getValueAsDouble() + bottomRightShooterMotor.getVelocity().getValueAsDouble() + topLeftShooterMotor.getVelocity().getValueAsDouble() + topRightShooterMotor.getVelocity().getValueAsDouble())
                / 4 * 60.0;
    }

    public double getTargetVelocity() {
        return targetSpeed;
    }

    public boolean atTargetSpeed() {
        return Math.abs(targetSpeed - getFlywheelSpeed()) <= Constants.ShooterConstants.FLYWHEEL_RPM_ACCEPTABLE_ERROR;
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
        SmartDashboard.putBoolean("Shooter/AtSpeed", atTargetSpeed());

        tuningMode = SmartDashboard.getBoolean("Shooter/TuningMode", false);
        SmartDashboard.putBoolean("Shooter/TuningMode", tuningMode);

        SmartDashboard.putNumber("Shooter velocity: ", getFlywheelSpeed());
        SmartDashboard.putNumber("Shooter supposed velocity: ", getTargetVelocity());



        // if (tuningMode) {
        //     tunedRPM = SmartDashboard.getNumber("Shooter/TunedRPM", tunedRPM);
        //     SmartDashboard.putNumber("Shooter/TunedRPM", tunedRPM);
        //     targetSpeed = tunedRPM;
        // }
    }
}
