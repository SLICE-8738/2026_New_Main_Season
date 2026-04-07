package frc.robot.commands.shooter;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;

/**
 * Aligns the robot heading toward a field target and shoots when ready.
 *
 * <p>Translation is still driver-controlled via the left stick.
 * Rotation is overridden by a heading PID to face the target.
 *
 */
public class AlignAndShoot extends Command {

    public enum Target {
        HUB, PASS_LEFT, PASS_RIGHT
    }

    private final Shooter m_shooter;
    private final Indexer m_indexer;
    private final CommandSwerveDrivetrain m_drivetrain;
    private final XboxController m_driverController;
    private final Target m_target;

    private Translation2d targetPosition;

    private ShuffleboardTab driverTab;
    private SimpleWidget currentTargetX;
    private SimpleWidget currentTargetY;

    // Field-centric request: driver controls X/Y, heading PID supplies rotation
    private final SwerveRequest.FieldCentric driveRequest = new SwerveRequest.FieldCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);

    public AlignAndShoot(Shooter shooter, Indexer indexer, CommandSwerveDrivetrain drivetrain,
            Target target, XboxController driverController) {
        m_shooter = shooter;
        m_indexer = indexer;
        m_drivetrain = drivetrain;
        m_target = target;
        m_driverController = driverController;

        driverTab = Shuffleboard.getTab("Driver");

        currentTargetX = driverTab.add("Target X Position", 0);
        currentTargetY = driverTab.add("Target Y Position", 0);

        addRequirements(/*m_shooter,*/ m_drivetrain);
    }

    @Override
    public void initialize() {
        boolean isBlue = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue;
        targetPosition = switch (m_target) {
            case HUB -> isBlue ? Constants.AlignTargets.BLUE_HUB : Constants.AlignTargets.RED_HUB;
            case PASS_LEFT -> isBlue ? Constants.AlignTargets.BLUE_PASS_LEFT : Constants.AlignTargets.RED_PASS_LEFT;
            case PASS_RIGHT -> isBlue ? Constants.AlignTargets.BLUE_PASS_RIGHT : Constants.AlignTargets.RED_PASS_RIGHT;
        };

        double dist = m_drivetrain.getDistanceTo(targetPosition);
        //m_shooter.calculateShot(dist, m_shooter.getHorizontalVelocity(dist, targetPosition));

    }

    @Override
    public void execute() {
        //Translation2d compensated = m_drivetrain.getCompensatedTarget(targetPosition);
        //double dist = m_drivetrain.getDistanceTo(compensated);

        /* 
        if (!m_shooter.isTuningMode()) {
            m_shooter.calculateShot(dist, m_shooter.getHorizontalVelocity(dist, compensated));
        }
        */

        currentTargetX.getEntry().setDouble(targetPosition.getX());
        currentTargetY.getEntry().setDouble(targetPosition.getY());


        // Driver controls translation, heading PID controls rotation
        double headingCorrection = m_drivetrain.getHeadingPIDOutput(targetPosition);
        
        m_drivetrain.setControl(driveRequest
                .withVelocityX(m_driverController.getLeftY() * MaxSpeed * 0.6)
                .withVelocityY(m_driverController.getLeftX() * MaxSpeed * 0.6)
                .withRotationalRate(headingCorrection));
        
        /*if (DriverStation.getAlliance().get().equals(Alliance.Red)) {
            var limelightPose = LimelightHelpers.getBotPoseEstimate_wpiRed("limelight-shooter"); //TODO figure this ou
            if (limelightPose != null && limelightPose.tagCount > 0 ) {
                m_drivetrain.addVisionMeasurement(limelightPose.pose, limelightPose.timestampSeconds);
            }
    
        } else if (DriverStation.getAlliance().get().equals(Alliance.Blue)) {
            var limelightPose = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-shooter");
            if (limelightPose != null && limelightPose.tagCount > 0) {
                m_drivetrain.addVisionMeasurement(limelightPose.pose, limelightPose.timestampSeconds);
            }   
            
       }*/
      

        //m_shooter.spinFlywheels(m_shooter.getTargetVelocity());

        /* 
        if (m_drivetrain.atTargetHeading() && m_shooter.atTargetSpeed() && m_shooter.atTargetPosition()) {
            m_indexer.runStageOneMotor(Constants.IndexerConstants.STAGE_ONE_INTAKE_SPEED);
            m_indexer.runStageTwoMotor(Constants.IndexerConstants.STAGE_TWO_INTAKE_SPEED);
        } else {
            m_indexer.stopAll();
        }
        */
    }

    @Override
    public void end(boolean interrupted) {

        if (DriverStation.getAlliance().get().equals(Alliance.Red)) {
            var limelightPose = LimelightHelpers.getBotPoseEstimate_wpiRed("limelight-shooter"); //TODO figure this ou
            if (limelightPose != null && limelightPose.tagCount > 0 ) {
                m_drivetrain.addVisionMeasurement(limelightPose.pose, limelightPose.timestampSeconds);
            }
    
        } else if (DriverStation.getAlliance().get().equals(Alliance.Blue)) {
            var limelightPose = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-shooter");
            if (limelightPose != null && limelightPose.tagCount > 0) {
                m_drivetrain.addVisionMeasurement(limelightPose.pose, limelightPose.timestampSeconds);
            }   
            
       }

        //m_shooter.windDownFlywheels();
        //m_indexer.stopAll();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
