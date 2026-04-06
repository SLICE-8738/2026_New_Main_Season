package frc.robot.commands.drive;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/**
 * Aligns the robot heading toward a field target and shoots when ready.
 *
 * Translation is still driver-controlled via the left stick.
 * Rotation is overridden by a heading PID to face the target.
 *
 */
public class AutoAlign extends Command {

    public enum Target {
        HUB, PASS_LEFT, PASS_RIGHT
    }


    private final CommandSwerveDrivetrain m_drivetrain;
    private final XboxController m_driverController;
    private final Target m_target;

    private Translation2d targetPosition;

    private ShuffleboardTab driverTab;

    private Pigeon2 m_Pigeon2;

    // Field-centric request: driver controls X/Y, heading PID supplies rotation
    private final SwerveRequest.FieldCentric driveRequest = new SwerveRequest.FieldCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);

    public AutoAlign(CommandSwerveDrivetrain drivetrain,
            Target target, XboxController driverController) {
        m_drivetrain = drivetrain;
        m_target = target;
        m_driverController = driverController;
        m_Pigeon2 = new Pigeon2(Constants.DriveConstants.GYRO_ID);

        driverTab = Shuffleboard.getTab("Driver");


        addRequirements(m_drivetrain);
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

    }

    @Override
    public void execute() {

        // Driver controls translation, heading PID controls rotation
        double headingCorrection = m_drivetrain.getHeadingPIDOutput(targetPosition);
        
        m_drivetrain.setControl(driveRequest
                .withVelocityX(m_driverController.getLeftY() * MaxSpeed)
                .withVelocityY(m_driverController.getLeftX() * MaxSpeed)
                .withRotationalRate(headingCorrection));
        
        LimelightHelpers.SetRobotOrientation("limelight-trench", m_Pigeon2.getYaw().getValueAsDouble(), 0.0, 0.0, 0.0, 0.0, 0.0);
        var limelightPose = LimelightHelpers.getBotPoseEstimate_wpiRed_MegaTag2("limelight-hub"); //TODO figure this out
        if (limelightPose != null && limelightPose.tagCount > 0 ) {
            m_drivetrain.addVisionMeasurement(limelightPose.pose, limelightPose.timestampSeconds);
        }

        /*
        if (DriverStation.getAlliance().get().equals(Alliance.Red)) {
            var limelightPose = LimelightHelpers.getBotPoseEstimate_wpiRed("limelight-hub"); //TODO figure this out
            if (limelightPose != null && limelightPose.tagCount > 0 ) {
                m_drivetrain.addVisionMeasurement(limelightPose.pose, limelightPose.timestampSeconds);
            }
    
        } else if (DriverStation.getAlliance().get().equals(Alliance.Blue)) {
            var limelightPose = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-hub");
            if (limelightPose != null && limelightPose.tagCount > 0) {
                m_drivetrain.addVisionMeasurement(limelightPose.pose, limelightPose.timestampSeconds);
            }   
        */
       }




    @Override
    public void end(boolean interrupted) {

        LimelightHelpers.SetRobotOrientation("limelight-trench", m_Pigeon2.getYaw().getValueAsDouble(), 0.0, 0.0, 0.0, 0.0, 0.0);
        var limelightPose = LimelightHelpers.getBotPoseEstimate_wpiRed_MegaTag2("limelight-hub"); //TODO figure this out
        if (limelightPose != null && limelightPose.tagCount > 0 ) {
             m_drivetrain.addVisionMeasurement(limelightPose.pose, limelightPose.timestampSeconds);
        }
        /*
        if (DriverStation.getAlliance().get().equals(Alliance.Red)) {
            LimelightHelpers.SetRobotOrientation("limelight-trench", m_Pigeon2.getYaw().getValueAsDouble(), 0.0, 0.0, 0.0, 0.0, 0.0);
            var limelightPose = LimelightHelpers.getBotPoseEstimate_wpiRed_MegaTag2("limelight-hub"); //TODO figure this out
            if (limelightPose != null && limelightPose.tagCount > 0 ) {
                m_drivetrain.addVisionMeasurement(limelightPose.pose, limelightPose.timestampSeconds);
            }
    
        } else if (DriverStation.getAlliance().get().equals(Alliance.Blue)) {
            LimelightHelpers.SetRobotOrientation("limelight-trench", m_Pigeon2.getYaw().getValueAsDouble(), 0.0, 0.0, 0.0, 0.0, 0.0);
            var limelightPose = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-hub");
            if (limelightPose != null && limelightPose.tagCount > 0) {
                m_drivetrain.addVisionMeasurement(limelightPose.pose, limelightPose.timestampSeconds);
            }   
            
       }
        */

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
