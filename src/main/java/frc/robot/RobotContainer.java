// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.indexer.SpinStageOne;
import frc.robot.commands.indexer.SpinStageTwo;
import frc.robot.commands.intake.OscillateIntake;
import frc.robot.commands.intake.ToggleIntake;
import frc.robot.commands.shooter.AlignAndShoot;
import frc.robot.commands.shooter.Shoot;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class RobotContainer {

    private final XboxController driverController = Buttons.controller1;

    // ==========================
    // Subsystems
    // ==========================

    public final Intake m_Intake;
    public final Indexer m_Indexer;
    public final Shooter m_Shooter;

    // ==========================
    // Commands
    // ==========================

    /* Intake */
    public final ToggleIntake m_ToggleIntake;
    public final OscillateIntake m_OscillateIntake;

    /* Indexer */
    public final SpinStageOne m_spinStageOne;
    public final SpinStageTwo m_spinStageTwo;

    /* Shooter */
    public final AlignAndShoot m_alignAndShootHub;
    public final AlignAndShoot m_alignAndPassLeft;
    public final AlignAndShoot m_alignAndPassRight;

    // Auto chooser
    private final SendableChooser<Command> autoChooser;

    /* Triggers */
    private Trigger oscillateTrigger;
    private Trigger indexerTrigger;

    // =====================
    // Generated Swerve Drivetrain Stuff
    // =====================

    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    
    public final CommandSwerveDrivetrain m_drivetrain = TunerConstants.createDrivetrain();

    public RobotContainer() {
        // ==========================
        // Subsystems
        // ==========================
        m_Intake = new Intake();
        m_Indexer = new Indexer();
        m_Shooter = new Shooter(m_drivetrain);

        autoChooser = AutoBuilder.buildAutoChooser();

        // ==========================
        // Commands
        // ==========================

        /* Intake */
        m_ToggleIntake    = new ToggleIntake(m_Intake, m_Indexer);
        m_OscillateIntake = new OscillateIntake(m_Intake);

        /* Indexer */
        m_spinStageOne = new SpinStageOne(m_Indexer, 1);
        m_spinStageTwo = new SpinStageTwo(m_Indexer, 1);

        /* Shooter */
        m_alignAndShootHub = new AlignAndShoot(m_Shooter, m_Indexer, m_drivetrain, AlignAndShoot.Target.HUB, driverController);
        m_alignAndPassLeft = new AlignAndShoot(m_Shooter, m_Indexer, m_drivetrain, AlignAndShoot.Target.PASS_LEFT, driverController);
        m_alignAndPassRight = new AlignAndShoot(m_Shooter, m_Indexer, m_drivetrain, AlignAndShoot.Target.PASS_RIGHT, driverController);

        /* Triggers */
        oscillateTrigger = new Trigger(() -> m_Indexer.getCurrentCommand() != null);
        indexerTrigger = new Trigger(() -> m_Shooter.atTargetSpeed());

        configureBindings();
    }

    private void configureBindings() {

        // ================
        // Driver Controls
        // ================

        /* Drivetrain */

        // Reset the field-centric heading on left bumper press.
        Buttons.controller1_minusButton.onTrue(m_drivetrain.runOnce(m_drivetrain::seedFieldCentric));

        /* Shooter */
        Buttons.controller1_RightTrigger.whileTrue(m_alignAndShootHub);

        /* Intake */
        Buttons.controller1_leftBumper.whileTrue(m_ToggleIntake);
        //Buttons.controller1_RightTrigger.whileTrue(m_OscillateIntake);

        // ============
        // Other Triggers
        // ============
        oscillateTrigger.whileTrue(m_OscillateIntake);
        indexerTrigger.whileTrue(m_spinStageOne.alongWith(m_spinStageTwo));


        // =====================
        // Generated Swerve Drivetrain Stuff
        // =====================

        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        m_drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            m_drivetrain.applyRequest(() ->
                drive.withVelocityX(-driverController.getRawAxis(1) * MaxSpeed) // Drive forward with negative Y (forward) // Left Y
                    .withVelocityY(-driverController.getRawAxis(0) * MaxSpeed) // Drive left with negative X (left) // Left X
                    .withRotationalRate(-driverController.getRawAxis(4) * MaxAngularRate) // Drive counterclockwise with negative X (left) // Right X
            )
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            m_drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );


        /* 
         * THIS IS FOR SYSID TESTING

        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));
        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
        // Reset the field-centric heading on left bumper press.
        joystick.back().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));
        drivetrain.registerTelemetry(logger::telemeterize);

        */

        
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
