// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.HashMap;
import java.util.concurrent.locks.AbstractQueuedLongSynchronizer.ConditionObject;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.indexer.SpinStageOne;
import frc.robot.commands.indexer.SpinStageTwo;

import frc.robot.commands.intake.OscillateIntake;
import frc.robot.commands.intake.ExtendIntake;
import frc.robot.commands.intake.RetractIntake;
import frc.robot.commands.intake.Spintake;
import frc.robot.commands.intake.Stoptake;

import frc.robot.commands.shooter.AlignAndShoot;
import frc.robot.commands.shooter.BasicShoot;
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
    public final ExtendIntake m_ExtendIntake;
    public final RetractIntake m_RetractIntake;
    public final Spintake m_Spintake;
    public final Stoptake m_Stoptake;
    public final OscillateIntake m_OscillateIntake;
    public final ExtendIntake m_IntakeCommand;

    /* Indexer */
    public final SpinStageOne m_spinStageOne;
    public final SpinStageTwo m_spinStageTwo;

    /* Shooter */
    public final AlignAndShoot m_alignAndShootHub;
    public final AlignAndShoot m_alignAndPassLeft;
    public final AlignAndShoot m_alignAndPassRight;
    public final Shoot m_shoot;
    public final BasicShoot m_BasicShootHub;

    // Auto chooser
    // TODO figure the heckin pathplanner code
  //  private final SendableChooser<Command> autoChooser;

    /* Triggers */
    private Trigger oscillateTrigger;
    private Trigger indexerTrigger;

    private Command autoCommand;
    //private Command lynkDESTROYER;
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
        //m_drivetrain = TunerConstants.createDrivetrain();
        m_Intake = new Intake();
        m_Indexer = new Indexer();
        m_Shooter = new Shooter(m_drivetrain);

       /*
       autoChooser = AutoBuilder.buildAutoChooser("Left Auto Trench");
       SmartDashboard.putData("Auto Mode", autoChooser);
       */
        

        // ==========================
        // Commands
        // ==========================

        /* Intake */
        m_ExtendIntake    = new ExtendIntake(m_Intake);
        m_RetractIntake   = new RetractIntake(m_Intake);
        m_Spintake        = new Spintake(m_Intake);
        m_OscillateIntake = new OscillateIntake(m_Intake);
        m_Stoptake        = new Stoptake(m_Intake);
        m_IntakeCommand   = new ExtendIntake(m_Intake);//new ConditionalCommand(m_ExtendIntake.andThen(m_Spintake), m_Stoptake, () -> (m_Intake.isStowed() == true));

        /* Indexer */
        m_spinStageOne = new SpinStageOne(m_Indexer, 1);
        m_spinStageTwo = new SpinStageTwo(m_Indexer, 1);

        /* Shooter */
        m_shoot = new Shoot(m_Shooter, m_drivetrain);
        m_BasicShootHub = new BasicShoot(m_Shooter);
        m_alignAndShootHub = new AlignAndShoot(m_Shooter, m_Indexer, m_drivetrain, AlignAndShoot.Target.HUB, driverController);
        m_alignAndPassLeft = new AlignAndShoot(m_Shooter, m_Indexer, m_drivetrain, AlignAndShoot.Target.PASS_LEFT, driverController);
        m_alignAndPassRight = new AlignAndShoot(m_Shooter, m_Indexer, m_drivetrain, AlignAndShoot.Target.PASS_RIGHT, driverController);

        /* Triggers */
        oscillateTrigger = new Trigger(() -> m_Indexer.getCurrentCommand() != null);
        //indexerTrigger = new Trigger(() -> m_Shooter.atTargetSpeed());
        indexerTrigger = new Trigger(() -> m_Shooter.getCurrentCommand() != null);

        NamedCommands.registerCommand("Spin Intake", m_Spintake);
        NamedCommands.registerCommand("Stop Intake", m_Stoptake);
        NamedCommands.registerCommand("Extend Intake", m_ExtendIntake);
        //NamedCommands.registerCommand("Retract Intake", m_RetractIntake);
        NamedCommands.registerCommand("Align & Shoot", m_alignAndShootHub);
        

        //autoChooser = AutoBuilder.buildAutoChooser("Left Auto Trench");
        //SmartDashboard.putData("Auto Mode", autoChooser);

        autoCommand = new ParallelCommandGroup(new BasicShoot(m_Shooter), 
            new SequentialCommandGroup(new WaitCommand(6.7),
                new ParallelCommandGroup(new SpinStageOne(m_Indexer, Constants.IndexerConstants.STAGE_ONE_INTAKE_SPEED), new SpinStageTwo(m_Indexer, Constants.IndexerConstants.STAGE_TWO_INTAKE_SPEED))));

        /*
        lynkDESTROYER = new Command() {
            
        };
        */

        configureBindings();
        
        //CommandScheduler.getInstance().schedule(FollowPathCommand.warmupCommand());
    }

    private void configureBindings() {

        // ================
        // Driver Controls
        // ================

        /* Drivetrain */

        // Reset the field-centric heading on minus press.
        Buttons.controller1_minusButton.onTrue(m_drivetrain.runOnce(m_drivetrain::seedFieldCentric));

        /* Shooter */

        //Buttons.controller1_XButton.whileTrue(m_shoot);
        //Buttons.controller1_YButton.whileTrue(m_spinStageTwo); //TODO idek whats going on with these indexers bro; probably shooter
        Buttons.controller1_YButton.whileTrue(new SpinStageOne(m_Indexer, Constants.IndexerConstants.STAGE_ONE_INTAKE_SPEED).
            alongWith(new SpinStageTwo(m_Indexer, Constants.IndexerConstants.STAGE_TWO_INTAKE_SPEED)));
        Buttons.controller1_RightTrigger.whileTrue(m_shoot);//m_BasicShootHub);
        Buttons.controller1_leftBumper.whileTrue(m_alignAndPassLeft);
        Buttons.controller1_rightBumper.whileTrue(m_alignAndPassRight);

        //Buttons.controller1_XButton.whileTrue(m_BasicShootHub);

        /* Intake */
        
        Buttons.controller1_LeftTrigger.onTrue(m_IntakeCommand.alongWith(new SpinStageOne(m_Indexer, Constants.IndexerConstants.STAGE_ONE_INTAKE_PASSIVE_SPEED)));
        Buttons.controller1_AButton.onTrue(m_RetractIntake);

        // ============
        // Other Triggers
        // ============
        // TODO fix and uncomment after testing
        //oscillateTrigger.whileTrue(m_OscillateIntake);
        // TODO fix and uncomment after testing
        //indexerTrigger.whileTrue(m_spinStageOne.alongWith(m_spinStageTwo));
        indexerTrigger.whileTrue(new WaitCommand(2).
            andThen(new SpinStageOne(m_Indexer, Constants.IndexerConstants.STAGE_ONE_INTAKE_SPEED).
                alongWith(new SpinStageTwo(m_Indexer, Constants.IndexerConstants.STAGE_TWO_INTAKE_SPEED))));

        // =====================
        // Generated Swerve Drivetrain Stuff
        // =====================

        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        m_drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            m_drivetrain.applyRequest(() ->
                drive.withVelocityX(driverController.getRawAxis(1) * MaxSpeed) // Drive forward with negative Y (forward) // Left Y
                    .withVelocityY(driverController.getRawAxis(0) * MaxSpeed) // Drive left with negative X (left) // Left X
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
        //return autoChooser.getSelected();
        return autoCommand;
    }
}
