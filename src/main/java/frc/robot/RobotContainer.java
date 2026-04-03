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
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.drive.AutoAlign;
import frc.robot.commands.indexer.SpinStageOne;
import frc.robot.commands.indexer.SpinStageTwo;
import frc.robot.commands.intake.OscillateIntake;
import frc.robot.commands.intake.ExtendIntake;
import frc.robot.commands.intake.RetractIntake;
import frc.robot.commands.intake.Spintake;
import frc.robot.commands.intake.Stoptake;
import frc.robot.commands.intake.TestIntake;
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
    public final CommandSwerveDrivetrain m_drivetrain;

    public final Intake m_Intake;
    public final Indexer m_Indexer;
    public final Shooter m_Shooter;

    // ==========================
    // Commands
    // ==========================

    /* Driving */
    public final AutoAlign m_AutoAlignHub;

    /* Intake */
    public final ExtendIntake m_ExtendIntake;
    public final RetractIntake m_RetractIntake;
    public final Spintake m_Spintake;
    public final Stoptake m_Stoptake;
    public final OscillateIntake m_OscillateIntake;
    public final ExtendIntake m_IntakeCommand;

    public final TestIntake m_TestExtendIntake;
    public final TestIntake m_TestRetractIntake;

    /* Indexer */
    public final SpinStageOne m_spinStageOne;
    public final SpinStageTwo m_spinStageTwo;
    public final SpinStageOne m_stopStageOne;

    /* Shooter */
    
    //Probably not going to be utilizing Jimits AlignAndShoot command from here on out
    public final AlignAndShoot m_alignAndShootHub;
    //public final AlignAndShoot m_alignAndPassLeft;
    //public final AlignAndShoot m_alignAndPassRight;
    // */
    public final Shoot m_shoot;
    public final BasicShoot m_BasicShootHub;

    // Auto chooser
    // TODO figure the heckin pathplanner code
    private final SendableChooser<Command> autoChooser;

    /* Triggers */
    private Trigger driveIntakeTrigger;
    private Trigger oscillateTrigger;
    private Trigger indexerTrigger;

    /* Other Commands */
    //private Command autoCommand;

    // =====================
    // Generated Swerve Drivetrain Stuff
    // =====================

    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    
    private ShuffleboardData m_ShuffleboardData;
    
    public RobotContainer() {

        // ==========================
        // Subsystems
        // ==========================
        m_drivetrain = TunerConstants.createDrivetrain();
        m_Intake = new Intake();
        m_Indexer = new Indexer();
        m_Shooter = new Shooter(m_drivetrain);
        
        

        // ==========================
        // Commands
        // ==========================

        /* Drive */
        m_AutoAlignHub = new AutoAlign(m_drivetrain, AutoAlign.Target.HUB, driverController);

        /* Intake */
        m_ExtendIntake    = new ExtendIntake(m_Intake);
        m_RetractIntake   = new RetractIntake(m_Intake);
        m_Spintake        = new Spintake(m_Intake);
        m_OscillateIntake = new OscillateIntake(m_Intake);
        m_Stoptake        = new Stoptake(m_Intake);
        // Removed the conditional command because it is not working properly, and is overriding manual controls
        m_IntakeCommand   = new ExtendIntake(m_Intake);//new ConditionalCommand(m_ExtendIntake.andThen(m_Spintake), m_Stoptake, () -> (m_Intake.isStowed() == true));

        m_TestExtendIntake = new TestIntake(m_Intake, 0.05);
        m_TestRetractIntake = new TestIntake(m_Intake, -0.05);

        /* Indexer */
        m_spinStageOne = new SpinStageOne(m_Indexer, 1);
        m_spinStageTwo = new SpinStageTwo(m_Indexer, 1);
        m_stopStageOne = new SpinStageOne(m_Indexer, 0);

        /* Shooter */
        m_shoot = new Shoot(m_Shooter, m_drivetrain);
        m_BasicShootHub = new BasicShoot(m_Shooter);
        
        //* Probably not going to be utilizing Jimits AlignAndShoot command from here on out
        
        m_alignAndShootHub = new AlignAndShoot(m_Shooter, m_Indexer, m_drivetrain, AlignAndShoot.Target.HUB, driverController);
        //m_alignAndPassLeft = new AlignAndShoot(m_Shooter, m_Indexer, m_drivetrain, AlignAndShoot.Target.PASS_LEFT, driverController);
        //m_alignAndPassRight = new AlignAndShoot(m_Shooter, m_Indexer, m_drivetrain, AlignAndShoot.Target.PASS_RIGHT, driverController);
        // */

        /* Autonomous */

        /*
         * autoCommand = new ParallelCommandGroup(new BasicShoot(m_Shooter), 
         *    new SequentialCommandGroup(new WaitCommand(6.7),
         *         new ParallelCommandGroup(new SpinStageOne(m_Indexer, Constants.IndexerConstants.STAGE_ONE_INTAKE_SPEED), new SpinStageTwo(m_Indexer, Constants.IndexerConstants.STAGE_TWO_INTAKE_SPEED))));
        */

        autoChooser = AutoBuilder.buildAutoChooser("Left Auto Trench");
        SmartDashboard.putData("Auto Mode", autoChooser);

        /*
        TODO more auto stuff to figure the heck out of
        */
        NamedCommands.registerCommand("Intake", m_IntakeCommand);
        NamedCommands.registerCommand("Spin Stage One", m_spinStageOne);
        NamedCommands.registerCommand("Spin Stage Two", m_spinStageTwo);
        NamedCommands.registerCommand("Stop Intake", m_Stoptake);
        NamedCommands.registerCommand("Extend Intake", m_ExtendIntake);
        //NamedCommands.registerCommand("Retract Intake", m_RetractIntake);
        NamedCommands.registerCommand("Shoot", m_shoot);
        

        //autoChooser = AutoBuilder.buildAutoChooser("Left Auto Trench");

        /* Triggers */
        driveIntakeTrigger = new Trigger(() -> m_Intake.getCurrentCommand() != null);

        oscillateTrigger = new Trigger(() -> m_Indexer.getCurrentCommand() != null);
        // indexerTrigger = new Trigger(() -> m_Shooter.atTargetSpeed()); This trigger currently does not work; there is some
        // sort of error where the atTargetSpeed() method isn't returning the proper values
        indexerTrigger = new Trigger(() -> m_Shooter.getCurrentCommand() != null);



        CommandScheduler.getInstance().schedule(FollowPathCommand.warmupCommand());

        m_ShuffleboardData = new ShuffleboardData(m_drivetrain, m_Intake, m_Shooter);

        configureBindings();
        
    }

    private void configureBindings() {


        // ================
        // Driver Controls
        // ================

        /* Drivetrain */
        Buttons.controller1_leftBumper.whileTrue(m_AutoAlignHub);

        // Reset the field-centric heading on minus press.
        Buttons.controller1_minusButton.onTrue(m_drivetrain.runOnce(m_drivetrain::seedFieldCentric));

        /* Shooter */

        
        //TODO please please please fix the shooter so we can uncomment this code, Harrissh
        Buttons.controller1_RightTrigger.whileTrue(m_shoot.alongWith(new SequentialCommandGroup(new WaitCommand(2.5), new SpinStageTwo(m_Indexer, Constants.IndexerConstants.STAGE_ONE_INTAKE_SPEED))));//m_BasicShootHub);
        //Buttons.controller1_leftBumper.whileTrue(m_alignAndPassLeft);
        //Buttons.controller1_rightBumper.whileTrue(m_alignAndPassRight);
        //Buttons.controller1_RightTrigger.whileTrue(m_alignAndShootHub);
        
        //Buttons.controller1_RightTrigger.whileTrue(m_BasicShootHub.alongWith(new SpinStageOne(m_Indexer, Constants.IndexerConstants.STAGE_ONE_INTAKE_SPEED)));

        
        /* Intake */
        
        Buttons.controller1_LeftTrigger.onTrue(m_ExtendIntake.alongWith(new SpinStageOne(m_Indexer, Constants.IndexerConstants.STAGE_ONE_INTAKE_PASSIVE_SPEED)));
        Buttons.controller1_AButton.onTrue(new ParallelRaceGroup(m_RetractIntake, new SpinStageOne(m_Indexer, Constants.IndexerConstants.STAGE_ONE_INTAKE_SPEED)));
        
        /* Indexer */

        Buttons.controller1_YButton.whileTrue(new SpinStageOne(m_Indexer, Constants.IndexerConstants.STAGE_ONE_INTAKE_SPEED));
        Buttons.controller1_BButton.whileTrue(new SpinStageTwo(m_Indexer, Constants.IndexerConstants.STAGE_ONE_INTAKE_SPEED));
        
        Buttons.controller1_povUp.whileTrue(m_TestExtendIntake);
        Buttons.controller1_povDown.whileTrue(m_TestRetractIntake);


        // ============
        // Other Triggers
        // ============


        // TODO fix and uncomment after testing
        //oscillateTrigger.whileTrue(m_OscillateIntake);
        // TODO fix and uncomment after testing
        /*
        indexerTrigger.whileTrue(new WaitCommand(2).
            andThen(new SpinStageOne(m_Indexer, Constants.IndexerConstants.STAGE_ONE_INTAKE_SPEED).
                alongWith(new SpinStageTwo(m_Indexer, Constants.IndexerConstants.STAGE_TWO_INTAKE_SPEED))));
        */

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
        // TODO THIS IS FOR SYSID TESTING

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

        
        driveIntakeTrigger.whileTrue(m_drivetrain.applyRequest(() ->
                drive.withVelocityX(driverController.getRawAxis(1) * Constants.DriveConstants.MAX_INTAKE_LINEAR_VELOCITY) // Drive forward with negative Y (forward) // Left Y
                     .withVelocityY(driverController.getRawAxis(0) * Constants.DriveConstants.MAX_INTAKE_LINEAR_VELOCITY) // Drive left with negative X (left) // Left X
                     .withRotationalRate(-driverController.getRawAxis(4) * MaxAngularRate) // Drive counterclockwise with negative X (left) // Right X
            ));
        
        
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
        //return autoCommand;
    }
}
