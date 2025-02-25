// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.ClimbConstants;
import frc.robot.Constants.Elevator;
import frc.robot.Constants.LimelightConstants;
import frc.robot.commands.swervedrive.auto.AutoAimToReef;
import frc.robot.commands.swervedrive.auto.TurnToAprilTagCommand;

//for old climb system
import frc.robot.subsystems.climber.ClimbSubsystem;

//modern subsystems
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.Elevator.ElevatorSubsystem;
import frc.robot.subsystems.Elevator.Algae.IntakeState;
import frc.robot.subsystems.Elevator.Coral;
import frc.robot.subsystems.Elevator.Algae;

import frc.robot.subsystems.LEDS.LEDs;

import frc.robot.utils.utils;

import java.io.File;
import edu.wpi.first.wpilibj2.command.Commands;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.swervedrive.auto.TurnToAprilTagCommand;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.utils.Elastic;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import com.pathplanner.lib.auto.NamedCommands;
import swervelib.SwerveInputStream;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

import java.util.List;



/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final SwerveSubsystem m_robotDrive = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
  "swerve"));
  //private final LimelightSubsystem m_robotLimelight = new LimelightSubsystem("limelight"); //will delete
  private final ElevatorSubsystem m_ElevatorSubsystem = new ElevatorSubsystem();
  private final Coral m_CoralSubsystem = new Coral();
  private final Algae m_AlgaeSubsystem = new Algae();
  private final ClimbSubsystem m_ClimbSubsystem = new ClimbSubsystem();

  private final LEDs m_LEDs = new LEDs();
    
  SendableChooser<Command> m_chooser = new SendableChooser<>();
  
  // The driver's controller
  CommandXboxController m_driverController0 = new CommandXboxController(OIConstants.kDriverControllerPort0);
  CommandXboxController m_driverController1 = new CommandXboxController(OIConstants.kDriverControllerPort1);
    
  double driveSpeedElevatorControl = (m_ElevatorSubsystem.getPosition())/Elevator.kMaxHeight;
  double translationSpeedCoef = 0.7;
  double rotationSpeedCoef = 0.6;

  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(m_robotDrive.getSwerveDrive(),
    //Y direction
    () -> -MathUtil.applyDeadband(m_driverController0.getLeftY(), OIConstants.LEFT_Y_DEADBAND_1)
    *(OIConstants.kDefaultDriveSpeed+
    OIConstants.kDriveSpeedIncreaseConstant*
    MathUtil.applyDeadband(m_driverController0.getRightTriggerAxis(), OIConstants.RIGHT_TRIGGER_DEADBAND_1))
    *(1-Math.abs(translationSpeedCoef*m_ElevatorSubsystem.getPosition())/Elevator.kMaxHeight),

    //X direction
    () -> -MathUtil.applyDeadband(m_driverController0.getLeftX(), OIConstants.LEFT_X_DEADBAND_1)
    *(OIConstants.kDefaultDriveSpeed+
    OIConstants.kDriveSpeedIncreaseConstant*
    MathUtil.applyDeadband(m_driverController0.getRightTriggerAxis(), OIConstants.RIGHT_TRIGGER_DEADBAND_1))
    *(1-Math.abs(translationSpeedCoef*m_ElevatorSubsystem.getPosition())/Elevator.kMaxHeight))
    //rot
    .withControllerRotationAxis(() -> 
      -MathUtil.applyDeadband(m_driverController0.getRightX(), OIConstants.RIGHT_X_DEADBAND_1)
      *(OIConstants.kDefaultDriveSpeed+
      OIConstants.kDriveSpeedIncreaseConstant*
      MathUtil.applyDeadband(m_driverController0.getRightTriggerAxis(), OIConstants.RIGHT_TRIGGER_DEADBAND_1))
      *(1-Math.abs(rotationSpeedCoef*m_ElevatorSubsystem.getPosition())/Elevator.kMaxHeight))//*(1-driveSpeedElevatorControl*rotationSpeedCoef))
    .deadband(0.1)
    .scaleTranslation(
      0.8)
    .allianceRelativeControl(true);
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    
    configureButtonBindings();

    m_LEDs.setColor(255, 0, 0, 0, 0, 60);
    
    // Configure pathlanner Commands
    NamedCommands.registerCommand("L1", m_ElevatorSubsystem.goToElevatorStowCommand()
      .andThen(m_CoralSubsystem.scoreL1Command()));
    NamedCommands.registerCommand("L2", m_ElevatorSubsystem.goToElevatorL2Command()
      .andThen(m_CoralSubsystem.scoreL24Command())
      .andThen(m_ElevatorSubsystem.goToElevatorStowCommand()));
    NamedCommands.registerCommand("L3", m_ElevatorSubsystem.goToElevatorL3Command()
      .andThen(m_CoralSubsystem.scoreL24Command())
      .andThen(m_ElevatorSubsystem.goToElevatorStowCommand()));
    NamedCommands.registerCommand("L4", m_ElevatorSubsystem.goToElevatorL4Command()
      .andThen(m_CoralSubsystem.scoreL24Command())
      .andThen(m_ElevatorSubsystem.goToElevatorStowCommand()));

    NamedCommands.registerCommand("Go to stow", m_ElevatorSubsystem.goToElevatorStowCommand());
    NamedCommands.registerCommand("Go to L2", m_ElevatorSubsystem.goToElevatorL2Command());
    NamedCommands.registerCommand("Go to L3", m_ElevatorSubsystem.goToElevatorL3Command());
    NamedCommands.registerCommand("Go to L4", m_ElevatorSubsystem.goToElevatorL4Command());

    NamedCommands.registerCommand("shoot L1", m_CoralSubsystem.scoreL1Command());
    NamedCommands.registerCommand("shoot L24", m_CoralSubsystem.scoreL24Command());
    NamedCommands.registerCommand("intake", m_CoralSubsystem.intakeCommand());
    
    SmartDashboard.putBoolean("Controler/Test", false);

    //End of Pathplanner named commands


    //Auto Chooser
    Shuffleboard.getTab("Important").add("auto chooser", m_chooser);
    m_chooser.setDefaultOption("do nothing", null);
    m_chooser.addOption("Center to Coral", new PathPlannerAuto("Center to Coral"));
    m_chooser.addOption("Center to H1", new PathPlannerAuto("Center to H1"));
    m_chooser.addOption("2m test", new PathPlannerAuto("2m test"));
    m_chooser.addOption("Start to Reef", new PathPlannerAuto("Start To Reef"));
    m_chooser.addOption("2m spin test", new PathPlannerAuto("2m spin test"));

    //Shuffleboard.getTab("Important").add("Path side")

  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    //when the right stick is pushed down, moves wheels in x formation to stop all movement
    //m_driverController0.x().whileTrue(Commands.run(() -> m_robotDrive.lock())); 
    m_driverController0.leftStick().whileTrue(Commands.run(() -> m_robotDrive.zeroGyro()));
    m_driverController0.rightStick().whileTrue(m_robotDrive.run(()->m_robotDrive.lock()));

    Command driveFieldOrientedAnglularVelocity = m_robotDrive.driveFieldOriented(driveAngularVelocity);
    
    m_robotDrive.setDefaultCommand(driveFieldOrientedAnglularVelocity);

    //contolloer 0 elevator commands

    m_driverController0.a().whileTrue(m_ElevatorSubsystem.goToElevatorStowCommand());
    m_driverController0.b().whileTrue(m_ElevatorSubsystem.goToElevatorL2Command());
    m_driverController0.x().whileTrue(m_ElevatorSubsystem.goToElevatorL3Command());
    m_driverController0.y().whileTrue(m_ElevatorSubsystem.goToElevatorL4Command());



    m_ClimbSubsystem.setDefaultCommand(
      Commands.run(()->m_ClimbSubsystem.setClimb(0.2*m_driverController1.getLeftY()), m_ClimbSubsystem));


    //Start of Coral Commands

    //TODO: ensure Algae is at stow for commands

    //L1
    m_driverController1.leftBumper().whileTrue(m_ElevatorSubsystem.goToElevatorStowCommand());
    //L2
    m_driverController1.x().whileTrue((new AutoAimToReef(m_robotDrive, false, 2))
      .andThen(m_ElevatorSubsystem.goToElevatorL2Command())
      .andThen(m_CoralSubsystem.scoreL24Command())
      .andThen(m_ElevatorSubsystem.goToElevatorStowCommand()));
    m_driverController1.a().whileTrue((new AutoAimToReef(m_robotDrive, true, 2))
      .andThen(m_ElevatorSubsystem.goToElevatorL2Command())
      .andThen(m_CoralSubsystem.scoreL24Command())
      .andThen(m_ElevatorSubsystem.goToElevatorStowCommand()));
      //L3
    m_driverController1.y().whileTrue((new AutoAimToReef(m_robotDrive, false, 3))
      .andThen(m_ElevatorSubsystem.goToElevatorL3Command())
      .andThen(m_CoralSubsystem.scoreL24Command())
      .andThen(m_ElevatorSubsystem.goToElevatorStowCommand()));
    m_driverController1.b().whileTrue((new AutoAimToReef(m_robotDrive, true, 3))
      .andThen(m_ElevatorSubsystem.goToElevatorL3Command())
      .andThen(m_CoralSubsystem.scoreL24Command())
      .andThen(m_ElevatorSubsystem.goToElevatorStowCommand()));
    //L4
    m_driverController1.rightBumper().whileTrue((new AutoAimToReef(m_robotDrive, false, 4))
      .andThen(m_ElevatorSubsystem.goToElevatorL4Command())
      .andThen(m_CoralSubsystem.scoreL24Command())
      .andThen(m_ElevatorSubsystem.goToElevatorStowCommand()));
    m_driverController1.rightTrigger().whileTrue((new AutoAimToReef(m_robotDrive, true, 4))
      .andThen(m_ElevatorSubsystem.goToElevatorL4Command())
      .andThen(m_CoralSubsystem.scoreL24Command())
      .andThen(m_ElevatorSubsystem.goToElevatorStowCommand()));

    //End of Coral Commands Level Commands
    

    //score/intake comands
    m_driverController1.leftTrigger().onTrue(m_CoralSubsystem.intakeCommand().unless(()->m_CoralSubsystem.hasCoral()));

    m_driverController1.back().whileTrue(m_CoralSubsystem.scoreL1Command());
    m_driverController1.start().onTrue(m_CoralSubsystem.scoreL24Command()
      .andThen(new WaitCommand(.25))
      .andThen(m_ElevatorSubsystem.goToElevatorStowCommand()));

    //End of Coral Commands

    //Algae manual controls
    m_driverController1.povDown().whileTrue(m_AlgaeSubsystem.run(()->m_AlgaeSubsystem.setWristMotor(0.5))
      .finallyDo(()->m_AlgaeSubsystem.setWristMotor(0)));
    m_driverController1.povUp().whileTrue(m_AlgaeSubsystem.run(()->m_AlgaeSubsystem.setWristMotor(-0.5))
      .finallyDo(()->m_AlgaeSubsystem.setWristMotor(0)));
    m_driverController1.povLeft().whileTrue(m_AlgaeSubsystem.run(()->m_AlgaeSubsystem.setIntakeMotor(-0.25))
      .finallyDo(()->m_AlgaeSubsystem.setIntakeMotor(0)));
    m_driverController1.povRight().whileTrue(m_AlgaeSubsystem.run(()->m_AlgaeSubsystem.setIntakeMotor(0.35))
      .finallyDo(()->m_AlgaeSubsystem.setIntakeMotor(0.35)));

    m_driverController1.leftStick().whileTrue(m_ElevatorSubsystem.goToAlgaeLowCommand());
    m_driverController1.leftStick().whileTrue(m_ElevatorSubsystem.goToAlgaeHighCommand());
    /*m_driverController1.povRight().whileTrue(m_AlgaeSubsystem.run(()->m_AlgaeSubsystem.turnPIDon(true))
      .finallyDo(()->m_AlgaeSubsystem.turnPIDon(false)));*/

    //Algae Position Controls
    // m_driverController1.povDown().whileTrue(m_AlgaeSubsystem.grabAlgaeCommand()
    //     .alongWith(m_ElevatorSubsystem.goToAlgaeLowCommand())); //will not stop algae after command is realased
    // m_driverController1.povUp().whileTrue(m_AlgaeSubsystem.grabAlgaeCommand()
    //     .alongWith(m_ElevatorSubsystem.goToAlgaeHighCommand())); //will not stop algae after command is realased
    // m_driverController1.povRight().whileTrue(m_AlgaeSubsystem.groundIntakeCommand()
    //   .alongWith(m_ElevatorSubsystem.goToElevatorStowCommand()));
    // m_driverController1.povLeft().whileTrue(m_AlgaeSubsystem.stowAlgaeCommand()
    //     .alongWith(m_ElevatorSubsystem.goToElevatorStowCommand()));
      
    // m_driverController0.start().onTrue(Commands.run(()->m_AlgaeSubsystem.turnPIDon(false)));

    // m_driverController1.rightStick().whileTrue(m_AlgaeSubsystem.scoreCommand());
    // m_driverController1.leftStick().whileTrue(m_AlgaeSubsystem.intakeCommand());

    
    //agae elevator comands
    //m_driverController1.leftStick().whileTrue(m_ElevatorSubsystem.goToAlgaeLowCommand());
    //m_driverController1.rightStick().whileTrue(m_ElevatorSubsystem.goToAlgaeHighCommand());

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    // An example command will be run in autonomous
    return m_chooser.getSelected();
    //return new PathPlannerAuto("Start To Reef");
    /*return m_robotArm.moveToPosCommand(Math.PI/4)
      .andThen(m_robotArm.moveToPosCommand(0.3919))
      .andThen(m_robotIntake.shootSpeakerCommand());*/
    //return new SequentialCommandGroup(m_robotArm.moveToPosCommand(Math.PI/3), m_robotArm.moveToPosCommand(0.3919), m_robotIntake.shootAmpCommand());
    //return m_robotDrive.getAutonomousCommand("test Auto");
  }
  
  public void disabledInit (){
    m_ElevatorSubsystem.disabledInit();
    m_CoralSubsystem.disabled();
    m_AlgaeSubsystem.disabledInit();
  }
 
  public void writePeriodicOutputs(){
    
    SmartDashboard.putBoolean("Driver/Angle Lock", (Math.abs(LimelightHelpers.getTX(LimelightConstants.kLimelightName))<15));
    SmartDashboard.putBoolean("Driver/Sidways Lock",Math.abs(LimelightHelpers.getBotPose_TargetSpace(LimelightConstants.kLimelightName)[0])<2);
    SmartDashboard.putNumber("LY", m_driverController1.getLeftY());
  }

  public void setMotorBrake(boolean brake)
  {
    m_robotDrive.setMotorBrake(brake);
  }
}