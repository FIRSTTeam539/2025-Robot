// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
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
import frc.robot.commands.swervedrive.auto.TurnToAprilTagCommand;

//for old climb system
import frc.robot.subsystems.climber.ClimbSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

//modern subsystems
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.Elevator.ElevatorSubsystem;
import frc.robot.subsystems.Elevator.Coral.IntakeState;
import frc.robot.subsystems.Elevator.Coral;

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
    
  SendableChooser<Command> m_chooser = new SendableChooser<>();
  
  // The driver's controller
  CommandXboxController m_driverController0 = new CommandXboxController(OIConstants.kDriverControllerPort0);
  CommandXboxController m_driverController1 = new CommandXboxController(OIConstants.kDriverControllerPort1);
  //Command turnToAprilTagCommand = new TurnToAprilTagCommand(m_robotDrive, m_robotLimelight);
  //SequentialCommandGroup x = new SequentialCommandGroup(new TurnToAprilTagCommand(m_robotDrive, m_robotLimelight), m_robotArm.moveToPositionCommand());
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(m_robotDrive.getSwerveDrive(),
    () -> -m_driverController0.getLeftY(),
    () -> -m_driverController0.getLeftX())
    .withControllerRotationAxis(() -> m_driverController0.getRightX() * -1)
    .deadband(0.17)
    .scaleTranslation(
      0.8)
    .allianceRelativeControl(true);
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    
    configureButtonBindings();
    
    // Configure default command
    
    //Shuffleboard.getTab("Arm").add(m_robotArm);
    NamedCommands.registerCommand("L3", m_ElevatorSubsystem.goToElevatorL3Command()
      .andThen(m_CoralSubsystem.scoreL24Command())
      .andThen(m_ElevatorSubsystem.goToElevatorStowCommand()));

    Shuffleboard.getTab("Important").add("auto chooser", m_chooser);
    m_chooser.setDefaultOption("do nothing", null);
    m_chooser.addOption("2m test", new PathPlannerAuto("2m test"));
    m_chooser.addOption("Start to Reef", new PathPlannerAuto("Start To Reef"));
    m_chooser.addOption("2m spin test", new PathPlannerAuto("2m spin test"));
    m_chooser.addOption("1C I", new PathPlannerAuto("1C I"));
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
    m_driverController0.y().whileTrue(Commands.run(() -> m_robotDrive.zeroGyro()));

    //m_driverController0.rightBumper().whileTrue(new TurnToAprilTagCommand(m_robotDrive, m_robotLimelight, 2));
    Command driveFieldOrientedAnglularVelocity = m_robotDrive.driveFieldOriented(driveAngularVelocity);
    m_robotDrive.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    //m_driverController0.button(1).onTrue(Commands.run(()->SmartDashboard.putBoolean("yay!", false)));



    /*m_driverController1.leftBumper().whileTrue(m_ElevatorSubsystem.run(()->m_ElevatorSubsystem.setElevatorPower(0.2))
                                    .finallyDo(()->m_ElevatorSubsystem.stop()));//set motor id
    m_driverController1.rightBumper().whileTrue(m_ElevatorSubsystem.run(()->m_ElevatorSubsystem.setElevatorPower(-0.2))
                                    .finallyDo(()->m_ElevatorSubsystem.stop()));*/
    
    m_driverController1.a().whileTrue(m_ElevatorSubsystem.goToElevatorStowCommand());
    //m_driverController1.b().whileTrue(m_ElevatorSubsystem.run(()->m_ElevatorSubsystem.goToElevatorL2()));
    m_driverController1.b().whileTrue(m_ElevatorSubsystem.goToElevatorL2Command());
    m_driverController1.x().whileTrue(m_ElevatorSubsystem.goToElevatorL3Command());
    m_driverController1.y().whileTrue(m_ElevatorSubsystem.goToElevatorL4Command());
    
    // m_driverController1.leftBumper().onTrue(m_CoralSubsystem.intakeCommand());
    m_driverController1.rightBumper().whileTrue(m_CoralSubsystem.scoreL24Command().andThen(m_ElevatorSubsystem.goToElevatorStowCommand()));

    m_driverController1.rightTrigger().onTrue(m_ElevatorSubsystem.run(()->m_ElevatorSubsystem.stop()));
    m_driverController1.leftTrigger().onTrue(m_ElevatorSubsystem.run(()->m_ElevatorSubsystem.reset()));

    m_driverController0.povDown().whileTrue(m_robotDrive.driveToPose(new Pose2d(new Translation2d(3.032, 3.830), new Rotation2d(0))));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    // An example command will be run in autonomous
    return new PathPlannerAuto("Start To Reef");
    /*return m_robotArm.moveToPosCommand(Math.PI/4)
      .andThen(m_robotArm.moveToPosCommand(0.3919))
      .andThen(m_robotIntake.shootSpeakerCommand());*/
    //return new SequentialCommandGroup(m_robotArm.moveToPosCommand(Math.PI/3), m_robotArm.moveToPosCommand(0.3919), m_robotIntake.shootAmpCommand());
    //return m_robotDrive.getAutonomousCommand("test Auto");
  }
  
  public void disabledInit (){
    m_ElevatorSubsystem.disabledInit();
  }

  public void setMotorBrake(boolean brake)
  {
    m_robotDrive.setMotorBrake(brake);
  }
}