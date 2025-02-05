// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
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

import frc.robot.subsystems.climber.ClimbSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.Elevator.ArmSubsystem;
import frc.robot.subsystems.Elevator.IntakeSubsystem;
import frc.robot.subsystems.Elevator.ElevatorSubsystem;

import frc.robot.utils.utils;

import java.io.File;
import edu.wpi.first.wpilibj2.command.Commands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.swervedrive.auto.TurnToAprilTagCommand;
import frc.robot.Constants.ArmConstants;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import com.pathplanner.lib.auto.NamedCommands;
import swervelib.SwerveInputStream;

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
  private final LimelightSubsystem m_robotLimelight = new LimelightSubsystem("limelight");
  private final ElevatorSubsystem m_ElevatorSubsystem = new ElevatorSubsystem();
    
  SendableChooser<Command> m_chooser = new SendableChooser<>();
  
  // The driver's controller
  CommandXboxController m_driverController0 = new CommandXboxController(OIConstants.kDriverControllerPort0);
  CommandXboxController m_driverController1 = new CommandXboxController(OIConstants.kDriverControllerPort1);
  //Command turnToAprilTagCommand = new TurnToAprilTagCommand(m_robotDrive, m_robotLimelight);
  //SequentialCommandGroup x = new SequentialCommandGroup(new TurnToAprilTagCommand(m_robotDrive, m_robotLimelight), m_robotArm.moveToPositionCommand());
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(m_robotDrive.getSwerveDrive(),
    () -> m_driverController0.getLeftY(),
    () -> m_driverController0.getLeftX())
    .withControllerRotationAxis(m_driverController0::getRightX)
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

    Shuffleboard.getTab("Important").add("auto chooser", m_chooser);
    m_chooser.addOption("do nothing", null);
    m_chooser.addOption("2m test", new PathPlannerAuto("2m test"));
    m_chooser.addOption("2m spin test", new PathPlannerAuto("2m spin test"));
    m_chooser.addOption("test Auto", new PathPlannerAuto("test Auto"));
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
    //m_driverController0.y().whileTrue(Commands.run(() -> m_robotDrive.zeroGyro()));
    //m_driverController0.rightBumper().whileTrue(new TurnToAprilTagCommand(m_robotDrive, m_robotLimelight, 2));
    Command driveFieldOrientedAnglularVelocity = m_robotDrive.driveFieldOriented(driveAngularVelocity);
    m_robotDrive.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    //m_driverController0.button(1).onTrue(Commands.run(()->SmartDashboard.putBoolean("yay!", false)));



    m_driverController1.leftBumper().whileTrue(m_ElevatorSubsystem.run(()->m_ElevatorSubsystem.setElevatorPower(0.2))
                                    .finallyDo(()->m_ElevatorSubsystem.stop()));//set motor id
    m_driverController1.rightBumper().whileTrue(m_ElevatorSubsystem.run(()->m_ElevatorSubsystem.setElevatorPower(-0.2))
                                    .finallyDo(()->m_ElevatorSubsystem.stop()));
    
    m_driverController1.a().whileTrue(m_ElevatorSubsystem.run(()->m_ElevatorSubsystem.goToElevatorStow()));
    m_driverController1.b().whileTrue(m_ElevatorSubsystem.run(()->m_ElevatorSubsystem.goToElevatorL2()));
    m_driverController1.x().whileTrue(m_ElevatorSubsystem.run(()->m_ElevatorSubsystem.goToElevatorL3()));
    m_driverController1.y().whileTrue(m_ElevatorSubsystem.run(()->m_ElevatorSubsystem.goToElevatorL4()));

    m_driverController1.rightTrigger().onTrue(m_ElevatorSubsystem.run(()->m_ElevatorSubsystem.stop()));
    m_driverController1.leftTrigger().onTrue(m_ElevatorSubsystem.run(()->m_ElevatorSubsystem.reset()));
    /*m_driverController0.leftBumper().whileTrue(m_robotArm.moveToPosCommand(0.1));drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    m_driverController0.rightBumper().whileTrue(m_robotArm.moveToPosCommand(Math.PI/3));
    
    //m_driverController1.leftBumper().whileTrue(Commands.run(()->m_robotClimb.setDualClimb(MathUtil.applyDeadband(m_driverController1.getLeftY(), 0.2), MathUtil.applyDeadband(m_driverController1.getRightY(), 0.2)*0.6), m_robotClimb));
    m_driverController1.rightBumper().whileTrue(m_robotIntake.shootSpeakerCommand());
    m_driverController1.y().whileTrue(m_robotIntake.intakeCommand().andThen(m_robotArm.moveToPosCommand(0.1)));
    m_driverController1.x().whileTrue(m_robotIntake.shootAmpCommand());
    //m_driverController1.a().onTrue(m_robotArm.disableArm());
    m_driverController1.povUp().whileTrue(Commands.run(()->m_robotArm.setArmVelocity(MathUtil.applyDeadband(m_driverController1.getRightTriggerAxis(), 0.1)-MathUtil.applyDeadband(m_driverController1.getLeftTriggerAxis(),0.1)*0.25), m_robotArm));
    
    //m_driverController1.a().whileTrue(m_robotIntake.justShootCommand(1));//MathUtil.applyDeadband(m_driverController1.getLeftTriggerAxis(),0.1)*0.5));
    m_driverController1.b().whileTrue(m_robotIntake.justIntakeCommand(0.3 ));
    m_driverController1.povDown().whileTrue(m_robotIntake.justIntakeCommand(-0.2));
    m_driverController1.povRight().whileTrue(m_robotArm.moveToPosCommand(0.450));
    //m_driverController1.povRight().whileTrue(m_robotArm.moveToPosCommand(0.450));
    //m_driverController1.povRight().whileTrue(m_robotArm.moveToPosCommand(0.3919));
    m_driverController1.povLeft().whileTrue(m_robotArm.moveToPosCommand(1.067));
    //m_driverController1.povLeft().whileTrue(m_robotArm.moveToPosCommand(0.6128));
    
    //m_driverController1.x().whileTrue(m_robotIntake.justIntakeCommand(0));
    //m_driverController1.b().whileTrue(m_robotIntake.justShootCommand(0));

    //m_driverController1.povLeft().whileTrue(m_robotArm.moveToPosCommand(Math.PI/4));

    //m_driverController1.x().whileTrue(m_robotClimb.climbLeftCommand(0.2));
    //m_driverController1.b().whileTrue(m_robotClimb.climbRightCommand(0.2));
    //m_driverController1.y().whileTrue(m_robotClimb.climbRightCommand(-0.2));
    //m_driverController1.leftBumper().whileTrue(m_robotClimb.climbLeftCommand(-0.2));
   
    //m_driverController1.x().whileTrue(m_robotArm.moveArm(0.3));
    //m_driverController1.b().whileTrue(m_robotArm.moveArm(-0.2));
    
    //m_driverController1.leftBumper().whileTrue(Commands.run((()->m_robotArm.moveArmAtSpeed(0.2)), m_robotArm));
    //m_driverController1.rightBumper().whileTrue(Commands.run((()->m_robotArm.moveArmAtSpeed(-0.2)), m_robotArm));
    //m_driverController1.leftTrigger().onTrue(m_robotIntake.intakeCommand());
    //m_driverController1.rightBumper().onTrue(m_robotIntake.shootSpeakerCommand());
    //m_driverController1.leftBumper().onTrue(m_robotIntake.shootSpeakerCommand());
    //m_driverController0.rightBumper().onTrue(Commands.run(() -> Climb.Climb)));*/
  }
  /*public void periodic(){
    if(m_robotLimelight.getTV()){
      if(Math.sqrt(Math.pow(m_robotDrive.getPose().getX()-m_robotLimelight.getBotPose2d().getX(), 2)+Math.pow(m_robotDrive.getPose().getY()-m_robotLimelight.getBotPose2d().getY(), 2))<1){
        m_robotDrive.addVisionMeasurement(m_robotLimelight.getBotPose2d(), Timer.getFPGATimestamp());
      }
    }
  }*/
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    // An example command will be run in autonomous
    return m_chooser.getSelected();
    /*return m_robotArm.moveToPosCommand(Math.PI/4)
      .andThen(m_robotArm.moveToPosCommand(0.3919))
      .andThen(m_robotIntake.shootSpeakerCommand());*/
    //return new SequentialCommandGroup(m_robotArm.moveToPosCommand(Math.PI/3), m_robotArm.moveToPosCommand(0.3919), m_robotIntake.shootAmpCommand());
    //return m_robotDrive.getAutonomousCommand("test Auto");
  }
  

  public void setMotorBrake(boolean brake)
  {
    //m_robotDrive.setMotorBrake(brake);
  }
}