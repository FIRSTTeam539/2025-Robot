// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

//import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.config.PIDConstants;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
//import com.revrobotics.jni.CANSparkMax.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;
import edu.wpi.first.math.geometry.Translation3d;
import swervelib.parser.PIDFConfig;
import edu.wpi.first.wpilibj.CounterBase;
import edu.wpi.first.wpilibj.Encoder;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final double ROBOT_MASS = (125) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.5; //s, 20ms + 110ms sprk max velocity lag

  public static final class Auton
  {

    //public static final PIDFConfig xAutoPID     = new PIDFConfig(0.7, 0, 0);
    //public static final PIDFConfig yAutoPID     = new PIDFConfig(0.7, 0, 0);
    //public static final PIDFConfig angleAutoPID = new PIDFConfig(0.4, 0, 0.01);


    
    //public static final PIDFConfig TranslationPID = new PIDFConfig(0, 0, 0);
    //public static final PIDFConfig angleAutoPID   = new PIDFConfig(0.4, 0, 0.01);

    public static final double MAX_SPEED        = 3.8;
    public static final double MAX_ACCELERATION = 2;
  }
   public static final class AutonConstants
  { // DO NOT DELTE (until this numbers are ut into the new auto PID and tuned)
    ///important numbers
    public static final PIDConstants TRANSLATION_PID = new PIDConstants(1.1, 0, 0.1); //1.3, 0.15 -- 3.5, 0.5
    public static final PIDConstants ANGLE_PID   = new PIDConstants(0.7, 0, 0.05); //2.2, 0 -- 4, 0
  }

  public static class Elevator {
    public static final int kElevatorLeftMotorId = 10;
    public static final int kElevatorRightMotorId = 9;

    public static final double kAcceptableError = 0.3;

    public static final double kP = 0.25; //0.25
    public static final double kI = 0;
    public static final double kD = 0.3; //6
    public static final double kIZone = 5.0;
    public static final double kG = 1.1;//1.1;//-1.1;//1.1;

    public static final double kMaxVelocity = 60;//65
    public static final double kMaxAcceleration = 100;//150

    public static final int kMaxCurrent = 40;
    public static final double kMaxPowerUp = -0.3; //-0.4
    public static final double kMaxPowerDown = 0.3;//0.4

    public static final double kStowHeight = 0.1;
    public static final double kL2Height = 9.1;
    public static final double kL3Height = 26.14;
    public static final double kL4Height = 54.5; //55.15
    public static final double kMaxHeight = 56.2;
    public static final double kGroundAlgaeHeight = 0.0;
    public static final double kScoreAlgaeHeight = 0.0;
    public static final double kLowAlgaeHeight = 20.8;
    public static final double kHighAlgaeHeight = 42.5;
  }


  public static class Coral {
    public static final int kLeftMotorId = 14;
    public static final int kRightMotorId = 13;

    public static final int kLaserId = 59;
    public static final int kColorId = 58;

    public static final double kMaxCurrent = 20;

    public static final double kP = 0.0;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kIZone = 0;

    public static final double kIntakeSpeed = 0.25;
    public static final double kReverseSpeed = -0.3;
    public static final double kL1Speed = 0.35;
    public static final double kL24Speed = 0.4;
    public static final double kIndexSpeed = 0.05;
    public static final double kSpeedDifference = kL1Speed * 0.80;
  }

  public static class Algae {
    // WRIST
    public static final int kWristMotorId = 11;
    public static final int kIntakeMotorId = 12;

    public static final int kWristEncoderId = 0;

    public static final int kMaxWristCurrent = 0;//not set up

    public static final double kWristP = 0.02;
    public static final double kWristI = 0.0;
    public static final double kWristD = 0.0;

    public static final double kWristKS = 0.0;
    public static final double kWristKG = 0.0;
    public static final double kWristKV = 0;//0.100;
    public static final double kWristKA = 0.0;

    //public static final double kWristMaxVelocity = 0.0001;//690.0;
    //public static final double kWristMaxAcceleration = 100;//1380.0;

    public static final double kStowAngle = 210;
    public static final double kDeAlgaeAngle = 58;
    public static final double kGroundIntakeAngle = 105;

    public static final double kWristOffset = 113; //TODO: find true value

    public static final double kAllowedWristError = 1;

    // INTAKE
    public static final int kMaxIntakeCurrent = 20;

    public static final double kIntakeSpeed = 0.3;
    public static final double kAlgaeHoldSpeed = 0.35;
    public static final double kEjectSpeed = -0.3;
    public static final double kGroundIntakeSpeed = -0.3;
  }

  public static final class ClimbConstants {

    public static final int kClimbSparkMaxCANID = 15;

    public static final double kStaticArmRate = 0;
  }
  public static class LEDs {
    public static final int k_PWMId = 9;
    public static final int k_totalLength = 300;
  }

  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double MAX_SPEED = 6;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

    public static final double WHEEL_LOCK_TIME = 10;

    public static final double kDirectionSlewRate = 1.2; // radians per second
    public static final double kMagnitudeSlewRate = 1.8; // percent per second (1 = 100%)
    public static final double kRotationalSlewRate = 2.0; // percent per second (1 = 100%)

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(24);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(24);

    public static final double APRILTAG_X_TOLERANCE = 1.5;
    public static final double APRILTAG_Y_TOLERANCE = 0.5;
    public static final double APRILTAG_ROTATION_TOLERANCE = .025; // Radians
    public static final double APRILTAG_TY_MAGIC_OFFSET = 12.5;

    public static final double AMP_TX_SETPOINT = 0;
    public static final double AMP_TY_SETPOINT = -10;
    public static final double AMP_ROTATION_SETPOINT = Math.PI / 2;
    public static final double AUTO_TRANSLATE_DEBOUNCE_SECONDS = 0.1;


    //AutoAim Constants
    public static final double kPFarAprilTagRot = 0.01;
    public static final double kPAprilTagRot = 0.045;
    public static final double kPAprilTagTranDist = 1.4;
    public static final double kPLaserDist = 2.5;
    public static final double kPAprilTagTranSide = 2.2;
    public static final double kDistFF = 0.05;

    public static final double kAcceptableErrorSide = 0.01;

    public static final double kDistOffset = 0.4;
    public static final double kSideOffLeft = -0.14;
    public static final double kSideOffRight = 0.14;
    public static final double kRotOffset = 0;
    
    public static final int kDistLaserId = 61;

    public static final double kL2Dist = 0.09;
    public static final double kL3Dist = 0.09;
    public static final double kL4Dist = 0.26;

  }
  public static final class LimelightConstants{
    public static final String kLimelightName = "limelight";

    public static final double kSideLockError = 0.15;
    public static final double kRotateLockError =10;

    public static final double forward = Units.inchesToMeters(15);
    public static final double side = 0;
    public static final double up = Units.inchesToMeters(6);
    public static final double pitch = 85;
    public static final double roll = 0;
    public static final double yaw = 0;
    
    public static final double XStDevs = 0.5;
    public static final double YStDevs = 0.5;
  }

  public static final class FieldConstants {

  }

  public static final class OIConstants {
    public static final int kDriverControllerPort0 = 0;
    public static final int kDriverControllerPort1 = 1;
    public static final double kDriveDeadband = 0.05;


    public static final double kDefaultDriveSpeed = 0.6;
    public static final double kDriveSpeedIncreaseConstant = 1- kDefaultDriveSpeed;
    //controller 0
    // Joystick Deadband
    public static final double LEFT_X_DEADBAND_1 = 0.05;
    public static final double LEFT_Y_DEADBAND_1 = 0.05;
    public static final double RIGHT_X_DEADBAND_1 = 0.05;

    public static final double RIGHT_TRIGGER_DEADBAND_1 = 0.05;
    public static final double TURN_CONSTANT = 0.75;

    //controller 1
    public static final double RIGHT__Y_DEADBAND_2 = 0.05;
    public static final double LEFT_X__DEADBAND_2 = 0.1;
    public static final double RIGHT_TRIGGER_DEADBAND_2 = 0.05;
  }



}