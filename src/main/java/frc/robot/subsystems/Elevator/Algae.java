package frc.robot.subsystems.Elevator;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

//import frc.robot.simulation.SimulatableCANSparkMax;
import com.revrobotics.spark.SparkMax;
import frc.robot.wrappers.REVThroughBoreEncoder;

public class Algae extends SubsystemBase {

  /*-------------------------------- Private instance variables ---------------------------------*/
  private static Algae mInstance;
  private PeriodicIO mPeriodicIO;
  private boolean on;

  public static Algae getInstance() {
    if (mInstance == null) {
      mInstance = new Algae();
    }
    return mInstance;
  }

  public enum IntakeState {
    NONE,
    STOW,
    DEALGAE,
    GROUND
  }

  private SparkMax mWristMotor;
  private final ProfiledPIDController mWristPIDController;
  private final ArmFeedforward mWristFeedForward;

  private SparkMax mIntakeMotor;

  private final REVThroughBoreEncoder mWristAbsEncoder = new REVThroughBoreEncoder(Constants.Algae.kWristEncoderId);

  public Algae() {

    mPeriodicIO = new PeriodicIO();

    // WRIST
    mWristMotor = new SparkMax(Constants.Algae.kWristMotorId, MotorType.kBrushless);
    SparkMaxConfig wristConfig = new SparkMaxConfig();
    wristConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(Constants.Algae.kMaxWristCurrent)
        .inverted(true);

    mWristMotor.configure(
        wristConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    // Wrist PID
    mWristPIDController = new ProfiledPIDController(
        Constants.Algae.kWristP,
        Constants.Algae.kWristI,
        Constants.Algae.kWristD,
        new TrapezoidProfile.Constraints(
            Constants.Algae.kWristMaxVelocity,
            Constants.Algae.kWristMaxAcceleration));

    // Wrist Feedforward
    mWristFeedForward = new ArmFeedforward(
        Constants.Algae.kWristKS,
        Constants.Algae.kWristKG,
        Constants.Algae.kWristKV,
        Constants.Algae.kWristKA);

    // INTAKE
    mIntakeMotor = new SparkMax(Constants.Algae.kIntakeMotorId, MotorType.kBrushless);
    SparkMaxConfig intakeConfig = new SparkMaxConfig();

    intakeConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(Constants.Algae.kMaxIntakeCurrent)
        .inverted(true);

    mIntakeMotor.configure(
        intakeConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  private static class PeriodicIO {
    double wrist_target_angle = 139;
    double wrist_voltage = 0.0;

    double intake_power = 0.0;

    boolean on = false;

    IntakeState state = IntakeState.STOW;
  }

  /*-------------------------------- Generic Subsystem Functions --------------------------------*/

  @Override
  public void periodic() {
    if(on){
      double pidCalc = mWristPIDController.calculate(getWristAngle(), mPeriodicIO.wrist_target_angle);
      /*double ffCalc = mWristFeedForward.calculate(Math.toRadians(getWristReferenceToHorizontal()),
          Math.toRadians(mWristPIDController.getSetpoint().velocity));*/
      double ffCalc = 0;

      mPeriodicIO.wrist_voltage =Math.max(Math.min(pidCalc + ffCalc, 0.2), -0.2);
    }
    writePeriodicOutputs();
    outputTelemetry();
  }

  public void writePeriodicOutputs() {
    mWristMotor.set(mPeriodicIO.wrist_voltage);

    mIntakeMotor.set(mPeriodicIO.intake_power);
  }

  public void turnPIDon(boolean turn){
    this.on = turn;
  }

  public void stop() {
    mPeriodicIO.wrist_voltage = 0.0;
    mPeriodicIO.wrist_target_angle = Constants.Algae.kStowAngle;

    mWristMotor.set(0.0);
    mIntakeMotor.set(0.0);
  }

  public void outputTelemetry() {
    SmartDashboard.putNumber("Wrist/Position", getWristAngle());
    SmartDashboard.putNumber("Wrist/Target", mPeriodicIO.wrist_target_angle);
    SmartDashboard.putNumber("Wrist/Current", mWristMotor.getOutputCurrent());
    SmartDashboard.putNumber("Wrist/Output", mWristMotor.getAppliedOutput());
    SmartDashboard.putNumber("Wrist/Voltage", mPeriodicIO.wrist_voltage);
    SmartDashboard.putNumber("Wrist/Frequency", mWristAbsEncoder.getFrequency());
    SmartDashboard.putNumber("Wrist/PIDVolt", mWristPIDController.calculate(getWristAngle(), mPeriodicIO.wrist_target_angle));

    SmartDashboard.putNumber("Intake/Current", mIntakeMotor.getOutputCurrent());
    SmartDashboard.putNumber("Intake/Output", mIntakeMotor.getAppliedOutput());
    SmartDashboard.putNumber("Intake/Power", mPeriodicIO.intake_power);
  }

  public void reset() {
  }

  /*---------------------------------- Custom Public Functions ----------------------------------*/

  public void stow() {
    mPeriodicIO.wrist_target_angle = Constants.Algae.kStowAngle;

    mPeriodicIO.state = IntakeState.STOW;
    // mPeriodicIO.intake_power = 0.0;
  }

  public void setWristMotor(double power){
    mPeriodicIO.wrist_voltage = power;
    //mWristMotor.set(power);
    //mPeriodicIO.on = true;
  }

  public void setIntakeMotor(double power){
    mPeriodicIO.intake_power = power;
    //mIntakeMotor.set(power);
  }

  public void grabAlgae() {
    mPeriodicIO.wrist_target_angle = Constants.Algae.kDeAlgaeAngle;
    mPeriodicIO.intake_power = Constants.Algae.kIntakeSpeed;

    mPeriodicIO.state = IntakeState.DEALGAE;
  }

  public void score() {
    if (mPeriodicIO.state == IntakeState.GROUND) {
      mPeriodicIO.intake_power = -Constants.Algae.kEjectSpeed;
    } else {
      mPeriodicIO.intake_power = Constants.Algae.kEjectSpeed;
    }
  }

  public void groundIntake() {
    mPeriodicIO.wrist_target_angle = Constants.Algae.kGroundIntakeAngle;
    mPeriodicIO.intake_power = Constants.Algae.kGroundIntakeSpeed;

    mPeriodicIO.state = IntakeState.GROUND;
  }

  public void stopAlgae() {
    mPeriodicIO.intake_power = 0.0;
    mPeriodicIO.wrist_target_angle = Constants.Algae.kStowAngle;
  }

  /*---------------------------------- Custom Private Functions ---------------------------------*/

  public double getWristAngle() {
    // TODO:
    // This used to be `getAbsolutePosition` in the old API
    // but I'm not sure if `get` is the correct replacement
    return Units.rotationsToDegrees(mWristAbsEncoder.get());
  }

  public double getWristReferenceToHorizontal() {
    return getWristAngle() - Constants.Algae.kWristOffset;
  }

  public IntakeState getState() {
    return mPeriodicIO.state;
  }

  // public double getSpeedFromState(IntakeState state) {
  // switch (state) {
  
  // case NONE:
  //   return 0.0;
  // case INTAKE:
  //   return RobotConstants.config.Intake.k_intakeSpeed;
  // case INDEX:
  //   return RobotConstants.config.Intake.k_ejectSpeed;
  // case READY:
  //   return RobotConstants.config.Intake.k_feedShooterSpeed;
  // default:
  //   return 0.0;
  // }
  // }
}