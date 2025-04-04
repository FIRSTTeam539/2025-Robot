package frc.robot.subsystems.Elevator;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
//import frc.robot.simulation.SimulatableCANSparkMax;
import frc.robot.utils.Elastic;

import com.reduxrobotics.canand.CanandDevice;
import com.reduxrobotics.sensors.canandcolor.Canandcolor;
import com.ctre.phoenix.led.CANdle;
import com.revrobotics.spark.SparkFlex;
//import frc.robot.subsystems.LEDS.LEDs;

public class Coral extends SubsystemBase {

  /*-------------------------------- Private instance variables ---------------------------------*/
  private static Coral mInstance;
  private PeriodicIO mPeriodicIO;
  //public CANdle m_CANdle60 = new CANdle(60);
   //public final LEDs m_leds = LEDs.getInstance();

  public static Coral getInstance() {
    if (mInstance == null) {
      mInstance = new Coral();
    }
    return mInstance;
  }

  public enum IntakeState {
    NONE,
    INTAKE,
    // REVERSE,
    INDEX,
    READY,
    SCORE
  }

  // private ThriftyNova mLeftMotor;
  // private ThriftyNova mRightMotor;
  private SparkFlex mLeftMotor; // spark flex
  private SparkFlex mRightMotor;

  private LaserCan mLaserCAN;
  private Canandcolor mCanandcolor;

  public Coral() {
    super("Coral");

    mPeriodicIO = new PeriodicIO();

    mLeftMotor = new SparkFlex(Constants.Coral.kLeftMotorId, MotorType.kBrushless);
    mRightMotor = new SparkFlex(Constants.Coral.kRightMotorId, MotorType.kBrushless);
    //TODO: implement canandcolor

    SparkMaxConfig coralConfig = new SparkMaxConfig();

    coralConfig.idleMode(IdleMode.kBrake);

    mLeftMotor.configure(
        coralConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    mRightMotor.configure(
        coralConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    // Configure the LaserCan (intake)
    try{
      mLaserCAN = new LaserCan(Constants.Coral.kLaserId);
    } catch(Exception e){
      Elastic.sendNotification(new Elastic.Notification(Elastic.Notification.NotificationLevel.ERROR, 
      "Coral Laser Can Error", "Coral LaserCAN Initialization Error"));
    }
    try {
      mLaserCAN.setRangingMode(LaserCan.RangingMode.SHORT);
      mLaserCAN.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 16, 16));
      mLaserCAN.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
    } catch (ConfigurationFailedException e) {
      System.out.println("Configuration failed! " + e);
    }

    // Configure the Canandcolor (outake)
    try{
      mCanandcolor = new Canandcolor(Constants.Coral.kColorId);
    } catch (Exception e){
      Elastic.sendNotification(new Elastic.Notification(Elastic.Notification.NotificationLevel.ERROR, 
      "Coral CANANDColor Error", "Coral CANANDColor Initialization Error"));
    }
  }

  private static class PeriodicIO {
    double rpm = 0.0;
    double speed_diff = 0.0;

    int index_debounce = 0;

    LaserCan.Measurement measurement;

    IntakeState state = IntakeState.NONE;
  }

  /*-------------------------------- Generic Subsystem Functions --------------------------------*/

  @Override
  public void periodic() {
    mPeriodicIO.measurement = mLaserCAN.getMeasurement();
    writePeriodicOutputs();
    checkAutoTasks();
    outputTelemetry();
  }

  public void writePeriodicOutputs() {
    mLeftMotor.set(mPeriodicIO.rpm - mPeriodicIO.speed_diff);
    mRightMotor.set(-mPeriodicIO.rpm);
  }

  public void stop() {
    mPeriodicIO.rpm = 0.0;
    mPeriodicIO.speed_diff = 0.0;
    mPeriodicIO.state = IntakeState.NONE;
  }

  public void stopKeepState() {
    mPeriodicIO.rpm = 0.0;
    mPeriodicIO.speed_diff = 0.0;
  }

  public void outputTelemetry() {
    SmartDashboard.putNumber("RPM/target", mPeriodicIO.rpm);

    LaserCan.Measurement measurement = mPeriodicIO.measurement;
    if (measurement != null) {
      SmartDashboard.putNumber("Laser/distance", measurement.distance_mm);
      SmartDashboard.putNumber("Laser/ambient", measurement.ambient);
      SmartDashboard.putNumber("Laser/budget_ms", measurement.budget_ms);
      SmartDashboard.putNumber("Laser/status", measurement.status);

      SmartDashboard.putBoolean("Laser/hasCoral", isHoldingCoralViaLaserCAN());
    }
  }

  public boolean hasCoral(){
    return !isHoldingCoralViaLaserCAN()&isHoldingCoralViaCanandColor();
  }

  public void reset() {
    stopCoral();
  }

  /*---------------------------------- Custom Public Functions ----------------------------------*/

  private boolean isHoldingCoralViaLaserCAN() {
    try {
      return mPeriodicIO.measurement.distance_mm < 75.0;
    } catch(Exception e){
      Elastic.sendNotification(new Elastic.Notification(Elastic.Notification.NotificationLevel.ERROR, 
      "Coral Laser Can Error", "Coral LaserCAN Read Error"));
      return false;
    }
  }

  private boolean isHoldingCoralViaCanandColor() {
    try {
      return mCanandcolor.getProximity() < 0.05; // Experimentally determined on 2025-02-08
    } catch(Exception e){
      Elastic.sendNotification(new Elastic.Notification(Elastic.Notification.NotificationLevel.ERROR, 
      "Coral CANANDColor Error", "Coral CANANDColor Read Error"));
      return false;
    }
  }

  private void setSpeed(double rpm) {
    mPeriodicIO.speed_diff = 0.0;
    mPeriodicIO.rpm = rpm;
  }

  public void intake() {
    mPeriodicIO.speed_diff = 0.0;
    mPeriodicIO.rpm = Constants.Coral.kIntakeSpeed;
    mPeriodicIO.state = IntakeState.INTAKE;

     //m_leds.setColor(Color.kYellow);
  }

  public Command setIntakeCommand(){
    return this.run(()->this.intake());
  }

  // public void reverse() {
  //   mPeriodicIO.speed_diff = 0.0;
  //   mPeriodicIO.rpm = Constants.Coral.kReverseSpeed;
  //   mPeriodicIO.state = IntakeState.REVERSE;
  // }

  public void index() {
    mPeriodicIO.speed_diff = 0.0;
    mPeriodicIO.rpm = Constants.Coral.kIndexSpeed;
    mPeriodicIO.state = IntakeState.INDEX;

    //m_leds.setColor(Color.kBlue);
  }

  public Command scoreL24Command() {
    return this.run(() -> this.scoreL24())
    .until(()->this.isNoneState()).finallyDo(()->this.setState(IntakeState.NONE));
  }

  public Command scoreL1Command() {
    return this.run(() -> this.scoreL1())
    .until(()->this.isNoneState()).finallyDo(()->this.setState(IntakeState.NONE));
  }

  public boolean isNoneState(){
    return (!isHoldingCoralViaLaserCAN() && !isHoldingCoralViaCanandColor());
  }

  public Command indexCommand(){
    return this.run(()->this.index())
    .until(()->!isHoldingCoralViaLaserCAN())
    .finallyDo(()->this.stopKeepState());
  }


  public Command intakeCommand() {
     return this.run(() -> this.intake())
     .until(()->isHoldingCoralViaLaserCAN() && isHoldingCoralViaCanandColor())
     .andThen(this.indexCommand())
    .finallyDo(()->this.stopKeepState());
  }

  public void scoreL1() {
    mPeriodicIO.speed_diff = Constants.Coral.kSpeedDifference;
     mPeriodicIO.rpm = Constants.Coral.kL1Speed;
     mPeriodicIO.state = IntakeState.SCORE;
  }

  public void scoreL24() {
     mPeriodicIO.speed_diff = 0.0;
     mPeriodicIO.rpm = Constants.Coral.kL24Speed;
     mPeriodicIO.state = IntakeState.SCORE;
  }

  public void setState(final IntakeState state) {
    mPeriodicIO.state = state;
  }

  public void stopCoral() {
    mPeriodicIO.rpm = 0.0;
    mPeriodicIO.speed_diff = 0.0;
    mPeriodicIO.state = IntakeState.NONE;
  }

  public IntakeState getIntakeState(){
    return mPeriodicIO.state;
  }

  /*---------------------------------- Custom Private Functions ---------------------------------*/

  private void checkAutoTasks() {
    switch (mPeriodicIO.state) {
      case NONE:
        this.setSpeed(0);
        //m_CANdle60.setLEDs(255, 0, 0); //All m_Candle60 functions here changed for new LED code
        if (isHoldingCoralViaLaserCAN()) {
          this.setState(IntakeState.INTAKE);
        }
        break;
      case INTAKE:
        this.setSpeed(Constants.Coral.kIntakeSpeed);
        //m_CANdle60.setLEDs(255, 128, 0);
        if (isHoldingCoralViaLaserCAN() && isHoldingCoralViaCanandColor()) {
          this.setState(IntakeState.INDEX);
        }
        break;
      case INDEX:
        this.setSpeed(Constants.Coral.kIndexSpeed);
        //m_CANdle60.setLEDs(255, 255, 0);
        if (!isHoldingCoralViaLaserCAN()) {
          this.setState(IntakeState.READY);
        }
        break;
      case READY:
        this.setSpeed(0.0);
        // For now, the state will be moved to SCORE externally via controller1
        //m_CANdle60.setLEDs(0, 255, 0);
        break;
      default:
        break;
    }
  }

  public void disabled(){
    this.setState(IntakeState.NONE);
  }
}