package frc.robot.subsystems.Elevator;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;


import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

//

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;


public class ElevatorSubsystem extends SubsystemBase {
   /*-------------------------------- Private instance variables ---------------------------------*/
   private static ElevatorSubsystem mInstance;
   private PeriodicIO mPeriodicIO;
 
   // private static final double kPivotCLRampRate = 0.5;
   // private static final double kCLRampRate = 0.5;
 
   public static ElevatorSubsystem getInstance() {
     if (mInstance == null) {
       mInstance = new ElevatorSubsystem();
     }
     return mInstance;
   }
 
   private SparkMax mLeftMotor;
   private RelativeEncoder mLeftEncoder;
   private SparkClosedLoopController mLeftPIDController;
 
   private SparkMax mRightMotor;
   private RelativeEncoder mRightEncoder;
   private SparkClosedLoopController mRightPIDController;
 
   private TrapezoidProfile mProfile;
   private TrapezoidProfile.State mCurState = new TrapezoidProfile.State();
   private TrapezoidProfile.State mGoalState = new TrapezoidProfile.State();
   private double prevUpdateTime = Timer.getFPGATimestamp();
 
   public ElevatorSubsystem() {
 
     mPeriodicIO = new PeriodicIO();
 
     SparkMaxConfig elevatorConfig = new SparkMaxConfig();
 
     elevatorConfig.closedLoop // invert Down/up (switch them or get negative of line 164
         .pid(Constants.Elevator.kP, Constants.Elevator.kI, Constants.Elevator.kD)
         .iZone(Constants.Elevator.kIZone)
         .maxOutput(Constants.Elevator.kMaxPowerDown) //maybe switch to be max power up
         .minOutput(Constants.Elevator.kMaxPowerUp); // maybe switch to max power down
 
     elevatorConfig.smartCurrentLimit(Constants.Elevator.kMaxCurrent);
 
     elevatorConfig.idleMode(IdleMode.kBrake);
    
     // RIGHT ELEVATOR MOTOR
    mRightMotor = new SparkMax(Constants.Elevator.kElevatorRightMotorId, MotorType.kBrushless);
    mRightEncoder = mRightMotor.getEncoder();
    mRightPIDController = mRightMotor.getClosedLoopController();
    mRightMotor.configure(
        elevatorConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

     // LEFT ELEVATOR MOTOR
     mLeftMotor = new SparkMax(Constants.Elevator.kElevatorLeftMotorId, MotorType.kBrushless);
     mLeftEncoder = mLeftMotor.getEncoder();
     mLeftPIDController = mLeftMotor.getClosedLoopController();
     mLeftMotor.configure(
         elevatorConfig.follow(mRightMotor),
         ResetMode.kResetSafeParameters,
         PersistMode.kPersistParameters);
 
     mProfile = new TrapezoidProfile(
         new TrapezoidProfile.Constraints(
             Constants.Elevator.kMaxVelocity,
             Constants.Elevator.kMaxAcceleration));
   }
 
   public enum ElevatorState {
     NONE,
     STOW,
     L2,
     L3,
     L4,
     A1,
     A2
   }
 
   private static class PeriodicIO {
     double elevator_target = 0.0;
     double elevator_power = 0.0;
 
     boolean is_elevator_pos_control = false;
 
     ElevatorState state = ElevatorState.STOW;
   }
 
   /*-------------------------------- Generic Subsystem Functions --------------------------------*/
 
   @Override
   public void periodic() {
    SmartDashboard.putNumber("Elevator/Speed Target", mPeriodicIO.elevator_target);
    SmartDashboard.putNumber("Elevator/Position/Current", mRightEncoder.getPosition());
    SmartDashboard.putNumber("Elevator/Position/Target", mPeriodicIO.elevator_target);
    writePeriodicOutputs();
     // TODO: Use this pattern to only drive slowly when we're really high up
     // if(mPivotEncoder.getPosition() > Constants.kPivotScoreCount) {
     // mPeriodicIO.is_pivot_low = true;
     // } else {
     // mPeriodicIO.is_pivot_low = false;
     // }
   }
 
   public void writePeriodicOutputs() {
     double curTime = Timer.getFPGATimestamp();
     double dt = curTime - prevUpdateTime;
     prevUpdateTime = curTime;
     if (mPeriodicIO.is_elevator_pos_control) {
       // Update goal
       mGoalState.position = mPeriodicIO.elevator_target;
 
       // Calculate new state
       prevUpdateTime = curTime;
       mCurState = mProfile.calculate(dt, mCurState, mGoalState);
 
       // Set PID controller to new state
       mRightPIDController.setReference(
           mCurState.position,
           SparkBase.ControlType.kPosition,
           ClosedLoopSlot.kSlot0,
           Constants.Elevator.kG,
           ArbFFUnits.kVoltage);
     } else {
       mCurState.position = mRightEncoder.getPosition();
       mCurState.velocity = 0;
       mRightMotor.set(mPeriodicIO.elevator_power);
     }
   }
 
   public void stop() {
     mPeriodicIO.is_elevator_pos_control = false;
     mPeriodicIO.elevator_power = 0.0;
 
     mRightMotor.set(0.0);
   }
 
   
   public void outputTelemetry() {

     SmartDashboard.putNumber("Position/Current", mRightEncoder.getPosition());
     SmartDashboard.putNumber("Position/Target", mPeriodicIO.elevator_target);
     SmartDashboard.putNumber("Velocity/Current", mRightEncoder.getVelocity());
 
     SmartDashboard.putNumber("Position/Setpoint", mCurState.position);
     SmartDashboard.putNumber("Velocity/Setpoint", mCurState.velocity);
 
     SmartDashboard.putNumber("Current/Left", mLeftMotor.getOutputCurrent());
     SmartDashboard.putNumber("Current/Right", mRightMotor.getOutputCurrent());
     SmartDashboard.putNumber("elevator power", mPeriodicIO.elevator_power);
 
     SmartDashboard.putNumber("Output/Left", mLeftMotor.getAppliedOutput());
     SmartDashboard.putNumber("Output/Right", mRightMotor.getAppliedOutput());
 
     //SmartDashboard.putNumber("State", mPeriodicIO.state);
   }

   public void reset() {
     mRightEncoder.setPosition(0.0);
   }
 
   /*---------------------------------- Custom Public Functions ----------------------------------*/
 
   public ElevatorState getState() {
     return mPeriodicIO.state;
   }
   /**
    * Sets the power of the elevator(probably)
    * @param power
    */
   public void setElevatorPower(double power) {
     SmartDashboard.putNumber("setElevatorPower", power);
     mPeriodicIO.is_elevator_pos_control = false;
     mPeriodicIO.elevator_power = power;
   }
 
   public void goToElevatorStow() {
     mPeriodicIO.is_elevator_pos_control = true;
     mPeriodicIO.elevator_target = Constants.Elevator.kStowHeight;
     mPeriodicIO.state = ElevatorState.STOW;
   }

   public Command goToElevatorStowCommand(){
    return this.run(()->this.goToElevatorStow())
    .until(()->Math.abs(Constants.Elevator.kStowHeight - mRightEncoder.getPosition())<Constants.Elevator.kAcceptableError);
   } 
 
   public void goToElevatorL2() {
     mPeriodicIO.is_elevator_pos_control = true;
     mPeriodicIO.elevator_target = Constants.Elevator.kL2Height;
     mPeriodicIO.state = ElevatorState.L2;
   }

   public Command goToElevatorL2Command(){
    return this.run(()->this.goToElevatorL2())
    .until(()->Math.abs(Constants.Elevator.kL2Height - mRightEncoder.getPosition())<Constants.Elevator.kAcceptableError);
   }
 
   public void goToElevatorL3() {
     mPeriodicIO.is_elevator_pos_control = true;
     mPeriodicIO.elevator_target = Constants.Elevator.kL3Height;
     mPeriodicIO.state = ElevatorState.L3;
   }

   public Command goToElevatorL3Command(){
    return this.run(()->this.goToElevatorL3())
    .until(()->Math.abs(Constants.Elevator.kL3Height - mRightEncoder.getPosition())<Constants.Elevator.kAcceptableError);
   }
 
   public void goToElevatorL4() {
     mPeriodicIO.is_elevator_pos_control = true;
     mPeriodicIO.elevator_target = Constants.Elevator.kL4Height;
     mPeriodicIO.state = ElevatorState.L4;
   }

   public Command goToElevatorL4Command(){
    return this.run(()->this.goToElevatorL4())
    .until(()->Math.abs(Constants.Elevator.kL4Height - mRightEncoder.getPosition())<Constants.Elevator.kAcceptableError);
   }
 
   public void goToAlgaeLow() {
     mPeriodicIO.is_elevator_pos_control = true;
     mPeriodicIO.elevator_target = Constants.Elevator.kLowAlgaeHeight;
     mPeriodicIO.state = ElevatorState.A1;
   }
 
   public void goToAlgaeHigh() {
     mPeriodicIO.is_elevator_pos_control = true;
     mPeriodicIO.elevator_target = Constants.Elevator.kHighAlgaeHeight;
     mPeriodicIO.state = ElevatorState.A2;
   }

   public void disabledInit (){
    this.stop();
   }
 
   /*---------------------------------- Custom Private Functions ---------------------------------*/   
}