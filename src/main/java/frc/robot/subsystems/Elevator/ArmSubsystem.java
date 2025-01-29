package frc.robot.subsystems.Elavator;

//Spencer Anthony wrote this code
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;


import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;



import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;


import edu.wpi.first.wpilibj.DutyCycleEncoder;//changed


import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileSubsystem;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.controller.ArmFeedforward;


// for refrence; not currently in use
public class ArmSubsystem extends SubsystemBase{
    private final int SMART_MOTION_SLOT = 0;
      // Offset in rotations to add to encoder value - offset from arm horizontal to sensor zero
    private final SparkMax armLeader = new SparkMax(ArmConstants.kArmSparkMaxCANID1, MotorType.kBrushless);
    private final SparkMax armFollower = new SparkMax(ArmConstants.kArmSparkMaxCANID2, MotorType.kBrushless);
    private final SparkMaxConfig armLeaderConfig = new SparkMaxConfig();
    private final SparkMaxConfig armFollowerConfig = new SparkMaxConfig();
    //private final RelativeEncoder encoder1 = armLeader.getEncoder();
    //private final RelativeEncoder encoder2 = armFollower.getEncoder();
    private final DutyCycleEncoder armEnc = new DutyCycleEncoder(ArmConstants.kEncoderID, ArmConstants.kEncoderDistancePerRotation, ArmConstants.kArmOffsetRads);
    /*private final ArmFeedforward m_feedforward =
        new ArmFeedforward(
            ArmConstants.kSVolts, ArmConstants.kGVolts,
            ArmConstants.kVVoltSecondPerRad, ArmConstants.kAVoltSecondSquaredPerRad);*/

    //private Double targetPosition = null;
    /**
     * 
     */
      /** The shooter subsystem for the robot. */
    public ArmSubsystem() {
        armLeaderConfig
            .idleMode(IdleMode.kBrake)
            .inverted(true);
        armFollowerConfig.follow(armLeader, true);

        armLeader.configure(armLeaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        armLeader.configure(armLeaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
    @Override
    public void periodic() {
        //SmartDashboard.putNumber("arm Position Radians", getArmPositionRadians());
        SmartDashboard.putNumber("arm possition", getArmAngleRadians());
    }

    public void moveArmAtSpeed(double speed){
        //targetPosition = null;
        armLeader.set(speed);
        armFollower.set(-speed);
        SmartDashboard.putNumber("arm speed", speed);
        //System.out.println("this");
    }
    
    public double getArmAngleRadians(){
        return armEnc.get();
    }

    public Command moveToPosCommand(double position){
        return this.run(()->{
            //while(Math.abs(armEnc.getAbsolutePosition()-position)> ArmConstants.allowedErr){
                /*armLeader.set(-MathUtil.clamp(ArmConstants.kP*(Math.abs(armEnc.getAbsolutePosition()-position))+ArmConstants.holdArmPower, 
                ArmConstants.kMaxUpSpeed, ArmConstants.kMaxDownSpeed));*/
                this.moveArmAtSpeed(MathUtil.clamp(ArmConstants.kP*(position-getArmAngleRadians()), -0.2, 0.2));
                SmartDashboard.putNumber("goal", position);
                SmartDashboard.putNumber("error", position-getArmAngleRadians());
            //}
        }).unless(()->position>ArmConstants.kMaxUpPos || position < ArmConstants.kMaxDownPos)
        .until(()->Math.abs(getArmAngleRadians()-position)< ArmConstants.allowedErr)
        .andThen(this.run(()->this.moveArmAtSpeed(MathUtil.clamp(ArmConstants.kP*(position-getArmAngleRadians()), -0.2, 0.2))).withTimeout(0.2));
    }

    /**
    * Convert from arm position in radians to encoder rotations
    * @param armRadians arm position in radians
    * @return equivilant encoder position, in rotations
    */
    static double armRadiansToEncoderRotations(double armRadians) {
        return Units.radiansToRotations(armRadians) - ArmConstants.ENCODER_OFFSET;
    }

    /**
    * Stop the elevator
    */
    public void stop() {
        //targetPosition = null;
        armLeader.stopMotor();
    }

        /**
     *sets arm to a possition
     * 
     * @param armMoveControl control to move the arm with, + is up, - is down
     *
     * 
     */
    public void setArmVelocity(double armMoveControl){
        //targetPosition = null;
        double shootArmAxis;
        if ((getArmAngleRadians()>= ArmConstants.kMaxUpPos && armMoveControl >0)||(getArmAngleRadians()<= ArmConstants.kMaxDownPos && armMoveControl<0)){
            shootArmAxis = MathUtil.clamp(ArmConstants.holdArmPower*Math.cos(getArmAngleRadians()), ArmConstants.kMaxDownSpeed, ArmConstants.kMaxUpSpeed);
        } else if ((getArmAngleRadians()>= ArmConstants.kMaxUpPos-0.1 && armMoveControl >0.6)||(getArmAngleRadians()<= ArmConstants.kMaxDownPos+0.1 && armMoveControl<-0.1)){
            shootArmAxis = MathUtil.clamp(armMoveControl*0.25+(ArmConstants.holdArmPower*Math.cos(getArmAngleRadians())), ArmConstants.kMaxDownSpeed, ArmConstants.kMaxUpSpeed);
        } else if((getArmAngleRadians() <= Math.PI/2 && armMoveControl <0)||(getArmAngleRadians()>=Math.PI/2 &&armMoveControl>0)){
            shootArmAxis = MathUtil.clamp(armMoveControl*0.37+(ArmConstants.holdArmPower*Math.cos(getArmAngleRadians())), ArmConstants.kMaxDownSpeed, ArmConstants.kMaxUpSpeed);
        } else {
            shootArmAxis = MathUtil.clamp(armMoveControl+(ArmConstants.holdArmPower*Math.cos(getArmAngleRadians())), ArmConstants.kMaxDownSpeed, ArmConstants.kMaxUpSpeed); // Apply axis clamp and invert for driver control
        }
        armLeader.set(shootArmAxis * ArmConstants.kArmRate);
        //armFollower.set(armMoveControl);
        
        SmartDashboard.putNumber("speed", armMoveControl);
        SmartDashboard.putNumber("shoot arm move control", shootArmAxis);
        SmartDashboard.putNumber("arm power", shootArmAxis * ArmConstants.kArmRate); // put arm speed on Smartdash
        SmartDashboard.putNumber("arm vel", armLeader.getAbsoluteEncoder().getVelocity());
        SmartDashboard.putNumber("arm enc test", armLeader.getEncoder().getPosition());
    }

    public Command moveArm(double input){
        return this.run(()->this.setArmVelocity(input));
    }
   
    public Command disableArm () {
        return this.startEnd(() -> this.setArmVelocity(0), () -> this.setArmVelocity(0));
    }

    public Command stopArm(){
        return this.runOnce(()->this.stop());
    }

}