package frc.robot.subsystems.climber;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;

public class ClimbSubsystem extends SubsystemBase{

    private final SparkMax climbLeft = new SparkMax(ClimbConstants.kClimbSparkMaxCANIDLeft, MotorType.kBrushless);
    private final SparkMax climbRight = new SparkMax(ClimbConstants.kClimbSparkMaxCANIDRight, MotorType.kBrushless);
    private final SparkMaxConfig configRight = new SparkMaxConfig();
    private final SparkMaxConfig configLeft = new SparkMaxConfig();
    private final RelativeEncoder LEncoder = climbLeft.getEncoder();
    private final RelativeEncoder REncoder = climbRight.getEncoder();


    public ClimbSubsystem(){
        configRight.idleMode(IdleMode.kBrake);
        configLeft
            .idleMode(IdleMode.kBrake)
            .inverted(true);
        climbLeft.configure(configLeft, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        climbRight.configure(configRight, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        //climbRight.follow(climbLeft, true);
        // l  307
    }

    //public double 
    @Override
    public void periodic() {
        SmartDashboard.putNumber("left climber possition", LEncoder.getPosition());
        SmartDashboard.putNumber("right climber possition", REncoder.getPosition());
    }
    
    public void setClimb(double rate){
        climbLeft.set(-rate);
        climbRight.set(-rate);
    }
    public void setDualClimb(double lrate, double rrate){
        /*if (!(LEncoder.getPosition() <=0 && -lrate <0)){
            climbLeft.set(-lrate);
        }
        if (!(REncoder.getPosition() <= 0 && -rrate <0)){
            climbRight.set(-rrate);
        }*/
        climbLeft.set(-lrate);
        climbRight.set(-rrate);
    }
    public Command climbRightCommand(double rate){
        return this.run(()->{
            climbRight.set(-rate);
        });
    }
    public Command climbLeftCommand(double rate){
        return this.run(()->{
            climbLeft.set(-rate);
        });
    }
    public Command holdCommand(){
        return this.run(()->this.setClimb(ClimbConstants.kStaticArmRate));
    }
    public Command stop(){
        return this.startEnd(()->setClimb(0), ()->setClimb(0));
    }

    public Command climbCommand(double rate){
        /*if (rate >= ClimbConstants.kStaticArmRate && rate <= 0) {
            return this.run(()->setClimb(ClimbConstants.kStaticArmRate));
        }*/
        return this.run(()->{
            climbLeft.set(-rate);
            climbRight.set(-rate);
        });
    }
}
