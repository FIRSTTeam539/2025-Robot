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

    private final SparkMax climb = new SparkMax(ClimbConstants.kClimbSparkMaxCANID, MotorType.kBrushless);
    private final SparkMaxConfig config = new SparkMaxConfig();
    private final RelativeEncoder Encoder = climb.getEncoder();


    public ClimbSubsystem(){
        config
            .idleMode(IdleMode.kBrake)
            .inverted(true);
        climb.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        //climbRight.follow(climbLeft, true);
        // l  307
    }

    //public double 
    @Override
    public void periodic() {
        SmartDashboard.putNumber("climb/climber possition", Encoder.getPosition());
    }
    
    public void setClimb(double rate){
        climb.set(rate);
    }
    public Command climbCommand(double rate){
        return this.run(()->{
            climb.set(rate);
            SmartDashboard.putNumber("climb/clim rate", rate);
        }).finallyDo(()->this.stop());
    }
    public Command holdCommand(){
        return this.run(()->this.setClimb(ClimbConstants.kStaticArmRate));
    }
    public Command stop(){
        return this.startEnd(()->setClimb(0), ()->setClimb(0));
    }
}
