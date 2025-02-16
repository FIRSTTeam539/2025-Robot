package frc.robot.commands.swervedrive.auto;

import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.LimelightConstants;


public class TurnToAprilTagCommand extends Command{
    private final SwerveSubsystem swerve;
    private final double offset;

    public TurnToAprilTagCommand(SwerveSubsystem swerve, double offset){
        this.swerve = swerve;
        this.offset = offset;

        addRequirements(swerve);
    }

    @Override
    public void initialize()
    {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute()
    {
        swerve.drive(new Translation2d(0, 0), (-DriveConstants.kPAprilTagRot*(LimelightHelpers.getTX(LimelightConstants.kLimelightName)-offset)), false);
        //swerve.drive(new Translation2d(0, 0), (-DriveConstants.kPAprilTagRot*10*(LimelightHelpers.getTargetPose_RobotSpace(LimelightConstants.kLimelightName)[1]-offset)), false);
        
        /*if (limelight.getTV()){
            swerve.drive(swerve.getTargetSpeeds(0 ,0, Rotation2d.fromDegrees(-limelight.getTX())));   
        }*/
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted)
    {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished()
    {
        if (LimelightHelpers.getTX(LimelightConstants.kLimelightName)< 0.1-offset && LimelightHelpers.getTX(LimelightConstants.kLimelightName)> -0.1-offset){
            return true;
        /* else if (Math.abs(limelight.getTX())<=DriveConstants.allowedAutoAimErrorRadians){
            return true;*/
        }
        return false;
    }

}
