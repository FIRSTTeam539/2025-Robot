package frc.robot.commands.swervedrive.auto;

import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;


public class TurnToAprilTagCommand extends Command{
    private final LimelightSubsystem limelight;
    private final SwerveSubsystem swerve;
    private final double offset;

    public TurnToAprilTagCommand(SwerveSubsystem swerve, LimelightSubsystem limelight, double offset){
        this.limelight = limelight;
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
        swerve.drive(new Translation2d(0, 0), (-DriveConstants.kPAprilTag*(limelight.getTX()-offset)), false);
        
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
        if (limelight.getTX()< 0.1-offset && limelight.getTX()> -0.1-offset){
            return true;
        /* else if (Math.abs(limelight.getTX())<=DriveConstants.allowedAutoAimErrorRadians){
            return true;*/
        }
        return false;
    }

}
