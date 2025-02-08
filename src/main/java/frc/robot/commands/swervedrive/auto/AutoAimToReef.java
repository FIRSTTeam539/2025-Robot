package frc.robot.commands.swervedrive.auto;

import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

public class AutoAimToReef extends Command{
    SwerveSubsystem swerve;
    double x;
    double y;
    AutoAimToReef(SwerveSubsystem swerve){
        this.swerve = swerve;

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
        //make it go to the correct position

        x=swerve.getPose().getX();
        y=swerve.getPose().getY();

        swerve.driveToPose(new Pose2d(new Translation2d(0,0), new Rotation2d(0)));
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
        return false;
    }
}
