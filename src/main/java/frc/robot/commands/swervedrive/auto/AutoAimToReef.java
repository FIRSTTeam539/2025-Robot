package frc.robot.commands.swervedrive.auto;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.LimelightConstants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.RawFiducial;

public class AutoAimToReef extends Command{
    SwerveSubsystem swerve;
    double x;
    double y;
    double distToRobot;
    double distToCamera;
    double ambiguity;
    public AutoAimToReef(SwerveSubsystem swerve){
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
        RawFiducial[] fiducials = LimelightHelpers.getRawFiducials("");
        for (RawFiducial fiducial : fiducials) {
            int id = fiducial.id;                    // Tag ID
            double txnc = fiducial.txnc;             // X offset (no crosshair)
            double tync = fiducial.tync;             // Y offset (no crosshair)
            double ta = fiducial.ta;                 // Target area
            this.distToCamera = fiducial.distToCamera;  // Distance to camera
            this.distToRobot = fiducial.distToRobot;    // Distance to robot
            ambiguity = fiducial.ambiguity;   // Tag pose ambiguity
        }
        /*swerve.drive(new Translation2d(DriveConstants.kPAprilTagTran*(distToRobot-1),
        0),
        0,
        false);*/
        LimelightHelpers.setFiducial3DOffset("limelight", 
                0.0,    // Forward offset
                DriveConstants.kSideOff,    // Side offset  
                0.0     // Height offset
                );
        if(distToRobot > 3){
            swerve.drive(new Translation2d(DriveConstants.kPAprilTagTranDist*(distToCamera-DriveConstants.kDistOffset), 0), 
            (-DriveConstants.kPAprilTagRot*(LimelightHelpers.getTX(LimelightConstants.kLimelightName)-DriveConstants.kRotOffset)),
             false);
        } else {
            swerve.drive(new Translation2d(DriveConstants.kPAprilTagTranDist*(distToCamera-DriveConstants.kDistOffset),
            DriveConstants.kPAprilTagTranSide*(LimelightHelpers.getBotPose_TargetSpace(LimelightConstants.kLimelightName)[0]-DriveConstants.kSideOff)),
            (-DriveConstants.kPAprilTagRot*(LimelightHelpers.getTX(LimelightConstants.kLimelightName)-DriveConstants.kRotOffset)),
            false);
        }

        SmartDashboard.putNumber("ambiguity", ambiguity);
        SmartDashboard.putNumber("distToRobot", distToRobot);


        //SmartDashboard.putNumber("Aim TX", LimelightHelpers.getBotPose_TargetSpace(LimelightConstants.kLimelightName)[0]);


    
  }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted)
    {
        swerve.drive(new Translation2d(0,0), 0,false);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished()
    {   if (Math.abs(LimelightHelpers.getTX(LimelightConstants.kLimelightName)-DriveConstants.kRotOffset)< 0.1 
        && Math.abs(distToCamera-DriveConstants.kDistOffset)<0.05 
        && Math.abs(LimelightHelpers.getBotPose_TargetSpace(LimelightConstants.kLimelightName)[0]-DriveConstants.kSideOff)<0.2){
            return true;
    }
        return false;
    }
}
