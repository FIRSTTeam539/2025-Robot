package frc.robot.commands.swervedrive.auto;

import frc.robot.Constants;
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
import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;

public class AutoAimToReef extends Command{
    SwerveSubsystem swerve;
    double x;
    double y;
    double distToRobot;
    double distToCamera;
    double ambiguity;
    private LaserCan mLaserCAN;
    boolean forward = false;
    public AutoAimToReef(SwerveSubsystem swerve){
        this.swerve = swerve;
         mLaserCAN = new LaserCan(DriveConstants.kDistLaserId);
        try {
            mLaserCAN.setRangingMode(LaserCan.RangingMode.SHORT);
            mLaserCAN.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 16, 16));
            mLaserCAN.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
        } catch (ConfigurationFailedException e) {
            System.out.println("Configuration failed! " + e);
        }

        addRequirements(swerve);
    }

    @Override
    public void initialize()
    {
        forward = false;
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
        SmartDashboard.putBoolean("Controls/forward", forward);
        SmartDashboard.putNumber("Controls/Dist", mLaserCAN.getMeasurement().distance_mm);
        if(distToRobot > 3){
            swerve.drive(new Translation2d(DriveConstants.kPAprilTagTranDist*(distToCamera-DriveConstants.kDistOffset), 0), 
            (-DriveConstants.kPAprilTagRot*(LimelightHelpers.getTX(LimelightConstants.kLimelightName)-DriveConstants.kRotOffset)),
             false);
        } else {
            if (((distToCamera-DriveConstants.kDistOffset)< 0.1
            && Math.abs(LimelightHelpers.getTX(LimelightConstants.kLimelightName)-DriveConstants.kRotOffset)< 1
            && Math.abs(LimelightHelpers.getBotPose_TargetSpace(LimelightConstants.kLimelightName)[0]-DriveConstants.kSideOff)<DriveConstants.kAcceptableErrorSide)) {
                forward = true;
            }
            if (forward == false){
                swerve.drive(new Translation2d(DriveConstants.kPAprilTagTranDist*(distToCamera-DriveConstants.kDistOffset),
                DriveConstants.kPAprilTagTranSide*(LimelightHelpers.getBotPose_TargetSpace(LimelightConstants.kLimelightName)[0]-DriveConstants.kSideOff)),
                (-DriveConstants.kPAprilTagRot*(LimelightHelpers.getTX(LimelightConstants.kLimelightName)-DriveConstants.kRotOffset)),
                false);
            } else {

                swerve.drive(new Translation2d(
                    (((double) mLaserCAN.getMeasurement().distance_mm)/1000 - 0.35)*2*DriveConstants.kPAprilTagTranDist, 0), 
                    0,
                    false
                );
            }
            //swerve.drive(new Translation2d(0, 0), 0, false);
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
    {   if (Math.abs(LimelightHelpers.getTX(LimelightConstants.kLimelightName)-DriveConstants.kRotOffset)< 1 
        && Math.abs((((double) mLaserCAN.getMeasurement().distance_mm)/1000 - 0.5))<0.01 
        && Math.abs(LimelightHelpers.getBotPose_TargetSpace(LimelightConstants.kLimelightName)[0]-DriveConstants.kSideOff)<DriveConstants.kAcceptableErrorSide){
            return true;
        } else if (
            !LimelightHelpers.getTV(LimelightConstants.kLimelightName)
            && Math.abs((((double) mLaserCAN.getMeasurement().distance_mm)/1000 - 0.35))<0.01){
                return true;
        }
        return false;
    }
}
