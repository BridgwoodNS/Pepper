// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;

public class Vision extends SubsystemBase {
  boolean isVisionEnabled = true;
  boolean useMegaTag2 = true;
  boolean doRejectUpdate = false;
  CommandSwerveDrivetrain m_drive;
  String ll = "limelight";


   private final NetworkTable table = NetworkTableInstance.getDefault().getTable("Pose");
  private final DoubleArrayPublisher limelightPub = table.getDoubleArrayTopic("llPose").publish();



  /** Creates a new Vision. */
  public Vision(CommandSwerveDrivetrain drive) {
    m_drive = drive;




  }

  public void setVisionEnabled(boolean useVision) {
    isVisionEnabled = useVision;
  }

  public void setUseMegaTag2(boolean useTwo) {
    useMegaTag2 = useTwo;
  }

  public void setRejectUpdate(boolean reject) {
    doRejectUpdate = reject;
  }

  public void updateOdometry() {

     if(useMegaTag2 == false)
    {
      LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");
      
      if(mt1.tagCount == 1 && mt1.rawFiducials.length == 1)
      {
        if(mt1.rawFiducials[0].ambiguity > .7)
        {
          doRejectUpdate = true;
        }
        if(mt1.rawFiducials[0].distToCamera > 3)
        {
          doRejectUpdate = true;
        }
      }
      if(mt1.tagCount == 0)
      {
        doRejectUpdate = true;
      }

      if(!doRejectUpdate)
      {
        m_drive.setVisionMeasurementStdDevs(VecBuilder.fill(.5,.5,9999999));
        m_drive.addVisionMeasurement(
            mt1.pose,
            mt1.timestampSeconds);
            publishToField(mt1);
      }
    }
    else if (useMegaTag2 == true)
    {
      LimelightHelpers.SetRobotOrientation("limelight", m_drive.getState().Pose.getRotation().getDegrees(), 0, 0, 0, 0, 0);
      LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
      if(Math.abs(m_drive.getState().Speeds.omegaRadiansPerSecond) > 720 * 0.017453) // if our angular velocity is greater than 720 degrees per second, ignore vision updates
      {
        doRejectUpdate = true;
      }
      if(mt2.tagCount == 0)
      {
        doRejectUpdate = true;
      }
      if(!doRejectUpdate)
      {
        m_drive.setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,9999999));
        m_drive.addVisionMeasurement(
            mt2.pose,
            mt2.timestampSeconds);
            publishToField(mt2);
            

      }
    }
    
  
  }

  public double getTxOfNearestAprilTag() {
    return LimelightHelpers.getTX(ll);
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if(DriverStation.isDisabled()){
      blinkLEDS().schedule();
      useMegaTag2 = false;
      
    }else
    {
      ledsOn().schedule();
      useMegaTag2 = true;
    }

    if(isVisionEnabled)
    {
      updateOdometry();
    }
  }


  public Command blinkLEDS() {
    return runOnce(() -> LimelightHelpers.setLEDMode_ForceBlink(ll))
        .andThen(Commands.waitSeconds(2))
        .andThen(() -> LimelightHelpers.setLEDMode_ForceOff(ll));
  }


  public Command ledsOff() {
    return runOnce(() -> LimelightHelpers.setLEDMode_ForceOff(ll));
  }

  public Command ledsOn() {
    return runOnce(() -> LimelightHelpers.setLEDMode_ForceOn(ll));
  }



  private void publishToField(LimelightHelpers.PoseEstimate limelightMeasurement) {
    // If you have a Field2D you can easily push it that way here.
    limelightPub.set(new double[] {
      limelightMeasurement.pose.getX(),
      limelightMeasurement.pose.getY(),
      limelightMeasurement.pose.getRotation().getDegrees()
    });
  }

}
