// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;

public class Vision extends SubsystemBase {
  boolean isVisionEnabled = true;
  boolean useMegaTag2 = true;
  boolean doRejectUpdate = false;
  CommandSwerveDrivetrain m_drive;



  /** Creates a new Vision. */
  public Vision(CommandSwerveDrivetrain drive) {
    m_drive = drive;




  }

  public void setVisionEnabled(boolean enabled) {
    isVisionEnabled = enabled;
  }

  public void setUseMegaTag2(boolean use) {
    useMegaTag2 = use;
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
      }
    }
  
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
