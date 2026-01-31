// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;

public class LimeLightSubsystem extends SubsystemBase {

  public NetworkTable m_limeLightRightTable; 
   public NetworkTable m_limeLightTopTable;
   public NetworkTable m_limeLightLeftTable; 
   public Pose2d m_closestTagPose;
  // Limelight Left: http://10.37.39.11:5801/
  //Limelight Right: http://10.37.39.12:5801/
  
  /** Creates a new LimeLightSubsystem. */
  public LimeLightSubsystem() {
    m_limeLightRightTable = NetworkTableInstance.getDefault().getTable("limelight-right");
    m_limeLightTopTable = NetworkTableInstance.getDefault().getTable("limelight-top");
    m_limeLightLeftTable = NetworkTableInstance.getDefault().getTable("limelight-left");
    m_limeLightRightTable.getEntry("pipeline").setNumber(0);
    // SmartDashboard.putData("Field", m_field);

  }
  public Pose2d getBotPoseRightLL(){
    if(DriverStation.getAlliance().get() == Alliance.Red)
      m_limeLightRightTable.getEntry("pipeline").setNumber(0);
    else
      m_limeLightRightTable.getEntry("pipeline").setNumber(1);

    double[] botRotArray = m_limeLightRightTable.getEntry("botpose").getDoubleArray(new double[10]); 
    double[] botPoseArray = m_limeLightRightTable.getEntry("botpose_orb").getDoubleArray(new double[10]); 
    Pose2d botPose;
      if(DriverStation.getAlliance().get() == Alliance.Red)  botPose = new Pose2d(botPoseArray[0]+8.7736, botPoseArray[1]+4.0257, Rotation2d.fromDegrees(botRotArray[5] + 180));
      else botPose = new Pose2d(botPoseArray[0] + 8.7736, botPoseArray[1] + 4.0257, Rotation2d.fromDegrees(botRotArray[5]));
      return botPose;

  }

  public Pose2d getRobotRelativeTargetPose(){
    double[] targetPoseArray = m_limeLightRightTable.getEntry("targetpose_robotspace").getDoubleArray(new double[10]);
    Pose2d targetPose = new Pose2d(targetPoseArray[2], targetPoseArray[0], Rotation2d.fromDegrees(targetPoseArray[4])); 

    return targetPose;
  }

  public int getRightID(){
    return ((int) m_limeLightRightTable.getEntry("tid").getDouble(-1));
  }

  public int getLeftID(){
    return ((int) m_limeLightLeftTable.getEntry("tid").getDouble(-1));
  }

  public int getTopID(){
    return ((int) m_limeLightTopTable.getEntry("tid").getDouble(-1));
  }

  
  public int getTopIDCount(){
    return ((int) m_limeLightTopTable.getEntry("botpose_orb").getDoubleArray(new double[10])[7]);

  }

  public int getRightIDCount(){
    return ((int) m_limeLightRightTable.getEntry("botpose_orb").getDoubleArray(new double[10])[7]);

  }

  public int getLeftIDCount(){
    return ((int) m_limeLightLeftTable.getEntry("botpose_orb").getDoubleArray(new double[10])[7]);

  }
  public double getRightLimelightTime(){
    return LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-right").timestampSeconds;
  }
  public double getLeftLimelightTime(){
    return LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-left").timestampSeconds;
  }
  public double getTopLimelightTime(){
    return LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelihgt-top").timestampSeconds;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Limelight Left Pipline", m_limeLightLeftTable.getEntry("pipeline").getNumber(-1).doubleValue());
    SmartDashboard.putNumber("Limelight Right Pipline", m_limeLightRightTable.getEntry("pipeline").getNumber(-1).doubleValue());
  }

  @Override
  public void simulationPeriodic() {
  }
}