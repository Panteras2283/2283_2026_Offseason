// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import gg.questnav.questnav.QuestNav;
import gg.questnav.questnav.PoseFrame;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Rotation3d;



public class Questnav extends SubsystemBase {
  QuestNav questNav = new QuestNav();
  Transform3d ROBOT_TO_QUEST = new Transform3d(0.0, 0.0, 0.0, new Rotation3d());
  PoseFrame[] poseFrames = questNav.getAllUnreadPoseFrames();
  

  /** Creates a new Questnav. */
  public Questnav() {
    if (poseFrames.length > 0) {
    Pose3d questPose = poseFrames[poseFrames.length - 1].questPose3d();
    Pose3d robotPose = questPose.transformBy(ROBOT_TO_QUEST.inverse());
  }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    questNav.commandPeriodic();

  }
}
