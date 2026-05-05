// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import gg.questnav.questnav.QuestNav;
import gg.questnav.questnav.PoseFrame;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Questnav extends SubsystemBase {
    
    private final QuestNav questNav = new QuestNav();
    private final CommandSwerveDrivetrain drivetrain; // Change this to your actual swerve subsystem class name-

    // TODO: Measure exactly where the Quest is relative to the center of your robot.
    // X = Forward/Back, Y = Left/Right, Z = Up/Down (in meters)
    private static final Transform3d ROBOT_TO_QUEST = new Transform3d(0.0, 0.0, 0.0, new Rotation3d());
    
    // Standard Deviations: How much we trust the X, Y, and Rotational data
    private static final Matrix<N3, N1> QUESTNAV_STD_DEVS = VecBuilder.fill(0.02, 0.02, 0.035);

    // Constructor requires passing in your swerve subsystem
    public Questnav(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;

        // --- Safety Callbacks ---
        questNav.onConnected(() -> System.out.println("Quest connected!"));
        questNav.onDisconnected(() -> DriverStation.reportWarning("Quest disconnected!", false));
        questNav.onTrackingAcquired(() -> System.out.println("Quest tracking acquired!"));
        questNav.onTrackingLost(() -> DriverStation.reportWarning("Quest tracking lost! Falling back to Odometry.", false));
    }

    @Override
    public void periodic() {
        // CRITICAL: This must run every loop for commands and callbacks to work
        questNav.commandPeriodic();

        // Fetch unread frames from the headset
        PoseFrame[] questFrames = questNav.getAllUnreadPoseFrames();

        for (PoseFrame frame : questFrames) {
            // Ensure the headset is actually tracking safely
            if (frame.isTracking()) {
                Pose3d questPose = frame.questPose3d();
                double timestamp = frame.dataTimestamp();

                // Calculate the robot's center position by reversing the offset
                Pose3d robotPose = questPose.transformBy(ROBOT_TO_QUEST.inverse());

                // Add to your Swerve Drive Pose Estimator
                // NOTE: Depending on your Swerve library (YAGSL/AdvantageKit), this method name might be slightly different.
                drivetrain.addVisionMeasurement(
                    robotPose.toPose2d(), 
                    timestamp, 
                    QUESTNAV_STD_DEVS
                );
            }
        }
    }
    
    // Optional: Call this to reset the headset to a known starting point (like auto start)
    public void resetPose(Pose3d knownRobotPose) {
        Pose3d questPose = knownRobotPose.transformBy(ROBOT_TO_QUEST);
        questNav.setPose(questPose);
    }
}