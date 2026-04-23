// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.*;
import frc.robot.Constants;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShakeFeeder extends SequentialCommandGroup {
  private Intake s_Intake;
  private static final double POSITION_TOLERANCE = 1.0;
  /** Creates a new ShakeFeeder. */
  public ShakeFeeder(Intake s_Intake) {
    this.s_Intake = s_Intake;
    addRequirements(s_Intake);
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(()-> s_Intake.setPosition(Constants.Intake.LeftShakeUpPos, Constants.Intake.RightShakeUpPos)),
      new WaitUntilCommand(()-> isAtPosition(Constants.Intake.LeftShakeUpPos, Constants.Intake.RightShakeUpPos)),
      new InstantCommand(()-> s_Intake.setPosition(Constants.Intake.LeftShakeDownPos, Constants.Intake.RightShakeDownPos)),
      new WaitUntilCommand(()-> isAtPosition(Constants.Intake.LeftShakeDownPos, Constants.Intake.RightShakeDownPos))
    );
  }
  private boolean isAtPosition(double targetPosLeft, double targetPosRight) {
    return ((Math.abs(s_Intake.getLeftPosition() - targetPosLeft) <= POSITION_TOLERANCE) && (Math.abs(s_Intake.getRightPosition() - targetPosRight) <= POSITION_TOLERANCE));
  }
}
