// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Climber;
import frc.robot.Constants;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Climb_L1 extends SequentialCommandGroup {
  private Climber s_Climber;
  private static final double POSITION_TOLERANCE = 1.0;
  /** Creates a new Climb_L1. */
  public Climb_L1(Climber s_Climber) {
    addRequirements(s_Climber);
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(()-> s_Climber.setElevatorPos(Constants.Climber.DownPos)),
      new WaitUntilCommand(()->isAtPosition(Constants.Climber.DownPos)),
      new InstantCommand(()-> s_Climber.setElevatorPos(Constants.Climber.UpPos)),
      new WaitUntilCommand(()->isAtPosition(Constants.Climber.UpPos)),

      new InstantCommand(()-> s_Climber.setElevatorPos(Constants.Climber.DownPos)),
      new WaitUntilCommand(()->isAtPosition(Constants.Climber.DownPos))
    );
  }

  private boolean isAtPosition(double targetPos) {
    return Math.abs(s_Climber.getElevatorPosition() - targetPos) <= POSITION_TOLERANCE;
  }
}
