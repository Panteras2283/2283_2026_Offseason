// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Set;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Climber;
import frc.robot.Constants;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Climb extends SequentialCommandGroup {

  private Climber s_Climber;
  private static final double POSITION_TOLERANCE = 1.0;
  /** Creates a new Climb. */
  public Climb(Climber s_Climber) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addRequirements(s_Climber);
    addCommands(
      climbCycle(),
      climbCycle(),
      climbCycle(),
      new InstantCommand(()-> s_Climber.setElevatorPos(Constants.Climber.DownPos), s_Climber),
      new WaitUntilCommand(()-> isAtPosition(Constants.Climber.DownPos))
      
      /*new InstantCommand(() -> s_Climber.setElevatorPos(Constants.Climber.DownPos), s_Climber),
      new WaitUntilCommand(() -> isAtPosition(Constants.Climber.DownPos)),
      
      new InstantCommand(() -> s_Climber.setElevatorPos(Constants.Climber.UpPos), s_Climber),
      new WaitUntilCommand(() -> isAtPosition(Constants.Climber.UpPos)),

      climbCycle(),

      climbCycle(),

      new InstantCommand(() -> s_Climber.setElevatorPos(Constants.Climber.elevatorFootPos), s_Climber),
      new WaitUntilCommand(() -> isAtPosition(Constants.Climber.elevatorFootPos)),
      new InstantCommand(() -> {
        s_Climber.setElevatorPos(Constants.Climber.elevatorFootPos);
        s_Climber.FootOut();
      }, s_Climber)

    */);
  }
  /*private SequentialCommandGroup climbCycle() {
    return new SequentialCommandGroup(
      new InstantCommand(() -> s_Climber.setElevatorPos(Constants.Climber.elevatorFootPos), s_Climber),
      new WaitUntilCommand(() -> isAtPosition(Constants.Climber.elevatorFootPos)),

      new InstantCommand(() -> {
        s_Climber.setElevatorPos(Constants.Climber.elevatorFootPos);
        s_Climber.FootOut();
      }, s_Climber),
      
      new WaitCommand(0.1), 
      
      new InstantCommand(() -> {
        s_Climber.setElevatorPos(Constants.Climber.UpPos);
        s_Climber.FootIn();
      }, s_Climber),
      new WaitUntilCommand(() -> isAtPosition(Constants.Climber.UpPos))
    );
  }*/
  private SequentialCommandGroup climbCycle(){
    return new SequentialCommandGroup(
      new InstantCommand(()->s_Climber.setElevatorPos(Constants.Climber.DownPos), s_Climber),
      new WaitUntilCommand(()->isAtPosition(Constants.Climber.DownPos)),
      new InstantCommand(()->s_Climber.setElevatorPos(Constants.Climber.UpPos), s_Climber),
      new WaitUntilCommand(()->isAtPosition(Constants.Climber.UpPos))
    );
  }
  private boolean isAtPosition(double targetPos) {
    return Math.abs(s_Climber.getElevatorPosition() - targetPos) <= POSITION_TOLERANCE;
  }
}
