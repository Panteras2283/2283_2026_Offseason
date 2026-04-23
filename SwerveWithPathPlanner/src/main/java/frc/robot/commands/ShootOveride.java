// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ShootOveride extends Command {

  private Shooter s_Shooter;
  private Spindexer s_Spindexer;
  private Superstructure s_Superstructure;
  private Kicker s_Kicker;
  /** Creates a new ShootOveride. */
  public ShootOveride(Shooter s_Shooter, Spindexer s_Spindexer, Superstructure s_Superstructure, Kicker s_Kicker) {
    this.s_Shooter = s_Shooter;
    this.s_Kicker = s_Kicker;
    this.s_Spindexer = s_Spindexer;
    this.s_Superstructure = s_Superstructure;
    addRequirements(s_Kicker, s_Shooter, s_Spindexer, s_Superstructure);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    s_Shooter.setTargetRPM(true, s_Superstructure.solution.effectiveDistance());
    s_Kicker.Kick(0.85);
    s_Spindexer.SpinCW();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
