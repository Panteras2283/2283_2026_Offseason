// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SimpleShoot extends Command {
private Shooter s_Shooter;
private Kicker s_Kicker;
private Spindexer s_Spindexer;
private double power;

  /** Creates a new SimpleShoot. */
  public SimpleShoot(Shooter s_Shooter, Kicker s_Kicker, Spindexer s_Spindexer, double power) {
    this.s_Shooter = s_Shooter;
    this.s_Kicker = s_Kicker;
    this.s_Spindexer = s_Spindexer;
    this.power = power;

    addRequirements(s_Shooter, s_Kicker, s_Spindexer);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    s_Shooter.setRPM(true, power);
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
