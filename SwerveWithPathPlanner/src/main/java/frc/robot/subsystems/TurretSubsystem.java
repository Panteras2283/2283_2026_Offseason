// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// CTRE Imports
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class TurretSubsystem extends SubsystemBase {
  /** Creates a new Turret. */

  private final TalonFX turretMotor;
  private final String Turret;
  
  // Motion Magic control request
  private final MotionMagicVoltage positionRequest = new MotionMagicVoltage(0);

  // Check with CAD
  private static final double GEAR_RATIO = 38.8889;

  private static final double SOFT_LIMIT_FWD_ROT = 0.48;
  private static final double SOFT_LIMIT_BWD_ROT = -0.48;

  private static final double AIM_TOLERANCE_DEG = 0.08;

  // IMPORTANT: These values will need to be retuned for Phoenix 6 / Kraken!
  private static final double kP = 50; 
  private static final double kI = 0.0;
  private static final double kD = 0.1;
  private static final double kFF = 4.67; // Acts as kV in Phoenix 6$

  private static final double kS = 0.50;

  // REV was in RPM, Phoenix 6 needs RPS (Rotations per second)
  private static final double maxVel_RPS = 2.5; 
  private static final double maxAcc_RPSps = 25.0; 

  public TurretSubsystem(int canId, String Turret) {
     this.Turret = Turret;
     this.turretMotor = new TalonFX(canId);

     
     configureTurret();
  }

  private void configureTurret(){
    TalonFXConfiguration config = new TalonFXConfiguration();

    // Configure Gear Ratio (1 motor rotation = 1/23.34 mechanism rotations)
    config.Feedback.SensorToMechanismRatio = GEAR_RATIO;
    
    // Soft Limits
    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = SOFT_LIMIT_FWD_ROT;
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = SOFT_LIMIT_BWD_ROT;
    
    // Motion Magic (Cruise velocity & Acceleration)
    config.MotionMagic.MotionMagicCruiseVelocity = maxVel_RPS;
    config.MotionMagic.MotionMagicAcceleration = maxAcc_RPSps;

    // PID & FeedForward
    config.Slot0.kP = kP;
    config.Slot0.kI = kI;
    config.Slot0.kD = kD;
    config.Slot0.kV = kFF; 
    config.Slot0.kS = kS;

    // Current limit & Voltage Comp
    config.CurrentLimits.StatorCurrentLimit = 60;
    config.CurrentLimits.StatorCurrentLimitEnable = true;

    // Apply configuration
    turretMotor.getConfigurator().apply(config);
    turretMotor.setNeutralMode(NeutralModeValue.Coast);

    // Reset position to 0 on boot
    turretMotor.setPosition(0);
  }

  public void setTargetAngle(Rotation2d targetAngle){
    double targetRotations = targetAngle.getRotations();

    targetRotations = MathUtil.inputModulus(targetRotations, -0.5, 0.5);
    targetRotations = MathUtil.clamp(targetRotations, SOFT_LIMIT_BWD_ROT, SOFT_LIMIT_FWD_ROT);

    // Send control request to the TalonFX
    turretMotor.setControl(positionRequest.withPosition(targetRotations));
  }

  public Rotation2d getCurrentAngle(){
    // Read position using Phoenix 6 API (already converted to mechanism rotations by the config)
    double rotations = turretMotor.getPosition().getValueAsDouble();
    return Rotation2d.fromRotations(rotations);
  }

  public void resetPosition(){
    turretMotor.setPosition(0);
  }

  @Override
  public void periodic() {
    double turretRPS = turretMotor.getVelocity().getValueAsDouble()*GEAR_RATIO;
    double turretRPM = turretRPS * 60;
    double turretAngle = turretMotor.getPosition().getValueAsDouble();
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("AngleDeg", getCurrentAngle().getDegrees());
    SmartDashboard.putNumber("TurretRPS", turretRPS);

  }
}