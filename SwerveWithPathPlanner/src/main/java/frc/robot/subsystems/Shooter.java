// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkBase.ControlType;
import com.ctre.phoenix6.controls.VoltageOut;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Utils.ShootingTables;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */

  private final TalonFX FlywheelRight;
  private final TalonFX FlywheelLeft;
  private double currentRPM = 0;



  private final VelocityVoltage flywheelRequest = new VelocityVoltage(0); 
  private final VoltageOut voltageRequest = new VoltageOut(0);

  private String Shooter;

  private double flywheelTargetRPM = 0;

  private final VoltageOut coastRequest = new VoltageOut(0);

  
  
  private static final double RPM_TOLERANCE_PERCENT = 0.15;

  public Shooter(int flywheelCanID, int flywheelCanID2, String Shooter) {
    this.Shooter = Shooter;

    FlywheelRight = new TalonFX(flywheelCanID, "rio");
    configureFlywheel();
    FlywheelLeft = new TalonFX(flywheelCanID2, "rio");
    configureFlywheel2();
    
  }
  private void configureFlywheel(){
    TalonFXConfiguration cfg = new TalonFXConfiguration();
    cfg.CurrentLimits.StatorCurrentLimit = 150.0;
    cfg.CurrentLimits.StatorCurrentLimitEnable = true;
    cfg.Slot0.kP = 0.45;
    cfg.Slot0.kI = 0;
    cfg.Slot0.kV = 0.12;
    cfg.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    FlywheelRight.getConfigurator().apply(cfg);
    FlywheelRight.setNeutralMode(NeutralModeValue.Coast);
    }

    private void configureFlywheel2(){
    TalonFXConfiguration cfg2 = new TalonFXConfiguration();
    cfg2.CurrentLimits.StatorCurrentLimit = 150.0;
    cfg2.CurrentLimits.StatorCurrentLimitEnable = true;
    cfg2.Slot0.kP = 0.45;
    cfg2.Slot0.kI = 0;
    cfg2.Slot0.kV = 0.12;

    FlywheelLeft.getConfigurator().apply(cfg2);
    FlywheelLeft.setNeutralMode(NeutralModeValue.Coast);
    FlywheelLeft.setControl(new Follower(Constants.Shooter.motorID, MotorAlignmentValue.Opposed));
    }

  public void setRPM(boolean forcePrecise, double targetRPM){
      this.flywheelTargetRPM = targetRPM;
      double targetRPS = flywheelTargetRPM / 60.0;

      // Ensure 0 RPM physically stops the motors
      if (targetRPM == 0) {
          FlywheelRight.setControl(voltageRequest.withOutput(0));
          return;
      }

      if(forcePrecise){
        FlywheelRight.setControl(flywheelRequest.withVelocity(targetRPS));
      }else{
        if(currentRPM > flywheelTargetRPM + 100){
          FlywheelRight.setControl(voltageRequest.withOutput(0));
        }else{
           FlywheelRight.setControl(flywheelRequest.withVelocity(targetRPS)); 
        }
      }
  }

  public void setTargetRPM(boolean forcePrecise, double distanceMeters){
    double mappedRPM = ShootingTables.FlywheelMap.get(distanceMeters);
    setRPM(forcePrecise, mappedRPM);
    }

 
  

  /*public void setTargetDistance(double distanceMeters){


    if (flywheelTargetRPM > 100){ 
      double targetRPS = flywheelTargetRPM / 60;
      FlywheelRight.setControl(flywheelRequest.withVelocity(targetRPS));
    } else{
      stop();
    }
  }*/

  public void stop(){
    setRPM(false, 0);
  }

  public boolean isReadyToFire(){
    double flywheelVelRPM = FlywheelRight.getVelocity().getValueAsDouble() * 60;

    double flywheelError = Math.abs(flywheelTargetRPM - flywheelVelRPM);

    boolean flywheelReady =  flywheelError < (flywheelTargetRPM * RPM_TOLERANCE_PERCENT);

    return flywheelReady && (flywheelTargetRPM > 100);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  double currentRPM = FlywheelRight.getVelocity().getValueAsDouble()*60;

  SmartDashboard.putNumber(Shooter + "/FlywheelRightRPM", FlywheelRight.getVelocity().getValueAsDouble() * 60.0);
  SmartDashboard.putNumber(Shooter + "/FlywheelLeftRPM", FlywheelRight.getVelocity().getValueAsDouble() * 60.0);
  SmartDashboard.putNumber(Shooter + "/ShooterTargetRPM", flywheelTargetRPM);

  SmartDashboard.putBoolean(Shooter + "/Ready", isReadyToFire());
  }
}
