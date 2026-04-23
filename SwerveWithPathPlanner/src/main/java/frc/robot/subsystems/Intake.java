// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;

import frc.robot.Constants;
import frc.robot.Utils.ShootingTables;


public class Intake extends SubsystemBase {
 /** Creates a new Intake_demo. */
  private TalonFX FeederRight = new TalonFX(Constants.Intake.FeederRightID);
  private TalonFX FeederLeft = new TalonFX(Constants.Intake.FeederLeftID);
  private TalonFX pivotLeft = new TalonFX(Constants.Intake.PivotLeftID);
  private TalonFX pivotRight = new TalonFX(Constants.Intake.PivotRightID);

  private final MotionMagicVoltage leftPivotRequest = new MotionMagicVoltage(0);
  private final MotionMagicVoltage rightPivotRequest = new MotionMagicVoltage(0);
  private final VelocityVoltage feederRequest = new VelocityVoltage(0); 
  private final LEDs leds;


  private double intakeTargetRPM = 0;

  public boolean isFeeding = false;
  
  public Intake(LEDs leds) {
    configureMotionMagic();
    configureFeeder();
    this.leds = leds;
  }

  private void configureFeeder(){
    TalonFXConfiguration cfgF = new TalonFXConfiguration();
    cfgF.CurrentLimits.StatorCurrentLimit = 150.0;
    cfgF.CurrentLimits.StatorCurrentLimitEnable = true;
    cfgF.Slot0.kP = 0.33;
    cfgF.Slot0.kV = 0.12;

    FeederRight.getConfigurator().apply(cfgF);
    FeederLeft.getConfigurator().apply(cfgF);
    FeederRight.setNeutralMode(NeutralModeValue.Coast);
    FeederLeft.setNeutralMode(NeutralModeValue.Coast);

    FeederLeft.setControl(new Follower(Constants.Intake.FeederRightID, MotorAlignmentValue.Opposed));
    }

  private void configureMotionMagic(){
    TalonFXConfiguration cfgMm = new TalonFXConfiguration();

    cfgMm.Slot0.kP = 20.0;   
    cfgMm.Slot0.kI = 0.0;
    cfgMm.Slot0.kD = 0.0;
    cfgMm.Slot0.kV = 0.12;       
    cfgMm.Slot0.kS = 0.2;

    cfgMm.MotionMagic.MotionMagicCruiseVelocity = 180;
    cfgMm.MotionMagic.MotionMagicAcceleration = 230;
    cfgMm.MotionMagic.MotionMagicJerk = 0;

    cfgMm.CurrentLimits.StatorCurrentLimit = 60.0;
    cfgMm.CurrentLimits.StatorCurrentLimitEnable = true;

    pivotLeft.getConfigurator().apply(cfgMm);
    pivotRight.getConfigurator().apply(cfgMm);

    pivotLeft.setNeutralMode(NeutralModeValue.Brake);
    pivotRight.setNeutralMode(NeutralModeValue.Brake);

    pivotLeft.setPosition(0);
    pivotRight.setPosition(0);
  }
    
  @Override
  public void periodic() {
    SmartDashboard.putNumber("LeftPivotPos", pivotLeft.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("RightPivotPos", pivotRight.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("FeederRightVel", FeederRight.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("FeederCurrent", FeederRight.getStatorCurrent().getValueAsDouble());
    SmartDashboard.putNumber("FeederLeftVel", FeederLeft.getVelocity().getValueAsDouble());
  }

  public void setNeutralMode(NeutralModeValue mode) {
    pivotLeft.setNeutralMode(mode);
    pivotRight.setNeutralMode(mode);
  }

  public void setPosition(double posLeft, double posRight){
    pivotLeft.setControl(leftPivotRequest.withPosition(posLeft));
    pivotRight.setControl(rightPivotRequest.withPosition(posRight));
  }


  public void Down(){
    isFeeding = true;
    pivotLeft.setControl(leftPivotRequest.withPosition(Constants.Intake.LeftFeedPos));
    pivotRight.setControl(rightPivotRequest.withPosition(Constants.Intake.RightFeedPos));
    FeederRight.setControl(feederRequest.withVelocity(50));
    leds.Feed();
  }  

  public void feed(){
    isFeeding = true;
    FeederRight.setControl(feederRequest.withVelocity(50));
    leds.Feed(); 
  } 

  public void up(){
    isFeeding = false;
    pivotLeft.setControl(leftPivotRequest.withPosition(Constants.Intake.LeftUpPos));
    pivotRight.setControl(rightPivotRequest.withPosition(Constants.Intake.RightUpPos));
  }

  public void outake(){
    isFeeding = false;
    FeederRight.set(-0.5);
    
  } 

  public double getRightPosition() {
    return pivotRight.getPosition().getValueAsDouble();
  }

  public double getLeftPosition(){
    return pivotLeft.getPosition().getValueAsDouble();
  }

  public void stop(){
    isFeeding = false;
    FeederRight.set(0);
    leds.Default();
  }
}
