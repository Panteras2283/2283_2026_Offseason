// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {

  private TalonFX elevatorRight = new TalonFX(Constants.Climber.elevatorRightID);
  private TalonFX elevatorLeft = new TalonFX(Constants.Climber.elevatorLeftID);
 // private TalonFX Foot = new TalonFX(Constants.Climber.FootID);

  private final MotionMagicVoltage elevatorRightRequest = new MotionMagicVoltage(0); 
  private final MotionMagicVoltage elevatorLeftRequest = new MotionMagicVoltage(0);
  //private final MotionMagicVoltage FootRequest = new MotionMagicVoltage(0); 
  /** Creates a new Climber. */
  public Climber() {
    configureMotionMagic();
  }

  private void configureMotionMagic(){
    TalonFXConfiguration cfgMm = new TalonFXConfiguration();

    cfgMm.Slot0.kP = 0.1;   
    cfgMm.Slot0.kI = 0.0;
    cfgMm.Slot0.kD = 0.0;
    cfgMm.Slot0.kV = 0.12;       
    cfgMm.Slot0.kS = 0.2;

    cfgMm.MotionMagic.MotionMagicCruiseVelocity = 150;
    cfgMm.MotionMagic.MotionMagicAcceleration = 150;
    cfgMm.MotionMagic.MotionMagicJerk = 0;

    cfgMm.CurrentLimits.StatorCurrentLimit = 60.0;
    cfgMm.CurrentLimits.StatorCurrentLimitEnable = true;

    elevatorRight.getConfigurator().apply(cfgMm);
    elevatorLeft.getConfigurator().apply(cfgMm);

    elevatorRight.setNeutralMode(NeutralModeValue.Brake);
    elevatorLeft.setNeutralMode(NeutralModeValue.Brake);

    elevatorRight.setPosition(0);
    elevatorLeft.setPosition(0);

    elevatorLeft.setControl(new Follower(Constants.Climber.elevatorRightID, MotorAlignmentValue.Opposed));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("RightElevatorPos", elevatorRight.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("LeftElevatorPos", elevatorLeft.getPosition().getValueAsDouble());
  }

  public void setElevatorPos(double position){
    elevatorRight.setControl(elevatorRightRequest.withPosition(position));
  }

  public double getElevatorPosition() {
    return elevatorRight.getPosition().getValueAsDouble();
  }

 /*  public void setFootPos(double position){
    Foot.setControl(FootRequest.withPosition(position));
  }*/

  /*public void FootOut(){
   Foot.setControl(FootRequest.withPosition(Constants.Climber.FootOutPos));
  }

  public void FootIn(){
   Foot.setControl(FootRequest.withPosition(Constants.Climber.FootInPos));
  }*/
  
  public void resetEncoders(){
    elevatorRight.setControl(elevatorRightRequest.withPosition(0));
    //Foot.setControl(FootRequest.withPosition(0));
  }

  public void fullDown(){
    elevatorRight.set(-0.1);
    //Foot.set(-0.1);
    System.out.println("ELEVATOR DOWN");
  }
  public void fullStop(){
    elevatorRight.set(0);
    //Foot.set(0);
  }
  // Add this method inside Climber.java
  public Command fullDownCommand() {
    return this.runOnce(this::fullDown);
  }
}
