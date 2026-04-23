// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;

public class Spindexer extends SubsystemBase {

  private SparkFlex SpindexerMotor = new SparkFlex(Constants.Spindexer.SpindexermotorID, MotorType.kBrushless);
  private SparkFlexConfig SpindexerConfig = new SparkFlexConfig();
  private SparkClosedLoopController spindexerClosedLoopController = SpindexerMotor.getClosedLoopController();
  private RelativeEncoder SpindexerEncoder = SpindexerMotor.getEncoder();

  public double SpindexerCurrent = 0;

  public boolean jammed = false;
 

  /** Creates a new Spindexer. */
  public Spindexer() {

    SpindexerConfig.closedLoop.pid(Constants.Spindexer.kP, Constants.Spindexer.kI, Constants.Spindexer.kD);
    SpindexerConfig.closedLoop.feedForward.kV(Constants.Spindexer.kFF);
    SpindexerConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);

    SpindexerMotor.configure(SpindexerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
}
  

  @Override
  public void periodic() {
    double SpindexerRPM = Math.abs(SpindexerEncoder.getVelocity()); // Use Math.abs
    double SpindexerCurrent = SpindexerMotor.getOutputCurrent(); 
    
    
    if(SpindexerCurrent > 200 && SpindexerRPM < 20) {
        jammed = true;
    } else {
        jammed = false;
    }

    SmartDashboard.putBoolean("Spindexer Jammed", jammed);
  }

  
  // In frc.robot.subsystems.Spindexer.java
  public void setVelocity(double rpm) {
    // Use kVelocity control type to maintain a specific speed
    spindexerClosedLoopController.setSetpoint(
        rpm, 
        ControlType.kVelocity, 
        ClosedLoopSlot.kSlot0
    );
  }

  public void SpinCW(){
    this.setVelocity(4500);
  }

  public void SpinCCW(){
    this.setVelocity(-1000);
  }

  public void Spin(double speed){
    this.setVelocity(speed);
  }

  public void stop(){
    SpindexerMotor.set(0);
  }
}
