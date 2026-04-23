// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Utils.ShootingPhysics;
import frc.robot.Utils.ShootingPhysics.AimingSolution;
import frc.robot.Utils.ShootingTables;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;

import java.lang.Thread.State;
import java.util.function.Supplier;


public class Superstructure extends SubsystemBase {
  /** Creates a new Superstructure. */

  private final TurretSubsystem turret;
  private final Shooter shooter;
  private final Kicker kicker;
  private final Spindexer spindexer;
  private final LEDs leds; 
  //private final ShooterSubsystem shooter;

  private final Supplier<Pose2d> poseSupplier;
  private final Supplier<ChassisSpeeds> speedSupplier;

  private final Translation2d TURRET_OFFSET = new Translation2d(-0.15, 0.185);

  private final Translation2d BLUE_TARGET = new Translation2d(4.554, 4.068);
  private final Translation2d RED_TARGET = new Translation2d(11.9, 4);
  private final Translation2d BLUE_NSRTARGET = new Translation2d(2.78, 2.07);
  private final Translation2d BLUE_NSLTARGET = new Translation2d(2.78, 6.25);
  private final Translation2d RED_NSRTARGET = new Translation2d(13.84, 6.31);
  private final Translation2d RED_NSLTARGET = new Translation2d(13.84, 2.07);
  private final double middle_y = 4.00;
  private Translation2d currentTarget = BLUE_TARGET;
  public boolean wantsUnjam = false;
  private boolean autoUnjamming = false;
  private final Timer unjamTimer = new Timer();

  private final CommandXboxController operator;

  public boolean isTurretLockedOn = false;

  public boolean wantsIDLE = false;
  public boolean wantsShoot = false;
  public boolean shooting = false;
  public boolean idle = false;
  private boolean hasSpunUp = false;
  public AimingSolution solution;

  //public boolean shooting = false;

  private final StructPublisher<Pose2d> turretTargetPub;

  private double operatorOffset = 0;

  public final double IDLERPM = 1500;

  public enum States{
    OFF,
    IDLE,
    SHOOTING,
    UNJAM
  }


  public Superstructure(TurretSubsystem turret, Shooter shooter, 
  Supplier<Pose2d> poseSupplier, Supplier<ChassisSpeeds> speedSupplier, 
  CommandXboxController operator, Kicker kicker, Spindexer spindexer, LEDs leds) {
    this.turret = turret;
    this.poseSupplier = poseSupplier;
    this.speedSupplier = speedSupplier;
    this.operator = operator;
    this.shooter = shooter;
    this.kicker = kicker;
    this.spindexer = spindexer;
    this.leds = leds;


    
    var table = NetworkTableInstance.getDefault().getTable("Superstructure");
    turretTargetPub = table.getStructTopic("Turret", Pose2d.struct).publish();
  }

  public void toggleIdle(){
    wantsIDLE = !wantsIDLE;
  }

  public void toggleShooting(){
    wantsShoot = !wantsShoot;
  }

  public void toggleUnjam(boolean unjam) {
    wantsUnjam = unjam;
  }

  public double getErrorDegrees(){
    Rotation2d rotations = turret.getCurrentAngle();
    AimingSolution solution = calculateAiming();
    Rotation2d targetAngle = solution.turretAngle();
    Rotation2d errorDeg = rotations.minus(targetAngle);
    return errorDeg.getDegrees();
  }

  /*public Superstructure(TurretSubsystem turret, ShooterSubsystem shooter, Supplier<Pose2d> poseSupplier, Supplier<ChassisSpeeds> speedSupplier) {
    this.turret = turret;
    this.shooter = shooter;   
    this.poseSupplier = poseSupplier;
    this.speedSupplier = speedSupplier;
    
    var table = NetworkTableInstance.getDefault().getTable("Superstructure");
    turretTargetPub = table.getStructTopic("Turret", Pose2d.struct).publish();
  } */


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    Pose2d robotPose = poseSupplier.get();
    ChassisSpeeds robotSpeeds = speedSupplier.get();
    
    operatorOffset = operator.getLeftX();

    var alliance = DriverStation.getAlliance();


    if(alliance.isPresent() && alliance.get() == Alliance.Blue){
      if((robotPose.getX() >= BLUE_TARGET.getX()) && robotPose.getY() >= middle_y){
        currentTarget = BLUE_NSLTARGET;
      }else if((robotPose.getX() >= BLUE_TARGET.getX()) && robotPose.getY() < middle_y){
        currentTarget = BLUE_NSRTARGET;
      }else{
        currentTarget = BLUE_TARGET;
      }
    }else{
      if((robotPose.getX() <= RED_TARGET.getX()) && robotPose.getY() >= middle_y){
        currentTarget = RED_NSRTARGET;
      }else if((robotPose.getX() <= RED_TARGET.getX()) && robotPose.getY() < middle_y){
        currentTarget = RED_NSLTARGET;
      }else{
        currentTarget = RED_TARGET;
      }
    }


    

    SmartDashboard.putNumber("ErrorDeg", getErrorDegrees());
   

    

    isTurretLockedOn = runAimingLoop(
      turret, shooter, robotPose, robotSpeeds, TURRET_OFFSET, currentTarget, "Turret", turretTargetPub
    );

    }
    
    public void runStateMachine() {
    States state = States.OFF;
    if(wantsShoot && spindexer.jammed && !autoUnjamming){
      autoUnjamming = true;
      unjamTimer.restart();
    }
    if(autoUnjamming && unjamTimer.hasElapsed(0.3)){
      autoUnjamming = false;
      unjamTimer.stop();
    }if(wantsUnjam || autoUnjamming){
      state = States.UNJAM;
    }else if(wantsShoot){
      state = States.SHOOTING;
    }else if(wantsIDLE){
      state = States.IDLE;
    }else{
      state = States.OFF;
    }

    SmartDashboard.putString("Superstructure-state", state.toString());
    

    switch (state){
      case OFF:
        handleOFF();
        break;
      case IDLE:
        handleIDLE();
        break;
      case SHOOTING:
        handleSHOOTING();
        break;
      case UNJAM:
        handleUNJAM();
        break;
    }
  }    
    public void handleOFF(){
      shooting = false;
      idle = false;
      hasSpunUp = false;
      turret.setTargetAngle(new Rotation2d(0));
      shooter.setRPM(false, 0);
      kicker.stop();
      spindexer.stop();
      leds.Default();
    }

    public void handleIDLE(){
      shooting = false;
      idle = true;
      hasSpunUp = false;
      AimingSolution solution = calculateAiming();
      turret.setTargetAngle(solution.turretAngle());
      //shooter.setRPM(false, IDLERPM);
      shooter.stop();
      spindexer.stop();
      kicker.stop();
      leds.idle();
    }

    private void handleSHOOTING(){
      AimingSolution solution = calculateAiming();
      Rotation2d targetAngle = solution.turretAngle();
      idle = false;
      shooting = true;
      if(Math.abs(operatorOffset) > 0.05) {
        targetAngle = targetAngle.plus(Rotation2d.fromDegrees(operatorOffset * 10));
        turret.setTargetAngle(targetAngle);
      } else {
        turret.setTargetAngle(targetAngle);
      }

    

      //shooter.setRPM(true, 3000);
      shooter.setTargetRPM(true, solution.effectiveDistance());

      boolean shooterReady = shooter.isReadyToFire();
      boolean turretLocked = Math.abs(getErrorDegrees()) < 2.0;

      if (shooterReady) {
          hasSpunUp = true;
      }
      //boolean locked = turretLocked && shooterReady;
      boolean locked = turretLocked && hasSpunUp;


      SmartDashboard.putBoolean("Superstructure-Locked", locked);
      SmartDashboard.putNumber(turret + "/ErrorDeg", getErrorDegrees());

      if (locked) {
        kicker.Kick(0.85);
        spindexer.SpinCW(); 
        leds.RTF();
      } else {
        leds.Default();
        kicker.stop();
        spindexer.stop();
      }
      
    }

    private void handleUNJAM() {
      shooting = false;
      idle = false;
      hasSpunUp = false;
    
      shooter.stop();
      kicker.stop();
      spindexer.SpinCCW(); 
    
      leds.Antijam(); 
  }

    public AimingSolution calculateAiming(){
      Pose2d robotPose = poseSupplier.get();
      ChassisSpeeds robotSpeeds = speedSupplier.get();

      double rawDistance = robotPose.getTranslation().getDistance(currentTarget);
      double estimatedExitVel = ShootingTables.ExitVelocityMap.get(rawDistance);
      
      AimingSolution solution = ShootingPhysics.calculateAimingSolution(
          robotPose, robotSpeeds, TURRET_OFFSET, currentTarget, estimatedExitVel
      );

      Rotation2d fixedAngle = solution.turretAngle();
      solution = new AimingSolution(fixedAngle, solution.effectiveDistance(), solution.virtualTarget());

      turretTargetPub.set(solution.virtualTarget());
      
      return solution;
    }

    private boolean runAimingLoop(TurretSubsystem turret, Shooter shooter, Pose2d robotPose, ChassisSpeeds robotSpeeds, Translation2d offset, Translation2d targetLocation, String sideName, StructPublisher<Pose2d> publisher){
      double rawDistance = robotPose.getTranslation().getDistance(targetLocation);
      double estimatedExitVel = ShootingTables.ExitVelocityMap.get(rawDistance);
      
      // FIX 1: We use "this.solution =" instead of "AimingSolution solution ="
      // This saves the math to the public variable so ShootOveride.java can read it!
      this.solution = ShootingPhysics.calculateAimingSolution(robotPose, robotSpeeds, offset, targetLocation, estimatedExitVel);

      publisher.set(this.solution.virtualTarget());

      double targetRPM = ShootingTables.FlywheelMap.get(this.solution.effectiveDistance());

      boolean turretAtTarget = Math.abs(getErrorDegrees()) < 2.0;
      boolean shooterAtSpeed = shooter.isReadyToFire();
      
      boolean locked = turretAtTarget;
      SmartDashboard.putBoolean(sideName + "/Locked", locked);
      SmartDashboard.putBoolean("Shooting", shooting);
      SmartDashboard.putBoolean("Idle", idle);
      SmartDashboard.putBoolean("hasSpunUp", hasSpunUp);

      // FIX 2: Updated references for SmartDashboard
      SmartDashboard.putNumber(sideName + "/Aim/Dist_Effective", this.solution.effectiveDistance());
      SmartDashboard.putNumber(sideName + "/Aim/Target_Angle", this.solution.turretAngle().plus(Rotation2d.fromDegrees(operatorOffset * 10)).getDegrees());
      SmartDashboard.putNumber(sideName + "/Aim/RPM/_Top", targetRPM);
      
      return locked;
  }

  /*private boolean runAimingLoop(TurretSubsystem turret, ShooterSubsystem shooter, Pose2d robotPose, ChassisSpeeds robotSpeeds, Translation2d offset, Translation2d targetLocation, String sideName, StructPublisher<Pose2d> publisher){
      double rawDistance = robotPose.getTranslation().getDistance(targetLocation);
      double estimatedExitVel = ShootingTables.ExitVelocityMap.get(rawDistance);
      AimingSolution solution = ShootingPhysics.calculateAimingSolution(robotPose, robotSpeeds, offset, targetLocation, estimatedExitVel);

      publisher.set(solution.virtualTarget());

      double targetRPM = ShootingTables.FlywheelMap.get(solution.effectiveDistance());

      SmartDashboard.putNumber(sideName + "/Aim/Dist_Effective", solution.effectiveDistance());
      SmartDashboard.putNumber(sideName + "/Aim/Target_Angle", solution.turretAngle().getDegrees());
      SmartDashboard.putNumber(sideName + "/Aim/RPM/_Top", targetRPM);

      turret.setTargetAngle(solution.turretAngle());

      shooter.setTargetDistance(solution.effectiveDistance());

      boolean turretAtTarget = Math.abs(turret.getErrorDegrees()) < 2.0;
      boolean shooterAtSpeed = shooter.isReadyToFire();

      boolean locked = turretAtTarget && shooterAtSpeed;
      SmartDashboard.putBoolean(sideName + "/Locked", locked);

      return locked;
  } */

  public boolean canShoot(){return isTurretLockedOn;}
}
