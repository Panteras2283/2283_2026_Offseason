// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.jar.Attributes.Name;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.*;
import frc.robot.generated.TunerConstants;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.Constants;

public class RobotContainer {
    //Constants
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    private double FeedingMaxSpeed = 3.5;
    private double ShootingMaxSpeed = 1;

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);
    private final CommandXboxController operator = new CommandXboxController(1);

    //Subsystems
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final Intake s_Intake;
    public final LEDs s_LEDs = new LEDs();
    public final Kicker s_Kicker = new Kicker();
    //public final Climber s_Climber;
    public final Spindexer s_Spindexer = new Spindexer();
    public final TurretSubsystem s_Turret = new TurretSubsystem(25, "Turret");
    public final Shooter s_Shooter = new Shooter(Constants.Shooter.motorID, Constants.Shooter.motor2ID, "Shooter");

    public final Superstructure superstructure = new Superstructure(s_Turret, s_Shooter,
    ()->drivetrain.getState().Pose,
    ()->drivetrain.getState().Speeds, operator, s_Kicker, s_Spindexer, s_LEDs);
    
    public final VisionSubsystem s_Vision;
    
    


    /* Path follower */
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {

        
        
        //Subsystems
        s_Intake = new Intake(s_LEDs);
        s_Vision = new VisionSubsystem(drivetrain);
        //s_Climber = new Climber();

        //s_Climber.setDefaultCommand(new ClimberDefault(s_Climber));

        
        NamedCommands.registerCommand("Feed", new RunCommand(()-> s_Intake.Down(), s_Intake));
        NamedCommands.registerCommand("toggleShoot", new InstantCommand(()-> superstructure.toggleShooting()));
        NamedCommands.registerCommand("shake feeder", new ShakeFeeder(s_Intake).repeatedly());
        NamedCommands.registerCommand("Feedn't", new InstantCommand(()-> s_Intake.stop(), s_Intake));
        NamedCommands.registerCommand("toggleIdle", new InstantCommand(()-> superstructure.toggleIdle()));


        autoChooser = AutoBuilder.buildAutoChooser("Tests");
        SmartDashboard.putData("Auto Mode", autoChooser);


        
        configureBindings();

        // Warmup PathPlanner to avoid Java pauses
        FollowPathCommand.warmupCommand().schedule();
    }
    private double getDynamicMaxSpeed() {
        if (superstructure.shooting || superstructure.wantsShoot) {
            return ShootingMaxSpeed;
        } 
        else if (s_Intake.isFeeding) {
            return FeedingMaxSpeed;
        }
        return MaxSpeed;
    }
    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.

        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() -> {
                double currentMaxSpeed = getDynamicMaxSpeed();
                return drive.withVelocityX(-joystick.getLeftY() * currentMaxSpeed) 
                    .withVelocityY(-joystick.getLeftX() * currentMaxSpeed) 
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate); 
            })
        );
        /*drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );*/

        superstructure.setDefaultCommand(new RunCommand(() -> superstructure.runStateMachine(),superstructure, s_Turret, s_Shooter, s_Kicker, s_Spindexer));
        s_Intake.setDefaultCommand(new IntakeDefault(s_Intake));

        /*s_Turret.setDefaultCommand(
            new RunCommand(() -> {
                // Read left X axis. Adjust to getLeftY() if you prefer up/down instead of left/right
                double joystickVal = operator.getLeftX(); 
                s_Turret.setTargetAngle(Rotation2d.fromDegrees(joystickVal * -160.0));
            }, s_Turret)
        );*/
      // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

  

        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));
        // reset the field-centric heading on left bumper press
        joystick.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));


        //Drive to Pose (Precise Positioning)
        joystick.rightBumper().whileTrue(
            new Precise_DriveToPose_cmd(
                drivetrain, 
                Constants.AutopilotConstants.kPathConstraints, 
                () -> 0, 
                () -> 2, 
                joystick, 
                joystick
            )
        );

            /*Turret */
       operator.leftBumper().onTrue(new InstantCommand(superstructure::toggleIdle));
       operator.rightBumper().onTrue(new InstantCommand(superstructure::toggleShooting)); 

            /*Intake */
       operator.pov(180).onTrue(new InstantCommand(()->s_Intake.Down()));
       operator.pov(180).onFalse(new InstantCommand(()->s_Intake.stop()));
       operator.pov(0).onTrue(new InstantCommand(()->s_Intake.outake()));
       operator.pov(0).onFalse(new InstantCommand(()->s_Intake.stop()));

       operator.pov(270).onTrue(new InstantCommand(()->s_Intake.up()));

       operator.a().onTrue(new ShakeFeeder(s_Intake).repeatedly());
       operator.a().onFalse(s_Intake.getDefaultCommand());

       /*Spindexer */
       operator.x().onTrue(new InstantCommand(()->superstructure.toggleUnjam(true)));
       operator.x().onFalse(new InstantCommand(()->superstructure.toggleUnjam(false)));

       /*Shooter */
       operator.y().whileTrue(new ShootOveride(s_Shooter, s_Spindexer, superstructure, s_Kicker));

       /*Robot Unjam*/
       operator.start().whileTrue(new RobotUnjam(s_Intake, s_Spindexer));


            
            /*Climber */
       //operator.leftStick().whileTrue(s_Climber.fullDownCommand());
        //operator.leftStick().onFalse(s_Climber.getDefaultCommand());
        //operator.rightStick().onTrue(new InstantCommand(()-> s_Climber.resetEncoders(), s_Climber));
        //operator.rightStick().onFalse(s_Climber.getDefaultCommand());
        
        //operator.y().onTrue(new Climb(s_Climber));
        //operator.y().onFalse(s_Climber.getDefaultCommand());


        
        

        //Drive to Pose (Fast Positioning)
        joystick.leftBumper().whileTrue(
            new Fast_DriveToPose_cmd(
                drivetrain, 
                Constants.AutopilotConstants.kPathConstraints, 
                () -> 0, 
                () -> 0, 
                joystick, 
                joystick
            )
        );

        joystick.pov(0).whileTrue(new Fast_DriveToPose_cmd(drivetrain, Constants.AutopilotConstants.kPathConstraints,()->1, ()->1, joystick, operator));



        

        


        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }
}
