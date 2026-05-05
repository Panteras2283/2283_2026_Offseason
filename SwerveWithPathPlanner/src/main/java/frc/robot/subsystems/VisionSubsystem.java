package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.proto.Photon;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.io.IOException;
import java.util.Optional;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;
import gg.questnav.questnav.PoseFrame;

//QUESTNAV
/*import gg.questnav.questnav.QuestNav;
import gg.questnav.questnav.PoseFrame; */


public class VisionSubsystem extends SubsystemBase {
    private final CommandSwerveDrivetrain drivetrain;
    
    private static final String kLimelightName = "limelight-three";

    private final PhotonCamera photon1; 
   //private final PhotonCamera photon2;
    private PhotonPoseEstimator photonPoseEstimator1; 
    //private PhotonPoseEstimator photonPoseEstimator2;
    private static final String kPhotonCameraName1 = "arducam_left";
    //private static final String kPhotonCameraName2 = "arducam_right";
    private final boolean kUseLimelight = true;

    
    private static final Transform3d kRobotToCam1 = new Transform3d(
        new Translation3d(-0.34, 0.35025, 0.52), 
        new Rotation3d(0, Math.toRadians(0), Math.PI) 
    );

    //QUESTNAV
    //private final QuestNav questNav = new QuestNav(); 

    //private static final Transform3d ROBOT_TO_QUEST = new Transform3d(0.0, 0.0, 0.0, new Rotation3d());

    //private static final Matrix<N3, N1> QUESTNAV_STD_DEVS = VecBuilder.fill(0.02, 0.02, 0.035);






    /*private static final Transform3d kRobotToCam2 = new Transform3d(
        new Translation3d(0.29972, -0.32004, 0.5207), 
        new Rotation3d(0,0,0).rotateBy( new Rotation3d(0,Math.toRadians(-45),0))
    );*/

  

    public VisionSubsystem(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;


        SmartDashboard.putBoolean("arducam_left", true);
       // SmartDashboard.putBoolean("arducam_right", true);
        SmartDashboard.putBoolean("limelight-three", kUseLimelight);
        //Initialize cameras 
        photon1 = new PhotonCamera(kPhotonCameraName1);
        //photon2 = new PhotonCamera(kPhotonCameraName2);

        //Load field layout and set up pose estimator
        AprilTagFieldLayout fieldLayout = null;
        try {
            fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
        } catch (Exception e) {
            System.out.println("Failed to load AprilTagFieldLayout: " + e.getMessage());
        }
        
        if(fieldLayout != null){
            //Estimator for camera 1
            photonPoseEstimator1 = new PhotonPoseEstimator(
                fieldLayout,
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                kRobotToCam1
            );
            //Estimator for camera 2
           /*  photonPoseEstimator2 = new PhotonPoseEstimator(
                fieldLayout,
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                kRobotToCam2
            );*/
        }

        //QUESTNAV
        /*questNav.onConnected(() -> System.out.println("Quest connected!"));
        questNav.onDisconnected(() -> DriverStation.reportWarning("Quest disconnected!", false));
        questNav.onTrackingAcquired(() -> System.out.println("Quest tracking acquired!"));
        questNav.onTrackingLost(() -> DriverStation.reportWarning("Quest tracking lost! Falling back to Limelight/Odometry.", false));
        questNav.onLowBattery(20, level -> DriverStation.reportWarning("Quest battery low: " + level + "%", false));*/
        
    }


    @Override
    public void periodic() {
       updateLimelight();
       updatePhotonVision(photonPoseEstimator1, photon1);
       //updatePhotonVision(photonPoseEstimator2, photon2);


       //QUESTNAV
       /* 
       questNav.commandPeriodic();
       updateQuestNav();*/

    }

    /*private void updateQuestNav() {
        // Fetch unread frames from the headset
        PoseFrame[] questFrames = questNav.getAllUnreadPoseFrames();

        for (PoseFrame frame : questFrames) {
            // Ensure the headset is actually tracking safely
            if (frame.isTracking()) {
                Pose3d questPose = frame.questPose3d();
                double timestamp = frame.dataTimestamp();

                // Calculate the robot's center position by reversing the offset
                Pose3d robotPose = questPose.transformBy(ROBOT_TO_QUEST.inverse());

                // Add to your Swerve Drive Pose Estimator
                drivetrain.addVisionMeasurement(
                    robotPose.toPose2d(), 
                    timestamp, 
                    QUESTNAV_STD_DEVS
                );
            }
        }
    }*/

    private void updateLimelight(){
        /* * 1. Send Orientation to Limelight 
         * Required for MegaTag2 to calculate the pose correctly.
         * We retrieve the robot's current heading from the drivetrain (Gyro).
         */
        var driveState = drivetrain.getState();
        double headingDeg = driveState.Pose.getRotation().getDegrees();
        
        LimelightHelpers.SetRobotOrientation(kLimelightName, headingDeg, 0, 0, 0, 0, 0);

        /* * 2. Read Pose Estimate 
         * getBotPoseEstimate_wpiBlue_MegaTag2 uses the orientation sent above 
         * to compute the robot's location on the field (Blue Alliance origin).
         */
        PoseEstimate llMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(kLimelightName);

        if (llMeasurement != null && llMeasurement.tagCount > 0) {
            /*
             * 3. Filter and Apply
             * Reject measurements if the robot is spinning too fast to avoid camera blur/latency issues.
             */
            double omegaRps = Units.radiansToRotations(driveState.Speeds.omegaRadiansPerSecond);
            if (Math.abs(omegaRps) < 2.0) {
                Matrix<N3, N1> stdDevs = getLimelightStdDevs(llMeasurement);
                drivetrain.addVisionMeasurement(llMeasurement.pose, llMeasurement.timestampSeconds, stdDevs);
            }
        }
    }

    private void updatePhotonVision(PhotonPoseEstimator estimator, PhotonCamera camera){
        if (estimator == null || camera == null) return;

        //1. Get latest result from the camera
        PhotonPipelineResult result = camera.getLatestResult();
        //2. Stop if no targets
        if (!result.hasTargets()) return;
        //3. Update the estimator with camera result
        Optional<EstimatedRobotPose> estimatedPose = estimator.update(result);
        //4. Filter and apply 
        if (estimatedPose.isPresent()){
            EstimatedRobotPose camPose = estimatedPose.get();

            Matrix<N3, N1> stdDevs = getPhotonStdDevs(camPose);

            drivetrain.addVisionMeasurement(
                camPose.estimatedPose.toPose2d(),
                camPose.timestampSeconds,
                stdDevs
            );
        }

    }

    /**
     * Calculates standard deviations based on the number of tags seen and their distance.
     * Lower numbers = more trust in the vision measurement.
     */
    private Matrix<N3, N1> getLimelightStdDevs(PoseEstimate estimate) {
        double xyStds;
        double degStds;

        if (estimate.tagCount >= 2) {
            // High trust if multiple tags are visible
            xyStds = 0.05;
            degStds = 3;
        } else {
            // Scale trust based on distance to the single tag
            double dist = estimate.avgTagDist;
            if (dist > 4.0) {
                 xyStds = 1.0; 
                 degStds = 10;
            } else {
                 xyStds = 0.5 + (0.1 * dist);
                 degStds =  5 + (2* dist);
            }
        }
        return VecBuilder.fill(xyStds, xyStds, Units.degreesToRadians(degStds));
    }

    private Matrix<N3, N1> getPhotonStdDevs(EstimatedRobotPose estimate) {
        double xyStds;
        
        // Calculate average distance
        double avgDist = 0;
        int count = 0;
        for (var target : estimate.targetsUsed) {
            avgDist += target.getBestCameraToTarget().getTranslation().getNorm();
            count++;
        }
        if (count > 0) avgDist /= count;

        // Trust scales with distance, even for multi-tag
        if (count >= 2){
            // Start at 0.1m trust, add more uncertainty as distance increases
            xyStds = 0.1 + (0.05 * avgDist * avgDist); 
        } else {
            // Single tag: Extract ambiguity
            double ambiguity = estimate.targetsUsed.get(0).getPoseAmbiguity();

            // If ambiguity is high (> 0.2) or invalid (-1), the tag could be flipped.
            if (ambiguity > 0.2 || ambiguity == -1.0) {
                // Set a massive standard deviation to mathematically ignore this translation
                xyStds = 100.0; 
            } else {
                // Single tag is less trustworthy. 
                // Scale base trust by distance AND penalize it based on how ambiguous it is.
                xyStds = 0.5 + (0.15 * avgDist * avgDist) + (ambiguity * 10.0);
            }
        }

        // Use a massive number for rotation to trust the Pigeon (Gyro) exclusively
        double degStds = 9999999.0; 

        return VecBuilder.fill(xyStds, xyStds, Units.degreesToRadians(degStds));
    }

    /*public void resetQuestPose(Pose3d knownRobotPose) {
        Pose3d questPose = knownRobotPose.transformBy(ROBOT_TO_QUEST);
        questNav.setPose(questPose);
    } */
}