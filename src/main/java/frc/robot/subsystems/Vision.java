package frc.robot.subsystems;

import edu.wpi.first.cameraserver.CameraServer;

import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Logging;
import edu.wpi.first.apriltag.jni.AprilTagJNI;

import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;

import edu.wpi.first.apriltag.AprilTagDetection;
import edu.wpi.first.apriltag.AprilTagDetector;
import edu.wpi.first.apriltag.AprilTagPoseEstimator;
import edu.wpi.first.apriltag.AprilTagPoseEstimator.Config;

public class Vision extends SubsystemBase {
    UsbCamera camera1;
    UsbCamera camera2;
    NetworkTableEntry cameraSelection;

    CvSink cvSink;
    CvSource outputStream;

    AprilTagDetector detector;

    AprilTagPoseEstimator estimator;

    Config cameraConfig;

    Mat source;
    Mat output;
    

    public Vision() {

        camera1 = CameraServer.startAutomaticCapture(0);
        camera1.setResolution(300, 300);
        camera2 = CameraServer.startAutomaticCapture(1);
        camera2.setResolution(300, 300);
        //creates 2 usb cameras

        cvSink = CameraServer.getVideo();
        outputStream = CameraServer.putVideo("Blur", 640, 480);
        //creates a cvsink for camera1 that allows the apriltagdetector to detect apriltags by 

        cameraSelection = NetworkTableInstance.getDefault().getTable("").getEntry("CameraSelection");
        detector = new AprilTagDetector();
        //adds a new apriltagdetector
        detector.addFamily("36h11", 1);
        //adds the tag family being used in this year's competition

        source = new Mat();
        output = new Mat();

        //cameraConfig = new Config(0.1524, 672.41409, 673.81997, 320.73015, 241.70637);
        cameraConfig = new Config(0.1524, 271.75562,272.26474, 322.90119, 227.79356);

        estimator = new AprilTagPoseEstimator(cameraConfig);
    }

    boolean frontCamera = true;

    public void switchCamera() {
        if (frontCamera) {
            Logging.log("Camera", "Switching to camera 2");
            cameraSelection.setString(camera2.getName());
            frontCamera = false;
        } else {
            Logging.log("Camera", "Switching to camera 1");
            cameraSelection.setString(camera1.getName());
            frontCamera = true;
        }
    }

    AprilTagDetection[] detections;
    //creates the apriltagdetection array
    Transform3d myPosition;
    //creates myPosition as a way to set pose


    @Override
    public void periodic() {

        try{
            cvSink.grabFrame(source);
            Imgproc.cvtColor(source, output, Imgproc.COLOR_BGR2GRAY);
            outputStream.putFrame(output);

            detections = detector.detect(output);
            myPosition = poseDetermine();
        } catch (Exception E){
           // Logging.log("Vision:Periodic",E.getMessage());
        }

        //Send the position to the dashboard if the position does not equal null
        if(myPosition != null){
            SmartDashboard.putNumber("tagX", myPosition.getX());
            SmartDashboard.putNumber("tagY", myPosition.getY());
            SmartDashboard.putNumber("tagZ", myPosition.getZ());

            double roll = myPosition.getRotation().getX();
            double pitch = myPosition.getRotation().getY();
            double yaw = myPosition.getRotation().getZ();
            //gets the x y and z coordinates from poseDetermine
    
            SmartDashboard.putNumber("tagPitch", pitch);
            SmartDashboard.putNumber("tagRoll", roll);
            SmartDashboard.putNumber("tagYaw", yaw);
            
        }
        //other stuff
    }

    public Transform3d poseDetermine() {
        if (detections == null || detections.length == 0) { // No April tag detected
            return null;
        } else { // April Tag detected
            AprilTagDetection detection = detections[0];
            return estimator.estimate(detection); 
        }


    }
}