package frc.robot.subsystems;

import edu.wpi.first.cameraserver.CameraServer;

import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
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

        cvSink = CameraServer.getVideo();
        outputStream = CameraServer.putVideo("Blur", 640, 480);

        cameraSelection = NetworkTableInstance.getDefault().getTable("").getEntry("CameraSelection");
        detector = new AprilTagDetector();
        detector.addFamily("tag16h5");

        source = new Mat();
        output = new Mat();

        cameraConfig = new Config(0.1524, 1430, 1430, 480, 620);

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
    Transform3d myPosition;


    @Override
    public void periodic() {

        cvSink.grabFrame(source);
        Imgproc.cvtColor(source, output, Imgproc.COLOR_BGR2GRAY);
        outputStream.putFrame(output);

        detections = detector.detect(output);
        myPosition = poseDetermine();

        //Send the position to the dashboard
        if(myPosition != null){
            SmartDashboard.putNumber("x", myPosition.getX());
            SmartDashboard.putNumber("y", myPosition.getY());
            SmartDashboard.putNumber("z", myPosition.getZ());
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