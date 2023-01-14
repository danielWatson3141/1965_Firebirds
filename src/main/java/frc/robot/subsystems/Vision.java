package frc.robot.subsystems;

import edu.wpi.first.cameraserver.CameraServer;

import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Logging;
import edu.wpi.first.apriltag.jni.AprilTagJNI;

import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;

import edu.wpi.first.apriltag.AprilTagDetection;
import edu.wpi.first.apriltag.AprilTagDetector;

public class Vision extends SubsystemBase {
    UsbCamera camera1;
    UsbCamera camera2;
    NetworkTableEntry cameraSelection;

    CvSink cvSink;
    CvSource outputStream;

    AprilTagDetector detector;

    Mat source;
    Mat output;

    long m_native;

    public Vision() {

        camera1 = CameraServer.startAutomaticCapture(0);
        camera1.setResolution(300, 300);
        camera2 = CameraServer.startAutomaticCapture(1);
        camera2.setResolution(300, 300);

        cvSink = CameraServer.getVideo();
        outputStream = CameraServer.putVideo("Blur", 640, 480);

        cameraSelection = NetworkTableInstance.getDefault().getTable("").getEntry("CameraSelection");
        detector = new AprilTagDetector();

        Mat source = new Mat();
        Mat output = new Mat();

        long m_native = AprilTagJNI.createDetector();
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

    @Override
    public void periodic() {

        cvSink.grabFrame(source);
        Imgproc.cvtColor(source, output, Imgproc.COLOR_BGR2GRAY);
        outputStream.putFrame(output);
        detections = detector.detect(output);

    }

    public Pose3d poseDetermine() {
        if (AprilTagDetection[].class == null)  {
           return null;
          } else {
           return null;
          }
          
    }
}