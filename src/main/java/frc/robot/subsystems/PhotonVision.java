package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PhotonVision extends SubsystemBase {

    final double CAMERA_HEIGHT_METERS = 0;
    final double TARGET_HEIGHT_METERS = 0;

    final double CAMERA_PITCH_RADIANS = 0;

    final double GOAL_RANGE_METERS = 1.3;

    PhotonCamera shooterCamera = new PhotonCamera("2");

    PhotonPipelineResult shooterCamResult = new PhotonPipelineResult();

    PhotonTrackedTarget bestTarget;
    boolean hasTargets;
    int targetID;

    public PhotonVision() {
        

    }

    public void checkVisionResults(){
        shooterCamResult = shooterCamera.getLatestResult();

        hasTargets = shooterCamResult.hasTargets();

        if (hasTargets){
            bestTarget = shooterCamResult.getBestTarget();
            targetID = bestTarget.getFiducialId();
        }

    }




    public void teleopPeriodic(){


    }




}