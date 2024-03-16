package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Logging;

public class PathPlanner extends SubsystemBase{

    public PathPlanner(MecanumDrivetrain m_drivetrain) {

        // AutoBuilder.configureHolonomic(
        //     m_drivetrain.m_odometry.getPoseMeters(),
        //     m_drivetrain.m_odometry.resetPosition(m_drivetrain.m_gyro.getRotation2d(), m_drivetrain.getCurrentDistances(), m_drivetrain.m_odometry.getPoseMeters()),
        //     m_drivetrain.getCurrentChassisSpeed())
        // );

    }

}

