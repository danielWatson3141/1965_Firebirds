package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Logging;

public class PnuLifter extends SubsystemBase {

    private XboxController myController;

    public static final double UP_RATE_LIMIT = .2;
    public static final double DOWN_RATE_LIMIT = .2;

    public PnuLifter(XboxController cont) {
        myController = cont;
    }

    @Override
    public void periodic() {

        // get how much time has passed since last iteration
        double elapsedTime = time_elapsed();

    }

    long previousTime = 0;

    private double time_elapsed() {
        // Get the current time
        long currentTime = System.currentTimeMillis();
        // Calculate how much time has passed
        if (previousTime == 0) {
            previousTime = currentTime;
        }

        long elapsedTime = currentTime - previousTime;
        // Set the previous time up for later
        previousTime = currentTime;

        return elapsedTime;
    }
}