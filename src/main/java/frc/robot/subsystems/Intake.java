package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Logging;

public class Intake extends SubsystemBase {

    private WPI_TalonSRX frontRoller;
    private WPI_TalonSRX rearRoller;
    private WPI_TalonSRX indexMotor;

    private double motorSpeed;

    private boolean sensor1;
    private boolean sensor2;

    public Intake() {

        // TODO: look up how to add motorcolntrollergroups now its been deprecated
        WPI_TalonSRX frontRoller = new WPI_TalonSRX(69);
        WPI_TalonSRX rearRoller = new WPI_TalonSRX(69);
        WPI_TalonSRX indexMotor = new WPI_TalonSRX(69);

        motorSpeed = 0.7;
        sensor1 = false;
        sensor2 = false;

    }

    public void runRoller() {
        frontRoller.set(motorSpeed);
        rearRoller.set(motorSpeed);
    }

    public void timedRoller() {
        frontRoller.set(0);
        rearRoller.set(0);

    }

    public void stopRoller() {
        frontRoller.set(0);
        rearRoller.set(0);

    }

    public void runIndex() {
        if (sensor1) {
            indexMotor.set(motorSpeed);
        }
    }

    public void noteLoaded() {
        SmartDashboard.putBoolean("noteLoaded", sensor2==true);
    }

    public void stopIndex() {
        indexMotor.set(0);
    }

    public void periodic() {

    }

}
