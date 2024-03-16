package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Logging;

public class IntakeShooter extends SubsystemBase {

    private final boolean HAVE_INDEX_MOTOR = true;
    
    WPI_TalonSRX rollerMotor = new WPI_TalonSRX(9);

    WPI_TalonSRX indexIntake = new WPI_TalonSRX(10);
    WPI_TalonSRX indexShooter = new WPI_TalonSRX(11);

    CANSparkMax shooterMotor1 = new CANSparkMax(12, MotorType.kBrushless);
    CANSparkMax shooterMotor2 = new CANSparkMax(13, MotorType.kBrushless);

    private final SimpleMotorFeedforward m_shooterFeedforward = new SimpleMotorFeedforward(3, 1);

    RelativeEncoder shooterEncoder1 = shooterMotor1.getEncoder();
    RelativeEncoder shooterEncoder2 = shooterMotor2.getEncoder();

    public double INTAKE_SPEED = 0.6;
    private final double INTAKE_TIMEOUT = 5;

    private final double SHOOTER_TIMER_SECONDS = .25;
    private final double INDEX_SHOOTER_SPEED = 1;
    private final double SPEAKER_SHOOTER_SPEED = 1;
    private final double AMP_SHOOTER_SPEED = .10;
    
    private final Joystick m_stick;

    DigitalInput limitSwitch1 = new DigitalInput(0);
    DigitalInput limitSwitch2 = new DigitalInput(1);

    boolean shooterMode;

    double shooterMotor1Amps;
    double shooterMotor2Amps;

    public IntakeShooter(Joystick input_stick) {

        shooterMotor2.follow(shooterMotor1);
        shooterMode = true;

        shooterMotor1.setIdleMode(IdleMode.kCoast);
        shooterMotor2.setIdleMode(IdleMode.kCoast);

        shooterMotor1.setSmartCurrentLimit(20);
        shooterMotor2.setSmartCurrentLimit(20);

        indexShooter.setInverted(true);

        indexIntake.setInverted(true);
       

        m_stick = input_stick;

    }

    public boolean switch1State() {
        return !limitSwitch1.get();
    }

    public boolean switch2State() {
        return !limitSwitch2.get();
    }

    public void runIntakeMotors(double speed) {
        rollerMotor.set(speed);
        indexIntake.set(speed);
    }

    public void stopIntakeSequence() {
        rollerMotor.stopMotor();
        indexIntake.stopMotor();
    }

    public void runShooterMotors(double speed) {
        shooterMotor1.set(speed);
    }

    public void stopShooterSequence() {
        shooterMotor1.stopMotor();
        indexShooter.stopMotor();

    }

    public void setIndexShooter(double speed) {
        Logging.log("IntakeShooter", "set index motor");
        indexShooter.set(speed);

    }

    public Command getShootCommand() {
        Command r_command = Commands.sequence(
                new InstantCommand(() -> runShooterMotors(m_stick.getRawButton(2) ? AMP_SHOOTER_SPEED : SPEAKER_SHOOTER_SPEED)),
                Commands.waitSeconds(SHOOTER_TIMER_SECONDS),
                new InstantCommand(() -> setIndexShooter(INDEX_SHOOTER_SPEED)),
                Commands.waitSeconds(Double.POSITIVE_INFINITY)
                );

        r_command.addRequirements(this);

        r_command = r_command.finallyDo(() -> stopShooterSequence());
        return r_command;
    }

    public Command getIntakeCommand() {
        Command r_command = new RunCommand(() -> runIntakeMotors(INTAKE_SPEED)).until(() -> switch1State()).withTimeout(INTAKE_TIMEOUT);

        r_command.addRequirements(this);

        r_command = r_command.finallyDo(() -> stopIntakeSequence());
        return r_command;
    }

    public Command testIntakeRunCommand() {
        Command r_command = (new InstantCommand(() -> runIntakeMotors(INTAKE_SPEED)));

        r_command.addRequirements(this);

        r_command = r_command.finallyDo(() -> stopIntakeSequence());
        return r_command;
    }

    public Command testIntakeStopCommand() {
        Command r_command = (new InstantCommand(() -> stopIntakeSequence()));

        r_command.addRequirements(this);
        return r_command;
    }

    public Command testShootRunCommand() {
        Command r_command = Commands.sequence(new InstantCommand(() -> runShooterMotors(m_stick.getRawButton(2) ? AMP_SHOOTER_SPEED : SPEAKER_SHOOTER_SPEED)), new InstantCommand(() -> setIndexShooter(INDEX_SHOOTER_SPEED)));

        r_command.addRequirements(this);

        r_command = r_command.finallyDo(() -> stopShooterSequence());
        return r_command;
    }

    public Command testShootStopCommand() {
        Command r_command = (new InstantCommand(() -> stopShooterSequence()));

        r_command.addRequirements(this);
        return r_command;
    }

    public void intakeShooterTestLogging(){

        shooterMotor1Amps = shooterMotor1.getOutputCurrent();
        shooterMotor2Amps = shooterMotor2.getOutputCurrent();

        SmartDashboard.putNumber("shooter 1 amps", shooterMotor1Amps);
        SmartDashboard.putNumber("shooter 2 amps", shooterMotor2Amps);
        SmartDashboard.putNumber("shooter total amps", shooterMotor1Amps + shooterMotor2Amps);
        SmartDashboard.putNumber("shooter1 velocity", shooterEncoder1.getVelocity());
        SmartDashboard.putNumber("shooter2 velocity", shooterEncoder2.getVelocity());
    }


    public void periodic() {

        
    }
}

// new ConditionalCommand(new InstantCommand(() -> setIntakeMotors(0)), )