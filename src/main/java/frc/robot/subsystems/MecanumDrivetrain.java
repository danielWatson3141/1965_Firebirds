package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;

import java.util.Map;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Logging;

public class MecanumDrivetrain extends SubsystemBase {

  public final ADXRS450_Gyro m_gyro = new ADXRS450_Gyro();

  // public double gyroAngle = m_gyro.getAngle();

  private SlewRateLimiter throttleLimiterX;
  private SlewRateLimiter throttleLimiterY;
  private SlewRateLimiter rotationLimiter;

  private final double TRANSLATION_DEADZONE = 0.08;
  private final double ROTATION_DEADZONE = 0.11;

  private final double ROTATION_TOLERANCE = 3; // degrees
  // private double locationX = 0.2794;
  // private double locationY = 0.3048;

  Joystick m_stick;

  Rotation2d gyroAngle;
  Rotation2d POVvalue;

  CANSparkMax m_frontLeft = new CANSparkMax(1, MotorType.kBrushless);
  CANSparkMax m_rearLeft = new CANSparkMax(2, MotorType.kBrushless);
  CANSparkMax m_frontRight = new CANSparkMax(3, MotorType.kBrushless);
  CANSparkMax m_rearRight = new CANSparkMax(4, MotorType.kBrushless);

  RelativeEncoder m_frontLeftEncoder = m_frontLeft.getEncoder();
  RelativeEncoder m_rearLeftEncoder = m_rearLeft.getEncoder();
  RelativeEncoder m_frontRightEncoder = m_frontRight.getEncoder();
  RelativeEncoder m_rearRightEncoder = m_rearRight.getEncoder();

  double initialFLEncoder;
  double initialFREncoder;
  double initialRLEncoder;
  double initialRREncoder;

  double driveEncoderMean;

  private double rSetpoint = 0;
  private double rError = 0;
  private double KpSlider = 3;
  private PIDController rotationPID = new PIDController(2, 0, 0);

  private double tXError = 0;
  private double tYError = 0;
  private double xTranslation = 0;
  private double yTranslation = 0;
  private final double SHOOT_DISTANCE = 0;
  private PIDController translationPID = new PIDController(0.6, 0, 0);

  private final double ROTATION_RATE = 4;
  private final double TRANSLATION_RATE = 4;

  private boolean ROTATION_LOCK = false;
  private boolean ROTATION_FEEDBACK = true;

  private final long DRIVE_AUTO_WAIT = 500;// fast speed for initial testing
  private final double DRIVE_AUTO_SPEED = 0.2;

  public boolean fieldRelative = false;

  MecanumDrive m_robotDrive = new MecanumDrive(m_frontLeft::set, m_rearLeft::set, m_frontRight::set, m_rearRight::set);

  public double setPoint;

  private final Vision m_vision = new Vision();

  // Shuffleboard setting up, oh boy
  public final ShuffleboardTab drivetrainTab = Shuffleboard.getTab(getName());
  public final GenericEntry drivePercentEntry = drivetrainTab.add("drive %", 0)
      .withSize(2, 2).withPosition(2, 3).withWidget(BuiltInWidgets.kDial).getEntry();
  public final ShuffleboardLayout fieldRobotListLayout = drivetrainTab.getLayout("Field Robot", BuiltInLayouts.kList)
      .withSize(2, 2).withPosition(0, 3).withProperties(Map.of("Label position", "HIDDEN"));
  public GenericEntry fieldBooleanEntry;
  public final ShuffleboardLayout slidersListLayout = drivetrainTab.getLayout("Sliders", BuiltInLayouts.kList)
      .withSize(3, 3).withPosition(0, 0);
  public GenericEntry throttleMaxSliderEntry;
  public GenericEntry KpSliderEntry;
  public final ShuffleboardLayout rotateSetpointListLayout = drivetrainTab
      .getLayout("Rotate Setpoint Values", BuiltInLayouts.kList)
      .withSize(1, 2).withPosition(3, 0);
  public final ShuffleboardLayout rotateSetpointGraphLayout = drivetrainTab
      .getLayout("Rotate Setpoint graph", BuiltInLayouts.kGrid)
      .withSize(2, 2).withPosition(4, 0);
  public GenericEntry rotateSetpointEntry;
  public GenericEntry rotateErrorEntry;
  public GenericEntry gyroAngleEntry;
  public final ShuffleboardLayout wheelSpeedListLayout = drivetrainTab
      .getLayout("Wheel Speeds List", BuiltInLayouts.kList)
      .withSize(2, 5).withPosition(8, 0);
  public GenericEntry FLspeed;
  public GenericEntry FRspeed;
  public GenericEntry RLspeed;
  public GenericEntry RRspeed;
  public final ShuffleboardLayout axisGraphLayout = drivetrainTab.getLayout("axis graph", BuiltInLayouts.kGrid)
      .withSize(2, 2).withPosition(6, 0);
  public GenericEntry stickXEntry;
  public GenericEntry stickYEntry;
  public GenericEntry stickZEntry;
  public final ShuffleboardLayout translationListLayout = drivetrainTab
      .getLayout("Translation values List", BuiltInLayouts.kList)
      .withSize(2, 2).withPosition(6, 3);
  public final ShuffleboardLayout translationGraphLayout = drivetrainTab
      .getLayout("Translation values graph", BuiltInLayouts.kGrid)
      .withSize(2, 2).withPosition(4, 3);
  public GenericEntry translationXEntry;
  public GenericEntry translationYEntry;

  public MecanumDrivetrain(Joystick input_stick) {

    resetEncoder();

    m_frontLeft.setIdleMode(IdleMode.kBrake);
    m_rearLeft.setIdleMode(IdleMode.kBrake);
    m_frontRight.setIdleMode(IdleMode.kBrake);
    m_rearRight.setIdleMode(IdleMode.kBrake);

    m_stick = input_stick;

    m_frontRight.setInverted(true);
    m_rearRight.setInverted(true);

    throttleLimiterX = new SlewRateLimiter(TRANSLATION_RATE);
    throttleLimiterY = new SlewRateLimiter(TRANSLATION_RATE);
    rotationLimiter = new SlewRateLimiter(ROTATION_RATE);

    SmartDashboard.putNumber("Throttle max%", 100);

    rotationPID.setTolerance(ROTATION_TOLERANCE);

    // for testing
    SmartDashboard.putNumber("Kp value", KpSlider);

    // setting up more shuffleboard stuff
    fieldBooleanEntry = fieldRobotListLayout.add("field or robot toggle", fieldRelative)
        .withWidget(BuiltInWidgets.kToggleSwitch).getEntry();
    fieldBooleanEntry = fieldRobotListLayout.add("field or robot box", fieldRelative)
        .withWidget(BuiltInWidgets.kBooleanBox)
        .getEntry();
    throttleMaxSliderEntry = slidersListLayout.add("Max Throttle %", 100).withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min", 0, "max", 100)).getEntry();
    KpSliderEntry = slidersListLayout.add("Kp value", KpSlider).withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min", 0, "max", 5)).getEntry();
    rotateSetpointEntry = rotateSetpointListLayout.add("setpoint list", rSetpoint).getEntry();
    rotateSetpointEntry = rotateSetpointGraphLayout.add("setpoint graph", rSetpoint).getEntry();
    rotateErrorEntry = rotateSetpointListLayout.add("error list", rError).getEntry();
    rotateErrorEntry = rotateSetpointGraphLayout.add("error graph", rError).getEntry();
    gyroAngleEntry = rotateSetpointListLayout.add("gyro angle list", 0).getEntry();
    gyroAngleEntry = rotateSetpointGraphLayout.add("gyro angle graph", 0).getEntry();
    FLspeed = wheelSpeedListLayout.add("Front Left Wheel", 0).getEntry();
    FRspeed = wheelSpeedListLayout.add("Front Right Wheel", 0).getEntry();
    RLspeed = wheelSpeedListLayout.add("Rear Left Wheel", 0).getEntry();
    RRspeed = wheelSpeedListLayout.add("Rear Right Wheel", 0).getEntry();
    stickXEntry = axisGraphLayout.add("X axis", 0).getEntry();
    stickYEntry = axisGraphLayout.add("Y axis", 0).getEntry();
    stickZEntry = axisGraphLayout.add("Z axis", 0).getEntry();
    translationXEntry = translationListLayout.add("X translation list", xTranslation).getEntry();
    translationXEntry = translationGraphLayout.add("X translation graph", xTranslation).getEntry();
    translationYEntry = translationListLayout.add("Y translation list", tYError).getEntry();
    translationYEntry = translationGraphLayout.add("Y translation graph", tYError).getEntry();
  }

  // multipliers for values
  final double SPEED_CAP = .6;
  public double driveSpeed;

  public void setSpeed() {
    // get percentage from the 4th axis and converts it from 0% - 100%
    double throttle_value = ((-m_stick.getRawAxis(3) + 1) / 2) * (SmartDashboard.getNumber("Throttle max%", 100) / 100);
    // sets the sped based on the cap and percentage
    driveSpeed = SPEED_CAP * throttle_value;
    // documents the current percentage of the motors for driver
    drivePercentEntry.setDouble(throttle_value * 100);
  }

  public double deadzone(double input, double deadzone) {
    double absInput = Math.abs(input);
    if (absInput <= deadzone) {
      return 0;
    } else {
      double value = (absInput - deadzone) / (1 - deadzone);
      if (input < 0) {
        value = -value;
      }
      return value;
    }
  }

  private final double ENCODER_CONVERSION_FACTOR = 25;

  public double getDistanceTravelled() {
    double frontLeftEncoderValue = (m_frontLeftEncoder.getPosition() - initialFLEncoder) / ENCODER_CONVERSION_FACTOR;
    double frontRightEncoderValue = (m_frontRightEncoder.getPosition() - initialFREncoder) / ENCODER_CONVERSION_FACTOR;
    double rearLeftEncoderValue = (m_rearLeftEncoder.getPosition() - initialRLEncoder) / ENCODER_CONVERSION_FACTOR;
    double rearRightEncoderValue = (m_rearRightEncoder.getPosition() - initialRREncoder) / ENCODER_CONVERSION_FACTOR;

    driveEncoderMean = (frontLeftEncoderValue + frontRightEncoderValue + rearLeftEncoderValue + rearRightEncoderValue) / 4;

    return driveEncoderMean;
  }

  public void resetEncoder() {
    initialFLEncoder = m_frontLeftEncoder.getPosition();
    initialFREncoder = m_frontRightEncoder.getPosition();
    initialRLEncoder = m_rearLeftEncoder.getPosition();
    initialRREncoder = m_rearRightEncoder.getPosition();
  }

  public Rotation2d gyroAngle() {
    gyroAngle = m_gyro.getRotation2d().times(-1);
    return gyroAngle;
  }

  public void gyroReset() {
    m_gyro.reset();
    rSetpoint = 0;
    Logging.log(getSubsystem(), "reset gyroscope");
  }

  public void switchDriveRelativity() {
    fieldRelative = !fieldRelative;
  }

  public void lifterModeToggle(){
    ROTATION_FEEDBACK = !ROTATION_FEEDBACK;
  }

  public void driveAuto(double autoSpeed) {
    m_frontLeft.set(autoSpeed);
    m_rearLeft.set(autoSpeed);
    m_frontRight.set(autoSpeed);
    m_rearRight.set(autoSpeed);
  }

  public Command driveAutoCommand() {
    Command r_command = Commands.sequence(
        new InstantCommand(() -> driveAuto(DRIVE_AUTO_SPEED)),
        Commands.waitSeconds(DRIVE_AUTO_WAIT),
        new InstantCommand(() -> driveAuto(0)));

    r_command.addRequirements(this);
    return r_command;
  }

  public void drive() {

    gyroAngle();

    double drive_x = 0;
    double drive_y = 0;
    double drive_z = 0;

    // **** POV drive mode
    // If the POV stick is active, put raw POV input into the drivetrain.
    // TODO: This will not have directional stability. Remove / Update?
    if (m_stick.getPOV() != -1) {
      POVvalue = Rotation2d.fromDegrees(m_stick.getPOV());
      m_robotDrive.drivePolar(driveSpeed, POVvalue, 0);

      // **** April Tag Tracking mode
      // If the vision system has a detection and button 3 is held, track the april
      // tag and position the robot directly in front of it.
    } else if (m_vision.myPosition != null && m_stick.getRawButton(3)) {

      if (m_vision.VISION_WORKING) {

        tYError = m_vision.myPosition.getTranslation().getY();
        xTranslation = m_vision.myPosition.getTranslation().getX();
        yTranslation = translationPID.calculate(tYError, SHOOT_DISTANCE);

        if (Math.abs(xTranslation) <= 1) {
          tXError = xTranslation;
        } else {
          tXError = (xTranslation < 0) ? -1 : 1;
        }

        rError = Units.radiansToDegrees((m_vision.myPosition.getRotation().getZ()));
        drive_x = translationPID.calculate(tXError, 0);

        if (Math.abs(yTranslation) <= 1) {
          drive_y = yTranslation;
        } else {
          drive_y = (yTranslation < 0) ? -1 : 1;
        }
      }

      // **** Joystick Control Mode
      // If neither POV mode nor tracking mode is active, then take stick input.
    } else {
      // Rotation is locked by default, unlock when button is held
      if (!ROTATION_LOCK || m_stick.getRawButton(2)) {
        drive_z = rotationLimiter.calculate((deadzone(m_stick.getZ(), ROTATION_DEADZONE)));
      } else {
        drive_z = 0;
      }

      drive_z = rotationLimiter.calculate((deadzone(m_stick.getZ(), ROTATION_DEADZONE)));

      // Update the setpoint by the drive_z input, this moves the desired heading.
      rSetpoint = (rSetpoint + drive_z);

      // Calculate the current error, we will pass this to the PID loop for corrective
      // input
      rError = (m_gyro.getAngle() - rSetpoint);

      // Pull the translation input from the stick, apply deadzone and slew rateP
      drive_x = throttleLimiterX.calculate(deadzone(m_stick.getX(), TRANSLATION_DEADZONE)) * driveSpeed;
      drive_y = throttleLimiterY.calculate(deadzone(-m_stick.getY(), TRANSLATION_DEADZONE)) * driveSpeed;

    }

    // Mecanum seems to consider 'X' the forward direction, so we're passing y, x, z
    // to driveCartesian on purpose.
    m_robotDrive.driveCartesian(
        drive_y,
        drive_x,
        ROTATION_FEEDBACK ? rotationPID.calculate(rError, 0) / 180 : drive_z,
        fieldRelative ? gyroAngle : Rotation2d.fromDegrees(0));

    stickXEntry.setDouble(drive_x);
    stickYEntry.setDouble(drive_y);
    stickZEntry.setDouble(drive_z);
  }

  @Override
  public void periodic() {

    SmartDashboard.putNumber("rotation setpoint", rSetpoint);
    SmartDashboard.putNumber("rotation error", rError);

    SmartDashboard.putNumber("encoder value", getDistanceTravelled());

    rotateSetpointEntry.setDouble(rSetpoint);
    rotateErrorEntry.setDouble(rError);

    setSpeed();

    // SmartDashboard.putNumber("gyroAngle", m_gyro.getRotation2d().getDegrees() *
    // -1);
    gyroAngleEntry.setDouble(m_gyro.getRotation2d().getDegrees() * -1);

    SmartDashboard.putNumber("FL_SPEED", m_frontLeftEncoder.getVelocity());
    SmartDashboard.putNumber("RL_SPEED", m_rearLeftEncoder.getVelocity());
    SmartDashboard.putNumber("FR_SPEED", m_frontRightEncoder.getVelocity());
    SmartDashboard.putNumber("RR_SPEED", m_rearRightEncoder.getVelocity());
    FLspeed.setDouble(m_frontLeftEncoder.getVelocity());
    FRspeed.setDouble(m_frontRightEncoder.getVelocity());
    RLspeed.setDouble(m_rearLeftEncoder.getVelocity());
    RRspeed.setDouble(m_rearRightEncoder.getVelocity());

    // for testng
    KpSlider = KpSliderEntry.getDouble(KpSlider);
    rotationPID.setP(KpSlider);
  }
}