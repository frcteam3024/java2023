
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.DriveConstants.*;

public class SwerveSubsystem extends SubsystemBase {

  private final SwerveModule frontLeft = new SwerveModule(
    FL_DRIVE_MOTOR_PORT,
    FL_TURN_MOTOR_PORT,
    FL_DRIVE_ENCODER_REVERSED,
    FL_TURN_ENCODER_REVERSED,
    FL_ABSOLUTE_ENCODER_PORT,
    FL_ABSOLUTE_ENCODER_OFFSET_RAD,
    FL_ABSOLUTE_ENCODER_REVERSED);

  private final SwerveModule frontRight = new SwerveModule(
    FR_DRIVE_MOTOR_PORT,
    FR_TURN_MOTOR_PORT,
    FR_DRIVE_ENCODER_REVERSED,
    FR_TURN_ENCODER_REVERSED,
    FR_ABSOLUTE_ENCODER_PORT,
    FR_ABSOLUTE_ENCODER_OFFSET_RAD,
    FR_ABSOLUTE_ENCODER_REVERSED);

  private final SwerveModule backLeft = new SwerveModule(
    BL_DRIVE_MOTOR_PORT,
    BL_TURN_MOTOR_PORT,
    BL_DRIVE_ENCODER_REVERSED,
    BL_TURN_ENCODER_REVERSED,
    BL_ABSOLUTE_ENCODER_PORT,
    BL_ABSOLUTE_ENCODER_OFFSET_RAD,
    BL_ABSOLUTE_ENCODER_REVERSED);

  private final SwerveModule backRight = new SwerveModule(
    BR_DRIVE_MOTOR_PORT,
    BR_TURN_MOTOR_PORT,
    BR_DRIVE_ENCODER_REVERSED,
    BR_TURN_ENCODER_REVERSED,
    BR_ABSOLUTE_ENCODER_PORT,
    BR_ABSOLUTE_ENCODER_OFFSET_RAD,
    BR_ABSOLUTE_ENCODER_REVERSED);

  private final SwerveModule[] swerveModules = {
    frontLeft,
    frontRight,
    backLeft,
    backRight
  };

  private final AHRS gyro = new AHRS(SPI.Port.kMXP);

  public SwerveSubsystem() {
    // allow 1 sec for gyro to boot up. on a separate thread so other
    // functions can continue to run
    new Thread(() -> {
      try {
        Thread.sleep(1000);
        zeroHeading();
      } catch (InterruptedException e) {
        e.printStackTrace();
        Thread.currentThread().interrupt();
      }
    }).start();
  }

  public void zeroHeading() {
    gyro.reset();
  }

  public double getGyroDegrees() {
    // range (-180, 180]
    return Math.IEEEremainder(gyro.getAngle(), 360);
  }

  public Rotation2d getGyroRotation2d() {
    return Rotation2d.fromDegrees(getGyroDegrees());
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Gyro Degrees", getGyroDegrees());
  }

  public void stopModules() {
    for (SwerveModule module : swerveModules)
      module.stop();
  }

  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, MAX_PHYSICAL_SPEED_METERS_PER_SEC);
    for (int i=0; i<4; i++) {
      swerveModules[i].setState(desiredStates[i]);
    }
  }

  public void brakeMode() {
    for (SwerveModule module : swerveModules)
      module.brakeMode();
  }

  public void coastMode() {
    for (SwerveModule module : swerveModules)
      module.coastMode();
  }

  /**

  ///////////////////////////////
  // SPEED && ANGLE FUNCTIONS: //
  ///////////////////////////////
  
  public double[] readDriverInputs() {
    double rawDriverXAxis = Robot.robotContainer.getDriverRawAxis(Constants.DRIVE_X_AXIS);
    double rawDriverYAxis = Robot.robotContainer.getDriverRawAxis(Constants.DRIVE_Y_AXIS);
    //double driverTwist = Robot.robotContainer.getDriverRawAxis(Constants.DRIVE_ROTATE);
    double driverRawSlider = Robot.robotContainer.getDriverRawAxis(Constants.DRIVE_SLIDER);
    double driverSensitivity = adjustSliderRange(driverRawSlider);
    double[] fixedDriverJoyInputs = fixDriverJoystickInputs(rawDriverXAxis, rawDriverYAxis);
    double driverXAxis = fixedDriverJoyInputs[0];
    double driverYAxis = fixedDriverJoyInputs[1];

    double[] driverInputs = {
      driverXAxis,
      driverYAxis,
      //driverTwist,
      driverSensitivity
    };

    for (int i=0; i<driverInputs.length; i++)
      if (Math.abs(driverInputs[i]) < Constants.DRIVER_AXIS_THRESHOLDS[i])
        driverInputs[i] = 0;

    return driverInputs;
  }

  public double[][] calculateTargetVectors(double[] driverInputs) {

    double driverXAxis = driverInputs[0];
    double driverYAxis = driverInputs[1];
    //double driverTwist = driverInputs[2];
    double driverSensitivity = driverInputs[2];

    //double rotationMagnitude = driverTwist;
    double translationDirection = 180.0/Math.PI * Math.atan2(driverXAxis, driverYAxis);
    double translationMagnitude = Math.sqrt(Math.pow(driverXAxis,2) + Math.pow(driverYAxis,2));
    double[] translationVector = {translationMagnitude, translationDirection};

    // 4x2 arrays: rows = vectors
    // col 0 = magnitudes, col 1 = directions
    //double[][] rotationVectors = new double[4][2];
    double[][] resultantVectors = new double[4][2];
    double[] resultantSpeeds = new double[4];
    double[] resultantAngles = new double[4];
    double[] targetAngles = new double[4];

    for (int i=0; i<4; i++) {
      //double rotationDirection = Constants.ROTATION_DIRECTIONS[i];
      //double[] rotationVector = {rotationMagnitude, rotationDirection};
      //rotationVectors[i] = rotationVector;
      //resultantVectors[i] = resultantVector(translationVector, rotationVectors[i]); 
      resultantVectors[i] = translationVector;

      double resultantMagnitude = driverSensitivity * resultantVectors[i][0];
      double resultantDirection = resultantVectors[i][1];
      resultantSpeeds[i] = resultantMagnitude;
      resultantAngles[i] = resultantDirection;

      targetAngles[i] = resultantAngles[i];
      //swerveModules[i].targetAngle = resultantDirection;
    }
    double[] targetSpeeds = normalizeSpeeds(resultantSpeeds);
    
    Robot.displaySystem.printValueToDash("translation mag ", translationMagnitude);
    Robot.displaySystem.printValueToDash("translation dir ", translationDirection);
    //Robot.displaySystem.printVectorsToDash("rotation Vector", rotationVectors);
    
    double[][] targetVectors = {targetSpeeds, targetAngles};
    return targetVectors;
  }

  public void setMotorMode(String motorMode) {
    for (int i=0; i<4; i++) {
      if (motorMode.equals("brake"))
        Robot.driveTrain.swerveModules[i].brakeMode();
      else if (motorMode.equals("coast"))
        Robot.driveTrain.swerveModules[i].coastMode();
      else throw new InvalidParameterException("invalid motor mode \"" + motorMode +
        " \"motors can only be set to brake or coast mode");
    }
  }

  ///////////////////////////////
  // ANGLE-SPECIFIC FUNCTIONS: //
  ///////////////////////////////

  public void setMotorAngles(double[] targetAngles) {
    double[] currentMotorAngles = getCurrentMotorAngles();
    double[] angleMotorOutputs = calculateAngleMotorOutputs(currentMotorAngles, targetAngles);
    setAngleMotorOutputs(angleMotorOutputs);
    Robot.displaySystem.printArrayToDash(" taget angle", targetAngles);
    Robot.displaySystem.printArrayToDash(" current angle", currentMotorAngles);
    Robot.displaySystem.printArrayToDash(" angle output", angleMotorOutputs);
  }

  public double[] getCurrentMotorAngles() {
    // outputs in degrees
    double[] currentMotorAngles = new double[4];
    for (int i=0; i<4; i++)
      currentMotorAngles[i] = swerveModules[i].getAngleEncoder();
    return currentMotorAngles;
  }

  private double[] calculateAngleMotorOutputs(double[] currentMotorAngles, double[] targetAngles) {
    double[] angleMotorOutputs = new double[4];
    for (int i=0; i<4; i++) {
      double currentDegError =  targetAngles[i] - currentMotorAngles[i];
      double standardizedDegError = standardizeAngle(currentDegError);
      double finalError = standardizedDegError / 180.0;
      angleMotorOutputs[i] = pid.calculate(finalError);
      // angleMotorOutputs[i] = Constants.kP * finalError;
      Robot.displaySystem.printValueToDash(Robot.driveTrain.swerveModules[i].getLocation() + " error", standardizedDegError);
      Robot.displaySystem.printValueToDash(Robot.driveTrain.swerveModules[i].getLocation() + " before kP", finalError);
      Robot.displaySystem.printValueToDash(Robot.driveTrain.swerveModules[i].getLocation() + " after kP", angleMotorOutputs[i]);
    }
    return angleMotorOutputs;
  }

  private void setAngleMotorOutputs(double[] angleMotorOutputs) {
    for (int i=0; i<4; i++) {
      if (Math.abs(angleMotorOutputs[i]) < 0.00)
        angleMotorOutputs[i] = 0.0;
      swerveModules[i].setAngleMotorOutput(angleMotorOutputs[i]);
    }
  }

  /**
   * private double[] shortestPathAngles(double[] targetAngles) {
   *   double[] shortestPathAngles = new double[4];
   *   //boolean[] flippedWheels = new boolean[4];
   *   double flipThreshold = Constants.ANGLE_FLIP_THRESHOLD;
   *   for (int i=0; i<4; i++) {
   *     double normalizedAngle;
   *     double angleMod = targetAngles[i] % 360;
   *     if (angleMod <= flipThreshold) {
   *       normalizedAngle = angleMod;
   *       //swerveModules[i].flipped = false;
   *     }else if (flipThreshold < angleMod && angleMod < 360 - flipThreshold) {
   *       normalizedAngle = 180 - angleMod;
   *       //swerveModules[i].flipped = true;
   *     }else{
   *       normalizedAngle = angleMod - 360;
   *       //swerveModules[i].flipped = false;
   *     }
   *     shortestPathAngles[i] = normalizedAngle;
   *   }
   *   return shortestPathAngles;
   * }
   *

   public double standardizeAngle(double angle) {
    return ((angle + 180.0) % 360.0) - 180.0;
   }

  public void resetSwerveOffsets() {
    for ( SwerveModule module : swerveModules )
      module.resetOffsets();
  }
  
  ///////////////////////////////
  // SPEED-SPECIFIC FUNCTIONS: //
  ///////////////////////////////

  public void setMotorSpeeds(double[] targetSpeeds) {
    double[] speedMotorOutputs = targetSpeeds;
    setSpeedMotorOutputs(speedMotorOutputs);
  }

  private double[] normalizeSpeeds(double[] inputSpeeds) {
    // scale speed magnitudes to within [-1,1] if any are outside that range
    double maxSpeed = arrayMax(inputSpeeds);
    double speedScalar;

    if (maxSpeed > 1)
      speedScalar = maxSpeed;
    else
      speedScalar = 1;

    double[] normalizedSpeeds = new double[4];
    for (int i=0; i<4; i++) {
      normalizedSpeeds[i] = inputSpeeds[i] / speedScalar;
      /** 
      if (normalizedSpeeds[i] < 0.01) {
        normalizedSpeeds[i] = 0;
      }
      if (flippedWheels[i]) {
        normalizedSpeeds[i] *= -1;
      }
      *
    }
    return normalizedSpeeds;
  }

  public void setSpeedMotorOutputs(double[] speedMotorOutputs) {
    for (int i=0; i<4; i++)
      swerveModules[i].setSpeedMotorOutput(speedMotorOutputs[i]);
  }

  //////////////////////
  // MISC. FUNCTIONS: //
  //////////////////////

  private static double[] resultantVector(double[] vector1, double[] vector2) {
  
    double mag1 = vector1[0];
    double dir1 = vector1[1];
    double mag2 = vector2[0];
    double dir2 = vector2[1];

    double x1 = mag1 * Math.cos(dir1); 
    double x2 = mag1 * Math.cos(dir2); 
    double y1 = mag2 * Math.sin(dir1); 
    double y2 = mag2 * Math.sin(dir2); 

    double resultantX = x1+x2;
    double resultantY = y1+y2;
    double resultantMagnitude = Math.sqrt(Math.pow(resultantX,2)+Math.pow(resultantY,2));
    double resultantDirection = Math.atan2(resultantX,resultantY);
    double[] vector = {resultantMagnitude, resultantDirection};
    return vector;
  }

  private double arrayMax(double[] array) {
    double max = 0;
    for (int i=0; i<4; i++)
      if (array[i] > max)
        max = array[i];
    return max;
  }

  private double adjustSliderRange(double driverRawSlider) {
    return 0.5*(1-driverRawSlider);
  }

  private double[] fixDriverJoystickInputs(double rawDriverXAxis, double rawDriverYAxis) {
    // https://www.desmos.com/calculator/rxa17a8nvr
    rawDriverYAxis *= -1;
    double radAngle = Math.atan2(rawDriverXAxis, rawDriverYAxis);
    double squareMagnitude = Math.min(Math.abs(1/Math.cos(radAngle)),Math.abs(1/Math.sin(radAngle)));
    double finalDriverXAxis = rawDriverXAxis/squareMagnitude;
    double finalDriverYAxis = rawDriverYAxis/squareMagnitude;

    double[] fixedDriverInputs = {finalDriverXAxis, finalDriverYAxis};
    return fixedDriverInputs;
  }
  */

}