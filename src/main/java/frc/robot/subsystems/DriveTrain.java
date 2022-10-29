
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.security.InvalidParameterException;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.commands.SwerveDrive;


public class DriveTrain extends SubsystemBase {

  SwerveModule swerveFL = new SwerveModule("FL", Constants.FL_SWERVE_MODULE_ID);
  SwerveModule swerveFR = new SwerveModule("FR", Constants.FR_SWERVE_MODULE_ID);
  SwerveModule swerveBL = new SwerveModule("BL", Constants.BL_SWERVE_MODULE_ID);
  SwerveModule swerveBR = new SwerveModule("BR", Constants.BR_SWERVE_MODULE_ID);

  private SwerveModule[] swerveModules = {
    swerveFL,
    swerveFR,
    swerveBL,
    swerveBR
  };

  private PIDController pid = new PIDController(Constants.kP, Constants.kI, Constants.kD);
  // docs.wpilib.org/en/stable/docs/software/advanced-controls/controllers/pidcontroller.html

  public DriveTrain() {
    /** Creates a new DriveTrain. */
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  //@Override
  public void initDefaultCommand() {
    setDefaultCommand(new SwerveDrive());
  }

  // TODO: implement shortest path angles and flipping wheels once current code runs successfully

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
   */

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
      */
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

}