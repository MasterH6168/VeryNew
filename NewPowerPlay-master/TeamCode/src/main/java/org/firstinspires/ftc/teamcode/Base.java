//
//package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.hardware.bosch.BNO055IMU;
//import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
//import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
//
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.util.Range;
//
////import org.firstinspires.ftc.robotcontroller.external.samples.MB1242.AsyncMB1242;
//import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
//import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
//
//@Disabled
//public class Base {
//
//
//    // Total Motors: 5
//    // Total Servos: 2
//    public DcMotor frontright      = null;
//    public DcMotor frontleft     = null;
//    public DcMotor backleft    = null;
//    public DcMotor backright   = null;
//    public DcMotor lift        = null;
//    public Servo   rightgrabber      = null;
//    public Servo   leftgrabber    = null;
//
//
//    // Total Sensors: 4
//    public ModernRoboticsI2cRangeSensor rangebrothers   = null;
//    public ModernRoboticsI2cGyro        gyro        = null;
//
//    static final Integer cpr = 13; //counts per rotation
//    static final Integer gearratio = 40;
//    static final Double diameter = 4.125;
//    static final Double cpi = (cpr * gearratio) / (Math.PI * diameter); //counts per inch, 28cpr * gear ratio / (2 * pi * diameter (in inches, in the center))
//    static final Double bias = 0.8;//default 0.8
//    static final Double meccyBias = 0.9;//change to adjust only strafing movement
//    public double amountError = 2;
//
//    static final double HEADING_THRESHOLD = 1;      // As tight as we can make it with an integer gyro
//    static final double P_TURN_COEFF = 0.1;     // Larger is more responsive, but also less stable
//    static final double P_DRIVE_COEFF = 0.07;     // Larger is more responsive, but also less stabl
//
//    static final Double conversion = cpi * bias;
//    static final Boolean exit = false;
//
//    BNO055IMU imu;
//    Orientation angles;
//    Acceleration gravity;
//
//    HardwareMap hwMap = null;
//
//    public void init(HardwareMap ahwMap, OpMode opMode) {
//        hwMap = ahwMap;
//
//        frontleft = hwMap.dcMotor.get("frontleft");
//        frontright   = hwMap.get(DcMotor.class, "frontright");
//        backleft  = hwMap.get(DcMotor.class, "backleft");
//        backright = hwMap.get(DcMotor.class, "backright");
//        lift      = hwMap.get(DcMotor.class, "lift");
//        leftgrabber    = hwMap.get(Servo.class,"leftgrabber");
//        rightgrabber  = hwMap.get(Servo.class, "rightgrabber");
//
//        rangebrothers = hwMap.get(ModernRoboticsI2cRangeSensor.class,"backRange");
//        rangebrothers.initialize();
//
//
//        gyro = hwMap.get(ModernRoboticsI2cGyro.class,"Gyro");
//        gyro.initialize();
//        gyro.calibrate();
//        while(gyro.isCalibrating());
//        opMode.telemetry.addLine("Gyro Calibrated");
//        opMode.telemetry.update();
//
//        backleft.setDirection(DcMotor.Direction.FORWARD);
//        frontleft.setDirection(DcMotor.Direction.FORWARD);
//        backright.setDirection(DcMotor.Direction.REVERSE);
//        frontright.setDirection(DcMotor.Direction.REVERSE);
//        lift.setDirection(DcMotor.Direction.FORWARD);
////        leftgrabber.setDirection(DcMotor.Direction.FORWARD);
////        rightgrabber.setDirection(DcMotor.Direction.FORWARD);
//
//        rightDT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        leftDT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        rightDT.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        leftDT.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//        leftDT.setPower(0);
//        rightDT.setPower(0);
//        leftDuck.setPower(0);
//        rightDuck.setPower(0);
//        lift.setPower(0);
//        bucket.setPosition(1.0);
//        leftClaw.setPosition(0.4);
//
//        opMode.telemetry.addLine("Initialization Complete");
//        opMode.telemetry.update();
//    }
//
//
//}
