package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name="OneConeRight")
public class RightFast extends LinearOpMode {

    DcMotor frontleft;
    DcMotor frontright;
    DcMotor backleft;
    DcMotor backright;
    DcMotor lift;
    DcMotor rightgrabber;
    DcMotor leftgrabber;

    ColorSensor color;
    ModernRoboticsI2cRangeSensor rangebrothers;
    Rev2mDistanceSensor rangebrothers2;
    ModernRoboticsI2cGyro gyro;



    //28 * 20 / (2ppi * 4.125)
    Double width = 16.0; //inches
    Integer cpr = 13; //counts per rotation
    Integer gearratio = 40;
    Double diameter = 4.125;
    Double cpi = (cpr * gearratio) / (Math.PI * diameter); //counts per inch, 28cpr * gear ratio / (2 * pi * diameter (in inches, in the center))
    Double bias = 1.0;//default 0.8
    Double meccyBias = 0.9;//change to adjust only strafing movement
    double amountError = 2;

    static final double HEADING_THRESHOLD = 1;      // As tight as we can make it with an integer gyro
    static final double P_TURN_COEFF = 0.1;     // Larger is more responsive, but also less stable
    static final double P_DRIVE_COEFF = 0.07;     // Larger is more responsive, but also less stable

    double DRIVE_SPEED08 = 0.88;
    double DRIVE_SPEED07 = 0.4;
    double DRIVE_SPEED06 = 0.3;
    double ranged = 0.15;


    public static double   MAX_ACCEPTABLE_ERROR = 10;

    double TURN_SPEED = 0.65;
    double TURN_SPEED_FIX = 0.3;
    double TIME = 0.4;


    Double conversion = cpi * bias;
    Boolean exit = false;


    Orientation angles;
    Acceleration gravity;

    private ElapsedTime runtime = new ElapsedTime();

    public void runOpMode() throws InterruptedException {



        frontleft = hardwareMap.dcMotor.get("frontleft");
        frontright = hardwareMap.dcMotor.get("frontright");
        backleft = hardwareMap.dcMotor.get("backleft");
        backright = hardwareMap.dcMotor.get("backright");
        lift = hardwareMap.get(DcMotor.class, "lift");
        leftgrabber = hardwareMap.get(DcMotor.class, "leftgrabber");
        rightgrabber = hardwareMap.get(DcMotor.class, "rightgrabber");
        gyro = hardwareMap.get(ModernRoboticsI2cGyro.class, "Gyro");
        color = hardwareMap.get(ColorSensor.class, "sensor_color");
        rangebrothers = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "rangebrothers");
        rangebrothers2 = hardwareMap.get(Rev2mDistanceSensor.class, "rangebrothers2");


        backleft.setDirection(DcMotor.Direction.FORWARD);
        frontleft.setDirection(DcMotor.Direction.FORWARD);
        backright.setDirection(DcMotor.Direction.REVERSE);
        frontright.setDirection(DcMotor.Direction.REVERSE);
        lift.setDirection(DcMotor.Direction.FORWARD);
        leftgrabber.setDirection(DcMotor.Direction.FORWARD);
        rightgrabber.setDirection(DcMotor.Direction.FORWARD);



        backleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftgrabber.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightgrabber.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        backleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftgrabber.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightgrabber.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        backleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftgrabber.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightgrabber.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        backleft.setPower(0);
        frontleft.setPower(0);
        backright.setPower(0);
        frontright.setPower(0);
        lift.setPower(0);
        leftgrabber.setPower(0);
        rightgrabber.setPower(0);

        VisionRight detector = new VisionRight(this);


        telemetry.addLine("Start Gyro");
        telemetry.update();
        gyro.calibrate();
        while (gyro.isCalibrating()) ;
        telemetry.addLine("Gyro Calibrated");
        telemetry.addData("Angle: ", gyro.getIntegratedZValue());
        telemetry.update();


        waitForStart();

        telemetry.addData("Red", detector.one);
        telemetry.addData("Green", detector.two);
        telemetry.addData("Blue", detector.three);
        telemetry.update();

        //pos strafe = left strafing
        //high =
        //mid =
        //low =




        if (detector.one == true) {


            gyroDrive(DRIVE_SPEED06, 11, 11, 11, 11, 0);
            drivebackleftandfrontright(32,DRIVE_SPEED06);
            gyroTurn(TURN_SPEED, 88);
            gyroHold(TURN_SPEED_FIX,88, TIME);
            gyroDrive(DRIVE_SPEED07,-10,-10,-10,-10,88);
            golift(31,.6);
            gyroHold(TURN_SPEED_FIX,88, TIME);
            gyroDrive(DRIVE_SPEED07,-36,-36,-36,-36,88);
            gyroTurn(TURN_SPEED, 53);
            gyroHold(TURN_SPEED_FIX,53, TIME);
            gyroDrive(DRIVE_SPEED07,-13.8,-13.8,-13.8,-13.8,53);


            //drop cone at high 1
            letgogirl();
            gyroDrive(DRIVE_SPEED07,13,13,13,13,41);
            gyroTurn(TURN_SPEED, 88);
            gyroHold(TURN_SPEED_FIX,88, TIME);
            gyroDrive(DRIVE_SPEED07,6,6,6,6,0);

//            gyroDrive(DRIVE_SPEED07,10.8,10.8,10.8,10.8,39);
//            gyroHold(TURN_SPEED_FIX,41, TIME);
//            golift(-19, .6);
//            gyroTurn(TURN_SPEED,0);
//            gyroHold(TURN_SPEED_FIX,0, TIME);
//            gyroDrive(DRIVE_SPEED08, -56, -56, -56, -56,0);
//            gyroDrive(DRIVE_SPEED05, -5, -5, -5, -5,0);
//            findline(0.5);
//            golift(-6,.7);
//            getitgirl();
//            golift(26, .9);
//            gyroDrive(DRIVE_SPEED07, 22, 22, 22, 22,0);
//            gyroHold(TURN_SPEED_FIX,0, TIME);
//            gyroTurn(TURN_SPEED, 142.5);
//            gyroHold(TURN_SPEED_FIX,142.5, TIME);
//
//            gyroDrive(DRIVE_SPEED07,-13.5, -13.5,-13.5,-13.5,142.5);
////
////            //drop cone at high 2
//            letgogirl();
//
//            gyroDrive(DRIVE_SPEED07,13.5,13.5,13.5,13.5,142);
//            golift(-19,.7);
//            gyroTurn(TURN_SPEED,4);
//            gyroHold(TURN_SPEED_FIX,4, TIME);
//            gyroDrive(DRIVE_SPEED07, -24,-24, -24, -24,4);
//
//            golift(-9,.7);
//            getitgirl();
//            golift(26, .9);
//            gyroDrive(DRIVE_SPEED07, 23, 23, 23, 23,0);
//
//            gyroTurn(TURN_SPEED, 145);
//            gyroHold(TURN_SPEED_FIX,145, .5);
//            gyroDrive(DRIVE_SPEED07,-12,-12,-12,-12,145);
//
//            letgogirl();
//            gyroDrive(DRIVE_SPEED08,12,12,12,12,140);
//            golift(-27,1);
//            gyroTurn(1, -180);
//            gyroDrive(1,-25,-25,-25,-25,-180);
//
//
//
//
        } else if (detector.two == true) {
            gyroDrive(DRIVE_SPEED06, 11, 11, 11, 11, 0);
            drivebackleftandfrontright(32,DRIVE_SPEED06);
            gyroTurn(TURN_SPEED, 88);
            gyroHold(TURN_SPEED_FIX,88, TIME);
            gyroDrive(DRIVE_SPEED07,-10,-10,-10,-10,88);
            golift(31,.6);
            gyroHold(TURN_SPEED_FIX,88, TIME);
            gyroDrive(DRIVE_SPEED07,-36,-36,-36,-36,88);
            gyroTurn(TURN_SPEED, 53);
            gyroHold(TURN_SPEED_FIX,53, TIME);
            gyroDrive(DRIVE_SPEED07,-13.8,-13.8,-13.8,-13.8,53);


            //drop cone at high 1
            letgogirl();
            gyroDrive(DRIVE_SPEED07,13,13,13,13,41);
            gyroTurn(TURN_SPEED, 0);
            gyroHold(TURN_SPEED_FIX,0, TIME);
            gyroDrive(DRIVE_SPEED07,-24,-24,-24,-24,0);

//            gyroDrive(DRIVE_SPEED07,11,11,11,11,0);
//            drivebackleftandfrontright(32,DRIVE_SPEED05);
//            gyroTurn(TURN_SPEED, 88);
//            gyroHold(TURN_SPEED_FIX,88, TIME);
//            gyroDrive(DRIVE_SPEED07,-13,-13,-13,-13,88);
//            golift(31,1);
//            gyroHold(TURN_SPEED_FIX,88, TIME);
//            gyroDrive(DRIVE_SPEED07,-34,-34,-34,-34,88);
//            gyroTurn(TURN_SPEED, 39.5);
//            gyroHold(TURN_SPEED_FIX,39, TIME);
//            gyroDrive(DRIVE_SPEED07,-10,-10,-10,-10,39);
//
//
////            //drop cone at high 1
//            letgogirl();
//            gyroDrive(DRIVE_SPEED07,10.3,10.3,10.3,10.3,39);
//            gyroHold(TURN_SPEED_FIX,41, TIME);
//            golift(-19, .6);
//            gyroTurn(TURN_SPEED,0);
//            gyroHold(TURN_SPEED_FIX,0, TIME);
//            gyroDrive(DRIVE_SPEED08, -56, -56, -56, -56,0);
//            gyroDrive(DRIVE_SPEED05, -5, -5, -5, -5,0);
//            findline(0.5);
//            golift(-6,.7);
//            getitgirl();
//            golift(26, .9);
//            gyroDrive(DRIVE_SPEED07, 22, 22, 22, 22,0);
//            gyroHold(TURN_SPEED_FIX,0, TIME);
//            gyroTurn(TURN_SPEED, 142.5);
//            gyroHold(TURN_SPEED_FIX,142.5, TIME);
//
//            gyroDrive(DRIVE_SPEED07,-13.5, -13.5,-13.5,-13.5,142.5);
////
////            //drop cone at high 2
//            letgogirl();
//
//            gyroDrive(DRIVE_SPEED07,13.5,13.5,13.5,13.5,142);
//            golift(-19,.7);
//            gyroTurn(TURN_SPEED,4);
//            gyroHold(TURN_SPEED_FIX,4, TIME);
//            gyroDrive(DRIVE_SPEED07, -24,-24, -24, -24,4);
//
//            golift(-9,.7);
//            getitgirl();
//            golift(26, .9);
//            gyroDrive(DRIVE_SPEED07, 23, 23, 23, 23,0);
//
//            gyroTurn(TURN_SPEED, 145);
//            gyroHold(TURN_SPEED_FIX,145, .5);
//            gyroDrive(DRIVE_SPEED07,-12,-12,-12,-12,145);
//
//            letgogirl();
//            gyroDrive(1,23,23,23,23,88);
//
        }
//
         else if (detector.three == true) {



            gyroDrive(DRIVE_SPEED06, 11, 11, 11, 11, 0);
            drivebackleftandfrontright(32,DRIVE_SPEED06);
            gyroTurn(TURN_SPEED, 88);
            gyroHold(TURN_SPEED_FIX,88, TIME);
            gyroDrive(DRIVE_SPEED07,-10,-10,-10,-10,88);
            golift(31,.6);
            gyroHold(TURN_SPEED_FIX,88, TIME);
            gyroDrive(DRIVE_SPEED07,-36,-36,-36,-36,88);
            gyroTurn(TURN_SPEED, 53);
            gyroHold(TURN_SPEED_FIX,53, TIME);
            gyroDrive(DRIVE_SPEED07,-13.8,-13.8,-13.8,-13.8,53);


            //drop cone at high 1
            letgogirl();
            gyroDrive(DRIVE_SPEED07,13,13,13,13,41);
            gyroTurn(TURN_SPEED, 0);
            gyroHold(TURN_SPEED_FIX,0, TIME);
            gyroDrive(DRIVE_SPEED07,-55,-55,-55,-55,0);
//
//            gyroDrive(DRIVE_SPEED07,11,11,11,11,0);
//            drivebackleftandfrontright(32,DRIVE_SPEED05);
//            gyroTurn(TURN_SPEED, 88);
//            gyroHold(TURN_SPEED_FIX,88, TIME);
//            gyroDrive(DRIVE_SPEED07,-13,-13,-13,-13,88);
//            golift(31,1);
//            gyroHold(TURN_SPEED_FIX,88, TIME);
//            gyroDrive(DRIVE_SPEED07,-34,-34,-34,-34,88);
//            gyroTurn(TURN_SPEED, 39.5);
//            gyroHold(TURN_SPEED_FIX,39, TIME);
//            gyroDrive(DRIVE_SPEED07,-10.5,-10.5,-10.5,-10.5,39);
//
//
////            //drop cone at high 1
//            letgogirl();
//            gyroDrive(DRIVE_SPEED07,10.8,10.8,10.8,10.8,39);
//            gyroHold(TURN_SPEED_FIX,41, TIME);
//            golift(-19, .6);
//            gyroTurn(TURN_SPEED,0);
//            gyroHold(TURN_SPEED_FIX,0, TIME);
//            gyroDrive(DRIVE_SPEED08, -56, -56, -56, -56,0);
//            gyroDrive(DRIVE_SPEED05, -5, -5, -5, -5,0);
//            findline(1);
//            golift(-6,.7);
//            getitgirl();
//            golift(26, .9);
//            gyroDrive(DRIVE_SPEED07, 22, 22, 22, 22,0);
//            gyroHold(TURN_SPEED_FIX,0, TIME);
//            gyroTurn(TURN_SPEED, 142.5);
//            gyroHold(TURN_SPEED_FIX,142.5, TIME);
//
//            gyroDrive(DRIVE_SPEED07,-13.5, -13.5,-13.5,-13.5,142.5);
////
////            //drop cone at high 2
//            letgogirl();
//
//            gyroDrive(DRIVE_SPEED07,13.5,13.5,13.5,13.5,142);
//            golift(-19,.7);
//            gyroTurn(TURN_SPEED,4);
//            gyroHold(TURN_SPEED_FIX,4, TIME);
//            gyroDrive(DRIVE_SPEED07, -24,-24, -24, -24,4);
//
//            golift(-9,.7);
//            getitgirl();
//            golift(26, .9);
//            gyroDrive(DRIVE_SPEED07, 23, 23, 23, 23,0);
//
//            gyroTurn(TURN_SPEED, 145);
//            gyroHold(TURN_SPEED_FIX,145, .5);
//            gyroDrive(DRIVE_SPEED07,-12,-12,-12,-12,145);
//
//            letgogirl();
//            gyroDrive(DRIVE_SPEED08,12,12,12,12,140);
//            golift(-27,1);
//            gyroTurn(1, -15);
//            gyroDrive(1,-25,-25,-25,-25,-15);
//
//
//
        }


        }











    public void rangeDrive (double speed, double backDistance) {

        speed = Range.clip(Math.abs(speed), 0.0, 1.0);

//        if (rangebrothers.getDistance(DistanceUnit.INCH) >= backDistance) {
            while (rangebrothers.getDistance(DistanceUnit.INCH) < backDistance) {
                if (Math.abs(rangebrothers.getDistance(DistanceUnit.INCH) - backDistance) > MAX_ACCEPTABLE_ERROR) {
                    if (!rangeCheck(rangebrothers, backDistance)) {
                        break;
                    }
                }
                frontleft.setPower(speed);
                backleft.setPower(speed);
                frontright.setPower(speed);
                backright.setPower(speed);

                telemetry.addData("Sensor Back Distance: ", rangebrothers.getDistance(DistanceUnit.INCH));
                telemetry.addData("Target Back Distance: ", backDistance);
                telemetry.addLine("Moving Backwards");
                telemetry.update();
            }
//            while (rangebrothers.getDistance(DistanceUnit.INCH) > backDistance) {
//                if (Math.abs(rangebrothers.getDistance(DistanceUnit.INCH) - backDistance) > MAX_ACCEPTABLE_ERROR) {
//                    if (!rangeCheck(rangebrothers, backDistance)) {
//                        break;
//                    }
//                }
//                frontleft.setPower(-speed);
//                backleft.setPower(-speed);
//                frontright.setPower(-speed);
//                backright.setPower(-speed);
//
//                telemetry.addData("Sensor Back Distance: ", rangebrothers.getDistance(DistanceUnit.INCH));
//                telemetry.addData("Target Back Distance: ", backDistance);
//                telemetry.addLine("Moving Forwards");
//                telemetry.update();
//            }
        frontleft.setPower(0);
        backleft.setPower(0);
        frontright.setPower(0);
        backright.setPower(0);


        }



        private boolean rangeCheck(ModernRoboticsI2cRangeSensor range_sensor, double desired_distance){
            final int TRIES = 3;
            for (int i = 0; i < TRIES; i++){
                if (Math.abs(range_sensor.getDistance(DistanceUnit.INCH) - desired_distance) < MAX_ACCEPTABLE_ERROR){
                    return true;
                }
                telemetry.addData("TRY ",i);
                telemetry.addData("Range Value: ", range_sensor.getDistance(DistanceUnit.INCH));
                telemetry.addData("Target: ", desired_distance);
                telemetry.update();
                try {
                    Thread.sleep(1500);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
            return false;
        }






    public void sensewall1(){
        while (rangebrothers.getDistance(DistanceUnit.CM) <= 85) {
            backright.setPower(.3);
            backleft.setPower(.3);
            frontright.setPower(.3);
            frontleft.setPower(.3);

        }
    }

    public void sensewall2(){
        while (rangebrothers.getDistance(DistanceUnit.CM) <= 52) {
            backright.setPower(.3);
            backleft.setPower(.3);
            frontright.setPower(.3);
            frontleft.setPower(.3);

        }
        backright.setPower(0);
        backleft.setPower(0);
        frontright.setPower(0);
        frontleft.setPower(0);
    }


    public void findline(double holdTime){


        ElapsedTime holdTimer = new ElapsedTime();

        // keep looping while we have time remaining.
        holdTimer.reset();
            while (color.blue() <= 270 && (holdTimer.time() < holdTime)) {
                backright.setPower(-.05);
                backleft.setPower(.05);
                frontright.setPower(.05);
                frontleft.setPower(-.05);

        }

        backright.setPower(0);
        backleft.setPower(0);
        frontright.setPower(0);
        frontleft.setPower(0);
    }

    public void moveToPosition(double inches, double speed){
        //
        int move = (int)(Math.round(inches*conversion));
        //
        backleft.setTargetPosition(backleft.getCurrentPosition() + move);
        frontleft.setTargetPosition(frontleft.getCurrentPosition() + move);
        backright.setTargetPosition(backright.getCurrentPosition() + move);
        frontright.setTargetPosition(frontright.getCurrentPosition() + move);
        //
        frontleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backright.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //
        frontleft.setPower(speed);
        backleft.setPower(speed);
        frontright.setPower(speed);
        backright.setPower(speed);
        //
        while (frontleft.isBusy() && frontright.isBusy() && backleft.isBusy() && backright.isBusy()){
            if (exit){
                frontright.setPower(0);
                frontleft.setPower(0);
                backright.setPower(0);
                backleft.setPower(0);
                return;
            }
        }
        frontright.setPower(0);
        frontleft.setPower(0);
        backright.setPower(0);
        backleft.setPower(0);
        frontright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        return;


    }




    public void drivebackleftandfrontright(double inches, double speed) {


        int move = (int) (Math.round(inches * cpi * meccyBias));

        frontleft.setTargetPosition(frontleft.getCurrentPosition() + move);
        backright.setTargetPosition(backright.getCurrentPosition() + move);

        frontleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backright.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontleft.setPower(speed);
        backright.setPower(speed);

        while ( frontleft.isBusy() && backright.isBusy()) {
        }

        frontleft.setPower(0);
        backright.setPower(0);
        return;

    }


    public void golift(double inches, double speed){

        telemetry.addData("Encodercount", lift.getCurrentPosition() );
        telemetry.update();
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        int move =  -(int)(Math.round(inches*93.3));

        lift.setTargetPosition(lift.getCurrentPosition() + move);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setPower(speed);
    }


    public void letgogirl(){
        golift(-6,.7);
        while(lift.isBusy()){
        }
        leftgrabber.setPower(-1);
        rightgrabber.setPower(1);
        sleep(700);
        rightgrabber.setPower(0);
        leftgrabber.setPower(0);
        golift(6,.9);
    }


    public void getitgirl(){
        while(lift.isBusy()){
        }
        leftgrabber.setPower(.7);
        rightgrabber.setPower(-.7);
        sleep(1000);
        rightgrabber.setPower(0);
        leftgrabber.setPower(0);
    }




//    public void turnWithGyro(double degrees, double speedDirection) {
//        //<editor-fold desc="Initialize">
//        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//        double yaw = -angles.firstAngle;//make this negative
//        telemetry.addData("Speed Direction", speedDirection);
//        telemetry.addData("Yaw", yaw);
//        telemetry.update();
//        //
//        telemetry.addData("stuff", speedDirection);
//        telemetry.update();
//        //
//        double first;
//        double second;
//        //</editor-fold>
//        //
//        if (speedDirection > 0) {//set target positions
//            //<editor-fold desc="turn right">
//            if (degrees > 10) {
//                first = (degrees - 10) + devertify(yaw);
//                second = degrees + devertify(yaw);
//            } else {
//                first = devertify(yaw);
//                second = degrees + devertify(yaw);
//            }
//            //</editor-fold>
//        } else {
//            //<editor-fold desc="turn left">
//            if (degrees > 10) {
//                first = devertify(-(degrees - 10) + devertify(yaw));
//                second = devertify(-degrees + devertify(yaw));
//            } else {
//                first = devertify(yaw);
//                second = devertify(-degrees + devertify(yaw));
//            }
//            //
//            //</editor-fold>
//        }
//        //
//        //<editor-fold desc="Go to position">
//        Double firsta = convertify(first - 5);//175
//        Double firstb = convertify(first + 5);//-175
//        //
//        turnWithEncoder(speedDirection);
//        //
//        if (Math.abs(firsta - firstb) < 11) {
//            while (!(firsta < yaw && yaw < firstb) && opModeIsActive()) {//within range?
//                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//                gravity = imu.getGravity();
//                yaw = -angles.firstAngle;
//                telemetry.addData("Position", yaw);
//                telemetry.addData("first before", first);
//                telemetry.addData("first after", convertify(first));
//                telemetry.update();
//            }
//        } else {
//            //
//            while (!((firsta < yaw && yaw < 180) || (-180 < yaw && yaw < firstb)) && opModeIsActive()) {//within range?
//                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//                gravity = imu.getGravity();
//                yaw = -angles.firstAngle;
//                telemetry.addData("Position", yaw);
//                telemetry.addData("first before", first);
//                telemetry.addData("first after", convertify(first));
//                telemetry.update();
//            }
//        }
//        //
//        Double seconda = convertify(second - 5);//175
//        Double secondb = convertify(second + 5);//-175
//        //
//        turnWithEncoder(speedDirection / 3);
//        //
//        if (Math.abs(seconda - secondb) < 11) {
//            while (!(seconda < yaw && yaw < secondb) && opModeIsActive()) {//within range?
//                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//                gravity = imu.getGravity();
//                yaw = -angles.firstAngle;
//                telemetry.addData("Position", yaw);
//                telemetry.addData("second before", second);
//                telemetry.addData("second after", convertify(second));
//                telemetry.update();
//            }
//            while (!((seconda < yaw && yaw < 180) || (-180 < yaw && yaw < secondb)) && opModeIsActive()) {//within range?
//                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//                gravity = imu.getGravity();
//                yaw = -angles.firstAngle;
//                telemetry.addData("Position", yaw);
//                telemetry.addData("second before", second);
//                telemetry.addData("second after", convertify(second));
//                telemetry.update();
//            }
//            frontleft.setPower(0);
//            frontright.setPower(0);
//            backleft.setPower(0);
//            backright.setPower(0);
//        }
//        //</editor-fold>
//        //
//        frontleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        frontright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        backleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        backright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        frontleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        frontright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        backleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        backright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//    }

    //
    /*
    This function uses the encoders to strafe left or right.
    Negative input for inches results in left strafing.
     */
    public void strafeToPosition(double inches, double speed) {
        //
        int move = (int) (Math.round(inches * cpi * meccyBias));
        //
        backleft.setTargetPosition(backleft.getCurrentPosition() - move);
        frontleft.setTargetPosition(frontleft.getCurrentPosition() + move);
        backright.setTargetPosition(backright.getCurrentPosition() + move);
        frontright.setTargetPosition(frontright.getCurrentPosition() - move);
        //
        frontleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //
        frontleft.setPower(speed);
        backleft.setPower(speed);
        frontright.setPower(speed);
        backright.setPower(speed);
        //
        while (frontleft.isBusy() && frontright.isBusy() && backleft.isBusy() && backright.isBusy()) {
        }
        frontright.setPower(0);
        frontleft.setPower(0);
        backright.setPower(0);
        backleft.setPower(0);
        return;
    }

    //
    /*
    These functions are used in the turnWithGyro function to ensure inputs
    are interpreted properly.
     */
    public double devertify(double degrees) {
        if (degrees < 0) {
            degrees = degrees + 360;
        }
        return degrees;
    }

    public double convertify(double degrees) {
        if (degrees > 179) {
            degrees = -(360 - degrees);
        } else if (degrees < -180) {
            degrees = 360 + degrees;
        } else if (degrees > 360) {
            degrees = degrees - 360;
        }
        return degrees;
    }

    //
    /*
    This function is called at the beginning of the program to activate
    the IMU Integrated Gyro.
     */
//    public void initGyro() {
//        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
//        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
//        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
//        //parameters.calibrationDataFile = "GyroCal.json"; // see the calibration sample opmode
//        parameters.loggingEnabled = true;
//        parameters.loggingTag = "IMU";
//        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
//        //
//        imu = hardwareMap.get(BNO055IMU.class, "imu");
//        imu.initialize(parameters);
//    }

    //
    /*
    This function is used in the turnWithGyro function to set the
    encoder mode and turn.
     */
    public void turnWithEncoder(double input) {
        frontleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //
        frontleft.setPower(input);
        backleft.setPower(input);
        frontright.setPower(-input);
        backright.setPower(-input);
    }

    public void gyroDrive(double speed,
                          double frontLeftInches, double frontRightInches, double backLeftInches,
                          double backRightInches,
                          double angle) {

        int newFrontLeftTarget;
        int newFrontRightTarget;
        int newBackLeftTarget;
        int newBackRightTarget;

        double HalfMaxOne;
        double HalfMaxTwo;

        double max;

        double error;
        double steer;
        double frontLeftSpeed;
        double frontRightSpeed;
        double backLeftSpeed;
        double backRightSpeed;

        double ErrorAmount;
        boolean goodEnough = false;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newFrontLeftTarget = frontleft.getCurrentPosition() + (int) (frontLeftInches * cpi);
            newFrontRightTarget = frontright.getCurrentPosition() + (int) (frontRightInches * cpi);
            newBackLeftTarget = backleft.getCurrentPosition() + (int) (backLeftInches * cpi);
            newBackRightTarget = backright.getCurrentPosition() + (int) (backRightInches * cpi);


            // Set Target and Turn On RUN_TO_POSITION
            frontleft.setTargetPosition(newFrontLeftTarget);
            frontright.setTargetPosition(newFrontRightTarget);
            backleft.setTargetPosition(newBackLeftTarget);
            backright.setTargetPosition(newBackRightTarget);

            frontleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backright.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // start motion.
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            frontleft.setPower(Math.abs(speed));
            frontright.setPower(Math.abs(speed));
            backleft.setPower(Math.abs(speed));
            backright.setPower(Math.abs(speed));
            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                    ((frontleft.isBusy() && frontright.isBusy()) && (backleft.isBusy() && backright.isBusy())) && !goodEnough) {


                // adjust relative speed based on heading error.
                error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (frontLeftInches < 0 && frontRightInches < 0 && backLeftInches < 0 && backRightInches < 0)
                    steer *= -1.0;

                frontLeftSpeed = speed - steer;
                backLeftSpeed = speed - steer;
                backRightSpeed = speed + steer;
                frontRightSpeed = speed + steer;

                // Normalize speeds if either one exceeds +/- 1.0;
                HalfMaxOne = Math.max(Math.abs(frontLeftSpeed), Math.abs(backLeftSpeed));
                HalfMaxTwo = Math.max(Math.abs(frontRightSpeed), Math.abs(backRightSpeed));
                max = Math.max(Math.abs(HalfMaxOne), Math.abs(HalfMaxTwo));
                if (max > 1.0) {
                    frontLeftSpeed /= max;
                    frontRightSpeed /= max;
                    backLeftSpeed /= max;
                    backRightSpeed /= max;
                }

                frontleft.setPower(frontLeftSpeed);
                frontright.setPower(frontRightSpeed);
                backleft.setPower(backLeftSpeed);
                backright.setPower(backRightSpeed);

                // Display drive status for the driver.
//                telemetry.addData("Err/St", "%5.1f/%5.1f", error, steer);
//                telemetry.addData("Target", "%7d:%7d", newBackLeftTarget, newBackRightTarget, newFrontLeftTarget, newFrontRightTarget);
//                telemetry.addData("Actual", "%7d:%7d", backleft.getCurrentPosition(), backright.getCurrentPosition(), frontleft.getCurrentPosition(), frontright.getCurrentPosition());
//                telemetry.addData("Speed", "%5.2f:%5.2f", backLeftSpeed, backRightSpeed, frontLeftSpeed, frontRightSpeed);
//                telemetry.update();

                ErrorAmount = ((Math.abs(((newBackLeftTarget) - (backleft.getCurrentPosition())))
                        + (Math.abs(((newFrontLeftTarget) - (frontleft.getCurrentPosition()))))
                        + (Math.abs((newBackRightTarget) - (backright.getCurrentPosition())))
                        + (Math.abs(((newFrontRightTarget) - (frontright.getCurrentPosition()))))) / cpi);
                if (ErrorAmount < amountError) {
                    goodEnough = true;
                }
            }

            // Stop all motion;
            frontleft.setPower(0);
            frontright.setPower(0);
            backleft.setPower(0);
            backright.setPower(0);

            // Turn off RUN_TO_POSITION

            frontleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void gyroTurn(double speed, double angle) {
        frontleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF)) {
            // Update telemetry & Allow time for other processes to run.
            telemetry.update();
        }
        frontleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backright.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }

    /**
     * Method to obtain & hold a heading for a finite amount of time
     * Move will stop once the requested time has elapsed
     *
     * @param speed    Desired speed of turn.
     * @param angle    Absolute Angle (in Degrees) relative to last gyro reset.
     *                 0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                 If a relative angle is required, add/subtract from current heading.
     * @param holdTime Length of time (in seconds) to hold the specified heading.
     */


    public void gyroHold(double speed, double angle, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();

        // keep looping while we have time remaining.
        holdTimer.reset();
        while (opModeIsActive() && (holdTimer.time() < holdTime)) {
            // Update telemetry & Allow time for other processes to run.
            onHeading(speed, angle, P_TURN_COEFF);
            telemetry.update();


        }

        // Stop all motion;
        frontleft.setPower(0);
        backleft.setPower(0);
        backright.setPower(0);
        frontright.setPower(0);
    }


    /**
     * Perform one cycle of closed loop heading control.
     *
     * @param speed  Desired speed of turn.
     * @param angle  Absolute Angle (in Degrees) relative to last gyro reset.
     *               0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *               If a relative angle is required, add/subtract from current heading.
     * @param PCoeff Proportional Gain coefficient
     * @return
     */
    boolean onHeading(double speed, double angle, double PCoeff) {
        double error;
        double steer;
        boolean onTarget = false;
        double leftSpeed;
        double rightSpeed;

        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        } else {
            steer = getSteer(error, PCoeff);
            rightSpeed = speed * steer;
            leftSpeed = -rightSpeed;
        }

        // Send desired speeds to motors.
        frontleft.setPower(leftSpeed);
        backleft.setPower(leftSpeed);
        backright.setPower(rightSpeed);
        frontright.setPower(rightSpeed);

        // Display it for the driver.
//        telemetry.addData("Target", "%5.2f", angle);
//        telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
//        telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);

        return onTarget;
    }

    /**
     * getError determines the error between the target angle and the robot's current heading
     *
     * @param targetAngle Desired angle (relative to global reference established at last Gyro Reset).
     * @return error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
     * +ve error means the robot should turn LEFT (CCW) to reduce error.
     */
    public double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - gyro.getHeading();
        while (robotError > 180) robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return -robotError;
    }

    /**
     * returns desired steering force.  +/- 1 range.  +ve = steer left
     *
     * @param error  Error angle in robot relative degrees
     * @param PCoeff Proportional Gain Coefficient
     * @return
     */
    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -DRIVE_SPEED07, 1);
    }
}
