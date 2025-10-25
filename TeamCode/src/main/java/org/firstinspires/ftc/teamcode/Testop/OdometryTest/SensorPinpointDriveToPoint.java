package org.firstinspires.ftc.teamcode.Testop.OdometryTest;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.Locale;

@Autonomous(name="Pinpoint Navigation Example", group="Pinpoint")
//@Disabled



public class SensorPinpointDriveToPoint extends LinearOpMode {

    DcMotor leftFrontDrive;
    DcMotor rightFrontDrive;
    DcMotor leftBackDrive;
    DcMotor rightBackDrive;

    GoBildaPinpointDriver odo; // Declare OpMode member for the Odometry Computer
    DriveToPoint nav = new DriveToPoint(this); //OpMode member for the point-to-point navigation class

    Pose2D makeTarget(double xpos, double ypos, double hpos){
        return new Pose2D(DistanceUnit.MM,xpos,ypos, AngleUnit.DEGREES,hpos);
    }


    void DriveToTarget(Pose2D TARGET, double power, double holdTime, double powerMultiplier){
        while(opModeIsActive()){
            odo.update();
            //        telemetry.addData("Reached target", nav.driveTo(odo.getPosition(), TARGET_1, 0.7, 0));
            if (nav.driveTo(odo.getPosition(), TARGET, power, holdTime)){
                telemetry.addLine("at position #1!");
    //                        odo.resetPosAndIMU();
    //                        sleep(300);
                powerMultiplier = 0.85;
                break;
            }
            else{
                telemetry.addLine("going to position #1");
                if(getRuntime()>1 && powerMultiplier != 1){
                    powerMultiplier = 1;
                }
            }
            leftFrontDrive.setPower(powerMultiplier*nav.getMotorPower(DriveToPoint.DriveMotor.LEFT_FRONT));
            rightFrontDrive.setPower(powerMultiplier*nav.getMotorPower(DriveToPoint.DriveMotor.RIGHT_FRONT));
            leftBackDrive.setPower(powerMultiplier*nav.getMotorPower(DriveToPoint.DriveMotor.LEFT_BACK));
            rightBackDrive.setPower(powerMultiplier*nav.getMotorPower(DriveToPoint.DriveMotor.RIGHT_BACK));

//            telemetry.addData("current state:",stateMachine);
            telemetry.addData("LF motor power:",nav.getMotorPower(DriveToPoint.DriveMotor.LEFT_FRONT));
            telemetry.addData("RF motor power:",nav.getMotorPower(DriveToPoint.DriveMotor.RIGHT_FRONT));
            telemetry.addData("LB motor power:",nav.getMotorPower(DriveToPoint.DriveMotor.LEFT_BACK));
            telemetry.addData("RB motor power:",nav.getMotorPower(DriveToPoint.DriveMotor.RIGHT_BACK));


            Pose2D pos = odo.getPosition();
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Position", data);

            telemetry.update();
        }
    }

    // Note: will overshoot the x-coordinate by the tolerance, but not the y-coordinate
    static final Pose2D TARGET_1 = new Pose2D(DistanceUnit.MM,200,0, AngleUnit.DEGREES,0);
    static final Pose2D TARGET_2 = new Pose2D(DistanceUnit.MM, 300, -200, AngleUnit.DEGREES, 0);
    static double powerMultiplier = 0.85;
//    static final Pose2D TARGET_3 = new Pose2D(DistanceUnit.MM, 300,150, AngleUnit.DEGREES,0);
//    static final Pose2D TARGET_4 = new Pose2D(DistanceUnit.MM, 100, , AngleUnit.DEGREES, 90);
//    static final Pose2D TARGET_5 = new Pose2D(DistanceUnit.MM, 100, 0, AngleUnit.DEGREES, 0);


    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.

        leftFrontDrive  = hardwareMap.get(DcMotor.class, "fl");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "fr");
        leftBackDrive   = hardwareMap.get(DcMotor.class, "rl");
        rightBackDrive  = hardwareMap.get(DcMotor.class, "rr");

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        odo = hardwareMap.get(GoBildaPinpointDriver.class,"odo");
//        odo.setOffsets(-142.0, 120.0); //these are tuned for 3110-0002-0001 Product Insight #1
        odo.setOffsets(-67.0, -168.0); // change later ?

        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);

        odo.recalibrateIMU();

        odo.resetPosAndIMU();

        //nav.setXYCoefficients(0.02,0.002,0.0,DistanceUnit.MM,12);
        //nav.setYawCoefficients(1,0,0.0, AngleUnit.DEGREES,2);
        nav.setDriveType(DriveToPoint.DriveType.MECANUM);

        telemetry.addData("Status", "Initialized");
        telemetry.addData("X offset", odo.getXOffset());
        telemetry.addData("Y offset", odo.getYOffset());
        telemetry.addData("Device Version Number:", odo.getDeviceVersion());
        telemetry.addData("Device Scalar", odo.getYawScalar());
        telemetry.update();

        // Wait for the game to start (driver presses START)
        waitForStart();
        resetRuntime();
//        Pose2D CurrentTARGET = TARGET_1;
        double power = 0.7, holdTime = 0.5;
        DriveToTarget(makeTarget(200,0,0), power, holdTime, 0.7);
        DriveToTarget(makeTarget(200,200,0), power, holdTime, 0.7);
        DriveToTarget(makeTarget(-200,0,0), power, holdTime, 0.7);
        DriveToTarget(makeTarget(200,-200,0), power, holdTime, 0.7);
        DriveToTarget(makeTarget(0,0,90), power, holdTime, 0.7);
        DriveToTarget(makeTarget(0,0,-90), power, holdTime, 0.7);
        DriveToTarget(makeTarget(0,0,179), power, holdTime, 0.7);
        DriveToTarget(makeTarget(0,0,0), power, holdTime, 0.7);


    }}
