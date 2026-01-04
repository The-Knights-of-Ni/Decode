package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Subsystems.Drive.DriveToPoint;
import org.firstinspires.ftc.teamcode.Util.AllianceColor;

import java.util.HashMap;
import java.util.Locale;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@Autonomous(name = "Auto2025")
public class Auto2025 extends LinearOpMode {
    ElapsedTime timer;
    private Robot robot;
    double timeCurrent;
    double timePre;
    DcMotor frontLeftMotor;
    DcMotor backLeftMotor;
    DcMotor frontRightMotor;
    DcMotor backRightMotor;

    private void initOpMode() {
        // Initialize DC motor objects
        timer = new ElapsedTime();
        HashMap<String, Boolean> flags = new HashMap<>();
        flags.put("web", true);
        flags.put("vision", false);
        DriveToPoint nav = new DriveToPoint(this);
        this.robot = new Robot(hardwareMap, telemetry, timer, nav, AllianceColor.BLUE, gamepad1, gamepad2, flags);
        timeCurrent = timer.nanoseconds();
        timePre = timeCurrent;

        telemetry.addData("Waiting for start", "...");
    }

    void DriveToTarget(Pose2D TARGET, double power, double holdTime, double powerMultiplier, double patience, double timeOut){
        double startTime = getRuntime();
        while(opModeIsActive()){
            robot.odo.update();
            if (robot.nav.driveTo(robot.odo.getPosition(), TARGET, power, holdTime)){
                telemetry.addLine("at position #1!");
                // Sleep to give the reset position time, as it takes 0.25s
                //                        odo.resetPosAndIMU();
                //                        sleep(300);
                powerMultiplier = 0.85;
                break;
            }
            else{
                telemetry.addLine("going to position #1");
                if(getRuntime()-startTime>patience && powerMultiplier != 1){
                    // If the robot has not reached the target after an adjustable amount of time, make it go faster
                    powerMultiplier = 1;
                }
                if(getRuntime()-startTime>timeOut && timeOut>0){
                    // Stop if robot is taking too long if timeout is positive
                    telemetry.addData("Exceeded time limit (seconds) of ", timeOut);
                    break;
                }
            }
            frontLeftMotor.setPower(powerMultiplier*robot.nav.getMotorPower(DriveToPoint.DriveMotor.LEFT_FRONT));
            frontRightMotor.setPower(powerMultiplier*robot.nav.getMotorPower(DriveToPoint.DriveMotor.RIGHT_FRONT));
            backLeftMotor.setPower(powerMultiplier*robot.nav.getMotorPower(DriveToPoint.DriveMotor.LEFT_BACK));
            backRightMotor.setPower(powerMultiplier*robot.nav.getMotorPower(DriveToPoint.DriveMotor.RIGHT_BACK));

            telemetry.addData("LF motor power:",robot.nav.getMotorPower(DriveToPoint.DriveMotor.LEFT_FRONT));
            telemetry.addData("RF motor power:",robot.nav.getMotorPower(DriveToPoint.DriveMotor.RIGHT_FRONT));
            telemetry.addData("LB motor power:",robot.nav.getMotorPower(DriveToPoint.DriveMotor.LEFT_BACK));
            telemetry.addData("RB motor power:",robot.nav.getMotorPower(DriveToPoint.DriveMotor.RIGHT_BACK));

            Pose2D pos = robot.odo.getPosition();
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Position", data);

        }
    }

    Pose2D makeTarget(double xpos, double ypos, double hpos){
        return new Pose2D(DistanceUnit.MM,xpos,ypos, AngleUnit.DEGREES,hpos);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initOpMode();

        frontLeftMotor = hardwareMap.dcMotor.get("fl"); //1 port
        backLeftMotor = hardwareMap.dcMotor.get("rl");  //0
        frontRightMotor = hardwareMap.dcMotor.get("fr");    //3
        backRightMotor = hardwareMap.dcMotor.get("rr"); //2
        DcMotor turretMotor = hardwareMap.dcMotor.get("turretMotor"); // ext 1
        DcMotor shootMotor = hardwareMap.dcMotor.get("shootMotor"); // ext 0
        DcMotor intakeMotor = hardwareMap.dcMotor.get("intakeMotor"); // ext 3
        Servo lift = hardwareMap.get(Servo.class, "lift"); // ext 0 servo
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        robot.odo.update();
        intakeMotor.setPower(-1);
        DriveToTarget(makeTarget(660,0,0), 0.6, 0.5, 0.7, 1, 3);
        DriveToTarget(makeTarget(660,0,90), 0.6, 0.5, 0.7, 1, 2);
//        DriveToTarget(makeTarget(660,210,90), 0.6, 0.5, 0.7, 1, 3);
        DriveToTarget(makeTarget(660,390,90), 0.4, 0.5, 0.7, 1, 3);
        DriveToTarget(makeTarget(660,0,90), 0.6, 0.5, 0.7, 1, 3);
        DriveToTarget(makeTarget(660,0,0), 0.6, 0.5, 0.7, 1, 2);
        DriveToTarget(makeTarget(20,0,0), 0.5, 0.5, 0.7, 1, 3);
        shootMotor.setPower(0.85);

//        DriveToTarget(makeTarget(600,0,90), 0.8, 0.5, 0.7, 1, 5);

         // Shoot from starting postion for blue
//        lift.setPosition(0);
//        shootMotor.setPower(-0.8);     // from far
//        turretMotor.setPower(0);
//        Thread.sleep(3500);
//
//        lift.setPosition(0.6);
//        Thread.sleep(500);
//        lift.setPosition(0);
//        Thread.sleep(500);
//
//        intakeMotor.setPower(-1);
//        Thread.sleep(500);
//        intakeMotor.setPower(0);
//
//        lift.setPosition(0.6);
//        Thread.sleep(500);
//        lift.setPosition(0);
//
//        Thread.sleep(1000);
    }
}
