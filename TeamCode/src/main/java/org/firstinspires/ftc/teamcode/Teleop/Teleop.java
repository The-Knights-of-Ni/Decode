package org.firstinspires.ftc.teamcode.Teleop;


import android.os.Build;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Subsystems.Vision.AprilTagLimelightTest;


import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Subsystems.Drive.DriveToPoint;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystems.Control.*;
import org.firstinspires.ftc.teamcode.Subsystems.Drive.MotorGeneric;
import org.firstinspires.ftc.teamcode.Util.AllianceColor;
import org.firstinspires.ftc.teamcode.Robot;

import java.util.HashMap;
import java.util.List;
import java.util.Locale;


@TeleOp(name = "TeleOp")
public class Teleop extends LinearOpMode {
    double deltaT;
    double timeCurrent;
    double timePre;
    ElapsedTime timer;
    private Robot robot;
    DcMotor frontLeftMotor;
    DcMotor backLeftMotor;
    DcMotor frontRightMotor;
    DcMotor backRightMotor;
    double ShootMotorPower = 0.0;


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
//        telemetry.update();
    }


    public double sigmoid(double x, double k){
        return 1/(1+Math.exp(-k*x));
    }

    public double autoAimSpeed(double dist, double k){
        double magnitude = 0.5, tolerance = 2.5;
        return (-magnitude*sigmoid(dist-tolerance,k) +
                magnitude-magnitude*sigmoid(dist+tolerance,k));
    }

    Pose2D makeTarget(double xpos, double ypos, double hpos){
        return new Pose2D(DistanceUnit.MM,xpos,ypos, AngleUnit.DEGREES,hpos);
    }

    void DriveToTarget(Pose2D TARGET, double power, double holdTime, double powerMultiplier, double patience, double timeOut){
        double startTime = getRuntime();
        while(opModeIsActive()){
            robot.odo.update();
            //        telemetry.addData("Reached target", nav.driveTo(odo.getPosition(), TARGET_1, 0.7, 0));
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

//            telemetry.update();
        }
    }

    public void adjustAngle(double dist) {
        robot.odo.update();
        Pose2D pos = robot.odo.getPosition();
        Pose2D TARGET = makeTarget(pos.getX(DistanceUnit.MM),pos.getY(DistanceUnit.MM),pos.getHeading(AngleUnit.DEGREES)-dist);
        DriveToTarget(TARGET, 0.8, 0.5, 0.7, 1, 5);
    }

    /**
     * Override of runOpMode()
     *
     * <p>Please do not swallow the InterruptedException, as it is used in cases where the op mode
     * needs to be terminated early.
     *
     * @see LinearOpMode
     */
    @Override
    public void runOpMode() throws InterruptedException {
        initOpMode();

        // Activates bulk reading, a faster way of reading data
        // IMPORTANT: Caches have to be cleared every loop
        List<LynxModule> allHubs = robot.hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        ElapsedTime timer = new ElapsedTime();
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);
        telemetry.log().add("Initialized, ready to start");
//        telemetry.update();
        waitForStart();

        telemetry.clearAll();
        timeCurrent = timer.nanoseconds();
        timePre = timeCurrent;

        final double sensitivityHighPower = 1.0; // multiply inputs with this on high power mode
        final double sensitivityLowPower = 0.5; // multiply inputs with this on non-high power mode


        boolean twoGamepads = true;
        boolean intakeOn = false;
        boolean flapOpen = false;

        telemetry.addLine("dwbug 0");

        frontLeftMotor = hardwareMap.dcMotor.get("fl"); //1 port
        backLeftMotor = hardwareMap.dcMotor.get("rl");  //0
        frontRightMotor = hardwareMap.dcMotor.get("fr");    //3
        backRightMotor = hardwareMap.dcMotor.get("rr"); //2
        DcMotor turretMotor = hardwareMap.dcMotor.get("turretMotor"); // ext 1
        DcMotor shootMotor = hardwareMap.dcMotor.get("shootMotor"); // ext 0
        DcMotor intakeMotor = hardwareMap.dcMotor.get("intakeMotor"); // ext 3
//        Servo lift = hardwareMap.get(Servo.class, "lift"); // ext 0 servo

        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addLine("dwbug 1");

        /*
        * 0 = color not selected
        * 1 = red
        * 2 = blue
        * */
        int color = 0;

        telemetry.addLine("dwbug 2");

        while (opModeIsActive()) {
            // Clears cache to refresh data
            for (LynxModule hub : allHubs) {
                hub.clearBulkCache();
            }

            // update data from gamepads
            Robot.updateGamepads();

            telemetry.addLine("While loop exists :D");

            if(color == 0){
                if (twoGamepads){
                    if(Robot.gamepad1.bButton.isPressed() && Robot.gamepad2.bButton.isPressed()){
                        // red alliance
                        color = 1;
                    }
                    else if(Robot.gamepad1.xButton.isPressed() && Robot.gamepad2.xButton.isPressed()){
                        // blue alliance
                        color = 2;
                    }
                    else{
                        // v v    ideally this doesn't happen too much
                        telemetry.addLine("Alliance color not yet selected.");
                        continue;
                    }
                }
                else{
                    if(Robot.gamepad1.bButton.hasPressedPrev()){
                        // red alliance
                        color = 1;
                    }
                    else if(Robot.gamepad1.xButton.hasPressedPrev()){
                        // blue alliance
                        color = 2;
                    }
                    else{
                        // v v    ideally this doesn't happen too much
                        telemetry.addLine("Alliance color not yet selected.");
                        continue;
                    }
                }
            }

            if(color == 2){
                telemetry.addLine("Color is blue");
            }
            else if(color == 1){
                telemetry.addLine("Color is red");
            }
            telemetry.addData("Shootmotor power is ", ShootMotorPower);

            robot.limelight.loop();

            double distToTarget = 0.0;

            if(color == 2 && robot.limelight.detectBlue){
                distToTarget = robot.limelight.getDistanceFromTags( robot.limelight.blueGoal.getTargetArea() );
            }
            else if(color == 1 && robot.limelight.detectRed){
                distToTarget = robot.limelight.getDistanceFromTags( robot.limelight.redGoal.getTargetArea() );
            }

            telemetry.addLine("\n");

            telemetry.addData("Target Shootmotor power is ", (0.002*distToTarget+0.45));
            telemetry.addData("Distance is ", distToTarget);

            if(gamepad1.dpad_down){
                double degreeError = 0.0;

                if(color == 2 && robot.limelight.detectBlue){
                    degreeError = robot.limelight.blueGoal.getTargetXDegrees();
                }
                else if(color == 1 && robot.limelight.detectRed){
                    degreeError = robot.limelight.redGoal.getTargetXDegrees();
                }

                if(Math.abs(degreeError) > 15){
                    adjustAngle(degreeError);
                }
                else {
                    double aimSpeed = autoAimSpeed(degreeError , 7);
                    robot.control.turretMotor.setMotorEnable();
                    robot.control.turretMotor.setPower(aimSpeed);
                }
            }
            else{
                robot.control.turretMotor.setPower(0);
            }


            if(gamepad1.dpad_up){
                robot.odo.update();
                Pose2D pos = robot.odo.getPosition();
                telemetry.addLine("Trying to move in the x-direction");
                Pose2D TARGET = makeTarget(pos.getX(DistanceUnit.MM)+100,pos.getY(DistanceUnit.MM),pos.getHeading(AngleUnit.DEGREES));
                DriveToTarget(TARGET, 0.8, 0.5, 0.7, 1, 5);
            }

            if(Robot.gamepad1.dPadLeft.isPressed() && !Robot.gamepad1.dPadLeft.hasPressedPrev()){
                ShootMotorPower -= 0.01;
                telemetry.addData("Motor power is now",ShootMotorPower);
//                telemetry.update();
            }

            if(Robot.gamepad1.dPadRight.isPressed() && !Robot.gamepad1.dPadRight.hasPressedPrev()){
                ShootMotorPower += 0.01;
                telemetry.addData("Motor power is now",ShootMotorPower);
//                telemetry.update();
            }

            // Get current time and compute delta
            timeCurrent = timer.nanoseconds();
            deltaT = timeCurrent - timePre;
            timePre = timeCurrent;

            // gets the motor powers for drive from gamepad1
            // y button activates low speed mode
            // it gets the x and y positioning from the left stick and turns based on the right stick's x
            // calcMotorPowers creates a MotorGeneric
//            MotorGeneric<Double> motorPowers;
//            if (Robot.gamepad1.yButton.toggle) {
//                motorPowers = robot.drive.calcMotorPowers(Robot.gamepad1.leftStickX * sensitivityHighPower, Robot.gamepad1.leftStickY * sensitivityHighPower, Robot.gamepad1.rightStickX * sensitivityHighPower);
//            } else {
//                motorPowers = robot.drive.calcMotorPowers(Robot.gamepad1.leftStickX * sensitivityLowPower, Robot.gamepad1.leftStickY * sensitivityLowPower, Robot.gamepad1.rightStickX * sensitivityLowPower);
//            }
//            robot.drive.setDrivePowers(motorPowers);
            // Switch to one gamepad
//            if (Robot.gamepad1.xButton.isPressed() || Robot.gamepad2.xButton.isPressed()) {
//                twoGamepads = !twoGamepads;
//                telemetry.log().add("Switching to " + (twoGamepads ? "two" : "one") + " gamepad mode");
//            }

            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;


            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);

            if (gamepad1.a) {
                robot.control.startIntake();
            }
            if (gamepad1.b) {
                robot.control.stopIntake();
            }
            if (gamepad1.y) {
                robot.control.startShoot(ShootMotorPower);
                Thread.sleep(5000);
                robot.control.lift.setPosition(0.6);
            }
            if (gamepad1.x) {
                robot.control.stopShoot();
                robot.control.lift.setPosition(0);
                shootMotor.setPower(0);
            }

            // un-depreicate later
            if (twoGamepads && false) {
                // Why is shooting and intake on separate gamepads??


                // Starting the shooting motor (Trigger Left)
                if (Robot.gamepad2.triggerLeft > 0.05) {
                    robot.control.startShoot(ShootMotorPower);
                    // put limelight tests for teleop here for now?
                    telemetry.log().add("Starting the shoot motor");
                    robot.limelight.loop();
                }

                // Stopping the shooting motor (Bumper Left)
                if (Robot.gamepad2.bumperLeft.isPressed()) {
                    robot.control.stopShoot();
                    telemetry.log().add("Stopping the shoot motor");
                }

                // Starting and stopping the intake motor
                if (Robot.gamepad2.yButton.isPressed()) {
                    if (intakeOn){
                        robot.control.stopIntake();
                        telemetry.log().add("Stopping the intake");
                    }
                    else {
                        robot.control.startIntake();
                        telemetry.log().add("Starting the intake");
                    }
                    intakeOn = !intakeOn;
                }

                if (Robot.gamepad1.aButton.isPressed()) {
                    ShootMotorPower += 0.10;
                    telemetry.addData("Motor power reduced by 0.1 and is now ",ShootMotorPower);
                }
                if (Robot.gamepad1.bButton.isPressed()) {
                    ShootMotorPower -= 0.10;
                    telemetry.addData("Motor power increased by 0.1 and is now ",ShootMotorPower);
                }
                if (Robot.gamepad1.xButton.isPressed()) {
                    ShootMotorPower = -0.85;
                    telemetry.addData("Motor power is reset and is now ",ShootMotorPower);
                }
                if (Robot.gamepad1.yButton.isPressed()) {
                    robot.control.shootMotor.setPower(ShootMotorPower);
                    telemetry.log().add("Shooting motor with motor power", ShootMotorPower);
                }
                // We need to add incrementing button later
            } else {
                // TODO: single gamepad controls
            }

            telemetry.update();
        }
    }
}