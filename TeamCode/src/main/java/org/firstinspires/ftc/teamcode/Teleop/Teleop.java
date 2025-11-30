package org.firstinspires.ftc.teamcode.Teleop;


import android.os.Build;


import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystems.Control.*;
import org.firstinspires.ftc.teamcode.Subsystems.Drive.MotorGeneric;
import org.firstinspires.ftc.teamcode.Util.AllianceColor;
import org.firstinspires.ftc.teamcode.Robot;

import java.util.HashMap;
import java.util.List;


@TeleOp(name = "TeleOp")
public class Teleop extends LinearOpMode {
    double deltaT;
    double timeCurrent;
    double timePre;
    ElapsedTime timer;
    private Robot robot;


    private void initOpMode() {
        // Initialize DC motor objects
        timer = new ElapsedTime();
        HashMap<String, Boolean> flags = new HashMap<>();
        flags.put("web", true);
        flags.put("vision", false);
        this.robot = new Robot(hardwareMap, telemetry, timer, AllianceColor.BLUE, gamepad1, gamepad2, flags);
        timeCurrent = timer.nanoseconds();
        timePre = timeCurrent;


        telemetry.addData("Waiting for start", "...");
        telemetry.update();
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
//        robot.control.initDevicesTeleop();
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);
        telemetry.log().add("Initialized, ready to start");
        telemetry.update();
        waitForStart();

        telemetry.clearAll();
        timeCurrent = timer.nanoseconds();
        timePre = timeCurrent;

        final double sensitivityHighPower = 1.0; // multiply inputs with this on high power mode
        final double sensitivityLowPower = 0.5; // multiply inputs with this on non-high power mode
        boolean twoGamepads = true;
        boolean intakeOn = false;
        boolean flapOpen = false;

        while (opModeIsActive()) {
            // Clears cache to refresh data
            for (LynxModule hub : allHubs) {
                hub.clearBulkCache();
            }

            // update data from gamepads
            robot.updateGamepads();

            // Get current time and compute delta
            timeCurrent = timer.nanoseconds();
            deltaT = timeCurrent - timePre;
            timePre = timeCurrent;

            // gets the motor powers for drive from gamepad1
            // y button activates low speed mode
            // it gets the x and y positioning from the left stick and turns based on the right stick's x
            // calcMotorPowers creates a MotorGeneric
            MotorGeneric<Double> motorPowers;
            if (Robot.gamepad1.yButton.toggle) {
                motorPowers = robot.drive.calcMotorPowers(Robot.gamepad1.leftStickX * sensitivityHighPower, Robot.gamepad1.leftStickY * sensitivityHighPower, Robot.gamepad1.rightStickX * sensitivityHighPower);
            } else {
                motorPowers = robot.drive.calcMotorPowers(Robot.gamepad1.leftStickX * sensitivityLowPower, Robot.gamepad1.leftStickY * sensitivityLowPower, Robot.gamepad1.rightStickX * sensitivityLowPower);
            }
            robot.drive.setDrivePowers(motorPowers);
            // Switch to one gamepad
            if (Robot.gamepad1.xButton.isPressed() || Robot.gamepad2.xButton.isPressed()) {
                twoGamepads = !twoGamepads;
                telemetry.log().add("Switching to " + (twoGamepads ? "two" : "one") + " gamepad mode");
            }
            if (twoGamepads) {
                // Starting the shooting motor (Trigger Left)
                if (Robot.gamepad2.triggerLeft > 0.05) {
                    robot.control.startShoot();
                    telemetry.log().add("Starting the shoot motor");
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

                // Opening and closing the shooting flap
                if (Robot.gamepad2.xButton.isPressed()) {
                    if (flapOpen) {
                        robot.control.midFlap();
                        telemetry.log().add("Stopping the flap");
                    } else {
                        robot.control.wideFlap();
                        telemetry.log().add("Starting the flap");
                    }
                    flapOpen = !flapOpen;
                }

                if (Robot.gamepad1.aButton.isPressed()) {
                    robot.control.shootMotor.setPower(-0.65);
//                    robot.control.holdShootVelocity(-5);
                    telemetry.log().add("This is -0.65");
                }
                if (Robot.gamepad1.bButton.isPressed()) {
                    robot.control.shootMotor.setPower(-0.75);
//                    robot.control.holdShootVelocity(-7.5);
                    telemetry.log().add("This is -0.75");
                }
                if (Robot.gamepad1.xButton.isPressed()) {
                    robot.control.shootMotor.setPower(-0.85);
//                    robot.control.holdShootVelocity(-10);
                    telemetry.log().add("This is -0.85");
                }
                if (Robot.gamepad1.yButton.isPressed()) {
                    robot.control.shootMotor.setPower(-0.95);
//                    robot.control.holdShootVelocity(-10);
                    telemetry.log().add("This is -0.95");
                }
                // We need to add incrementing button later

            } else {
                // TODO: single gamepad controls
            }

            telemetry.update();
        }
    }
}