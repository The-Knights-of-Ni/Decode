package org.firstinspires.ftc.teamcode.Auto;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "Auto2025")
public class Auto2025 extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("fl"); //1 port
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("rl");  //0
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("fr");    //3
        DcMotor backRightMotor = hardwareMap.dcMotor.get("rr"); //2
        DcMotor turretMotor = hardwareMap.dcMotor.get("turretMotor"); // ext 1
        DcMotor shootMotor = hardwareMap.dcMotor.get("shootMotor"); // ext 0
        DcMotor intakeMotor = hardwareMap.dcMotor.get("intakeMotor"); // ext 3
        Servo lift = hardwareMap.get(Servo.class, "lift"); // ext 0 servo
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        intakeMotor.setPower(-0.1);
        shootMotor.setPower(-0.9);     // from far
        turretMotor.setPower(0);
        Thread.sleep(5000);

        lift.setPosition(0.6);
        Thread.sleep(2000);
        lift.setPosition(0);

        Thread.sleep(2000);

        lift.setPosition(0.6);
        Thread.sleep(2000);
        lift.setPosition(0);
    }
}
