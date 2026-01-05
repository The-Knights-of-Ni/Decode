package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Subsystems.Drive.DriveToPoint;
import org.firstinspires.ftc.teamcode.Util.AllianceColor;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import java.util.HashMap;
import java.util.List;

//@TeleOp(name = "TeleOp2")
public class Teleop2 extends LinearOpMode {
    private Robot robot;
    private DcMotor frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor, shootMotor, turretMotor;
    private double shootMotorPower = 0.5; // Default shooting power

    private void initOpMode() {
        HashMap<String, Boolean> flags = new HashMap<>();
        flags.put("web", true);
        flags.put("vision", true);
        DriveToPoint nav = new DriveToPoint(this);
        robot = new Robot(hardwareMap, telemetry, new ElapsedTime(), nav, AllianceColor.BLUE, gamepad1, gamepad2, flags);
        frontLeftMotor = hardwareMap.dcMotor.get("fl");
        backLeftMotor = hardwareMap.dcMotor.get("rl");
        frontRightMotor = hardwareMap.dcMotor.get("fr");
        backRightMotor = hardwareMap.dcMotor.get("rr");
        shootMotor = hardwareMap.dcMotor.get("shootMotor");
        turretMotor = hardwareMap.dcMotor.get("turretMotor");
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initOpMode();
        List<LynxModule> allHubs = robot.hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
        telemetry.log().add("TeleOp2 Initialized, ready to start");
        waitForStart();
        while (opModeIsActive()) {
            for (LynxModule hub : allHubs) hub.clearBulkCache();
            // Drive Control (Mecanum)
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x * 1.1;
            double rx = gamepad1.right_stick_x;
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;
            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);
            // Limelight Vision: auto-aim and distance-based shooting
            robot.limelight.loop();
            double distToTarget = 0.0;
            if (robot.limelight.detectBlue) {
                distToTarget = robot.limelight.getDistanceFromTags(robot.limelight.blueGoal);
                double turretAngle = robot.limelight.blueGoal.getTargetXDegrees();
                // Adjust turret based on vision data
                turretMotor.setPower(autoAimSpeed(turretAngle, 7));
                // Set shooting power based on distance
                shootMotorPower = 0.002 * distToTarget + 0.45;
            } else {
                turretMotor.setPower(0);
            }
            // Shoot when Y is pressed
            if (gamepad1.y) {
                shootMotor.setPower(shootMotorPower);
            } else if (gamepad1.x) {
                shootMotor.setPower(0);
            }
            telemetry.addData("ShootMotor Power", shootMotorPower);
            telemetry.addData("Distance to Target", distToTarget);
            telemetry.update();
        }
    }
    // Sigmoid and auto-aim helpers
    public double sigmoid(double x, double k) {
        return 1 / (1 + Math.exp(-k * x));
    }
    public double autoAimSpeed(double dist, double k) {
        double magnitude = 0.5, tolerance = 2.5;
        return (-magnitude * sigmoid(dist - tolerance, k) + magnitude - magnitude * sigmoid(dist + tolerance, k));
    }
}
