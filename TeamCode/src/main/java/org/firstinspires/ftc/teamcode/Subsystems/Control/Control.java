package org.firstinspires.ftc.teamcode.Subsystems.Control;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystems.Subsystem;
//import org.firstinspires.ftc.teamcode.Util.ServoEx;


/**
 * Control subsystem for controlling arms and claws
 */
public class Control extends Subsystem {
    public final Servo shootFlap;
    public final DcMotorEx shootMotor;
    public final DcMotorEx intakeMotor;

    public Control(Telemetry telemetry, Servo shootFlap, DcMotorEx intakeMotor, DcMotorEx shootMotor) {
//
//    public Control(Telemetry telemetry, Servo shootFlap, DcMotorEx intakeMotor, DcMotorEx shootMotor) {
        super(telemetry, "control");
        //Initializing instance variables
        this.shootFlap = shootFlap;
        this.shootMotor = shootMotor;
        this.intakeMotor = intakeMotor;
    }

    /**
     * Gets all defaults, directions,etc. ready for the autonomous period
     */
    public void initDevicesAuto() {
        shootMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shootMotor.setDirection(DcMotor.Direction.FORWARD);
        shootMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setDirection(DcMotor.Direction.FORWARD);
        shootFlap.setDirection(Servo.Direction.FORWARD);
    }

//    /*public void initDevicesAuto() {
//
//    }*/

    /**
     * Gets all defaults, directions,etc. ready for the teleop period
     */
    public void initDevicesTeleop() {
        shootMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shootMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }


    /**
     * Begins the process of opening the claw.
     * Does not wait for the claw action to finish opening before terminating the method and
     * allowing other functions to begin.
     */
    public void midFlap() {
        shootFlap.setPosition(0.25);
    }

    /**
     * Begins the process of closing the claw.
     * Does not wait for the claw action to finish opening before terminating the method and
     * allowing other functions to begin.
     */
    public void wideFlap() {
        shootFlap.setPosition(0.5);
    }

    public void startIntake() {
        intakeMotor.setMotorEnable();
        intakeMotor.setVelocity(-30);
    }

    public void stopIntake() {
        intakeMotor.setMotorDisable();
        intakeMotor.setVelocity(0);

    }

    public void startShoot() {
        shootMotor.setMotorEnable();
        shootMotor.setVelocity(-40);
    }


    public void stopShoot() {
        shootMotor.setMotorDisable();
        shootMotor.setVelocity(0);
    }

//    /**
//     * Opens the claw fully.
//     * The method will not terminate until the claw is fully open, meaning that only the action
//     * of the claw opening can be occurring at the given time.
//     */
//    public void openClawSync() {
//        claw.setPosition(0);
//        while (Math.abs(claw.getPosition() - 0) > 0.05) {
//            try {
//                Thread.sleep(20);
//            } catch (InterruptedException e) {
//                /*Thread.sleep() can throw an exception called an InterruptedException, which is
//                thrown if another thread interrupts the current running thread.
//                Java requires that this be caught.
//                To catch it, we simply throw a runtime exception, which means out code need not
//                worry about this exception any more.
//                For more information, see the Javadocs on Thread.sleep()
//                 */
//                throw new RuntimeException(e);
//            }
//        }
//    }
//
//    /**
//     * Closes the claw fully.
//     * The method will not terminate until the claw is fully closed, meaning that only the action
//     * of the claw closing can be occurring at the given time.
//     */
//    public void closeClawSync() {
//        claw.setPosition(1);
//        while (Math.abs(claw.getPosition() - 1) > 0.05) {
//            try {
//                Thread.sleep(20);
//            } catch (InterruptedException e) {
//                /*See note on InterruptedException above*/
//                throw new RuntimeException(e);
//            }
//        }
//    }

    /**
     * Begins the process of moving the pivot.
     * Does not wait for the pivot movement to finish before terminating the method an allowing
     * other functions to begin.
     * @param newPosition The position to move the pivot to
     */
    /*
    public void movePivot(PivotPosition newPosition) {
        pivot.setPower(-0.8);
        while (pivot.getCurrentPosition() < newPosition.pos) {
//            pivot1.setPower(-0.8);
//        pivot2.setPower(0.8);
//        while (pivot1.getCurrentPosition() < newPosition.pos) {
            try {
                Thread.sleep(10);
            } catch (InterruptedException e) {
                /*See note on InterruptedException above*/
    //      throw new RuntimeException(e);
}
// }
//   pivot.setPower(0);

//        pivot1.setPower(0);
//        pivot2.setPower(0);
// }

//    /**
//     * Moves the pivot fully.
//     * The method will not terminate until the pivot is fully moved, meaning that only the action
//     * of the pivot can be occurring at the given time.
//     * @param newPosition The position to move the pivot to
//     */
//    public void movePivotSync(PivotPosition newPosition) {
//        movePivot(newPosition);
//        while (pivot.isBusy()) {
////            while (pivot1.isBusy()) {
//            try {
//                Thread.sleep(20);
//            } catch (InterruptedException e) {
//                /*See note on InterruptedException above*/
//                throw new RuntimeException(e);
//            }
//        }
//    }

//
//    /*TODO: Determine what values UP and DOWN should be, and/or replace UP and DOWN with any other positions we need*/
//    /**
//     * An enum to keep track of the positions we need the slide to be in regularly.
//     */
//    public enum PivotPosition {
//        UP(2222),
//        DOWN(0);
//        public final int pos;
//
//        PivotPosition(int pos) {
//            this.pos = pos;
//        }
//    }
//    public void moveLinearSlide(LinearSlidePosition newPosition) {
//        linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        linearSlide.setTargetPosition(newPosition.pos);
//        linearSlide2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        linearSlide2.setTargetPosition(newPosition.pos);
//    }


//    public void moveLinearSlideSync(LinearSlidePosition newPosition) {
//        moveLinearSlide(newPosition);
//        while (Math.abs(linearSlide.getCurrentPosition() - newPosition.pos) > 25) {
//            try {
//                Thread.sleep(20);
//            } catch (InterruptedException e) {
//                /*See note on InterruptedException above*/
//                throw new RuntimeException(e);
//            }
//        }
//    }


//    public void moveArm(LinearSlidePosition slidePosition, PivotPosition pivotPosition) {
//        moveLinearSlide(slidePosition);
//        movePivot(pivotPosition);
//    }