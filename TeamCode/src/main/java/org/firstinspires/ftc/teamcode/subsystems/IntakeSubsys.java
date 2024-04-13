
package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@Config
public class IntakeSubsys {
    final DcMotorEx intakeArmMotor;
    final Servo intakeClawServoLeft;
    final Servo intakeClawServoRight;

    final Servo intakePivotServo;

    // intake claw servo for different positions
    // this needs to be tuned with servo tester or using FtCDashboard
    //-----------------------------------------------------------------
    public static double INTAKE_CLAW_SERVO_LEFT_INIT = 0.3;
    public static double INTAKE_CLAW_SERVO_RIGHT_INIT = 0.3;

    public static double INTAKE_CLAW_SERVO_LEFT_OPEN = 0.4;
    public static double INTAKE_CLAW_SERVO_RIGHT_OPEN = 0.4;

    public static double INTAKE_CLAW_SERVO_LEFT_CLOSE = 0.5;
    public static double INTAKE_CLAW_SERVO_RIGHT_CLOSE = 0.8;

    public static double INTAKE_PIVOT_SERVO_GRAB = 0.66;
    public static double INTAKE_PIVOT_SERVO_FOLD = 0.24;
    public static double INTAKE_PIVOT_SERVO_SCORE = 0.68;
    public static double INTAKE_PIVOT_SERVO_LOW = 0.62;
    public static double INTAKE_PIVOT_SERVO_HANG = 1;

    // intake arm encoder value for different positions
    // this needs to be tuned with motor tester or using FtCDashboard
    //-----------------------------------------------------------------
    public static int INTAKE_ARM_INTAKE_POSITION = -287;
    public static int INTAKE_ARM_INTAKE_SCORE = -700;
    public static int INTAKE_ARM_INTAKE_SAFE_CROSS = 0;
    public static int INTAKE_ARM_INTAKE_LOW_SCORE = -450;
    public static int INTAKE_ARM_INTAKE_HANGING = -1200;

    public enum ClawState {
        CLOSED,
        LEFT_CLOSED,
        RIGHT_CLOSED,
        LEFT_OPENED,
        RIGHT_OPENED,
        OPENED
    }
    public ClawState clawState;

    public IntakeSubsys(HardwareMap hardwareMap) {
        intakeArmMotor  = hardwareMap.get(DcMotorEx.class, "big_arm");

        // change the direction accordingly, make sure the position encoder value is UP
        // ------------------------------------------------------------
        intakeArmMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        intakeArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        intakeClawServoLeft = hardwareMap.get(ServoImplEx.class, "clawServoL");
        intakeClawServoRight = hardwareMap.get(ServoImplEx.class, "clawServoR");
        intakePivotServo = hardwareMap.get(ServoImplEx.class, "pan");
    }

    public void initialize(boolean isAuto) {
        intakeClawServoLeft.setPosition(INTAKE_CLAW_SERVO_LEFT_INIT);
        intakeClawServoRight.setPosition(INTAKE_CLAW_SERVO_RIGHT_INIT);
    }

    /**
     * Open the claws based on the need
     *
     * @param clawState
    **/
    public void openClaws(ClawState clawState) {
        if(clawState == ClawState.LEFT_OPENED || clawState == ClawState.OPENED) {
            intakeClawServoLeft.setPosition(INTAKE_CLAW_SERVO_LEFT_OPEN);
        }

        if(clawState == ClawState.RIGHT_OPENED || clawState == ClawState.OPENED) {
            intakeClawServoRight.setPosition(INTAKE_CLAW_SERVO_RIGHT_OPEN);
        }

        this.clawState = clawState;
    }

    /**
     * Close the claws based on the need
     *
     * @param clawState
    **/
    public void closeClaws(ClawState clawState) {
        if(clawState == ClawState.LEFT_CLOSED || clawState == ClawState.CLOSED) {
            intakeClawServoLeft.setPosition(INTAKE_CLAW_SERVO_LEFT_CLOSE);
        }

        if(clawState == ClawState.RIGHT_CLOSED || clawState == ClawState.CLOSED) {
            intakeClawServoRight.setPosition(INTAKE_CLAW_SERVO_RIGHT_CLOSE);
        }

        this.clawState = clawState;
    }

    public void prepareToIntake() {

        // move the arm to the intake position
        intakeArmMotor.setTargetPosition(INTAKE_ARM_INTAKE_POSITION);
        intakeArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        intakeArmMotor.setPower(0.4);

        // move the pivot arm
        intakePivotServo.setPosition(INTAKE_PIVOT_SERVO_GRAB);

    }

    public void grabPixels() {
        openClaws(ClawState.CLOSED);
    }

    public void prepareToDrive() {
        // move the arm to the safe position
        intakeArmMotor.setTargetPosition(INTAKE_ARM_INTAKE_SAFE_CROSS);

        // Turn On RUN_TO_POSITION
        intakeArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        intakeArmMotor.setPower(0.6);

        // move the pivot arm
        intakePivotServo.setPosition(INTAKE_PIVOT_SERVO_FOLD);

        //claws close
        intakeClawServoLeft.setPosition(0.195);
        intakeClawServoRight.setPosition(0.747);


    }

    public void prepareToScore() {
        // move the arm to the safe position
        intakeArmMotor.setTargetPosition(INTAKE_ARM_INTAKE_SCORE);

        // Turn On RUN_TO_POSITION
        intakeArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        intakeArmMotor.setPower(0.9);

        // move the pivot arm
        intakePivotServo.setPosition(INTAKE_PIVOT_SERVO_SCORE);

        // open the claws
        openClaws(ClawState.CLOSED);
    }

    public void prepareToLowScore() {
        intakeArmMotor.setTargetPosition(INTAKE_ARM_INTAKE_LOW_SCORE);

        intakeArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        intakeArmMotor.setPower(0.9);

        intakePivotServo.setPosition(INTAKE_PIVOT_SERVO_LOW);
    }

    public void prepareToHang() {
        intakeArmMotor.setTargetPosition(INTAKE_ARM_INTAKE_HANGING);

        intakeArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        intakeArmMotor.setPower(0.9);

        intakePivotServo.setPosition(INTAKE_PIVOT_SERVO_HANG);
    }

}
