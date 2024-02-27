/* Copyright (c) 2021 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


/*
 * This file contains an example of a Linear "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode is executed.
 *
 * This particular OpMode illustrates driving a 4-motor Omni-Directional (or Holonomic) robot.
 * This code will work with either a Mecanum-Drive or an X-Drive train.
 * Both of these drives are illustrated at https://gm0.org/en/latest/docs/robot-design/drivetrains/holonomic.html
 * Note that a Mecanum drive must display an X roller-pattern when viewed from above.
 *
 * Also note that it is critical to set the correct rotation direction for each motor.  See details below.
 *
 * Holonomic drives provide the ability for the robot to move in three axes (directions) simultaneously.
 * Each motion axis is controlled by one Joystick axis.
 *
 * 1) Axial:    Driving forward and backward               Left-joystick Forward/Backward
 * 2) Lateral:  Strafing right and left                     Left-joystick Right and Left
 * 3) Yaw:      Rotating Clockwise and counter clockwise    Right-joystick Right and Left
 *
 * This code is written assuming that the right-side motors need to be reversed for the robot to drive forward.
 * When you first test your robot, if it moves backward when you push the left stick forward, then you must flip
 * the direction of all 4 motors (see code below).
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name="Vroom Vroom", group="Linear OpMode")

public class VroomVroom extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private static ElapsedTime panCd;
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor bigArm = null;
    private DcMotorEx ramp = null;
    private DcMotor spinPixel = null;
    private DcMotorEx hang = null;
    // private Servo panUD = null;
    private CRServo panUD = null;
    private CRServo panUD2 = null;
    private CRServo planeOpen = null;
    private Servo pixelDrop = null;
    @Override
    public void runOpMode() {

        panCd = new ElapsedTime();
        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
        bigArm = hardwareMap.get(DcMotor.class, "big_arm");
        ramp = hardwareMap.get(DcMotorEx.class, "ramp");
        spinPixel = hardwareMap.get(DcMotorEx.class, "spin");
        hang = hardwareMap.get(DcMotorEx.class, "hanging");
        // panUD = hardwareMap.get(Servo.class, "pan");
        panUD = hardwareMap.get(CRServo.class, "pan");
        panUD2 = hardwareMap.get(CRServo.class, "pan2");
        planeOpen = hardwareMap.get(CRServo.class, "plane");
        pixelDrop = hardwareMap.get(Servo.class, "p3Drop");

        // ########################################################################################
        // !!!            IMPORTANT Drive Information. Test your motor directions.            !!!!!
        // ########################################################################################
        // Most robots need the motors on one side to be reversed to drive forward.
        // The motor reversals shown here are for a "direct drive" robot (the wheels turn the same direction as the motor shaft)
        // If your robot has additional gear reductions or uses a right-angled drive, it's important to ensure
        // that your motors are turning in the correct direction.  So, start out with the reversals here, BUT
        // when you first test your robot, push the left joystick forward and observe the direction the wheels turn.
        // Reverse the direction (flip FORWARD <-> REVERSE ) of any wheel that runs backward
        // Keep testing until ALL the wheels move the robot forward when you push the left joystick forward.
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
        bigArm.setDirection(DcMotor.Direction.FORWARD);
        ramp.setDirection(DcMotorEx.Direction.FORWARD);
        spinPixel.setDirection(DcMotorEx.Direction.REVERSE);
        hang.setDirection(DcMotorEx.Direction.FORWARD);
        // panUD.setDirection(Servo.Direction.REVERSE);
        panUD.setDirection(CRServo.Direction.REVERSE);
        panUD2.setDirection(CRServo.Direction.FORWARD);
        planeOpen.setDirection(CRServo.Direction.FORWARD);
        pixelDrop.setDirection(Servo.Direction.FORWARD);



        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();
        double pixelDropPos = 0;
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double max;

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial   = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral =  gamepad1.left_stick_x;
            double yaw     =  gamepad1.right_stick_x;
            double panServoPos = 0;
            double panServoPower= 0;
            double planePower;
            double spinPower = 0;




            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double leftFrontPower  = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower   = axial - lateral + yaw;
            double rightBackPower  = axial + lateral - yaw;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower  /= max;
                rightFrontPower /= max;
                leftBackPower   /= max;
                rightBackPower  /= max;
            }

            // Servo controlling power???
            // idk if this is the right way to do this but it wrks sooo
            // Pan up and down
            if(gamepad2.dpad_up) {
                panServoPower = 1;
            } if(gamepad2.dpad_down)
            {
                panServoPower = -1;
            }
            // Uncomment for servo mode
          /*  if(gamepad2.dpad_up) {
                if (panCd.seconds() < 5) {
                    return;
                }
                panCd.reset();
                panServoPos = 1.0;
            }
            if (gamepad2.dpad_down) {
                if(panCd.seconds() < 5) {
                    return;
                }
                panCd.reset();
                panServoPos = 0;
            }

           */
            // Plane
            if(gamepad1.right_bumper && gamepad1.left_bumper) {
                planePower = -1;
            } else {
                planePower = 0;
            }

            // Ramp
            if(gamepad2.right_trigger > 0.3){
                spinPower = 0.8;
            } else if(gamepad2.left_trigger > 0.3){
                spinPower = -0.8;
            } else if(gamepad2.left_trigger < 0.3 || gamepad2.right_trigger < 0.3) {
                spinPower = 0.0;
            }

            // Pixel Drop
            if(gamepad1.a) {
                pixelDropPos = 0;
            } else if (gamepad1.y) {
                pixelDropPos = 1;
            }

            // This is test code:
            //
            // Uncomment the following code to test your motor directions.
            // Each button should make the corresponding motor run FORWARD.
            //   1) First get all the motors to take to correct positions on the robot
            //      by adjusting your Robot Configuration if necessary.
            //   2) Then make sure they run in the correct direction by modifying the
            //      the setDirection() calls above.
            // Once the correct motors move in the correct direction re-comment this code.

            /*
            leftFrontPower  = gamepad1.x ? 1.0 : 0.0;  // X gamepad
            leftBackPower   = gamepad1.a ? 1.0 : 0.0;  // A gamepad
            rightFrontPower = gamepad1.y ? 1.0 : 0.0;  // Y gamepad
            rightBackPower  = gamepad1.b ? 1.0 : 0.0;  // B gamepad
            */

            // Send calculated power to wheels
            leftFrontDrive.setPower(leftFrontPower * 0.65);
            rightFrontDrive.setPower(rightFrontPower * 0.65);
            leftBackDrive.setPower(leftBackPower * 0.65);
            rightBackDrive.setPower(rightBackPower * 0.65);
            bigArm.setPower(gamepad2.y ? 1.0 : 0);
            bigArm.setPower(gamepad2.a ? -0.7 : 0);
            ramp.setPower(gamepad2.b ? 1.0 : 0);
            ramp.setPower(gamepad2.x ? -1.0 : 0);
            spinPixel.setPower(spinPower);
            hang.setPower(gamepad1.dpad_up ? 1.0 : 0);
            hang.setPower(gamepad1.dpad_down ? -1.0 : 0);
            // panUD.setPosition(panServoPos);
            panUD.setPower(panServoPower);
            panUD2.setPower(panServoPower);
            planeOpen.setPower(planePower);
            pixelDrop.setPosition(pixelDropPos);





            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.addData("Pan Up/Down","%1f", panServoPos);
            telemetry.addData("Right Trigger", "%1f", gamepad2.right_trigger);
            telemetry.addData("Pixel Drop Pos", "%1f", pixelDropPos);

            telemetry.update();
        }
    }}
