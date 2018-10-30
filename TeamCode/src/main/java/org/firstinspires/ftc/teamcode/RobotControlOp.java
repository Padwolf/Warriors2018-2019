package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.HardwareMap;


@TeleOp(name="ControllerRobot")

public class RobotControlOp extends OpMode{
        CRServo collector;
        DcMotor armY;
        DcMotor leftDrive;
        DcMotor rightDrive;
        double speedR = 1.0;
        double speedL = 1.0;
        double speedArm = 0.3;

        @Override
        public void init() {

            leftDrive = hardwareMap.dcMotor.get("LeftDrive");
            rightDrive = hardwareMap.dcMotor.get("RightDrive");
            armY = hardwareMap.dcMotor.get("armUp");
            collector = hardwareMap.crservo.get ("collector");
        }
        @Override
        public void loop() {
            if (gamepad1.left_stick_button) {
                speedL = 0.5;

            } else {
                speedL = 1.00;
            }
            if (gamepad1.right_stick_button) {
                speedR = 0.5;

            } else {
                speedR = 1.00;

            }

            leftDrive.setPower(-Range.clip(-gamepad1.left_stick_y, -1.0, 1.0) * speedL);
            rightDrive.setPower(-Range.clip(gamepad1.right_stick_y, -1.0, 1.0) * speedR);
            armY.setPower((gamepad1.right_trigger - gamepad1.left_trigger) * speedArm); //These triggers contradict each other
            collector.setPower(gamepad1.left_stick_x );
        }
    }