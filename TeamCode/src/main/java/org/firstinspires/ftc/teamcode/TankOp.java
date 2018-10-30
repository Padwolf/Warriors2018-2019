package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="TankDrive")
public class TankOp extends OpMode {
    HardwareInterface hardware = new HardwareInterface(hardwareMap);
    double speedR = 1.0;
    double speedL = 1.0;
    double speedArm = 0.0001;
    @Override
    public void init() {



    }

    @Override
    public void loop() {
        if( gamepad1.left_stick_button) {
            speedL = 0.5;

        }
        else {
            speedL = 1.00;
        }
        if( gamepad1.right_stick_button)  {
                speedR = 0.5;

            } else{
                speedR = 1.00;

        }

        hardware.leftDrive.setPower(-Range.clip(-gamepad1.left_stick_y, -1.0, 1.0)*speedL);
        hardware.rightDrive.setPower(-Range.clip(gamepad1.right_stick_y, -1.0, 1.0)*speedR);
        hardware.armY.setPower((gamepad1.right_trigger - gamepad1.left_trigger)*speedArm); //These triggers contradict each other
        //hardware.collector.setPower(gamepad1.left_stick_x );


    }
}