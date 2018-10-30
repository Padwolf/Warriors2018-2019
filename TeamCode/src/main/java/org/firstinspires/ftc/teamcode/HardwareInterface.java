package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class HardwareInterface {

    public DcMotor leftDrive;
    public DcMotor rightDrive;
    public DcMotor armY;
    //public CRServo collector;

    HardwareInterface(HardwareMap hardwareMap){

        leftDrive = hardwareMap.dcMotor.get("LeftDrive");
        rightDrive = hardwareMap.dcMotor.get("RightDrive");
        armY = hardwareMap.dcMotor.get("armUp");
        //collector = hardwareMap.crservo.get("Collector");

    }
}
