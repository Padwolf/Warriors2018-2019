package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;

@TeleOp(name="Vuforia Navigation - Simple")
public class VuNavSimple extends OpMode {
    Navigation nav = new Navigation();
    @Override
    public void init() {
        nav.initialize(hardwareMap);
    }

    @Override
    public void start() {
        nav.beginTracking();
    }

    @Override
    public void loop() {
        nav.update();
        if (!nav.visibleImages.isEmpty()) {
            telemetry.addData("Translation", "x: %f, y: %f, z: %f", nav.getX(), nav.getY(), nav.getZ());
            telemetry.addData("Rotation", "Heading: %f, Pitch: %f, Roll: %f", nav.getHeading(), nav.getPitch(), nav.getRoll());
            for (VuforiaTrackable trackable : nav.visibleImages) {
                telemetry.addLine(trackable.getName());
            }
        } else {
            telemetry.addLine("No target visible");
        }
    }
}
