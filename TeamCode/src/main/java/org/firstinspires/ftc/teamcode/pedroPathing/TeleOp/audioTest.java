package org.firstinspires.ftc.teamcode.pedroPathing.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.ftccommon.SoundPlayer;

@TeleOp(name = "Play Sound Example", group = "Examples")
public class audioTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        // Get the ID for your kloufrens.mp3 file from res/raw
        int soundID = hardwareMap.appContext.getResources()
                .getIdentifier("kloufrens", "raw", hardwareMap.appContext.getPackageName());

        // Check that the sound file exists
        if (soundID == 0) {
            telemetry.addLine("❌ Sound file not found! Make sure kloufrens.mp3 is in res/raw/");
            telemetry.update();
        } else {
            telemetry.addLine("✅ Sound ready! Press A to play it.");
            telemetry.update();
        }

        waitForStart();

        while (opModeIsActive()) {
            // When the 'A' button is pressed, play the sound
            if (gamepad1.cross) {
                SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, soundID);
                telemetry.addLine("🎵 Playing kloufrens.mp3");
                telemetry.update();
                sleep(2000); // small delay to avoid replaying too fast
            }
        }
    }
}
