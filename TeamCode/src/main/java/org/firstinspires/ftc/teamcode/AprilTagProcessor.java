package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import java.util.List;

public class AprilTagProcessor extends OpMode {
    private org.firstinspires.ftc.vision.apriltag.AprilTagProcessor aprilTagProcessor;
    private VisionPortal visionPortal;
    private Johnny6 johnny6;
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void init(){
        WebcamName webcamName=hardwareMap.get(WebcamName.class,"Trebuchet eye");
        aprilTagProcessor= org.firstinspires.ftc.vision.apriltag.AprilTagProcessor.easyCreateWithDefaults();
        visionPortal=VisionPortal.easyCreateWithDefaults(webcamName,aprilTagProcessor);
    }
    @Override
    public void init_loop(){
        List<AprilTagDetection> currentDetection=aprilTagProcessor.getDetections();
        StringBuilder idsFound=new StringBuilder();
        for(AprilTagDetection detection:currentDetection){
            idsFound.append(detection.id);
            idsFound.append(' ');
        }
        telemetry.addData("April Tags: ",idsFound);
    }
    @Override
    public void start(){
        visionPortal.stopStreaming();
    }
    public void loop(){

    }
}
