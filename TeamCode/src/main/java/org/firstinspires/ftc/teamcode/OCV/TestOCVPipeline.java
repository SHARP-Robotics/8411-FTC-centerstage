package org.firstinspires.ftc.teamcode.OCV;

import org.openftc.easyopencv.OpenCvPipeline;
import org.opencv.core.Mat;

class TestOCVPipeline extends OpenCvPipeline {
    @Override
    public Mat processFrame(Mat input){
        return input;
    }
}
