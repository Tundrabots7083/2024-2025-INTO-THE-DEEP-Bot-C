package org.firstinspires.ftc.teamcode.ftc7083.subsystem.processors;


import org.firstinspires.ftc.vision.opencv.ColorSpace;
import org.opencv.core.Scalar;

    /**
     * An {@link org.firstinspires.ftc.vision.opencv.ColorRange represents a 3-channel minimum/maximum
     * range for a given color space}
     */
    public class ColorRangeEx
    {
        protected final ColorSpace colorSpace;
        protected final Scalar min;
        protected final Scalar max;

        // -----------------------------------------------------------------------------
        // DEFAULT OPTIONS
        // -----------------------------------------------------------------------------

        public static final org.firstinspires.ftc.vision.opencv.ColorRange BLUE = new org.firstinspires.ftc.vision.opencv.ColorRange(
                ColorSpace.YCrCb,
                new Scalar( 16,   0, 155),
                new Scalar(255, 127, 255)
        );

        public static final org.firstinspires.ftc.vision.opencv.ColorRange RED = new org.firstinspires.ftc.vision.opencv.ColorRange(
                ColorSpace.YCrCb,
                new Scalar( 32, 176,  0),
                new Scalar(255, 255, 132)
        );

        public static final org.firstinspires.ftc.vision.opencv.ColorRange YELLOW = new org.firstinspires.ftc.vision.opencv.ColorRange(
                ColorSpace.YCrCb,
                new Scalar( 32, 128,   0),
                new Scalar(255, 170, 120)
        );

        public static final org.firstinspires.ftc.vision.opencv.ColorRange GREEN = new org.firstinspires.ftc.vision.opencv.ColorRange(
                ColorSpace.YCrCb,
                new Scalar( 32,   0,   0),
                new Scalar(255, 120, 133)
        );

        // -----------------------------------------------------------------------------
        // ROLL YOUR OWN
        // -----------------------------------------------------------------------------

        public ColorRangeEx(ColorSpace colorSpace, Scalar min, Scalar max)
        {
            this.colorSpace = colorSpace;
            this.min = min;
            this.max = max;
        }
    }

