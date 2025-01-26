package org.firstinspires.ftc.teamcode.ftc7083.subsystem.processors;

import com.qualcomm.robotcore.util.Range;
import org.opencv.core.Rect;

    /**
     * An {@link ImageRegionEx} defines an area of an image buffer in terms of either a typical
     * image processing coordinate system wherein the origin is in the top left corner and
     * the domain of X and Y is dictated by the resolution of said image buffer; OR a "unity center"
     * coordinate system wherein the origin is at the middle of the image and the domain of
     * X and Y is {-1, 1} such that the region can be defined independent of the actual resolution
     * of the image buffer.
     */
    public class ImageRegionEx
    {
        final boolean imageCoords;
        final double left, top, right, bottom;

        /**
         * Internal constructor
         * @param imageCoords whether these coordinates are typical image processing coordinates
         * @param left left coordinate
         * @param top  top coordinate
         * @param right right coordiante
         * @param bottom bottom coordinate
         */
        private ImageRegionEx(boolean imageCoords, double left, double top, double right, double bottom )
        {
            this.left = left;
            this.top = top;
            this.right = right;
            this.bottom = bottom;
            this.imageCoords = imageCoords;
        }

        /**
         * Construct an {@link ImageRegionEx} using typical image processing coordinates
         *
         *  --------------------------------------------
         *  | (0,0)-------X                            |
         *  |  |                                       |
         *  |  |                                       |
         *  |  Y                                       |
         *  |                                          |
         *  |                           (width,height) |
         *  --------------------------------------------
         *
         * @param left left X coordinate {0, width}
         * @param top top Y coordinate {0, height}
         * @param right right X coordinate {0, width}
         * @param bottom bottom Y coordinate {0, height}
         * @return an {@link ImageRegionEx} object describing the region
         */
        public static ImageRegionEx asImageCoordinates(int left, int top, int right, int bottom )
        {
            return new ImageRegionEx(true, left, top, right, bottom);
        }

        /**
         * Construct an {@link ImageRegionEx} using "Unity Center" coordinates
         *
         *  --------------------------------------------
         *  | (-1,1)             Y               (1,1) |
         *  |                    |                     |
         *  |                    |                     |
         *  |                  (0,0) ----- X           |
         *  |                                          |
         *  | (-1,-1)                          (1, -1) |
         *  --------------------------------------------
         *
         * @param left left X coordinate {-1, 1}
         * @param top top Y coordinate {-1, 1}
         * @param right right X coordinate {-1, 1}
         * @param bottom bottom Y coordinate {-1, 1}
         * @return an {@link ImageRegionEx} object describing the region
         */
        public static ImageRegionEx asUnityCenterCoordinates(double left, double top, double right, double bottom)
        {
            return new ImageRegionEx(false, left, top, right, bottom);
        }

        /**
         * Construct an {@link ImageRegionEx} representing the entire frame
         * @return an {@link ImageRegionEx} representing the entire frame
         */
        public static ImageRegionEx entireFrame()
        {
            return ImageRegionEx.asUnityCenterCoordinates(-1, 1, 1, -1);
        }

        /**
         * Create an OpenCV Rect object which is representative of this {@link ImageRegionEx}
         * for a specific image buffer size
         *
         * @param imageWidth width of the image buffer
         * @param imageHeight height of the image buffer
         * @return OpenCV Rect
         */
        protected Rect asOpenCvRect(int imageWidth, int imageHeight)
        {
            Rect rect = new Rect();

            if (imageCoords)
            {
                rect.x = (int) left;
                rect.y = (int) top;
                rect.width = (int) (right - left);
                rect.height = (int) (bottom - top);
            }
            else // unity center
            {
                rect.x = (int) Range.scale(left, -1, 1, 0, imageWidth);
                rect.y = (int) ( imageHeight - Range.scale(top, -1, 1, 0, imageHeight));
                rect.width = (int) Range.scale(right - left, 0, 2, 0, imageWidth);
                rect.height = (int) Range.scale(top - bottom, 0, 2, 0, imageHeight);
            }

            // Adjust the window position to ensure it stays on the screen.  push it back into the screen area.
            // We could just crop it instead, but then it may completely miss the screen.
            rect.x = Math.max(rect.x, 0);
            rect.x = Math.min(rect.x, imageWidth - rect.width);
            rect.y = Math.max(rect.y, 0);
            rect.y = Math.min(rect.y, imageHeight - rect.height);

            return rect;
        }
    }
