package org.firstinspires.ftc.teamcode.ftc7083.subsystem.processors;

import org.opencv.core.Point;
import org.opencv.core.RotatedRect;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.TreeMap;

public class SampleProcessor {

    public Double execute(List<ColorBlobLocatorProcessorEx.Blob> blobs) {

        ColorBlobLocatorProcessorEx.Util.filterByArea(20000, 2300000, blobs);  // filter out very small blobs.
        ColorBlobLocatorProcessorEx.Util.filterByAspectRatio(2.0,2.66,blobs);

        Double angle = null;

        // Display the size (area) and center location for each Blob.
        for(ColorBlobLocatorProcessorEx.Blob b : blobs) {
            RotatedRect boxFit = b.getBoxFit();

            Point[] corners = new Point[4];
            boxFit.points(corners);

            angle = calculateOrientation(corners);
        }

        return angle;
    }

    public Double calculateOrientation(Point[] corners) {

        if(corners.length == 0){
            return null;
        }

        int n = corners.length;

        List<DistancePair> distances = new ArrayList<>();

        // Step 1: Compute all pairwise distances
        for (int i = 0; i < n; i++) {
            for (int j = i + 1; j < n; j++) {
                int distance = (int)Math.hypot((corners[i].x - corners[j].x),(corners[i].y - corners[j].y));
                distances.add(new DistancePair(distance, i, j));
            }
        }

        // Step 2: Group distances by length and count occurrences
        Map<Double, List<DistancePair>> groupedDistances = new TreeMap<>();
        for (DistancePair pair : distances) {
            groupedDistances.computeIfAbsent(pair.distance, k -> new ArrayList<>()).add(pair);
        }

        // Step 3: Identify the two most common distances (shorter and longer sides)
        List<Double> uniqueDistances = new ArrayList<>(groupedDistances.keySet());

        // Exclude the diagonal (largest distance)
        double shorterSideLength = uniqueDistances.get(0);
        double longerSideLength = uniqueDistances.get(1);

        // Step 4: Calculate orientation using the first pair from the longer side
        DistancePair longerSide = Objects.requireNonNull(groupedDistances.get(longerSideLength)).get(1);
        double x1 = corners[longerSide.point1].x;
        double y1 = corners[longerSide.point1].y;
        double x2 = corners[longerSide.point2].x;
        double y2 = corners[longerSide.point2].y;

        double angleRad = Math.atan2(y2 - y1, x2 - x1);
        double angleDeg = Math.toDegrees(angleRad);

        // Normalize the angle to [0, 360)
        return (angleDeg + 360) % 360;
    }
}

 class DistancePair {
    double distance;
    int point1;
    int point2;

    DistancePair(double distance, int point1, int point2) {
        this.distance = distance;
        this.point1 = point1;
        this.point2 = point2;
    }
 }

