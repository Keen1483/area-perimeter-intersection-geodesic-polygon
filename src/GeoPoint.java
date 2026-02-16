
/**
 * Class representing a geographic point
 */
public class GeoPoint {

    double latitude;   // in degrees
    double longitude;  // in degrees

    public GeoPoint(double lat, double lon) {
        this.latitude = lat;
        this.longitude = lon;
    }

    @Override
    public String toString() {
        return String.format("(%.6f°, %.6f°)", latitude, longitude);
    }
}
