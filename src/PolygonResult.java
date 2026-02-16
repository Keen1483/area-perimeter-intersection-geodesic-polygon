/**
 * Calculation result for a polygon
 */
public class PolygonResult {
    double area;       // in m²
    double perimeter;  // in m

    public PolygonResult(double area, double perimeter) {
        this.area = area;
        this.perimeter = perimeter;
    }

    @Override
    public String toString() {
        return String.format("Aire: %.2f m² (%.6f km²)\nPérimètre: %.2f m (%.3f km)",
                area, area / 1_000_000, perimeter, perimeter / 1000);
    }
}
