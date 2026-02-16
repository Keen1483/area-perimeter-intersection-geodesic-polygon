import java.util.List;

/**
 * Classe pour représenter le résultat d'intersection
 */
public class IntersectionResult {
    boolean intersects;
    double intersectionArea;
    List<GeoPoint> intersectionPolygon;

    public IntersectionResult(boolean intersects, double area, List<GeoPoint> polygon) {
        this.intersects = intersects;
        this.intersectionArea = area;
        this.intersectionPolygon = polygon;
    }

    @Override
    public String toString() {
        if (!intersects) {
            return "Les polygones ne se touchent pas (aire = 0 m²)";
        }
        return String.format(
                "Intersection détectée\n" +
                        "Aire: %.2f m² (%.6f km²)\n" +
                        "Nombre de sommets: %d",
                intersectionArea,
                intersectionArea / 1_000_000,
                intersectionPolygon.size()
        );
    }
}
