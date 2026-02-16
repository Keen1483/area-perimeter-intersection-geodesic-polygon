import java.util.List;
import java.util.stream.Collectors;

public class Main {
    public static void main(String[] args) {

        System.out.println("=== CALCULATEUR DE POLYGONE GÉODÉSIQUE ===");
        System.out.println("Avec calcul d'intersection\n");

        // ========== TEST 1: Polygone original du Cameroun ==========
        System.out.println("TEST 1: Polygone du Cameroun");
        List<String> coordsStr1 = List.of(
                "3.899792,11.523022",
                "3.913930,11.524018",
                "3.912703,11.535203",
                "3.904144,11.532831"
        );

        List<GeoPoint> polygon1 = coordsStr1.stream()
                .map(GeodesicPolygonCalculator::parseCoordinate)
                .collect(Collectors.toList());

        PolygonResult result1 = GeodesicPolygonCalculator.calculatePolygon(polygon1);
        System.out.println("Polygone 1:");
        for (int i = 0; i < polygon1.size(); i++) {
            System.out.println("  Point " + (i + 1) + ": " + polygon1.get(i));
        }
        System.out.println(result1);

        // ========== TEST 2: Polygone qui chevauche partiellement ==========
        System.out.println("\n\nTEST 2: Polygone avec chevauchement partiel");
        List<GeoPoint> polygon2 = List.of(
                new GeoPoint(3.905, 11.528),  // Intérieur à polygon1
                new GeoPoint(3.915, 11.529),  // Extérieur à polygon1
                new GeoPoint(3.914, 11.540),  // Extérieur à polygon1
                new GeoPoint(3.906, 11.538)   // Proche du bord de polygon1
        );

        PolygonResult result2 = GeodesicPolygonCalculator.calculatePolygon(polygon2);
        System.out.println("Polygone 2:");
        for (int i = 0; i < polygon2.size(); i++) {
            System.out.println("  Point " + (i + 1) + ": " + polygon2.get(i));
        }
        System.out.println(result2);

        // Calcul de l'intersection
        System.out.println("\n--- CALCUL DE L'INTERSECTION ---");
        long startTime = System.nanoTime();

        IntersectionResult intersection = GeodesicPolygonCalculator.calculateIntersection(polygon1, polygon2);

        long endTime = System.nanoTime();
        double durationMs = (endTime - startTime) / 1_000_000.0;

        System.out.println(intersection);
        if (intersection.intersects) {
            System.out.println("\nPoints du polygone d'intersection:");
            for (int i = 0; i < intersection.intersectionPolygon.size(); i++) {
                System.out.println("  Point " + (i + 1) + ": " +
                        intersection.intersectionPolygon.get(i));
            }

            // Pourcentages par rapport aux polygones originaux
            double pct1 = (intersection.intersectionArea / result1.area) * 100;
            double pct2 = (intersection.intersectionArea / result2.area) * 100;
            System.out.println(String.format("\nPourcentage de Polygone 1: %.2f%%", pct1));
            System.out.println(String.format("Pourcentage de Polygone 2: %.2f%%", pct2));
        }
        System.out.println("\nTemps de calcul: " + String.format("%.3f", durationMs) + " ms");

        // ========== TEST 3: Polygones disjoints ==========
        System.out.println("\n\nTEST 3: Polygones disjoints (pas de chevauchement)");
        List<GeoPoint> polygon3 = List.of(
                new GeoPoint(3.920, 11.540),
                new GeoPoint(3.930, 11.541),
                new GeoPoint(3.929, 11.550),
                new GeoPoint(3.921, 11.549)
        );

        IntersectionResult intersection2 = GeodesicPolygonCalculator.calculateIntersection(polygon1, polygon3);
        System.out.println(intersection2);

        // ========== TEST 4: Un polygone complètement inclus dans l'autre ==========
        System.out.println("\n\nTEST 4: Polygone complètement inclus");
        List<GeoPoint> polygon4 = List.of(
                new GeoPoint(3.905, 11.527),
                new GeoPoint(3.908, 11.528),
                new GeoPoint(3.907, 11.530),
                new GeoPoint(3.906, 11.529)
        );

        PolygonResult result4 = GeodesicPolygonCalculator.calculatePolygon(polygon4);
        IntersectionResult intersection3 = GeodesicPolygonCalculator.calculateIntersection(polygon1, polygon4);
        System.out.println(intersection3);

        if (intersection3.intersects) {
            System.out.println(String.format(
                    "\nAire de polygon4: %.2f m²\n" +
                            "Aire d'intersection: %.2f m²\n" +
                            "Différence: %.2f m² (%.4f%%)",
                    result4.area,
                    intersection3.intersectionArea,
                    Math.abs(result4.area - intersection3.intersectionArea),
                    Math.abs(result4.area - intersection3.intersectionArea) / result4.area * 100
            ));
        }

        // ========== TEST 5: Cas réel - Deux parcelles voisines ==========
        System.out.println("\n\nTEST 5: Deux parcelles voisines avec frontière commune");

        List<GeoPoint> parcelle1 = List.of(
                new GeoPoint(3.900, 11.525),
                new GeoPoint(3.910, 11.526),
                new GeoPoint(3.909, 11.535),
                new GeoPoint(3.901, 11.534)
        );

        List<GeoPoint> parcelle2 = List.of(
                new GeoPoint(3.910, 11.526),  // Point commun
                new GeoPoint(3.920, 11.527),
                new GeoPoint(3.919, 11.536),
                new GeoPoint(3.909, 11.535)   // Point commun
        );

        IntersectionResult intersection4 = GeodesicPolygonCalculator.calculateIntersection(parcelle1, parcelle2);
        System.out.println(intersection4);
        System.out.println("Note: Deux parcelles partageant uniquement un bord " +
                "ont une intersection d'aire nulle (ou quasi-nulle).");
    }
}