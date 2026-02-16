import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

public class GeodesicPolygonCalculator {

    // Constants for the WGS-84 ellipsoid
    private static final double A = 6378137.0;              // Equator semi-axis (m)
    private static final double F = 1.0 / 298.257223563;    // Flattening
    private static final double B = A * (1 - F);            // Polar semi-axis (m)
    private static final double E2 = F * (2 - F);           // Eccentricity of ellipsoid

    // Tolérance pour la convergence de Newton
    private static final double EPSILON = 1e-15;
    private static final int MAX_ITERATIONS = 20;

    /**
     * ÉTAPE 1: Convertir latitude géodésique en latitude paramétrique
     * Formule (29) du document: tan(β) = (1 - f) × tan(φ)
     */
    private static double geodeticToParametric(double phi) {
        double phiRad = Math.toRadians(phi);
        return Math.atan((1 - F) * Math.tan(phiRad));
    }

    /**
     * ÉTAPE 2: Calculer les coefficients g1,i de la série binomiale
     * Formule (13) du document: g1,0 = 1; g1,i>0 = (2i-3)/(2i) × e² × g1,i-1
     */
    private static double[] calculateG1Coefficients(int n) {
        double[] g1 = new double[n];
        g1[0] = 1.0;

        for (int i = 1; i < n; i++) {
            g1[i] = ((2.0 * i - 3.0) / (2.0 * i)) * E2 * g1[i - 1];
        }

        return g1;
    }

    /**
     * ÉTAPE 3: Calculer les coefficients t_k pour l'aire
     * Formule (55-56) du document
     */
    private static double[] calculateTCoefficients(double[] g1, int n) {
        double[] t = new double[n];

        // Calcul de t0 avec la formule de Mollweide (56)
        t[0] = 0.5 + Math.sqrt(1 - E2) / (4 * Math.sqrt(E2)) *
                Math.log((1 + Math.sqrt(E2)) / (1 - Math.sqrt(E2)));

        // Calcul récursif des autres coefficients (55)
        for (int k = 1; k < n; k++) {
            t[k] = ((2.0 * k - 1.0) * t[k - 1] - g1[k - 1]) / (2.0 * k);
        }

        return t;
    }

    /**
     * ÉTAPE 4: Calculer les coefficients g2,k
     * Formule (62) du document: g2,k = Σ(tj × g1,k-j) pour j de 0 à k
     */
    private static double[] calculateG2Coefficients(double[] g1, double[] t, int n) {
        double[] g2 = new double[n];

        for (int k = 0; k < n; k++) {
            g2[k] = 0.0;
            for (int j = 0; j <= k; j++) {
                g2[k] += t[j] * g1[k - j];
            }
        }

        return g2;
    }

    /**
     * ÉTAPE 5: Calculer Ls(C²) - Formule (26)
     */
    private static double calculateLs(double C2, double[] g1, double[] gamma) {
        double ls = 0.0;
        for (int i = 1; i < g1.length; i++) {
            ls += g1[i] * gamma[i];
        }
        return ls;
    }

    /**
     * ÉTAPE 6: Calculer Ss(C²) - Formule (26)
     */
    private static double calculateSs(double C2, double[] g1, double[] gamma) {
        double ss = 0.0;
        for (int i = 1; i < g1.length; i++) {
            ss += g1[i - 1] * gamma[i];
        }
        return ss;
    }

    /**
     * ÉTAPE 7: Calculer les coefficients gamma_i - Formule (22)
     */
    private static double[] calculateGamma(double C2, int n) {
        double[] gamma = new double[n];
        gamma[0] = 0.0;
        gamma[1] = 1.0;

        for (int i = 2; i < n; i++) {
            gamma[i] = ((2.0 * i - 3.0) / (2.0 * i - 2.0)) * (1 + C2) * gamma[i - 1]
                    - ((2.0 * i - 4.0) / (2.0 * i - 2.0)) * C2 * gamma[i - 2];
        }

        return gamma;
    }

    /**
     * ÉTAPE 8: Calculer les coefficients theta_k - Formule (67)
     */
    private static double[] calculateTheta(double C2, double rho2, int n) {
        double[] theta = new double[n];
        theta[0] = 0.0;

        for (int k = 1; k < n; k++) {
            theta[k] = (-Math.pow(rho2, k - 1) + (2.0 * k - 2.0) * C2 * theta[k - 1])
                    / (2.0 * k - 1.0);
        }

        return theta;
    }

    /**
     * ÉTAPE 9: Calculer la fonction Ar - Formule (69)
     */
    private static double calculateAr(double C2, double rho2, double[] g2, double[] theta) {
        double ar = 0.0;
        for (int k = 1; k < g2.length; k++) {
            ar += g2[k] * theta[k];
        }
        return (rho2 / 2.0) * ar;
    }

    /**
     * ÉTAPE 10: Résoudre le problème géodésique inverse
     * Retourne: [C, alpha1, alpha2, s12, sigma12, omega12]
     */
    private static double[] solveInverseGeodetic(GeoPoint p1, GeoPoint p2) {
        // Conversion en latitudes paramétriques
        double beta1 = geodeticToParametric(p1.latitude);
        double beta2 = geodeticToParametric(p2.latitude);

        double rho1 = Math.cos(beta1);
        double rho2 = Math.cos(beta2);
        double zeta1 = Math.sin(beta1);
        double zeta2 = Math.sin(beta2);

        // Différence de longitude
        double lambda12 = Math.toRadians(p2.longitude - p1.longitude);

        // Normalisation de lambda12 dans [-π, π]
        while (lambda12 > Math.PI) lambda12 -= 2 * Math.PI;
        while (lambda12 < -Math.PI) lambda12 += 2 * Math.PI;

        // Estimation initiale de omega12 (longitude sphérique)
        double omega12 = lambda12;

        // Calcul de sigma12 (distance sphérique) - Formule (94)
        double q1 = rho2 * Math.sin(omega12);
        double q2 = rho1 * zeta2 - rho2 * zeta1 * Math.cos(omega12);
        double q5 = Math.sqrt(q1 * q1 + q2 * q2);
        double sigma12 = Math.atan2(q5, zeta1 * zeta2 + rho1 * rho2 * Math.cos(omega12));

        // Estimation initiale de C - Formule (106)
        double[] g1 = calculateG1Coefficients(8);
        double Cmax = Math.min(rho1, rho2);
        double[] gamma_init = calculateGamma(Cmax * Cmax, 8);
        double Lsm = calculateLs(Cmax * Cmax, g1, gamma_init);

        double C = (rho1 * rho2 * Math.sin(lambda12)) /
                (Math.sin(sigma12) + Lsm * sigma12 * rho1 * rho2 * Math.cos(lambda12));

        // Limiter C à la plage valide
        C = Math.max(-Cmax, Math.min(Cmax, C));

        // Itération de Newton - Section 7.1 du document
        int iterations = 0;
        double delta = Double.MAX_VALUE;

        while (Math.abs(delta) > EPSILON && iterations < MAX_ITERATIONS) {
            double C2 = C * C;
            double[] gamma = calculateGamma(C2, 8);
            double Ls = calculateLs(C2, g1, gamma);
            double Ss = calculateSs(C2, g1, gamma);

            // Calcul des fonctions E (extension sphérique) - Formules (31-32)
            double alpha1 = Math.asin(C / rho1);
            double alpha2 = Math.asin(C / rho2);
            double E1 = zeta1 * rho1 * Math.cos(alpha1);
            double E2 = zeta2 * rho2 * Math.cos(alpha2);

            // Approximation simplifiée: Le ≈ 0 pour l'initialisation
            double Le1 = 0.0;
            double Le2 = 0.0;

            // Équation fondamentale (76)
            delta = lambda12 - omega12 - C * (sigma12 * Ls + E2 * Le2 - E1 * Le1);

            // Dérivées pour Newton - Formules (82-83)
            double d1 = Ls * sigma12 + Le2 * E2 - Le1 * E1
                    + C2 * sigma12 * (g1[2] + (1 + 3 * C2) / 2.0 * g1[3]);
            double d2 = 1 + C2 * Ls;

            // Coefficients h1, h2 - Formules (85-87)
            double h1 = Math.sin(sigma12);
            double h2 = -rho1 * rho2 * Math.cos(alpha1) * Math.cos(alpha2);

            // Choix dynamique du paramètre - Section 7.1
            double D = d1 * h2 - d2 * h1;

            if (Math.abs(D) > 1e-10) {
                double dC = h2 * delta / D;
                double dOmega = -h1 * delta / D;

                // Choix adaptatif du paramètre (89-90)
                if (Math.abs(h1 * d2) < Math.abs(h2 * d1)) {
                    C += dC;
                } else {
                    omega12 += dOmega;
                    // Recalcul de sigma12
                    q1 = rho2 * Math.sin(omega12);
                    q2 = rho1 * zeta2 - rho2 * zeta1 * Math.cos(omega12);
                    q5 = Math.sqrt(q1 * q1 + q2 * q2);
                    sigma12 = Math.atan2(q5, zeta1 * zeta2 + rho1 * rho2 * Math.cos(omega12));
                    C = rho1 * rho2 * Math.sin(omega12) / Math.sin(sigma12);
                }
            } else {
                C -= delta / d1;
            }

            // Limiter C à la plage valide
            C = Math.max(-Cmax, Math.min(Cmax, C));

            iterations++;
        }

        // Calcul final des azimuths et de la distance - Section 7.2
        double C2 = C * C;
        double[] gamma = calculateGamma(C2, 8);
        double Ss = calculateSs(C2, g1, gamma);

        double alpha1 = Math.asin(C / rho1);
        double alpha2 = Math.asin(C / rho2);

        // Correction du signe des azimuths
        double m1 = Math.sqrt(rho1 * rho1 - C2);
        double m2 = Math.sqrt(rho2 * rho2 - C2);

        if (zeta2 - zeta1 * Math.cos(sigma12) < 0) m1 = -m1;
        if (zeta1 - zeta2 * Math.cos(sigma12) >= 0) m2 = -m2;

        alpha1 = Math.atan2(C, m1);
        alpha2 = Math.atan2(C, m2);

        // Distance normalisée S12 - Formule (40)
        double E1 = zeta1 * rho1 * Math.cos(alpha1);
        double E2 = zeta2 * rho2 * Math.cos(alpha2);

        double Se1 = 0.0;  // Approximation simplifiée
        double Se2 = 0.0;

        double S12 = sigma12 * Ss + E2 * Se2 - E1 * Se1;
        double s12 = A * S12;  // Distance en mètres

        return new double[]{C, alpha1, alpha2, s12, sigma12, omega12};
    }

    /**
     * ÉTAPE 11: Calculer l'aire sous une géodésique - Formule (72)
     */
    private static double calculateAreaUnderGeodesic(GeoPoint p1, GeoPoint p2) {
        // Résoudre le problème inverse
        double[] result = solveInverseGeodetic(p1, p2);
        double C = result[0];
        double alpha1 = result[1];
        double alpha2 = result[2];

        // Conversion en latitudes paramétriques
        double beta1 = geodeticToParametric(p1.latitude);
        double beta2 = geodeticToParametric(p2.latitude);
        double rho1 = Math.cos(beta1);
        double rho2 = Math.cos(beta2);

        // Calcul des coefficients
        double[] g1 = calculateG1Coefficients(8);
        double[] t = calculateTCoefficients(g1, 8);
        double[] g2 = calculateG2Coefficients(g1, t, 8);

        // Différence d'azimuths - directement calculée
        double alpha12 = alpha2 - alpha1;

        // Normalisation dans [-π, π]
        while (alpha12 > Math.PI) alpha12 -= 2 * Math.PI;
        while (alpha12 < -Math.PI) alpha12 += 2 * Math.PI;

        // Calcul de la fonction Ar pour chaque point
        double C2 = C * C;
        double[] theta1 = calculateTheta(C2, rho1 * rho1, 8);
        double[] theta2 = calculateTheta(C2, rho2 * rho2, 8);
        double Ar1 = calculateAr(C2, rho1 * rho1, g2, theta1);
        double Ar2 = calculateAr(C2, rho2 * rho2, g2, theta2);

        // Aire normalisée sous la géodésique - Formule (72)
        double A12 = g2[0] * alpha12
                + Math.sin(2 * alpha2) * Ar2
                - Math.sin(2 * alpha1) * Ar1;

        return A12;  // Aire normalisée (à multiplier par a² pour obtenir l'aire en m²)
    }

    /**
     * ÉTAPE 12: Calculer l'aire et le périmètre du polygone
     * Formule (73) du document pour l'aire
     */
    public static PolygonResult calculatePolygon(List<GeoPoint> points) {
        if (points.size() < 3) {
            throw new IllegalArgumentException("Un polygone doit avoir au moins 3 points");
        }

        double totalArea = 0.0;      // Aire normalisée totale
        double totalPerimeter = 0.0; // Périmètre en mètres

        int n = points.size();

        // Parcourir tous les segments du polygone
        for (int i = 0; i < n; i++) {
            GeoPoint p1 = points.get(i);
            GeoPoint p2 = points.get((i + 1) % n);  // Retour au premier point

            // Aire sous la géodésique (normalisée)
            double areaSegment = calculateAreaUnderGeodesic(p1, p2);
            totalArea += areaSegment;

            // Distance du segment
            double[] result = solveInverseGeodetic(p1, p2);
            double distance = result[3];
            totalPerimeter += distance;
        }

        // Conversion de l'aire normalisée en m² - Formule (73)
        double areaM2 = Math.abs(totalArea * A * A);

        return new PolygonResult(areaM2, totalPerimeter);
    }

    /**
     * Méthode utilitaire pour parser une chaîne de coordonnées
     * Format: "latitude,longitude"
     */
    public static GeoPoint parseCoordinate(String coord) {
        String[] parts = coord.split(",");
        if (parts.length != 2) {
            throw new IllegalArgumentException("Format invalide: " + coord);
        }
        double lat = Double.parseDouble(parts[0].trim());
        double lon = Double.parseDouble(parts[1].trim());
        return new GeoPoint(lat, lon);
    }


    /**
     * ÉTAPE 13: Vérifier si un point est à l'intérieur d'un polygone
     * Utilise l'algorithme du "ray casting" adapté aux géodésiques
     *
     * Principe: Tracer un rayon depuis le point vers l'est (longitude croissante)
     * et compter combien de fois il croise les bords du polygone.
     * - Nombre impair de croisements = intérieur
     * - Nombre pair = extérieur
     */
    private static boolean isPointInsidePolygon(GeoPoint point, List<GeoPoint> polygon) {
        int n = polygon.size();
        int crossings = 0;

        // Coordonnées du point de test
        double testLat = point.latitude;
        double testLon = point.longitude;

        for (int i = 0; i < n; i++) {
            GeoPoint p1 = polygon.get(i);
            GeoPoint p2 = polygon.get((i + 1) % n);

            // Le segment croise-t-il l'horizontale passant par le point?
            if ((p1.latitude <= testLat && p2.latitude > testLat) ||
                    (p2.latitude <= testLat && p1.latitude > testLat)) {

                // Calculer la longitude d'intersection du segment avec l'horizontale
                // Approximation linéaire (suffisante pour petites distances)
                double lonIntersection = p1.longitude +
                        (testLat - p1.latitude) * (p2.longitude - p1.longitude) /
                                (p2.latitude - p1.latitude);

                // Le croisement est-il à l'est du point?
                if (lonIntersection > testLon) {
                    crossings++;
                }
            }
        }

        return (crossings % 2) == 1;
    }

    /**
     * ÉTAPE 14: Calculer l'intersection de deux segments géodésiques
     *
     * Utilise une approche paramétrique:
     * - Segment 1: P(t) pour t ∈ [0,1]
     * - Segment 2: Q(s) pour s ∈ [0,1]
     * - Cherche t,s tels que P(t) = Q(s)
     *
     * @return Point d'intersection ou null si pas d'intersection
     */
    private static GeoPoint findSegmentIntersection(
            GeodesicSegment seg1,
            GeodesicSegment seg2) {

        // Extraction des coordonnées
        double lat1a = seg1.start.latitude;
        double lon1a = seg1.start.longitude;
        double lat1b = seg1.end.latitude;
        double lon1b = seg1.end.longitude;

        double lat2a = seg2.start.latitude;
        double lon2a = seg2.start.longitude;
        double lat2b = seg2.end.latitude;
        double lon2b = seg2.end.longitude;

        // Approximation linéaire pour petites distances
        // (Pour une précision maximale, il faudrait résoudre avec les géodésiques exactes)

        // Équations paramétriques:
        // lat1 = lat1a + t * (lat1b - lat1a)
        // lon1 = lon1a + t * (lon1b - lon1a)
        // lat2 = lat2a + s * (lat2b - lat2a)
        // lon2 = lon2a + s * (lon2b - lon2a)

        double dlat1 = lat1b - lat1a;
        double dlon1 = lon1b - lon1a;
        double dlat2 = lat2b - lat2a;
        double dlon2 = lon2b - lon2a;

        // Normalisation des longitudes pour gérer le méridien ±180°
        if (Math.abs(dlon1) > 180) {
            dlon1 = dlon1 > 0 ? dlon1 - 360 : dlon1 + 360;
        }
        if (Math.abs(dlon2) > 180) {
            dlon2 = dlon2 > 0 ? dlon2 - 360 : dlon2 + 360;
        }

        // Résolution du système linéaire:
        // lat1a + t*dlat1 = lat2a + s*dlat2
        // lon1a + t*dlon1 = lon2a + s*dlon2
        //
        // Forme matricielle: [dlat1 -dlat2] [t] = [lat2a - lat1a]
        //                    [dlon1 -dlon2] [s]   [lon2a - lon1a]

        double determinant = dlat1 * (-dlon2) - dlon1 * (-dlat2);

        // Segments parallèles ou colinéaires
        if (Math.abs(determinant) < 1e-10) {
            return null;
        }

        double dlat = lat2a - lat1a;
        double dlon = lon2a - lon1a;

        // Normalisation
        if (Math.abs(dlon) > 180) {
            dlon = dlon > 0 ? dlon - 360 : dlon + 360;
        }

        // Règle de Cramer
        double t = (dlat * (-dlon2) - dlon * (-dlat2)) / determinant;
        double s = (dlat1 * dlon - dlon1 * dlat) / determinant;

        // Vérifier que l'intersection est dans les deux segments
        if (t >= 0 && t <= 1 && s >= 0 && s <= 1) {
            double intersectLat = lat1a + t * dlat1;
            double intersectLon = lon1a + t * dlon1;

            // Normalisation de la longitude
            while (intersectLon > 180) intersectLon -= 360;
            while (intersectLon < -180) intersectLon += 360;

            return new GeoPoint(intersectLat, intersectLon);
        }

        return null;
    }

    /**
     * ÉTAPE 15: Trouver tous les points d'intersection entre deux polygones
     *
     * Teste chaque segment du polygone 1 contre chaque segment du polygone 2
     *
     * @return Liste de tous les points d'intersection
     */
    private static List<GeoPoint> findAllIntersections(
            List<GeoPoint> polygon1,
            List<GeoPoint> polygon2) {

        List<GeoPoint> intersections = new ArrayList<>();
        Set<String> uniquePoints = new HashSet<>();  // Pour éviter les doublons

        int n1 = polygon1.size();
        int n2 = polygon2.size();

        // Tester chaque paire de segments
        for (int i = 0; i < n1; i++) {
            GeodesicSegment seg1 = new GeodesicSegment(
                    polygon1.get(i),
                    polygon1.get((i + 1) % n1)
            );

            for (int j = 0; j < n2; j++) {
                GeodesicSegment seg2 = new GeodesicSegment(
                        polygon2.get(j),
                        polygon2.get((j + 1) % n2)
                );

                GeoPoint intersection = findSegmentIntersection(seg1, seg2);

                if (intersection != null) {
                    // Clé unique pour éviter les doublons (précision à 1e-9 degré ≈ 0.1mm)
                    String key = String.format("%.9f,%.9f",
                            intersection.latitude,
                            intersection.longitude);

                    if (uniquePoints.add(key)) {
                        intersections.add(intersection);
                    }
                }
            }
        }

        return intersections;
    }

    /**
     * ÉTAPE 16: Construire le polygone d'intersection
     *
     * Utilise l'algorithme de Sutherland-Hodgman adapté aux polygones géodésiques
     *
     * Principe:
     * 1. Trouver tous les points d'intersection
     * 2. Inclure les sommets de poly1 qui sont dans poly2
     * 3. Inclure les sommets de poly2 qui sont dans poly1
     * 4. Trier les points dans l'ordre géométrique (sens horaire ou antihoraire)
     *
     * @return Polygone d'intersection (liste ordonnée de points)
     */
    private static List<GeoPoint> buildIntersectionPolygon(
            List<GeoPoint> polygon1,
            List<GeoPoint> polygon2) {

        List<GeoPoint> intersectionPoints = new ArrayList<>();

        // 1. Ajouter les points d'intersection entre les bords
        List<GeoPoint> edgeIntersections = findAllIntersections(polygon1, polygon2);
        intersectionPoints.addAll(edgeIntersections);

        // 2. Ajouter les sommets de polygon1 qui sont dans polygon2
        for (GeoPoint p : polygon1) {
            if (isPointInsidePolygon(p, polygon2)) {
                intersectionPoints.add(p);
            }
        }

        // 3. Ajouter les sommets de polygon2 qui sont dans polygon1
        for (GeoPoint p : polygon2) {
            if (isPointInsidePolygon(p, polygon1)) {
                intersectionPoints.add(p);
            }
        }

        // Si moins de 3 points, pas de polygone d'intersection valide
        if (intersectionPoints.size() < 3) {
            return new ArrayList<>();
        }

        // 4. Éliminer les doublons
        intersectionPoints = removeDuplicatePoints(intersectionPoints);

        if (intersectionPoints.size() < 3) {
            return new ArrayList<>();
        }

        // 5. Trier les points dans l'ordre géométrique
        return sortPointsClockwise(intersectionPoints);
    }

    /**
     * ÉTAPE 17: Éliminer les points en double
     *
     * Deux points sont considérés identiques s'ils sont à moins de 1e-9° (≈0.1mm)
     */
    private static List<GeoPoint> removeDuplicatePoints(List<GeoPoint> points) {
        List<GeoPoint> unique = new ArrayList<>();

        for (GeoPoint p : points) {
            boolean isDuplicate = false;

            for (GeoPoint existing : unique) {
                double latDiff = Math.abs(p.latitude - existing.latitude);
                double lonDiff = Math.abs(p.longitude - existing.longitude);

                if (latDiff < 1e-9 && lonDiff < 1e-9) {
                    isDuplicate = true;
                    break;
                }
            }

            if (!isDuplicate) {
                unique.add(p);
            }
        }

        return unique;
    }

    /**
     * ÉTAPE 18: Trier les points dans le sens horaire autour de leur centroïde
     *
     * Algorithme:
     * 1. Calculer le centroïde (point moyen)
     * 2. Calculer l'angle polaire de chaque point par rapport au centroïde
     * 3. Trier par angle croissant
     *
     * Cette méthode garantit que les points forment un polygone valide
     */
    private static List<GeoPoint> sortPointsClockwise(List<GeoPoint> points) {
        if (points.size() < 3) {
            return points;
        }

        // Calculer le centroïde
        double centerLat = 0;
        double centerLon = 0;

        for (GeoPoint p : points) {
            centerLat += p.latitude;
            centerLon += p.longitude;
        }

        centerLat /= points.size();
        centerLon /= points.size();

        final double finalCenterLat = centerLat;
        final double finalCenterLon = centerLon;

        // Trier par angle polaire (dans le sens horaire = angle décroissant)
        List<GeoPoint> sorted = new ArrayList<>(points);
        sorted.sort((p1, p2) -> {
            double angle1 = Math.atan2(p1.latitude - finalCenterLat, p1.longitude - finalCenterLon);
            double angle2 = Math.atan2(p2.latitude - finalCenterLat, p2.longitude - finalCenterLon);
            return Double.compare(angle2, angle1);  // Ordre décroissant pour sens horaire
        });

        return sorted;
    }

    /**
     * ÉTAPE 19: Vérifier si deux polygones se chevauchent (test rapide)
     *
     * Utilise les boîtes englobantes (bounding boxes) pour un test préliminaire rapide
     *
     * @return true si les boîtes englobantes se chevauchent
     */
    private static boolean boundingBoxesOverlap(
            List<GeoPoint> polygon1,
            List<GeoPoint> polygon2) {

        // Calculer les boîtes englobantes
        double[] bbox1 = calculateBoundingBox(polygon1);
        double[] bbox2 = calculateBoundingBox(polygon2);

        // [minLat, maxLat, minLon, maxLon]
        double minLat1 = bbox1[0], maxLat1 = bbox1[1];
        double minLon1 = bbox1[2], maxLon1 = bbox1[3];
        double minLat2 = bbox2[0], maxLat2 = bbox2[1];
        double minLon2 = bbox2[2], maxLon2 = bbox2[3];

        // Test de chevauchement
        boolean latOverlap = !(maxLat1 < minLat2 || maxLat2 < minLat1);
        boolean lonOverlap = !(maxLon1 < minLon2 || maxLon2 < minLon1);

        return latOverlap && lonOverlap;
    }

    /**
     * Calculer la boîte englobante d'un polygone
     *
     * @return [minLat, maxLat, minLon, maxLon]
     */
    private static double[] calculateBoundingBox(List<GeoPoint> polygon) {
        double minLat = Double.MAX_VALUE;
        double maxLat = -Double.MAX_VALUE;
        double minLon = Double.MAX_VALUE;
        double maxLon = -Double.MAX_VALUE;

        for (GeoPoint p : polygon) {
            minLat = Math.min(minLat, p.latitude);
            maxLat = Math.max(maxLat, p.latitude);
            minLon = Math.min(minLon, p.longitude);
            maxLon = Math.max(maxLon, p.longitude);
        }

        return new double[]{minLat, maxLat, minLon, maxLon};
    }

    /**
     * MÉTHODE PRINCIPALE: Calculer l'intersection de deux polygones
     *
     * Cette méthode orchestre toutes les étapes pour calculer l'aire d'intersection
     *
     * Algorithme:
     * 1. Test rapide des boîtes englobantes
     * 2. Construction du polygone d'intersection
     * 3. Calcul de l'aire du polygone d'intersection
     *
     * @param polygon1 Premier polygone (liste de points GPS)
     * @param polygon2 Deuxième polygone (liste de points GPS)
     * @return Résultat contenant l'aire d'intersection et le polygone
     */
    public static IntersectionResult calculateIntersection(
            List<GeoPoint> polygon1,
            List<GeoPoint> polygon2) {

        // Validation
        if (polygon1.size() < 3 || polygon2.size() < 3) {
            throw new IllegalArgumentException("Chaque polygone doit avoir au moins 3 points");
        }

        // Test rapide: les boîtes englobantes se chevauchent-elles?
        if (!boundingBoxesOverlap(polygon1, polygon2)) {
            return new IntersectionResult(false, 0.0, new ArrayList<>());
        }

        // Construire le polygone d'intersection
        List<GeoPoint> intersectionPolygon = buildIntersectionPolygon(polygon1, polygon2);

        // Pas d'intersection valide?
        if (intersectionPolygon.size() < 3) {
            return new IntersectionResult(false, 0.0, new ArrayList<>());
        }

        // Calculer l'aire du polygone d'intersection
        PolygonResult result = calculatePolygon(intersectionPolygon);

        return new IntersectionResult(
                true,
                result.area,
                intersectionPolygon
        );
    }
}