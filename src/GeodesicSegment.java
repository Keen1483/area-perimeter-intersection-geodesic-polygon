/**
 * Classe représentant un segment géodésique
 */
public class GeodesicSegment {
    GeoPoint start;
    GeoPoint end;

    public GeodesicSegment(GeoPoint start, GeoPoint end) {
        this.start = start;
        this.end = end;
    }

    @Override
    public String toString() {
        return String.format("[%s → %s]", start, end);
    }
}
