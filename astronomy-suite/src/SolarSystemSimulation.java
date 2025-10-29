import javax.swing.*;
import java.awt.*;
import java.awt.event.*;
import java.awt.geom.Ellipse2D;
import java.awt.geom.Path2D;
import java.awt.geom.Point2D;
import java.util.ArrayList;

/**
 * Solar System Simulation – A real-time, interactive 2D orbital mechanics visualization.
 * Features:
 *  - Accurate Keplerian orbits using Newton's method to solve the eccentric anomaly.
 *  - Moons with independent orbital periods and scaled sizes.
 *  - Smooth camera panning and instant planet-following mode.
 *  - Dynamic zoom, time scaling, and interactive HUD.
 *  - Anti-aliased rendering with high visual quality.
 *
 * @author Ethan Lin
 * @version 1.1
 */
public class SolarSystemSimulation extends JPanel
        implements ActionListener, MouseWheelListener, MouseListener {

    /* ==============================
       PHYSICAL CONSTANTS (SI UNITS)
       ============================== */
    private static final double AU  = 1.496e11;               // 1 Astronomical Unit in metres
    private static final double DAY = 86_400.0;               // Seconds in one Earth day
    private static final double G   = 6.67430e-11;             // Gravitational constant (m³ kg⁻¹ s⁻²)
    private static final double M_SUN = 1.98847e30;           // Mass of the Sun (kg)

    // Scale: 250 pixels = 1 AU → converts real distances to screen pixels
    private static final double METERS_TO_PIXELS = 250.0 / AU;

    /* ==============================
       CAMERA SYSTEM
       ============================== */
    private boolean followingPlanet = false;     // Is camera locked to a planet?
    private int     cameraFocusIndex = -1;       // Index of planet being followed (-1 = none)
    private double  camX = 0, camY = 0;          // Camera center position in metres (world space)

    /* ==============================
       SIMULATION STATE
       ============================== */
    private double zoom       = 1.0;              // Current zoom level (1.0 = default)
    private double timeScale  = 1_000;            // Simulation speed: sim-seconds per real-second
    private double simTimeSec = 0;               // Total elapsed simulation time in seconds
    private long   lastNanos;                    // Timestamp of last frame (for delta time)

    /* ==============================
       PLANET DATA (8 major planets)
       ============================== */
    private static final String[] NAMES = {
            "Mercury", "Venus", "Earth", "Mars",
            "Jupiter", "Saturn", "Uranus", "Neptune", "Pluto"
    };

    // Semi-major axis in metres (a)
    private static final double[] A_METERS = {
            0.387*AU, 0.723*AU, AU, 1.524*AU,
            5.203*AU, 9.537*AU, 19.191*AU, 30.07*AU, 39.482*AU
    };

    // Orbital eccentricity (e)
    private static final double[] ECC = {
            0.2056, 0.0068, 0.0167, 0.0934,
            0.0489, 0.0565, 0.0472, 0.0086, 0.2488
    };

    // Orbital period in Earth years
    private static final double[] PERIOD_Y = {
            0.241, 0.615, 1.000, 1.881,
            11.86, 29.46, 84.01, 164.8, 247.94
    };

    // Planet radius in metres
    private static final double[] RADIUS_M = {
            2.439e6, 6.052e6, 6.371e6, 3.390e6,
            6.9911e7, 5.8232e7, 2.5362e7, 2.4622e7,
            1.1883e6
    };

    private static final double SUN_RADIUS_M = 6.9634e8;  // Sun's radius in metres

    // Visual colors for each planet
    private static final Color[] COLORS = {
            Color.GRAY, new Color(255,180,0), Color.CYAN, Color.RED,
            new Color(255,200,100), new Color(200,180,150),
            new Color(150,200,255), new Color(100,150,255), new Color(233, 226, 214)
    };

    /* ==============================
       MOON DATA (per planet)
       ============================== */
    // Orbital radii of moons around their parent planet (metres)
    private static final double[][] MOON_ORBIT_M = {
            {}, {}, {384_400_000.0},                                      // Earth: The Moon
            {9_377_000.0, 20_560_000.0},                                  // Mars: Phobos, Deimos
            {421_700_000.0, 671_000_000.0, 1_070_000_000.0, 1_883_000_000.0}, // Jupiter: Io, Europa, Ganymede, Callisto
            {1_221_830_000.0, 238_000_000.0, 527_000_000.0},               // Saturn: Titan, Enceladus, Rhea
            {436_300_000.0, 583_520_000.0, 498_900_000.0},                 // Uranus: Titania, Oberon, Umbriel
            {356_000_000.0},                                               // Neptune: Triton
            {}
    };

    // Moon orbital periods in Earth days
    private static final double[][] MOON_PERIOD_DAYS = {
            {}, {}, {27.3},
            {0.318, 1.26},
            {1.77, 3.55, 7.15, 16.69},
            {16.69, 1.37, 2.74},
            {8.71, 13.46, 10.66},
            {5.88}, {}
    };

    // Moon radius relative to parent planet
    private static final double[][] MOON_SIZE_RATIO = {
            {}, {}, {0.27},
            {0.15, 0.12},
            {0.25, 0.22, 0.30, 0.26},
            {0.40, 0.20, 0.25},
            {0.25, 0.20, 0.20},
            {0.30}, {}
    };

    // Moon names
    private static final String[][] MOON_NAMES = {
            {}, {}, {"Moon"},
            {"Phobos", "Deimos"},
            {"Io", "Europa", "Ganymede", "Callisto"},
            {"Titan", "Enceladus", "Rhea"},
            {"Titania", "Oberon", "Umbriel"},
            {"Triton"}, {}
    };

    /* ==============================
       ORBITAL STATE VARIABLES
       ============================== */
    private final double[] meanAnomaly = new double[NAMES.length];     // M: mean anomaly (radians)
    private final double[][] moonAngle = new double[NAMES.length][];  // Angular position of each moon

    private static final int ORBIT_RESOLUTION = 10000; // number of points per orbit path
    private static final ArrayList<ArrayList<Point2D.Double>> orbitPaths = new ArrayList<>();

    // Current computed positions and velocities
    private final double[] planetX = new double[NAMES.length];        // x-position in metres
    private final double[] planetY = new double[NAMES.length];        // y-position in metres
    private final double[] planetDistM = new double[NAMES.length];    // distance from Sun
    private final double[] planetSpeedKmS = new double[NAMES.length]; // orbital speed (km/s)

    private int selectedPlanet = -1;  // Index of currently selected/followed planet

    /* ==============================
       CONSTRUCTOR
       ============================== */
    public SolarSystemSimulation() {
        // Panel setup
        setBackground(Color.BLACK);
        setPreferredSize(new Dimension(1200, 800));

        // Input listeners
        addMouseWheelListener(this);
        addMouseListener(this);

        // Initialize random starting positions for visual variety
        for (int i = 0; i < NAMES.length; i++) {
            meanAnomaly[i] = Math.random() * 2 * Math.PI;
        }
        for (int i = 0; i < NAMES.length; i++) {
            moonAngle[i] = new double[MOON_NAMES[i].length];
            for (int m = 0; m < moonAngle[i].length; m++) {
                moonAngle[i][m] = Math.random() * 2 * Math.PI;
            }
        }

        for (int i = 0; i < NAMES.length; i++) {
            ArrayList<Point2D.Double> orbit = new ArrayList<>();
            double a = A_METERS[i];
            double e = ECC[i];
            double b = a * Math.sqrt(1 - e * e);

            // Step evenly in eccentric anomaly for smooth curve
            for (int j = 0; j <= ORBIT_RESOLUTION; j++) {
                double E = 2 * Math.PI * j / ORBIT_RESOLUTION;
                double x = a * (Math.cos(E) - e);
                double y = b * Math.sin(E);
                orbit.add(new Point2D.Double(x, y));
            }

            orbitPaths.add(orbit);
        }

        // Start animation timer (~60 FPS)
        Timer timer = new Timer(16, this);
        timer.start();
        lastNanos = System.nanoTime();
    }

    /* ==============================
       SIMULATION UPDATE (per frame)
       ============================== */
    @Override
    public void actionPerformed(ActionEvent ev) {
        long now = System.nanoTime();
        double dtReal = (now - lastNanos) / 1e9;  // Real-time delta (seconds)
        lastNanos = now;

        double dtSim = dtReal * timeScale;        // Simulated time step
        simTimeSec += dtSim;

        /* ----- Update Planet Orbits ----- */
        for (int i = 0; i < NAMES.length; i++) {
            double periodSec = PERIOD_Y[i] * 365.25 * DAY;  // Orbital period in seconds
            double n = 2 * Math.PI / periodSec;             // Mean motion (rad/s)
            meanAnomaly[i] = (meanAnomaly[i] + n * dtSim) % (2 * Math.PI);
        }

        /* ----- Update Moon Orbits ----- */
        for (int i = 0; i < NAMES.length; i++) {
            for (int m = 0; m < moonAngle[i].length; m++) {
                double periodSec = MOON_PERIOD_DAYS[i][m] * DAY;
                double n = 2 * Math.PI / periodSec;
                moonAngle[i][m] = (moonAngle[i][m] + n * dtSim) % (2 * Math.PI);
            }
        }

        /* ----- Compute Planet Positions (Keplerian Orbit) ----- */
        for (int i = 0; i < NAMES.length; i++) {
            double a = A_METERS[i];     // semi-major axis
            double e = ECC[i];          // eccentricity
            double b = a * Math.sqrt(1 - e * e);  // semi-minor axis

            double M = meanAnomaly[i];  // Mean anomaly

            // Solve Kepler's equation: M = E - e*sin(E) using Newton-Raphson
            double E = M;  // Initial guess
            for (int it = 0; it < 40; it++) {
                double f  = E - e * Math.sin(E) - M;           // Function
                double fp = 1 - e * Math.cos(E);               // Derivative
                double d  = f / fp;                            // Newton step
                E -= d;
                if (Math.abs(d) < 1e-12) break;                // Converged
            }

            // True anomaly coordinates in orbital plane
            double x = a * (Math.cos(E) - e);
            double y = b * Math.sin(E);

            planetX[i] = x;
            planetY[i] = y;
            planetDistM[i] = Math.hypot(x, y);

            // Vis-viva equation: v = √[GM(2/r - 1/a)]
            double v = Math.sqrt(G * M_SUN * (2 / planetDistM[i] - 1 / a));
            planetSpeedKmS[i] = v / 1_000.0;  // Convert to km/s
        }

        /* ----- Camera Movement ----- */
        if (followingPlanet && cameraFocusIndex >= 0) {
            // Instant snap to selected planet
            camX = planetX[cameraFocusIndex];
            camY = planetY[cameraFocusIndex];
        } else {
            // Smooth return to origin when not following
            double targetX = 0, targetY = 0;
            double speed = 8.0;  // Smoothness factor
            double lerp = 1 - Math.exp(-speed * dtReal);  // Exponential ease
            camX += (targetX - camX) * lerp;
            camY += (targetY - camY) * lerp;
        }

        repaint();  // Trigger redraw
    }

    /* ==============================
       INPUT: Mouse Wheel (Zoom & Time)
       ============================== */
    @Override
    public void mouseWheelMoved(MouseWheelEvent e) {
        if (e.isControlDown()) {
            // Ctrl + Scroll → Adjust simulation speed
            timeScale = e.getWheelRotation() < 0 ? timeScale * 2 : timeScale / 2;
            timeScale = Math.max(1, Math.min(timeScale, 1e9));  // Clamp
        } else {
            // Normal Scroll → Adjust zoom
            zoom = e.getWheelRotation() < 0 ? zoom * 1.1 : zoom / 1.1;
            zoom = Math.max(0.025, Math.min(zoom, 1000));  // Clamp
        }
    }

    /* ==============================
       INPUT: Mouse Click (Planet Selection)
       ============================== */
    private void handleClick(int mx, int my) {
        int cx = getWidth() / 2, cy = getHeight() / 2;

        for (int i = 0; i < NAMES.length; i++) {
            // Convert planet world position to screen coordinates
            double px = cx + (planetX[i] - camX) * METERS_TO_PIXELS * zoom;
            double py = cy + (planetY[i] - camY) * METERS_TO_PIXELS * zoom;
            double pr = Math.max(RADIUS_M[i] * METERS_TO_PIXELS * zoom, 4);  // Min size

            // Click detection with 2x radius tolerance
            double dx = mx - px, dy = my - py;
            if (dx * dx + dy * dy < pr * pr * 4) {
                if (cameraFocusIndex == i && followingPlanet) {
                    // Click again to unfollow
                    followingPlanet = false;
                    cameraFocusIndex = -1;
                    selectedPlanet = -1;
                } else {
                    // Follow this planet
                    followingPlanet = true;
                    cameraFocusIndex = i;
                    selectedPlanet = i;
                }
                return;
            }
        }
        // Clicked empty space → unfollow
        followingPlanet = false;
        cameraFocusIndex = -1;
        selectedPlanet = -1;
    }

    // MouseListener stubs
    @Override public void mouseClicked(MouseEvent e) { handleClick(e.getX(), e.getY()); }
    @Override public void mousePressed(MouseEvent e) {}
    @Override public void mouseReleased(MouseEvent e) {}
    @Override public void mouseEntered(MouseEvent e) {}
    @Override public void mouseExited(MouseEvent e) {}

    /* ==============================
       RENDERING
       ============================== */
    @Override
    protected void paintComponent(Graphics g) {
        super.paintComponent(g);
        Graphics2D g2 = (Graphics2D) g;
        g2.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);
        g2.setRenderingHint(RenderingHints.KEY_RENDERING, RenderingHints.VALUE_RENDER_QUALITY);

        int cx = getWidth() / 2, cy = getHeight() / 2;
        java.util.List<Rectangle> labelBoxes = new java.util.ArrayList<>();

        /* ----- Draw the Sun ----- */
        double sunR = SUN_RADIUS_M * METERS_TO_PIXELS * zoom;
        double sunX = cx - sunR - camX * METERS_TO_PIXELS * zoom;
        double sunY = cy - sunR - camY * METERS_TO_PIXELS * zoom;
        g2.setColor(Color.ORANGE);
        g2.fill(new Ellipse2D.Double(sunX, sunY, 2 * sunR, 2 * sunR));

        FontMetrics fm = g2.getFontMetrics();

        /* ----- Draw Each Planet & Orbit ----- */
        for (int i = 0; i < NAMES.length; i++) {
            double worldX = planetX[i], worldY = planetY[i];
            double px = cx + (worldX - camX) * METERS_TO_PIXELS * zoom;
            double py = cy + (worldY - camY) * METERS_TO_PIXELS * zoom;
            double pr = RADIUS_M[i] * METERS_TO_PIXELS * zoom;
            if (pr < 0.5) pr = 0.5;  // Minimum visible size

            /* --- Precomputed Orbit Path (smooth Path2D) --- */
            ArrayList<Point2D.Double> orbit = orbitPaths.get(i);
            Path2D.Double path = new Path2D.Double();

            if (!orbit.isEmpty()) {
                Point2D.Double first = orbit.getFirst();
                double sx = cx + (first.x - camX) * METERS_TO_PIXELS * zoom;
                double sy = cy + (first.y - camY) * METERS_TO_PIXELS * zoom;
                path.moveTo(sx, sy);

                for (int j = 1; j < orbit.size(); j++) {
                    Point2D.Double p = orbit.get(j);
                    double x = cx + (p.x - camX) * METERS_TO_PIXELS * zoom;
                    double y = cy + (p.y - camY) * METERS_TO_PIXELS * zoom;
                    path.lineTo(x, y);
                }
                path.closePath();
            }

            g2.setColor(Color.DARK_GRAY);
            g2.setStroke(new BasicStroke(1f));
            g2.draw(path);

            /* --- Planet Body --- */
            g2.setColor(COLORS[i]);
            g2.fill(new Ellipse2D.Double(px - pr, py - pr, 2 * pr, 2 * pr));

            /* --- Planet Label (with overlap avoidance) --- */
            g2.setColor(Color.WHITE);
            String name = NAMES[i];
            int w = fm.stringWidth(name), h = fm.getHeight();
            double angle = Math.atan2(py - cy, px - cx);
            int lx = (int) (px + Math.cos(angle) * (pr + 12));
            int ly = (int) (py + Math.sin(angle) * (pr + 12));
            Rectangle box = new Rectangle(lx, ly - h, w, h);

            // Simple anti-overlap: shift down if colliding
            int tries = 0;
            while (tries < 4) {
                boolean overlap = false;
                for (Rectangle r : labelBoxes) {
                    if (r.intersects(box)) {
                        box.y += h;
                        overlap = true;
                        tries++;
                        break;
                    }
                }
                if (!overlap) break;
            }
            labelBoxes.add(box);
            g2.drawString(name, box.x, box.y + h - 3);

            /* --- Selection Highlight --- */
            if (i == selectedPlanet) {
                g2.setColor(Color.YELLOW);
                g2.draw(new Ellipse2D.Double(px - pr - 3, py - pr - 3, 2 * pr + 6, 2 * pr + 6));
            }

            /* --- Moons (only visible at high zoom) --- */
            if (zoom > 30 && MOON_NAMES[i].length > 0) {
                g2.setColor(Color.LIGHT_GRAY);
                for (int m = 0; m < MOON_NAMES[i].length; m++) {
                    double r = MOON_ORBIT_M[i][m];
                    double ang = moonAngle[i][m];
                    double mx = worldX + r * Math.cos(ang);
                    double my = worldY + r * Math.sin(ang);

                    double mpx = cx + (mx - camX) * METERS_TO_PIXELS * zoom;
                    double mpy = cy + (my - camY) * METERS_TO_PIXELS * zoom;
                    double mr = pr * MOON_SIZE_RATIO[i][m];
                    if (mr < 0.5) mr = 0.5;

                    g2.fill(new Ellipse2D.Double(mpx - mr, mpy - mr, 2 * mr, 2 * mr));

                    g2.setColor(Color.WHITE);
                    g2.drawString(MOON_NAMES[i][m], (int) (mpx + mr + 4), (int) (mpy - mr - 2));
                    g2.setColor(Color.LIGHT_GRAY);
                }
            }
        }

        /* ----- HUD (Top-left) ----- */
        g2.setColor(Color.WHITE);
        g2.drawString(String.format("Zoom: %.2fx", zoom), 10, 20);
        g2.drawString(String.format("Time ×%.0f (%.2f days/s)", timeScale, timeScale / DAY), 10, 40);
        g2.drawString(String.format("Sim: %.2f y", simTimeSec / (365.25 * DAY)), 10, 60);
        g2.drawString("Scroll=Zoom | Ctrl+Scroll=Time | Click=Follow", 10, 80);

        /* ----- Info Panel (Top-right) ----- */
        if (selectedPlanet != -1) {
            int i = selectedPlanet;
            int w = 260, h = 110, x0 = getWidth() - w - 20, y0 = 20;
            g2.setColor(new Color(0, 0, 0, 150));
            g2.fillRoundRect(x0, y0, w, h, 15, 15);
            g2.setColor(Color.WHITE);
            g2.drawRoundRect(x0, y0, w, h, 15, 15);
            g2.drawString("Planet: " + NAMES[i], x0 + 15, y0 + 25);
            g2.drawString(String.format("Radius: %.0f km", RADIUS_M[i] / 1_000), x0 + 15, y0 + 45);
            g2.drawString(String.format("Dist: %.3f AU", planetDistM[i] / AU), x0 + 15, y0 + 65);
            g2.drawString(String.format("Speed: %.2f km/s", planetSpeedKmS[i]), x0 + 15, y0 + 85);
            g2.drawString(String.format("Period: %.3f y", PERIOD_Y[i]), x0 + 15, y0 + 105);
        }
    }

    /* ==============================
       MAIN METHOD – Launch App
       ============================== */
    public static void main(String[] args) {
        JFrame frame = new JFrame("Solar System Simulation");
        frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        frame.add(new SolarSystemSimulation());
        frame.pack();
        frame.setLocationRelativeTo(null);
        frame.setVisible(true);
    }
}