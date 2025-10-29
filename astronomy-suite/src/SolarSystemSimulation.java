import javax.swing.*;
import java.awt.*;
import java.awt.event.*;
import java.awt.geom.Ellipse2D;
import java.awt.geom.Path2D;
import java.awt.geom.Point2D;
import java.util.ArrayList;

/**
 * Solar System Simulation – A real-time, interactive 2D orbital mechanics visualization. Using NASA J2000 Planet Data
 * Extended: All major moons added and their orbits modelled as Keplerian ellipses
 * around their parent planets (not assumed circular anymore).
 * <p>
 * Notes:
 *  - Moon orbital elements (a, e, period) are taken from the arrays in the file.
 *  - Kepler's equation (M = E - e sin E) is solved per moon using Newton-Raphson each frame.
 *  - Moon orbit paths are precomputed in the planet-centered frame and then translated
 *    to the planet's world position at render time.
 *  - This is a 2D in-plane simulation (no inclination projection). Small argument-of-periapsis
 *    offsets are applied to make orbits vary visually.
 *
 * @author Ethan Lin
 * @version 1.5
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
   CAMERA SYSTEM (enhanced)
   ============================== */
    private int selectedPlanet = -1;
    private boolean followingPlanet = false;
    private int     cameraFocusIndex = -1;
    private double  camX = 0, camY = 0;

    private boolean cameraIsApproaching = false;
    private static final double ARRIVAL_THRESHOLD = 1e7;

    // === AUTO-ZOOM TOGGLE ===
    private boolean autoZoomEnabled = false;      // Z key toggles
    private double targetZoom = 1.0;              // Desired when auto-zoom ON
    private double currentZoom = 1.0;             // Smoothly interpolated
    private static final double ZOOM_SPEED = 10.0;

    // === ORBIT VISIBILITY ===
    private static final double ORBIT_VISIBLE_ZOOM = 5.0;   // moons orbits appear
    private static final double PLANET_VISIBLE_ZOOM = 0.5;  // planet orbits appear

    /* ==============================
       SIMULATION STATE
       ============================== */
    private double timeScale  = 1_000;            // Simulation speed: sim-seconds per real-second
    private double simTimeSec = 0;               // Total elapsed simulation time in seconds
    private long   lastNanos;                    // Timestamp of last frame (for delta time)

    /* ==============================
       PLANET DATA (8 major planets + Pluto)
       ============================== */
    private static final String[] NAMES = {
            "Mercury", "Venus", "Earth", "Mars",
            "Jupiter", "Saturn", "Uranus", "Neptune", "Pluto"
    };

    // Planets: JPL Approximate Positions (DE430, J2000 ecliptic, valid 1800-2050)
    private static final double[] A_METERS = {
            5.7909e10,   // Mercury: 0.387098 AU
            1.0821e11,   // Venus:   0.723332 AU
            1.495978707e11, // Earth:  1.000000 AU (exact)
            2.2794e11,   // Mars:    1.523662 AU
            7.7854e11,   // Jupiter: 5.202603 AU
            1.4298e12,   // Saturn:  9.554747 AU
            2.8725e12,   // Uranus: 19.21814 AU
            4.5045e12,   // Neptune: 30.110387 AU
            5.9064e12    // Pluto:  39.482116 AU
    };

    private static final double[] ECC = {
            0.205630,  // Mercury
            0.006772,  // Venus
            0.016709,  // Earth
            0.093405,  // Mars
            0.048498,  // Jupiter
            0.055546,  // Saturn
            0.046381,  // Uranus
            0.008956,  // Neptune
            0.248827   // Pluto
    };

    private static final double[] PERIOD_Y = {
            0.2408467, // Mercury
            0.61519726, // Venus
            1.0000174, // Earth
            1.8808476, // Mars
            11.862615, // Jupiter
            29.447498, // Saturn
            84.016846, // Uranus
            164.79132, // Neptune
            247.92065  // Pluto
    };

    // Planet radius in metres
    private static final double[] RADIUS_M = {
            2.439e6, 6.052e6, 6.371e6, 3.390e6,
            6.9911e7, 5.8232e7, 2.5362e7, 2.4622e7,
            1.1883e6
    };

    // JPL Planet Elements at J2000 (deg; for computing M0)
    private static final double[] PLANET_L0 = { // Mean longitude (deg)
            252.25084, 181.97973, 100.46435, 355.45332, 34.40438, 49.94432, 313.23218, 304.88003, 238.92881
    };
    private static final double[] PLANET_VARPI0 = { // Longitude of perihelion (deg)
            77.45736, 131.60261, 102.93735, 336.04084, 14.75385, 92.43194, 170.96424, 44.97135, 113.76329
    };
    private static final double[] PLANET_OMEGA0 = { // Longitude of ascending node (deg)
            48.33167, 76.67992, 0.0, 49.57854, 100.55615, 113.71504, 74.22950, 131.72169, 110.30347
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
       Each moon: semi-major axis (m), period (days), size ratio, name, eccentricity, arg of peri (rad)
       ============================== */
    // Moons: NASA/JPL elements at J2000 (a in m, e, period in days, ω in rad)
    private static final double[][] MOON_A_M = {
            {}, {}, {3.844e8}, // Earth: Moon a=384400 km
            {9.377e6, 2.356e7}, // Mars: Phobos 9377 km, Deimos 23460 km
            {4.217e8, 6.71e8, 1.07e9, 1.883e9}, // Jupiter: Io 421700, Europa 671000, Ganymede 1070000, Callisto 1883000 km
            {1.222e9, 2.38e8, 5.27e8}, // Saturn: Titan 1221870 km, Enceladus 238000, Rhea 527000 km (partial; full set would be longer)
            {4.363e8, 5.835e8, 4.989e8}, // Uranus: Titania 436300, Oberon 583520, Umbriel 266000 km (adjusted to major)
            {3.547e8}, // Neptune: Triton 354759 km
            {} // Pluto: none major
    };

    private static final double[][] MOON_PERIOD_DAYS = {
            {}, {}, {27.321661}, // Moon
            {0.318910, 1.26244}, // Phobos, Deimos
            {1.769278, 3.551181, 7.154552, 16.689018}, // Io, Europa, Ganymede, Callisto
            {15.94542, 1.370218, 4.518212}, // Titan, Enceladus, Rhea
            {8.706234, 13.46339, 4.144176}, // Titania, Oberon, Umbriel
            {5.876854}, {} // Triton
    };

    private static final double[][] MOON_SIZE_RATIO = {
            {}, {}, {0.273},
            {0.11, 0.08},
            {0.286, 0.245, 0.413, 0.378},
            {0.404, 0.036, 0.152},
            {0.196, 0.182, 0.173},
            {0.212}, {}
    };

    private static final String[][] MOON_NAMES = {
            {}, {}, {"Moon"},
            {"Phobos", "Deimos"},
            {"Io", "Europa", "Ganymede", "Callisto"},
            {"Titan", "Enceladus", "Rhea"},
            {"Titania", "Oberon", "Umbriel"},
            {"Triton"}, {}
    };

    private static final double[][] MOON_ECC = {
            {}, {}, {0.05490}, // Moon
            {0.01550, 0.00021}, // Phobos, Deimos
            {0.00412, 0.00899, 0.00126, 0.00716}, // Io, Europa, Ganymede, Callisto
            {0.02880, 0.00470, 0.00100}, // Titan, Enceladus, Rhea
            {0.00110, 0.00090, 0.00390}, // Titania, Oberon, Umbriel
            {0.000016} // Triton (nearly circular)
    };

    private static final double[][] MOON_ARG_PERI = { // Argument of periapsis (rad) at J2000
            {}, {}, {2.034}, // Moon: ~116.6°
            {Math.PI/2, Math.PI}, // Phobos/Deimos: approximate equatorial
            {1.88, 0.85, 1.02, 5.92}, // Io ~107.7°, Europa ~48.7°, Ganymede ~58.6°, Callisto ~339.3°
            {0.20, 2.29, 5.80}, // Titan ~11.3°, Enceladus ~131.3°, Rhea ~332.5°
            {3.76, 0.44, 4.72}, // Titania ~215.5°, Oberon ~25.3°, Umbriel ~270.4°
            {Math.PI} // Triton: retrograde, ~180°
    };


    /* ==============================
       ORBITAL STATE VARIABLES
       ============================== */
    private final double[] meanAnomaly = new double[NAMES.length];     // M: mean anomaly (radians) for planets
    private final double[][] moonMean = new double[NAMES.length][];    // mean anomaly per moon

    private static final int ORBIT_RESOLUTION = 2000; // resolution for precomputed orbit paths
    private final ArrayList<ArrayList<Point2D.Double>> orbitPaths = new ArrayList<>();
    private final ArrayList<ArrayList<ArrayList<Point2D.Double>>> moonOrbitPaths = new ArrayList<>();

    // Current computed positions and velocities
    private final double[] planetX = new double[NAMES.length];        // x-position in metres
    private final double[] planetY = new double[NAMES.length];        // y-position in metres
    private final double[] planetDistM = new double[NAMES.length];    // distance from Sun
    private final double[] planetSpeedKmS = new double[NAMES.length]; // orbital speed (km/s)

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

        // Compute precise initial mean anomalies M0 = L0 - ϖ0 (rad) from JPL elements
        for (int i = 0; i < NAMES.length; i++) {
            double L_deg = PLANET_L0[i];
            double varpi_deg = PLANET_VARPI0[i];
            double M_deg = L_deg - varpi_deg;
            meanAnomaly[i] = Math.toRadians(M_deg % 360.0); // Normalize to [0, 2π)

            // Compute initial position for verification (optional: store in planetX/Y if desired)
            double omega_rad = Math.toRadians(PLANET_VARPI0[i] - PLANET_OMEGA0[i]); // arg peri = ϖ - Ω
            double Omega_rad = Math.toRadians(PLANET_OMEGA0[i]);
            Point2D.Double pos = computeInitialPosition(A_METERS[i], ECC[i], omega_rad, Omega_rad, meanAnomaly[i]);
            planetX[i] = pos.x;
            planetY[i] = pos.y; // Set initial positions
            planetDistM[i] = Math.hypot(pos.x, pos.y);
        }

        // Moons: Set initial M0 = 0 at J2000 (fast orbits; precise phase from HORIZONS if needed)
        for (int i = 0; i < NAMES.length; i++) {
            int numMoons = MOON_NAMES[i].length;
            moonMean[i] = new double[numMoons];
            for (int m = 0; m < numMoons; m++) {
                moonMean[i][m] = 0.0; // M0 = 0 rad at J2000
            }
        }

        // Precompute planet orbit paths
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

        // Precompute moon orbit paths (planet-centered)
        for (int i = 0; i < NAMES.length; i++) {
            ArrayList<ArrayList<Point2D.Double>> moonOrbitsForPlanet = new ArrayList<>();
            for (int m = 0; m < MOON_A_M[i].length; m++) {
                ArrayList<Point2D.Double> orbit = new ArrayList<>();
                double a = MOON_A_M[i][m];
                double e = MOON_ECC[i][m];
                double b = a * Math.sqrt(1 - e * e);
                double arg = MOON_ARG_PERI[i][m];
                for (int j = 0; j <= ORBIT_RESOLUTION; j++) {
                    double E = 2 * Math.PI * j / ORBIT_RESOLUTION;
                    double x = a * (Math.cos(E) - e);
                    double y = b * Math.sin(E);
                    // rotate by arg of periapsis
                    double rx = x * Math.cos(arg) - y * Math.sin(arg);
                    double ry = x * Math.sin(arg) + y * Math.cos(arg);
                    orbit.add(new Point2D.Double(rx, ry));
                }
                moonOrbitsForPlanet.add(orbit);
            }
            moonOrbitPaths.add(moonOrbitsForPlanet);
        }

        setFocusable(true);
        addKeyListener(new KeyAdapter() {
            @Override
            public void keyPressed(KeyEvent e) {
                if (e.getKeyCode() == KeyEvent.VK_Z) {
                    autoZoomEnabled = !autoZoomEnabled;
                    if (!autoZoomEnabled) {
                        // When turning off, keep current as target
                        targetZoom = currentZoom;
                    }
                }
            }
        });

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

        /* ----- Update Moon Mean Anomalies ----- */
        for (int i = 0; i < NAMES.length; i++) {
            for (int m = 0; m < moonMean[i].length; m++) {
                double periodSec = MOON_PERIOD_DAYS[i][m] * DAY;
                double n = 2 * Math.PI / periodSec;
                moonMean[i][m] = (moonMean[i][m] + n * dtSim) % (2 * Math.PI);
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

        /* ----- Camera Movement & Smart Zoom ----- */
        if (followingPlanet && cameraFocusIndex >= 0) {
            double targetX = planetX[cameraFocusIndex];
            double targetY = planetY[cameraFocusIndex];

            double dx = targetX - camX;
            double dy = targetY - camY;
            double dist = Math.hypot(dx, dy);

            // --- POSITION: smooth → snap lock ---
            if (cameraIsApproaching) {
                if (dist <= ARRIVAL_THRESHOLD) {
                    camX = targetX;
                    camY = targetY;
                    cameraIsApproaching = false;
                } else {
                    double speed = 12.0;
                    double lerp = 1 - Math.exp(-speed * dtReal);
                    camX += dx * lerp;
                    camY += dy * lerp;
                }
            } else {
                camX = targetX;
                camY = targetY;
            }

            // --- AUTO-ZOOM: only if enabled ---
            if (autoZoomEnabled) {
                double maxRadius = RADIUS_M[cameraFocusIndex];
                for (int m = 0; m < MOON_A_M[cameraFocusIndex].length; m++) {
                    double moonDist = MOON_A_M[cameraFocusIndex][m] * 1.1;
                    if (moonDist > maxRadius) maxRadius = moonDist;
                }

                int screenSize = Math.min(getWidth(), getHeight());
                double desiredPixels = screenSize * 0.6;
                targetZoom = desiredPixels / (maxRadius * METERS_TO_PIXELS * 2);
                targetZoom = Math.max(5.0, Math.min(targetZoom, 5000.0));
            }
            // else: targetZoom stays at last manual value

        } else {
            // Not following
            double speed = 8.0;
            double lerp = 1 - Math.exp(-speed * dtReal);
            camX *= (1 - lerp);
            camY *= (1 - lerp);

            targetZoom = 1.0;
            cameraIsApproaching = false;
            autoZoomEnabled = false; // optional: reset on unfollow
        }

        // --- Smooth zoom interpolation (only if autoZoom on) ---
        if (autoZoomEnabled) {
            double zoomLerp = 1 - Math.exp(-ZOOM_SPEED * dtReal);
            currentZoom += (targetZoom - currentZoom) * zoomLerp;
        }
        repaint();  // Trigger redraw
    }

    /* ==============================
       INPUT: Mouse Wheel (Zoom & Time)
       ============================== */
    @Override
    public void mouseWheelMoved(MouseWheelEvent e) {
        if (e.isControlDown()) {
            timeScale = e.getWheelRotation() < 0 ? timeScale * 2 : timeScale / 2;
            timeScale = Math.max(1, Math.min(timeScale, 1e9));
        } else {
            double factor = e.getWheelRotation() < 0 ? 1.1 : 1/1.1;
            currentZoom *= factor;
            currentZoom = Math.max(0.1, Math.min(currentZoom, 1000));

            // When manually zooming, update target so auto-zoom resumes from here
            targetZoom = currentZoom;
        }
    }

    /* ==============================
       INPUT: Mouse Click (Planet Selection)
       ============================== */
    private void handleClick(int mx, int my) {
        int cx = getWidth() / 2, cy = getHeight() / 2;

        for (int i = 0; i < NAMES.length; i++) {
            // Convert planet world position to screen coordinates
            double px = cx + (planetX[i] - camX) * METERS_TO_PIXELS * currentZoom;
            double py = cy + (planetY[i] - camY) * METERS_TO_PIXELS * currentZoom;
            double pr = Math.max(RADIUS_M[i] * METERS_TO_PIXELS * currentZoom, 4);  // Min size

            // Click detection with 2x radius tolerance
            double dx = mx - px, dy = my - py;
            if (dx * dx + dy * dy < pr * pr * 4) {
                if (cameraFocusIndex == i && followingPlanet) {
                    // Click again to unfollow
                    followingPlanet = false;
                    cameraFocusIndex = -1;
                    selectedPlanet = -1;
                    cameraIsApproaching = false;
                } else {
                    // Follow this planet
                    followingPlanet = true;
                    cameraIsApproaching = true;
                    cameraFocusIndex = i;
                    selectedPlanet = i;
                }
                return;
            }
        }
        // Clicked empty space → unfollow
        followingPlanet = false;
        currentZoom = Math.min(currentZoom, 50);
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
        double sunR = SUN_RADIUS_M * METERS_TO_PIXELS * currentZoom;
        double sunX = cx - sunR - camX * METERS_TO_PIXELS * currentZoom;
        double sunY = cy - sunR - camY * METERS_TO_PIXELS * currentZoom;
        g2.setColor(Color.ORANGE);
        g2.fill(new Ellipse2D.Double(sunX, sunY, 2 * sunR, 2 * sunR));

        FontMetrics fm = g2.getFontMetrics();

        /* ----- Draw Each Planet & Orbit ----- */
        for (int i = 0; i < NAMES.length; i++) {
            double worldX = planetX[i], worldY = planetY[i];
            double px = cx + (worldX - camX) * METERS_TO_PIXELS * currentZoom;
            double py = cy + (worldY - camY) * METERS_TO_PIXELS * currentZoom;
            double pr = RADIUS_M[i] * METERS_TO_PIXELS * currentZoom;
            if (pr < 0.5) pr = 0.5;  // Minimum visible size

            /* --- Precomputed Orbit Path (smooth Path2D) --- */
            Path2D.Double path = getADouble(i, cx, cy);

            // --- Planet orbit (only if zoomed out enough) ---
            if (currentZoom < PLANET_VISIBLE_ZOOM * 10) {  // show at overview
                g2.setColor(new Color (74, 74, 74));
            } else {
                g2.setColor(Color.DARK_GRAY);
            }
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

            /* --- Moons: draw moon orbits (planet-centered) and moon bodies --- */
            if (MOON_NAMES[i].length > 0) {
                // Draw moon orbits around the planet (translated)
                ArrayList<ArrayList<Point2D.Double>> moonOrbits = moonOrbitPaths.get(i);
                for (ArrayList<Point2D.Double> morb : moonOrbits) {
                    Path2D.Double mpath = new Path2D.Double();
                    Point2D.Double first = morb.getFirst();
                    double msx = cx + ((worldX + first.x) - camX) * METERS_TO_PIXELS * currentZoom;
                    double msy = cy + ((worldY + first.y) - camY) * METERS_TO_PIXELS * currentZoom;
                    mpath.moveTo(msx, msy);
                    for (int j = 1; j < morb.size(); j++) {
                        Point2D.Double p = morb.get(j);
                        double x = cx + ((worldX + p.x) - camX) * METERS_TO_PIXELS * currentZoom;
                        double y = cy + ((worldY + p.y) - camY) * METERS_TO_PIXELS * currentZoom;
                        mpath.lineTo(x, y);
                    }
                    mpath.closePath();
                    // --- Moon orbits (only when zoomed in) ---
                    if (currentZoom > ORBIT_VISIBLE_ZOOM && MOON_NAMES[i].length > 0) {
                        g2.setColor(new Color(120, 120, 120, 100));
                        g2.setStroke(new BasicStroke(0.8f));
                        g2.draw(mpath);
                    }
                }

                // Draw moon bodies (visible at reasonable zoom)
                if (currentZoom > 10) {
                    for (int m = 0; m < MOON_A_M[i].length; m++) {
                        double a = MOON_A_M[i][m];
                        double e = MOON_ECC[i][m];
                        double b = a * Math.sqrt(1 - e * e);
                        double M = moonMean[i][m];

                        // Solve Kepler for moon
                        double E = M;
                        for (int it = 0; it < 40; it++) {
                            double f  = E - e * Math.sin(E) - M;
                            double fp = 1 - e * Math.cos(E);
                            double d  = f / fp;
                            E -= d;
                            if (Math.abs(d) < 1e-12) break;
                        }
                        double mx = a * (Math.cos(E) - e);
                        double my = b * Math.sin(E);
                        // rotate by arg of periapsis
                        double arg = MOON_ARG_PERI[i][m];
                        double rx = mx * Math.cos(arg) - my * Math.sin(arg);
                        double ry = mx * Math.sin(arg) + my * Math.cos(arg);

                        // world coords = planet world + moon planet-centered coords
                        double moonWorldX = planetX[i] + rx;
                        double moonWorldY = planetY[i] + ry;

                        double mpx = cx + (moonWorldX - camX) * METERS_TO_PIXELS * currentZoom;
                        double mpy = cy + (moonWorldY - camY) * METERS_TO_PIXELS * currentZoom;
                        double mr = Math.max(RADIUS_M[i] * MOON_SIZE_RATIO[i][m] * METERS_TO_PIXELS * currentZoom, 0.6);

                        g2.setColor(Color.LIGHT_GRAY);
                        g2.fill(new Ellipse2D.Double(mpx - mr, mpy - mr, 2 * mr, 2 * mr));
                        g2.setColor(Color.WHITE);
                        g2.drawString(MOON_NAMES[i][m], (int) (mpx + mr + 4), (int) (mpy - mr - 2));
                    }
                }
            }
        }

        /* ----- HUD (Top-left) ----- */
        g2.setColor(Color.WHITE);
        g2.drawString(String.format("Zoom: %.2fx %s", currentZoom,
                followingPlanet&&autoZoomEnabled ? "(Auto)" : "(Manual)"), 10, 20);
        g2.drawString(String.format("Time ×%.0f (%.1f days/s)", timeScale, timeScale/DAY), 10, 40);
        g2.drawString(String.format("Sim: %.2f years", simTimeSec/(365.25*DAY)), 10, 60);

        if (followingPlanet && cameraFocusIndex >= 0) {
            g2.drawString(String.format("Following: %s", NAMES[cameraFocusIndex]), 10, 85);
            g2.drawString(String.format("Moons: %d | Target Zoom: %.1fx",
                    MOON_NAMES[cameraFocusIndex].length, targetZoom), 10, 105);
            g2.drawString("Z=Toggle Auto-Zoom When Selecting Planet", 10, 125);

        } else {
            g2.drawString("Scroll=Zoom | Ctrl+Scroll=Speed | Click=Follow", 10, 85);
        }

        /* ----- Info Panel (Top-right) ----- */
        if (selectedPlanet != -1) {
            int i = selectedPlanet;
            int w = 300, h = 140, x0 = getWidth() - w - 20, y0 = 20;
            g2.setColor(new Color(0, 0, 0, 150));
            g2.fillRoundRect(x0, y0, w, h, 15, 15);
            g2.setColor(Color.WHITE);
            g2.drawRoundRect(x0, y0, w, h, 15, 15);
            g2.drawString("Planet: " + NAMES[i], x0 + 15, y0 + 25);
            g2.drawString(String.format("Radius: %.0f km", RADIUS_M[i] / 1_000), x0 + 15, y0 + 45);
            g2.drawString(String.format("Dist: %.3f AU", planetDistM[i] / AU), x0 + 15, y0 + 65);
            g2.drawString(String.format("Speed: %.2f km/s", planetSpeedKmS[i]), x0 + 15, y0 + 85);
            g2.drawString(String.format("Period: %.3f y", PERIOD_Y[i]), x0 + 15, y0 + 105);
            if (MOON_NAMES[i].length > 0) {
                g2.drawString(String.format("Moons: %d", MOON_NAMES[i].length), x0 + 15, y0 + 125);
            }
        }
    }

    private Path2D.Double getADouble(int i, int cx, int cy) {
        ArrayList<Point2D.Double> orbit = orbitPaths.get(i);
        Path2D.Double path = new Path2D.Double();

        if (!orbit.isEmpty()) {
            Point2D.Double first = orbit.getFirst();
            double sx = cx + (first.x - camX) * METERS_TO_PIXELS * currentZoom;
            double sy = cy + (first.y - camY) * METERS_TO_PIXELS * currentZoom;
            path.moveTo(sx, sy);

            for (int j = 1; j < orbit.size(); j++) {
                Point2D.Double p = orbit.get(j);
                double x = cx + (p.x - camX) * METERS_TO_PIXELS * currentZoom;
                double y = cy + (p.y - camY) * METERS_TO_PIXELS * currentZoom;
                path.lineTo(x, y);
            }
            path.closePath();
        }
        return path;
    }

    /**
     * Compute initial position (x,y in meters, ecliptic J2000) from JPL Keplerian elements at epoch T=0 (J2000).
     * @param a semi-major axis (m)
     * @param e eccentricity
     * @param omega argument of periapsis (rad)
     * @param Omega longitude of ascending node (rad)
     * @param M mean anomaly at epoch (rad)
     * @return Point2D.Double (x, y)
     */
    private Point2D.Double computeInitialPosition(double a, double e, double omega, double Omega, double M) {
        // Solve Kepler's equation for eccentric anomaly E
        double E = M;
        for (int it = 0; it < 40; it++) {
            double f = E - e * Math.sin(E) - M;
            double fp = 1 - e * Math.cos(E);
            E -= f / fp;
            if (Math.abs(f) < 1e-12) break;
        }

        // Position in orbital plane
        double xr = a * (Math.cos(E) - e);
        double yr = a * Math.sqrt(1 - e * e) * Math.sin(E);

        // Rotate to ecliptic J2000 (2D: ignore inclination for in-plane sim)
        double x = xr * Math.cos(omega + Omega) - yr * Math.sin(omega + Omega);
        double y = xr * Math.sin(omega + Omega) + yr * Math.cos(omega + Omega);

        return new Point2D.Double(x, y);
    }

    /* ==============================
       MAIN METHOD – Launch App
       ============================== */
    public static void main(String[] args) {
        JFrame frame = new JFrame("Solar System Simulation (moons)");
        frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        frame.add(new SolarSystemSimulation());
        frame.pack();
        frame.setLocationRelativeTo(null);
        frame.setVisible(true);
    }
}
