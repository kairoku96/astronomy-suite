import javax.swing.*;
import java.awt.*;
import java.awt.event.*;
import java.awt.geom.AffineTransform;
import java.awt.geom.Ellipse2D;
import java.awt.geom.Path2D;
import java.awt.geom.Point2D;
import java.util.ArrayList;

/**
 * Solar System Simulation – real-time 2-D orbital mechanics with textures,
 * smooth camera, moon-following and auto-zoom.
 *
 * <p>This program simulates the Sun and 9 major celestial bodies (including Pluto)
 * using Keplerian orbital elements. It supports:
 * <ul>
 *   <li>Real-time Newtonian physics via Kepler's equation</li>
 *   <li>Interactive camera: click to follow planets/moons</li>
 *   <li>Smooth panning and exponential zoom interpolation</li>
 *   <li>Auto-zoom when following bodies</li>
 *   <li>Moon systems with independent orbits</li>
 *   <li>Orbit path rendering and dynamic HUD</li>
 * </ul>
 *
 * N-body mode (toggle with 'N') replaces the Kepler update with a velocity-Verlet
 * integrator that accounts for mutual gravitational attraction between planets
 * and the central Sun (Sun is fixed at origin).
 *
 * @author Ethan Lin (original) + N-body integration by ChatGPT
 * @version 3.0
 */
public class SolarSystemSimulation extends JPanel
        implements ActionListener, MouseWheelListener, MouseListener {

    /* ==============================
       PHYSICAL CONSTANTS (SI UNITS)
       ============================== */
    /** 1 Astronomical Unit in meters */
    private static final double AU  = 1.496e11;
    /** Seconds in one Earth day */
    private static final double DAY = 86_400.0;
    /** Gravitational constant (m³ kg⁻¹ s⁻²) */
    private static final double G   = 6.67430e-11;
    /** Mass of the Sun (kg) */
    private static final double M_SUN = 1.98847e30;

    /** Conversion factor: meters → screen pixels at base zoom (250 px per AU) */
    private static final double METERS_TO_PIXELS = 250.0 / AU;

    /* ==============================
       CAMERA SYSTEM
       ============================== */
    /** Index of currently selected planet (-1 = none) */
    private int selectedPlanet = -1;
    /** Whether camera is locked to a planet */
    private boolean followingPlanet = false;
    /** Index of planet currently tracked by camera */
    private int     cameraFocusIndex = -1;
    /** Current camera center in world coordinates (meters) */
    private double  camX = 0, camY = 0;

    /** True if camera is still moving toward target (smooth approach) */
    private boolean cameraIsApproaching = false;
    /** Distance threshold (m) to stop smoothing camera motion */
    private static final double ARRIVAL_THRESHOLD = 1e7;

    // ----- Moon selection -----
    /** Index of selected moon on current planet */
    private int selectedMoon = -1;
    /** Index of moon being tracked by camera */
    private int   cameraFocusMoon = -1;
    /** Camera center when following a moon */
    private double moonCamX = 0, moonCamY = 0;
    /** Whether camera is locked to a moon */
    private boolean followingMoon = false;
    /** True if moon camera is approaching target */
    private boolean moonCameraIsApproaching = false;

    // ----- Auto-zoom -----
    /** Toggle: auto-adjust zoom when following bodies */
    private boolean autoZoomEnabled = false;
    /** Target zoom level (auto-computed) */
    private double targetZoom = 1.0;
    /** Current zoom level (smoothly interpolated) */
    private double currentZoom = 1.0;
    /** Speed of zoom interpolation (higher = faster) */
    private static final double ZOOM_SPEED = 10.0;
    private int zoomMode = 0;                   // 0=Planet, 1=Orbit, 2=Manual
    private static final String[] ZOOM_MODE_NAMES = {"Planet", "Orbit", "Manual"};

    // ----- Orbit visibility thresholds -----
    /** Zoom level at which orbits become visible */
    private static final double ORBIT_VISIBLE_ZOOM = 5.0;
    /** Zoom level at which planets become visible */
    private static final double PLANET_VISIBLE_ZOOM = 0.5;

    /* ==============================
       SIMULATION STATE
       ============================== */
    /** Time acceleration factor (1 = real-time, 1000 = 1000× faster) */
    private double timeScale  = 1_000;
    /** Total simulated time elapsed (seconds) */
    private double simTimeSec = 0;
    /** Timestamp of last frame (nanoseconds) */
    private long   lastNanos;

    /* ==============================
       PLANET DATA (Keplerian Elements)
       ============================== */
    /** Names of celestial bodies */
    private static final String[] NAMES = {
            "Mercury", "Venus", "Earth", "Mars",
            "Jupiter", "Saturn", "Uranus", "Neptune", "Pluto"
    };

    /** Semi-major axis (m) */
    private static final double[] A_METERS = {
            5.7909e10, 1.0821e11, 1.495978707e11, 2.2794e11,
            7.7854e11, 1.4298e12, 2.8725e12, 4.5045e12, 5.9064e12
    };

    /** Orbital eccentricity */
    private static final double[] ECC = {
            0.205630, 0.006772, 0.016709, 0.093405,
            0.048498, 0.055546, 0.046381, 0.008956, 0.248827
    };

    /** Orbital period (Earth years) */
    private static final double[] PERIOD_Y = {
            0.2408467, 0.61519726, 1.0000174, 1.8808476,
            11.862615, 29.447498, 84.016846, 164.79132, 247.92065
    };

    /** Physical radius (m) */
    private static final double[] RADIUS_M = {
            2.439e6, 6.052e6, 6.371e6, 3.390e6,
            6.9911e7, 5.8232e7, 2.5362e7, 2.4622e7, 1.1883e6
    };

    /** Mean longitude at J2000 (degrees) */
    private static final double[] PLANET_L0 = {
            252.25084, 181.97973, 100.46435, 355.45332, 34.40438,
            49.94432, 313.23218, 304.88003, 238.92881
    };

    /** Longitude of perihelion at J2000 (degrees) */
    private static final double[] PLANET_VARPI0 = {
            77.45736, 131.60261, 102.93735, 336.04084, 14.75385,
            92.43194, 170.96424, 44.97135, 113.76329
    };

    /** Longitude of ascending node at J2000 (degrees) */
    private static final double[] PLANET_OMEGA0 = {
            48.33167, 76.67992, 0.0, 49.57854, 100.55615,
            113.71504, 74.22950, 131.72169, 110.30347
    };

    /** Sun's radius (m) */
    private static final double SUN_RADIUS_M = 6.9634e8;

    /** Visual color for each planet */
    private static final Color[] COLORS = {
            new Color(169, 169, 169),   // Mercury – dark gray, rocky
            new Color(210, 180, 140),   // Venus – pale tan/yellowish beige
            new Color(81, 150, 208),    // Earth – bluish with some green tones
            new Color(188, 39, 50),     // Mars – reddish ochre
            new Color(255, 204, 102),   // Jupiter – light orange-brown (cloud bands)
            new Color(194, 178, 128),   // Saturn – muted golden tan
            new Color(173, 216, 230),   // Uranus – light cyan with a hint of green
            new Color(72, 118, 255),    // Neptune – deep azure blue
            new Color(233, 226, 214)    // Pluto – pale off-white/icy beige
    };

    /* ==============================
       MOON DATA
       ============================== */
    /** Semi-major axis of moons relative to parent planet (m) */
    private static final double[][] MOON_A_M = {
            {}, {}, {3.844e8},
            {9.377e6, 2.356e7},
            {4.217e8, 6.71e8, 1.07e9, 1.883e9},
            {1.222e9, 2.38e8, 5.27e8},
            {4.363e8, 5.835e8, 4.989e8},
            {3.547e8}, {}
    };

    /** Moon orbital period (Earth days) */
    private static final double[][] MOON_PERIOD_DAYS = {
            {}, {}, {27.321661},
            {0.318910, 1.26244},
            {1.769278, 3.551181, 7.154552, 16.689018},
            {15.94542, 1.370218, 4.518212},
            {8.706234, 13.46339, 4.144176},
            {5.876854}, {}
    };

    /** Moon radius as ratio of parent planet radius */
    private static final double[][] MOON_SIZE_RATIO = {
            {}, {}, {0.273},
            {0.11, 0.08},
            {0.286, 0.245, 0.413, 0.378},
            {0.404, 0.036, 0.152},
            {0.196, 0.182, 0.173},
            {0.212}, {}
    };

    /** Names of moons per planet */
    private static final String[][] MOON_NAMES = {
            {}, {}, {"Moon"},
            {"Phobos", "Deimos"},
            {"Io", "Europa", "Ganymede", "Callisto"},
            {"Titan", "Enceladus", "Rhea"},
            {"Titania", "Oberon", "Umbriel"},
            {"Triton"}, {}
    };

    /** Moon orbital eccentricity */
    private static final double[][] MOON_ECC = {
            {}, {}, {0.05490},
            {0.01550, 0.00021},
            {0.00412, 0.00899, 0.00126, 0.00716},
            {0.02880, 0.00470, 0.00100},
            {0.00110, 0.00090, 0.00390},
            {0.000016}, {}
    };

    /** Argument of periapsis for moons (radians) */
    private static final double[][] MOON_ARG_PERI = {
            {}, {}, {2.034},
            {Math.PI/2, Math.PI},
            {1.88, 0.85, 1.02, 5.92},
            {0.20, 2.29, 5.80},
            {3.76, 0.44, 4.72},
            {Math.PI}, {}
    };

    /** Computed moon radii (planet radius × ratio) */
    private static final double[][] MOON_RADIUS_M = new double[NAMES.length][];

    /* Static initializer: compute actual moon radii */
    static {
        for (int i = 0; i < NAMES.length; i++) {
            int n = MOON_SIZE_RATIO[i].length;
            MOON_RADIUS_M[i] = new double[n];
            for (int m = 0; m < n; m++) {
                MOON_RADIUS_M[i][m] = RADIUS_M[i] * MOON_SIZE_RATIO[i][m];
            }
        }
    }

    /* ==============================
       ORBITAL STATE
       ============================== */
    /** Current mean anomaly for each planet (radians) */
    private final double[] meanAnomaly = new double[NAMES.length];
    /** Mean anomaly for each moon (per planet) */
    private final double[][] moonMean = new double[NAMES.length][];

    /** Number of points to use when drawing orbits */
    private static final int ORBIT_RESOLUTION = 10000;

    /** Precomputed orbit paths for planets (in planet-local coords) */
    private final ArrayList<ArrayList<Point2D.Double>> orbitPaths = new ArrayList<>();
    /** Precomputed moon orbit paths (relative to parent planet) */
    private final ArrayList<ArrayList<ArrayList<Point2D.Double>>> moonOrbitPaths = new ArrayList<>();

    /** Current world position of planets (m) */
    private final double[] planetX = new double[NAMES.length];
    private final double[] planetY = new double[NAMES.length];
    /** Distance from Sun (m) */
    private final double[] planetDistM = new double[NAMES.length];
    /** Orbital speed (km/s) */
    private final double[] planetSpeedKmS = new double[NAMES.length];

    /* ==============================
       N-BODY EXTENSIONS
       ============================== */
    /** Whether N-body mode is active (toggle with 'N') */
    private boolean nBodyEnabled = false;

    /** Planet masses in kg (realistic values) */
    private final double[] bodyMass = new double[] {
            3.3011e23,   // Mercury
            4.8675e24,   // Venus
            5.97237e24,  // Earth
            6.4171e23,   // Mars
            1.8982e27,   // Jupiter
            5.6834e26,   // Saturn
            8.6810e25,   // Uranus
            1.02413e26,  // Neptune
            1.303e22     // Pluto
    };

    /** Velocities (m/s) */
    private final double[] vx = new double[NAMES.length];
    private final double[] vy = new double[NAMES.length];

    /** Accelerations (m/s^2) - reused per-step */
    private final double[] ax = new double[NAMES.length];
    private final double[] ay = new double[NAMES.length];

    /** Alive flags (false after merged) */
    private final boolean[] alive = new boolean[NAMES.length];

    /* ==============================
   SHADING SYSTEM
   ============================== */
    private boolean shadingEnabled = true;

    /* ==============================
       CONSTRUCTOR
       ============================== */
    /**
     * Initializes the simulation panel, sets up UI, precomputes orbits,
     * and starts the animation timer.
     */
    public SolarSystemSimulation() {
        setBackground(Color.BLACK);
        setPreferredSize(new Dimension(1200, 800));
        addMouseWheelListener(this);
        addMouseListener(this);

        // Initialize planet positions at J2000 epoch
        for (int i = 0; i < NAMES.length; i++) {
            double L_deg = PLANET_L0[i];
            double varpi_deg = PLANET_VARPI0[i];
            double M_deg = L_deg - varpi_deg;  // Mean anomaly at epoch
            meanAnomaly[i] = Math.toRadians(M_deg % 360.0);

            double omega_rad = Math.toRadians(PLANET_VARPI0[i] - PLANET_OMEGA0[i]); // argument of perihelion
            double Omega_rad = Math.toRadians(PLANET_OMEGA0[i]);                    // longitude of node
            Point2D.Double pos = computeInitialPosition(A_METERS[i], ECC[i], omega_rad, Omega_rad, meanAnomaly[i]);
            planetX[i] = pos.x;
            planetY[i] = pos.y;
            planetDistM[i] = Math.hypot(pos.x, pos.y);

            // Mark alive
            alive[i] = true;
        }

        // Initialize moon mean anomaly arrays
        for (int i = 0; i < NAMES.length; i++) {
            int num = MOON_NAMES[i].length;
            moonMean[i] = new double[num];
        }

        // Pre-compute planet orbit paths (elliptical in planet-local frame)
        for (int i = 0; i < NAMES.length; i++) {
            ArrayList<Point2D.Double> orbit = new ArrayList<>();
            double a = A_METERS[i], e = ECC[i], b = a * Math.sqrt(1 - e * e);
            for (int j = 0; j <= ORBIT_RESOLUTION; j++) {
                double E = 2 * Math.PI * j / ORBIT_RESOLUTION;
                double x = a * (Math.cos(E) - e);
                double y = b * Math.sin(E);
                orbit.add(new Point2D.Double(x, y));
            }
            orbitPaths.add(orbit);
        }

        // Pre-compute moon orbit paths (rotated by argument of periapsis)
        for (int i = 0; i < NAMES.length; i++) {
            ArrayList<ArrayList<Point2D.Double>> moonOrbits = new ArrayList<>();
            for (int m = 0; m < MOON_A_M[i].length; m++) {
                ArrayList<Point2D.Double> orbit = new ArrayList<>();
                double a = MOON_A_M[i][m], e = MOON_ECC[i][m], b = a * Math.sqrt(1 - e * e);
                double arg = MOON_ARG_PERI[i][m];
                for (int j = 0; j <= ORBIT_RESOLUTION; j++) {
                    double E = 2 * Math.PI * j / ORBIT_RESOLUTION;
                    double x = a * (Math.cos(E) - e);
                    double y = b * Math.sin(E);
                    double rx = x * Math.cos(arg) - y * Math.sin(arg);
                    double ry = x * Math.sin(arg) + y * Math.cos(arg);
                    orbit.add(new Point2D.Double(rx, ry));
                }
                moonOrbits.add(orbit);
            }
            moonOrbitPaths.add(moonOrbits);
        }

        // Initialize velocities consistent with current Kepler positions (approximate tangential)
        initializeVelocitiesFromKepler();

        // Key listener: Z toggles auto-zoom, S toggles shadows, N toggles n-body
        setFocusable(true);
        addKeyListener(new KeyAdapter() {
            @Override public void keyPressed(KeyEvent e) {
                int code = e.getKeyCode();
                if (code == KeyEvent.VK_Z) {
                    autoZoomEnabled = !autoZoomEnabled;
                    if (!autoZoomEnabled) targetZoom = currentZoom;
                } else if (code == KeyEvent.VK_X) {
                    // Cycle zoom mode: Planet → Orbit → Manual → ...
                    if (followingPlanet && cameraFocusIndex >= 0) {
                        zoomMode = (zoomMode + 1) % 3;
                        updateAutoZoomTarget(); // Recompute zoom based on new mode
                    }
                } else if (code == KeyEvent.VK_S) {
                    shadingEnabled = !shadingEnabled;
                } else if (code == KeyEvent.VK_N) {
                    nBodyEnabled = !nBodyEnabled;
                    System.out.println("N-body simulation: " + nBodyEnabled);
                    // If turning on N-body, keep velocities as initialized; if turning off, reset Kepler mean anomalies to match current positions?
                }
            }
        });

        // Start 60 FPS timer
        Timer timer = new Timer(16, this);
        timer.start();
        lastNanos = System.nanoTime();
    }

    /* ==============================
       SIMULATION UPDATE (60 FPS)
       ============================== */
    /**
     * Called ~60 times per second. Updates physics, camera, and triggers repaint.
     */
    @Override
    public void actionPerformed(ActionEvent ev) {
        long now = System.nanoTime();
        double dtReal = (now - lastNanos) / 1e9;  // Real delta time (seconds)
        lastNanos = now;

        double dtSim = dtReal * timeScale;        // Simulated delta time
        simTimeSec += dtSim;

        // Update planet mean anomalies (only used in Kepler mode)
        if (!nBodyEnabled) {
            for (int i = 0; i < NAMES.length; i++) {
                double periodSec = PERIOD_Y[i] * 365.25 * DAY;
                double n = 2 * Math.PI / periodSec;   // Mean motion
                meanAnomaly[i] = (meanAnomaly[i] + n * dtSim) % (2 * Math.PI);
            }

            // Update moon mean anomalies
            for (int i = 0; i < NAMES.length; i++) {
                for (int m = 0; m < moonMean[i].length; m++) {
                    double periodSec = MOON_PERIOD_DAYS[i][m] * DAY;
                    double n = 2 * Math.PI / periodSec;
                    moonMean[i][m] = (moonMean[i][m] + n * dtSim) % (2 * Math.PI);
                }
            }
        } else {
            // When in N-body, advance moon mean anomalies to keep label positions consistent (not physically necessary)
            for (int i = 0; i < NAMES.length; i++) {
                for (int m = 0; m < moonMean[i].length; m++) {
                    double periodSec = MOON_PERIOD_DAYS[i][m] * DAY;
                    double n = 2 * Math.PI / periodSec;
                    moonMean[i][m] = (moonMean[i][m] + n * dtSim) % (2 * Math.PI);
                }
            }
        }

        // Update positions either via Kepler solver or N-body integrator
        if (!nBodyEnabled) {
            // Solve Kepler's equation for planet positions (original)
            for (int i = 0; i < NAMES.length; i++) {
                double a = A_METERS[i], e = ECC[i], b = a * Math.sqrt(1 - e * e);
                double M = meanAnomaly[i];
                double E = M;
                for (int it = 0; it < 40; it++) {
                    double f = E - e * Math.sin(E) - M;
                    double fp = 1 - e * Math.cos(E);
                    E -= f / fp;
                    if (Math.abs(f) < 1e-12) break;
                }
                double x = a * (Math.cos(E) - e);
                double y = b * Math.sin(E);
                planetX[i] = x; planetY[i] = y;
                planetDistM[i] = Math.hypot(x, y);
                double v = Math.sqrt(G * M_SUN * (2 / planetDistM[i] - 1 / a));
                planetSpeedKmS[i] = v / 1_000.0;

                // Keep velocities consistent (approx) so toggling to N-body is smoother
                double theta = Math.atan2(y, x);
                vx[i] = -v * Math.sin(theta);
                vy[i] =  v * Math.cos(theta);
            }
        } else {
            // ---------- N-BODY NEWTONIAN GRAVITY (velocity-Verlet) ----------

            // Compute accelerations at current positions (due to Sun + mutual)
            computeAccelerations(ax, ay);

            // First half velocity update
            for (int i = 0; i < NAMES.length; i++) {
                if (!alive[i]) continue;
                vx[i] += 0.5 * ax[i] * dtSim;
                vy[i] += 0.5 * ay[i] * dtSim;
            }

            // Position update
            for (int i = 0; i < NAMES.length; i++) {
                if (!alive[i]) continue;
                planetX[i] += vx[i] * dtSim;
                planetY[i] += vy[i] * dtSim;
            }

            // Recompute accelerations at new positions
            computeAccelerations(ax, ay);

            // Second half velocity update
            for (int i = 0; i < NAMES.length; i++) {
                if (!alive[i]) continue;
                vx[i] += 0.5 * ax[i] * dtSim;
                vy[i] += 0.5 * ay[i] * dtSim;
            }

            // Update distances/speeds for HUD
            for (int i = 0; i < NAMES.length; i++) {
                if (!alive[i]) {
                    planetDistM[i] = Double.NaN;
                    planetSpeedKmS[i] = 0;
                    continue;
                }
                planetDistM[i] = Math.hypot(planetX[i], planetY[i]);
                double vmag = Math.hypot(vx[i], vy[i]);
                planetSpeedKmS[i] = vmag / 1_000.0;
            }

            // Collision handling: simple merging if two alive bodies overlap
            handleCollisions();
        }

        /* ----- CAMERA & AUTO-ZOOM ----- */
        if (followingMoon && cameraFocusIndex >= 0 && cameraFocusMoon >= 0) {
            /* ---------- MOON FOLLOWING ---------- */
            int p = cameraFocusIndex, m = cameraFocusMoon;
            double a = MOON_A_M[p][m], e = MOON_ECC[p][m], b = a * Math.sqrt(1 - e * e);
            double M = moonMean[p][m];
            double E = M;
            for (int it = 0; it < 40; it++) {
                double f = E - e * Math.sin(E) - M;
                double fp = 1 - e * Math.cos(E);
                E -= f / fp;
                if (Math.abs(f) < 1e-12) break;
            }
            double mx = a * (Math.cos(E) - e);
            double my = b * Math.sin(E);
            double arg = MOON_ARG_PERI[p][m];
            double rx = mx * Math.cos(arg) - my * Math.sin(arg);
            double ry = mx * Math.sin(arg) + my * Math.cos(arg);
            double targetX = planetX[p] + rx;
            double targetY = planetY[p] + ry;

            double dx = targetX - moonCamX;
            double dy = targetY - moonCamY;
            double dist = Math.hypot(dx, dy);

            if (moonCameraIsApproaching) {
                if (dist <= ARRIVAL_THRESHOLD) {
                    moonCamX = targetX; moonCamY = targetY;
                    moonCameraIsApproaching = false;
                } else {
                    double speed = 12.0;
                    double lerp = 1 - Math.exp(-speed * dtReal);
                    moonCamX += dx * lerp;
                    moonCamY += dy * lerp;
                }
            } else {
                moonCamX = targetX; moonCamY = targetY;
            }
            camX = moonCamX; camY = moonCamY;

            /* moon auto-zoom (unchanged) */
            if (autoZoomEnabled) {
                double moonRadius = MOON_RADIUS_M[p][m];
                double maxRadius = moonRadius * 1.5;
                int screen = Math.min(getWidth(), getHeight());
                double desired = screen * 0.6;
                targetZoom = desired / (maxRadius * METERS_TO_PIXELS * 2);
                targetZoom = Math.max(50.0, Math.min(targetZoom, 50000.0));
            }

        } else if (followingPlanet && cameraFocusIndex >= 0) {
            /* ---------- PLANET FOLLOWING ---------- */
            double targetX = planetX[cameraFocusIndex];
            double targetY = planetY[cameraFocusIndex];
            double dx = targetX - camX;
            double dy = targetY - camY;
            double dist = Math.hypot(dx, dy);

            if (cameraIsApproaching) {
                if (dist <= ARRIVAL_THRESHOLD) {
                    camX = targetX; camY = targetY;
                    cameraIsApproaching = false;
                } else {
                    double speed = 12.0;
                    double lerp = 1 - Math.exp(-speed * dtReal);
                    camX += dx * lerp;
                    camY += dy * lerp;
                }
            } else {
                camX = targetX; camY = targetY;
            }

            /* ---------- AUTO-ZOOM (Planet / Orbit / Manual) ---------- */
            if (autoZoomEnabled) {
                updateAutoZoomTarget();               // <-- NEW CALL
            }

        } else {
            /* ---------- NO FOLLOW – return to Sun ---------- */
            double speed = 8.0;
            double lerp = 1 - Math.exp(-speed * dtReal);
            camX *= (1 - lerp);
            camY *= (1 - lerp);
            moonCamX = camX; moonCamY = camY;

            targetZoom = 1.0;
            cameraIsApproaching = moonCameraIsApproaching = false;
            autoZoomEnabled = false;                 // auto-zoom off when not following
        }

        /* ----- SMOOTH ZOOM INTERPOLATION ----- */
        if (autoZoomEnabled) {
            double lerp = 1 - Math.exp(-ZOOM_SPEED * dtReal);
            currentZoom += (targetZoom - currentZoom) * lerp;
        }

        repaint();
    }

    /* ==============================
       INPUT HANDLING
       ============================== */
    @Override
    public void mouseWheelMoved(MouseWheelEvent e) {
        if (e.isControlDown()) {
            // Ctrl+Scroll: adjust time scale
            timeScale = e.getPreciseWheelRotation() < 0 ? timeScale * 2 : timeScale / 2;
            timeScale = Math.max(1, Math.min(timeScale, 1e9));
        } else {
            // Normal scroll: zoom
            double factor = e.getPreciseWheelRotation() < 0 ? 1.1 : 1/1.1;
            currentZoom *= factor;
            currentZoom = Math.max(0.04, Math.min(currentZoom, 50000));
            targetZoom = currentZoom;
        }
    }

    /**
     * Handles click logic: select planet → moon → unfollow
     */
    private void handleClick(int mx, int my) {
        int cx = getWidth() / 2, cy = getHeight() / 2;

        // MOON CLICK (only when zoomed in and following planet)
        if (followingPlanet && cameraFocusIndex >= 0 && currentZoom > 10) {
            int p = cameraFocusIndex;
            for (int m = 0; m < MOON_A_M[p].length; m++) {
                // Compute moon world position
                double a = MOON_A_M[p][m], e = MOON_ECC[p][m], b = a * Math.sqrt(1 - e * e);
                double M = moonMean[p][m];
                double E = M;
                for (int it = 0; it < 40; it++) {
                    double f = E - e * Math.sin(E) - M;
                    double fp = 1 - e * Math.cos(E);
                    E -= f / fp;
                    if (Math.abs(f) < 1e-12) break;
                }
                double mx_ = a * (Math.cos(E) - e);
                double my_ = b * Math.sin(E);
                double arg = MOON_ARG_PERI[p][m];
                double rx = mx_ * Math.cos(arg) - my_ * Math.sin(arg);
                double ry = mx_ * Math.sin(arg) + my_ * Math.cos(arg);
                double moonWorldX = planetX[p] + rx;
                double moonWorldY = planetY[p] + ry;

                double mpx = cx + (moonWorldX - camX) * METERS_TO_PIXELS * currentZoom;
                double mpy = cy + (moonWorldY - camY) * METERS_TO_PIXELS * currentZoom;
                double mr  = Math.max(RADIUS_M[p] * MOON_SIZE_RATIO[p][m] * METERS_TO_PIXELS * currentZoom, 2.0);

                double dx = mx - mpx, dy = my - mpy;
                if (dx * dx + dy * dy < mr * mr * 4) {
                    if (cameraFocusMoon == m && followingMoon) {
                        // Unfollow moon → back to planet
                        followingMoon = false;
                        cameraFocusMoon = -1;
                        selectedMoon = -1;
                        moonCameraIsApproaching = false;
                        camX = planetX[p];
                        camY = planetY[p];
                    } else {
                        // Follow moon
                        followingMoon = true;
                        moonCameraIsApproaching = true;
                        cameraFocusMoon = m;
                        selectedMoon = m;
                        moonCamX = camX;
                        moonCamY = camY;
                    }
                    cameraIsApproaching = false;
                    return;
                }
            }
        }

        // PLANET CLICK
        for (int i = 0; i < NAMES.length; i++) {
            if (!alive[i]) continue;
            double px = cx + (planetX[i] - camX) * METERS_TO_PIXELS * currentZoom;
            double py = cy + (planetY[i] - camY) * METERS_TO_PIXELS * currentZoom;
            double pr = Math.max(RADIUS_M[i] * METERS_TO_PIXELS * currentZoom, 4);

            double dx = mx - px, dy = my - py;
            if (dx * dx + dy * dy < pr * pr * 4) {
                if (cameraFocusIndex == i && followingPlanet && !followingMoon) {
                    // Unfollow everything
                    followingPlanet = false;
                    cameraFocusIndex = -1;
                    cameraFocusMoon = -1;
                    selectedPlanet = -1;
                    selectedMoon = -1;
                    cameraIsApproaching = false;
                    moonCameraIsApproaching = false;
                    autoZoomEnabled = false;
                    currentZoom = 1.0;
                } else {
                    // Follow planet
                    autoZoomEnabled = true;
                    followingPlanet = true;
                    followingMoon = false;
                    cameraIsApproaching = true;
                    cameraFocusIndex = i;
                    selectedPlanet = i;
                    selectedMoon = -1;
                    cameraFocusMoon = -1;
                }
                return;
            }
        }

        // CLICKED EMPTY SPACE → return to Sun
        followingPlanet = followingMoon = false;
        cameraFocusIndex = cameraFocusMoon = -1;
        selectedPlanet = selectedMoon = -1;
        cameraIsApproaching = moonCameraIsApproaching = false;
        currentZoom = 1.0;
    }

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
        g2.setComposite(AlphaComposite.SrcOver);
        g2.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);
        g2.setRenderingHint(RenderingHints.KEY_RENDERING, RenderingHints.VALUE_RENDER_QUALITY);

        int cx = getWidth() / 2, cy = getHeight() / 2;
        java.util.List<Rectangle> labelBoxes = new ArrayList<>();

        // Draw Sun
        double sunR = SUN_RADIUS_M * METERS_TO_PIXELS * currentZoom;
        double sunX = cx - sunR - camX * METERS_TO_PIXELS * currentZoom;
        double sunY = cy - sunR - camY * METERS_TO_PIXELS * currentZoom;
        g2.setColor(Color.ORANGE);
        g2.fill(new Ellipse2D.Double(sunX, sunY, 2*sunR, 2*sunR));

        FontMetrics fm = g2.getFontMetrics();

        // Draw planets and moons
        for (int i = 0; i < NAMES.length; i++) {
            if (!alive[i]) continue; // skip merged/deactivated bodies visually

            double worldX = planetX[i], worldY = planetY[i];
            double px = cx + (worldX - camX) * METERS_TO_PIXELS * currentZoom;
            double py = cy + (worldY - camY) * METERS_TO_PIXELS * currentZoom;
            double pr = RADIUS_M[i] * METERS_TO_PIXELS * currentZoom;
            if (pr < 0.5) pr = 0.5;

            // Draw orbit (Kepler path preview) — still drawn even in N-body for context
            Path2D.Double path = getADouble(i, cx, cy);
            g2.setColor(currentZoom < PLANET_VISIBLE_ZOOM * 10 ? new Color(74,74,74) : Color.DARK_GRAY);
            g2.setStroke(new BasicStroke(1f));
            g2.draw(path);

            if (i == 5) {  // Saturn ring rendering remains unchanged
                double[] innerFactors = {1.1351, 2.075, 2.52, 1.55, 1.0};
                double[] outerFactors = {2.0, 2.4795, 2.65, 1.7, 1.495};
                Color[] colors = {
                        new Color(152, 135, 107, 180),
                        new Color(104, 92, 73, 180),
                        new Color(104, 92, 73, 180),
                        new Color(42, 37, 30, 180),
                        new Color(42, 37, 30, 180)
                };

                AffineTransform old = g2.getTransform();
                g2.translate(px, py);
                g2.rotate(Math.toRadians(26.7));
                g2.setComposite(AlphaComposite.SrcOver);

                for (int r = 0; r < innerFactors.length; r++) {
                    double innerAU = RADIUS_M[i] / AU * innerFactors[r];
                    double outerAU = RADIUS_M[i] / AU * outerFactors[r];
                    double innerPx = innerAU * AU * METERS_TO_PIXELS * currentZoom;
                    double outerPx = outerAU * AU * METERS_TO_PIXELS * currentZoom;

                    g2.setColor(colors[r]);
                    g2.setStroke(new BasicStroke(
                            (float) ((outerPx - innerPx) * 0.6f),
                            BasicStroke.CAP_BUTT,
                            BasicStroke.JOIN_ROUND
                    ));
                    g2.draw(new Ellipse2D.Double(-outerPx, -outerPx, outerPx * 2, outerPx * 2));
                }
                g2.setTransform(old);
            }

            // Draw planet body
            g2.setColor(COLORS[i]);
            Ellipse2D.Double planetEllipse = new Ellipse2D.Double(px - pr, py - pr, 2*pr, 2*pr);

            // Draw base color
            g2.fill(planetEllipse);

            // Apply shading if enabled
            if (shadingEnabled && pr > 3) {
                drawRealisticShading(g2, planetX[i], planetY[i], pr, cx, cy, i);
            }

            // Draw label with overlap avoidance
            g2.setColor(Color.WHITE);
            String name = NAMES[i];
            int w = fm.stringWidth(name), h = fm.getHeight();
            double angle = Math.atan2(py - cy, px - cx);
            int lx = (int)(px + Math.cos(angle)*(pr+12));
            int ly = (int)(py + Math.sin(angle)*(pr+12));
            Rectangle box = new Rectangle(lx, ly-h, w, h);
            int tries = 0;
            while (tries < 4) {
                boolean overlap = false;
                for (Rectangle r : labelBoxes) {
                    if (r.intersects(box)) { box.y += h; overlap = true; tries++; break; }
                }
                if (!overlap) break;
            }
            labelBoxes.add(box);
            g2.drawString(name, box.x, box.y + h - 3);

            // Selection ring
            if (i == selectedPlanet) {
                g2.setColor(Color.YELLOW);
                g2.draw(new Ellipse2D.Double(px - pr - 3, py - pr - 3, 2*pr + 6, 2*pr + 6));
            }

            // Draw moons (same as before)
            if (MOON_NAMES[i].length > 0) {
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
                    if (currentZoom > ORBIT_VISIBLE_ZOOM) {
                        g2.setColor(new Color(120,120,120,100));
                        g2.setStroke(new BasicStroke(0.8f));
                        g2.draw(mpath);
                    }
                }

                // Draw moon bodies (only when zoomed in)
                if (currentZoom > 10) {
                    for (int m = 0; m < MOON_A_M[i].length; m++) {
                        // Solve moon position
                        double a = MOON_A_M[i][m], e = MOON_ECC[i][m], b = a * Math.sqrt(1 - e * e);
                        double M = moonMean[i][m];
                        double E = M;
                        for (int it = 0; it < 40; it++) {
                            double f = E - e * Math.sin(E) - M;
                            double fp = 1 - e * Math.cos(E);
                            E -= f / fp;
                            if (Math.abs(f) < 1e-12) break;
                        }
                        double mx = a * (Math.cos(E) - e);
                        double my = b * Math.sin(E);
                        double arg = MOON_ARG_PERI[i][m];
                        double rx = mx * Math.cos(arg) - my * Math.sin(arg);
                        double ry = mx * Math.sin(arg) + my * Math.cos(arg);
                        double moonWorldX = planetX[i] + rx;
                        double moonWorldY = planetY[i] + ry;

                        double mpx = cx + (moonWorldX - camX) * METERS_TO_PIXELS * currentZoom;
                        double mpy = cy + (moonWorldY - camY) * METERS_TO_PIXELS * currentZoom;
                        double mr = Math.max(RADIUS_M[i] * MOON_SIZE_RATIO[i][m] * METERS_TO_PIXELS * currentZoom, 0.6);

                        // Base moon
                        g2.setColor(Color.LIGHT_GRAY);
                        g2.fill(new Ellipse2D.Double(mpx - mr, mpy - mr, 2*mr, 2*mr));

                        // Shading for moon
                        if (shadingEnabled && mr > 2) {
                            drawRealisticShading(g2, moonWorldX, moonWorldY, mr, cx, cy, -1);
                        }

                        g2.setColor(Color.WHITE);
                        g2.drawString(MOON_NAMES[i][m], (int)(mpx + mr + 4), (int)(mpy - mr - 2));

                        if (followingPlanet && selectedMoon == m) {
                            g2.setColor(Color.YELLOW);
                            g2.draw(new Ellipse2D.Double(mpx - mr - 3, mpy - mr - 3, 2*mr + 6, 2*mr + 6));
                        }
                    }
                }
            }
        }

        // HUD
        g2.setColor(Color.WHITE);
        g2.drawString(String.format("Zoom: %.2fx %s", currentZoom,
                followingPlanet&&autoZoomEnabled ? "(Auto)" : "(Manual)"), 10, 20);
        g2.drawString(String.format("Time ×%.0f (%.1f days/s)", timeScale, timeScale/DAY), 10, 40);
        g2.drawString(String.format("Sim: %.2f years", simTimeSec/(365.25*DAY)), 10, 60);

        if (nBodyEnabled) {
            g2.drawString("N-body: ON (press N to toggle)", 10, 80);
        } else {
            g2.drawString("N-body: OFF (press N to toggle)", 10, 80);
        }

        if (followingMoon && cameraFocusIndex >= 0 && cameraFocusMoon >= 0) {
            int p = cameraFocusIndex, m = cameraFocusMoon;
            g2.drawString(String.format("Following: %s → %s",
                    NAMES[p], MOON_NAMES[p][m]), 10, 105);
            g2.drawString(String.format("Moon Radius: %.0f km | Zoom: %.1fx (Auto)",
                    MOON_RADIUS_M[p][m]/1_000, targetZoom), 10, 125);
        } else if (followingPlanet && cameraFocusIndex >= 0) {
            g2.drawString(String.format("Following: %s", NAMES[cameraFocusIndex]), 10, 105);
            g2.drawString(String.format("Moons: %d | Target Zoom: %.1fx",
                    MOON_NAMES[cameraFocusIndex].length, targetZoom), 10, 125);
        } else {
            g2.drawString("Scroll=Zoom | Ctrl+Scroll=Speed | Click=Follow", 10, 105);
        }
        g2.drawString("Z=Auto-Zoom: " + (autoZoomEnabled ? "ON" : "OFF"), 10, 145);
        if (followingPlanet && cameraFocusIndex >= 0) {
            g2.drawString("X=Cycle Zoom: " + ZOOM_MODE_NAMES[zoomMode], 10, 165);
        }
        g2.drawString("S=Shading: " + (shadingEnabled ? "ON" : "OFF"), 10, 185);

        // Info panel
        if (selectedPlanet != -1 && alive[selectedPlanet]) {
            int i = selectedPlanet;
            int w = 300, h = 140, x0 = getWidth() - w - 20, y0 = 20;
            g2.setColor(new Color(0,0,0,150));
            g2.fillRoundRect(x0, y0, w, h, 15, 15);
            g2.setColor(Color.WHITE);
            g2.drawRoundRect(x0, y0, w, h, 15, 15);
            g2.drawString("Planet: " + NAMES[i], x0+15, y0+25);
            g2.drawString(String.format("Radius: %.0f km", RADIUS_M[i]/1_000), x0+15, y0+45);
            if (Double.isFinite(planetDistM[i]))
                g2.drawString(String.format("Dist: %.3f AU", planetDistM[i]/AU), x0+15, y0+65);
            g2.drawString(String.format("Speed: %.2f km/s", planetSpeedKmS[i]), x0+15, y0+85);
            g2.drawString(String.format("Period: %.3f y", PERIOD_Y[i]), x0+15, y0+105);
            if (MOON_NAMES[i].length > 0)
                g2.drawString(String.format("Moons: %d", MOON_NAMES[i].length), x0+15, y0+125);
        }
    }

    /** Helper: builds orbit path from precomputed points */
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

    /** Computes initial position from Keplerian elements */
    private Point2D.Double computeInitialPosition(double a, double e, double omega, double Omega, double M) {
        double E = M;
        for (int it = 0; it < 40; it++) {
            double f = E - e * Math.sin(E) - M;
            double fp = 1 - e * Math.cos(E);
            E -= f / fp;
            if (Math.abs(f) < 1e-12) break;
        }
        double xr = a * (Math.cos(E) - e);
        double yr = a * Math.sqrt(1 - e * e) * Math.sin(E);
        double x = xr * Math.cos(omega + Omega) - yr * Math.sin(omega + Omega);
        double y = xr * Math.sin(omega + Omega) + yr * Math.cos(omega + Omega);
        return new Point2D.Double(x, y);
    }

    /**
     * Realistic directional shading: full gradient from lit to dark side.
     * Light source: Sun at (0,0).
     */
    private void drawRealisticShading(Graphics2D g2, double worldX, double worldY,
                                      double radiusPx, int cx, int cy, int index) {
        if (radiusPx < 2) return;

        double centerX = cx + (worldX - camX) * METERS_TO_PIXELS * currentZoom;
        double centerY = cy + (worldY - camY) * METERS_TO_PIXELS * currentZoom;

        // Light direction: from body to Sun
        double lightX = -worldX;
        double lightY = -worldY;
        double distToSun = Math.hypot(lightX, lightY);
        if (distToSun == 0) return;
        lightX /= distToSun;
        lightY /= distToSun;

        // Clip to body
        Ellipse2D.Double fillEllipse =
                new Ellipse2D.Double(centerX - radiusPx - 1, centerY - radiusPx - 1, 2*(radiusPx + 1), 2*(radiusPx + 1));
        g2.fill(fillEllipse);

        // Narrow gradient range for sharp transition
        float brightX = (float)(centerX + lightX * radiusPx * 1.0f);
        float brightY = (float)(centerY + lightY * radiusPx * 1.0f);
        float darkX   = (float)(centerX - lightX * radiusPx * 0.2f);
        float darkY   = (float)(centerY - lightY * radiusPx * 0.2f);

        // HIGHLIGHT: semi-transparent white (overlays on base color)
        // SHADOW: 100% opaque black
        Color HIGHLIGHT;
        if (index == -1)
            HIGHLIGHT = new Color(Color.lightGray.getRed(), Color.lightGray.getGreen(), Color.lightGray.getBlue(), 140);  // reduced alpha slightly for subtlety
        else
            HIGHLIGHT = new Color(COLORS[index].getRed(), COLORS[index].getGreen(), COLORS[index].getBlue(), 140);
        Color SHADOW    = new Color(0, 0, 0, 255);        // FULLY OPAQUE BLACK

        GradientPaint gp = new GradientPaint(
                brightX, brightY, HIGHLIGHT,
                darkX,   darkY,   SHADOW
        );
        g2.setPaint(gp);
        g2.fill(fillEllipse);

        // Optional: subtle rim highlight
        g2.setColor(new Color(255, 255, 255, 40));
        g2.setStroke(new BasicStroke(1.5f));
        g2.draw(fillEllipse);

        g2.setClip(null);
    }


    /** Re-calculates targetZoom according to the current zoomMode */
    private void updateAutoZoomTarget() {
        if (cameraFocusIndex < 0 || followingMoon) return;

        int i = cameraFocusIndex;
        int screen = Math.min(getWidth(), getHeight());
        double desired = screen * 0.6;               // fill 60 % of the smaller side

        double maxRadius;

        if (zoomMode == 0) {                         // 0 = Planet + moons
            maxRadius = RADIUS_M[i];
            targetZoom = desired / (maxRadius * METERS_TO_PIXELS * 2);
            targetZoom = Math.max(5.0, Math.min(targetZoom, 5000.0));

        } else if (zoomMode == 1) {                  // 1 = Whole orbit
            maxRadius = RADIUS_M[i];
            for (int m = 0; m < MOON_A_M[i].length; m++) {
                double moonDist = MOON_A_M[i][m] * 1.2;
                if (moonDist > maxRadius) maxRadius = moonDist;
            }
            targetZoom = desired / (maxRadius * METERS_TO_PIXELS * 2);
            targetZoom = Math.max(5.0, Math.min(targetZoom, 5000.0));

        } else {                                     // 2 = Manual – keep current
            targetZoom = currentZoom;
            autoZoomEnabled = false;
        }
    }

    /* ==============================
       N-BODY SUPPORTING METHODS
       ============================== */

    /** Initialize velocities from Kepler-derived positions (approx tangential) */
    private void initializeVelocitiesFromKepler() {
        for (int i = 0; i < NAMES.length; i++) {
            double x = planetX[i], y = planetY[i];
            double r = Math.hypot(x, y);
            if (r == 0) { vx[i] = vy[i] = 0; continue; }
            // approximate instantaneous speed from vis-viva
            double a = A_METERS[i];
            double v = Math.sqrt(Math.abs(G * M_SUN * (2.0 / r - 1.0 / a)));
            double theta = Math.atan2(y, x);
            // set velocity perpendicular (counter-clockwise)
            vx[i] = -v * Math.sin(theta);
            vy[i] =  v * Math.cos(theta);
        }
    }

    /**
     * Compute accelerations for all planets due to Sun (fixed at origin) and mutual gravity.
     * Results stored in out arrays axOut, ayOut.
     */
    private void computeAccelerations(double[] axOut, double[] ayOut) {
        // Reset
        for (int i = 0; i < NAMES.length; i++) { axOut[i] = 0; ayOut[i] = 0; }

        // Acceleration due to Sun (fixed at origin)
        for (int i = 0; i < NAMES.length; i++) {
            if (!alive[i]) continue;
            double x = planetX[i], y = planetY[i];
            double r2 = x*x + y*y + 1.0E7 * 1.0E7;
            double r = Math.sqrt(r2);
            if (r == 0) continue;
            double aSun = -G * M_SUN / (r2);
            axOut[i] += aSun * (x / r);
            ayOut[i] += aSun * (y / r);
        }

        // Mutual gravity between planets
        for (int i = 0; i < NAMES.length; i++) {
            if (!alive[i]) continue;
            for (int j = i+1; j < NAMES.length; j++) {
                if (!alive[j]) continue;
                double dx = planetX[j] - planetX[i];
                double dy = planetY[j] - planetY[i];
                double r2 = dx*dx + dy*dy + 1.0E7 * 1.0E7;
                double r = Math.sqrt(r2);
                if (r == 0) continue;
                double F = G / (r2); // will multiply by masses below
                double ux = dx / r, uy = dy / r;

                // acceleration contribution
                double ai =  F * bodyMass[j];
                double aj =  F * bodyMass[i];

                axOut[i] += ai * ux / bodyMass[i];
                ayOut[i] += ai * uy / bodyMass[i];

                axOut[j] -= aj * ux / bodyMass[j];
                ayOut[j] -= aj * uy / bodyMass[j];
            }
        }
    }

    /** Handle simple collisions: merge j into i when distance < sum radii */
    private void handleCollisions() {
        for (int i = 0; i < NAMES.length; i++) {
            if (!alive[i]) continue;
            for (int j = i + 1; j < NAMES.length; j++) {
                if (!alive[j]) continue;
                double dx = planetX[j] - planetX[i];
                double dy = planetY[j] - planetY[i];
                double dist = Math.hypot(dx, dy);
                double minDist = RADIUS_M[i] + RADIUS_M[j];

                if (dist < minDist) {
                    // Merge smaller mass into larger mass (momentum cons.)
                    int big = bodyMass[i] >= bodyMass[j] ? i : j;
                    int small = (big == i) ? j : i;

                    double M1 = bodyMass[big];
                    double M2 = bodyMass[small];
                    double Msum = M1 + M2;

                    // Momentum conservation -> new velocity
                    double vxNew = (vx[big]*M1 + vx[small]*M2) / Msum;
                    double vyNew = (vy[big]*M1 + vy[small]*M2) / Msum;

                    // Update big
                    bodyMass[big] = Msum;
                    vx[big] = vxNew;
                    vy[big] = vyNew;

                    // Optionally increase visual radius with simple mass->radius scaling (assume constant density)
                    double density = M1 / (4.0/3.0 * Math.PI * Math.pow(RADIUS_M[big], 3));
                    double newRadius = Math.cbrt( (3.0 * Msum) / (4.0 * Math.PI * density) );
                    RADIUS_M[big] = newRadius;

                    // Deactivate small
                    alive[small] = false;
                    bodyMass[small] = 0;
                    vx[small] = vy[small] = 0;
                    planetX[small] = planetY[small] = 1e30; // send far away
                }
            }
        }
    }

    /* ==============================
       MAIN
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
