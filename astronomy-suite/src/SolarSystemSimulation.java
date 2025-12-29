import javax.swing.*;
import javax.swing.Timer;
import javax.swing.tree.*;
import java.awt.*;
import java.awt.event.*;
import java.awt.geom.AffineTransform;
import java.awt.geom.Ellipse2D;
import java.awt.geom.Path2D;
import java.awt.geom.Point2D;
import java.awt.geom.Rectangle2D;
import java.util.*;

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

//TODO: Sympletic Integration, leapfrog, rebound, saturn rings
public class SolarSystemSimulation extends JPanel
        implements ActionListener, MouseWheelListener, MouseListener, MouseMotionListener {

    /* ==============================
       PHYSICAL CONSTANTS (SI UNITS)
       ============================== */
    /** 1 Astronomical Unit in meters */
    private static final double AU  = 1.496e11;
    /** Seconds in one Earth day */
    private static final double DAY = 86_400.0;
    /** Seconds in one Earth year */
    private static final double YEAR = 365.25 * DAY;
    /** Gravitational constant (m³ kg⁻¹ s⁻²) */
    private static final double G   = 6.67430e-11;
    /** Mass of the Sun (kg) */
    private static final double M_SUN = 1.98847e30;
    /** Mutable Sun mass used in simulation (can be changed at runtime) */
    private final double sunMass = M_SUN;

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

    // ----- Visibility & UI toggles -----
    /** Whether to draw all planet orbits */
    private boolean showPlanetOrbits = true;
    /** Whether to draw planetary rings (e.g., Saturn) */
    private boolean showRings = true;
    /** Whether to draw text labels for bodies */
    private boolean showLabels = true;
    /** If true, only the selected planet (and its moons) is rendered */
    private boolean showOnlySelected = false;

    // ----- Asteroid and Debris Belt Visibility -----
    /** Whether to draw the asteroid belt */
    private boolean showAsteroidBelt = false;
    /** Whether to draw the Kuiper belt */
    private boolean showKuiperBelt = false;
    /** Whether to draw ice lines (frost line) */
    private boolean showIceLines = false;

    // ----- Visibility toggles for individual planets -----
    /** Array tracking whether each planet should be visible */
    private final boolean[] planetVisible = new boolean[12];

    // ----- UI components -----
    /** Text field used by the top search bar to jump to bodies */
    private JTextField searchField;
    /** Popup list of name suggestions for the search field */
    private JPopupMenu searchSuggestionsPopup;
    /** List component that shows filtered planet/moon names */
    private JList<String> searchSuggestionsList;
    /** Tree navigation of planets and moons */
    private JTree bodyTree;
    /** Tree model for navigator */
    private DefaultTreeModel bodyTreeModel;
    /** Root node of the tree */
    private DefaultMutableTreeNode treeRootNode;
    /** Array of planet tree nodes */
    private DefaultMutableTreeNode[] planetTreeNodes;
    /** Cached paths to planet nodes */
    private TreePath[] planetTreePaths;
    /** Cached paths to moon nodes */
    private TreePath[][] moonTreePaths;
    /** Guard to prevent recursive selection syncing */
    private boolean suppressTreeSelection;

    // ----- Time controls -----
    private static final double TIME_SCALE_MIN = 1.0; // minimum 1.0 s/s
    private static final double TIME_SCALE_MAX = 100 * YEAR;
    private static final int TIME_SLIDER_MIN = 0;
    private static final int TIME_SLIDER_MAX = 1000;
    /** True when the simulation clock is paused */
    private boolean simulationPaused = false;
    /** Slider used to adjust time scale linearly */
    private JSlider timeScaleSlider;
    /** Label reflecting the current time scale and pause state */
    private JLabel timeScaleLabel;
    /** Prevent slider event loops when syncing from code */
    private boolean suppressTimeSliderEvents = false;

    /** List of user-launched objects in the simulation */
    private final java.util.List<LaunchedObject> launchedObjects = new ArrayList<>();

    // ----- Scale ruler dragging -----
    /** Current X position of scale ruler (default bottom-left) */
    private int scaleRulerX = 20;
    /** Current Y position of scale ruler (default bottom-left) */
    private int scaleRulerY = -60; // Relative to bottom (negative)
    /** Whether scale ruler is being dragged */
    private boolean scaleDragging = false;
    /** Mouse offset when dragging scale ruler */
    private int scaleDragOffsetX = 0, scaleDragOffsetY = 0;

    // ----- Barycenter motion variables -----
    /** Sun's X position offset due to barycentric motion */
    private double sunBarycenterX = 0.0;
    /** Sun's Y position offset due to barycentric motion */
    private double sunBarycenterY = 0.0;

    /** Toggle: auto-adjust zoom when following bodies */
    private boolean autoZoomEnabled = false;
    /** Target zoom level (auto-computed) */
    private double targetZoom = 1.0;
    /** Current zoom level (smoothly interpolated) */
    private double currentZoom = 1.0;
    /** Speed of zoom interpolation (higher = faster) */
    private static final double ZOOM_SPEED = 10.0;
    private int zoomMode = 0;                   // 0=Planet, 1=Orbit, 2=Manual

    // ----- Orbit visibility thresholds -----
    /** Zoom level at which orbits become visible */
    private static final double ORBIT_VISIBLE_ZOOM = 5.0;

    /* ==============================
       SIMULATION STATE
       ============================== */
    /** Time acceleration factor (1 = real-time, 1000 = 1000× faster) */
    private double timeScale  = DAY;
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
            "Jupiter", "Saturn", "Uranus", "Neptune", "Pluto",
            "Ceres", "Haumea", "Eris"
    };

    /** Body type classification for orbit coloring (planet, gas_giant, dwarf_planet) */
    private static final String[] BODY_TYPE = {
            "planet", "planet", "planet", "planet",
            "gas_giant", "gas_giant", "gas_giant", "gas_giant", "dwarf_planet",
            "dwarf_planet", "dwarf_planet", "dwarf_planet"
    };

    /** Semi-major axis (m) */
    private static final double[] A_METERS = {
            5.7909e10, 1.0821e11, 1.495978707e11, 2.2794e11,
            7.7854e11, 1.4298e12, 2.8725e12, 4.5045e12, 5.9064e12,
            2.767 * AU, 43.335 * AU, 67.781 * AU
    };

    /** Orbital eccentricity */
    private static final double[] ECC = {
            0.205630, 0.006772, 0.016709, 0.093405,
            0.048498, 0.055546, 0.046381, 0.008956, 0.248827,
            0.0758, 0.1887, 0.4407
    };

    /** Orbital period (Earth years) */
    private static final double[] PERIOD_Y = {
            0.2408467, 0.61519726, 1.0000174, 1.8808476,
            11.862615, 29.447498, 84.016846, 164.79132, 247.92065,
            4.60, 284.0, 557.0
    };

    /** Physical radius (m) */
    private static final double[] RADIUS_M = {
            2.439e6, 6.052e6, 6.371e6, 3.390e6,
            6.9911e7, 5.8232e7, 2.5362e7, 2.4622e7, 1.1883e6,
            4.73e5, 8.16e5, 1.163e6
    };

    /** Mean longitude at J2000 (degrees) */
    private static final double[] PLANET_L0 = {
            252.25084, 181.97973, 100.46435, 355.45332, 34.40438,
            49.94432, 313.23218, 304.88003, 238.92881,
            181.0, 210.0, 35.0
    };

    /** Longitude of perihelion at J2000 (degrees) */
    private static final double[] PLANET_VARPI0 = {
            77.45736, 131.60261, 102.93735, 336.04084, 14.75385,
            92.43194, 170.96424, 44.97135, 113.76329,
            73.0, 122.0, 151.0
    };

    /** Longitude of ascending node at J2000 (degrees) */
    private static final double[] PLANET_OMEGA0 = {
            48.33167, 76.67992, 0.0, 49.57854, 100.55615,
            113.71504, 74.22950, 131.72169, 110.30347,
            80.0, 122.0, 36.0
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
            new Color(233, 226, 214),   // Pluto – pale off-white/icy beige
            new Color(180, 200, 210),   // Ceres – muted light gray/blue
            new Color(210, 230, 255),   // Haumea – icy bluish-white
            new Color(190, 205, 220)    // Eris – cold pale blue-gray
    };

    /** Orbit colors based on body type */
    private static final java.util.Map<String, Color> ORBIT_COLORS_BY_TYPE = java.util.Collections.unmodifiableMap(
            new java.util.HashMap<>() {{
                put("planet", new Color(200, 100, 150, 100));          // Purple/magenta for terrestrial planets
                put("gas_giant", new Color(210, 170, 80, 100));        // Golden/tan for gas giants
                put("dwarf_planet", new Color(180, 140, 180, 100));    // Purple-gray for dwarf planets
            }}
    );

    private static final double[] PLANET_MASS = {
            3.3011e23, 4.8675e24, 5.97237e24, 6.4171e23,
            1.8982e27, 5.6834e26, 8.6810e25, 1.02413e26, 1.303e22,
            9.39e20, 4.01e21, 1.66e22
    };

    /* ==============================
       MOON DATA
       ============================== */
    /** Semi-major axis of moons relative to parent planet (m) */
    private static final double[][] MOON_A_M = {
            {}, {}, {3.844e8},
            {9.377e6, 2.356e7},
            {4.217e8, 6.71e8, 1.07e9, 1.883e9},
            {1.222e9, 2.38e8, 5.27e8, 1.861e8, 2.946e8, 3.561e8, 3.561e9},
            {4.363e8, 5.835e8, 4.989e8, 1.913e8, 1.299e8},
            {3.547e8}, {1.195e7}, {}, {}, {}
    };

    /** Moon orbital period (Earth days) */
    private static final double[][] MOON_PERIOD_DAYS = {
            {}, {}, {27.321661},
            {0.318910, 1.26244},
            {1.769278, 3.551181, 7.154552, 16.689018},
            {15.94542, 1.370218, 4.518212, 0.942422, 1.887802, 2.736915, 79.3215},
            {8.706234, 13.46339, 4.144176, 1.376491, 1.413479},
            {5.876854}, {6.38723}, {}, {}, {}
    };

    /** Moon radius as ratio of parent planet radius */
    private static final double[][] MOON_SIZE_RATIO = {
            {}, {}, {0.273},
            {0.11, 0.08},
            {0.286, 0.245, 0.413, 0.378},
            {0.404, 0.036, 0.152, 0.034, 0.024, 0.025, 0.009},
            {0.196, 0.182, 0.173, 0.096, 0.063},
            {0.212}, {0.501},
            {}, {}, {}
    };

    /** Names of moons per planet */
    private static final String[][] MOON_NAMES = {
            {}, {}, {"Moon"},
            {"Phobos", "Deimos"},
            {"Io", "Europa", "Ganymede", "Callisto"},
            {"Titan", "Enceladus", "Rhea", "Mimas", "Tethys", "Dione", "Iapetus"},
            {"Titania", "Oberon", "Umbriel", "Ariel", "Miranda"},
            {"Triton"}, {"Charon"}, {}, {}, {}
    };

    /** Moon orbital eccentricity */
    private static final double[][] MOON_ECC = {
            {}, {}, {0.05490},
            {0.01550, 0.00021},
            {0.00412, 0.00899, 0.00126, 0.00716},
            {0.02880, 0.00470, 0.00100, 0.00200, 0.00000, 0.00020, 0.02800},
            {0.00110, 0.00090, 0.00390, 0.00120, 0.00080},
            {0.000016}, {0.00259}, {}, {}, {}
    };

    /** Argument of periapsis for moons (radians) */
    private static final double[][] MOON_ARG_PERI = {
            {}, {}, {2.034},
            {Math.PI/2, Math.PI},
            {1.88, 0.85, 1.02, 5.92},
            {0.20, 2.29, 5.80, 4.40, 0.0, 2.10, 1.20},
            {3.76, 0.44, 4.72, 2.50, 1.80},
            {Math.PI}, {0.0}, {}, {}, {}
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
   SHADING SYSTEM
   ============================== */
    private boolean shadingEnabled = true;

    // No collision effects when strictly Keplerian.

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
        addMouseMotionListener(this);

        // Initialize all planets as visible by default
        Arrays.fill(planetVisible, true);
        // Hide dwarf planets except Pluto (Pluto=8, Ceres=9, Haumea=10, Eris=11)
        planetVisible[9] = false;  // Ceres
        planetVisible[10] = false; // Haumea
        planetVisible[11] = false; // Eris

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
        }

        // Initialize moon mean anomaly arrays
        for (int i = 0; i < NAMES.length; i++) {
            int num = MOON_NAMES[i].length;
            moonMean[i] = new double[num];
        }

        // Pre-compute planet orbit paths with proper orbital element transformations
        for (int i = 0; i < NAMES.length; i++) {
            ArrayList<Point2D.Double> orbit = new ArrayList<>();
            double a = A_METERS[i], e = ECC[i], b = a * Math.sqrt(1 - e * e);
            double omega = Math.toRadians(PLANET_VARPI0[i] - PLANET_OMEGA0[i]); // argument of perihelion
            double Omega = Math.toRadians(PLANET_OMEGA0[i]);                    // longitude of ascending node
            for (int j = 0; j <= ORBIT_RESOLUTION; j++) {
                double E = 2 * Math.PI * j / ORBIT_RESOLUTION;
                // Orbital plane coordinates
                double xr = a * (Math.cos(E) - e);
                double yr = b * Math.sin(E);
                // Apply orbital element rotations to get ecliptic plane coordinates
                double x = xr * Math.cos(omega + Omega) - yr * Math.sin(omega + Omega);
                double y = xr * Math.sin(omega + Omega) + yr * Math.cos(omega + Omega);
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

        // Key listener: Z toggles auto-zoom, S toggles shading.
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
                        updateAutoZoomTarget();
                    }
                } else if (code == KeyEvent.VK_S) {
                    shadingEnabled = !shadingEnabled;
                } else if (code == KeyEvent.VK_O) {
                    // Toggle planet orbits
                    showPlanetOrbits = !showPlanetOrbits;
                } else if (code == KeyEvent.VK_R) {
                    // Toggle rings
                    showRings = !showRings;
                } else if (code == KeyEvent.VK_L) {
                    // Toggle labels
                    showLabels = !showLabels;
                } else if (code == KeyEvent.VK_B) {
                    // Toggle "show only selected planet" mode
                    showOnlySelected = !showOnlySelected;
                } else if (code == KeyEvent.VK_SPACE || code == KeyEvent.VK_P) {
                    // Play / Pause toggle
                    simulationPaused = !simulationPaused;
                    updateTimeScaleLabel();
                } else if (code == KeyEvent.VK_BACK_SPACE) {
                    // Backspace: delete last launched object
                    if (!launchedObjects.isEmpty()) {
                        launchedObjects.removeLast();
                        repaint();
                    }
                    e.consume();
                } else if (code == KeyEvent.VK_F) {
                    // F: quick-launch a default object from origin
                    quickLaunchDefault();
                    e.consume();
                } else if (code == KeyEvent.VK_M) {
                    // Toggle display of planet masses
                    for (int i = 0; i < NAMES.length; i++) {
                        if (selectedPlanet == i) {
                            selectedPlanet = -1;  // Deselect if already selected
                            break;
                        }
                    }
                    selectedPlanet = (selectedPlanet + 1) % NAMES.length;
                    e.consume();
                }
            }
        });
        // Ensure key actions work regardless of focus using InputMap/ActionMap
        InputMap im = getInputMap(JComponent.WHEN_IN_FOCUSED_WINDOW);
        ActionMap am = getActionMap();
        im.put(KeyStroke.getKeyStroke(KeyEvent.VK_F, 0), "launchDefault");
        am.put("launchDefault", new AbstractAction() {
            @Override public void actionPerformed(ActionEvent e) { quickLaunchDefault(); }
        });
        im.put(KeyStroke.getKeyStroke(KeyEvent.VK_BACK_SPACE, 0), "deleteLastLaunched");
        am.put("deleteLastLaunched", new AbstractAction() {
            @Override public void actionPerformed(ActionEvent e) {
                if (!launchedObjects.isEmpty()) {
                    launchedObjects.removeLast();
                    repaint();
                }
            }
        });

        // Start 60 FPS timer
        Timer timer = new Timer(16, this);
        timer.start();
        lastNanos = System.nanoTime();
    }

    // Helper: whether a given planet should be drawn under current visibility settings
    private boolean isPlanetVisible(int planetIndex) {
        // Check individual planet visibility toggle
        if (planetIndex >= 0 && planetIndex < planetVisible.length && !planetVisible[planetIndex]) {
            return false;
        }
        // Check showOnlySelected mode
        if (!showOnlySelected) return true;
        if (selectedPlanet == -1) return true;
        return planetIndex == selectedPlanet;
    }

    /* ==============================
       INPUT HANDLING
       ============================== */
    @Override
    public void mouseWheelMoved(MouseWheelEvent e) {
        if (e.isControlDown()) {
            // Ctrl+Scroll: adjust time scale
            double proposed = e.getPreciseWheelRotation() < 0 ? timeScale * 2 : timeScale / 2;
            proposed = Math.max(1.0, proposed); // clamp to 1.0 s/s minimum
            setTimeScaleAndSync(proposed);
        } else {
            // Normal scroll: zoom
            double factor = e.getPreciseWheelRotation() < 0 ? 1.1 : 1/1.1;
            currentZoom *= factor;
            currentZoom = Math.max(0.02, Math.min(currentZoom, 50000));
            targetZoom = currentZoom;
        }
    }

    /**
     * Handles click logic: select planet → moon → unfollow
     */
    private void handleClick(int mx, int my) {
        exitSearchModeIfActive();
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
                        // Use barycentric coordinates
                        camX = planetX[p] + sunBarycenterX;
                        camY = planetY[p] + sunBarycenterY;
                        // Snap immediately back to planet view
                        focusPlanetImmediate(p);
                    } else {
                        // Follow moon
                        followingMoon = true;
                        moonCameraIsApproaching = false; // we'll snap immediately
                        cameraFocusMoon = m;
                        selectedMoon = m;
                        cameraIsApproaching = false;
                        // Snap immediately to moon position and zoom
                        focusMoonImmediate(p, m);
                    }
                    return;
                }
            }
        }

        // PLANET CLICK
        for (int i = 0; i < NAMES.length; i++) {
            // Use barycentric coordinates for click detection
            double px = cx + (planetX[i] + sunBarycenterX - camX) * METERS_TO_PIXELS * currentZoom;
            double py = cy + (planetY[i] + sunBarycenterY - camY) * METERS_TO_PIXELS * currentZoom;
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
                    followingPlanet = true;
                    followingMoon = false;
                    cameraFocusIndex = i;
                    selectedPlanet = i;
                    selectedMoon = -1;
                    cameraFocusMoon = -1;
                    autoZoomEnabled = true;
                    // Snap immediately to planet position and zoom
                    focusPlanetImmediate(i);
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

    @Override public void mouseClicked(MouseEvent e) {
        if (!handleScaleRulerClick(e.getX(), e.getY(), getHeight())) {
            handleClick(e.getX(), e.getY());
        }
    }
    @Override public void mousePressed(MouseEvent e) {
        handleScaleRulerClick(e.getX(), e.getY(), getHeight());
    }
    @Override public void mouseReleased(MouseEvent e) {
        stopScaleRulerDrag();
    }
    @Override public void mouseEntered(MouseEvent e) {}
    @Override public void mouseExited(MouseEvent e) {}

    @Override public void mouseDragged(MouseEvent e) {
        updateScaleRulerDrag(e.getX(), e.getY());
        repaint();
    }

    @Override public void mouseMoved(MouseEvent e) {}

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

        // Draw Sun at origin in world coordinates, offset by barycenter
        double sunR = SUN_RADIUS_M * METERS_TO_PIXELS * currentZoom;
        double sunWorldX = sunBarycenterX;  // Apply barycenter offset for Sun motion
        double sunWorldY = sunBarycenterY;  // Apply barycenter offset for Sun motion
        double sunXpix = cx + (sunWorldX - camX) * METERS_TO_PIXELS * currentZoom - sunR;
        double sunYpix = cy + (sunWorldY - camY) * METERS_TO_PIXELS * currentZoom - sunR;
        g2.setColor(Color.ORANGE);
        g2.fill(new Ellipse2D.Double(sunXpix, sunYpix, 2*sunR, 2*sunR));

        FontMetrics fm = g2.getFontMetrics();

        // Draw asteroid belt and Kuiper belt as rings
        drawAsteroidBeltRing(g2, cx, cy);
        drawKuiperBeltRing(g2, cx, cy);

        // Draw ice lines
        drawIceLines(g2, cx, cy);

        // Draw planets and moons
        for (int i = 0; i < NAMES.length; i++) {
            if (!isPlanetVisible(i)) continue;

            // Planet positions are heliocentric, so add Sun's barycentric offset
            double worldX = planetX[i] + sunBarycenterX;
            double worldY = planetY[i] + sunBarycenterY;
            double px = cx + (worldX - camX) * METERS_TO_PIXELS * currentZoom;
            double py = cy + (worldY - camY) * METERS_TO_PIXELS * currentZoom;
            double pr = RADIUS_M[i] * METERS_TO_PIXELS * currentZoom;
            if (pr < 0.5) pr = 0.5;

            // Draw orbit (respect visibility toggle)
            if (showPlanetOrbits) {
                Path2D.Double path = getADouble(i, cx, cy);
                g2.setColor(getOrbitColor(i));
                g2.setStroke(new BasicStroke(1f));
                g2.draw(path);
            }

            // Draw Saturn rings only if enabled
            if (showRings && i == 5) {
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

                // Draw ring bands (no shadows)
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

            // Selection ring (drawn AFTER rings, BEFORE planet body so it doesn't get too thick from rotation)
            if (i == selectedPlanet) {
                g2.setColor(Color.YELLOW);
                g2.setStroke(new BasicStroke(2f));
                g2.draw(new Ellipse2D.Double(px - pr - 3, py - pr - 3, 2*pr + 6, 2*pr + 6));
            }

            // Draw planet body
            g2.setColor(COLORS[i]);
            Ellipse2D.Double planetEllipse = new Ellipse2D.Double(px - pr, py - pr, 2*pr, 2*pr);

            // Apply shading if enabled, otherwise draw flat color
            if (shadingEnabled && pr > 3) {
                drawRealisticShading(g2, planetX[i], planetY[i], pr, px, py, i);
            } else {
                // Draw base color (no shading)
                g2.fill(planetEllipse);
            }

            // Draw label with overlap avoidance
            if (showLabels) {
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
                        g2.setColor(new Color(100, 150, 100, 80));  // Green-tinted for moon orbits
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
                        // Moon position relative to planet's barycentric position
                        double moonWorldX = planetX[i] + sunBarycenterX + rx;
                        double moonWorldY = planetY[i] + sunBarycenterY + ry;

                        double mpx = cx + (moonWorldX - camX) * METERS_TO_PIXELS * currentZoom;
                        double mpy = cy + (moonWorldY - camY) * METERS_TO_PIXELS * currentZoom;
                        double mr = Math.max(RADIUS_M[i] * MOON_SIZE_RATIO[i][m] * METERS_TO_PIXELS * currentZoom, 0.6);

                        // Draw moon with or without shading
                        if (shadingEnabled && mr > 2) {
                            drawRealisticShading(g2, moonWorldX, moonWorldY, mr, mpx, mpy, -1);
                        } else {
                            // Base moon (no shading)
                            g2.setColor(Color.LIGHT_GRAY);
                            g2.fill(new Ellipse2D.Double(mpx - mr, mpy - mr, 2*mr, 2*mr));
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

        drawLaunchedObjects(g2, cx, cy);

        // Draw dynamic scale ruler at bottom-left
        drawDynamicScale(g2, getHeight());

         // HUD
         g2.setColor(Color.WHITE);
        g2.drawString(String.format("Zoom: %.2fx %s", currentZoom,
                followingPlanet&&autoZoomEnabled ? "(Auto)" : "(Manual)"), 10, 20);
        g2.drawString("Time: " + formatTimeScale(timeScale), 10, 40);
        g2.drawString(String.format("Sim: %.2f years", simTimeSec/(365.25*DAY)), 10, 60);
        g2.drawString(String.format("Launched: %d", launchedObjects.size()), 10, 80);

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
        }

        // Info panel
        if (selectedPlanet != -1) {
            int i = selectedPlanet;
            // Larger, more comfortable panel; extra height when moon metadata is visible
            int baseHeight = 210;
            int extraMoonHeight = 120;
            int w = 380;
            int h = baseHeight + ((selectedMoon >= 0 && selectedMoon < MOON_NAMES[i].length) ? extraMoonHeight : 0);
            int x0 = getWidth() - w - 20, y0 = 20;

            // Panel background with subtle border
            g2.setColor(new Color(10, 10, 20, 200));
            g2.fillRoundRect(x0, y0, w, h, 18, 18);
            g2.setColor(new Color(200, 220, 255, 200));
            g2.setStroke(new BasicStroke(1.5f));
            g2.drawRoundRect(x0, y0, w, h, 18, 18);

            // Use a slightly smaller, clean font for dense info
            Font oldFont = g2.getFont();
            g2.setFont(oldFont.deriveFont(Font.PLAIN, 12f));

            int lineY = y0 + 26;
            int lineStep = 16;
            int colX = x0 + 16;

            // Header
            g2.setColor(new Color(230, 240, 255));
            g2.drawString("Planet", colX, lineY); lineY += lineStep;
            g2.setColor(Color.WHITE);
            g2.drawString("Name: " + NAMES[i], colX, lineY); lineY += lineStep;

            // Planet core properties
            g2.drawString(String.format("Radius: %.0f km", RADIUS_M[i]/1_000), colX, lineY); lineY += lineStep;
            if (Double.isFinite(planetDistM[i])) {
                g2.drawString(String.format("Distance: %.3f AU (%.0f km)",
                        planetDistM[i]/AU, planetDistM[i]/1_000.0), colX, lineY);
                lineY += lineStep;
            }
            g2.drawString(String.format("Speed: %.2f km/s", planetSpeedKmS[i]), colX, lineY); lineY += lineStep;

            double a_m = A_METERS[i];
            double e = ECC[i];
            double periodYears = PERIOD_Y[i];
            double periodDays = periodYears * 365.25;
            double peri_m = a_m * (1.0 - e);
            double apo_m  = a_m * (1.0 + e);

            double M = meanAnomaly[i];
            double frac = (M % (2 * Math.PI)) / (2 * Math.PI);
            if (frac < 0) frac += 1.0;
            double pct = frac * 100.0;

            g2.drawString(String.format("a: %.3f AU  e: %.3f", a_m / AU, e), colX, lineY); lineY += lineStep;
            g2.drawString(String.format("Peri: %.3f AU  Apo: %.3f AU", peri_m / AU, apo_m / AU), colX, lineY); lineY += lineStep;
            g2.drawString(String.format("Period: %.1f d (%.2f y)", periodDays, periodYears), colX, lineY); lineY += lineStep;
            g2.drawString(String.format("Orbit progress: %.1f%%", pct), colX, lineY); lineY += lineStep;

            g2.drawString(String.format("Central mass: %.3e kg", sunMass), colX, lineY); lineY += lineStep;
            if (MOON_NAMES[i].length > 0) {
                g2.drawString(String.format("Moons: %d", MOON_NAMES[i].length), colX, lineY); lineY += lineStep;
            }

            // Moon metadata block when a moon is selected
            if (selectedMoon >= 0 && selectedMoon < MOON_NAMES[i].length) {
                int m = selectedMoon;
                lineY += lineStep; // spacing before moon section

                g2.setColor(new Color(230, 240, 255));
                g2.drawString("Moon", colX, lineY); lineY += lineStep;
                g2.setColor(Color.WHITE);
                g2.drawString("Name: " + MOON_NAMES[i][m], colX, lineY); lineY += lineStep;

                double aMoon = MOON_A_M[i][m];
                double eMoon = MOON_ECC[i][m];
                double periodSidDays = MOON_PERIOD_DAYS[i][m];
                double periMoon = aMoon * (1.0 - eMoon);
                double apoMoon  = aMoon * (1.0 + eMoon);

                // Approx synodic period relative to parent: 1 / (1/P_moon - 1/P_planet)
                double synodicDays;
                if (periodSidDays > 0 && periodDays > 0 && Math.abs(1.0/periodSidDays - 1.0/periodDays) > 1e-9) {
                    synodicDays = 1.0 / (1.0/periodSidDays - 1.0/periodDays);
                } else {
                    synodicDays = Double.NaN;
                }

                g2.drawString(String.format("a: %.0f km  e: %.3f", aMoon/1_000.0, eMoon), colX, lineY); lineY += lineStep;
                g2.drawString(String.format("Peri: %.0f km  Apo: %.0f km", periMoon/1_000.0, apoMoon/1_000.0), colX, lineY); lineY += lineStep;
                g2.drawString(String.format("Sidereal period: %.2f d", periodSidDays), colX, lineY); lineY += lineStep;
                if (!Double.isNaN(synodicDays)) {
                    g2.drawString(String.format("Synodic vs %s: %.2f d", NAMES[i], synodicDays), colX, lineY);
                }
            }

            // Restore font
            g2.setFont(oldFont);
        }
    }

    /**
     * Draw Kuiper belt (beyond Neptune)
     * Draw as a ring using concentric circles
     */
    private void drawKuiperBeltRing(Graphics2D g2, int cx, int cy) {
        if (!showKuiperBelt) return;

        // Kuiper belt: ~30 AU to 55 AU from Sun
        double innerAU = 30.0;
        double outerAU = 55.0;
        double innerPx = innerAU * AU * METERS_TO_PIXELS * currentZoom;
        double outerPx = outerAU * AU * METERS_TO_PIXELS * currentZoom;

        // Only draw if visible at current zoom
        if (outerPx > 0.5) {
            // Center on Sun's barycentric position
            double sunScreenX = cx + (sunBarycenterX - camX) * METERS_TO_PIXELS * currentZoom;
            double sunScreenY = cy + (sunBarycenterY - camY) * METERS_TO_PIXELS * currentZoom;

            // Draw ring as stroked circle (the band between inner and outer radius)
            g2.setColor(new Color(180, 200, 220, 100));
            g2.setStroke(new BasicStroke((float)(outerPx - innerPx)));
            double midRadius = (innerPx + outerPx) / 2.0;
            Ellipse2D.Double ring = new Ellipse2D.Double(sunScreenX - midRadius, sunScreenY - midRadius, 2*midRadius, 2*midRadius);
            g2.draw(ring);
        }
    }

    /**
     * Draw asteroid belt (between Mars and Jupiter)
     * Draw as a ring using concentric circles
     */
    private void drawAsteroidBeltRing(Graphics2D g2, int cx, int cy) {
        if (!showAsteroidBelt) return;

        // Asteroid belt: ~2.2 AU to 3.2 AU from Sun
        double innerAU = 2.2;
        double outerAU = 3.2;
        double innerPx = innerAU * AU * METERS_TO_PIXELS * currentZoom;
        double outerPx = outerAU * AU * METERS_TO_PIXELS * currentZoom;

        // Only draw if visible at current zoom
        if (outerPx > 0.5) {
            // Center on Sun's barycentric position
            double sunScreenX = cx + (sunBarycenterX - camX) * METERS_TO_PIXELS * currentZoom;
            double sunScreenY = cy + (sunBarycenterY - camY) * METERS_TO_PIXELS * currentZoom;

            // Draw ring as stroked circle (the band between inner and outer radius)
            g2.setColor(new Color(200, 140, 80, 100));
            g2.setStroke(new BasicStroke((float)(outerPx - innerPx)));
            double midRadius = (innerPx + outerPx) / 2.0;
            Ellipse2D.Double ring = new Ellipse2D.Double(sunScreenX - midRadius, sunScreenY - midRadius, 2*midRadius, 2*midRadius);
            g2.draw(ring);
        }
    }

    /**
     * Draw frost lines for H2O and CO2
     * Shows regions where different volatile compounds can condense
     */
    private void drawIceLines(Graphics2D g2, int cx, int cy) {
        if (!showIceLines) return;

        // Center on Sun's barycentric position
        double sunScreenX = cx + (sunBarycenterX - camX) * METERS_TO_PIXELS * currentZoom;
        double sunScreenY = cy + (sunBarycenterY - camY) * METERS_TO_PIXELS * currentZoom;

        // H2O frost line (water ice): ~4.85 AU
        double h2oFrostLineAU = 4.85;
        double h2oFrostLinePx = h2oFrostLineAU * AU * METERS_TO_PIXELS * currentZoom;

        // CO2 frost line (dry ice): ~30.0 AU
        double co2FrostLineAU = 30.0;
        double co2FrostLinePx = co2FrostLineAU * AU * METERS_TO_PIXELS * currentZoom;

        // Only draw if visible
        if (h2oFrostLinePx > 1.0) {
            // H2O frost line: bright cyan with cyan label
            drawFrostLine(g2, sunScreenX, sunScreenY, h2oFrostLinePx, new Color(100, 200, 255, 120), "H2O Frost Line");
        }

        if (co2FrostLinePx > 1.0) {
            // CO2 frost line: pale blue with blue label
            drawFrostLine(g2, sunScreenX, sunScreenY, co2FrostLinePx, new Color(150, 180, 220, 100), "CO2 Frost Line");
        }
    }

    /**
     * Helper: Draw a single frost line with label and fade effect
     */
    private void drawFrostLine(Graphics2D g2, double cx, double cy, double radiusPx, Color color, String label) {
        // Draw main line
        g2.setColor(color);
        g2.setStroke(new BasicStroke(2f));
        Ellipse2D.Double circle = new Ellipse2D.Double(cx - radiusPx, cy - radiusPx, 2*radiusPx, 2*radiusPx);
        g2.draw(circle);

        // Draw fade/glow effect (3 circles with decreasing opacity)
        for (int i = 1; i <= 3; i++) {
            float alpha = color.getAlpha() / 255f * (1.0f - i / 4.0f);
            g2.setColor(new Color(color.getRed(), color.getGreen(), color.getBlue(), (int)(alpha * 255)));
            g2.setStroke(new BasicStroke(0.5f));
            double fadeRadius = radiusPx + i * 2;
            Ellipse2D.Double fadeCircle = new Ellipse2D.Double(cx - fadeRadius, cy - fadeRadius, 2*fadeRadius, 2*fadeRadius);
            g2.draw(fadeCircle);
        }

        // Draw label at top of frost line circle
        if (label != null) {
            g2.setColor(color);  // Use same color as line for label
            Font oldFont = g2.getFont();
            g2.setFont(oldFont.deriveFont(Font.BOLD, 11f));
            FontMetrics fm = g2.getFontMetrics();
            int labelWidth = fm.stringWidth(label);
            int labelX = (int)(cx - (double) labelWidth / 2);
            int labelY = (int)(cy - radiusPx - 5);  // Slightly above the line
            g2.drawString(label, labelX, labelY);
            g2.setFont(oldFont);
        }
    }

    /** Helper: builds orbit path from precomputed points */
    private Path2D.Double getADouble(int i, int cx, int cy) {
        ArrayList<Point2D.Double> orbit = orbitPaths.get(i);
        Path2D.Double path = new Path2D.Double();
        if (!orbit.isEmpty()) {
            Point2D.Double first = orbit.getFirst();
            // Orbits are heliocentric, so center them on the Sun's current barycentric position
            double sx = cx + (first.x + sunBarycenterX - camX) * METERS_TO_PIXELS * currentZoom;
            double sy = cy + (first.y + sunBarycenterY - camY) * METERS_TO_PIXELS * currentZoom;
            path.moveTo(sx, sy);
            for (int j = 1; j < orbit.size(); j++) {
                Point2D.Double p = orbit.get(j);
                double x = cx + (p.x + sunBarycenterX - camX) * METERS_TO_PIXELS * currentZoom;
                double y = cy + (p.y + sunBarycenterY - camY) * METERS_TO_PIXELS * currentZoom;
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
     * Get the bounding rectangle of the scale ruler for hit detection.
     */
    private Rectangle getScaleRulerBounds(int canvasHeight) {
        int scaleY = (scaleRulerY < 0) ? (canvasHeight + scaleRulerY) : scaleRulerY;
        int panelWidth = 200;
        int panelHeight = 65;
        return new Rectangle(scaleRulerX - 5, scaleY - 5, panelWidth, panelHeight);
    }

    /**
     * Check if a click is within the scale ruler bounds and start dragging if so.
     */
    private boolean handleScaleRulerClick(int mx, int my, int canvasHeight) {
        Rectangle bounds = getScaleRulerBounds(canvasHeight);
        if (bounds.contains(mx, my)) {
            scaleDragging = true;
            scaleDragOffsetX = mx - scaleRulerX;
            scaleDragOffsetY = my - (scaleRulerY < 0 ? (canvasHeight + scaleRulerY) : scaleRulerY);
            return true;
        }
        return false;
    }

    /**
     * Update scale ruler position during dragging.
     */
    private void updateScaleRulerDrag(int mx, int my) {
        if (!scaleDragging) return;
        scaleRulerX = mx - scaleDragOffsetX;

        // Convert back to relative coordinates if near bottom, absolute if moved away
        scaleRulerY = my - scaleDragOffsetY;
    }

    /**
     * Stop dragging the scale ruler.
     */
    private void stopScaleRulerDrag() {
        scaleDragging = false;
    }

    /**
     * Draw a dynamic scale ruler at the bottom-left, showing distances in AU and light-seconds/minutes/hours.
     * Similar to Google Maps scale. Position is draggable.
     */
    private void drawDynamicScale(Graphics2D g2, int canvasHeight) {
        // Position: calculate Y based on bottom offset or fixed position
        int scaleX = scaleRulerX;
        int scaleY = (scaleRulerY < 0) ? (canvasHeight + scaleRulerY) : scaleRulerY;

        // Determine the scale distance in pixels
        // We want a nice round number: 50, 100, 200, 500, etc.
        double pixelsPerAU = METERS_TO_PIXELS * currentZoom * AU;
        double minPixels = 80.0; // Minimum pixels for the scale bar
        double maxPixels = 150.0; // Maximum pixels for the scale bar

        // Calculate what 1 AU represents in pixels at current zoom
        double distanceInAUForMinPixels = minPixels / pixelsPerAU;

        // Find a nice round number (1, 2, 5, 10, 20, 50, 100, 200, 500, 1000, etc.)
        double scaleAU = findNiceRoundNumber(distanceInAUForMinPixels);
        double scalePixels = scaleAU * pixelsPerAU;

        // Limit scale bar size
        while (scalePixels > maxPixels && scaleAU >= 0.01) {
            scaleAU = scaleAU / 2.0;
            scalePixels = scaleAU * pixelsPerAU;
        }
        while (scalePixels < minPixels && scaleAU < 100000) {
            scaleAU = scaleAU * 2.0;
            scalePixels = scaleAU * pixelsPerAU;
        }

        // Convert AU to other units
        double scaleMeters = scaleAU * AU;
        double lightSecond = 299792458.0; // meters per light-second
        double scaleLightSeconds = scaleMeters / lightSecond;

        String distanceLabel;
        if (scaleAU >= 1.0) {
            distanceLabel = String.format("%.2f AU", scaleAU);
        } else {
            distanceLabel = String.format("%.4f AU", scaleAU);
        }

        // Format light-time distance
        String timeLightLabel;
        if (scaleLightSeconds >= 3600.0) {
            double hours = scaleLightSeconds / 3600.0;
            timeLightLabel = String.format("%.2f light-hours", hours);
        } else if (scaleLightSeconds >= 60.0) {
            double minutes = scaleLightSeconds / 60.0;
            timeLightLabel = String.format("%.2f light-minutes", minutes);
        } else {
            timeLightLabel = String.format("%.2f light-seconds", scaleLightSeconds);
        }

        // Draw semi-transparent background panel (no outline)
        g2.setColor(new Color(20, 20, 40, 140));
        int panelWidth = 200;
        int panelHeight = 65;
        g2.fillRoundRect(scaleX - 5, scaleY - 5, panelWidth, panelHeight, 8, 8);

        // Draw scale bar (horizontal line with ticks)
        int barY = scaleY + 15;
        int barStartX = scaleX + 10;

        g2.setColor(new Color(200, 220, 255));
        g2.setStroke(new BasicStroke(2.0f));
        g2.drawLine(barStartX, barY, (int)(barStartX + scalePixels), barY);

        // Tick marks at start and end
        int tickHeight = 6;
        g2.drawLine(barStartX, barY - tickHeight, barStartX, barY + tickHeight);
        g2.drawLine((int)(barStartX + scalePixels), barY - tickHeight, (int)(barStartX + scalePixels), barY + tickHeight);

        // Draw text labels
        Font oldFont = g2.getFont();
        g2.setFont(oldFont.deriveFont(Font.PLAIN, 11f));
        g2.setColor(new Color(220, 235, 255));

        FontMetrics fm = g2.getFontMetrics();
        String auLabel = distanceLabel;
        int auLabelWidth = fm.stringWidth(auLabel);
        g2.drawString(auLabel, barStartX + (int)(scalePixels / 2) - auLabelWidth / 2, barY - 12);

        String lightLabel = timeLightLabel;
        int lightLabelWidth = fm.stringWidth(lightLabel);
        g2.drawString(lightLabel, barStartX + (int)(scalePixels / 2) - lightLabelWidth / 2, barY + 22);

        g2.setFont(oldFont);
    }

    /**
     * Find a nice round number >= target for scale display.
     * Examples: 1, 2, 5, 10, 20, 50, 100, 200, 500, 1000, etc.
     */
    private double findNiceRoundNumber(double target) {
        if (target <= 0) return 1.0;

        double magnitude = Math.pow(10, Math.floor(Math.log10(target)));
        double normalized = target / magnitude;

        double[] roundNumbers = {1, 2, 5, 10};
        for (double num : roundNumbers) {
            if (num >= normalized) {
                return num * magnitude;
            }
        }
        return 10 * magnitude;
    }

    /**
     * Realistic directional shading for top-down view: linear gradient with half circle in shadow.
     * Light source: Sun at barycenter.
     * @param px, py - screen coordinates of planet center
     */
    private void drawRealisticShading(Graphics2D g2, double worldX, double worldY,
                                      double radiusPx, double px, double py, int index) {
        if (radiusPx < 2) return;

        // Light direction: from body to Sun (Sun is at barycentric position)
        double lightX = sunBarycenterX - worldX;
        double lightY = sunBarycenterY - worldY;
        double distToSun = Math.hypot(lightX, lightY);
        if (distToSun == 0) return;
        lightX /= distToSun;
        lightY /= distToSun;

        // Create ellipse matching the planet's position
        Ellipse2D.Double fillEllipse =
                new Ellipse2D.Double(px - radiusPx, py - radiusPx, 2*radiusPx, 2*radiusPx);

        // Set clip to body shape BEFORE drawing gradient
        Shape oldClip = g2.getClip();
        g2.setClip(fillEllipse);

        // Gradient endpoints: very close together for abrupt transition
        // Lit point: just 0.3x radius toward sun (creates sharp edge)
        // Dark point: just 0.3x radius away from sun
        float litX = (float)(px + lightX * radiusPx * 0.2);
        float litY = (float)(py + lightY * radiusPx * 0.2);
        float darkX = (float)(px - lightX * radiusPx * 0.15);
        float darkY = (float)(py - lightY * radiusPx * 0.15);

        // Colors: normal planet color on lit side, black on shadow side
        Color litColor;
        if (index == -1) {
            // For moons - light gray
            litColor = Color.LIGHT_GRAY;
        } else {
            // For planets - use their normal color
            litColor = COLORS[index];
        }
        Color shadowColor = Color.BLACK;

        // Create linear gradient from lit to dark side
        GradientPaint gp = new GradientPaint(
                litX, litY, litColor,
                darkX, darkY, shadowColor
        );

        g2.setPaint(gp);
        g2.fill(fillEllipse);

        // Restore clip
        g2.setClip(oldClip);

        // Reset paint to solid color for subsequent drawing
        g2.setPaint(null);
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
       Time slider ↔ timescale helpers & UI sync
       ============================== */
    /** Slider -> timeScale mapping: slider value is exponent*100, timeScale = 10^(val/100) */
    private double sliderValueToTimeScale(int sliderVal) {
        int clamped = Math.max(TIME_SLIDER_MIN, Math.min(sliderVal, TIME_SLIDER_MAX));
        double fraction = (clamped - TIME_SLIDER_MIN) / (double)(TIME_SLIDER_MAX - TIME_SLIDER_MIN);
        double ts = TIME_SCALE_MIN + fraction * (TIME_SCALE_MAX - TIME_SCALE_MIN);
        return Math.max(TIME_SCALE_MIN, Math.min(ts, TIME_SCALE_MAX));
    }

    private int timeScaleToSliderValue(double ts) {
        double clamped = Math.max(TIME_SCALE_MIN, Math.min(ts, TIME_SCALE_MAX));
        double fraction = (clamped - TIME_SCALE_MIN) / (TIME_SCALE_MAX - TIME_SCALE_MIN);
        return (int)Math.round(TIME_SLIDER_MIN + fraction * (TIME_SLIDER_MAX - TIME_SLIDER_MIN));
    }

    private void updateSliderFromTimeScale() {
        if (timeScaleSlider == null) return;
        suppressTimeSliderEvents = true;
        try {
            int val = timeScaleToSliderValue(timeScale);
            timeScaleSlider.setValue(val);
        } finally {
            suppressTimeSliderEvents = false;
        }
    }

    private void updateTimeScaleLabel() {
        if (timeScaleLabel == null) return;
        String prefix = simulationPaused ? "Paused" : "Time";
        timeScaleLabel.setText(prefix + " " + formatTimeScale(timeScale));
    }

    /**
     * Formats the timeScale (in seconds per real second) into a human-friendly
     * unit: s/s, min/s, hr/s, day/s, week/s, month/s, yr/s depending on magnitude.
     */
    private String formatTimeScale(double ts) {
        if (!Double.isFinite(ts)) ts = 0;
        final double MINUTE = 60.0;
        final double HOUR = 3600.0;
        final double WEEK = 7 * DAY;
        final double MONTH = 30 * DAY;
        // Choose the most suitable unit
        String unit;
        double value;
        if (ts < MINUTE) { // seconds per second
            unit = "s/s";
            value = ts;
        } else if (ts < HOUR) { // minutes per second
            unit = "min/s";
            value = ts / MINUTE;
        } else if (ts < DAY) { // hours per second
            unit = "hr/s";
            value = ts / HOUR;
        } else if (ts < WEEK) { // days per second
            unit = "day/s";
            value = ts / DAY;
        } else if (ts < MONTH) { // weeks per second
            unit = "week/s";
            value = ts / WEEK;
        } else if (ts < YEAR) { // months per second (approx 30 days)
            unit = "month/s";
            value = ts / MONTH;
        } else { // years per second
            unit = "yr/s";
            value = ts / YEAR;
        }
        // Use 3 decimals for small values, 2 for medium, 1 for large
        String fmt;
        double abs = Math.abs(value);
        if (abs < 1) fmt = "%.3f"; else if (abs < 10) fmt = "%.2f"; else fmt = "%.1f";
        return String.format(fmt + " %s", value, unit);
    }

    private void setTimeScaleAndSync(double newScale) {
        timeScale = Math.max(TIME_SCALE_MIN, Math.min(newScale, TIME_SCALE_MAX));
        updateSliderFromTimeScale();
        updateTimeScaleLabel();
    }

    /**
     * Gets the orbit color for a given planet based on its body type.
     */
    private Color getOrbitColor(int planetIndex) {
        if (planetIndex < 0 || planetIndex >= BODY_TYPE.length) {
            return new Color(100, 100, 100, 100);
        }
        String type = BODY_TYPE[planetIndex];
        return ORBIT_COLORS_BY_TYPE.getOrDefault(type, new Color(100, 100, 100, 100));
    }

    @Override
    public void actionPerformed(ActionEvent ev) {
        long now = System.nanoTime();
        double dtReal = (now - lastNanos) / 1e9;  // Real delta time (seconds)
        lastNanos = now;

        double dtSim = dtReal * timeScale;        // Simulated delta time
        simTimeSec += dtSim;

        // Update planet mean anomalies (Keplerian motion)
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

        // Solve Kepler's equation for planet positions
        for (int i = 0; i < NAMES.length; i++) {
            double a = A_METERS[i];
            double ecc = ECC[i];
            double omega = Math.toRadians(PLANET_VARPI0[i] - PLANET_OMEGA0[i]); // argument of perihelion
            double Omega = Math.toRadians(PLANET_OMEGA0[i]);                    // longitude of node
            Point2D.Double pos = computeInitialPosition(a, ecc, omega, Omega, meanAnomaly[i]);
            planetX[i] = pos.x;
            planetY[i] = pos.y;

            // Compute distance and approximate speed (vis-viva)
            planetDistM[i] = Math.hypot(planetX[i], planetY[i]);
            double r = planetDistM[i];
            double mu = G * (sunMass + PLANET_MASS[i]);
            double v = Math.sqrt(Math.max(0.0, mu * (2.0 / r - 1.0 / a)));
            planetSpeedKmS[i] = v / 1000.0;
        }

        // Compute barycenter of the system (Sun + planets) from current Kepler positions
        computeBarycenter();

        // Update launched objects with gravitational physics
        updateLaunchedObjects(dtSim);

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
            // Use barycentric coordinates for camera target
            double targetX = planetX[p] + sunBarycenterX + rx;
            double targetY = planetY[p] + sunBarycenterY + ry;

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
            // Use barycentric coordinates for camera target
            double targetX = planetX[cameraFocusIndex] + sunBarycenterX;
            double targetY = planetY[cameraFocusIndex] + sunBarycenterY;
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

    /** Compute system barycenter and set sunBarycenterX/Y accordingly. */
    private void computeBarycenter() {
        double totalPx = 0.0, totalPy = 0.0;
        for (int i = 0; i < NAMES.length; i++) {
            double m = PLANET_MASS[i];
            totalPx += m * planetX[i];
            totalPy += m * planetY[i];
        }
        // Sun offset so that center of mass is at origin: M_sun * r_sun + sum m_i r_i = 0
        sunBarycenterX = -totalPx / M_SUN;
        sunBarycenterY = -totalPy / M_SUN;
    }

    /**
     * Custom checkbox menu item that doesn't close the menu
     */
    static class StayOpenCheckBoxMenuItem extends JCheckBoxMenuItem {

        public StayOpenCheckBoxMenuItem(String text, boolean selected) {
            super(text, selected);
        }

        @Override
        protected void processMouseEvent(MouseEvent evt) {
            if (evt.getID() == MouseEvent.MOUSE_RELEASED) {
                // Capture current menu path BEFORE the default click closes menus.
                final MenuSelectionManager msm = MenuSelectionManager.defaultManager();
                final MenuElement[] oldPath = msm.getSelectedPath();

                // Let Swing do the normal toggle + ActionListener firing order.
                super.doClick(0);

                // Re-apply the previous selection path on the next tick to keep the menu open.
                SwingUtilities.invokeLater(() -> {
                    if (oldPath != null && oldPath.length > 0) {
                        msm.setSelectedPath(oldPath);
                    }
                });

                evt.consume();
            } else {
                super.processMouseEvent(evt);
            }
        }
    }

    /* ==============================
       MAIN
       ============================== */
    public static void main(String[] args) {
        JFrame frame = new JFrame("Solar System Simulation");
        frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        SolarSystemSimulation panel = new SolarSystemSimulation();
        // ----- Build professional-style UI frame -----
         // Top-level menu bar
         JMenuBar menuBar = new JMenuBar();

        // Simulation menu
        JMenu simMenu = new JMenu("Simulation");
        JMenuItem resetViewItem = new JMenuItem("Reset View to Sun");
        resetViewItem.addActionListener(_ -> {
            panel.followingPlanet = false;
            panel.followingMoon = false;
            panel.cameraFocusIndex = -1;
            panel.cameraFocusMoon = -1;
            panel.selectedPlanet = -1;
            panel.selectedMoon = -1;
            panel.camX = 0;
            panel.camY = 0;
            panel.currentZoom = 1.0;
        });
        JMenuItem exitItem = new JMenuItem("Exit");
        exitItem.addActionListener(_ -> System.exit(0));
        simMenu.add(resetViewItem);
        simMenu.addSeparator();
        simMenu.add(exitItem);

        // View menu – mirrors keyboard toggles
        JMenu viewMenu = new JMenu("View");
        StayOpenCheckBoxMenuItem orbitsItem = new StayOpenCheckBoxMenuItem("Show planet orbits", true);
        orbitsItem.addActionListener(_ -> {
            panel.showPlanetOrbits = orbitsItem.isSelected();
            panel.repaint();
        });
        StayOpenCheckBoxMenuItem ringsItem = new StayOpenCheckBoxMenuItem("Show Saturn rings", true);
        ringsItem.addActionListener(_ -> {
            panel.showRings = ringsItem.isSelected();
            panel.repaint();
        });
        StayOpenCheckBoxMenuItem labelsItem = new StayOpenCheckBoxMenuItem("Show labels", true);
        labelsItem.addActionListener(_ -> {
            panel.showLabels = labelsItem.isSelected();
            panel.repaint();
        });
        StayOpenCheckBoxMenuItem shadingItem = new StayOpenCheckBoxMenuItem("Enable shading", true);
        shadingItem.addActionListener(_ -> {
            panel.shadingEnabled = shadingItem.isSelected();
            panel.repaint();
        });
        StayOpenCheckBoxMenuItem selectedOnlyItem = new StayOpenCheckBoxMenuItem("Show only selected planet", false);
        selectedOnlyItem.addActionListener(_ -> {
            panel.showOnlySelected = selectedOnlyItem.isSelected();
            panel.repaint();
        });
        viewMenu.add(orbitsItem);
        viewMenu.add(ringsItem);
        viewMenu.add(labelsItem);
        viewMenu.add(shadingItem);
        viewMenu.addSeparator();
        viewMenu.add(selectedOnlyItem);

        // Add planet visibility submenu
        viewMenu.addSeparator();
        JMenu planetsMenu = new JMenu("Planets");
        for (int i = 0; i < NAMES.length; i++) {
            final int planetIndex = i;
            // Dwarf planets (except Pluto) start unchecked
            boolean initialState = panel.planetVisible[i];
            StayOpenCheckBoxMenuItem checkbox = new StayOpenCheckBoxMenuItem(NAMES[i], initialState);
            checkbox.addActionListener(_ -> {
                panel.planetVisible[planetIndex] = checkbox.isSelected();
                panel.updateTreeVisibility();
                panel.repaint();
            });
            planetsMenu.add(checkbox);
        }
        viewMenu.add(planetsMenu);

        // Add asteroid/debris belts submenu
        viewMenu.addSeparator();
        JMenu beltsMenu = new JMenu("Belts & Lines");
        StayOpenCheckBoxMenuItem asteroidBeltItem = new StayOpenCheckBoxMenuItem("Asteroid Belt", false);
        asteroidBeltItem.addActionListener(_ -> {
            panel.showAsteroidBelt = asteroidBeltItem.isSelected();
            panel.repaint();
        });
        StayOpenCheckBoxMenuItem kuiperBeltItem = new StayOpenCheckBoxMenuItem("Kuiper Belt", false);
        kuiperBeltItem.addActionListener(_ -> {
            panel.showKuiperBelt = kuiperBeltItem.isSelected();
            panel.repaint();
        });
        StayOpenCheckBoxMenuItem iceLinesItem = new StayOpenCheckBoxMenuItem("Frost Lines", false);
        iceLinesItem.addActionListener(_ -> {
            panel.showIceLines = iceLinesItem.isSelected();
            panel.repaint();
        });
        beltsMenu.add(asteroidBeltItem);
        beltsMenu.add(kuiperBeltItem);
        beltsMenu.add(iceLinesItem);
        viewMenu.add(beltsMenu);
        JMenu helpMenu = new JMenu("Help");
        JMenuItem controlsItem = new JMenuItem("Controls...");
        controlsItem.addActionListener(_ -> JOptionPane.showMessageDialog(
                frame,
                """
                        Controls:
                        - Mouse wheel: Zoom in/out
                        - Ctrl + wheel: Change time speed
                        - Click planet: Follow/unfollow
                        - Click moon (while following planet): Follow/unfollow moon
                        - Z: Toggle auto-zoom
                        - X: Cycle zoom mode (Planet/Orbit/Manual)
                        - O: Toggle planet orbits
                        - R: Toggle Saturn rings
                        - L: Toggle labels
                        - B: Show only selected planet
                        - S: Toggle shading
                        
                        Use View menu for:
                        - Asteroid Belt
                        - Kuiper Belt
                        - Frost Lines (H2O and CO2)""",
                "Controls",
                JOptionPane.INFORMATION_MESSAGE));
        JMenuItem aboutItem = new JMenuItem("About");
        aboutItem.addActionListener(_ -> JOptionPane.showMessageDialog(
                frame,
                "Solar System Simulation\n" +
                        "Interactive 2-D Keplerian model with moons and auto-zoom.",
                "About",
                JOptionPane.INFORMATION_MESSAGE));
        helpMenu.add(controlsItem);
        helpMenu.addSeparator();
        helpMenu.add(aboutItem);

        menuBar.add(simMenu);
        menuBar.add(viewMenu);
        JMenu launchMenu = new JMenu("Launch");
        JMenuItem launchItem = new JMenuItem("Launch object...");
        launchItem.addActionListener(_ -> panel.showLaunchDialog(frame));
        JMenuItem quickLaunchItem = new JMenuItem("Quick launch (F)");
        quickLaunchItem.addActionListener(_ -> panel.quickLaunchDefault());
        launchMenu.add(launchItem);
        launchMenu.add(quickLaunchItem);
        menuBar.add(launchMenu);
        menuBar.add(helpMenu);
        frame.setJMenuBar(menuBar);

        // ----- Top search bar -----
        JPanel topBar = new JPanel(new BorderLayout(8, 0));
        topBar.setBorder(BorderFactory.createEmptyBorder(4, 8, 4, 8));
        JLabel searchLabel = new JLabel("Search body:");
        panel.searchField = new JTextField();
        panel.searchField.setToolTipText("Type a planet or major moon name (e.g., Earth, Titan)");

        // Ensure panel can still receive key events when search is not focused
        panel.setFocusTraversalKeysEnabled(false);

        // Build suggestion popup for search
        panel.searchSuggestionsPopup = new JPopupMenu();
        panel.searchSuggestionsPopup.setFocusable(false);
        panel.searchSuggestionsList = new JList<>();
        panel.searchSuggestionsList.setSelectionMode(ListSelectionModel.SINGLE_SELECTION);
        panel.searchSuggestionsList.setFocusable(false);
        JScrollPane suggScroll = new JScrollPane(panel.searchSuggestionsList);
        suggScroll.setFocusable(false);
        suggScroll.setPreferredSize(new Dimension(220, 140));
        panel.searchSuggestionsPopup.add(suggScroll);

        // Precompute all searchable names (planets + moons)
        java.util.List<String> allNames = new java.util.ArrayList<>();
        Collections.addAll(allNames, NAMES);
        for (int i = 0; i < NAMES.length; i++) {
            Collections.addAll(allNames, MOON_NAMES[i]);
        }

        // Update suggestions as the user types
        panel.searchField.getDocument().addDocumentListener(new javax.swing.event.DocumentListener() {
            private void updateSuggestions() {
                String text = panel.searchField.getText();
                if (text == null) text = "";
                String q = text.trim().toLowerCase();
                if (q.isEmpty()) {
                    panel.searchSuggestionsPopup.setVisible(false);
                    return;
                }
                java.util.List<String> matches = new java.util.ArrayList<>();
                for (String name : allNames) {
                    if (name.toLowerCase().startsWith(q)) {
                        matches.add(name);
                    }
                }
                if (matches.isEmpty()) {
                    panel.searchSuggestionsPopup.setVisible(false);
                    return;
                }
                panel.searchSuggestionsList.setListData(matches.toArray(new String[0]));
                try {
                    Rectangle r = panel.searchField.getBounds();
                    panel.searchSuggestionsPopup.show(panel.searchField, 0, r.height);
                } catch (Exception ignored) {}
            }
            @Override public void insertUpdate(javax.swing.event.DocumentEvent e) { updateSuggestions(); }
            @Override public void removeUpdate(javax.swing.event.DocumentEvent e) { updateSuggestions(); }
            @Override public void changedUpdate(javax.swing.event.DocumentEvent e) { updateSuggestions(); }
        });

        // Clicking a suggestion uses that body
        panel.searchSuggestionsList.addMouseListener(new MouseAdapter() {
            @Override public void mouseClicked(MouseEvent e) {
                if (e.getClickCount() == 2) {
                    String sel = panel.searchSuggestionsList.getSelectedValue();
                    if (sel != null) {
                        panel.searchField.setText(sel);
                        panel.searchSuggestionsPopup.setVisible(false);
                        panel.searchAndFocus();
                    }
                }
            }
        });


        topBar.add(searchLabel, BorderLayout.WEST);
        topBar.add(panel.searchField, BorderLayout.CENTER);

        JButton searchButton = new JButton("Go");
        searchButton.addActionListener(_ -> panel.searchAndFocus());
        panel.searchField.addActionListener(_ -> panel.searchAndFocus());

        // Add key listener to search field to handle Escape and forward global keys
        panel.searchField.addKeyListener(new KeyAdapter() {
            @Override public void keyPressed(KeyEvent e) {
                int code = e.getKeyCode();

                // Check if this is a global keybind that should be forwarded to panel
                boolean isGlobalKeybind = (code == KeyEvent.VK_Z || code == KeyEvent.VK_X || code == KeyEvent.VK_S ||
                                          code == KeyEvent.VK_O || code == KeyEvent.VK_R || code == KeyEvent.VK_L ||
                                          code == KeyEvent.VK_B || code == KeyEvent.VK_SPACE || code == KeyEvent.VK_P);

                // Handle search-specific keys
                if (!panel.searchSuggestionsPopup.isVisible()) {
                    // Popup not visible - handle Escape to clear and return focus
                    if (code == KeyEvent.VK_ESCAPE) {
                        panel.searchField.setText("");
                        panel.searchSuggestionsPopup.setVisible(false);
                        panel.requestFocusInWindow();
                        e.consume();
                        return;
                    }
                } else {
                    // Popup is visible - handle navigation
                    if (code == KeyEvent.VK_DOWN) {
                        int idx = panel.searchSuggestionsList.getSelectedIndex();
                        if (idx < panel.searchSuggestionsList.getModel().getSize() - 1) {
                            panel.searchSuggestionsList.setSelectedIndex(idx + 1);
                            panel.searchSuggestionsList.ensureIndexIsVisible(idx + 1);
                        }
                        e.consume();
                        return;
                    } else if (code == KeyEvent.VK_UP) {
                        int idx = panel.searchSuggestionsList.getSelectedIndex();
                        if (idx > 0) {
                            panel.searchSuggestionsList.setSelectedIndex(idx - 1);
                            panel.searchSuggestionsList.ensureIndexIsVisible(idx - 1);
                        }
                        e.consume();
                        return;
                    } else if (code == KeyEvent.VK_ENTER) {
                        String sel = panel.searchSuggestionsList.getSelectedValue();
                        if (sel != null) {
                            panel.searchField.setText(sel);
                        }
                        panel.searchSuggestionsPopup.setVisible(false);
                        panel.searchAndFocus();
                        e.consume();
                        return;
                    } else if (code == KeyEvent.VK_ESCAPE) {
                        panel.searchSuggestionsPopup.setVisible(false);
                        e.consume();
                        return;
                    }
                }

                // Forward global keybinds to the panel and prevent them from being typed
                if (isGlobalKeybind) {
                    // Forward to panel by creating a synthetic event
                    KeyEvent synthetic = new KeyEvent(panel, KeyEvent.KEY_PRESSED, System.currentTimeMillis(),
                            e.getModifiersEx(), code, e.getKeyChar());
                    panel.dispatchEvent(synthetic);
                    e.consume();  // Consume the event so it's not typed into the search field
                }
            }
        });

        topBar.add(searchButton, BorderLayout.EAST);

        // Add mouse listener to top bar to close search when clicking outside search field
        topBar.addMouseListener(new MouseAdapter() {
            @Override public void mouseClicked(MouseEvent e) {
                // If click is not on search field, close the popup
                if (e.getSource() != panel.searchField && !panel.isSearchUIComponent(e.getComponent())) {
                    if (panel.searchSuggestionsPopup != null) {
                        panel.searchSuggestionsPopup.setVisible(false);
                    }
                }
            }
        });

        // Main content with top bar + simulation panel
        JPanel content = new JPanel(new BorderLayout());
        content.add(topBar, BorderLayout.NORTH);

        // Left navigation: hierarchical planet→moon tree
        JPanel leftNav = new JPanel(new BorderLayout());
        leftNav.setBorder(BorderFactory.createEmptyBorder(8, 8, 8, 4));
        JLabel navLabel = new JLabel("Objects");
        navLabel.setBorder(BorderFactory.createEmptyBorder(0, 0, 4, 0));
        JPanel treePanel = new JPanel(new BorderLayout());
        treePanel.add(navLabel, BorderLayout.NORTH);

        // Build tree nodes
        DefaultMutableTreeNode root = new DefaultMutableTreeNode("Solar System");
        panel.treeRootNode = root;
        panel.planetTreePaths = new TreePath[NAMES.length];
        panel.planetTreeNodes = new DefaultMutableTreeNode[NAMES.length];
        panel.moonTreePaths = new TreePath[NAMES.length][];
        for (int i = 0; i < NAMES.length; i++) {
            DefaultMutableTreeNode planetNode = new DefaultMutableTreeNode(new TreeBody("planet", i, -1, NAMES[i]));
            panel.planetTreeNodes[i] = planetNode;
            // Only add to tree if planet is visible
            if (panel.planetVisible[i]) {
                root.add(planetNode);
            }
            panel.planetTreePaths[i] = new TreePath(new Object[]{root, planetNode});
            panel.moonTreePaths[i] = new TreePath[MOON_NAMES[i].length];
            for (int m = 0; m < MOON_NAMES[i].length; m++) {
                DefaultMutableTreeNode moonNode = new DefaultMutableTreeNode(new TreeBody("moon", i, m, MOON_NAMES[i][m]));
                planetNode.add(moonNode);
                panel.moonTreePaths[i][m] = new TreePath(new Object[]{root, planetNode, moonNode});
            }
        }
        panel.bodyTreeModel = new DefaultTreeModel(root);
        panel.bodyTree = new JTree(panel.bodyTreeModel);
        panel.bodyTree.setRootVisible(true);
        panel.bodyTree.setShowsRootHandles(true);
        panel.bodyTree.setCellRenderer(new BodyTreeRenderer());
        panel.bodyTree.getSelectionModel().setSelectionMode(TreeSelectionModel.SINGLE_TREE_SELECTION);
        for (int i = 0; i < panel.bodyTree.getRowCount(); i++) {
            panel.bodyTree.expandRow(i);
        }
        panel.bodyTree.addTreeSelectionListener(e -> panel.handleTreeSelection(e.getPath()));
        JScrollPane treeScroll = new JScrollPane(panel.bodyTree);
        treeScroll.setPreferredSize(new Dimension(220, 0));
        treePanel.add(treeScroll, BorderLayout.CENTER);

        leftNav.add(treePanel, BorderLayout.CENTER);

        content.add(leftNav, BorderLayout.WEST);
        content.add(panel, BorderLayout.CENTER);
        //
        // Bottom: time controls (play/pause, fine slider, presets, label)
        JPanel timeControlsPanel = new JPanel();
        timeControlsPanel.setBorder(BorderFactory.createEmptyBorder(8, 8, 8, 8));
        timeControlsPanel.setLayout(new GridBagLayout());
        GridBagConstraints timeGbc = new GridBagConstraints();
        timeGbc.fill = GridBagConstraints.HORIZONTAL;

        JButton playPause = new JButton(panel.simulationPaused ? "Play" : "Pause");
        playPause.addActionListener(_ -> {
            panel.simulationPaused = !panel.simulationPaused;
            playPause.setText(panel.simulationPaused ? "Play" : "Pause");
            panel.updateTimeScaleLabel();
        });

        panel.timeScaleSlider = new JSlider(TIME_SLIDER_MIN, TIME_SLIDER_MAX,
                panel.timeScaleToSliderValue(panel.timeScale));
        panel.timeScaleSlider.setMajorTickSpacing((TIME_SLIDER_MAX - TIME_SLIDER_MIN) / 5);
        panel.timeScaleSlider.setMinorTickSpacing((TIME_SLIDER_MAX - TIME_SLIDER_MIN) / 20);
        panel.timeScaleSlider.setPaintTicks(false);
        panel.timeScaleSlider.setPreferredSize(new Dimension(220, 32));
        panel.timeScaleSlider.addChangeListener(_ -> {
            if (panel.suppressTimeSliderEvents) return;
            panel.timeScale = panel.sliderValueToTimeScale(panel.timeScaleSlider.getValue());
            panel.updateTimeScaleLabel();
        });

        timeGbc.gridx = 0; timeGbc.gridy = 0;
        timeGbc.weightx = 0.33;
        timeGbc.insets = new Insets(0, 0, 0, 6);
        timeControlsPanel.add(playPause, timeGbc);
        timeGbc = new GridBagConstraints();
        timeGbc.gridx = 1; timeGbc.gridy = 0;
        timeGbc.weightx = 0.67;
        timeGbc.fill = GridBagConstraints.HORIZONTAL;
        timeControlsPanel.add(panel.timeScaleSlider, timeGbc);

        JPanel presetGrid = new JPanel(new GridLayout(2, 3, 4, 4));
        presetGrid.setOpaque(false);
        JButton oneSecondBtn = new JButton("1 s/s");
        oneSecondBtn.addActionListener(_ -> panel.setTimeScaleAndSync(TIME_SCALE_MIN));
        JButton oneMinuteBtn = new JButton("1 min/s");
        oneMinuteBtn.addActionListener(_ -> panel.setTimeScaleAndSync(60.0));
        JButton oneHourBtn = new JButton("1 hr/s");
        oneHourBtn.addActionListener(_ -> panel.setTimeScaleAndSync(3600.0));
        JButton dayBtn = new JButton("1 day/s");
        dayBtn.addActionListener(_ -> panel.setTimeScaleAndSync(DAY));
        JButton weekBtn = new JButton("1 week/s");
        weekBtn.addActionListener(_ -> panel.setTimeScaleAndSync(7 * DAY));
        JButton monthBtn = new JButton("1 month/s");
        monthBtn.addActionListener(_ -> panel.setTimeScaleAndSync(30 * DAY));
        JButton threeMonthBtn = new JButton("3 months/s");
        threeMonthBtn.addActionListener(_ -> panel.setTimeScaleAndSync(90 * DAY));
        JButton yearBtn = new JButton("1 year/s");
        yearBtn.addActionListener(_ -> panel.setTimeScaleAndSync(YEAR));
        JButton decadeBtn = new JButton("10 years/s");
        decadeBtn.addActionListener(_ -> panel.setTimeScaleAndSync(10 * YEAR));
        presetGrid.add(oneSecondBtn);
        presetGrid.add(oneMinuteBtn);
        presetGrid.add(oneHourBtn);
        presetGrid.add(dayBtn);
        presetGrid.add(weekBtn);
        presetGrid.add(monthBtn);
        presetGrid.add(threeMonthBtn);
        presetGrid.add(yearBtn);
        presetGrid.add(decadeBtn);
        timeGbc = new GridBagConstraints();
        timeGbc.gridx = 0; timeGbc.gridy = 1;
        timeGbc.gridwidth = 2;
        timeGbc.insets = new Insets(4, 0, 0, 0);
        timeControlsPanel.add(presetGrid, timeGbc);

        panel.timeScaleLabel = new JLabel();
        panel.updateTimeScaleLabel();
        timeGbc = new GridBagConstraints();
        timeGbc.gridx = 0; timeGbc.gridy = 2;
        timeGbc.gridwidth = 2;
        timeGbc.insets = new Insets(4, 0, 0, 0);
        timeGbc.anchor = GridBagConstraints.WEST;
        timeControlsPanel.add(panel.timeScaleLabel, timeGbc);
        timeGbc = new GridBagConstraints();
        timeGbc.gridx = 0; timeGbc.gridy = 3;
        timeGbc.gridwidth = 2;
        timeGbc.weighty = 1.0;
        timeControlsPanel.add(Box.createVerticalGlue(), timeGbc);

        JSplitPane leftSplit = new JSplitPane(JSplitPane.VERTICAL_SPLIT, treePanel, timeControlsPanel);
        leftSplit.setResizeWeight(0.9);
        leftSplit.setDividerLocation(0.9);
         leftSplit.setContinuousLayout(true);
         leftSplit.setDividerSize(6);
         leftSplit.setBorder(null);
         leftNav.add(leftSplit, BorderLayout.CENTER);

        content.add(leftNav, BorderLayout.WEST);
        content.add(panel, BorderLayout.CENTER);

        frame.setContentPane(content);

        // Bind global actions at window level (RootPane) for launching/deleting
        JRootPane rootPane = frame.getRootPane();
        InputMap rim = rootPane.getInputMap(JComponent.WHEN_IN_FOCUSED_WINDOW);
        ActionMap ram = rootPane.getActionMap();
        rim.put(KeyStroke.getKeyStroke(KeyEvent.VK_F, 0), "rootLaunchDefault");
        ram.put("rootLaunchDefault", new AbstractAction() {
            @Override public void actionPerformed(ActionEvent e) { panel.quickLaunchDefault(); }
        });
        rim.put(KeyStroke.getKeyStroke(KeyEvent.VK_BACK_SPACE, 0), "rootDeleteLastLaunched");
        ram.put("rootDeleteLastLaunched", new AbstractAction() {
            @Override public void actionPerformed(ActionEvent e) {
                if (!panel.launchedObjects.isEmpty()) {
                    panel.launchedObjects.removeLast();
                    panel.repaint();
                }
            }
        });

        frame.pack();
        frame.setLocationRelativeTo(null);
        frame.setVisible(true);
         // Display Sun mass in title for context
         frame.setTitle("Solar System Simulation (Press M: planet mass, U: Sun mass)");
     }

    // Simple search function: focus camera on a planet or moon by name
    private void searchAndFocus() {
        if (searchField == null) return;
        String query = searchField.getText();
        if (query == null) return;
        query = query.trim();
        if (query.isEmpty()) return;

        String qLower = query.toLowerCase();

        // 1) Try planets
        for (int i = 0; i < NAMES.length; i++) {
            if (NAMES[i].toLowerCase().equals(qLower)) {
                // Check if planet is visible
                if (!planetVisible[i]) {
                    JOptionPane.showMessageDialog(this,
                            NAMES[i] + " is currently hidden.\n\n" +
                            "Enable it in View > Planets > " + NAMES[i] + " to view it.",
                            "Object Hidden",
                            JOptionPane.INFORMATION_MESSAGE);
                    return;
                }
                followingPlanet = true;
                followingMoon = false;
                cameraFocusIndex = i;
                selectedPlanet = i;
                selectedMoon = -1;
                cameraFocusMoon = -1;
                autoZoomEnabled = true;
                focusPlanetImmediate(i);
                return;
            }
        }

        // 2) Try moons (check parent planet visibility)
        for (int i = 0; i < NAMES.length; i++) {
            for (int m = 0; m < MOON_NAMES[i].length; m++) {
                if (MOON_NAMES[i][m].toLowerCase().equals(qLower)) {
                    // Check if parent planet is visible
                    if (!planetVisible[i]) {
                        JOptionPane.showMessageDialog(this,
                                MOON_NAMES[i][m] + " (moon of " + NAMES[i] + ") is currently hidden.\n\n" +
                                "Enable " + NAMES[i] + " in View > Planets > " + NAMES[i] + " to view it.",
                                "Object Hidden",
                                JOptionPane.INFORMATION_MESSAGE);
                        return;
                    }
                    followingPlanet = true;
                    followingMoon = true;
                    cameraFocusIndex = i;
                    selectedPlanet = i;
                    cameraFocusMoon = m;
                    selectedMoon = m;
                    autoZoomEnabled = true;
                    focusMoonImmediate(i, m);
                    return;
                }
            }
        }

        // Nothing found – optional feedback: briefly flash background or use dialog
        JOptionPane.showMessageDialog(this,
                "'" + query + "' not found.\n\n" +
                "Try searching for a planet or moon name\n" +
                "(e.g., Earth, Mars, Titan, Europa)",
                "Object Not Found",
                JOptionPane.INFORMATION_MESSAGE);
    }

    private void exitSearchModeIfActive() {
        if (searchField == null) return;
        if (searchSuggestionsPopup != null) {
            searchSuggestionsPopup.setVisible(false);
        }
    }

    private boolean isSearchUIComponent(Component c) {
        if (c == null) return false;
        if (c == searchField) return true;
        if (searchSuggestionsPopup != null) {
            if (c == searchSuggestionsPopup) return true;
            return SwingUtilities.isDescendingFrom(c, searchSuggestionsPopup);
        }
        return false;
    }

    private void focusPlanetImmediate(int planetIndex) {
        if (planetIndex < 0 || planetIndex >= NAMES.length) return;
        // Use barycentric coordinates
        camX = planetX[planetIndex] + sunBarycenterX;
        camY = planetY[planetIndex] + sunBarycenterY;
        moonCamX = camX;
        moonCamY = camY;
        cameraIsApproaching = false;
        moonCameraIsApproaching = false;
        if (autoZoomEnabled) {
            updateAutoZoomTarget();
            currentZoom = targetZoom;
        }
        if (bodyTree != null) syncTreeSelection();
    }

    private void focusMoonImmediate(int planetIndex, int moonIndex) {
        if (planetIndex < 0 || planetIndex >= NAMES.length) return;
        if (moonIndex < 0 || moonIndex >= MOON_A_M[planetIndex].length) return;
        double a = MOON_A_M[planetIndex][moonIndex];
        double e = MOON_ECC[planetIndex][moonIndex];
        double b = a * Math.sqrt(1 - e * e);
        double M = moonMean[planetIndex][moonIndex];
        double E = M;
        for (int it = 0; it < 40; it++) {
            double f = E - e * Math.sin(E) - M;
            double fp = 1 - e * Math.cos(E);
            E -= f / fp;
            if (Math.abs(f) < 1e-12) break;
        }
        double mx = a * (Math.cos(E) - e);
        double my = b * Math.sin(E);
        double arg = MOON_ARG_PERI[planetIndex][moonIndex];
        double rx = mx * Math.cos(arg) - my * Math.sin(arg);
        double ry = mx * Math.sin(arg) + my * Math.cos(arg);
        // Use barycentric coordinates
        double targetX = planetX[planetIndex] + sunBarycenterX + rx;
        double targetY = planetY[planetIndex] + sunBarycenterY + ry;
        moonCamX = targetX;
        moonCamY = targetY;
        camX = moonCamX;
        camY = moonCamY;
        moonCameraIsApproaching = false;
        cameraIsApproaching = false;
        autoZoomEnabled = true;
        double moonRadius = MOON_RADIUS_M[planetIndex][moonIndex];
        double maxRadius = moonRadius * 1.5;
        int screen = Math.min(getWidth(), getHeight());
        double desired = screen * 0.6;
        targetZoom = desired / (maxRadius * METERS_TO_PIXELS * 2);
        targetZoom = Math.max(50.0, Math.min(targetZoom, 50000.0));
        currentZoom = targetZoom;
        if (bodyTree != null) syncTreeSelection();
    }

    private void handleTreeSelection(TreePath path) {
        if (suppressTreeSelection || path == null) return;
        Object last = ((DefaultMutableTreeNode) path.getLastPathComponent()).getUserObject();
        if (!(last instanceof TreeBody body)) return;
        if ("planet".equals(body.type)) {
            followingPlanet = true;
            followingMoon = false;
            cameraFocusIndex = body.planetIndex;
            selectedPlanet = body.planetIndex;
            selectedMoon = -1;
            cameraFocusMoon = -1;
            autoZoomEnabled = true;
            focusPlanetImmediate(body.planetIndex);
        } else if ("moon".equals(body.type)) {
            followingPlanet = true;
            followingMoon = true;
            cameraFocusIndex = body.planetIndex;
            selectedPlanet = body.planetIndex;
            cameraFocusMoon = body.moonIndex;
            selectedMoon = body.moonIndex;
            autoZoomEnabled = true;
            focusMoonImmediate(body.planetIndex, body.moonIndex);
        }
        repaint();
    }

    private void syncTreeSelection() {
        if (bodyTree == null) return;
        suppressTreeSelection = true;
        try {
            if (followingMoon && cameraFocusIndex >= 0 && cameraFocusMoon >= 0) {
                bodyTree.setSelectionPath(moonTreePaths[cameraFocusIndex][cameraFocusMoon]);
                bodyTree.scrollPathToVisible(moonTreePaths[cameraFocusIndex][cameraFocusMoon]);
            } else if (followingPlanet && cameraFocusIndex >= 0) {
                bodyTree.setSelectionPath(planetTreePaths[cameraFocusIndex]);
                bodyTree.scrollPathToVisible(planetTreePaths[cameraFocusIndex]);
            } else {
                bodyTree.clearSelection();
            }
        } finally {
            suppressTreeSelection = false;
        }
    }

    /**
     * Updates the tree to show/hide planets based on planetVisible array
     */
    private void updateTreeVisibility() {
        if (bodyTreeModel == null || treeRootNode == null || planetTreeNodes == null) return;

        for (int i = 0; i < NAMES.length; i++) {
            DefaultMutableTreeNode planetNode = planetTreeNodes[i];
            boolean isCurrentlyInTree = treeRootNode.isNodeChild(planetNode);
            boolean shouldBeVisible = planetVisible[i];

            if (shouldBeVisible && !isCurrentlyInTree) {
                // Add planet to tree
                // Find correct position (maintain original order)
                int insertIndex = 0;
                for (int j = 0; j < i; j++) {
                    if (planetVisible[j]) {
                        insertIndex++;
                    }
                }
                bodyTreeModel.insertNodeInto(planetNode, treeRootNode, insertIndex);
            } else if (!shouldBeVisible && isCurrentlyInTree) {
                // Remove planet from tree
                bodyTreeModel.removeNodeFromParent(planetNode);
            }
        }

        // Expand all rows to show moons
        for (int i = 0; i < bodyTree.getRowCount(); i++) {
            bodyTree.expandRow(i);
        }
    }

    // ----- Launchable projectiles -----
    private static final double TRAIL_DISTANCE_THRESHOLD = 5e8;
    private static final int MAX_TRAIL_POINTS = 800;

    private void showLaunchDialog(Window owner) {
        JDialog dialog = new JDialog(owner, "Launch Object", Dialog.ModalityType.APPLICATION_MODAL);
        JPanel form = new JPanel(new GridBagLayout());
        GridBagConstraints gbc = new GridBagConstraints();
        gbc.insets = new Insets(4, 4, 4, 4);
        gbc.fill = GridBagConstraints.HORIZONTAL;

        JTextField nameField = new JTextField("Custom object", 14);
        JTextField massField = new JTextField("1000");
        JTextField radiusField = new JTextField("500");
        JTextField speedField = new JTextField("20");
        String[] speedUnits = {"km/s", "m/s"};
        JComboBox<String> speedUnitCombo = new JComboBox<>(speedUnits);
        speedUnitCombo.setSelectedIndex(0);
        JTextField angleField = new JTextField("0");

        final Color[] selectedColor = {new Color(0x55CCFF)};
        final LaunchedShape[] selectedShape = {LaunchedShape.CIRCLE};
        final double[] selectedWidth = {0};
        final double[] selectedHeight = {0};

        JButton colorButton = new JButton("Color");
        colorButton.setBackground(selectedColor[0]);
        colorButton.setOpaque(true);
        colorButton.addActionListener(_ -> {
            Color chosen = JColorChooser.showDialog(dialog, "Pick object color", selectedColor[0]);
            if (chosen != null) {
                selectedColor[0] = chosen;
                colorButton.setBackground(chosen);
            }
        });

        gbc.gridx = 0; gbc.gridy = 0;
        form.add(new JLabel("Name:"), gbc);
        gbc.gridx = 1;
        form.add(nameField, gbc);
        gbc.gridx = 0; gbc.gridy++;
        form.add(new JLabel("Mass (kg):"), gbc);
        gbc.gridx = 1;
        form.add(massField, gbc);
        gbc.gridx = 0; gbc.gridy++;
        form.add(new JLabel("Radius (m):"), gbc);
        gbc.gridx = 1;
        form.add(radiusField, gbc);
        gbc.gridx = 0; gbc.gridy++;
        form.add(new JLabel("Speed (value):"), gbc);
        gbc.gridx = 1;
        form.add(speedField, gbc);
        gbc.gridx = 0; gbc.gridy++;
        form.add(new JLabel("Speed unit:"), gbc);
        gbc.gridx = 1;
        form.add(speedUnitCombo, gbc);
        gbc.gridx = 0; gbc.gridy++;
        form.add(new JLabel("Angle (deg):"), gbc);
        gbc.gridx = 1;
        form.add(angleField, gbc);
        gbc.gridx = 0; gbc.gridy++;
        form.add(new JLabel("Trail color:"), gbc);
        gbc.gridx = 1;
        form.add(colorButton, gbc);
        gbc.gridx = 0; gbc.gridy++;
        form.add(new JLabel("Angle: 0° = +X, 90° = +Y (CCW)"), gbc);

        java.util.List<LaunchPreset> presets = java.util.List.of(
                new LaunchPreset("Photon", 0, 0, 0, 0, 300_000, 25, new Color(156, 114, 255), LaunchedShape.CIRCLE));
        JPanel presetPanel = new JPanel(new GridLayout(1, presets.size(), 4, 4));
        for (LaunchPreset preset : presets) {
            JButton presetBtn = new JButton(preset.name);
            presetBtn.addActionListener(_ -> {
                nameField.setText(preset.name);
                massField.setText(Double.toString(preset.massKg));
                radiusField.setText(Double.toString(preset.radiusM));
                speedField.setText(Double.toString(preset.speedKmS));
                speedUnitCombo.setSelectedItem("km/s");
                angleField.setText(Double.toString(preset.angleDeg));
                selectedColor[0] = preset.color;
                selectedShape[0] = preset.shape;
                selectedWidth[0] = preset.widthM;
                selectedHeight[0] = preset.heightM;
                colorButton.setBackground(preset.color);
            });
            presetPanel.add(presetBtn);
        }

        JPanel buttons = new JPanel(new FlowLayout(FlowLayout.RIGHT));
        JButton launchBtn = new JButton("Launch");
        launchBtn.addActionListener(_ -> {
            try {
                String name = nameField.getText().trim();
                if (name.isEmpty()) name = "Custom object";
                double massKg = Math.max(1.0, Double.parseDouble(massField.getText()));
                double radiusM = Math.max(1.0, Double.parseDouble(radiusField.getText()));
                double speedVal = Double.parseDouble(speedField.getText());
                String unit = (String) speedUnitCombo.getSelectedItem();
                double speedKmS = "m/s".equals(unit) ? (speedVal / 1000.0) : speedVal;
                double angleDeg = Double.parseDouble(angleField.getText());
                launchPresetObject(name, massKg, radiusM,
                        selectedWidth[0], selectedHeight[0],
                        speedKmS, angleDeg, selectedColor[0], selectedShape[0]);
                dialog.dispose();
            } catch (NumberFormatException ex) {
                JOptionPane.showMessageDialog(dialog,
                        "Enter numeric values for mass, radius, speed, and angle.",
                        "Invalid input", JOptionPane.ERROR_MESSAGE);
            }
        });
        JButton cancelBtn = new JButton("Cancel");
        cancelBtn.addActionListener(_ -> dialog.dispose());
        buttons.add(cancelBtn);
        buttons.add(launchBtn);

        JPanel container = new JPanel(new BorderLayout(0, 8));
        container.setBorder(BorderFactory.createEmptyBorder(10, 10, 10, 10));
        container.add(form, BorderLayout.NORTH);
        container.add(presetPanel, BorderLayout.CENTER);
        container.add(buttons, BorderLayout.SOUTH);
        dialog.setContentPane(container);
        dialog.pack();
        dialog.setResizable(false);
        dialog.setLocationRelativeTo(owner);
        dialog.setVisible(true);
    }

    private void launchPresetObject(String name, double massKg, double radiusM,
                                    double widthM, double heightM,
                                    double speedKmS, double angleDeg,
                                    Color color, LaunchedShape shape) {
        double speedMs = speedKmS * 1_000.0;
        double theta = Math.toRadians(angleDeg);
        double vx = Math.cos(theta) * speedMs;
        double vy = Math.sin(theta) * speedMs;
        LaunchedObject obj = new LaunchedObject(name, massKg, radiusM, widthM, heightM, color, shape);
        obj.x = 0;
        obj.y = 0;
        obj.vx = vx;
        obj.vy = vy;
        obj.trail.add(new Point2D.Double(obj.x, obj.y));
        launchedObjects.add(obj);
        repaint();
    }

    /** Quick-launch a default test object (used by keybind 'F'). */
    private void quickLaunchDefault() {
        launchPresetObject(
                "Test Object",
                1_000,
                500,
                0, 0,
                20,
                0,
                new Color(0x55, 0xCC, 0xFF),
                LaunchedShape.CIRCLE
        );
    }

    private void updateLaunchedObjects(double dtSim) {
        if (dtSim <= 0 || launchedObjects.isEmpty()) return;
        for (LaunchedObject obj : launchedObjects) {
            double ax = 0, ay = 0;
            double dxSun = -obj.x;
            double dySun = -obj.y;
            double invSun = inverseR3(dxSun, dySun);
            double sunFactor = G * sunMass * invSun;
            ax += dxSun * sunFactor;
            ay += dySun * sunFactor;
            for (int i = 0; i < NAMES.length; i++) {
                double dx = planetX[i] - obj.x;
                double dy = planetY[i] - obj.y;
                double inv = inverseR3(dx, dy);
                double factor = G * PLANET_MASS[i] * inv;
                ax += dx * factor;
                ay += dy * factor;
            }
            obj.vx += ax * dtSim;
            obj.vy += ay * dtSim;
            obj.x += obj.vx * dtSim;
            obj.y += obj.vy * dtSim;
            recordTrailPoint(obj);
        }
    }

    private double inverseR3(double dx, double dy) {
        double dist2 = dx * dx + dy * dy;
        if (dist2 < 1e6) dist2 = 1e6;
        double dist = Math.sqrt(dist2);
        return 1.0 / (dist2 * dist);
    }

    private void recordTrailPoint(LaunchedObject obj) {
        Point2D.Double last = obj.trail.peekLast();
        if (last == null || Point2D.distance(obj.x, obj.y, last.x, last.y) >= TRAIL_DISTANCE_THRESHOLD) {
            obj.trail.add(new Point2D.Double(obj.x, obj.y));
            while (obj.trail.size() > MAX_TRAIL_POINTS) obj.trail.removeFirst();
        }
    }

    private void drawLaunchedObjects(Graphics2D g2, int cx, int cy) {
        if (launchedObjects.isEmpty()) return;
        Stroke oldStroke = g2.getStroke();
        for (LaunchedObject obj : launchedObjects) {
            if (obj.trail.size() > 1) {
                Path2D.Double path = new Path2D.Double();
                boolean first = true;
                for (Point2D.Double point : obj.trail) {
                    double px = cx + (point.x - camX) * METERS_TO_PIXELS * currentZoom;
                    double py = cy + (point.y - camY) * METERS_TO_PIXELS * currentZoom;
                    if (first) { path.moveTo(px, py); first = false; }
                    else path.lineTo(px, py);
                }
                g2.setColor(new Color(obj.color.getRed(), obj.color.getGreen(), obj.color.getBlue(), 170));
                g2.setStroke(new BasicStroke(1.3f));
                g2.draw(path);
            }
            double px = cx + (obj.x - camX) * METERS_TO_PIXELS * currentZoom;
            double py = cy + (obj.y - camY) * METERS_TO_PIXELS * currentZoom;
            g2.setColor(obj.color);
            if (obj.shape == LaunchedShape.RECTANGLE) {
                double w = Math.max(1.0, obj.widthM) * METERS_TO_PIXELS * currentZoom;
                double h = Math.max(1.0, obj.heightM) * METERS_TO_PIXELS * currentZoom;
                w = Math.max(w, 6);
                h = Math.max(h, 3);
                AffineTransform old = g2.getTransform();
                g2.translate(px, py);
                g2.rotate(Math.atan2(obj.vy, obj.vx));
                g2.fill(new Rectangle2D.Double(-w / 2, -h / 2, w, h));
                g2.setColor(Color.WHITE);
                g2.draw(new Rectangle2D.Double(-w / 2, -h / 2, w, h));
                g2.setTransform(old);
            } else {
                double r = Math.max(3.0, obj.radiusM * METERS_TO_PIXELS * currentZoom);
                g2.fill(new Ellipse2D.Double(px - r, py - r, 2 * r, 2 * r));
            }
            g2.setColor(Color.WHITE);
            g2.drawString(obj.name, (int)(px + 6), (int)(py - 6));
        }
        g2.setStroke(oldStroke);
    }

    private enum LaunchedShape { CIRCLE, RECTANGLE }

    private static final class LaunchedObject {
        final String name;
        final double massKg;
        final double radiusM;
        final double widthM;
        final double heightM;
        final Color color;
        final LaunchedShape shape;
        double x, y, vx, vy;
        final Deque<Point2D.Double> trail = new ArrayDeque<>();

        LaunchedObject(String name, double massKg, double radiusM,
                       double widthM, double heightM, Color color, LaunchedShape shape) {
            this.name = name;
            this.massKg = massKg;
            this.radiusM = Math.max(1.0, radiusM);
            this.widthM = widthM > 0 ? widthM : this.radiusM * 2;
            this.heightM = heightM > 0 ? heightM : this.radiusM * 2;
            this.color = color;
            this.shape = shape;
        }
    }

    private static final class LaunchPreset {
        final String name;
        final double massKg;
        final double radiusM;
        final double widthM;
        final double heightM;
        final double speedKmS;
        final double angleDeg;
        final Color color;
        final LaunchedShape shape;

        LaunchPreset(String name, double massKg, double radiusM,
                     double widthM, double heightM, double speedKmS,
                     double angleDeg, Color color, LaunchedShape shape) {
            this.name = name;
            this.massKg = massKg;
            this.radiusM = radiusM;
            this.widthM = widthM;
            this.heightM = heightM;
            this.speedKmS = speedKmS;
            this.angleDeg = angleDeg;
            this.color = color;
            this.shape = shape;
        }
    }

    private static final class TreeBody {
        final String type;
        final int planetIndex;
        final int moonIndex;
        final String name;
        TreeBody(String type, int planetIndex, int moonIndex, String name) {
            this.type = type;
            this.planetIndex = planetIndex;
            this.moonIndex = moonIndex;
            this.name = name;
        }
        @Override public String toString() { return name; }
    }

    private static final class BodyTreeRenderer extends DefaultTreeCellRenderer {
        @Override
        public Component getTreeCellRendererComponent(JTree tree, Object value, boolean sel,
                                                      boolean expanded, boolean leaf, int row, boolean hasFocus) {
            Component c = super.getTreeCellRendererComponent(tree, value, sel, expanded, leaf, row, hasFocus);
            if (value instanceof DefaultMutableTreeNode node && node.getUserObject() instanceof TreeBody body) {
                if ("moon".equals(body.type)) {
                    setIcon(UIManager.getIcon("FileView.fileIcon"));
                } else if ("planet".equals(body.type)) {
                    setIcon(UIManager.getIcon("FileView.directoryIcon"));
                }
            } else {
                setIcon(UIManager.getIcon("FileView.hardDriveIcon"));
            }
            return c;
        }
    }
}
