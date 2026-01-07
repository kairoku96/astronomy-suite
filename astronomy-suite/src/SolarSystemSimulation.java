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
 * @author Ethan Lin (original)
 * @version 3.0
 */

//TODO: Sympletic Integration - leapfrog, rebound, saturn ring shadow, UI polish, API for planet data

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
    /** Reference to the main menu bar for toggling visibility */
    private JFrame parentFrame;
    /** Whether menu bar is currently visible */
    private boolean menuBarVisible = true;
    /** Reference to the search bar panel for toggling visibility */
    private JPanel searchBarPanel;
    /** Reference to the sidebar/navigation panel for toggling visibility */
    private JPanel sidebarPanel;
    /** Whether search bar and sidebar are currently visible */
    private boolean uiPanelsVisible = true;
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

    // ----- Educational features for lessons -----
    private boolean lessonMode = false;
    private int currentLesson = 1;              // Default to Lesson 1
    private boolean showKeplersLaw = false;     // Show Kepler's 3rd Law calculations
    private boolean showPerihelionAphelion = false; // Highlight perihelion/aphelion points
    private boolean showVelocityVectors = false;    // Show velocity vectors on planets
    private boolean showVelocityComparison = false; // Show velocity comparison at perihelion vs aphelion

    // ----- Guided Tour System -----
    private boolean guidedTourActive = false;
    private int tourStep = 0;
    private double tourStepStartTime = 0;
    private String tourMessage = "";
    private double tourMessageAlpha = 0.0;
    private static final double TOUR_MESSAGE_FADE_SPEED = 2.0;
    private boolean tourStepReady = false;  // True when user can advance to next step
    private boolean[] savedPlanetVisibility = null;  // Save visibility state before tour

    // ----- UI Size Settings -----
    enum UISize { SMALL, NORMAL, LARGE }
    private UISize uiSize = UISize.NORMAL;  // Default to normal (previously small)

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

        // Pre-compute planet orbit paths using Kepler's equation
        // E=0 is at perihelion (closest to Sun)
        for (int i = 0; i < NAMES.length; i++) {
            ArrayList<Point2D.Double> orbit = new ArrayList<>();
            double a = A_METERS[i], e = ECC[i], b = a * Math.sqrt(1 - e * e);
            double omega = Math.toRadians(PLANET_VARPI0[i] - PLANET_OMEGA0[i]); // argument of perihelion
            double Omega = Math.toRadians(PLANET_OMEGA0[i]);                    // longitude of ascending node
            for (int j = 0; j <= ORBIT_RESOLUTION; j++) {
                // Eccentric anomaly (E=0 is perihelion, E=π is aphelion)
                double E = 2 * Math.PI * j / ORBIT_RESOLUTION;
                // Orbital plane coordinates (Sun at one focus)
                double xr = a * (Math.cos(E) - e);
                double yr = b * Math.sin(E);
                // Apply orbital element rotations to get ecliptic plane coordinates
                double xTemp = xr * Math.cos(omega + Omega) - yr * Math.sin(omega + Omega);
                // Apply 90° clockwise rotation to fix coordinate system
                double x = xr * Math.sin(omega + Omega) + yr * Math.cos(omega + Omega);
                double y = -xTemp;
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
                    double xTemp = a * (Math.cos(E) - e);
                    double yTemp = b * Math.sin(E);
                    double rxTemp = xTemp * Math.cos(arg) - yTemp * Math.sin(arg);
                    // Apply 90° clockwise rotation to fix coordinate system
                    double rx = xTemp * Math.sin(arg) + yTemp * Math.cos(arg);
                    double ry = -rxTemp;
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
                } else if (code == KeyEvent.VK_H) {
                    // Toggle menu bar visibility
                    toggleMenuBarVisibility();
                    e.consume();
                } else if (code == KeyEvent.VK_N) {
                    // N: Next step in guided tour
                    if (guidedTourActive && tourStepReady) {
                        advanceTourStep();
                        e.consume();
                    }
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
            return true;
        }
        // Check showOnlySelected mode
        if (!showOnlySelected) return false;
        if (selectedPlanet == -1) return false;
        return planetIndex != selectedPlanet;
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

        // Draw Sun label during guided tour
        if (guidedTourActive && showLabels) {
            double sunCenterX = sunXpix + sunR;
            double sunCenterY = sunYpix + sunR;

            // Dynamic font size based on uiSize
            float labelFontSize = switch (uiSize) {
                case SMALL -> 9f;
                case NORMAL -> 11f;
                default -> 13f;
            };

            Font oldFont = g2.getFont();
            g2.setFont(oldFont.deriveFont(Font.BOLD, labelFontSize));
            String sunLabel = "Sun";
            FontMetrics fm = g2.getFontMetrics();
            int labelWidth = fm.stringWidth(sunLabel);
            int labelHeight = fm.getHeight();

            // Position label below the Sun
            int labelX = (int)(sunCenterX - labelWidth / 2.0);
            int labelY = (int)(sunCenterY + sunR + labelHeight + 5);

            // Draw label background box
            int padding = 4;
            g2.setColor(new Color(0, 0, 0, 180));
            g2.fillRoundRect(labelX - padding, labelY - labelHeight + 3,
                           labelWidth + padding * 2, labelHeight + 2, 4, 4);

            // Draw label text
            g2.setColor(new Color(255, 200, 100));
            g2.drawString(sunLabel, labelX, labelY);
            g2.setFont(oldFont);
        }

        // Draw asteroid belt and Kuiper belt as rings
        drawAsteroidBeltRing(g2, cx, cy);
        drawKuiperBeltRing(g2, cx, cy);

        // Draw ice lines
        drawIceLines(g2, cx, cy);

        // Draw planets and moons
        for (int i = 0; i < NAMES.length; i++) {
            if (isPlanetVisible(i)) continue;

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

                // Draw perihelion and aphelion markers - calculated from actual orbit path
                if (showPerihelionAphelion && !orbitPaths.get(i).isEmpty()) {
                    ArrayList<Point2D.Double> orbit = orbitPaths.get(i);
                    Point2D.Double periPoint = null;
                    Point2D.Double apoPoint = null;
                    double minDist = Double.MAX_VALUE;
                    double maxDist = 0;

                    // Find actual perihelion and aphelion from orbit path (heliocentric distances)
                    for (Point2D.Double p : orbit) {
                        double dist = Math.sqrt(p.x * p.x + p.y * p.y);
                        if (dist < minDist) {
                            minDist = dist;
                            periPoint = p;
                        }
                        if (dist > maxDist) {
                            maxDist = dist;
                            apoPoint = p;
                        }
                    }

                    if (periPoint != null && apoPoint != null) {
                        // Convert to screen coordinates (add Sun's barycentric offset)
                        double periX = cx + (periPoint.x + sunBarycenterX - camX) * METERS_TO_PIXELS * currentZoom;
                        double periY = cy + (periPoint.y + sunBarycenterY - camY) * METERS_TO_PIXELS * currentZoom;
                        double apoX = cx + (apoPoint.x + sunBarycenterX - camX) * METERS_TO_PIXELS * currentZoom;
                        double apoY = cy + (apoPoint.y + sunBarycenterY - camY) * METERS_TO_PIXELS * currentZoom;

                        // Smart marker sizing: scale with zoom but maintain minimum visibility
                        // At low zoom (zoomed out), markers are smaller but still visible
                        double baseMarkerSize = 6.0;
                        double zoomFactor = Math.max(0.3, Math.min(1.5, currentZoom / 5.0));
                        double markerSize = baseMarkerSize * zoomFactor;

                        // Calculate opacity based on screen distance from center (fade markers that are far off-screen)
                        double periDistFromCenter = Math.hypot(periX - cx, periY - cy);
                        double apoDistFromCenter = Math.hypot(apoX - cx, apoY - cy);
                        double maxScreenDist = Math.hypot(getWidth(), getHeight()) / 2.0;

                        Font oldFont = g2.getFont();
                        float fontSize = (float)(9f + 4f * zoomFactor);

                        // Draw perihelion marker (red, closest to Sun)
                        if (periDistFromCenter < maxScreenDist * 1.5) {
                            int alpha = (int)(220 * Math.max(0.4, 1.0 - periDistFromCenter / (maxScreenDist * 1.5)));
                            g2.setColor(new Color(255, 80, 80, alpha));
                            g2.setStroke(new BasicStroke((float)(2.0f * zoomFactor)));
                            g2.draw(new Ellipse2D.Double(periX - markerSize, periY - markerSize,
                                                          2*markerSize, 2*markerSize));
                            g2.setFont(oldFont.deriveFont(Font.BOLD, fontSize));
                            g2.drawString("P", (int)(periX + markerSize + 3), (int)(periY - markerSize + 1));
                        }

                        // Draw aphelion marker (blue, farthest from Sun)
                        if (apoDistFromCenter < maxScreenDist * 1.5) {
                            int alpha = (int)(220 * Math.max(0.4, 1.0 - apoDistFromCenter / (maxScreenDist * 1.5)));
                            g2.setColor(new Color(80, 150, 255, alpha));
                            g2.setStroke(new BasicStroke((float)(2.0f * zoomFactor)));
                            g2.draw(new Ellipse2D.Double(apoX - markerSize, apoY - markerSize,
                                                          2*markerSize, 2*markerSize));
                            g2.setFont(oldFont.deriveFont(Font.BOLD, fontSize));
                            g2.drawString("A", (int)(apoX + markerSize + 3), (int)(apoY - markerSize + 1));
                        }

                        g2.setFont(oldFont);
                    }
                }
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

            // Draw label with overlap avoidance and zoom-aware sizing
            if (showLabels) {
                // Scale labels based on zoom level
                float labelSize = (float)(11f + Math.min(3f, currentZoom * 0.5f));
                Font labelFont = g2.getFont().deriveFont(Font.PLAIN, labelSize);
                g2.setFont(labelFont);

                g2.setColor(Color.WHITE);
                String name = NAMES[i];
                FontMetrics labelFm = g2.getFontMetrics();
                int w = labelFm.stringWidth(name), h = labelFm.getHeight();

                // Position label away from planet, scaling offset inversely with zoom
                // When zoomed out, labels should be closer to planets
                double angle = Math.atan2(py - cy, px - cx);
                double baseOffset = 8.0; // Minimum offset from planet edge
                double zoomScaledOffset = Math.min(15.0, baseOffset * Math.max(1.0, currentZoom));
                double labelOffset = pr + zoomScaledOffset;
                int lx = (int)(px + Math.cos(angle) * labelOffset);
                int ly = (int)(py + Math.sin(angle) * labelOffset);
                Rectangle box = new Rectangle(lx, ly - h, w, h);

                // Smart overlap avoidance with limited retries
                int maxTries = (currentZoom < 1.0) ? 2 : 4; // Fewer retries when zoomed out
                int tries = 0;
                boolean foundSpot = false;
                while (tries < maxTries) {
                    boolean overlap = false;
                    for (Rectangle r : labelBoxes) {
                        if (r.intersects(box)) {
                            // When zoomed out, try moving horizontally as well as vertically
                            if (currentZoom < 1.0 && tries % 2 == 1) {
                                box.x += w / 2;
                            } else {
                                box.y += h;
                            }
                            overlap = true;
                            tries++;
                            break;
                        }
                    }
                    if (!overlap) {
                        foundSpot = true;
                        break;
                    }
                }

                // Only draw if we found a non-overlapping spot or if zoomed in enough
                if (foundSpot || currentZoom > 3.0) {
                    labelBoxes.add(box);
                    // Add subtle shadow for better readability when zoomed out
                    if (currentZoom < 2.0) {
                        g2.setColor(new Color(0, 0, 0, 150));
                        g2.drawString(name, box.x + 1, box.y + h - 2);
                        g2.setColor(Color.WHITE);
                    }
                    g2.drawString(name, box.x, box.y + h - 3);
                }

                g2.setFont(g2.getFont().deriveFont(Font.PLAIN, 12f)); // Reset font
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
                        double rxTemp = mx * Math.cos(arg) - my * Math.sin(arg);
                        // Apply 90° clockwise rotation to fix coordinate system
                        double rx = mx * Math.sin(arg) + my * Math.cos(arg);
                        double ry = -rxTemp;
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

        // Draw velocity vectors for planets (educational feature)
        if (showVelocityVectors) {
            drawVelocityVectors(g2, cx, cy);
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

            // Dynamic sizing based on uiSize setting
            int panelWidth, marginRight, marginTop;
            float headerFontSize, normalFontSize;
            int lineSpacing, topPadding, sidePadding;

            switch (uiSize) {
                case SMALL:
                    panelWidth = 240;
                    marginRight = 12;
                    marginTop = 12;
                    headerFontSize = 9f;
                    normalFontSize = 8f;
                    lineSpacing = 12;
                    topPadding = 18;
                    sidePadding = 10;
                    break;
                case NORMAL:
                    panelWidth = 280;
                    marginRight = 15;
                    marginTop = 15;
                    headerFontSize = 11f;
                    normalFontSize = 10f;
                    lineSpacing = 14;
                    topPadding = 22;
                    sidePadding = 12;
                    break;
                case LARGE:
                default:
                    panelWidth = 340;
                    marginRight = 18;
                    marginTop = 18;
                    headerFontSize = 12f;
                    normalFontSize = 11f;
                    lineSpacing = 16;
                    topPadding = 25;
                    sidePadding = 15;
                    break;
            }

            // Calculate dynamic height based on actual content
            int baseLines = 11;  // Header + Name + Radius + Distance + Speed + a/e + Peri/Apo + Period + Orbit progress + Central mass + Moons count
            int extraKeplerLines = showKeplersLaw ? 6 : 0;  // Header + divider + 3 calculation steps + result with error
            int extraMoonLines = 0;
            if (selectedMoon >= 0 && selectedMoon < MOON_NAMES[i].length) {
                extraMoonLines = 7;  // Moon header, spacing, name, a/e, peri/apo, sidereal, synodic
            }

            int totalLines = baseLines + extraKeplerLines + extraMoonLines;
            int w = panelWidth;
            int h = (topPadding + 10) + (totalLines * lineSpacing) + 10;  // Top padding + lines + bottom padding
            int x0 = getWidth() - w - marginRight, y0 = marginTop;

            // Modern gradient panel background
            GradientPaint gradient = new GradientPaint(
                x0, y0, new Color(20, 25, 40, 240),
                x0, y0 + h, new Color(15, 20, 35, 240)
            );
            g2.setPaint(gradient);
            g2.fillRoundRect(x0, y0, w, h, 15, 15);

            // Accent border with glow effect
            g2.setColor(new Color(100, 150, 255, 180));
            g2.setStroke(new BasicStroke(2f));
            g2.drawRoundRect(x0, y0, w, h, 15, 15);
            g2.setColor(new Color(100, 150, 255, 60));
            g2.setStroke(new BasicStroke(4f));
            g2.drawRoundRect(x0 - 1, y0 - 1, w + 2, h + 2, 16, 16);

            // Use a modern, clean font
            Font oldFont = g2.getFont();
            g2.setFont(oldFont.deriveFont(Font.PLAIN, normalFontSize));

            int lineY = y0 + topPadding;
            int lineStep = lineSpacing;
            int colX = x0 + sidePadding;

            // Header with accent color
            g2.setFont(oldFont.deriveFont(Font.BOLD, headerFontSize));
            g2.setColor(new Color(120, 180, 255));
            g2.drawString("PLANET", colX, lineY);
            lineY += lineStep + 4;

            g2.setFont(oldFont.deriveFont(Font.PLAIN, normalFontSize));
            g2.setColor(new Color(255, 255, 255));
            g2.drawString("Name: " + NAMES[i], colX, lineY); lineY += lineStep;

            // Planet core properties with improved colors
            g2.setColor(new Color(220, 230, 250));
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

            // Kepler's 3rd Law display for Lesson 1 - enhanced with detailed calculations
            if (showKeplersLaw) {
                double aAU = a_m / AU;
                double pSquared = periodYears * periodYears;
                double aCubed = aAU * aAU * aAU;
                double ratio = pSquared / aCubed;

                // Add spacing before Kepler's Law section
                lineY += (int)(lineStep * 0.3);

                // Draw prominent header with divider line
                g2.setFont(oldFont.deriveFont(Font.BOLD, headerFontSize));
                g2.setColor(new Color(255, 200, 80));
                g2.drawString("KEPLER'S 3RD LAW", colX, lineY);
                lineY += lineStep;

                // Draw divider line under header
                g2.setColor(new Color(255, 200, 80, 150));
                g2.setStroke(new BasicStroke(1.5f));
                g2.drawLine(colX, lineY - (int)(lineStep * 0.7), colX + w - (sidePadding * 2) - 8, lineY - (int)(lineStep * 0.7));

                // Show calculation steps with emphasis
                g2.setFont(oldFont.deriveFont(Font.PLAIN, normalFontSize * 0.95f));
                g2.setColor(new Color(230, 230, 250));

                // Step 1: Show P² calculation
                g2.drawString(String.format("P² = (%.2f y)² = %.4f y²", periodYears, pSquared), colX, lineY);
                lineY += lineStep;

                // Step 2: Show a³ calculation
                g2.drawString(String.format("a³ = (%.3f AU)³ = %.4f AU³", aAU, aCubed), colX, lineY);
                lineY += lineStep;

                // Step 3: Show the ratio calculation with highlight (hidden during guided tour)
                int boxHeight = (int)(lineStep * 1.3);
                if (!guidedTourActive) {
                    g2.setColor(new Color(255, 200, 50, 60));
                    g2.fillRoundRect(colX - 6, lineY - (int)(lineStep * 0.85), w - (sidePadding * 2), boxHeight, 8, 8);

                    // Draw border around result
                    g2.setColor(new Color(255, 220, 80, 200));
                    g2.setStroke(new BasicStroke(2.0f));
                    g2.drawRoundRect(colX - 6, lineY - (int)(lineStep * 0.85), w - (sidePadding * 2), boxHeight, 8, 8);
                }

                // Final ratio with larger, bold text
                g2.setFont(oldFont.deriveFont(Font.BOLD, normalFontSize * 1.15f));
                g2.setColor(new Color(255, 230, 100));
                g2.drawString(String.format("P² / a³ = %.4f", ratio), colX, lineY);

                // Show comparison to ideal value
                g2.setFont(oldFont.deriveFont(Font.PLAIN, normalFontSize * 0.9f));
                g2.setColor(new Color(180, 220, 255));
                double percentError = Math.abs(ratio - 1.0) * 100.0;
                g2.drawString(String.format("(%.2f%% from 1.000)", percentError), colX + (int)(w * 0.55), lineY);

                g2.setFont(oldFont.deriveFont(Font.PLAIN, normalFontSize));
                g2.setColor(new Color(220, 230, 250));
                lineY += lineStep + (int)(lineStep * 0.3);
            }

            g2.drawString(String.format("Orbit progress: %.1f%%", pct), colX, lineY); lineY += lineStep;

            g2.setColor(new Color(180, 200, 230));
            g2.drawString(String.format("Central mass: %.3e kg", sunMass), colX, lineY); lineY += lineStep;
            if (MOON_NAMES[i].length > 0) {
                g2.drawString(String.format("Moons: %d", MOON_NAMES[i].length), colX, lineY); lineY += lineStep;
            }

            // Moon metadata block when a moon is selected
            if (selectedMoon >= 0 && selectedMoon < MOON_NAMES[i].length) {
                int m = selectedMoon;
                lineY += lineStep + 2; // spacing before moon section

                g2.setFont(oldFont.deriveFont(Font.BOLD, headerFontSize));
                g2.setColor(new Color(150, 200, 255));
                g2.drawString("MOON", colX, lineY);
                lineY += lineStep + 2;

                g2.setFont(oldFont.deriveFont(Font.PLAIN, normalFontSize));
                g2.setColor(new Color(255, 255, 255));
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

                g2.setColor(new Color(220, 230, 250));
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

        // Modern lesson mode indicator banner
        if (lessonMode) {
            // Dynamic sizing based on uiSize setting
            int bannerWidth, bannerHeight;
            float titleFontSize, subtitleFontSize;

            switch (uiSize) {
                case SMALL:
                    bannerWidth = 240;
                    bannerHeight = 42;
                    titleFontSize = 10f;
                    subtitleFontSize = 8f;
                    break;
                case NORMAL:
                    bannerWidth = 280;
                    bannerHeight = 50;
                    titleFontSize = 12f;
                    subtitleFontSize = 10f;
                    break;
                case LARGE:
                default:
                    bannerWidth = 330;
                    bannerHeight = 60;
                    titleFontSize = 14f;
                    subtitleFontSize = 11f;
                    break;
            }

            int bannerX = (getWidth() - bannerWidth) / 2;
            int bannerY = 15;

            // Shadow effect
            g2.setColor(new Color(0, 0, 0, 80));
            g2.fillRoundRect(bannerX + 3, bannerY + 3, bannerWidth, bannerHeight, 15, 15);

            // Gradient background
            GradientPaint bannerGradient = new GradientPaint(
                bannerX, bannerY, new Color(40, 80, 160, 240),
                bannerX, bannerY + bannerHeight, new Color(60, 120, 200, 240)
            );
            g2.setPaint(bannerGradient);
            g2.fillRoundRect(bannerX, bannerY, bannerWidth, bannerHeight, 15, 15);

            // Accent border with glow
            g2.setColor(new Color(120, 180, 255, 200));
            g2.setStroke(new BasicStroke(2.5f));
            g2.drawRoundRect(bannerX, bannerY, bannerWidth, bannerHeight, 15, 15);
            g2.setColor(new Color(150, 200, 255, 80));
            g2.setStroke(new BasicStroke(5f));
            g2.drawRoundRect(bannerX - 1, bannerY - 1, bannerWidth + 2, bannerHeight + 2, 16, 16);

            // Title text with icon
            g2.setColor(new Color(255, 255, 255, 250));
            g2.setFont(g2.getFont().deriveFont(Font.BOLD, titleFontSize));
            String title = "🎓 Lesson " + currentLesson;
            int titleWidth = g2.getFontMetrics().stringWidth(title);
            int titleY = bannerY + (int)(bannerHeight * 0.4);
            g2.drawString(title, bannerX + (bannerWidth - titleWidth) / 2, titleY);

            // Lesson name
            String[] lessonNames = {
                "Introduction to Orbital Motion",
                "Velocity & Kepler's 2nd Law",
                "Scale of the Solar System",
                "Moon Systems",
                "Time Control"
            };
            String subtitle = lessonNames[currentLesson - 1];
            g2.setFont(g2.getFont().deriveFont(Font.PLAIN, subtitleFontSize));
            g2.setColor(new Color(220, 235, 255, 250));
            int subtitleWidth = g2.getFontMetrics().stringWidth(subtitle);
            int subtitleY = bannerY + (int)(bannerHeight * 0.72);
            g2.drawString(subtitle, bannerX + (bannerWidth - subtitleWidth) / 2, subtitleY);

            // Draw guided tour message if active
            if (guidedTourActive && !tourMessage.isEmpty()) {
                drawGuidedTourMessage(g2);
            }
        }
    }

    /**
     * Draw the guided tour message box with multi-line text
     * Box size dynamically adjusts to fit text content
     */
    private void drawGuidedTourMessage(Graphics2D g2) {
        int alpha = (int)(tourMessageAlpha * 255);
        if (alpha <= 0) return;

        // Dynamic sizing based on uiSize setting
        float fontSize;
        int minWidth, paddingH, paddingV, bottomMargin;

        switch (uiSize) {
            case SMALL:
                fontSize = 10f;
                minWidth = 180;
                paddingH = 32;
                paddingV = 28;
                bottomMargin = 50;
                break;
            case NORMAL:
                fontSize = 12f;
                minWidth = 220;
                paddingH = 40;
                paddingV = 35;
                bottomMargin = 60;
                break;
            case LARGE:
            default:
                fontSize = 14f;
                minWidth = 260;
                paddingH = 50;
                paddingV = 42;
                bottomMargin = 70;
                break;
        }

        // Split message into lines
        String[] lines = tourMessage.split("\n");

        // Calculate box dimensions based on text
        g2.setFont(g2.getFont().deriveFont(Font.PLAIN, fontSize));
        FontMetrics fm = g2.getFontMetrics();
        int maxWidth = 0;
        for (String line : lines) {
            int lineWidth = fm.stringWidth(line);
            maxWidth = Math.max(maxWidth, lineWidth);
        }

        // Add padding and ensure minimum size
        int boxWidth = Math.max(maxWidth + paddingH, minWidth);
        int lineHeight = fm.getHeight();
        int textHeight = lineHeight * lines.length;
        int boxHeight = textHeight + paddingV;

        // Center horizontally, place near bottom
        int boxX = (getWidth() - boxWidth) / 2;
        int boxY = getHeight() - boxHeight - bottomMargin;

        // Shadow effect
        g2.setColor(new Color(0, 0, 0, Math.min(alpha, 100)));
        g2.fillRoundRect(boxX + 4, boxY + 4, boxWidth, boxHeight, 12, 12);

        // Box background with gradient
        GradientPaint boxGradient = new GradientPaint(
            boxX, boxY, new Color(30, 30, 60, Math.min(alpha, 230)),
            boxX, boxY + boxHeight, new Color(50, 50, 90, Math.min(alpha, 230))
        );
        g2.setPaint(boxGradient);
        g2.fillRoundRect(boxX, boxY, boxWidth, boxHeight, 12, 12);

        // Border with glow effect
        g2.setColor(new Color(100, 150, 255, Math.min(alpha, 200)));
        g2.setStroke(new BasicStroke(2.5f));
        g2.drawRoundRect(boxX, boxY, boxWidth, boxHeight, 12, 12);

        // Optional: pulsing glow when ready to advance
        if (tourStepReady) {
            double pulse = Math.sin(System.currentTimeMillis() / 300.0) * 0.3 + 0.7;
            int glowAlpha = (int)(pulse * Math.min(alpha, 150));
            g2.setColor(new Color(150, 200, 255, glowAlpha));
            g2.setStroke(new BasicStroke(4.5f));
            g2.drawRoundRect(boxX - 2, boxY - 2, boxWidth + 4, boxHeight + 4, 14, 14);
        }

        // Draw text lines
        g2.setColor(new Color(255, 255, 255, alpha));
        g2.setFont(g2.getFont().deriveFont(Font.PLAIN, fontSize));
        int textY = boxY + (int)(paddingV * 0.6);
        for (String line : lines) {
            int textWidth = fm.stringWidth(line);
            int textX = boxX + (boxWidth - textWidth) / 2;
            g2.drawString(line, textX, textY);
            textY += lineHeight;
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
        double xTemp = xr * Math.cos(omega + Omega) - yr * Math.sin(omega + Omega);

        // Apply 90° clockwise rotation to fix coordinate system
        double x = xr * Math.sin(omega + Omega) + yr * Math.cos(omega + Omega);
        double y = -xTemp;
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
        float litX = (float)(px + lightX * radiusPx * 1);
        float litY = (float)(py + lightY * radiusPx * 1);
        float darkX = (float)(px - lightX * radiusPx * 0.35);
        float darkY = (float)(py - lightY * radiusPx * 0.35);

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

    /**
     * Toggles the visibility of the menu bar, search bar, and sidebar.
     * Called when user presses the 'H' key.
     */
    private void toggleMenuBarVisibility() {
        if (parentFrame == null) return;

        menuBarVisible = !menuBarVisible;
        uiPanelsVisible = !uiPanelsVisible;

        // Toggle menu bar
        JMenuBar menuBar = parentFrame.getJMenuBar();
        if (menuBar != null) {
            menuBar.setVisible(menuBarVisible);
        }

        // Toggle search bar
        if (searchBarPanel != null) {
            searchBarPanel.setVisible(uiPanelsVisible);
        }

        // Toggle sidebar
        if (sidebarPanel != null) {
            sidebarPanel.setVisible(uiPanelsVisible);
        }

        // Revalidate to update layout
        parentFrame.revalidate();
        parentFrame.repaint();
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
            double rxTemp = mx * Math.cos(arg) - my * Math.sin(arg);
            // Apply 90° clockwise rotation to fix coordinate system
            double rx = mx * Math.sin(arg) + my * Math.cos(arg);
            double ry = -rxTemp;
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

        /* ----- GUIDED TOUR UPDATE ----- */
        if (lessonMode && guidedTourActive) {
            updateGuidedTour(dtReal);
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
     * Updates the guided tour for different lessons
     * This method manages tour steps with manual progression (press N to continue)
     */
    private void updateGuidedTour(double dtReal) {
        // Update tour timer
        tourStepStartTime += dtReal;

        // Fade in tour message
        if (tourMessageAlpha < 1.0) {
            tourMessageAlpha += TOUR_MESSAGE_FADE_SPEED * dtReal;
            if (tourMessageAlpha > 1.0) tourMessageAlpha = 1.0;
        }

        // After message is fully faded in and camera settled, allow advancing
        if (tourMessageAlpha >= 1.0 && tourStepStartTime > 1.5) {
            tourStepReady = true;
        }

        // Route to appropriate lesson tour
        if (currentLesson == 1) {
            updateLesson1Tour(dtReal);
        } else if (currentLesson == 2) {
            updateLesson2Tour(dtReal);
        }
    }

    /**
     * Lesson 1 Tour: Introduction to Orbital Motion
     */
    private void updateLesson1Tour(double dtReal) {
        // Lesson 1 Tour Steps - execute setup on first frame only
        if (tourStepStartTime < dtReal * 2) { // First frame of this step
            switch (tourStep) {
                case 0: // Welcome & Setup
                    showPlanetOrbits = true;
                    showPerihelionAphelion = true;
                    showKeplersLaw = true;
                    hideAllPlanetsExcept(-1); // Show all for overview
                    returnToSun();
                    currentZoom = 2.0;
                    targetZoom = 2.0;
                    tourMessage = """
                            Welcome to Lesson 1: Introduction to Orbital Motion
                            We'll explore how planets orbit the Sun.
                            
                            Press 'N' to continue to the next step.""";
                    break;

                case 1: // Focus on Venus (circular orbit)
                    hideAllPlanetsExcept(findPlanetIndex("Venus"));
                    focusPlanetByNameWithZoom("Venus", 1.0);  // Show full orbit comfortably
                    tourMessage = """
                            Venus - Nearly Perfect Circle
                            Eccentricity ≈ 0.007
                            Notice how circular the orbit looks!
                            
                            Press 'N' to continue.""";
                    break;

                case 2: // Focus on Earth
                    hideAllPlanetsExcept(findPlanetIndex("Earth"));
                    focusPlanetByNameWithZoom("Earth", 0.79);  // Show full orbit comfortably
                    tourMessage = """
                            Earth - Also Quite Circular
                            Eccentricity ≈ 0.017
                            Our orbit is nearly circular too.
                            
                            Press 'N' to continue.""";
                    break;

                case 3: // Focus on Mars
                    hideAllPlanetsExcept(findPlanetIndex("Mars"));
                    focusPlanetByNameWithZoom("Mars", 0.5);  // Show full orbit
                    tourMessage = """
                            Mars - More Elliptical
                            Eccentricity ≈ 0.093
                            The orbit starts to look stretched.
                            
                            Press 'N' to continue.""";
                    break;

                case 4: // Focus on Mercury
                    hideAllPlanetsExcept(findPlanetIndex("Mercury"));
                    focusPlanetByNameWithZoom("Mercury", 1.75);  // Smaller orbit, can zoom more
                    tourMessage = """
                            Mercury - Quite Elliptical
                            Eccentricity ≈ 0.206
                            Notice the red (P) and blue (A) markers
                            showing perihelion and aphelion points.
                            
                            Press 'N' to continue.""";
                    break;

                case 5: // Focus on Pluto
                    hideAllPlanetsExcept(findPlanetIndex("Pluto"));
                    focusPlanetByNameWithZoom("Pluto", 0.02);  // Large orbit, zoom out (slightly more than 0.04)
                    tourMessage = """
                            Pluto - Highly Elliptical
                            Eccentricity ≈ 0.250
                            The most eccentric orbit in our tour!
                            Distance varies dramatically.
                            
                            Press 'N' to continue.""";
                    break;

                case 6: // Kepler's Third Law - Mercury
                    hideAllPlanetsExcept(findPlanetIndex("Mercury"));
                    focusPlanetByNameWithZoom("Mercury", 1.75);
                    tourMessage = """
                            Kepler's Third Law Verification
                            Mercury: P² / a³ ≈ 1.0
                            Check the yellow line in the info panel!
                            
                            Press 'N' to continue.""";
                    break;

                case 7: // Kepler's Third Law - Earth
                    hideAllPlanetsExcept(findPlanetIndex("Earth"));
                    focusPlanetByNameWithZoom("Earth", 0.79);
                    tourMessage = """
                            Kepler's Third Law Verification
                            Earth: P² / a³ = 1.0000 (exactly!)
                            By definition of AU and year.
                            
                            Press 'N' to continue.""";
                    break;

                case 8: // Kepler's Third Law - Jupiter
                    hideAllPlanetsExcept(findPlanetIndex("Jupiter"));
                    focusPlanetByNameWithZoom("Jupiter", 0.1);  // Large orbit, zoom out more
                    tourMessage = """
                            Kepler's Third Law Verification
                            Jupiter: P² / a³ ≈ 1.0
                            The ratio is constant for all planets!
                            
                            Press 'N' to continue.""";
                    break;

                case 9: // Return to overview
                    restorePlanetVisibility();
                    returnToSun();
                    currentZoom = 1.5;
                    targetZoom = 1.5;
                    tourMessage = """
                            Tour Complete!
                            All planet visibility has been restored.
                            Feel free to explore on your own.
                            
                            Press 'N' to end the tour.""";
                    break;

                case 10: // End tour
                    guidedTourActive = false;
                    tourStep = 0;
                    tourMessage = "";
                    tourMessageAlpha = 0.0;
                    tourStepReady = false;
                    restorePlanetVisibility();
                    break;
            }
        }
    }

    /**
     * Lesson 2 Tour: Orbital Velocity and Kepler's 2nd Law
     */
    private void updateLesson2Tour(double dtReal) {
        // Lesson 2 Tour Steps - execute setup on first frame only
        if (tourStepStartTime < dtReal * 2) { // First frame of this step
            switch (tourStep) {
                case 0: // Welcome & Setup
                    showPlanetOrbits = true;
                    showPerihelionAphelion = true;
                    showVelocityVectors = true;
                    showVelocityComparison = true;
                    hideAllPlanetsExcept(-1); // Show all for overview
                    returnToSun();
                    currentZoom = 2.0;
                    targetZoom = 2.0;
                    tourMessage = """
                            Welcome to Lesson 2: Orbital Velocity
                            We'll explore how velocity changes around an orbit.
                            Velocity vectors show speed and direction.
                            
                            Press 'N' to continue to the next step.""";
                    break;

                case 1: // Explain velocity vectors
                    returnToSun();
                    currentZoom = 1.5;
                    targetZoom = 1.5;
                    tourMessage = """
                            Velocity Vectors
                            Arrows show each planet's velocity.
                            Blue = slower, Red = faster.
                            Notice how colors change around the orbit!
                            
                            Press 'N' to continue.""";
                    break;

                case 2: // Mercury at perihelion
                    hideAllPlanetsExcept(findPlanetIndex("Mercury"));
                    focusPlanetByNameWithZoom("Mercury", 1.75);  // Same as Lesson 1
                    tourMessage = """
                            Mercury - High Eccentricity
                            Watch the velocity arrow change color
                            as Mercury moves around its orbit.
                            Red = fast (perihelion), Blue = slow (aphelion).
                            
                            Press 'N' to continue.""";
                    break;

                case 3: // Mars velocity demonstration
                    hideAllPlanetsExcept(findPlanetIndex("Mars"));
                    focusPlanetByNameWithZoom("Mars", 0.5);  // Same as Lesson 1
                    tourMessage = """
                            Mars - Velocity Variation
                            Current speed shown next to arrow.
                            Mars moves faster when closer to the Sun.
                            This is Kepler's 2nd Law in action!
                            
                            Press 'N' to continue.""";
                    break;

                case 4: // Earth circular orbit comparison
                    hideAllPlanetsExcept(findPlanetIndex("Earth"));
                    focusPlanetByNameWithZoom("Earth", 0.79);  // Same as Lesson 1
                    tourMessage = """
                            Earth - Nearly Constant Velocity
                            Earth's nearly circular orbit means
                            velocity stays almost constant.
                            Speed: ~30 km/s year-round.
                            
                            Press 'N' to continue.""";
                    break;

                case 5: // Pluto extreme case
                    hideAllPlanetsExcept(findPlanetIndex("Pluto"));
                    focusPlanetByNameWithZoom("Pluto", 0.02);  // Same as Lesson 1
                    tourMessage = """
                            Pluto - Extreme Velocity Change
                            Highly elliptical orbit = huge velocity range!
                            Perihelion: ~6.1 km/s (fast)
                            Aphelion: ~3.7 km/s (slow)
                            
                            Press 'N' to continue.""";
                    break;

                case 6: // Kepler's 2nd Law explanation
                    returnToSun();
                    currentZoom = 1.5;
                    targetZoom = 1.5;
                    hideAllPlanetsExcept(-1); // Show all
                    tourMessage = """
                            Kepler's Second Law
                            "Equal areas in equal times"
                            Planets move faster when closer to the Sun
                            to sweep equal orbital areas.
                            
                            Press 'N' to continue.""";
                    break;

                case 7: // Return to overview
                    restorePlanetVisibility();
                    returnToSun();
                    currentZoom = 1.5;
                    targetZoom = 1.5;
                    tourMessage = """
                            Velocity Tour Complete!
                            You can toggle velocity vectors anytime:
                            Education → Lesson 2 Features
                            
                            Press 'N' to end the tour.""";
                    break;

                case 8: // End tour
                    guidedTourActive = false;
                    tourStep = 0;
                    tourMessage = "";
                    tourMessageAlpha = 0.0;
                    tourStepReady = false;
                    showVelocityVectors = false;
                    showVelocityComparison = false;
                    restorePlanetVisibility();
                    break;
            }
        }
    }

    /**
     * Advances to the next tour step
     */
    private void advanceTourStep() {
        tourStep++;
        tourStepStartTime = 0;
        tourMessageAlpha = 0.0;
        tourStepReady = false;
    }

    /**
     * Find the index of a planet by name
     */
    private int findPlanetIndex(String name) {
        for (int i = 0; i < NAMES.length; i++) {
            if (NAMES[i].equals(name)) {
                return i;
            }
        }
        return -1;
    }

    /**
     * Focuses camera on a planet by name with custom zoom level
     */
    private void focusPlanetByNameWithZoom(String name, double zoom) {
        for (int i = 0; i < NAMES.length; i++) {
            if (NAMES[i].equals(name)) {
                selectedPlanet = i;
                selectedMoon = -1;
                cameraFocusMoon = -1;
                autoZoomEnabled = false; // Disable auto-zoom to use our custom zoom
                focusPlanetImmediate(i);
                targetZoom = zoom;
                currentZoom = zoom;
                return;
            }
        }
    }

    /**
     * Hide all planets except the specified one (or show all if index is -1)
     */
    private void hideAllPlanetsExcept(int planetIndex) {
        // Save current visibility state on first call
        if (savedPlanetVisibility == null) {
            savedPlanetVisibility = new boolean[planetVisible.length];
            System.arraycopy(planetVisible, 0, savedPlanetVisibility, 0, planetVisible.length);
        }

        if (planetIndex == -1) {
            // Show all planets
            Arrays.fill(planetVisible, true);
        } else {
            // Hide all except the specified planet
            for (int i = 0; i < planetVisible.length; i++) {
                planetVisible[i] = (i == planetIndex);
            }
        }
    }

    /**
     * Restore planet visibility to saved state
     */
    private void restorePlanetVisibility() {
        if (savedPlanetVisibility != null) {
            System.arraycopy(savedPlanetVisibility, 0, planetVisible, 0, planetVisible.length);
            savedPlanetVisibility = null;
        }
    }

    /**
     * Returns camera to Sun overview
     */
    private void returnToSun() {
        followingPlanet = followingMoon = false;
        cameraFocusIndex = cameraFocusMoon = -1;
        selectedPlanet = selectedMoon = -1;
        cameraIsApproaching = moonCameraIsApproaching = false;
        currentZoom = 1.0;
        targetZoom = 1.0;
        camX = 0;
        camY = 0;
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

    /**
     * Custom radio button menu item that doesn't close the menu
     */
    static class StayOpenRadioButtonMenuItem extends JRadioButtonMenuItem {
        public StayOpenRadioButtonMenuItem(String text, boolean selected) {
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
        panel.parentFrame = frame; // Set frame reference for menu toggling
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

        // UI Size submenu
        viewMenu.addSeparator();
        JMenu uiSizeMenu = new JMenu("UI Size");
        ButtonGroup uiSizeGroup = new ButtonGroup();

        StayOpenRadioButtonMenuItem smallUIItem = new StayOpenRadioButtonMenuItem("Small", false);
        smallUIItem.addActionListener(_ -> {
            panel.uiSize = UISize.SMALL;
            panel.repaint();
        });
        uiSizeGroup.add(smallUIItem);
        uiSizeMenu.add(smallUIItem);

        StayOpenRadioButtonMenuItem normalUIItem = new StayOpenRadioButtonMenuItem("Normal", true);
        normalUIItem.addActionListener(_ -> {
            panel.uiSize = UISize.NORMAL;
            panel.repaint();
        });
        uiSizeGroup.add(normalUIItem);
        uiSizeMenu.add(normalUIItem);

        StayOpenRadioButtonMenuItem largeUIItem = new StayOpenRadioButtonMenuItem("Large", false);
        largeUIItem.addActionListener(_ -> {
            panel.uiSize = UISize.LARGE;
            panel.repaint();
        });
        uiSizeGroup.add(largeUIItem);
        uiSizeMenu.add(largeUIItem);

        viewMenu.add(uiSizeMenu);

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
                        - H: Hide/show all UI (menu, search, sidebar)
                        - N: Next step in guided tour (when active)
                        
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

        // Education menu for lessons
        JMenu educationMenu = new JMenu("Education");
        StayOpenCheckBoxMenuItem lessonModeItem = new StayOpenCheckBoxMenuItem("Enable Lesson Mode", false);
        lessonModeItem.addActionListener(_ -> {
            panel.lessonMode = lessonModeItem.isSelected();
            // Stop tour if lesson mode is disabled
            if (!panel.lessonMode) {
                panel.guidedTourActive = false;
                panel.tourStep = 0;
                panel.tourMessage = "";
                panel.restorePlanetVisibility(); // Restore visibility
            }
            panel.repaint();
        });
        educationMenu.add(lessonModeItem);

        // Guided Tour toggle button
        JMenuItem guidedTourItem = new JMenuItem("Start Guided Tour");
        guidedTourItem.addActionListener(_ -> {
            if (!panel.lessonMode) {
                JOptionPane.showMessageDialog(frame,
                    "Please enable Lesson Mode first!",
                    "Lesson Mode Required",
                    JOptionPane.INFORMATION_MESSAGE);
                return;
            }

            // Always check current state and toggle opposite
            if (panel.guidedTourActive) {
                // Currently active - STOP the tour
                panel.guidedTourActive = false;
                panel.tourStep = 0;
                panel.tourMessage = "";
                panel.tourMessageAlpha = 0.0;
                panel.tourStepReady = false;
                panel.restorePlanetVisibility(); // Restore visibility when stopping
                guidedTourItem.setText("Start Guided Tour");
            } else {
                // Currently inactive - START the tour
                panel.guidedTourActive = true;
                panel.tourStep = 0;
                panel.tourStepStartTime = 0;
                panel.tourMessageAlpha = 0.0;
                panel.tourMessage = "";
                panel.tourStepReady = false;
                panel.savedPlanetVisibility = null; // Reset saved state
                guidedTourItem.setText("Stop Guided Tour");

                // Show info about controls
                JOptionPane.showMessageDialog(frame,
                        """
                                Guided Tour Started!
                                
                                Press 'N' to advance to the next step.
                                The tour will automatically adjust zoom and visibility.""",
                    "Guided Tour Controls",
                    JOptionPane.INFORMATION_MESSAGE);
            }
            panel.repaint();
        });
        educationMenu.add(guidedTourItem);
        educationMenu.addSeparator();

        // Lesson selector submenu
        JMenu lessonMenu = new JMenu("Select Lesson");
        ButtonGroup lessonGroup = new ButtonGroup();
        String[] lessonNames = {
            "Lesson 1: Orbital Motion",
            "Lesson 2: Velocity & Kepler's 2nd Law",
            "Lesson 3: Scale of Solar System",
            "Lesson 4: Moon Systems",
            "Lesson 5: Time Control"
        };
        for (int i = 0; i < lessonNames.length; i++) {
            final int lessonNum = i + 1;
            JRadioButtonMenuItem lessonItem = new JRadioButtonMenuItem(lessonNames[i], i == 0);
            lessonGroup.add(lessonItem);
            lessonItem.addActionListener(_ -> {
                panel.currentLesson = lessonNum;
                panel.repaint();
            });
            lessonMenu.add(lessonItem);
        }
        educationMenu.add(lessonMenu);
        educationMenu.addSeparator();

        // Lesson 1 specific features
        JMenu lesson1Menu = new JMenu("Lesson 1 Features");
        StayOpenCheckBoxMenuItem keplersLawItem = new StayOpenCheckBoxMenuItem("Show Kepler's 3rd Law", false);
        keplersLawItem.addActionListener(_ -> {
            panel.showKeplersLaw = keplersLawItem.isSelected();
            panel.repaint();
        });
        StayOpenCheckBoxMenuItem periAphelionItem = new StayOpenCheckBoxMenuItem("Show Perihelion/Aphelion", false);
        periAphelionItem.addActionListener(_ -> {
            panel.showPerihelionAphelion = periAphelionItem.isSelected();
            panel.repaint();
        });
        lesson1Menu.add(keplersLawItem);
        lesson1Menu.add(periAphelionItem);
        educationMenu.add(lesson1Menu);

        // Lesson 2 specific features (Velocity & Kepler's 2nd Law)
        JMenu lesson2Menu = new JMenu("Lesson 2 Features");
        StayOpenCheckBoxMenuItem velocityVectorsItem = new StayOpenCheckBoxMenuItem("Show Velocity Vectors", false);
        velocityVectorsItem.addActionListener(_ -> {
            panel.showVelocityVectors = velocityVectorsItem.isSelected();
            panel.repaint();
        });
        StayOpenCheckBoxMenuItem velocityComparisonItem = new StayOpenCheckBoxMenuItem("Show Velocity Labels", false);
        velocityComparisonItem.addActionListener(_ -> {
            panel.showVelocityComparison = velocityComparisonItem.isSelected();
            panel.repaint();
        });
        lesson2Menu.add(velocityVectorsItem);
        lesson2Menu.add(velocityComparisonItem);
        educationMenu.add(lesson2Menu);

        JMenuItem lessonGuideItem = new JMenuItem("Lesson Guide...");
        lessonGuideItem.addActionListener(_ -> panel.showLessonGuide(frame));
        educationMenu.addSeparator();
        educationMenu.add(lessonGuideItem);

        menuBar.add(educationMenu);
        menuBar.add(helpMenu);
        frame.setJMenuBar(menuBar);

        // ----- Top search bar -----
        JPanel topBar = new JPanel(new BorderLayout(8, 0));
        topBar.setBorder(BorderFactory.createEmptyBorder(4, 8, 4, 8));
        panel.searchBarPanel = topBar; // Store reference for toggling visibility
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
        panel.sidebarPanel = leftNav; // Store reference for toggling visibility
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

    /**
     * Draw velocity vectors for planets to show Kepler's 2nd Law
     */
    private void drawVelocityVectors(Graphics2D g2, int cx, int cy) {
        for (int i = 0; i < NAMES.length; i++) {
            if (isPlanetVisible(i)) continue;

            // Planet position
            double worldX = planetX[i] + sunBarycenterX;
            double worldY = planetY[i] + sunBarycenterY;
            double px = cx + (worldX - camX) * METERS_TO_PIXELS * currentZoom;
            double py = cy + (worldY - camY) * METERS_TO_PIXELS * currentZoom;

            // Calculate velocity direction (perpendicular to radius vector)
            double dx = worldX - sunBarycenterX;
            double dy = worldY - sunBarycenterY;
            double dist = Math.sqrt(dx * dx + dy * dy);

            // Velocity is perpendicular to position vector (for circular approximation)
            // For elliptical orbits, we use the actual speed from vis-viva equation
            double vx = -dy / dist;  // perpendicular
            double vy = dx / dist;

            // Scale velocity vector by actual speed
            double speed = planetSpeedKmS[i];
            double vectorScale = 0.00015 * currentZoom;  // Adjust this for visibility
            double vLength = speed * vectorScale;

            // Draw velocity vector as an arrow
            double endX = px + vx * vLength;
            double endY = py + vy * vLength;

            // Dynamic arrow sizing based on UI size
            float arrowWidth = switch (uiSize) {
                case SMALL -> 2.0f;
                case NORMAL -> 2.5f;
                default -> 3.0f;
            };

            // Color based on speed (blue for slow, red for fast)
            double mu = G * (M_SUN + PLANET_MASS[i]);
            double circularSpeed = Math.sqrt(mu / dist) / 1000.0;  // km/s
            double speedRatio = speed / circularSpeed;

            // Color gradient: blue (slow) to green (circular) to red (fast)
            Color vectorColor;
            if (speedRatio < 1.0) {
                // Slower than circular: blue-ish
                int blue = 255;
                int green = (int)(speedRatio * 150);
                vectorColor = new Color(50, green, blue, 220);
            } else {
                // Faster than circular: yellow to red
                int red = Math.min(255, (int)((speedRatio - 1.0) * 400 + 200));
                int green = Math.max(100, 255 - (int)((speedRatio - 1.0) * 300));
                vectorColor = new Color(red, green, 50, 220);
            }

            g2.setColor(vectorColor);
            g2.setStroke(new BasicStroke(arrowWidth));
            g2.drawLine((int)px, (int)py, (int)endX, (int)endY);

            // Draw arrowhead
            double arrowHeadSize = 8.0 * (arrowWidth / 2.5);
            double angle = Math.atan2(endY - py, endX - px);
            int x1 = (int)(endX - arrowHeadSize * Math.cos(angle - Math.PI / 6));
            int y1 = (int)(endY - arrowHeadSize * Math.sin(angle - Math.PI / 6));
            int x2 = (int)(endX - arrowHeadSize * Math.cos(angle + Math.PI / 6));
            int y2 = (int)(endY - arrowHeadSize * Math.sin(angle + Math.PI / 6));

            g2.fillPolygon(new int[]{(int)endX, x1, x2}, new int[]{(int)endY, y1, y2}, 3);

            // Draw speed label if showing velocity comparison
            if (showVelocityComparison) {
                float labelFontSize = switch (uiSize) {
                    case SMALL -> 9f;
                    case NORMAL -> 11f;
                    default -> 13f;
                };

                Font oldFont = g2.getFont();
                g2.setFont(oldFont.deriveFont(Font.BOLD, labelFontSize));
                String speedLabel = String.format("%.1f km/s", speed);
                FontMetrics fm = g2.getFontMetrics();
                int labelWidth = fm.stringWidth(speedLabel);
                int labelHeight = fm.getHeight();

                // Position label near arrow tip
                int labelX = (int)endX + 10;
                int labelY = (int)endY - 5;

                // Background box
                g2.setColor(new Color(0, 0, 0, 180));
                g2.fillRoundRect(labelX - 3, labelY - labelHeight + 3, labelWidth + 6, labelHeight, 4, 4);

                // Text
                g2.setColor(vectorColor);
                g2.drawString(speedLabel, labelX, labelY);
                g2.setFont(oldFont);
            }
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

    /**
     * Display a lesson guide dialog for the current lesson.
     */
    private void showLessonGuide(Window owner) {
        String[] lessonContent = {
            // Lesson 1: Introduction to Orbital Motion
            """
            Lesson 1: Introduction to Orbital Motion
            ========================================
            
            Learning Objectives:
            • Understand elliptical vs. circular orbits
            • Identify key orbital elements (a, e, perihelion, aphelion)
            • Relate orbital period to distance from Sun
            
            Activities:
            
            1. Orbit Comparison
               - Press 'O' to show all planet orbits
               - Click Venus (nearly circular orbit)
               - Click Pluto (highly elliptical orbit)
               - Compare eccentricities in the info panel
            
            2. Kepler's Third Law Verification
               - Enable: Education → Lesson 1 Features → Show Kepler's 3rd Law
               - Click Mercury: note P² / a³ ratio
               - Click Earth: note P² / a³ ratio (should be ~1.0)
               - Click Jupiter: verify the ratio is constant
               - The ratio P² / a³ should be the same for all planets!
            
            3. Speed Variations
               - Click Mercury to follow it
               - Press Ctrl+Scroll to speed up time
               - Watch the speed value change in the info panel
               - Observe: faster near the Sun (perihelion), slower far away (aphelion)
               - Enable: Education → Lesson 1 Features → Show Perihelion/Aphelion
            
            Assessment Questions:
            - Why is Mercury's orbit more elliptical than Earth's?
            - How does orbital speed relate to distance from the Sun?
            - Calculate: If a planet orbits at 4 AU, what would its period be?
              (Hint: Use P² = a³ with a = 4, so P² = 64, P = 8 years)
            """,

            // Lesson 2: Scale of the Solar System
            """
            Lesson 2: Scale of the Solar System
            ===================================
            
            Learning Objectives:
            • Appreciate the vast distances in the Solar System
            • Understand the AU (Astronomical Unit) as a measurement
            • Compare planet sizes and distances
            
            Activities:
            
            1. Distance Exploration
               - Zoom out to see the full system
               - Use the dynamic scale ruler (bottom-left) to measure distances
               - Drag the ruler to different positions
               - Compare inner planet spacing vs. outer planets
            
            2. Zoom Challenge
               - Click Earth → auto-zoom to planet view
               - Press 'X' to cycle zoom modes (Planet → Orbit → Manual)
               - Observe how small Earth is compared to its orbit
            
            3. Belt Visualization
               - View → Belts & Lines → Asteroid Belt
               - View → Belts & Lines → Kuiper Belt
               - View → Belts & Lines → Frost Lines
               - Discuss where different materials can exist
            """,

            // Lesson 3: Moon Systems
            """
            Lesson 3: Moon Systems and Hierarchical Orbits
            ==============================================
            
            Learning Objectives:
            • Understand moons as satellites orbiting planets
            • Compare moon systems of different planets
            • Observe orbital relationships
            
            Activities:
            
            1. Earth-Moon System
               - Click Earth, then zoom in
               - Click the Moon to see its orbit details
               - Note the Moon's orbital period (27.3 days)
            
            2. Jupiter's Galilean Moons
               - Click Jupiter and zoom in
               - Observe Io, Europa, Ganymede, Callisto
               - Speed up time to see their orbital motions
            
            3. Saturn's Moons
               - Click Saturn
               - Toggle rings with 'R'
               - Observe Titan and other moons
            """,

            // Lesson 4: Time Control
            """
            Lesson 4: Time and Simulation Control
            =====================================
            
            Learning Objectives:
            • Understand different time scales in astronomy
            • Use simulation controls for observation
            
            Activities:
            
            1. Time Scale Manipulation
               - Use Ctrl+Scroll to speed up time
               - Watch Mercury complete one orbit (~88 days)
               - Press Space or 'P' to pause
            
            2. Study Configurations
               - Pause simulation at interesting moments
               - Record data from info panels
               - Resume and observe changes
            """,

            // Lesson 5: Planetary Characteristics
            """
            Lesson 5: Planetary Characteristics
            ==================================
            
            Learning Objectives:
            • Compare physical properties of planets
            • Understand planet classification
            
            Activities:
            
            1. Size Comparison
               - Click different planets to compare radii
               - Earth: 6,371 km
               - Jupiter: 69,911 km (11× Earth)
               - Mercury: 2,439 km (0.38× Earth)
            
            2. Orbit Color Coding
               - Purple/magenta: terrestrial planets
               - Golden/tan: gas giants
               - Purple-gray: dwarf planets
            
            3. Features
               - Toggle shading with 'S'
               - Toggle Saturn's rings with 'R'
            """
        };

        int lessonIndex = currentLesson - 1;
        if (lessonIndex < 0 || lessonIndex >= lessonContent.length) {
            lessonIndex = 0;
        }

        JTextArea textArea = new JTextArea(lessonContent[lessonIndex]);
        textArea.setEditable(false);
        textArea.setFont(new Font("Monospaced", Font.PLAIN, 12));
        textArea.setCaretPosition(0);

        JScrollPane scrollPane = new JScrollPane(textArea);
        scrollPane.setPreferredSize(new Dimension(650, 500));

        JOptionPane.showMessageDialog(
            owner,
            scrollPane,
            "Lesson " + currentLesson + " Guide",
            JOptionPane.INFORMATION_MESSAGE
        );
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
