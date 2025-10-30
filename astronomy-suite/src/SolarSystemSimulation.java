import javax.swing.*;
import java.awt.*;
import java.awt.event.*;
import java.awt.geom.Ellipse2D;
import java.awt.geom.Path2D;
import java.awt.geom.Point2D;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

/**
 * Solar System Simulation – real-time 2-D orbital mechanics with textures,
 * smooth camera, moon-following and auto-zoom.
 *
 * @author Ethan Lin (original) + texture / smooth-unfollow patch
 * @version 2.0
 */
public class SolarSystemSimulation extends JPanel
        implements ActionListener, MouseWheelListener, MouseListener {

    /* ==============================
       PHYSICAL CONSTANTS (SI UNITS)
       ============================== */
    private static final double AU  = 1.496e11;               // 1 Astronomical Unit in metres
    private static final double DAY = 86_400.0;               // Seconds in one Earth day
    private static final double G   = 6.67430e-11;             // Gravitational constant
    private static final double M_SUN = 1.98847e30;           // Mass of the Sun (kg)

    private static final double METERS_TO_PIXELS = 250.0 / AU;

    /* ==============================
       CAMERA SYSTEM
       ============================== */
    private int selectedPlanet = -1;
    private boolean followingPlanet = false;
    private int     cameraFocusIndex = -1;
    private double  camX = 0, camY = 0;

    private boolean cameraIsApproaching = false;
    private static final double ARRIVAL_THRESHOLD = 1e7;

    // ----- Moon selection -----
    private int selectedMoon = -1;
    private int   cameraFocusMoon = -1;
    private double moonCamX = 0, moonCamY = 0;
    private boolean followingMoon = false;
    private boolean moonCameraIsApproaching = false;

    // ----- Auto-zoom -----
    private boolean autoZoomEnabled = false;
    private double targetZoom = 1.0;
    private double currentZoom = 1.0;
    private static final double ZOOM_SPEED = 10.0;

    // ----- Orbit visibility -----
    private static final double ORBIT_VISIBLE_ZOOM = 5.0;
    private static final double PLANET_VISIBLE_ZOOM = 0.5;

    /* ==============================
       SIMULATION STATE
       ============================== */
    private double timeScale  = 1_000;
    private double simTimeSec = 0;
    private long   lastNanos;

    /* ==============================
       PLANET DATA
       ============================== */
    private static final String[] NAMES = {
            "Mercury", "Venus", "Earth", "Mars",
            "Jupiter", "Saturn", "Uranus", "Neptune", "Pluto"
    };

    private static final double[] A_METERS = {
            5.7909e10, 1.0821e11, 1.495978707e11, 2.2794e11,
            7.7854e11, 1.4298e12, 2.8725e12, 4.5045e12, 5.9064e12
    };
    private static final double[] ECC = {
            0.205630, 0.006772, 0.016709, 0.093405,
            0.048498, 0.055546, 0.046381, 0.008956, 0.248827
    };
    private static final double[] PERIOD_Y = {
            0.2408467, 0.61519726, 1.0000174, 1.8808476,
            11.862615, 29.447498, 84.016846, 164.79132, 247.92065
    };
    private static final double[] RADIUS_M = {
            2.439e6, 6.052e6, 6.371e6, 3.390e6,
            6.9911e7, 5.8232e7, 2.5362e7, 2.4622e7, 1.1883e6
    };
    private static final double[] PLANET_L0 = {
            252.25084, 181.97973, 100.46435, 355.45332, 34.40438,
            49.94432, 313.23218, 304.88003, 238.92881
    };
    private static final double[] PLANET_VARPI0 = {
            77.45736, 131.60261, 102.93735, 336.04084, 14.75385,
            92.43194, 170.96424, 44.97135, 113.76329
    };
    private static final double[] PLANET_OMEGA0 = {
            48.33167, 76.67992, 0.0, 49.57854, 100.55615,
            113.71504, 74.22950, 131.72169, 110.30347
    };
    private static final double SUN_RADIUS_M = 6.9634e8;
    private static final Color[] COLORS = {
            Color.GRAY, new Color(255,180,0), Color.CYAN, Color.RED,
            new Color(255,200,100), new Color(200,180,150),
            new Color(150,200,255), new Color(100,150,255), new Color(233,226,214)
    };

    /* ==============================
       MOON DATA
       ============================== */
    private static final double[][] MOON_A_M = {
            {}, {}, {3.844e8},
            {9.377e6, 2.356e7},
            {4.217e8, 6.71e8, 1.07e9, 1.883e9},
            {1.222e9, 2.38e8, 5.27e8},
            {4.363e8, 5.835e8, 4.989e8},
            {3.547e8}, {}
    };
    private static final double[][] MOON_PERIOD_DAYS = {
            {}, {}, {27.321661},
            {0.318910, 1.26244},
            {1.769278, 3.551181, 7.154552, 16.689018},
            {15.94542, 1.370218, 4.518212},
            {8.706234, 13.46339, 4.144176},
            {5.876854}, {}
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
            {}, {}, {0.05490},
            {0.01550, 0.00021},
            {0.00412, 0.00899, 0.00126, 0.00716},
            {0.02880, 0.00470, 0.00100},
            {0.00110, 0.00090, 0.00390},
            {0.000016}, {}
    };
    private static final double[][] MOON_ARG_PERI = {
            {}, {}, {2.034},
            {Math.PI/2, Math.PI},
            {1.88, 0.85, 1.02, 5.92},
            {0.20, 2.29, 5.80},
            {3.76, 0.44, 4.72},
            {Math.PI}, {}
    };

    // ----- Moon physical radii (computed from planet radius * ratio) -----
    private static final double[][] MOON_RADIUS_M = new double[NAMES.length][];
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
       TEXTURES
       ============================== */
    private static final Map<String, Image> TEXTURES = new HashMap<>();

    private static final String[] PLANET_TEXTURES = {
            "mercury", "venus", "earth", "mars",
            "jupiter", "saturn", "uranus", "neptune", "pluto"
    };
    private static final String[][] MOON_TEXTURES = {
            {}, {}, {"moon"},
            {"phobos", "deimos"},
            {"io", "europa", "ganymede", "callisto"},
            {"titan", "enceladus", "rhea"},
            {"titania", "oberon", "umbriel"},
            {"triton"}, {}
    };
    private static final String SUN_TEXTURE = "sun";
    private static final String SATURN_RINGS = "saturn_rings";

    /* ==============================
       ORBITAL STATE
       ============================== */
    private final double[] meanAnomaly = new double[NAMES.length];
    private final double[][] moonMean = new double[NAMES.length][];

    private static final int ORBIT_RESOLUTION = 2000;
    private final ArrayList<ArrayList<Point2D.Double>> orbitPaths = new ArrayList<>();
    private final ArrayList<ArrayList<ArrayList<Point2D.Double>>> moonOrbitPaths = new ArrayList<>();

    private final double[] planetX = new double[NAMES.length];
    private final double[] planetY = new double[NAMES.length];
    private final double[] planetDistM = new double[NAMES.length];
    private final double[] planetSpeedKmS = new double[NAMES.length];

    /* ==============================
       CONSTRUCTOR
       ============================== */
    public SolarSystemSimulation() {
        setBackground(Color.BLACK);
        setPreferredSize(new Dimension(1200, 800));

        addMouseWheelListener(this);
        addMouseListener(this);

        // ----- initialise planet positions -----
        for (int i = 0; i < NAMES.length; i++) {
            double L_deg = PLANET_L0[i];
            double varpi_deg = PLANET_VARPI0[i];
            double M_deg = L_deg - varpi_deg;
            meanAnomaly[i] = Math.toRadians(M_deg % 360.0);

            double omega_rad = Math.toRadians(PLANET_VARPI0[i] - PLANET_OMEGA0[i]);
            double Omega_rad = Math.toRadians(PLANET_OMEGA0[i]);
            Point2D.Double pos = computeInitialPosition(A_METERS[i], ECC[i], omega_rad, Omega_rad, meanAnomaly[i]);
            planetX[i] = pos.x;
            planetY[i] = pos.y;
            planetDistM[i] = Math.hypot(pos.x, pos.y);
        }

        // ----- initialise moon mean anomalies -----
        for (int i = 0; i < NAMES.length; i++) {
            int num = MOON_NAMES[i].length;
            moonMean[i] = new double[num];
        }

        // ----- pre-compute orbits -----
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

        // ----- key listener (Z) -----
        setFocusable(true);
        addKeyListener(new KeyAdapter() {
            @Override public void keyPressed(KeyEvent e) {
                if (e.getKeyCode() == KeyEvent.VK_Z) {
                    autoZoomEnabled = !autoZoomEnabled;
                    if (!autoZoomEnabled) targetZoom = currentZoom;
                }
            }
        });

        // ----- load textures -----
        loadTexture(SUN_TEXTURE);
        for (String name : PLANET_TEXTURES) loadTexture(name);
        loadTexture(SATURN_RINGS);
        for (String[] moons : MOON_TEXTURES) {
            for (String name : moons) loadTexture(name);
        }

        // ----- start timer -----
        Timer timer = new Timer(16, this);
        timer.start();
        lastNanos = System.nanoTime();
    }

    private void loadTexture(String name) {
        try {
            String path = "/resources/" + name + (name.equals("sun") ? ".jpg" : ".png");
            java.net.URL url = getClass().getResource(path);
            if (url != null) {
                Image img = new ImageIcon(url).getImage();
                TEXTURES.put(name, img);
            } else {
                System.err.println("Texture not found: " + path);
            }
        } catch (Exception e) {
            System.err.println("Failed to load texture: " + name);
        }
    }

    /* ==============================
       SIMULATION UPDATE
       ============================== */
    @Override
    public void actionPerformed(ActionEvent ev) {
        long now = System.nanoTime();
        double dtReal = (now - lastNanos) / 1e9;
        lastNanos = now;

        double dtSim = dtReal * timeScale;
        simTimeSec += dtSim;

        // ----- planets -----
        for (int i = 0; i < NAMES.length; i++) {
            double periodSec = PERIOD_Y[i] * 365.25 * DAY;
            double n = 2 * Math.PI / periodSec;
            meanAnomaly[i] = (meanAnomaly[i] + n * dtSim) % (2 * Math.PI);
        }

        // ----- moons -----
        for (int i = 0; i < NAMES.length; i++) {
            for (int m = 0; m < moonMean[i].length; m++) {
                double periodSec = MOON_PERIOD_DAYS[i][m] * DAY;
                double n = 2 * Math.PI / periodSec;
                moonMean[i][m] = (moonMean[i][m] + n * dtSim) % (2 * Math.PI);
            }
        }

        // ----- planet positions -----
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
        }

        /* ----- CAMERA & AUTO-ZOOM ----- */
        if (followingMoon && cameraFocusIndex >= 0 && cameraFocusMoon >= 0) {
            // ----- MOON -----
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

            // auto-zoom: moon body
            if (autoZoomEnabled) {
                double moonRadius = MOON_RADIUS_M[p][m];
                double maxRadius = moonRadius * 1.5;
                int screen = Math.min(getWidth(), getHeight());
                double desired = screen * 0.6;
                targetZoom = desired / (maxRadius * METERS_TO_PIXELS * 2);
                targetZoom = Math.max(50.0, Math.min(targetZoom, 50000.0));
            }

        } else if (followingPlanet && cameraFocusIndex >= 0) {
            // ----- PLANET -----
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

            // auto-zoom: planet + moons
            if (autoZoomEnabled) {
                double maxRadius = RADIUS_M[cameraFocusIndex];
                for (int m = 0; m < MOON_A_M[cameraFocusIndex].length; m++) {
                    double moonDist = MOON_A_M[cameraFocusIndex][m] * 1.1;
                    if (moonDist > maxRadius) maxRadius = moonDist;
                }
                int screen = Math.min(getWidth(), getHeight());
                double desired = screen * 0.6;
                targetZoom = desired / (maxRadius * METERS_TO_PIXELS * 2);
                targetZoom = Math.max(5.0, Math.min(targetZoom, 5000.0));
            }

        } else {
            // ----- NOT FOLLOWING ANYTHING (smooth return to Sun) -----
            double speed = 8.0;
            double lerp = 1 - Math.exp(-speed * dtReal);
            camX *= (1 - lerp);
            camY *= (1 - lerp);
            moonCamX = camX; moonCamY = camY;

            targetZoom = 1.0;
            cameraIsApproaching = moonCameraIsApproaching = false;
            autoZoomEnabled = false;
        }

        // ----- smooth zoom interpolation -----
        if (autoZoomEnabled) {
            double lerp = 1 - Math.exp(-ZOOM_SPEED * dtReal);
            currentZoom += (targetZoom - currentZoom) * lerp;
        }

        repaint();
    }

    /* ==============================
       INPUT
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
            targetZoom = currentZoom;
        }
    }

    private void handleClick(int mx, int my) {
        int cx = getWidth() / 2, cy = getHeight() / 2;

        // ----- MOON CLICK -----
        if (followingPlanet && cameraFocusIndex >= 0 && currentZoom > 10) {
            int p = cameraFocusIndex;
            for (int m = 0; m < MOON_A_M[p].length; m++) {
                // moon world position
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
                        // unfollow moon → back to planet
                        followingMoon = false;
                        cameraFocusMoon = -1;
                        selectedMoon = -1;
                        moonCameraIsApproaching = false;
                        camX = planetX[p];
                        camY = planetY[p];
                        cameraIsApproaching = false;
                    } else {
                        // follow moon
                        followingMoon = true;
                        moonCameraIsApproaching = true;
                        cameraFocusMoon = m;
                        selectedMoon = m;
                        moonCamX = camX;
                        moonCamY = camY;
                        cameraIsApproaching = false;
                    }
                    return;
                }
            }
        }

        // ----- PLANET CLICK -----
        for (int i = 0; i < NAMES.length; i++) {
            double px = cx + (planetX[i] - camX) * METERS_TO_PIXELS * currentZoom;
            double py = cy + (planetY[i] - camY) * METERS_TO_PIXELS * currentZoom;
            double pr = Math.max(RADIUS_M[i] * METERS_TO_PIXELS * currentZoom, 4);

            double dx = mx - px, dy = my - py;
            if (dx * dx + dy * dy < pr * pr * 4) {
                if (cameraFocusIndex == i && followingPlanet && !followingMoon) {
                    // unfollow everything
                    followingPlanet = false;
                    cameraFocusIndex = -1;
                    cameraFocusMoon = -1;
                    selectedPlanet = -1;
                    selectedMoon = -1;
                    cameraIsApproaching = false;
                    moonCameraIsApproaching = false;
                } else {
                    // follow planet
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

        // ----- EMPTY SPACE (smooth return) -----
        followingPlanet = followingMoon = false;
        cameraFocusIndex = cameraFocusMoon = -1;
        selectedPlanet = selectedMoon = -1;
        cameraIsApproaching = moonCameraIsApproaching = false;
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
        g2.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);
        g2.setRenderingHint(RenderingHints.KEY_RENDERING, RenderingHints.VALUE_RENDER_QUALITY);

        int cx = getWidth() / 2, cy = getHeight() / 2;
        java.util.List<Rectangle> labelBoxes = new ArrayList<>();

        // ----- SUN -----
        double sunR = SUN_RADIUS_M * METERS_TO_PIXELS * currentZoom;
        double sunX = cx - sunR - camX * METERS_TO_PIXELS * currentZoom;
        double sunY = cy - sunR - camY * METERS_TO_PIXELS * currentZoom;

        Image sunImg = TEXTURES.get(SUN_TEXTURE);
        if (sunImg != null) {
            g2.drawImage(sunImg, (int)sunX, (int)sunY, (int)(2*sunR), (int)(2*sunR), null);
        } else {
            g2.setColor(Color.ORANGE);
            g2.fill(new Ellipse2D.Double(sunX, sunY, 2*sunR, 2*sunR));
        }

        FontMetrics fm = g2.getFontMetrics();

        // ----- PLANETS & MOONS -----
        for (int i = 0; i < NAMES.length; i++) {
            double worldX = planetX[i], worldY = planetY[i];
            double px = cx + (worldX - camX) * METERS_TO_PIXELS * currentZoom;
            double py = cy + (worldY - camY) * METERS_TO_PIXELS * currentZoom;
            double pr = RADIUS_M[i] * METERS_TO_PIXELS * currentZoom;
            if (pr < 0.5) pr = 0.5;

            // orbit
            Path2D.Double path = getADouble(i, cx, cy);
            g2.setColor(currentZoom < PLANET_VISIBLE_ZOOM * 10 ? new Color(74,74,74) : Color.DARK_GRAY);
            g2.setStroke(new BasicStroke(1f));
            g2.draw(path);

            // ----- planet texture -----
            String texName = PLANET_TEXTURES[i];
            Image planetImg = TEXTURES.get(texName);
            if (planetImg != null && pr > 3) {
                int ix = (int)(px - pr);
                int iy = (int)(py - pr);
                int iw = (int)(2*pr);
                int ih = (int)(2*pr);
                g2.drawImage(planetImg, ix, iy, iw, ih, null);
            } else {
                g2.setColor(COLORS[i]);
                g2.fill(new Ellipse2D.Double(px - pr, py - pr, 2*pr, 2*pr));
            }

            // Saturn rings (behind)
            if (i == 5) {
                Image rings = TEXTURES.get(SATURN_RINGS);
                if (rings != null && pr > 10) {
                    double scale = 2.7;
                    double rR = pr * scale;
                    int rx = (int)(px - rR);
                    int ry = (int)(py - rR);
                    int rw = (int)(2*rR);
                    int rh = (int)(2*rR);
                    g2.drawImage(rings, rx, ry, rw, rh, null);
                }
            }

            // label
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

            // selection highlight
            if (i == selectedPlanet) {
                g2.setColor(Color.YELLOW);
                g2.draw(new Ellipse2D.Double(px - pr - 3, py - pr - 3, 2*pr + 6, 2*pr + 6));
            }

            // ----- MOONS -----
            if (MOON_NAMES[i].length > 0) {
                // moon orbits
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

                // moon bodies
                if (currentZoom > 10) {
                    for (int m = 0; m < MOON_A_M[i].length; m++) {
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

                        // texture
                        String moonTex = MOON_TEXTURES[i][m].toLowerCase();
                        Image moonImg = TEXTURES.get(moonTex);
                        if (moonImg != null && mr > 2) {
                            int ix = (int)(mpx - mr);
                            int iy = (int)(mpy - mr);
                            int iw = (int)(2*mr);
                            int ih = (int)(2*mr);
                            g2.drawImage(moonImg, ix, iy, iw, ih, null);
                        } else {
                            g2.setColor(Color.LIGHT_GRAY);
                            g2.fill(new Ellipse2D.Double(mpx - mr, mpy - mr, 2*mr, 2*mr));
                        }

                        // label
                        g2.setColor(Color.WHITE);
                        g2.drawString(MOON_NAMES[i][m], (int)(mpx + mr + 4), (int)(mpy - mr - 2));

                        // selection highlight
                        if (followingPlanet && selectedMoon == m) {
                            g2.setColor(Color.YELLOW);
                            g2.draw(new Ellipse2D.Double(mpx - mr - 3, mpy - mr - 3, 2*mr + 6, 2*mr + 6));
                        }
                    }
                }
            }
        }

        // ----- HUD -----
        g2.setColor(Color.WHITE);
        g2.drawString(String.format("Zoom: %.2fx %s", currentZoom,
                followingPlanet&&autoZoomEnabled ? "(Auto)" : "(Manual)"), 10, 20);
        g2.drawString(String.format("Time ×%.0f (%.1f days/s)", timeScale, timeScale/DAY), 10, 40);
        g2.drawString(String.format("Sim: %.2f years", simTimeSec/(365.25*DAY)), 10, 60);

        if (followingMoon && cameraFocusIndex >= 0 && cameraFocusMoon >= 0) {
            int p = cameraFocusIndex, m = cameraFocusMoon;
            g2.drawString(String.format("Following: %s → %s",
                    NAMES[p], MOON_NAMES[p][m]), 10, 85);
            g2.drawString(String.format("Moon Radius: %.0f km | Zoom: %.1fx (Auto)",
                    MOON_RADIUS_M[p][m]/1_000, targetZoom), 10, 105);
        } else if (followingPlanet && cameraFocusIndex >= 0) {
            g2.drawString(String.format("Following: %s", NAMES[cameraFocusIndex]), 10, 85);
            g2.drawString(String.format("Moons: %d | Target Zoom: %.1fx",
                    MOON_NAMES[cameraFocusIndex].length, targetZoom), 10, 105);
        } else {
            g2.drawString("Scroll=Zoom | Ctrl+Scroll=Speed | Click=Follow", 10, 85);
        }
        g2.drawString("Z=Toggle Auto-Zoom When Selecting Planet/Moon", 10, 125);

        // ----- INFO PANEL -----
        if (selectedPlanet != -1) {
            int i = selectedPlanet;
            int w = 300, h = 140, x0 = getWidth() - w - 20, y0 = 20;
            g2.setColor(new Color(0,0,0,150));
            g2.fillRoundRect(x0, y0, w, h, 15, 15);
            g2.setColor(Color.WHITE);
            g2.drawRoundRect(x0, y0, w, h, 15, 15);
            g2.drawString("Planet: " + NAMES[i], x0+15, y0+25);
            g2.drawString(String.format("Radius: %.0f km", RADIUS_M[i]/1_000), x0+15, y0+45);
            g2.drawString(String.format("Dist: %.3f AU", planetDistM[i]/AU), x0+15, y0+65);
            g2.drawString(String.format("Speed: %.2f km/s", planetSpeedKmS[i]), x0+15, y0+85);
            g2.drawString(String.format("Period: %.3f y", PERIOD_Y[i]), x0+15, y0+105);
            if (MOON_NAMES[i].length > 0)
                g2.drawString(String.format("Moons: %d", MOON_NAMES[i].length), x0+15, y0+125);
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

    /* ==============================
       MAIN
       ============================== */
    public static void main(String[] args) {
        JFrame frame = new JFrame("Solar System Simulation – Textures & Smooth Unfollow");
        frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        frame.add(new SolarSystemSimulation());
        frame.pack();
        frame.setLocationRelativeTo(null);
        frame.setVisible(true);
    }
}