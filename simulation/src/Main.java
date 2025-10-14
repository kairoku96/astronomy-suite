// File: GravitySimGridDentLines.java
// Requires: LWJGL3 jars + joml-1.xx.jar on the classpath
// Compile example:
// javac -cp "libs/*" GravitySimGridDentLines.java
// Run example:
// java  -cp ".;libs/*" GravitySimGridDentLines    (use ':' on mac/linux)

import org.lwjgl.*;
import org.lwjgl.glfw.*;
import org.lwjgl.opengl.*;
import org.lwjgl.system.*;
import org.joml.*;

import java.lang.Math;
import java.nio.*;
import java.util.*;
import java.util.Random;

import static org.lwjgl.glfw.GLFW.*;
import static org.lwjgl.opengl.GL33C.*;
import static org.lwjgl.system.MemoryUtil.*;

import org.lwjgl.stb.STBTTAlignedQuad;
import org.lwjgl.stb.STBTTFontinfo;
import org.lwjgl.stb.STBTTPackContext;
import org.lwjgl.stb.STBTTPackedchar;
import static org.lwjgl.stb.STBTruetype.*;

public class Main {

    // Window
    private long window;
    private int width = 2048, height = 1280;

    // Font and Text Rendering
    private int textShaderProgram;
    private int textVAO, textVBO;
    private int fontTexture;
    private STBTTFontinfo fontInfo;
    private STBTTPackedchar.Buffer cdata;
    private static final int BITMAP_W = 512;
    private static final int BITMAP_H = 512;
    private static final float FONT_SIZE = 12.0f;
    private final float LABEL_OFFSET = 10.0f; // Pixels above planet in screen space

    private static List<Integer> moonParents = new ArrayList<>();

    // Physics (SI)
    private final double G = 6.67430e-11;
    // Visualization scaling: meters -> scene units
    private final double VIS_SCALE = 2e-8; // ~1 AU (1.496e11 m) -> 1496 scene units
    // System coverage (scene units)
    private final double GRID_HALF_SIZE_SCENE = 1500.0 * 90; // ~ ±2 AU
    private static final int GRID_DIVISIONS = 800;

    // Simulation time scaling
    private final double SIM_SECONDS_PER_REAL_SECOND = 60.0 * 60.0 * 24.0; // 1 real second => 1 day
    private boolean paused = false;

    private double mu_sun;
    private final double ORBIT_DT_FACTOR = 0.1; // Smaller time step for moons in orbit precomputation

    // Camera
    private Vector3f camPos = new Vector3f(0, 1200, 6000);
    private Vector3f camFront = new Vector3f(0, 0, -1);
    private Vector3f camUp = new Vector3f(0, 1, 0);
    private float yaw = -90f, pitch = -8f;
    private float lastX = width / 2f, lastY = height / 2f;
    private boolean firstMouse = true;
    private float fov = 45f;

    // GL objects
    private int sphereVAO, sphereVBO;
    private int sphereVertexCount;
    private int shaderProgram;

    // Grid VBO/VAO
    private int gridVAO, gridVBO;
    private int gridVertexCount;

    // Trails per planet
    private final int MAX_TRAIL_POINTS = 512;
    private final List<Integer> trailVAOs = new ArrayList<>();
    private final List<Integer> trailVBOs = new ArrayList<>();
    private final List<Boolean> trailUpdated = new ArrayList<>(); // Flag to update VBO only when necessary

    // N-body predicted orbits
    private final List<Integer> nbodyOrbitVAOs = new ArrayList<>();
    private final List<Integer> nbodyOrbitVBOs = new ArrayList<>();
    private final List<Integer> nbodyOrbitCounts = new ArrayList<>();
    private final List<Vector4f> nbodyOrbitColors = new ArrayList<>(); // Color with alpha for transparency

    // Visual tuning
    private final double DISPLAY_RADIUS_FACTOR = 1.0; // multiplier applied to radius*VIS_SCALE to get display radius
    private static final double SUN_DEPTH = 600.0; // Depth for Sun dent in scene units
    private static final double SUN_SIGMA = 1500.0; // Width for Sun dent in scene units
    private static final double MASS_POWER = 1; // Power to boost small planet dents
    private final double TRAIL_POINT_MIN_DISTANCE = 5.0; // scene units - only add trail point if moved this far

    // Planet structure (SI units for pos, vel, mass, radius)
    static class Planet {
        Vector3d pos;
        Vector3d vel;
        double mass;
        double radius;
        Vector3f color;
        float displayRadius;
        String name;
        LinkedList<Vector3f> trail = new LinkedList<>();
        // Add orbital elements for analytic orbit generation
        double a_au, e, i_rad, omega_rad, Omega_rad, M_rad, mu;
        Planet(Vector3d pos, Vector3d vel, double mass, double radius, Vector3f color, String name) {
            this.pos = pos;
            this.vel = vel;
            this.mass = mass;
            this.radius = radius;
            this.color = color;
            this.name = name;
        }
    }


    private final List<Planet> planets = new ArrayList<>();

    // Shader uniform locations for dents
    private int useDentLoc;
    private int planetXZLoc;
    private int planetDepthsLoc;
    private int planetSigmasLoc;
    private int numPlanetsLoc;

    // Precomputed depths and sigmas
    private float[] depths;
    private float[] sigmas;

    public static void main(String[] args) {
        new Main().run();
    }

    public void run() {
        init();
        loop();
        cleanup();
    }

    private void init() {
        if (!glfwInit()) throw new IllegalStateException("Unable to init GLFW");

        glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
        glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
        glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

        window = glfwCreateWindow(width, height, "Gravity: Planets + Grid (Flamm-like dents, lines)", NULL, NULL);
        if (window == NULL) throw new RuntimeException("Failed to create GLFW window");

        glfwMakeContextCurrent(window);
        glfwSwapInterval(1);
        GL.createCapabilities();

        glfwSetCursorPosCallback(window, this::mouseMove);
        glfwSetScrollCallback(window, this::scroll);
        glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);

        shaderProgram = buildShaders();

        textShaderProgram = buildTextShaders();
        initFont();
        createTextQuad();

        // Get new uniform locations
        useDentLoc = glGetUniformLocation(shaderProgram, "useDent");
        planetXZLoc = glGetUniformLocation(shaderProgram, "planetXZ[0]");
        planetDepthsLoc = glGetUniformLocation(shaderProgram, "planetDepths[0]");
        planetSigmasLoc = glGetUniformLocation(shaderProgram, "planetSigmas[0]");
        numPlanetsLoc = glGetUniformLocation(shaderProgram, "numPlanets");

        createSphereMesh(28, 28);
        createGridLines(GRID_DIVISIONS, GRID_HALF_SIZE_SCENE);

        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        glEnable(GL_DEPTH_TEST);
        glLineWidth(1.2f);

        // Constants
        double G = 6.67430e-11; // Gravitational constant
        double AU = 1.495978707e11; // Astronomical Unit (m)
        double T = 9414.5 / 36525.0; // Julian centuries since J2000 (≈ year 2025)

        // Sun
        Planet sun = new Planet(new Vector3d(0, 0, 0), new Vector3d(0, 0, 0),
                1.98847e30, 696340e3, new Vector3f(1.0f, 0.95f, 0.2f), "Sun");
        moonParents.add(0);
        planets.add(sun);

        double mu_sun = G * sun.mass;
        this.mu_sun = mu_sun;

        // -------------------- Mercury --------------------
        double a_mercury = 0.38709927 + 0.00000037 * T;
        double e_mercury = 0.20563593 + 0.00001906 * T;
        double i_mercury = Math.toRadians(7.00487);
        double Omega_mercury = Math.toRadians(48.33131383 - 0.12534081 * T);
        double varpi_mercury = Math.toRadians(77.45779628 + 0.16047689 * T);
        double L_mercury = Math.toRadians(252.25032350 + 149472.67411175 * T);
        double omega_mercury = varpi_mercury - Omega_mercury;
        double M_mercury = L_mercury - varpi_mercury;
        Vector3d[] state_mercury = getOrbitalState(a_mercury, e_mercury, i_mercury, omega_mercury, Omega_mercury, M_mercury, mu_sun);
        Planet mercury = new Planet(state_mercury[0], state_mercury[1], 3.3011e23, 2439.7e3, new Vector3f(0.7f, 0.7f, 0.7f), "Mercury");
        mercury.a_au = a_mercury;
        mercury.e = e_mercury;
        mercury.i_rad = i_mercury;
        mercury.omega_rad = omega_mercury;
        mercury.Omega_rad = Omega_mercury;
        mercury.M_rad = M_mercury;
        mercury.mu = mu_sun;
        moonParents.add(1); // Should be 0 (orbits Sun)
        planets.add(mercury);

        // -------------------- Venus --------------------
        double a_venus = 0.72333566 + 0.00000390 * T;
        double e_venus = 0.00677672 - 0.00004107 * T;
        double i_venus = Math.toRadians(3.39471);
        double Omega_venus = Math.toRadians(76.67984255 - 0.27769418 * T);
        double varpi_venus = Math.toRadians(131.60246718 + 0.00268329 * T);
        double L_venus = Math.toRadians(181.97909950 + 58517.81538729 * T);
        double omega_venus = varpi_venus - Omega_venus;
        double M_venus = L_venus - varpi_venus;
        Vector3d[] state_venus = getOrbitalState(a_venus, e_venus, i_venus, omega_venus, Omega_venus, M_venus, mu_sun);
        Planet venus = new Planet(state_venus[0], state_venus[1], 4.8675e24, 6051.8e3, new Vector3f(0.95f, 0.7f, 0.4f), "Venus");
        venus.a_au = a_venus;
        venus.e = e_venus;
        venus.i_rad = i_venus;
        venus.omega_rad = omega_venus;
        venus.Omega_rad = Omega_venus;
        venus.M_rad = M_venus;
        venus.mu = mu_sun;
        moonParents.add(2); // Should be 0
        planets.add(venus);

        // -------------------- Earth --------------------
        double a_earth = 1.00000261 + 0.00000562 * T;
        double e_earth = 0.01671123 - 0.00004392 * T;
        double i_earth = Math.toRadians(0.00005);
        double Omega_earth = Math.toRadians(0.0);
        double varpi_earth = Math.toRadians(102.93768193 + 0.32327364 * T);
        double L_earth = Math.toRadians(100.46457166 + 35999.37244981 * T);
        double omega_earth = varpi_earth - Omega_earth;
        double M_earth = L_earth - varpi_earth;
        Vector3d[] state_earth = getOrbitalState(a_earth, e_earth, i_earth, omega_earth, Omega_earth, M_earth, mu_sun);
        Planet earth = new Planet(state_earth[0], state_earth[1], 5.97219e24, 6378e3, new Vector3f(0.18f, 0.45f, 0.9f), "Earth");
        earth.a_au = a_earth;
        earth.e = e_earth;
        earth.i_rad = i_earth;
        earth.omega_rad = omega_earth;
        earth.Omega_rad = Omega_earth;
        earth.M_rad = M_earth;
        earth.mu = mu_sun;
        moonParents.add(3);
        planets.add(earth);

        // -------------------- Earth's Moon --------------------
        double mu_earth = G * earth.mass;
        double a_moon = 384399e3 / AU;
        double e_moon = 0.0549;
        double i_moon = Math.toRadians(5.145);
        double Omega_moon = Math.toRadians(Math.random() * 360);
        double omega_moon = Math.toRadians(Math.random() * 360);
        double M_moon = Math.random() * 2 * Math.PI;
        Vector3d[] state_moon = getOrbitalState(a_moon, e_moon, i_moon, omega_moon, Omega_moon, M_moon, mu_earth);
        state_moon[0].add(earth.pos);
        state_moon[1].add(earth.vel);
        Planet moon = new Planet(state_moon[0], state_moon[1], 7.346e22, 1737.4e3, new Vector3f(0.8f, 0.8f, 0.8f), "Moon");
        moonParents.add(-1);
        planets.add(moon);

        // -------------------- Mars --------------------
        double a_mars = 1.52371034 + 0.00001847 * T;
        double e_mars = 0.09339410 + 0.00007882 * T;
        double i_mars = Math.toRadians(1.85061);
        double Omega_mars = Math.toRadians(49.55953891 - 0.29257343 * T);
        double varpi_mars = Math.toRadians(-23.94362959 + 0.44441088 * T);
        double L_mars = Math.toRadians(-4.55343205 + 19140.30268499 * T);
        double omega_mars = varpi_mars - Omega_mars;
        double M_mars = L_mars - varpi_mars;
        Vector3d[] state_mars = getOrbitalState(a_mars, e_mars, i_mars, omega_mars, Omega_mars, M_mars, mu_sun);
        Planet mars = new Planet(state_mars[0], state_mars[1], 6.4171e23, 3389.5e3, new Vector3f(1.0f, 0.45f, 0.25f), "Mars");
        mars.a_au = a_mars;
        mars.e = e_mars;
        mars.i_rad = i_mars;
        mars.omega_rad = omega_mars;
        mars.Omega_rad = Omega_mars;
        mars.M_rad = M_mars;
        mars.mu = mu_sun;
        moonParents.add(5);
        planets.add(mars);

        // -------------------- Mars' Moons --------------------
        double mu_mars = G * mars.mass;
        // Phobos
        double a_phobos = 9376e3 / AU;
        double e_phobos = 0.0151;
        Vector3d[] state_phobos = getOrbitalState(a_phobos, e_phobos, i_mars, omega_mars, Omega_mars, Math.random() * 2 * Math.PI, mu_mars);
        state_phobos[0].add(mars.pos);
        state_phobos[1].add(mars.vel);
        Planet phobos = new Planet(state_phobos[0], state_phobos[1], 1.0659e16, 11.1e3, new Vector3f(0.9f, 0.7f, 0.7f), "Phobos");
        moonParents.add(-1);
        planets.add(phobos);
        // Deimos
        double a_deimos = 23458e3 / AU;
        double e_deimos = 0.00033;
        Vector3d[] state_deimos = getOrbitalState(a_deimos, e_deimos, i_mars, omega_mars, Omega_mars, Math.random() * 2 * Math.PI, mu_mars);
        state_deimos[0].add(mars.pos);
        state_deimos[1].add(mars.vel);
        Planet deimos = new Planet(state_deimos[0], state_deimos[1], 1.4762e15, 6.2e3, new Vector3f(0.85f, 0.65f, 0.65f), "Deimos");
        moonParents.add(-1);
        planets.add(deimos);

        // -------------------- Jupiter --------------------
        double a_jupiter = 5.20288700 - 0.00011607 * T;
        double e_jupiter = 0.04838624 + 0.00001381 * T;
        double i_jupiter = Math.toRadians(1.30530);
        double Omega_jupiter = Math.toRadians(100.49267687 - 0.20497492 * T);
        double varpi_jupiter = Math.toRadians(14.72847983 + 0.21252668 * T);
        double L_jupiter = Math.toRadians(34.35148386 + 3034.74612775 * T);
        double omega_jupiter = varpi_jupiter - Omega_jupiter;
        double M_jupiter = L_jupiter - varpi_jupiter;
        Vector3d[] state_jupiter = getOrbitalState(a_jupiter, e_jupiter, i_jupiter, omega_jupiter, Omega_jupiter, M_jupiter, mu_sun);
        Planet jupiter = new Planet(state_jupiter[0], state_jupiter[1], 1.8982e27, 69911e3, new Vector3f(0.95f, 0.8f, 0.6f), "Jupiter");
        jupiter.a_au = a_jupiter;
        jupiter.e = e_jupiter;
        jupiter.i_rad = i_jupiter;
        jupiter.omega_rad = omega_jupiter;
        jupiter.Omega_rad = Omega_jupiter;
        jupiter.M_rad = M_jupiter;
        jupiter.mu = mu_sun;
        moonParents.add(8);
        planets.add(jupiter);

        // Jupiter’s moons
        double mu_jupiter = G * jupiter.mass;
        double[][] jMoons = {
                {421800e3 / AU, 0.0041}, // Io
                {671100e3 / AU, 0.0094}, // Europa
                {1070400e3 / AU, 0.0013}, // Ganymede
                {1882700e3 / AU, 0.0074}  // Callisto
        };
        double[][] jMoonProps = {
                {8.9319e22, 1821.6e3},
                {4.7998e22, 1560.8e3},
                {1.4819e23, 2631.2e3},
                {1.0759e23, 2410.3e3}
        };
        Vector3f[] jMoonColors = {
                new Vector3f(0.9f, 0.9f, 0.5f),
                new Vector3f(0.8f, 0.8f, 0.9f),
                new Vector3f(0.7f, 0.6f, 0.5f),
                new Vector3f(0.6f, 0.5f, 0.4f)
        };
        String[] jMoonNames = {"Io", "Europa", "Ganymede", "Callisto"};
        for (int i = 0; i < jMoons.length; i++) {
            Vector3d[] s = getOrbitalState(jMoons[i][0], jMoons[i][1], i_jupiter, omega_jupiter, Omega_jupiter, Math.random() * 2 * Math.PI, mu_jupiter);
            s[0].add(jupiter.pos);
            s[1].add(jupiter.vel);
            moonParents.add(-1);
            planets.add(new Planet(s[0], s[1], jMoonProps[i][0], jMoonProps[i][1], jMoonColors[i], jMoonNames[i]));
        }

        // -------------------- Saturn --------------------
        double a_saturn = 9.53667594 - 0.00125060 * T;
        double e_saturn = 0.05386179 - 0.00050991 * T;
        double i_saturn = Math.toRadians(2.48446);
        double Omega_saturn = Math.toRadians(113.66242448 - 0.28867794 * T);
        double varpi_saturn = Math.toRadians(92.59887831 + 0.37809336 * T);
        double L_saturn = Math.toRadians(49.95424423 + 1222.49362201 * T);
        double omega_saturn = varpi_saturn - Omega_saturn;
        double M_saturn = L_saturn - varpi_saturn;
        Vector3d[] state_saturn = getOrbitalState(a_saturn, e_saturn, i_saturn, omega_saturn, Omega_saturn, M_saturn, mu_sun);
        Planet saturn = new Planet(state_saturn[0], state_saturn[1], 5.6834e26, 58232e3, new Vector3f(0.9f, 0.75f, 0.5f), "Saturn");
        saturn.a_au = a_saturn;
        saturn.e = e_saturn;
        saturn.i_rad = i_saturn;
        saturn.omega_rad = omega_saturn;
        saturn.Omega_rad = Omega_saturn;
        saturn.M_rad = M_saturn;
        saturn.mu = mu_sun;
        moonParents.add(13);
        planets.add(saturn);

        // Saturn’s moons
        double mu_saturn = G * saturn.mass;
        double[][] sMoons = {
                {1221870e3 / AU, 0.0288}, // Titan
                {527108e3 / AU, 0.001},   // Rhea
                {3560820e3 / AU, 0.0286}, // Iapetus
                {377396e3 / AU, 0.0022}   // Dione
        };
        double[][] sMoonProps = {
                {1.3452e23, 2574.7e3},
                {2.3065e21, 763.8e3},
                {1.8053e21, 734.5e3},
                {1.0955e21, 561.4e3}
        };
        Vector3f[] sMoonColors = {
                new Vector3f(0.9f, 0.6f, 0.3f),
                new Vector3f(0.8f, 0.8f, 0.8f),
                new Vector3f(0.7f, 0.7f, 0.7f),
                new Vector3f(0.75f, 0.75f, 0.75f)
        };
        String[] sMoonNames = {"Titan", "Rhea", "Iapetus", "Dione"};
        for (int i = 0; i < sMoons.length; i++) {
            Vector3d[] s = getOrbitalState(sMoons[i][0], sMoons[i][1], i_saturn, omega_saturn, Omega_saturn, Math.random() * 2 * Math.PI, mu_saturn);
            s[0].add(saturn.pos);
            s[1].add(saturn.vel);
            moonParents.add(-1);
            planets.add(new Planet(s[0], s[1], sMoonProps[i][0], sMoonProps[i][1], sMoonColors[i], sMoonNames[i]));
        }

        // -------------------- Uranus --------------------
        double a_uranus = 19.18916464 - 0.00196176 * T;
        double e_uranus = 0.04725744 - 0.00004392 * T;
        double i_uranus = Math.toRadians(0.76986);
        double Omega_uranus = Math.toRadians(74.00095922 - 0.15044533 * T);
        double varpi_uranus = Math.toRadians(96.93735127 + 0.29752443 * T);
        double L_uranus = Math.toRadians(313.03693600 + 428.48202785 * T);
        double omega_uranus = varpi_uranus - Omega_uranus;
        double M_uranus = L_uranus - varpi_uranus;
        Vector3d[] state_uranus = getOrbitalState(a_uranus, e_uranus, i_uranus, omega_uranus, Omega_uranus, M_uranus, mu_sun);
        Planet uranus = new Planet(state_uranus[0], state_uranus[1], 8.6810e25, 25362e3, new Vector3f(0.4f, 0.7f, 0.8f), "Uranus");
        uranus.a_au = a_uranus;
        uranus.e = e_uranus;
        uranus.i_rad = i_uranus;
        uranus.omega_rad = omega_uranus;
        uranus.Omega_rad = Omega_uranus;
        uranus.M_rad = M_uranus;
        uranus.mu = mu_sun;
        moonParents.add(18);
        planets.add(uranus);

        // -------------------- Neptune --------------------
        double a_neptune = 30.06992276 + 0.00026291 * T;
        double e_neptune = 0.00859048 + 0.00002938 * T;
        double i_neptune = Math.toRadians(1.76917);
        double Omega_neptune = Math.toRadians(131.79422500 - 0.18605295 * T);
        double varpi_neptune = Math.toRadians(44.96476227 + 0.46750727 * T);
        double L_neptune = Math.toRadians(304.87922929 + 218.45945325 * T);
        double omega_neptune = varpi_neptune - Omega_neptune;
        double M_neptune = L_neptune - varpi_neptune;
        Vector3d[] state_neptune = getOrbitalState(a_neptune, e_neptune, i_neptune, omega_neptune, Omega_neptune, M_neptune, mu_sun);
        Planet neptune = new Planet(state_neptune[0], state_neptune[1], 1.0243e26, 24622e3, new Vector3f(0.3f, 0.5f, 0.8f), "Neptune");
        neptune.a_au = a_neptune;
        neptune.e = e_neptune;
        neptune.i_rad = i_neptune;
        neptune.omega_rad = omega_neptune;
        neptune.Omega_rad = Omega_neptune;
        neptune.M_rad = M_neptune;
        neptune.mu = mu_sun;
        moonParents.add(19);
        planets.add(neptune);

        // Neptune’s moon Triton
        double mu_neptune = G * neptune.mass;
        double a_triton = 354759e3 / AU;
        double e_triton = 0.000016;
        Vector3d[] state_triton = getOrbitalState(a_triton, e_triton, i_neptune, omega_neptune, Omega_neptune, Math.random() * 2 * Math.PI, mu_neptune);
        state_triton[0].add(neptune.pos);
        state_triton[1].add(neptune.vel);
        Planet triton = new Planet(state_triton[0], state_triton[1], 2.139e22, 1353.4e3, new Vector3f(0.7f, 0.8f, 0.8f), "Triton");
        moonParents.add(-1);
        planets.add(triton);

        // -------------------- Pluto --------------------
        double a_pluto = 39.48211675 - 0.00031596 * T;
        double e_pluto = 0.24882730 + 0.00005170 * T;
        double i_pluto = Math.toRadians(17.14001206 + 0.00004818 * T);
        double Omega_pluto = Math.toRadians(110.30393684 - 0.01183482 * T);
        double varpi_pluto = Math.toRadians(224.06891629 - 0.04062942 * T);
        double L_pluto = Math.toRadians(238.92903833 + 145.20780515 * T);
        double omega_pluto = varpi_pluto - Omega_pluto;
        double M_pluto = L_pluto - varpi_pluto;
        Vector3d[] state_pluto = getOrbitalState(a_pluto, e_pluto, i_pluto, omega_pluto, Omega_pluto, M_pluto, mu_sun);
        Planet pluto = new Planet(state_pluto[0], state_pluto[1], 1.303e22, 1188.3e3, new Vector3f(0.7f, 0.8f, 0.9f), "Pluto");
        pluto.a_au = a_pluto;
        pluto.e = e_pluto;
        pluto.i_rad = i_pluto;
        pluto.omega_rad = omega_pluto;
        pluto.Omega_rad = Omega_pluto;
        pluto.M_rad = M_pluto;
        pluto.mu = mu_sun;
        moonParents.add(21);
        planets.add(pluto);

        // -------------------- Pluto's Moon (Charon) --------------------
        double mu_pluto = G * pluto.mass;
        double a_charon = 19571e3 / AU;
        double e_charon = 0.0002;
        Vector3d[] state_charon = getOrbitalState(a_charon, e_charon, i_pluto, omega_pluto, Omega_pluto, Math.random() * 2 * Math.PI, mu_pluto);
        state_charon[0].add(pluto.pos);
        state_charon[1].add(pluto.vel);
        Planet charon = new Planet(state_charon[0], state_charon[1], 1.586e21, 606e3, new Vector3f(0.85f, 0.85f, 0.9f), "Charon");
        moonParents.add(-1);
        planets.add(charon);

        // -------------------- Asteroid Belt --------------------
        int asteroidCount = 800;
        double mu_sun_local = mu_sun;
        Random rand = new Random();
        for (int i = 0; i < asteroidCount; i++) {
            double a_ast = 2.0 + rand.nextDouble() * 1.3;
            double e_ast = 0.02 + rand.nextDouble() * 0.12;
            double i_ast = Math.toRadians(rand.nextDouble() * 10);
            double Omega_ast = Math.toRadians(rand.nextDouble() * 360);
            double omega_ast = Math.toRadians(rand.nextDouble() * 360);
            double M_ast = Math.toRadians(rand.nextDouble() * 360);
            Vector3d[] state_ast = getOrbitalState(a_ast, e_ast, i_ast, omega_ast, Omega_ast, M_ast, mu_sun_local);
            double asteroidMass = 1e12 + rand.nextDouble() * 1e14;
            double asteroidRadius = 1.5e5 + rand.nextDouble() * 1.5e5;
            Vector3f asteroidColor = new Vector3f(
                    0.55f + rand.nextFloat() * 0.25f,
                    0.55f + rand.nextFloat() * 0.25f,
                    0.55f + rand.nextFloat() * 0.25f
            );
            Planet asteroid = new Planet(state_ast[0], state_ast[1], asteroidMass, asteroidRadius, asteroidColor, "");
            moonParents.add(-1);
            planets.add(asteroid);
        }

        // Precompute depths and sigmas
        double sunMass = planets.get(0).mass;
        depths = new float[planets.size()];
        sigmas = new float[planets.size()];
        for (int i = 0; i < planets.size(); i++) {
            double massRatio = planets.get(i).mass / sunMass;
            depths[i] = (float) (SUN_DEPTH * Math.pow(massRatio, MASS_POWER));
            sigmas[i] = (float) (SUN_SIGMA * Math.pow(massRatio, MASS_POWER));
        }

        // Compute display radii and allocate trail buffers
        for (Planet p : planets) {
            double sceneRadius = p.radius * VIS_SCALE;
            p.displayRadius = (float) (sceneRadius * DISPLAY_RADIUS_FACTOR);
            int tVAO = glGenVertexArrays();
            int tVBO = glGenBuffers();
            trailVAOs.add(tVAO);
            trailVBOs.add(tVBO);
            trailUpdated.add(true);
            glBindVertexArray(tVAO);
            glBindBuffer(GL_ARRAY_BUFFER, tVBO);
            glBufferData(GL_ARRAY_BUFFER, MAX_TRAIL_POINTS * 3 * Float.BYTES, GL_DYNAMIC_DRAW);
            glVertexAttribPointer(0, 3, GL_FLOAT, false, 3 * Float.BYTES, 0);
            glEnableVertexAttribArray(0);
            glBindVertexArray(0);
        }

        // Precompute n-body orbits for named bodies
        precomputeNBodyOrbits();
    }

    // Java
    private void precomputeNBodyOrbits() {
        int numPoints = 720;
        for (int i = 0; i < planets.size(); i++) {
            Planet p = planets.get(i);
            if (p.name.isEmpty() || moonParents.get(i) == -1) continue; // Skip asteroids

            int parentIndex = moonParents.get(i);
            boolean isMoon = (parentIndex >= 0 && parentIndex < planets.size() && parentIndex != i);

            List<Float> verts = new ArrayList<>();
            if (!isMoon) {
                // Analytic Keplerian orbit for planets
                for (int s = 0; s < numPoints; s++) {
                    double M = 2 * Math.PI * s / numPoints;
                    Vector3d[] state = getOrbitalState(
                            p.a_au, p.e, p.i_rad, p.omega_rad, p.Omega_rad, M, p.mu
                    );
                    Vector3d pos = state[0];
                    verts.add((float) (pos.x * VIS_SCALE));
                    verts.add((float) (pos.y * VIS_SCALE));
                    verts.add((float) (pos.z * VIS_SCALE));
                }
            } else {
                // Keep integration for moons
                double a_orbit = new Vector3d(p.pos).sub(planets.get(parentIndex).pos).length() / 1.495978707e11;
                double mu = G * planets.get(parentIndex).mass;
                double period_years = Math.sqrt(a_orbit * a_orbit * a_orbit * (mu_sun / mu));
                double period_sec = period_years * 3.15576e7;
                double dt = period_sec / numPoints * ORBIT_DT_FACTOR;

                List<Planet> clone = clonePlanets();
                Vector3d fixedParentPos = new Vector3d(clone.get(parentIndex).pos);
                Planet cp = clone.get(i);
                Vector3d relPos = new Vector3d(cp.pos).sub(fixedParentPos);
                verts.add((float) (relPos.x * VIS_SCALE));
                verts.add((float) (relPos.y * VIS_SCALE));
                verts.add((float) (relPos.z * VIS_SCALE));

                Vector3d[] acc = new Vector3d[clone.size()];
                for (int j = 0; j < clone.size(); j++) acc[j] = new Vector3d();
                for (int s = 0; s < numPoints - 1; s++) {
                    computeAccelerations(clone, acc);
                    for (int j = 0; j < clone.size(); j++) {
                        clone.get(j).pos.fma(dt, clone.get(j).vel).fma(0.5 * dt * dt, acc[j]);
                    }
                    Vector3d[] newAcc = new Vector3d[clone.size()];
                    for (int j = 0; j < clone.size(); j++) newAcc[j] = new Vector3d();
                    computeAccelerations(clone, newAcc);
                    for (int j = 0; j < clone.size(); j++) {
                        Planet pClone = clone.get(j);
                        pClone.vel.fma(0.5 * dt, new Vector3d(acc[j]).add(newAcc[j]));
                        acc[j].set(newAcc[j]);
                    }
                    cp = clone.get(i);
                    relPos.set(clone.get(i).pos).sub(fixedParentPos);
                    verts.add((float) (relPos.x * VIS_SCALE));
                    verts.add((float) (relPos.y * VIS_SCALE));
                    verts.add((float) (relPos.z * VIS_SCALE));
                }
            }
            float[] nbodyVerts = new float[verts.size()];
            for (int v = 0; v < verts.size(); v++) nbodyVerts[v] = verts.get(v);

            int nbodyVAO = glGenVertexArrays();
            int nbodyVBO = glGenBuffers();
            glBindVertexArray(nbodyVAO);
            glBindBuffer(GL_ARRAY_BUFFER, nbodyVBO);
            glBufferData(GL_ARRAY_BUFFER, nbodyVerts, GL_STATIC_DRAW);
            glVertexAttribPointer(0, 3, GL_FLOAT, false, 3 * Float.BYTES, 0);
            glEnableVertexAttribArray(0);
            glBindVertexArray(0);

            nbodyOrbitVAOs.add(nbodyVAO);
            nbodyOrbitVBOs.add(nbodyVBO);
            nbodyOrbitCounts.add(numPoints);
            nbodyOrbitColors.add(new Vector4f(p.color.x * 0.6f, p.color.y * 0.6f, p.color.z * 0.6f, 0.7f));
        }
    }


    // Add new method for computing accelerations
    private void computeAccelerations(List<Planet> ps, Vector3d[] acc) {
        int n = ps.size();
        for (int i = 0; i < n; i++) acc[i].set(0, 0, 0);
        for (int i = 0; i < n; i++) {
            Planet a = ps.get(i);
            boolean isMoon = moonParents.get(i) == -1 && i > 0;
            for (int j = 0; j < n; j++) {
                if (i == j) continue;
                Planet b = ps.get(j);
                if (isMoon && j != moonParents.get(i) && j != 0) {
                    Vector3d dirCheck = new Vector3d(b.pos).sub(a.pos);
                    double distCheck = dirCheck.length();
                    if (distCheck > 1e10) continue;
                }
                Vector3d dir = new Vector3d(b.pos).sub(a.pos);
                double dist = dir.length();
                if (dist < 1.0) continue;
                double invr2 = 1.0 / (dist * dist);
                double aScalar = G * b.mass * invr2;
                dir.normalize();
                acc[i].fma(aScalar, dir);
            }
        }
    }

    private List<Planet> clonePlanets() {
        List<Planet> clone = new ArrayList<>();
        for (Planet p : planets) {
            Planet cp = new Planet(new Vector3d(p.pos), new Vector3d(p.vel), p.mass, p.radius, new Vector3f(p.color), p.name);
            cp.displayRadius = p.displayRadius;
            cp.trail.addAll(p.trail); // Preserve trails
            clone.add(cp);
        }
        return clone;
    }

    // Java
    private Vector3d[] getOrbitalState(double a_au, double e, double i_rad, double omega_rad, double Omega_rad, double M_rad, double mu) {
        double a = a_au * 1.495978707e11;
        double M = M_rad;
        double E = M;
        double tol = 1e-10;
        int maxIter = 50;
        for (int iter = 0; iter < maxIter; iter++) {
            double dE = (E - e * Math.sin(E) - M) / (1 - e * Math.cos(E));
            E -= dE;
            if (Math.abs(dE) < tol) break;
        }
        double sqrt_term = Math.sqrt((1 + e) / (1 - e));
        double tan_E2 = Math.tan(E / 2);
        double f = 2 * Math.atan(sqrt_term * tan_E2);

        double cosf = Math.cos(f);
        double sinf = Math.sin(f);
        double cosE = Math.cos(E);
        double r = a * (1 - e * cosE);
        double p = a * (1 - e * e);
        double h = Math.sqrt(mu * p);

        Vector3d r_peri = new Vector3d(r * cosf, 0, r * sinf);
        Vector3d v_peri = new Vector3d(-sinf, 0, e + cosf).mul(mu / h);

        Matrix3d rot = new Matrix3d()
                .rotateY(Omega_rad)
                .rotateX(i_rad)
                .rotateY(omega_rad);

        Vector3d pos = new Vector3d();
        rot.transform(r_peri, pos);
        Vector3d vel = new Vector3d();
        rot.transform(v_peri, vel);

        return new Vector3d[]{pos, vel};
    }

    private void loop() {
        long lastNanos = System.nanoTime();
        while (!glfwWindowShouldClose(window)) {
            long now = System.nanoTime();
            double realSeconds = (now - lastNanos) / 1e9;
            lastNanos = now;

            handleInput(realSeconds);

            if (!paused) {
                double simSeconds = realSeconds * SIM_SECONDS_PER_REAL_SECOND;
                double step = 300.0;
                while (simSeconds > 0) {
                    double dt = Math.min(step, simSeconds);
                    physicsStep(planets, dt);
                    simSeconds -= dt;
                }
                updateTrails();
            }

            renderFrame();

            glfwSwapBuffers(window);
            glfwPollEvents();
        }
    }

    private void cleanup() {
        glDeleteProgram(shaderProgram);
        glDeleteVertexArrays(sphereVAO);
        glDeleteBuffers(sphereVBO);
        glDeleteVertexArrays(gridVAO);
        glDeleteBuffers(gridVBO);
        for (int vao : trailVAOs) glDeleteVertexArrays(vao);
        for (int vbo : trailVBOs) glDeleteBuffers(vbo);
        for (int vao : nbodyOrbitVAOs) glDeleteVertexArrays(vao);
        for (int vbo : nbodyOrbitVBOs) glDeleteBuffers(vbo);
        glDeleteProgram(textShaderProgram);
        glDeleteVertexArrays(textVAO);
        glDeleteBuffers(textVBO);
        glDeleteTextures(fontTexture);
        cdata.free();
        glfwDestroyWindow(window);
        glfwTerminate();
    }

    private void physicsStep(List<Planet> ps, double dt) {
        int n = ps.size();
        Vector3d[] acc = new Vector3d[n];
        for (int i = 0; i < n; i++) acc[i] = new Vector3d(0, 0, 0);

        for (int i = 0; i < n; i++) {
            Planet a = ps.get(i);
            boolean isMoon = moonParents.get(i) == -1 && i > 0;
            for (int j = 0; j < n; j++) {
                if (i == j) continue;
                Planet b = ps.get(j);
                if (isMoon && j != moonParents.get(i) && j != 0) {
                    Vector3d dirCheck = new Vector3d(b.pos).sub(a.pos);
                    double distCheck = dirCheck.length();
                    if (distCheck > 1e10) continue;
                }
                Vector3d dir = new Vector3d(b.pos).sub(a.pos);
                double dist = dir.length();
                if (dist < 1.0) continue;
                double invr2 = 1.0 / (dist * dist);
                double aScalar = G * b.mass * invr2;
                dir.normalize();
                acc[i].fma(aScalar, dir);
            }
        }
        for (int i = 0; i < n; i++) {
            Planet p = ps.get(i);
            p.vel.add(new Vector3d(acc[i]).mul(dt));
            p.pos.add(new Vector3d(p.vel).mul(dt));
        }
    }

    private void updateTrails() {
        for (int i = 0; i < planets.size(); i++) {
            Planet p = planets.get(i);
            if (p.name.isEmpty()) continue;
            Vector3f newPos = new Vector3f((float) (p.pos.x * VIS_SCALE), (float) (p.pos.y * VIS_SCALE), (float) (p.pos.z * VIS_SCALE));
            boolean changed = false;
            if (p.trail.isEmpty() || p.trail.getFirst().distance(newPos) > TRAIL_POINT_MIN_DISTANCE) {
                p.trail.addFirst(newPos);
                changed = true;
            }
            if (p.trail.size() > MAX_TRAIL_POINTS) {
                p.trail.removeLast();
                changed = true;
            }
            if (changed) {
                trailUpdated.set(i, true);
            }
            if (trailUpdated.get(i)) {
                int vbo = trailVBOs.get(i);
                glBindBuffer(GL_ARRAY_BUFFER, vbo);
                FloatBuffer fb = memAllocFloat(p.trail.size() * 3);
                for (Vector3f v : p.trail) fb.put(v.x).put(v.y).put(v.z);
                fb.flip();
                glBufferSubData(GL_ARRAY_BUFFER, 0, fb);
                memFree(fb);
                glBindBuffer(GL_ARRAY_BUFFER, 0);
                trailUpdated.set(i, false);
            }
        }
    }

    private void renderFrame() {
        glViewport(0, 0, width, height);
        glClearColor(0.03f, 0.06f, 0.12f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glUseProgram(shaderProgram);

        Matrix4f proj = new Matrix4f().perspective((float) Math.toRadians(fov), (float) width / height, 0.1f, 1e8f);
        Matrix4f view = new Matrix4f().lookAt(camPos, new Vector3f(camPos).add(camFront), camUp);
        int projLoc = glGetUniformLocation(shaderProgram, "projection");
        int viewLoc = glGetUniformLocation(shaderProgram, "view");
        int modelLoc = glGetUniformLocation(shaderProgram, "model");
        int colorLoc = glGetUniformLocation(shaderProgram, "color");
        int colorWithAlphaLoc = glGetUniformLocation(shaderProgram, "colorWithAlpha");
        FloatBuffer fb = BufferUtils.createFloatBuffer(16);
        glUniformMatrix4fv(projLoc, false, proj.get(fb));
        glUniformMatrix4fv(viewLoc, false, view.get(fb));

        // Prepare planet XZ positions for GPU
        float[] pxz = new float[planets.size() * 2];
        for (int i = 0; i < planets.size(); i++) {
            pxz[i * 2] = (float) (planets.get(i).pos.x * VIS_SCALE);
            pxz[i * 2 + 1] = (float) (planets.get(i).pos.z * VIS_SCALE);
        }
        glUniform2fv(planetXZLoc, pxz);
        glUniform1fv(planetDepthsLoc, depths);
        glUniform1fv(planetSigmasLoc, sigmas);
        glUniform1i(numPlanetsLoc, planets.size());

        // Draw grid lines (semi-transparent) with dents
        //glUniform1i(useDentLoc, 1);
        //glBindVertexArray(gridVAO);
        //glUniformMatrix4fv(modelLoc, false, new Matrix4f().identity().get(fb));
        //glUniform3f(colorLoc, 0.0f, 0.0f, 0.0f);
        //glUniform4f(colorWithAlphaLoc, 0.9f, 0.9f, 0.9f, 0.1f);
        //glDrawArrays(GL_LINES, 0, gridVertexCount);
        //glBindVertexArray(0);

        // Disable dents for other draws
        glUniform1i(useDentLoc, 0);

        // Draw planets (fully opaque)
        glBindVertexArray(sphereVAO);
        for (Planet p : planets) {
            double px = p.pos.x * VIS_SCALE;
            double py = p.pos.y * VIS_SCALE;
            double pz = p.pos.z * VIS_SCALE;
            Matrix4f model = new Matrix4f().translate((float) px, (float) py, (float) pz)
                    .scale(p.displayRadius);
            glUniformMatrix4fv(modelLoc, false, model.get(fb));
            glUniform3f(colorLoc, p.color.x, p.color.y, p.color.z);
            glUniform4f(colorWithAlphaLoc, 0.0f, 0.0f, 0.0f, 0.0f);
            glDrawArrays(GL_TRIANGLES, 0, sphereVertexCount);
        }
        glBindVertexArray(0);

        // Draw n-body orbits (semi-transparent)
        glLineWidth(1.0f);
        glDepthMask(false); // Disable depth write for transparency
        for (int i = 0; i < nbodyOrbitVAOs.size(); i++) {
            Planet p = planets.get(i);
            if (p.name.isEmpty()) continue; // Skip asteroids
            int parentIndex = moonParents.get(i);
            Matrix4f model = new Matrix4f().identity();
            if (parentIndex >= 0 && parentIndex != i) {
                // Translate moon orbits to follow parent
                Planet parent = planets.get(parentIndex);
                model.translate((float) (parent.pos.x * VIS_SCALE),
                        (float) (parent.pos.y * VIS_SCALE),
                        (float) (parent.pos.z * VIS_SCALE));
            }
            glUniformMatrix4fv(modelLoc, false, model.get(fb));
            Vector4f col = nbodyOrbitColors.get(i);
            glUniform4f(colorWithAlphaLoc, col.x, col.y, col.z, col.w);
            glUniform3f(colorLoc, 0.0f, 0.0f, 0.0f);
            glBindVertexArray(nbodyOrbitVAOs.get(i));
            glDrawArrays(GL_LINE_STRIP, 0, nbodyOrbitCounts.get(i));
        }
        glDepthMask(true);
        glBindVertexArray(0);

        // Draw trails (fully opaque)
        glLineWidth(2.0f);
        for (int i = 0; i < planets.size(); i++) {
            Planet p = planets.get(i);
            if (p.name.isEmpty()) continue;
            int vao = trailVAOs.get(i);
            glBindVertexArray(vao);
            Matrix4f model = new Matrix4f().identity();
            glUniformMatrix4fv(modelLoc, false, model.get(fb));
            glUniform3f(colorLoc, p.color.x, p.color.y, p.color.z);
            glUniform4f(colorWithAlphaLoc, 0.0f, 0.0f, 0.0f, 0.0f);
            glDrawArrays(GL_LINE_STRIP, 0, p.trail.size());
        }
        glBindVertexArray(0);

        // Render labels
        glUseProgram(textShaderProgram);
        glBindVertexArray(textVAO);
        glBindTexture(GL_TEXTURE_2D, fontTexture);
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        glDisable(GL_DEPTH_TEST);

        Matrix4f projT = new Matrix4f().ortho(0, width, height, 0, -1, 1);
        int projLocT = glGetUniformLocation(textShaderProgram, "projection");
        int colorLocT = glGetUniformLocation(textShaderProgram, "textColor");
        FloatBuffer fbT = BufferUtils.createFloatBuffer(16);
        glUniformMatrix4fv(projLocT, false, projT.get(fbT));
        glUniform3f(colorLocT, 1.0f, 1.0f, 1.0f);

        Matrix4f viewT = new Matrix4f().lookAt(camPos, new Vector3f(camPos).add(camFront), camUp);
        Matrix4f proj3D = new Matrix4f().perspective((float) Math.toRadians(fov), (float) width / height, 0.1f, 1e8f);
        Matrix4f vp = proj3D.mul(viewT, new Matrix4f());

        FloatBuffer quadBuf = BufferUtils.createFloatBuffer(4 * 4);
        STBTTAlignedQuad q = STBTTAlignedQuad.create();
        FloatBuffer xpos = BufferUtils.createFloatBuffer(1);
        FloatBuffer ypos = BufferUtils.createFloatBuffer(1);

        final float MOON_LABEL_DISTANCE = 150.0f;

        for (int i = 0; i < planets.size(); i++) {
            Planet p = planets.get(i);
            if (p.name.isEmpty()) continue;
            Vector3f planetPos = new Vector3f((float) (p.pos.x * VIS_SCALE), (float) (p.pos.y * VIS_SCALE), (float) (p.pos.z * VIS_SCALE));
            boolean isMoon = moonParents.get(i) == -1;
            if (isMoon) {
                float distance = camPos.distance(planetPos);
                if (distance > MOON_LABEL_DISTANCE) continue;
            }
            Vector4f pos = new Vector4f(planetPos.x, planetPos.y + p.displayRadius * 1.5f, planetPos.z, 1.0f);
            vp.transform(pos);
            if (pos.z <= 0) continue;
            pos.div(pos.w);
            float screenX = (pos.x * 0.5f + 0.5f) * width;
            float screenY = (1.0f - (pos.y * 0.5f + 0.5f)) * height - LABEL_OFFSET;
            if (screenX < 0 || screenX > width || screenY < 0 || screenY > height) continue;
            xpos.put(0, screenX);
            ypos.put(0, screenY);
            for (char c : p.name.toCharArray()) {
                if (c < 32 || c >= 127) continue;
                stbtt_GetPackedQuad(cdata, BITMAP_W, BITMAP_H, c - 32, xpos, ypos, q, false);
                quadBuf.clear();
                quadBuf.put(q.x0()).put(q.y0()).put(q.s0()).put(q.t0());
                quadBuf.put(q.x1()).put(q.y0()).put(q.s1()).put(q.t0());
                quadBuf.put(q.x0()).put(q.y1()).put(q.s0()).put(q.t1());
                quadBuf.put(q.x1()).put(q.y1()).put(q.s1()).put(q.t1());
                quadBuf.flip();
                glBindBuffer(GL_ARRAY_BUFFER, textVBO);
                glBufferSubData(GL_ARRAY_BUFFER, 0, quadBuf);
                glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
            }
        }
        glBindTexture(GL_TEXTURE_2D, 0);
        glBindVertexArray(0);
        glEnable(GL_DEPTH_TEST);
    }

    private void createGridLines(int divisions, double halfSizeScene) {
        int div = Math.max(4, divisions);
        double half = halfSizeScene;
        double step = (2.0 * half) / div;
        List<Float> verts = new ArrayList<>();
        for (int iz = 0; iz <= div; iz++) {
            double z = -half + iz * step;
            for (int ix = 0; ix < div; ix++) {
                double x0 = -half + ix * step;
                double x1 = x0 + step;
                verts.add((float) x0); verts.add(0f); verts.add((float) z);
                verts.add((float) x1); verts.add(0f); verts.add((float) z);
            }
        }
        for (int ix = 0; ix <= div; ix++) {
            double x = -half + ix * step;
            for (int iz = 0; iz < div; iz++) {
                double z0 = -half + iz * step;
                double z1 = z0 + step;
                verts.add((float) x); verts.add(0f); verts.add((float) z0);
                verts.add((float) x); verts.add(0f); verts.add((float) z1);
            }
        }
        gridVertexCount = verts.size() / 3;
        FloatBuffer fb = BufferUtils.createFloatBuffer(verts.size());
        for (Float f : verts) fb.put(f);
        fb.flip();
        gridVAO = glGenVertexArrays();
        gridVBO = glGenBuffers();
        glBindVertexArray(gridVAO);
        glBindBuffer(GL_ARRAY_BUFFER, gridVBO);
        glBufferData(GL_ARRAY_BUFFER, fb, GL_STATIC_DRAW);
        glVertexAttribPointer(0, 3, GL_FLOAT, false, 3 * Float.BYTES, 0);
        glEnableVertexAttribArray(0);
        glBindVertexArray(0);
    }

    private void createSphereMesh(int stacks, int sectors) {
        List<Float> verts = new ArrayList<>();
        for (int i = 0; i <= stacks; i++) {
            double theta = Math.PI * i / stacks;
            for (int j = 0; j <= sectors; j++) {
                double phi = 2.0 * Math.PI * j / sectors;
                float x = (float) (Math.sin(theta) * Math.cos(phi));
                float y = (float) Math.cos(theta);
                float z = (float) (Math.sin(theta) * Math.sin(phi));
                verts.add(x); verts.add(y); verts.add(z);
            }
        }
        List<Integer> idx = new ArrayList<>();
        for (int i = 0; i < stacks; i++) {
            for (int j = 0; j < sectors; j++) {
                int first = i * (sectors + 1) + j;
                int second = first + sectors + 1;
                idx.add(first); idx.add(second); idx.add(first + 1);
                idx.add(second); idx.add(second + 1); idx.add(first + 1);
            }
        }
        sphereVertexCount = idx.size();
        FloatBuffer buf = BufferUtils.createFloatBuffer(sphereVertexCount * 3);
        for (int id : idx) {
            int base = id * 3;
            buf.put(verts.get(base)).put(verts.get(base + 1)).put(verts.get(base + 2));
        }
        buf.flip();
        sphereVAO = glGenVertexArrays();
        sphereVBO = glGenBuffers();
        glBindVertexArray(sphereVAO);
        glBindBuffer(GL_ARRAY_BUFFER, sphereVBO);
        glBufferData(GL_ARRAY_BUFFER, buf, GL_STATIC_DRAW);
        glVertexAttribPointer(0, 3, GL_FLOAT, false, 3 * Float.BYTES, 0);
        glEnableVertexAttribArray(0);
        glBindVertexArray(0);
    }

    private int buildShaders() {
        int vs = glCreateShader(GL_VERTEX_SHADER);
        String vert = "#version 330 core\n" +
                "layout(location=0) in vec3 aPos;\n" +
                "uniform mat4 model;\n" +
                "uniform mat4 view;\n" +
                "uniform mat4 projection;\n" +
                "uniform bool useDent;\n" +
                "uniform vec2 planetXZ[32];\n" +
                "uniform float planetDepths[32];\n" +
                "uniform float planetSigmas[32];\n" +
                "uniform int numPlanets;\n" +
                "out vec3 vNormal;\n" +
                "out vec3 vPos;\n" +
                "void main(){\n" +
                "    vec3 modifiedPos = aPos;\n" +
                "    if(useDent){\n" +
                "        float dent = 0.0;\n" +
                "        for(int i=0; i<numPlanets; i++){\n" +
                "            vec2 dxz = modifiedPos.xz - planetXZ[i];\n" +
                "            float r2 = dot(dxz, dxz);\n" +
                "            dent -= planetDepths[i] * exp( -r2 / (2.0 * planetSigmas[i] * planetSigmas[i]) );\n" +
                "        }\n" +
                "        modifiedPos.y += dent;\n" +
                "    }\n" +
                "    vPos = vec3(model * vec4(modifiedPos,1.0));\n" +
                "    vNormal = mat3(transpose(inverse(model))) * aPos;\n" +
                "    gl_Position = projection * view * vec4(vPos,1.0);\n" +
                "}";
        glShaderSource(vs, vert);
        glCompileShader(vs);
        if (glGetShaderi(vs, GL_COMPILE_STATUS) == GL_FALSE) throw new RuntimeException("VS: " + glGetShaderInfoLog(vs));

        int fs = glCreateShader(GL_FRAGMENT_SHADER);
        String frag = "#version 330 core\n" +
                "in vec3 vNormal; in vec3 vPos; out vec4 FragColor;\n" +
                "uniform vec3 color; uniform vec4 colorWithAlpha;\n" +
                "void main() {\n" +
                "    vec3 light = normalize(vec3(0.3, 1.0, 0.2));\n" +
                "    float d = max(dot(normalize(vNormal), light), 0.1);\n" +
                "    vec3 col = color * d + color * 0.2;\n" +
                "    float alpha = colorWithAlpha.a > 0.0 ? colorWithAlpha.a : 1.0;\n" +
                "    vec3 finalColor = colorWithAlpha.a > 0.0 ? colorWithAlpha.rgb : col;\n" +
                "    FragColor = vec4(finalColor, alpha);\n" +
                "}";
        glShaderSource(fs, frag);
        glCompileShader(fs);
        if (glGetShaderi(fs, GL_COMPILE_STATUS) == GL_FALSE) throw new RuntimeException("FS: " + glGetShaderInfoLog(fs));

        int prog = glCreateProgram();
        glAttachShader(prog, vs); glAttachShader(prog, fs);
        glLinkProgram(prog);
        if (glGetProgrami(prog, GL_LINK_STATUS) == GL_FALSE) throw new RuntimeException("Link: " + glGetProgramInfoLog(prog));
        glDeleteShader(vs); glDeleteShader(fs);
        return prog;
    }

    private void initFont() {
        fontInfo = STBTTFontinfo.create();
        ByteBuffer ttf = loadFontFile();
        if (!stbtt_InitFont(fontInfo, ttf)) throw new RuntimeException("Failed to init font");
        ByteBuffer bitmap = memAlloc(BITMAP_W * BITMAP_H);
        STBTTPackContext pc = STBTTPackContext.create();
        cdata = STBTTPackedchar.malloc(95);
        stbtt_PackBegin(pc, bitmap, BITMAP_W, BITMAP_H, 0, 1);
        stbtt_PackSetOversampling(pc, 2, 2);
        stbtt_PackFontRange(pc, ttf, 0, FONT_SIZE, 32, cdata);
        stbtt_PackEnd(pc);
        fontTexture = glGenTextures();
        glBindTexture(GL_TEXTURE_2D, fontTexture);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RED, BITMAP_W, BITMAP_H, 0, GL_RED, GL_UNSIGNED_BYTE, bitmap);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glBindTexture(GL_TEXTURE_2D, 0);
        memFree(bitmap);
        memFree(ttf);
    }

    private ByteBuffer loadFontFile() {
        try (java.io.InputStream is = Main.class.getResourceAsStream("/arial.ttf")) {
            if (is == null) throw new RuntimeException("Font file not found");
            byte[] bytes = is.readAllBytes();
            ByteBuffer buffer = memAlloc(bytes.length);
            buffer.put(bytes).flip();
            return buffer;
        } catch (Exception e) {
            throw new RuntimeException("Failed to load font: " + e.getMessage());
        }
    }

    private void createTextQuad() {
        textVAO = glGenVertexArrays();
        textVBO = glGenBuffers();
        glBindVertexArray(textVAO);
        glBindBuffer(GL_ARRAY_BUFFER, textVBO);
        glBufferData(GL_ARRAY_BUFFER, 4 * 4 * Float.BYTES, GL_DYNAMIC_DRAW);
        glVertexAttribPointer(0, 2, GL_FLOAT, false, 4 * Float.BYTES, 0);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(1, 2, GL_FLOAT, false, 4 * Float.BYTES, 2 * Float.BYTES);
        glEnableVertexAttribArray(1);
        glBindVertexArray(0);
    }

    private int buildTextShaders() {
        int vs = glCreateShader(GL_VERTEX_SHADER);
        String vert = "#version 330 core\n" +
                "layout(location=0) in vec2 aPos;\n" +
                "layout(location=1) in vec2 aTexCoord;\n" +
                "out vec2 TexCoord;\n" +
                "uniform mat4 projection;\n" +
                "void main() {\n" +
                "    gl_Position = projection * vec4(aPos, 0.0, 1.0);\n" +
                "    TexCoord = aTexCoord;\n" +
                "}";
        glShaderSource(vs, vert);
        glCompileShader(vs);
        if (glGetShaderi(vs, GL_COMPILE_STATUS) == GL_FALSE) throw new RuntimeException("Text VS: " + glGetShaderInfoLog(vs));

        int fs = glCreateShader(GL_FRAGMENT_SHADER);
        String frag = "#version 330 core\n" +
                "in vec2 TexCoord;\n" +
                "out vec4 FragColor;\n" +
                "uniform sampler2D fontTexture;\n" +
                "uniform vec3 textColor;\n" +
                "void main() {\n" +
                "    float alpha = texture(fontTexture, TexCoord).r;\n" +
                "    FragColor = vec4(textColor, alpha);\n" +
                "}";
        glShaderSource(fs, frag);
        glCompileShader(fs);
        if (glGetShaderi(fs, GL_COMPILE_STATUS) == GL_FALSE) throw new RuntimeException("Text FS: " + glGetShaderInfoLog(fs));

        int prog = glCreateProgram();
        glAttachShader(prog, vs);
        glAttachShader(prog, fs);
        glLinkProgram(prog);
        if (glGetProgrami(prog, GL_LINK_STATUS) == GL_FALSE) throw new RuntimeException("Text Link: " + glGetProgramInfoLog(prog));
        glDeleteShader(vs);
        glDeleteShader(fs);
        return prog;
    }

    private void handleInput(double realSeconds) {
        float baseSpeed = (float) (2500.0 * realSeconds);
        float speed = baseSpeed;
        if (glfwGetKey(window, GLFW_KEY_LEFT_ALT) == GLFW_PRESS || glfwGetKey(window, GLFW_KEY_RIGHT_ALT) == GLFW_PRESS) {
            speed *= 0.1f;
        } else if (glfwGetKey(window, GLFW_KEY_LEFT_CONTROL) == GLFW_PRESS || glfwGetKey(window, GLFW_KEY_RIGHT_CONTROL) == GLFW_PRESS) {
            speed *= 5.0f;
        }
        if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS) camPos.add(new Vector3f(camFront).mul(speed));
        if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS) camPos.sub(new Vector3f(camFront).mul(speed));
        if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS) camPos.sub(new Vector3f(camFront).cross(camUp, new Vector3f()).normalize().mul(speed));
        if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS) camPos.add(new Vector3f(camFront).cross(camUp, new Vector3f()).normalize().mul(speed));
        if (glfwGetKey(window, GLFW_KEY_SPACE) == GLFW_PRESS) camPos.add(new Vector3f(camUp).mul(speed));
        if (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS) camPos.sub(new Vector3f(camUp).mul(speed));
        if (glfwGetKey(window, GLFW_KEY_P) == GLFW_PRESS) {
            paused = !paused;
            try {
                Thread.sleep(150);
            } catch (Exception ignored) {
            }
        }
        if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS) glfwSetWindowShouldClose(window, true);
    }

    private void mouseMove(long win, double xpos, double ypos) {
        if (firstMouse) {
            lastX = (float) xpos;
            lastY = (float) ypos;
            firstMouse = false;
        }
        float xoff = (float) (xpos - lastX), yoff = (float) (lastY - ypos);
        lastX = (float) xpos;
        lastY = (float) ypos;
        float sens = 0.12f;
        xoff *= sens;
        yoff *= sens;
        yaw += xoff;
        pitch += yoff;
        if (pitch > 89f) pitch = 89f;
        if (pitch < -89f) pitch = -89f;
        Vector3f f = new Vector3f();
        f.x = (float) (Math.cos(Math.toRadians(yaw)) * Math.cos(Math.toRadians(pitch)));
        f.y = (float) Math.sin(Math.toRadians(pitch));
        f.z = (float) (Math.sin(Math.toRadians(yaw)) * Math.cos(Math.toRadians(pitch)));
        camFront = f.normalize();
    }

    private void scroll(long w, double xoff, double yoff) {
        fov -= (float) yoff;
        if (fov < 12f) fov = 12f;
        if (fov > 80f) fov = 80f;
    }
}