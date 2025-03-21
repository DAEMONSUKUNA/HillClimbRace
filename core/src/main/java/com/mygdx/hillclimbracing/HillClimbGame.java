package com.mygdx.hillclimbracing;

import com.badlogic.gdx.ApplicationAdapter;
import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.Input;
import com.badlogic.gdx.graphics.GL20;
import com.badlogic.gdx.graphics.OrthographicCamera;
import com.badlogic.gdx.graphics.Texture;
import com.badlogic.gdx.graphics.g2d.SpriteBatch;
import com.badlogic.gdx.math.MathUtils;
import com.badlogic.gdx.math.Rectangle;
import com.badlogic.gdx.math.Vector2;
import com.badlogic.gdx.math.Vector3;
import com.badlogic.gdx.physics.box2d.*;
import com.badlogic.gdx.physics.box2d.joints.RevoluteJoint;
import com.badlogic.gdx.physics.box2d.joints.RevoluteJointDef;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.viewport.FitViewport;
import com.badlogic.gdx.utils.viewport.Viewport;

public class HillClimbGame extends ApplicationAdapter {
    private SpriteBatch batch;
    private Texture carTexture, wheelTexture, terrainTexture, respawnButtonTexture;
    private World world;
    private Box2DDebugRenderer debugRenderer;
    private Body carBody, groundBody;
    private Body leftWheelBody, rightWheelBody;
    private RevoluteJoint leftWheelJoint, rightWheelJoint;
    private static final float PPM = 24f; // Pixels per meter


    // Respawn related variables
    private boolean isCarFlipped = false;
    private final float FLIP_THRESHOLD = 0.5f; // Radians - about 30 degrees past horizontal
    private Rectangle respawnButtonRect;

    // Terrain rendering
    private Array<Vector2> terrainVerts;

    // Camera and viewport for better visualization
    private OrthographicCamera camera;
    private Viewport viewport;
    private static final float WORLD_WIDTH = 800;
    private static final float WORLD_HEIGHT = 480;

    // Engine properties
    private float enginePower = 300f;

    // Car dimensions
    private final float CAR_WIDTH = 5.0f;
    private final float CAR_HEIGHT = 4f;
    private final float WHEEL_RADIUS = 0.6f;
    private final float WHEEL_X_OFFSET = 1.5f;
    private final float WHEEL_Y_OFFSET = -.7f;

    // Spawn and initial terrain parameters
    private final float FLAT_START_LENGTH = 20.0f;
    private final float CAR_SPAWN_X = 5.0f;

    // Color constants
    private final float SKY_R = 0.4f;
    private final float SKY_G = 0.6f;
    private final float SKY_B = 1.0f;

    private final float TERRAIN_R = 0.3f;
    private final float TERRAIN_G = 0.8f;
    private final float TERRAIN_B = 0.3f;

    private final float CAR_R = 0.8f;
    private final float CAR_G = 0.2f;
    private final float CAR_B = 0.2f;

    private final float WHEEL_R = 0.3f;
    private final float WHEEL_G = 0.3f;
    private final float WHEEL_B = 0.3f;

    private final float RESPAWN_R = 1.0f;
    private final float RESPAWN_G = 0.3f;
    private final float RESPAWN_B = 0.3f;

    // Debug flag
    private boolean showDebug = true;

    @Override
    public void create() {
    batch = new SpriteBatch();

    // Load actual PNG files instead of placeholders
    carTexture = new Texture(Gdx.files.internal("car.png"));
    wheelTexture = new Texture(Gdx.files.internal("wheel.png"));
    terrainTexture = new Texture(Gdx.files.internal("terrain.png"));
    respawnButtonTexture = new Texture(Gdx.files.internal("respawn_button.png"));

    // Initialize button rectangle (for touch/click detection)
    respawnButtonRect = new Rectangle(20, 20, 80, 80);

    // Rest of your code remains the same...


        // Initialize button rectangle (for touch/click detection)
        respawnButtonRect = new Rectangle(20, 20, 80, 80);

        // Setup camera and viewport
        camera = new OrthographicCamera();
        viewport = new FitViewport(WORLD_WIDTH / PPM, WORLD_HEIGHT / PPM, camera);
        viewport.apply();
        camera.position.set(viewport.getWorldWidth() / 2, viewport.getWorldHeight() / 2, 0);

        // Initialize Box2D world
        world = new World(new Vector2(0, -12f), true);
        debugRenderer = new Box2DDebugRenderer();

        // First create terrain
        terrainVerts = createTerrain();

        // Then create car at proper spawn position
        createCarWithWheels();

        // Add collision listener
        setupContactListener();
    }

    private Texture createPlaceHolderTexture(int width, int height, float r, float g, float b) {
        com.badlogic.gdx.graphics.Pixmap pixmap = new com.badlogic.gdx.graphics.Pixmap(width, height, com.badlogic.gdx.graphics.Pixmap.Format.RGBA8888);
        pixmap.setColor(r, g, b, 1);
        pixmap.fill();
        Texture texture = new Texture(pixmap);
        pixmap.dispose();
        texture.setFilter(Texture.TextureFilter.Linear, Texture.TextureFilter.Linear);
        return texture;
    }

    private void setupContactListener() {
        world.setContactListener(new ContactListener() {
            @Override
            public void beginContact(Contact contact) {
                // Handle collision events if needed
            }

            @Override
            public void endContact(Contact contact) {
                // Handle end of collision events
            }

            @Override
            public void preSolve(Contact contact, Manifold oldManifold) {
                // Adjust collision properties before solving
            }

            @Override
            public void postSolve(Contact contact, ContactImpulse impulse) {
                // React to collision impulses
            }
        });
    }

    private Array<Vector2> createTerrain() {
        BodyDef groundDef = new BodyDef();
        groundDef.position.set(0, 0);
        groundBody = world.createBody(groundDef);

        // Create terrain with smoother hills and flat starting area
        ChainShape groundShape = new ChainShape();

        // Generate terrain vertices with smoother profile and flat start
        Vector2[] vertices = generateTerrain(200, 6000, 50);
        groundShape.createChain(vertices);

        FixtureDef fixtureDef = new FixtureDef();
        fixtureDef.shape = groundShape;
        fixtureDef.friction = 1.0f;
        fixtureDef.restitution = 0.1f;

        groundBody.createFixture(fixtureDef);
        groundShape.dispose();

        // Store terrain vertices for rendering
        Array<Vector2> terrainVertices = new Array<>(vertices.length);
        for (Vector2 vertex : vertices) {
            terrainVertices.add(new Vector2(vertex));
        }

        return terrainVertices;
    }

    private Vector2[] generateTerrain(int points, float width, float baseHeight) {
        Vector2[] vertices = new Vector2[points];

        float segmentWidth = width / (points - 1);
        float lastHeight = baseHeight / PPM;

        // Calculate the number of vertices needed for flat starting area
        int flatStartVertices = (int)(FLAT_START_LENGTH / (segmentWidth / PPM));

        for (int i = 0; i < points; i++) {
            float x = i * segmentWidth / PPM;
            float height;

            // Create a completely flat starting section
            if (i < flatStartVertices) {
                height = baseHeight / PPM;
            } else {
                // Generate smoother terrain with gentler slopes after the flat area
                height = baseHeight / PPM;

                // Use longer wavelengths and smaller amplitudes for smoother hills
                height += (MathUtils.sin((x - (flatStartVertices * segmentWidth / PPM)) * 0.8f) * 15f) / PPM;
                height += (MathUtils.sin((x - (flatStartVertices * segmentWidth / PPM)) * 0.3f) * 25f) / PPM;
                height += (MathUtils.sin((x - (flatStartVertices * segmentWidth / PPM)) * 2f) * 3f) / PPM;

                // Add small hills that look like classic Hill Climb Racing
                if (x > (flatStartVertices * segmentWidth / PPM) + 5 && MathUtils.random() < 0.08 && i > flatStartVertices) {
                    // Occasionally add a small hill or dip
                    float hillHeight = MathUtils.random(10f, 20f) / PPM;
                    float hillWidth = MathUtils.random(5, 15);

                    // Add a hill shape using a bell curve over the next few vertices
                    for (int j = 0; j < hillWidth && i + j < points; j++) {
                        // Calculate bell curve factor (0 to 1 and back to 0)
                        float factor = (float) Math.sin((j / hillWidth) * Math.PI);

                        // If we're on the current vertex, start the hill
                        if (j == 0) {
                            height += hillHeight * factor;
                        }
                    }
                }

                // Smooth transition from previous height (reduces jaggedness)
                if (i > flatStartVertices) {
                    height = lastHeight * 0.5f + height * 0.5f;
                }

                // Extremely reduced random variation
                height += MathUtils.random(-0.5f, 0.5f) / PPM;
            }

            // Ensure minimum height and save for smoothing
            height = Math.max(height, 10f / PPM);
            lastHeight = height;

            vertices[i] = new Vector2(x, height);
        }

        return vertices;
    }

    private void createCarWithWheels() {
        // Get the height of the terrain at the spawn point
        float spawnY = getTerrainHeightAt(CAR_SPAWN_X) + WHEEL_RADIUS + 0.5f;

        // Car body - Position it at the beginning of the flat area
        BodyDef carDef = new BodyDef();
        carDef.type = BodyDef.BodyType.DynamicBody;
        carDef.position.set(CAR_SPAWN_X, spawnY);
        carBody = world.createBody(carDef);

        // Car chassis shape
        PolygonShape carShape = new PolygonShape();
        carShape.setAsBox(CAR_WIDTH / 3, CAR_HEIGHT / 6);

        FixtureDef carFixture = new FixtureDef();
        carFixture.shape = carShape;
        carFixture.density = 8.0f;
        carFixture.friction = 0.6f;
        carFixture.restitution = 0.1f;
        carBody.createFixture(carFixture);
        carShape.dispose();

        // Create wheels
        createWheel(true); // Left wheel
        createWheel(false); // Right wheel
    }

    // Helper method to get terrain height at a specific x position
    private float getTerrainHeightAt(float x) {
        // Find the two terrain vertices between which x falls
        if (terrainVerts != null && terrainVerts.size > 1) {
            for (int i = 0; i < terrainVerts.size - 1; i++) {
                if (terrainVerts.get(i).x <= x && terrainVerts.get(i + 1).x >= x) {
                    // Interpolate height between these two points
                    Vector2 v1 = terrainVerts.get(i);
                    Vector2 v2 = terrainVerts.get(i + 1);

                    // Calculate interpolation factor
                    float t = (x - v1.x) / (v2.x - v1.x);

                    // Linear interpolation of height
                    return v1.y + t * (v2.y - v1.y);
                }
            }
        }

        // Default height if we couldn't find it
        return 4f;
    }

    private void createWheel(boolean isLeftWheel) {
        // Wheel properties
        float xOffset = isLeftWheel ? -WHEEL_X_OFFSET : WHEEL_X_OFFSET;

        // Wheel body - position relative to car body
        BodyDef wheelDef = new BodyDef();
        wheelDef.type = BodyDef.BodyType.DynamicBody;
        wheelDef.position.set(carBody.getPosition().x + xOffset, carBody.getPosition().y + WHEEL_Y_OFFSET);

        Body wheelBody = world.createBody(wheelDef);

        // Wheel shape
        CircleShape wheelShape = new CircleShape();
        wheelShape.setRadius(WHEEL_RADIUS);

        FixtureDef wheelFixture = new FixtureDef();
        wheelFixture.shape = wheelShape;
        wheelFixture.density = 1.0f;
        wheelFixture.friction = 2.0f;
        wheelFixture.restitution = 0.1f;

        wheelBody.createFixture(wheelFixture);
        wheelShape.dispose();

        // Create revolute joint to connect wheel to car
        RevoluteJointDef jointDef = new RevoluteJointDef();
        jointDef.bodyA = carBody;
        jointDef.bodyB = wheelBody;
        jointDef.localAnchorA.set(xOffset, WHEEL_Y_OFFSET);
        jointDef.localAnchorB.set(0, 0);
        jointDef.enableMotor = true;
        jointDef.maxMotorTorque = 200f;
        jointDef.motorSpeed = 0;

        // Create the joint and cast it to RevoluteJoint
        RevoluteJoint joint = (RevoluteJoint) world.createJoint(jointDef);

        // Store references to wheels and joints
        if (isLeftWheel) {
            leftWheelBody = wheelBody;
            leftWheelJoint = joint;
        } else {
            rightWheelBody = wheelBody;
            rightWheelJoint = joint;
        }
    }

    private void update() {
        float deltaTime = Gdx.graphics.getDeltaTime();

        // Step the physics world
        world.step(deltaTime, 6, 2);

        // Check if car is flipped
        isCarFlipped = checkIfCarFlipped();

        // Car controls
        handleInput();

        // Update camera to follow the car
        updateCamera();

        // Toggle debug rendering with B key
        if (Gdx.input.isKeyJustPressed(Input.Keys.B)) {
            showDebug = !showDebug;
        }

        // Respawn car with R key if flipped
        if (isCarFlipped && Gdx.input.isKeyJustPressed(Input.Keys.R)) {
            respawnCar();
        }

        // Check for touch/click on respawn button
        if (isCarFlipped && Gdx.input.justTouched()) {
            // Convert screen coordinates to UI coordinates
            Vector3 touchPos = new Vector3(Gdx.input.getX(), Gdx.input.getY(), 0);
            // Unproject from screen space to UI space
            camera.unproject(touchPos);

            // Check if button was clicked (in screen coordinates)
            if (respawnButtonRect.contains(touchPos.x, touchPos.y)) {
                respawnCar();
            }
        }
    }

    private boolean checkIfCarFlipped() {
        // Get car's rotation in radians (-π to π)
        float rotation = carBody.getAngle() % (2 * MathUtils.PI);

        // Check if car is significantly rotated upside down
        // We consider "upside down" when the car is rotated more than FLIP_THRESHOLD past horizontal
        return Math.abs(rotation) > MathUtils.PI - FLIP_THRESHOLD;
    }

    private void respawnCar() {
        // Get the height of the terrain at the spawn point
        float spawnY = getTerrainHeightAt(CAR_SPAWN_X) + WHEEL_RADIUS + 0.5f;

        // Reset car position and velocity
        carBody.setTransform(CAR_SPAWN_X, spawnY, 0);
        carBody.setLinearVelocity(0, 0);
        carBody.setAngularVelocity(0);

        // Reset wheel positions
        leftWheelBody.setTransform(CAR_SPAWN_X - WHEEL_X_OFFSET, spawnY + WHEEL_Y_OFFSET, 0);
        leftWheelBody.setLinearVelocity(0, 0);
        leftWheelBody.setAngularVelocity(0);

        rightWheelBody.setTransform(CAR_SPAWN_X + WHEEL_X_OFFSET, spawnY + WHEEL_Y_OFFSET, 0);
        rightWheelBody.setLinearVelocity(0, 0);
        rightWheelBody.setAngularVelocity(0);

        isCarFlipped = false;
    }

    private void handleInput() {
        // Flag to track if any key is being pressed
        boolean keyPressed = false;

        // Forward (D key or RIGHT arrow)
        if (Gdx.input.isKeyPressed(Input.Keys.D) || Gdx.input.isKeyPressed(Input.Keys.RIGHT)) {
            keyPressed = true;
            leftWheelJoint.setMotorSpeed(-enginePower);
            rightWheelJoint.setMotorSpeed(-enginePower);
        }

        // Backward (A key or LEFT arrow)
        if (Gdx.input.isKeyPressed(Input.Keys.A) || Gdx.input.isKeyPressed(Input.Keys.LEFT)) {
            keyPressed = true;
            leftWheelJoint.setMotorSpeed(enginePower);
            rightWheelJoint.setMotorSpeed(enginePower);
        }

        // Hard brake (Space key)
        if (Gdx.input.isKeyPressed(Input.Keys.SPACE)) {
            keyPressed = true;
            leftWheelJoint.setMotorSpeed(0);
            rightWheelJoint.setMotorSpeed(0);
            leftWheelBody.setAngularVelocity(leftWheelBody.getAngularVelocity() * 0.8f);
            rightWheelBody.setAngularVelocity(rightWheelBody.getAngularVelocity() * 0.8f);
        }

        // Reset motor speeds when no key is pressed
        if (!keyPressed) {
            leftWheelJoint.setMotorSpeed(0);
            rightWheelJoint.setMotorSpeed(0);
            // Apply slight damping to gradually stop the wheels
            leftWheelBody.setAngularVelocity(leftWheelBody.getAngularVelocity() * 0.98f);
            rightWheelBody.setAngularVelocity(rightWheelBody.getAngularVelocity() * 0.98f);
        }
    }

    private void updateCamera() {
        // Smoothly follow the car
        Vector2 carPosition = carBody.getPosition();

        // Add a look-ahead in the direction of movement
        float lookAheadX = carBody.getLinearVelocity().x * 0.7f;

        // Smoothly update camera position
        camera.position.x = camera.position.x + (carPosition.x + lookAheadX - camera.position.x) * 0.1f;
        float targetY = carPosition.y + 3f;
        camera.position.y = camera.position.y + (targetY - camera.position.y) * 0.1f;

        // Ensure camera stays above ground
        camera.position.y = Math.max(camera.position.y, 3f);

        // Set a good zoom level
        camera.zoom = 1.2f;

        camera.update();
    }

    @Override
    public void render() {
        update();

        // Clear screen with light blue sky color
        Gdx.gl.glClearColor(SKY_R, SKY_G, SKY_B, 1);
        Gdx.gl.glClear(GL20.GL_COLOR_BUFFER_BIT);

        // Set projection matrix to camera combined
        batch.setProjectionMatrix(camera.combined);

        batch.begin();

        // Draw terrain
        drawTerrain();

        // Draw car chassis
        drawCar();

        // Draw wheels
        drawWheel(batch, leftWheelBody);
        drawWheel(batch, rightWheelBody);

        // Draw respawn button if car is flipped
        if (isCarFlipped) {
            // Draw respawn button at fixed position relative to camera view
            float buttonX = camera.position.x - camera.viewportWidth * camera.zoom / 2 + 1.0f;
            float buttonY = camera.position.y - camera.viewportHeight * camera.zoom / 2 + 1.0f;
            batch.draw(respawnButtonTexture, buttonX, buttonY, 3.0f, 3.0f);

            // Update the button rectangle for touch detection
            respawnButtonRect.set(buttonX, buttonY, 3.0f, 3.0f);
        }

        batch.end();

        // Show Box2D debug information if enabled
        if (showDebug) {
            debugRenderer.render(world, camera.combined);
        }
    }

    private void drawTerrain() {
        // Draw terrain segments
        if (terrainVerts != null && terrainVerts.size > 1) {
            for (int i = 0; i < terrainVerts.size - 1; i++) {
                Vector2 v1 = terrainVerts.get(i);
                Vector2 v2 = terrainVerts.get(i + 1);

                // Calculate segment properties
                float x1 = v1.x;
                float y1 = v1.y;
                float x2 = v2.x;
                float y2 = v2.y;

                // Calculate width and height of the segment
                float width = Vector2.dst(x1, y1, x2, y2);
                float height = 0.3f;

                // Calculate angle
                float angle = MathUtils.atan2(y2 - y1, x2 - x1) * MathUtils.radiansToDegrees;

                // Draw segment as a texture
                batch.draw(terrainTexture,
                    x1, y1 - height/2,
                    0, height/2,
                    width, height,
                    1, 1,
                    angle,
                    0, 0,
                    terrainTexture.getWidth(), terrainTexture.getHeight(),
                    false, false);
            }

            // Fill the area beneath the terrain
            for (int i = 0; i < terrainVerts.size - 1; i++) {
                Vector2 v1 = terrainVerts.get(i);
                Vector2 v2 = terrainVerts.get(i + 1);

                // Calculate lowest y-position visible on screen
                float bottomY = camera.position.y - camera.viewportHeight * camera.zoom / 2 - 1.0f;

                // Draw filled rectangle from terrain down to bottom of screen
                batch.draw(terrainTexture,
                    v1.x, bottomY,
                    v2.x - v1.x, v1.y - bottomY);
            }
        }
    }

    private void drawCar() {
        float carAngle = carBody.getAngle() * MathUtils.radiansToDegrees;

        // Visual height is taller than physics height
        float visualHeight = CAR_HEIGHT * 1.5f; // Make the car 1.5x taller visually
        float heightOffset = (visualHeight - CAR_HEIGHT) / 2; // Center the taller visual

        // Draw car with increased height but same physics position
        batch.draw(carTexture,
                carBody.getPosition().x - CAR_WIDTH/2,
                carBody.getPosition().y - CAR_HEIGHT/2 - heightOffset, // Offset to center the taller visual
                CAR_WIDTH/2, visualHeight/2,
                CAR_WIDTH, visualHeight,
                1, 1,
                carAngle,
                0, 0,
                carTexture.getWidth(), carTexture.getHeight(),
                false, false);
    }

    private void drawWheel(SpriteBatch batch, Body wheelBody) {
        float diameter = WHEEL_RADIUS * 2;
        float wheelAngle = wheelBody.getAngle() * MathUtils.radiansToDegrees;

        // Draw wheel with proper rotation
        batch.draw(wheelTexture,
                wheelBody.getPosition().x - WHEEL_RADIUS,
                wheelBody.getPosition().y - WHEEL_RADIUS,
                WHEEL_RADIUS, WHEEL_RADIUS,
                diameter, diameter,
                1, 1,
                wheelAngle,
                0, 0,
                wheelTexture.getWidth(), wheelTexture.getHeight(),
                false, false);
    }

    @Override
    public void resize(int width, int height) {
        viewport.update(width, height);
        camera.position.set(viewport.getWorldWidth() / 2, viewport.getWorldHeight() / 2, 0);
    }

    @Override
    public void dispose() {
        batch.dispose();
        if (carTexture != null) carTexture.dispose();
        if (wheelTexture != null) wheelTexture.dispose();
        if (terrainTexture != null) terrainTexture.dispose();
        if (respawnButtonTexture != null) respawnButtonTexture.dispose();
        world.dispose();
        debugRenderer.dispose();
    }
}
