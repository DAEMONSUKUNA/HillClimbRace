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
    private static final float PPM = 24f;

    private boolean isCarFlipped = false;
    private final float FLIP_THRESHOLD = 0.5f;
    private Rectangle respawnButtonRect;

    private Array<Vector2> terrainVerts;

    private OrthographicCamera camera;
    private Viewport viewport;
    private static final float WORLD_WIDTH = 800;
    private static final float WORLD_HEIGHT = 480;

    private float enginePower = 300f;

    private final float CAR_WIDTH = 5.0f;
    private final float CAR_HEIGHT = 4f;
    private final float WHEEL_RADIUS = 0.6f;
    private final float WHEEL_X_OFFSET = 1.5f;
    private final float WHEEL_Y_OFFSET = -.7f;

    private final float FLAT_START_LENGTH = 20.0f;
    private final float CAR_SPAWN_X = 5.0f;

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

    private boolean showDebug = true;

    @Override
    public void create() {
    batch = new SpriteBatch();

    carTexture = new Texture(Gdx.files.internal("car.png"));
    wheelTexture = new Texture(Gdx.files.internal("wheel.png"));
    terrainTexture = new Texture(Gdx.files.internal("terrain.png"));
    respawnButtonTexture = new Texture(Gdx.files.internal("respawn_button.png"));

    respawnButtonRect = new Rectangle(20, 20, 80, 80);

        respawnButtonRect = new Rectangle(20, 20, 80, 80);

        camera = new OrthographicCamera();
        viewport = new FitViewport(WORLD_WIDTH / PPM, WORLD_HEIGHT / PPM, camera);
        viewport.apply();
        camera.position.set(viewport.getWorldWidth() / 2, viewport.getWorldHeight() / 2, 0);

        world = new World(new Vector2(0, -12f), true);
        debugRenderer = new Box2DDebugRenderer();

        terrainVerts = createTerrain();

        createCarWithWheels();

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

            }

            @Override
            public void endContact(Contact contact) {

            }

            @Override
            public void preSolve(Contact contact, Manifold oldManifold) {

            }

            @Override
            public void postSolve(Contact contact, ContactImpulse impulse) {

            }
        });
    }

    private Array<Vector2> createTerrain() {
        BodyDef groundDef = new BodyDef();
        groundDef.position.set(0, 0);
        groundBody = world.createBody(groundDef);

        ChainShape groundShape = new ChainShape();

        Vector2[] vertices = generateTerrain(200, 6000, 50);
        groundShape.createChain(vertices);

        FixtureDef fixtureDef = new FixtureDef();
        fixtureDef.shape = groundShape;
        fixtureDef.friction = 1.0f;
        fixtureDef.restitution = 0.1f;

        groundBody.createFixture(fixtureDef);
        groundShape.dispose();

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

        int flatStartVertices = (int)(FLAT_START_LENGTH / (segmentWidth / PPM));

        for (int i = 0; i < points; i++) {
            float x = i * segmentWidth / PPM;
            float height;

            if (i < flatStartVertices) {
                height = baseHeight / PPM;
            } else {

                height = baseHeight / PPM;

                height += (MathUtils.sin((x - (flatStartVertices * segmentWidth / PPM)) * 0.8f) * 15f) / PPM;
                height += (MathUtils.sin((x - (flatStartVertices * segmentWidth / PPM)) * 0.3f) * 25f) / PPM;
                height += (MathUtils.sin((x - (flatStartVertices * segmentWidth / PPM)) * 2f) * 3f) / PPM;

                if (x > (flatStartVertices * segmentWidth / PPM) + 5 && MathUtils.random() < 0.08 && i > flatStartVertices) {

                    float hillHeight = MathUtils.random(10f, 20f) / PPM;
                    float hillWidth = MathUtils.random(5, 15);

                    for (int j = 0; j < hillWidth && i + j < points; j++) {

                        float factor = (float) Math.sin((j / hillWidth) * Math.PI);

                        if (j == 0) {
                            height += hillHeight * factor;
                        }
                    }
                }

                if (i > flatStartVertices) {
                    height = lastHeight * 0.5f + height * 0.5f;
                }

                height += MathUtils.random(-0.5f, 0.5f) / PPM;
            }

            height = Math.max(height, 10f / PPM);
            lastHeight = height;

            vertices[i] = new Vector2(x, height);
        }

        return vertices;
    }

    private void createCarWithWheels() {

        float spawnY = getTerrainHeightAt(CAR_SPAWN_X) + WHEEL_RADIUS + 0.5f;

        BodyDef carDef = new BodyDef();
        carDef.type = BodyDef.BodyType.DynamicBody;
        carDef.position.set(CAR_SPAWN_X, spawnY);
        carBody = world.createBody(carDef);

        PolygonShape carShape = new PolygonShape();
        carShape.setAsBox(CAR_WIDTH / 3, CAR_HEIGHT / 6);

        FixtureDef carFixture = new FixtureDef();
        carFixture.shape = carShape;
        carFixture.density = 8.0f;
        carFixture.friction = 0.6f;
        carFixture.restitution = 0.1f;
        carBody.createFixture(carFixture);
        carShape.dispose();

        createWheel(true);
        createWheel(false);
    }

    private float getTerrainHeightAt(float x) {

        if (terrainVerts != null && terrainVerts.size > 1) {
            for (int i = 0; i < terrainVerts.size - 1; i++) {
                if (terrainVerts.get(i).x <= x && terrainVerts.get(i + 1).x >= x) {

                    Vector2 v1 = terrainVerts.get(i);
                    Vector2 v2 = terrainVerts.get(i + 1);

                    float t = (x - v1.x) / (v2.x - v1.x);

                    return v1.y + t * (v2.y - v1.y);
                }
            }
        }

        return 4f;
    }

    private void createWheel(boolean isLeftWheel) {

        float xOffset = isLeftWheel ? -WHEEL_X_OFFSET : WHEEL_X_OFFSET;

        BodyDef wheelDef = new BodyDef();
        wheelDef.type = BodyDef.BodyType.DynamicBody;
        wheelDef.position.set(carBody.getPosition().x + xOffset, carBody.getPosition().y + WHEEL_Y_OFFSET);

        Body wheelBody = world.createBody(wheelDef);

        CircleShape wheelShape = new CircleShape();
        wheelShape.setRadius(WHEEL_RADIUS);

        FixtureDef wheelFixture = new FixtureDef();
        wheelFixture.shape = wheelShape;
        wheelFixture.density = 1.0f;
        wheelFixture.friction = 2.0f;
        wheelFixture.restitution = 0.1f;

        wheelBody.createFixture(wheelFixture);
        wheelShape.dispose();

        RevoluteJointDef jointDef = new RevoluteJointDef();
        jointDef.bodyA = carBody;
        jointDef.bodyB = wheelBody;
        jointDef.localAnchorA.set(xOffset, WHEEL_Y_OFFSET);
        jointDef.localAnchorB.set(0, 0);
        jointDef.enableMotor = true;
        jointDef.maxMotorTorque = 200f;
        jointDef.motorSpeed = 0;

        RevoluteJoint joint = (RevoluteJoint) world.createJoint(jointDef);

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

        world.step(deltaTime, 6, 2);

        isCarFlipped = checkIfCarFlipped();

        handleInput();

        updateCamera();

        if (Gdx.input.isKeyJustPressed(Input.Keys.B)) {
            showDebug = !showDebug;
        }

        if (isCarFlipped && Gdx.input.isKeyJustPressed(Input.Keys.R)) {
            respawnCar();
        }

        if (isCarFlipped && Gdx.input.justTouched()) {

            Vector3 touchPos = new Vector3(Gdx.input.getX(), Gdx.input.getY(), 0);

            camera.unproject(touchPos);

            if (respawnButtonRect.contains(touchPos.x, touchPos.y)) {
                respawnCar();
            }
        }
    }

    private boolean checkIfCarFlipped() {

        float rotation = carBody.getAngle() % (2 * MathUtils.PI);

        return Math.abs(rotation) > MathUtils.PI - FLIP_THRESHOLD;
    }

    private void respawnCar() {

        float spawnY = getTerrainHeightAt(CAR_SPAWN_X) + WHEEL_RADIUS + 0.5f;

        carBody.setTransform(CAR_SPAWN_X, spawnY, 0);
        carBody.setLinearVelocity(0, 0);
        carBody.setAngularVelocity(0);

        leftWheelBody.setTransform(CAR_SPAWN_X - WHEEL_X_OFFSET, spawnY + WHEEL_Y_OFFSET, 0);
        leftWheelBody.setLinearVelocity(0, 0);
        leftWheelBody.setAngularVelocity(0);

        rightWheelBody.setTransform(CAR_SPAWN_X + WHEEL_X_OFFSET, spawnY + WHEEL_Y_OFFSET, 0);
        rightWheelBody.setLinearVelocity(0, 0);
        rightWheelBody.setAngularVelocity(0);

        isCarFlipped = false;
    }

    private void handleInput() {

        boolean keyPressed = false;

        if (Gdx.input.isKeyPressed(Input.Keys.D) || Gdx.input.isKeyPressed(Input.Keys.RIGHT)) {
            keyPressed = true;
            leftWheelJoint.setMotorSpeed(-enginePower);
            rightWheelJoint.setMotorSpeed(-enginePower);
        }

        if (Gdx.input.isKeyPressed(Input.Keys.A) || Gdx.input.isKeyPressed(Input.Keys.LEFT)) {
            keyPressed = true;
            leftWheelJoint.setMotorSpeed(enginePower);
            rightWheelJoint.setMotorSpeed(enginePower);
        }

        if (Gdx.input.isKeyPressed(Input.Keys.SPACE)) {
            keyPressed = true;
            leftWheelJoint.setMotorSpeed(0);
            rightWheelJoint.setMotorSpeed(0);
            leftWheelBody.setAngularVelocity(leftWheelBody.getAngularVelocity() * 0.8f);
            rightWheelBody.setAngularVelocity(rightWheelBody.getAngularVelocity() * 0.8f);
        }

        if (!keyPressed) {
            leftWheelJoint.setMotorSpeed(0);
            rightWheelJoint.setMotorSpeed(0);

            leftWheelBody.setAngularVelocity(leftWheelBody.getAngularVelocity() * 0.98f);
            rightWheelBody.setAngularVelocity(rightWheelBody.getAngularVelocity() * 0.98f);
        }
    }

    private void updateCamera() {

        Vector2 carPosition = carBody.getPosition();

        float lookAheadX = carBody.getLinearVelocity().x * 0.7f;

        camera.position.x = camera.position.x + (carPosition.x + lookAheadX - camera.position.x) * 0.1f;
        float targetY = carPosition.y + 3f;
        camera.position.y = camera.position.y + (targetY - camera.position.y) * 0.1f;

        camera.position.y = Math.max(camera.position.y, 3f);

        camera.zoom = 1.2f;

        camera.update();
    }

    @Override
    public void render() {
        update();

        Gdx.gl.glClearColor(SKY_R, SKY_G, SKY_B, 1);
        Gdx.gl.glClear(GL20.GL_COLOR_BUFFER_BIT);

        batch.setProjectionMatrix(camera.combined);

        batch.begin();

        drawTerrain();

        drawCar();

        drawWheel(batch, leftWheelBody);
        drawWheel(batch, rightWheelBody);

        if (isCarFlipped) {

            float buttonX = camera.position.x - camera.viewportWidth * camera.zoom / 2 + 1.0f;
            float buttonY = camera.position.y - camera.viewportHeight * camera.zoom / 2 + 1.0f;
            batch.draw(respawnButtonTexture, buttonX, buttonY, 3.0f, 3.0f);

            respawnButtonRect.set(buttonX, buttonY, 3.0f, 3.0f);
        }

        batch.end();

        if (showDebug) {
            debugRenderer.render(world, camera.combined);
        }
    }

    private void drawTerrain() {

        if (terrainVerts != null && terrainVerts.size > 1) {
            for (int i = 0; i < terrainVerts.size - 1; i++) {
                Vector2 v1 = terrainVerts.get(i);
                Vector2 v2 = terrainVerts.get(i + 1);

                float x1 = v1.x;
                float y1 = v1.y;
                float x2 = v2.x;
                float y2 = v2.y;

                float width = Vector2.dst(x1, y1, x2, y2);
                float height = 0.3f;

                float angle = MathUtils.atan2(y2 - y1, x2 - x1) * MathUtils.radiansToDegrees;

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

            for (int i = 0; i < terrainVerts.size - 1; i++) {
                Vector2 v1 = terrainVerts.get(i);
                Vector2 v2 = terrainVerts.get(i + 1);

                float bottomY = camera.position.y - camera.viewportHeight * camera.zoom / 2 - 1.0f;

                batch.draw(terrainTexture,
                    v1.x, bottomY,
                    v2.x - v1.x, v1.y - bottomY);
            }
        }
    }

    private void drawCar() {
        float carAngle = carBody.getAngle() * MathUtils.radiansToDegrees;

        float visualHeight = CAR_HEIGHT * 1.5f;
        float heightOffset = (visualHeight - CAR_HEIGHT) / 2;

        batch.draw(carTexture,
                carBody.getPosition().x - CAR_WIDTH/2,
                carBody.getPosition().y - CAR_HEIGHT/2 - heightOffset,
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
