#include "GameWidget.h"

#include <QCursor>
#include <QKeyEvent>
#include <QMatrix4x4>
#include <QMouseEvent>
#include <QPainter>
#include <QtMath>

#if defined(Q_OS_MACOS)
#include <OpenGL/gl.h>
#else
#include <GL/gl.h>
#endif

#include <algorithm>
#include <cmath>

namespace {
constexpr float DegToRad = 3.14159265358979323846f / 180.0f;

float clamp(float value, float low, float high)
{
    return std::max(low, std::min(value, high));
}

void drawCube(float centerX, float centerY, float centerZ, float sizeX, float sizeY, float sizeZ)
{
    const float x0 = centerX - sizeX * 0.5f;
    const float x1 = centerX + sizeX * 0.5f;
    const float y0 = centerY - sizeY * 0.5f;
    const float y1 = centerY + sizeY * 0.5f;
    const float z0 = centerZ - sizeZ * 0.5f;
    const float z1 = centerZ + sizeZ * 0.5f;

    glBegin(GL_QUADS);
    glVertex3f(x0, y0, z1); glVertex3f(x1, y0, z1); glVertex3f(x1, y1, z1); glVertex3f(x0, y1, z1);
    glVertex3f(x1, y0, z0); glVertex3f(x0, y0, z0); glVertex3f(x0, y1, z0); glVertex3f(x1, y1, z0);
    glVertex3f(x0, y1, z1); glVertex3f(x1, y1, z1); glVertex3f(x1, y1, z0); glVertex3f(x0, y1, z0);
    glVertex3f(x0, y0, z0); glVertex3f(x1, y0, z0); glVertex3f(x1, y0, z1); glVertex3f(x0, y0, z1);
    glVertex3f(x0, y0, z0); glVertex3f(x0, y0, z1); glVertex3f(x0, y1, z1); glVertex3f(x0, y1, z0);
    glVertex3f(x1, y0, z1); glVertex3f(x1, y0, z0); glVertex3f(x1, y1, z0); glVertex3f(x1, y1, z1);
    glEnd();
}

void drawSolidBox(const QVector3D &center, const QVector3D &size)
{
    const float x0 = center.x() - size.x() * 0.5f;
    const float x1 = center.x() + size.x() * 0.5f;
    const float y0 = center.y() - size.y() * 0.5f;
    const float y1 = center.y() + size.y() * 0.5f;
    const float z0 = center.z() - size.z() * 0.5f;
    const float z1 = center.z() + size.z() * 0.5f;

    glBegin(GL_QUADS);
    glNormal3f(0.0f, 0.0f, 1.0f);
    glVertex3f(x0, y0, z1); glVertex3f(x1, y0, z1); glVertex3f(x1, y1, z1); glVertex3f(x0, y1, z1);
    glNormal3f(0.0f, 0.0f, -1.0f);
    glVertex3f(x1, y0, z0); glVertex3f(x0, y0, z0); glVertex3f(x0, y1, z0); glVertex3f(x1, y1, z0);
    glNormal3f(0.0f, 1.0f, 0.0f);
    glVertex3f(x0, y1, z1); glVertex3f(x1, y1, z1); glVertex3f(x1, y1, z0); glVertex3f(x0, y1, z0);
    glNormal3f(0.0f, -1.0f, 0.0f);
    glVertex3f(x0, y0, z0); glVertex3f(x1, y0, z0); glVertex3f(x1, y0, z1); glVertex3f(x0, y0, z1);
    glNormal3f(-1.0f, 0.0f, 0.0f);
    glVertex3f(x0, y0, z0); glVertex3f(x0, y0, z1); glVertex3f(x0, y1, z1); glVertex3f(x0, y1, z0);
    glNormal3f(1.0f, 0.0f, 0.0f);
    glVertex3f(x1, y0, z1); glVertex3f(x1, y0, z0); glVertex3f(x1, y1, z0); glVertex3f(x1, y1, z1);
    glEnd();
}

void drawBoxTransformed(const QVector3D &center,
                        const QVector3D &size,
                        const QVector3D &color,
                        float pitchDegrees = 0.0f,
                        float yawDegrees = 0.0f,
                        float rollDegrees = 0.0f)
{
    glPushMatrix();
    glTranslatef(center.x(), center.y(), center.z());
    glRotatef(yawDegrees, 0.0f, 1.0f, 0.0f);
    glRotatef(pitchDegrees, 1.0f, 0.0f, 0.0f);
    glRotatef(rollDegrees, 0.0f, 0.0f, 1.0f);
    glColor3f(color.x(), color.y(), color.z());
    drawSolidBox(QVector3D(0.0f, 0.0f, 0.0f), size);
    glPopMatrix();
}

void drawSegmentBox(const QVector3D &start, const QVector3D &end, float thickness, const QVector3D &color)
{
    const QVector3D delta = end - start;
    const float length = delta.length();
    if (length < 0.001f) {
        return;
    }

    const float yaw = std::atan2(delta.x(), delta.z()) / DegToRad;
    const float horizontal = std::sqrt(delta.x() * delta.x() + delta.z() * delta.z());
    const float pitch = -std::atan2(delta.y(), horizontal) / DegToRad;
    drawBoxTransformed((start + end) * 0.5f, QVector3D(thickness, thickness, length), color, pitch, yaw);
}

bool intersectRayBox(const QVector3D &origin,
                     const QVector3D &direction,
                     const GameWidget::WorldBox &box,
                     float *hitDistance,
                     QVector3D *hitNormal)
{
    const QVector3D minCorner = box.center - box.size * 0.5f;
    const QVector3D maxCorner = box.center + box.size * 0.5f;
    float tMin = 0.001f;
    float tMax = 500.0f;
    QVector3D enterNormal;

    auto testAxis = [&](float originAxis, float directionAxis, float minAxis, float maxAxis, const QVector3D &axisNormal) {
        if (std::abs(directionAxis) < 0.0001f) {
            return originAxis >= minAxis && originAxis <= maxAxis;
        }

        float t1 = (minAxis - originAxis) / directionAxis;
        float t2 = (maxAxis - originAxis) / directionAxis;
        QVector3D normal = -axisNormal;
        if (t1 > t2) {
            std::swap(t1, t2);
            normal = axisNormal;
        }

        if (t1 > tMin) {
            tMin = t1;
            enterNormal = normal;
        }
        tMax = std::min(tMax, t2);
        return tMin <= tMax;
    };

    if (!testAxis(origin.x(), direction.x(), minCorner.x(), maxCorner.x(), QVector3D(1.0f, 0.0f, 0.0f))) {
        return false;
    }
    if (!testAxis(origin.y(), direction.y(), minCorner.y(), maxCorner.y(), QVector3D(0.0f, 1.0f, 0.0f))) {
        return false;
    }
    if (!testAxis(origin.z(), direction.z(), minCorner.z(), maxCorner.z(), QVector3D(0.0f, 0.0f, 1.0f))) {
        return false;
    }

    *hitDistance = tMin;
    *hitNormal = enterNormal;
    return true;
}

void addDecalQuad(const QVector3D &position, const QVector3D &normal, float size)
{
    QVector3D tangent = QVector3D::crossProduct(normal, QVector3D(0.0f, 1.0f, 0.0f));
    if (tangent.lengthSquared() < 0.01f) {
        tangent = QVector3D::crossProduct(normal, QVector3D(1.0f, 0.0f, 0.0f));
    }
    tangent.normalize();
    QVector3D bitangent = QVector3D::crossProduct(normal, tangent).normalized();

    const QVector3D p = position + normal * 0.018f;
    const QVector3D a = p - tangent * size - bitangent * size;
    const QVector3D b = p + tangent * size - bitangent * size;
    const QVector3D c = p + tangent * size + bitangent * size;
    const QVector3D d = p - tangent * size + bitangent * size;
    glBegin(GL_QUADS);
    glVertex3f(a.x(), a.y(), a.z());
    glVertex3f(b.x(), b.y(), b.z());
    glVertex3f(c.x(), c.y(), c.z());
    glVertex3f(d.x(), d.y(), d.z());
    glEnd();
}

float terrainNoise(float x, float z)
{
    const float rolling = std::sin(x * 0.045f) * 1.6f + std::cos(z * 0.038f) * 1.4f;
    const float ridges = std::sin((x + z) * 0.022f) * 1.2f + std::cos((x - z) * 0.031f) * 0.9f;
    const float detail = std::sin(x * 0.13f + z * 0.07f) * 0.32f;
    const float spawnFlatten = std::exp(-(x * x + (z - 70.0f) * (z - 70.0f)) / 2200.0f);
    return (rolling + ridges + detail + 2.4f) * (1.0f - spawnFlatten * 0.75f);
}

float deterministic01(int value)
{
    unsigned int x = static_cast<unsigned int>(value) * 747796405u + 2891336453u;
    x = ((x >> ((x >> 28u) + 4u)) ^ x) * 277803737u;
    x = (x >> 22u) ^ x;
    return float(x & 1023u) / 1023.0f;
}
}

GameWidget::GameWidget(QWidget *parent)
    : QOpenGLWidget(parent)
{
    setWindowTitle(QStringLiteral("Qt Wall Shooter"));
    setFocusPolicy(Qt::StrongFocus);
    setMouseTracking(true);

    connect(&m_frameTimer, &QTimer::timeout, this, &GameWidget::updateGame);
    m_frameTimer.start(16);
    m_elapsedTimer.start();
    buildWorld();
}

void GameWidget::initializeGL()
{
    glClearColor(0.36f, 0.49f, 0.68f, 1.0f);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);
    glEnable(GL_MULTISAMPLE);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glEnable(GL_FOG);
    const GLfloat fogColor[] = {0.36f, 0.49f, 0.68f, 1.0f};
    glFogfv(GL_FOG_COLOR, fogColor);
    glFogf(GL_FOG_MODE, GL_LINEAR);
    glFogf(GL_FOG_START, 85.0f);
    glFogf(GL_FOG_END, 240.0f);
}

void GameWidget::resizeGL(int width, int height)
{
    glViewport(0, 0, width, std::max(1, height));
}

void GameWidget::paintGL()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    QMatrix4x4 projection;
    const float fieldOfView = 70.0f + (28.0f - 70.0f) * m_scopeAmount;
    projection.perspective(fieldOfView, float(width()) / float(std::max(1, height())), 0.05f, 220.0f);

    QMatrix4x4 view;
    view.lookAt(m_position, m_position + forwardVector(), QVector3D(0.0f, 1.0f, 0.0f));

    glMatrixMode(GL_PROJECTION);
    glLoadMatrixf(projection.constData());
    glMatrixMode(GL_MODELVIEW);
    glLoadMatrixf(view.constData());

    drawScene();
    drawCrosshairAndHud();
}

void GameWidget::keyPressEvent(QKeyEvent *event)
{
    if (!event->isAutoRepeat()) {
        m_pressedKeys.insert(event->key());
        if (event->key() == Qt::Key_Space) {
            m_jumpQueued = true;
        }
    }

    if (event->key() == Qt::Key_Escape) {
        releaseMouseCapture();
    } else if (event->key() == Qt::Key_R) {
        resetGame();
    }
}

void GameWidget::keyReleaseEvent(QKeyEvent *event)
{
    if (!event->isAutoRepeat()) {
        m_pressedKeys.remove(event->key());
    }
}

void GameWidget::mouseMoveEvent(QMouseEvent *event)
{
    if (!m_mouseCaptured) {
        return;
    }

    const QPoint center(width() / 2, height() / 2);
    const QPoint delta = event->pos() - center;
    if (delta.isNull()) {
        return;
    }

    const float sensitivity = 0.12f * (1.0f - m_scopeAmount * 0.55f);
    m_yaw += float(delta.x()) * sensitivity;
    m_pitch = clamp(m_pitch - float(delta.y()) * sensitivity, -82.0f, 82.0f);

    centerCapturedMouse();
}

void GameWidget::mousePressEvent(QMouseEvent *event)
{
    if (event->button() == Qt::LeftButton) {
        if (!m_mouseCaptured) {
            captureMouse();
        }
        shoot();
    } else if (event->button() == Qt::RightButton) {
        if (!m_mouseCaptured) {
            captureMouse();
        }
        m_isScoped = true;
    }
}

void GameWidget::mouseReleaseEvent(QMouseEvent *event)
{
    if (event->button() == Qt::RightButton) {
        m_isScoped = false;
    }
}

void GameWidget::focusOutEvent(QFocusEvent *)
{
    m_pressedKeys.clear();
    m_isScoped = false;
    m_jumpQueued = false;
    releaseMouseCapture();
}

void GameWidget::buildWorld()
{
    m_worldBoxes.clear();

    auto addBox = [&](float x, float y, float z, float sx, float sy, float sz, const QVector3D &color) {
        m_worldBoxes.push_back({QVector3D(x, y, z), QVector3D(sx, sy, sz), color, true});
    };

    addBox(0.0f, 7.5f, -96.0f, 78.0f, 15.0f, 3.0f, QVector3D(0.54f, 0.58f, 0.63f));
    addBox(-54.0f, 5.5f, -56.0f, 28.0f, 11.0f, 26.0f, QVector3D(0.44f, 0.52f, 0.56f));
    addBox(54.0f, 8.0f, -46.0f, 25.0f, 16.0f, 25.0f, QVector3D(0.58f, 0.49f, 0.45f));
    addBox(0.0f, 3.0f, -28.0f, 34.0f, 6.0f, 22.0f, QVector3D(0.38f, 0.46f, 0.50f));
    addBox(-28.0f, 9.0f, -14.0f, 10.0f, 18.0f, 10.0f, QVector3D(0.69f, 0.61f, 0.49f));
    addBox(28.0f, 12.0f, -10.0f, 9.0f, 24.0f, 9.0f, QVector3D(0.43f, 0.50f, 0.66f));
    addBox(0.0f, 0.6f, 18.0f, 56.0f, 1.2f, 20.0f, QVector3D(0.31f, 0.41f, 0.39f));
    addBox(-78.0f, 3.0f, 12.0f, 20.0f, 6.0f, 48.0f, QVector3D(0.48f, 0.54f, 0.47f));
    addBox(82.0f, 4.0f, 22.0f, 22.0f, 8.0f, 42.0f, QVector3D(0.51f, 0.45f, 0.56f));

    for (int i = 0; i < 9; ++i) {
        addBox(-14.0f + i * 3.5f, 0.35f + i * 0.45f, 48.0f - i * 4.0f,
               6.0f, 0.7f + i * 0.25f, 4.0f, QVector3D(0.57f, 0.52f, 0.42f));
    }
    for (int i = 0; i < 12; ++i) {
        addBox(-45.0f, 0.3f + i * 0.55f, 50.0f - i * 4.2f,
               16.0f, 0.6f + i * 0.22f, 3.8f, QVector3D(0.46f, 0.57f, 0.51f));
    }
    for (int i = 0; i < 10; ++i) {
        addBox(46.0f, 0.35f + i * 0.75f, 42.0f - i * 4.0f,
               14.0f, 0.7f + i * 0.28f, 3.6f, QVector3D(0.56f, 0.47f, 0.49f));
    }

    addBox(-118.0f, 5.0f, 0.0f, 4.0f, 10.0f, 220.0f, QVector3D(0.33f, 0.38f, 0.45f));
    addBox(118.0f, 5.0f, 0.0f, 4.0f, 10.0f, 220.0f, QVector3D(0.33f, 0.38f, 0.45f));
    addBox(0.0f, 5.0f, -118.0f, 220.0f, 10.0f, 4.0f, QVector3D(0.33f, 0.38f, 0.45f));
    addBox(0.0f, 5.0f, 118.0f, 220.0f, 10.0f, 4.0f, QVector3D(0.33f, 0.38f, 0.45f));
}

void GameWidget::updateGame()
{
    const float deltaSeconds = std::min(0.05f, float(m_elapsedTimer.restart()) / 1000.0f);
    m_fpsUpdateTimer += deltaSeconds;
    ++m_framesSinceFpsUpdate;
    if (m_fpsUpdateTimer >= 0.35f) {
        m_currentFps = float(m_framesSinceFpsUpdate) / m_fpsUpdateTimer;
        m_framesSinceFpsUpdate = 0;
        m_fpsUpdateTimer = 0.0f;
    }

    const bool crouching = m_pressedKeys.contains(Qt::Key_Control) || m_pressedKeys.contains(Qt::Key_C);
    const float crouchTarget = crouching ? 1.0f : 0.0f;
    const float scopeTarget = m_isScoped ? 1.0f : 0.0f;
    m_crouchAmount += (crouchTarget - m_crouchAmount) * std::min(1.0f, deltaSeconds * 12.0f);
    m_scopeAmount += (scopeTarget - m_scopeAmount) * std::min(1.0f, deltaSeconds * 14.0f);

    QVector3D movement;
    if (m_pressedKeys.contains(Qt::Key_W)) {
        movement += forwardVector();
    }
    if (m_pressedKeys.contains(Qt::Key_S)) {
        movement -= forwardVector();
    }
    if (m_pressedKeys.contains(Qt::Key_D)) {
        movement += rightVector();
    }
    if (m_pressedKeys.contains(Qt::Key_A)) {
        movement -= rightVector();
    }

    const bool sprinting = m_pressedKeys.contains(Qt::Key_Shift) && !crouching && !m_isScoped;
    m_isWalking = !movement.isNull();
    if (m_isWalking) {
        m_walkCycle += deltaSeconds * (sprinting ? 10.0f : 7.0f);
    } else {
        m_walkCycle += deltaSeconds * 2.0f;
    }

    movement.setY(0.0f);
    if (!movement.isNull()) {
        movement.normalize();
        const float speed = (sprinting ? 14.0f : 8.5f) * (1.0f - m_crouchAmount * 0.42f) * (1.0f - m_scopeAmount * 0.28f);
        m_position += movement * speed * deltaSeconds;
    }

    m_position.setX(clamp(m_position.x(), -WorldHalfSize + 2.0f, WorldHalfSize - 2.0f));
    m_position.setZ(clamp(m_position.z(), -WorldHalfSize + 2.0f, WorldHalfSize - 2.0f));
    const float floor = groundHeightAt(m_position.x(), m_position.z()) + currentEyeHeight();

    if (m_jumpQueued && m_isGrounded && m_crouchAmount < 0.65f) {
        m_verticalVelocity = 8.8f;
        m_isGrounded = false;
    }
    m_jumpQueued = false;

    if (m_isGrounded) {
        const float heightError = floor - m_position.y();
        m_verticalVelocity += heightError * 38.0f * deltaSeconds;
        m_verticalVelocity *= std::pow(0.015f, deltaSeconds);
    } else {
        m_verticalVelocity -= 24.0f * deltaSeconds;
    }

    m_verticalVelocity = clamp(m_verticalVelocity, -32.0f, 14.0f);
    m_position.setY(m_position.y() + m_verticalVelocity * deltaSeconds);
    if (m_position.y() < floor) {
        m_position.setY(floor);
        m_verticalVelocity = 0.0f;
        m_isGrounded = true;
    } else if (m_position.y() > floor + 0.08f) {
        m_isGrounded = false;
    }
    if (m_position.y() > MaxClimbHeight) {
        m_position.setY(MaxClimbHeight);
        m_verticalVelocity = std::min(0.0f, m_verticalVelocity);
    }

    m_shotFlash = std::max(0.0f, m_shotFlash - deltaSeconds * 5.0f);
    for (BulletMark &mark : m_bulletMarks) {
        mark.age += deltaSeconds;
    }

    constexpr float bulletSpeed = 95.0f;
    for (auto bullet = m_flyingBullets.begin(); bullet != m_flyingBullets.end();) {
        bullet->traveled += bulletSpeed * deltaSeconds;
        if (bullet->traveled >= bullet->distance) {
            if (bullet->willHit) {
                m_bulletMarks.push_back({bullet->target, bullet->normal, 0.0f});
                if (m_bulletMarks.size() > 120) {
                    m_bulletMarks.erase(m_bulletMarks.begin());
                }
            }
            bullet = m_flyingBullets.erase(bullet);
        } else {
            ++bullet;
        }
    }

    update();
}

void GameWidget::shoot()
{
    ++m_totalShots;
    m_shotFlash = 1.0f;

    const QVector3D direction = forwardVector();
    const QVector3D start = m_position + rightVector() * 0.18f + direction * 0.45f + QVector3D(0.0f, -0.10f, 0.0f);
    const SurfaceHit hit = traceShot(m_position, direction);
    if (hit.hit) {
        const float distance = std::max(0.2f, (hit.position - start).length());
        m_flyingBullets.push_back({start, hit.position, hit.normal, distance, 0.0f, true});
    } else {
        const QVector3D target = start + direction * 180.0f;
        m_flyingBullets.push_back({start, target, QVector3D(0.0f, 1.0f, 0.0f), 180.0f, 0.0f, false});
    }
}

void GameWidget::resetGame()
{
    m_position = QVector3D(0.0f, groundHeightAt(0.0f, 74.0f) + PlayerEyeHeight, 74.0f);
    m_yaw = -90.0f;
    m_pitch = 0.0f;
    m_verticalVelocity = 0.0f;
    m_fpsUpdateTimer = 0.0f;
    m_framesSinceFpsUpdate = 0;
    m_walkCycle = 0.0f;
    m_crouchAmount = 0.0f;
    m_scopeAmount = 0.0f;
    m_isScoped = false;
    m_isWalking = false;
    m_isGrounded = true;
    m_jumpQueued = false;
    m_totalShots = 0;
    m_shotFlash = 0.0f;
    m_flyingBullets.clear();
    m_bulletMarks.clear();
}

void GameWidget::captureMouse()
{
    m_mouseCaptured = true;
    setCursor(Qt::BlankCursor);
    grabMouse();
    setFocus();
    centerCapturedMouse();
}

void GameWidget::releaseMouseCapture()
{
    if (!m_mouseCaptured) {
        return;
    }

    m_mouseCaptured = false;
    m_isScoped = false;
    QWidget::releaseMouse();
    unsetCursor();
}

void GameWidget::centerCapturedMouse()
{
    if (m_mouseCaptured) {
        QCursor::setPos(mapToGlobal(QPoint(width() / 2, height() / 2)));
    }
}

QVector3D GameWidget::forwardVector() const
{
    const float yaw = m_yaw * DegToRad;
    const float pitch = m_pitch * DegToRad;
    return QVector3D(std::cos(yaw) * std::cos(pitch),
                     std::sin(pitch),
                     std::sin(yaw) * std::cos(pitch)).normalized();
}

QVector3D GameWidget::rightVector() const
{
    return QVector3D::crossProduct(forwardVector(), QVector3D(0.0f, 1.0f, 0.0f)).normalized();
}

float GameWidget::currentEyeHeight() const
{
    return PlayerEyeHeight + (CrouchEyeHeight - PlayerEyeHeight) * m_crouchAmount;
}

GameWidget::SurfaceHit GameWidget::traceShot(const QVector3D &origin, const QVector3D &direction) const
{
    SurfaceHit best;
    best.distance = 500.0f;

    for (float t = 0.5f; t < best.distance; t += 1.0f) {
        const QVector3D position = origin + direction * t;
        if (std::abs(position.x()) > WorldHalfSize || std::abs(position.z()) > WorldHalfSize) {
            break;
        }
        const float terrain = terrainHeightAt(position.x(), position.z());
        if (position.y() <= terrain) {
            best = {QVector3D(position.x(), terrain, position.z()), terrainNormalAt(position.x(), position.z()), t, true};
            break;
        }
    }

    for (const WorldBox &box : m_worldBoxes) {
        float distance = 0.0f;
        QVector3D normal;
        if (intersectRayBox(origin, direction, box, &distance, &normal) && distance < best.distance) {
            best = {origin + direction * distance, normal, distance, true};
        }
    }

    return best;
}

float GameWidget::terrainHeightAt(float x, float z) const
{
    return terrainNoise(x, z);
}

QVector3D GameWidget::terrainNormalAt(float x, float z) const
{
    const float step = 0.75f;
    const float left = terrainHeightAt(x - step, z);
    const float right = terrainHeightAt(x + step, z);
    const float back = terrainHeightAt(x, z - step);
    const float front = terrainHeightAt(x, z + step);
    return QVector3D(left - right, step * 2.0f, back - front).normalized();
}

float GameWidget::groundHeightAt(float x, float z) const
{
    float height = terrainHeightAt(x, z);
    for (const WorldBox &box : m_worldBoxes) {
        const QVector3D half = box.size * 0.5f;
        if (x >= box.center.x() - half.x() && x <= box.center.x() + half.x()
            && z >= box.center.z() - half.z() && z <= box.center.z() + half.z()) {
            const float top = box.center.y() + half.y();
            if (top <= MaxClimbHeight) {
                height = std::max(height, top);
            }
        }
    }
    return height;
}

void GameWidget::drawScene()
{
    drawSky();
    drawGround();
    drawNature();
    drawWorldGeometry();
    drawPlayerBody();
    drawMirror();
    drawFlyingBullets();
    drawBulletMarks();
}

void GameWidget::drawSky()
{
    glDisable(GL_DEPTH_TEST);
    glDisable(GL_CULL_FACE);
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadIdentity();

    glBegin(GL_QUADS);
    glColor3f(0.12f, 0.16f, 0.26f);
    glVertex3f(-180.0f, -60.0f, -200.0f);
    glVertex3f(180.0f, -60.0f, -200.0f);
    glColor3f(0.50f, 0.68f, 0.86f);
    glVertex3f(180.0f, 130.0f, -200.0f);
    glVertex3f(-180.0f, 130.0f, -200.0f);
    glEnd();

    glPopMatrix();
    glEnable(GL_CULL_FACE);
    glEnable(GL_DEPTH_TEST);
}

void GameWidget::drawGround()
{
    glDisable(GL_CULL_FACE);

    const int cells = 60;
    const float step = (WorldHalfSize * 2.0f) / float(cells);
    for (int z = 0; z < cells; ++z) {
        glBegin(GL_TRIANGLE_STRIP);
        for (int x = 0; x <= cells; ++x) {
            for (int row = 0; row < 2; ++row) {
                const float px = -WorldHalfSize + x * step;
                const float pz = -WorldHalfSize + (z + row) * step;
                const float height = terrainHeightAt(px, pz);
                const QVector3D normal = terrainNormalAt(px, pz);
                const float lush = clamp((height + 1.5f) / 7.5f, 0.0f, 1.0f);
                glColor3f(0.10f + lush * 0.10f, 0.30f + lush * 0.22f, 0.13f + lush * 0.08f);
                glNormal3f(normal.x(), normal.y(), normal.z());
                glVertex3f(px, height, pz);
            }
        }
        glEnd();
    }

    glColor3f(0.70f, 0.75f, 0.62f);
    glLineWidth(3.0f);
    glBegin(GL_LINE_LOOP);
    glVertex3f(-WorldHalfSize, terrainHeightAt(-WorldHalfSize, -WorldHalfSize) + 0.08f, -WorldHalfSize);
    glVertex3f(WorldHalfSize, terrainHeightAt(WorldHalfSize, -WorldHalfSize) + 0.08f, -WorldHalfSize);
    glVertex3f(WorldHalfSize, terrainHeightAt(WorldHalfSize, WorldHalfSize) + 0.08f, WorldHalfSize);
    glVertex3f(-WorldHalfSize, terrainHeightAt(-WorldHalfSize, WorldHalfSize) + 0.08f, WorldHalfSize);
    glEnd();
    glEnable(GL_CULL_FACE);
}

void GameWidget::drawNature()
{
    glDisable(GL_CULL_FACE);
    glLineWidth(1.0f);
    glBegin(GL_LINES);
    for (int i = 0; i < 520; ++i) {
        const float rx = deterministic01(i * 17 + 5);
        const float rz = deterministic01(i * 29 + 11);
        const float x = -WorldHalfSize + rx * WorldHalfSize * 2.0f;
        const float z = -WorldHalfSize + rz * WorldHalfSize * 2.0f;
        if (std::abs(x) < 14.0f && z > 42.0f && z < 66.0f) {
            continue;
        }
        const float y = terrainHeightAt(x, z) + 0.04f;
        const float blade = 0.22f + deterministic01(i * 31 + 9) * 0.42f;
        const float sway = (deterministic01(i * 13 + 3) - 0.5f) * 0.22f;
        glColor3f(0.18f + deterministic01(i) * 0.10f, 0.48f + deterministic01(i + 7) * 0.22f, 0.18f);
        glVertex3f(x, y, z);
        glVertex3f(x + sway, y + blade, z + sway * 0.45f);
    }
    glEnd();

    for (int i = 0; i < 34; ++i) {
        const float x = -130.0f + deterministic01(i * 41 + 2) * 260.0f;
        const float z = -132.0f + deterministic01(i * 47 + 8) * 264.0f;
        if ((std::abs(x) < 24.0f && z > 36.0f && z < 76.0f) || std::abs(z - MirrorZ) < 7.0f) {
            continue;
        }
        const float y = terrainHeightAt(x, z);
        const float trunkHeight = 1.7f + deterministic01(i * 19) * 1.6f;
        glColor3f(0.25f, 0.16f, 0.09f);
        drawSolidBox(QVector3D(x, y + trunkHeight * 0.5f, z), QVector3D(0.55f, trunkHeight, 0.55f));
        glColor3f(0.10f, 0.34f + deterministic01(i + 3) * 0.16f, 0.15f);
        drawSolidBox(QVector3D(x, y + trunkHeight + 0.75f, z), QVector3D(3.0f, 1.6f, 3.0f));
        glColor3f(0.08f, 0.28f, 0.13f);
        drawSolidBox(QVector3D(x, y + trunkHeight + 1.65f, z), QVector3D(2.1f, 1.3f, 2.1f));
    }

    for (int i = 0; i < 46; ++i) {
        const float x = -140.0f + deterministic01(i * 53 + 1) * 280.0f;
        const float z = -140.0f + deterministic01(i * 59 + 4) * 280.0f;
        const float y = terrainHeightAt(x, z) + 0.18f;
        const float scale = 0.45f + deterministic01(i * 23) * 1.25f;
        glColor3f(0.28f, 0.30f, 0.28f);
        drawSolidBox(QVector3D(x, y, z), QVector3D(scale * 1.4f, scale * 0.55f, scale));
    }

    glEnable(GL_CULL_FACE);
}

void GameWidget::drawGrid()
{
    glDisable(GL_CULL_FACE);
    glLineWidth(1.0f);
    glColor4f(0.32f, 0.42f, 0.38f, 0.45f);
    glBegin(GL_LINES);
    for (int meter = -150; meter <= 150; meter += 10) {
        glVertex3f(float(meter), 0.04f, -WorldHalfSize);
        glVertex3f(float(meter), 0.04f, WorldHalfSize);
        glVertex3f(-WorldHalfSize, 0.04f, float(meter));
        glVertex3f(WorldHalfSize, 0.04f, float(meter));
    }
    glEnd();
    glEnable(GL_CULL_FACE);
}

void GameWidget::drawWorldGeometry()
{
    for (const WorldBox &box : m_worldBoxes) {
        glColor3f(box.color.x(), box.color.y(), box.color.z());
        drawSolidBox(box.center, box.size);

        glDisable(GL_CULL_FACE);
        glColor4f(0.92f, 0.96f, 0.92f, 0.18f);
        glLineWidth(1.0f);
        const QVector3D topCenter = box.center + QVector3D(0.0f, box.size.y() * 0.5f + 0.015f, 0.0f);
        const QVector3D topSize(box.size.x(), 0.0f, box.size.z());
        glBegin(GL_LINE_LOOP);
        glVertex3f(topCenter.x() - topSize.x() * 0.5f, topCenter.y(), topCenter.z() - topSize.z() * 0.5f);
        glVertex3f(topCenter.x() + topSize.x() * 0.5f, topCenter.y(), topCenter.z() - topSize.z() * 0.5f);
        glVertex3f(topCenter.x() + topSize.x() * 0.5f, topCenter.y(), topCenter.z() + topSize.z() * 0.5f);
        glVertex3f(topCenter.x() - topSize.x() * 0.5f, topCenter.y(), topCenter.z() + topSize.z() * 0.5f);
        glEnd();
        glEnable(GL_CULL_FACE);
    }
}

void GameWidget::drawPlayerBody()
{
    if (m_pitch > -12.0f) {
        return;
    }

    drawPlayerBodyModel(m_position, m_yaw, m_pitch, m_crouchAmount, false, true);
}

void GameWidget::drawPlayerBodyModel(const QVector3D &position, float yawDegrees, float pitchDegrees, float crouchAmount, bool includeHead, bool includeWeapon)
{
    const float stride = m_isWalking ? std::sin(m_walkCycle) : 0.0f;
    const float counterStride = m_isWalking ? std::sin(m_walkCycle + 3.14159265f) : 0.0f;
    const float footLift = m_isWalking ? std::abs(std::sin(m_walkCycle)) * 0.06f : 0.0f;
    const float bodyBob = m_isWalking ? std::abs(std::sin(m_walkCycle * 2.0f)) * 0.035f : 0.0f;
    const float hipDrop = crouchAmount * 0.26f;
    const float torsoDrop = crouchAmount * 0.42f;
    const float headDrop = crouchAmount * 0.52f;
    const float kneeForward = crouchAmount * 0.28f;

    glDisable(GL_CULL_FACE);
    glPushMatrix();
    glTranslatef(position.x(), position.y() - currentEyeHeight() + bodyBob, position.z());
    glRotatef(-(yawDegrees + 90.0f), 0.0f, 1.0f, 0.0f);

    const QVector3D suitDark(0.09f, 0.11f, 0.16f);
    const QVector3D suitMid(0.18f, 0.23f, 0.29f);
    const QVector3D suitLight(0.24f, 0.31f, 0.38f);
    const QVector3D armor(0.34f, 0.40f, 0.42f);
    const QVector3D skin(0.62f, 0.50f, 0.41f);
    const QVector3D black(0.035f, 0.040f, 0.050f);
    const QVector3D cyan(0.10f, 0.45f, 0.50f);

    drawBoxTransformed(QVector3D(0.0f, 1.18f - torsoDrop, -0.18f), QVector3D(0.72f, 0.54f, 0.34f), suitMid, crouchAmount * 5.0f);
    drawBoxTransformed(QVector3D(0.0f, 0.89f - hipDrop, -0.14f), QVector3D(0.56f, 0.38f, 0.30f), suitMid);
    drawBoxTransformed(QVector3D(0.0f, 1.43f - torsoDrop, -0.20f), QVector3D(0.92f, 0.18f, 0.30f), suitLight);
    drawBoxTransformed(QVector3D(0.0f, 0.66f - hipDrop, -0.10f), QVector3D(0.68f, 0.16f, 0.34f), suitDark);
    drawBoxTransformed(QVector3D(0.0f, 1.27f - torsoDrop, -0.39f), QVector3D(0.38f, 0.26f, 0.08f), armor);
    drawBoxTransformed(QVector3D(0.0f, 1.30f - torsoDrop, -0.45f), QVector3D(0.18f, 0.12f, 0.05f), cyan);
    drawBoxTransformed(QVector3D(-0.28f, 1.19f - torsoDrop, -0.41f), QVector3D(0.18f, 0.38f, 0.06f), armor);
    drawBoxTransformed(QVector3D(0.28f, 1.19f - torsoDrop, -0.41f), QVector3D(0.18f, 0.38f, 0.06f), armor);
    drawBoxTransformed(QVector3D(0.0f, 0.88f - hipDrop, -0.34f), QVector3D(0.36f, 0.16f, 0.06f), armor);

    const QVector3D leftHip(-0.22f, 0.66f - hipDrop, -0.10f);
    const QVector3D rightHip(0.22f, 0.66f - hipDrop, -0.10f);
    const QVector3D leftKnee(-0.21f, 0.38f + footLift * 0.4f - crouchAmount * 0.08f, -0.12f + stride * 0.10f - kneeForward);
    const QVector3D rightKnee(0.21f, 0.38f + footLift * 0.4f - crouchAmount * 0.08f, -0.12f + counterStride * 0.10f - kneeForward);
    const QVector3D leftAnkle(-0.21f, 0.16f + footLift, -0.18f + stride * 0.18f);
    const QVector3D rightAnkle(0.21f, 0.16f + footLift, -0.18f + counterStride * 0.18f);

    drawSegmentBox(leftHip, leftKnee, 0.19f, suitDark);
    drawSegmentBox(leftKnee, leftAnkle, 0.17f, suitDark);
    drawSegmentBox(rightHip, rightKnee, 0.19f, suitDark);
    drawSegmentBox(rightKnee, rightAnkle, 0.17f, suitDark);
    drawBoxTransformed(leftKnee, QVector3D(0.23f, 0.18f, 0.22f), armor);
    drawBoxTransformed(rightKnee, QVector3D(0.23f, 0.18f, 0.22f), armor);
    drawBoxTransformed(QVector3D(-0.21f, 0.06f + footLift, -0.30f + stride * 0.20f), QVector3D(0.34f, 0.16f, 0.54f), black);
    drawBoxTransformed(QVector3D(0.21f, 0.06f + footLift, -0.30f + counterStride * 0.20f), QVector3D(0.34f, 0.16f, 0.54f), black);
    drawBoxTransformed(QVector3D(-0.21f, 0.00f + footLift, -0.31f + stride * 0.20f), QVector3D(0.38f, 0.06f, 0.60f), QVector3D(0.01f, 0.012f, 0.014f));
    drawBoxTransformed(QVector3D(0.21f, 0.00f + footLift, -0.31f + counterStride * 0.20f), QVector3D(0.38f, 0.06f, 0.60f), QVector3D(0.01f, 0.012f, 0.014f));

    const QVector3D leftShoulder(-0.46f, 1.38f - torsoDrop, -0.24f);
    const QVector3D rightShoulder(0.46f, 1.38f - torsoDrop, -0.24f);
    const QVector3D leftElbow(-0.47f, 1.10f - torsoDrop * 0.80f, -0.58f);
    const QVector3D rightElbow(0.46f, 1.04f - torsoDrop * 0.80f, -0.42f);
    const QVector3D leftHand(-0.18f, 0.88f - torsoDrop * 0.62f, -1.05f);
    const QVector3D rightHand(0.15f, 0.76f - torsoDrop * 0.62f, -0.60f);

    drawSegmentBox(leftShoulder, leftElbow, 0.17f, suitLight);
    drawSegmentBox(leftElbow, leftHand, 0.15f, suitMid);
    drawSegmentBox(rightShoulder, rightElbow, 0.17f, suitLight);
    drawSegmentBox(rightElbow, rightHand, 0.15f, suitMid);
    drawBoxTransformed(leftShoulder, QVector3D(0.24f, 0.22f, 0.24f), suitLight);
    drawBoxTransformed(rightShoulder, QVector3D(0.24f, 0.22f, 0.24f), suitLight);
    drawBoxTransformed(leftShoulder + QVector3D(-0.08f, 0.02f, 0.00f), QVector3D(0.30f, 0.14f, 0.32f), armor);
    drawBoxTransformed(rightShoulder + QVector3D(0.08f, 0.02f, 0.00f), QVector3D(0.30f, 0.14f, 0.32f), armor);
    drawBoxTransformed(leftElbow, QVector3D(0.20f, 0.18f, 0.20f), armor);
    drawBoxTransformed(rightElbow, QVector3D(0.20f, 0.18f, 0.20f), armor);
    drawBoxTransformed((leftElbow + leftHand) * 0.5f + QVector3D(0.0f, 0.02f, -0.02f), QVector3D(0.19f, 0.10f, 0.28f), armor, -18.0f, -24.0f);
    drawBoxTransformed((rightElbow + rightHand) * 0.5f + QVector3D(0.0f, 0.02f, -0.02f), QVector3D(0.19f, 0.10f, 0.28f), armor, -16.0f, 24.0f);
    drawBoxTransformed(leftHand + QVector3D(0.0f, 0.07f, 0.03f), QVector3D(0.20f, 0.06f, 0.18f), suitDark, -8.0f, -6.0f);
    drawBoxTransformed(rightHand + QVector3D(0.0f, 0.07f, 0.03f), QVector3D(0.20f, 0.06f, 0.18f), suitDark, -10.0f, 4.0f);
    drawBoxTransformed(leftHand, QVector3D(0.18f, 0.15f, 0.20f), skin, -8.0f, -6.0f, 0.0f);
    drawBoxTransformed(rightHand, QVector3D(0.18f, 0.15f, 0.20f), skin, -10.0f, 4.0f, 0.0f);

    if (includeHead) {
        glPushMatrix();
        glTranslatef(0.0f, 1.68f - headDrop, -0.20f);
        glRotatef(-pitchDegrees * 0.45f, 1.0f, 0.0f, 0.0f);
        drawBoxTransformed(QVector3D(0.0f, -0.18f, 0.02f), QVector3D(0.20f, 0.18f, 0.18f), skin);
        drawBoxTransformed(QVector3D(0.0f, 0.04f, 0.0f), QVector3D(0.42f, 0.42f, 0.36f), skin);
        drawBoxTransformed(QVector3D(0.0f, 0.29f, -0.01f), QVector3D(0.48f, 0.16f, 0.40f), black);
        drawBoxTransformed(QVector3D(-0.09f, 0.06f, -0.20f), QVector3D(0.06f, 0.04f, 0.035f), black);
        drawBoxTransformed(QVector3D(0.09f, 0.06f, -0.20f), QVector3D(0.06f, 0.04f, 0.035f), black);
        glPopMatrix();
    }

    if (includeWeapon) {
        glPushMatrix();
        glTranslatef(0.0f, 0.0f, 0.0f);
        glRotatef(-pitchDegrees * 0.35f, 1.0f, 0.0f, 0.0f);

        const QVector3D gunDark(0.055f, 0.065f, 0.075f);
        const QVector3D gunBody(0.13f, 0.16f, 0.18f);
        const QVector3D gunMetal(0.34f, 0.38f, 0.40f);

        drawBoxTransformed(QVector3D(0.0f, 1.04f, -0.72f), QVector3D(0.42f, 0.28f, 0.58f), gunBody);
        drawBoxTransformed(QVector3D(0.0f, 1.03f, -1.16f), QVector3D(0.18f, 0.18f, 0.70f), gunDark);
        drawBoxTransformed(QVector3D(0.0f, 1.22f, -0.78f), QVector3D(0.34f, 0.07f, 0.52f), gunMetal);
        drawBoxTransformed(QVector3D(0.0f, 1.34f, -0.88f), QVector3D(0.23f, 0.17f, 0.18f), gunDark);
        drawBoxTransformed(QVector3D(0.0f, 1.34f, -1.02f), QVector3D(0.12f, 0.09f, 0.16f), QVector3D(0.10f, 0.45f, 0.50f));
        drawBoxTransformed(QVector3D(0.0f, 0.83f, -0.56f), QVector3D(0.20f, 0.44f, 0.18f), gunDark, -12.0f);
        drawBoxTransformed(QVector3D(0.0f, 0.93f, -0.86f), QVector3D(0.25f, 0.08f, 0.12f), gunMetal);
        drawBoxTransformed(QVector3D(0.0f, 1.08f, -0.30f), QVector3D(0.48f, 0.22f, 0.30f), gunDark, 5.0f);
        drawBoxTransformed(QVector3D(0.0f, 1.03f, -1.58f), QVector3D(0.28f, 0.24f, 0.18f), gunMetal);
        drawBoxTransformed(QVector3D(0.0f, 1.03f, -1.70f), QVector3D(0.16f, 0.16f, 0.08f),
                           m_shotFlash > 0.0f ? QVector3D(1.0f, 0.70f, 0.16f) : QVector3D(0.82f, 0.42f, 0.10f));
        glPopMatrix();
    }

    glPopMatrix();
    glEnable(GL_CULL_FACE);
}

void GameWidget::drawMirror()
{
    const float mirrorGround = terrainHeightAt(0.0f, MirrorZ);
    const QVector3D mirrorBase(0.0f, mirrorGround, MirrorZ);
    const float mirrorWidth = 18.0f;
    const float mirrorHeight = 7.5f;
    const float reflectedZ = MirrorZ * 2.0f - m_position.z();
    const float playerFloor = groundHeightAt(m_position.x(), m_position.z()) + currentEyeHeight();
    const float verticalOffset = m_position.y() - playerFloor;
    const QVector3D reflectedPosition(m_position.x(),
                                      groundHeightAt(m_position.x(), reflectedZ) + currentEyeHeight() + verticalOffset,
                                      reflectedZ);

    glDisable(GL_CULL_FACE);

    glColor3f(0.08f, 0.09f, 0.11f);
    drawSolidBox(QVector3D(mirrorBase.x(), mirrorBase.y() + mirrorHeight * 0.5f, mirrorBase.z() + 0.18f),
                 QVector3D(mirrorWidth + 1.0f, mirrorHeight + 1.0f, 0.35f));

    drawPlayerBodyModel(reflectedPosition, -m_yaw, m_pitch, m_crouchAmount, true, true);

    glColor4f(0.58f, 0.78f, 0.92f, 0.42f);
    glBegin(GL_QUADS);
    glVertex3f(-mirrorWidth * 0.5f, mirrorBase.y() + 0.55f, MirrorZ + 0.34f);
    glVertex3f(mirrorWidth * 0.5f, mirrorBase.y() + 0.55f, MirrorZ + 0.34f);
    glVertex3f(mirrorWidth * 0.5f, mirrorBase.y() + mirrorHeight, MirrorZ + 0.34f);
    glVertex3f(-mirrorWidth * 0.5f, mirrorBase.y() + mirrorHeight, MirrorZ + 0.34f);
    glEnd();

    glColor4f(0.92f, 0.98f, 1.0f, 0.65f);
    glLineWidth(2.0f);
    glBegin(GL_LINE_LOOP);
    glVertex3f(-mirrorWidth * 0.5f, mirrorBase.y() + 0.55f, MirrorZ + 0.36f);
    glVertex3f(mirrorWidth * 0.5f, mirrorBase.y() + 0.55f, MirrorZ + 0.36f);
    glVertex3f(mirrorWidth * 0.5f, mirrorBase.y() + mirrorHeight, MirrorZ + 0.36f);
    glVertex3f(-mirrorWidth * 0.5f, mirrorBase.y() + mirrorHeight, MirrorZ + 0.36f);
    glEnd();

    glColor4f(1.0f, 1.0f, 1.0f, 0.45f);
    glBegin(GL_LINES);
    glVertex3f(-mirrorWidth * 0.32f, mirrorBase.y() + mirrorHeight - 0.8f, MirrorZ + 0.38f);
    glVertex3f(mirrorWidth * 0.18f, mirrorBase.y() + 1.1f, MirrorZ + 0.38f);
    glVertex3f(-mirrorWidth * 0.10f, mirrorBase.y() + mirrorHeight - 0.5f, MirrorZ + 0.38f);
    glVertex3f(mirrorWidth * 0.38f, mirrorBase.y() + 1.8f, MirrorZ + 0.38f);
    glEnd();

    glEnable(GL_CULL_FACE);
}

void GameWidget::drawFlyingBullets()
{
    glDisable(GL_CULL_FACE);
    glLineWidth(3.0f);
    for (const FlyingBullet &bullet : m_flyingBullets) {
        const QVector3D direction = (bullet.target - bullet.start).normalized();
        const float visibleTravel = std::min(bullet.traveled, bullet.distance);
        const QVector3D position = bullet.start + direction * visibleTravel;
        const QVector3D trailStart = bullet.start + direction * std::max(0.0f, visibleTravel - 2.8f);

        glColor4f(1.0f, 0.76f, 0.20f, 0.90f);
        glBegin(GL_LINES);
        glVertex3f(trailStart.x(), trailStart.y(), trailStart.z());
        glVertex3f(position.x(), position.y(), position.z());
        glEnd();

        glColor3f(1.0f, 0.92f, 0.34f);
        drawSolidBox(position, QVector3D(0.11f, 0.11f, 0.11f));
    }
    glLineWidth(1.0f);
    glEnable(GL_CULL_FACE);
}

void GameWidget::drawBulletMarks()
{
    glDisable(GL_CULL_FACE);
    for (const BulletMark &mark : m_bulletMarks) {
        const float size = 0.20f;
        const float glow = clamp(1.0f - mark.age * 2.2f, 0.0f, 1.0f);
        glColor4f(0.08f + glow * 0.92f, 0.02f + glow * 0.22f, 0.015f, 0.9f);
        addDecalQuad(mark.position, mark.normal, size);
    }
    glEnable(GL_CULL_FACE);
}

void GameWidget::drawCrosshairAndHud()
{
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);

    const QPoint center(width() / 2, height() / 2);
    if (m_scopeAmount > 0.02f) {
        const int radius = int(std::min(width(), height()) * (0.34f + m_scopeAmount * 0.03f));
        QColor shade(0, 0, 0, int(185 * m_scopeAmount));
        painter.fillRect(QRect(0, 0, width(), center.y() - radius), shade);
        painter.fillRect(QRect(0, center.y() + radius, width(), height() - center.y() - radius), shade);
        painter.fillRect(QRect(0, center.y() - radius, center.x() - radius, radius * 2), shade);
        painter.fillRect(QRect(center.x() + radius, center.y() - radius, width() - center.x() - radius, radius * 2), shade);

        QPen scopePen(QColor(12, 22, 24, int(225 * m_scopeAmount)), 5);
        painter.setPen(scopePen);
        painter.drawEllipse(center, radius, radius);
        painter.setPen(QPen(QColor(120, 235, 225, int(180 * m_scopeAmount)), 1));
        painter.drawEllipse(center, radius - 14, radius - 14);
        painter.drawLine(center.x() - radius + 26, center.y(), center.x() - 18, center.y());
        painter.drawLine(center.x() + 18, center.y(), center.x() + radius - 26, center.y());
        painter.drawLine(center.x(), center.y() - radius + 26, center.x(), center.y() - 18);
        painter.drawLine(center.x(), center.y() + 18, center.x(), center.y() + radius - 26);
        painter.setPen(QPen(QColor(255, 255, 255, int(85 * m_scopeAmount)), 2));
        painter.drawArc(QRect(center.x() - radius + 28, center.y() - radius + 28, (radius - 28) * 2, (radius - 28) * 2),
                        40 * 16, 42 * 16);
    }

    const int gap = m_scopeAmount > 0.5f ? 5 : 8;
    const int length = m_scopeAmount > 0.5f ? 10 : 17;
    const QColor crosshair = m_shotFlash > 0.0f
        ? QColor(255, 215, 95)
        : QColor(235, 245, 238);

    QPen pen(crosshair, 2);
    painter.setPen(pen);
    painter.drawLine(center.x() - length, center.y(), center.x() - gap, center.y());
    painter.drawLine(center.x() + gap, center.y(), center.x() + length, center.y());
    painter.drawLine(center.x(), center.y() - length, center.x(), center.y() - gap);
    painter.drawLine(center.x(), center.y() + gap, center.x(), center.y() + length);

    painter.setPen(QColor(230, 236, 230));
    painter.setFont(QFont(QStringLiteral("Sans Serif"), 11));
    const QString hud = QStringLiteral("FPS: %1    WASD move | Shift sprint | Space jump | Ctrl/C crouch | Right click scope | Click shoot | R reset | Esc cursor    Pos: %2, %3, %4 m    Shots: %5  Hits: %6")
        .arg(QString::number(m_currentFps, 'f', 0))
        .arg(QString::number(m_position.x(), 'f', 1),
             QString::number(m_position.y(), 'f', 1),
             QString::number(m_position.z(), 'f', 1))
        .arg(m_totalShots)
        .arg(m_bulletMarks.size());
    painter.drawText(QRect(18, 14, width() - 36, 32), Qt::AlignLeft | Qt::AlignVCenter, hud);

    painter.setPen(QColor(170, 210, 185));
    painter.drawText(QRect(18, height() - 42, width() - 36, 26),
                     Qt::AlignLeft | Qt::AlignVCenter,
                     QStringLiteral("World: 300 x 300 meters | Rolling hills, grass, trees, rocks, mirror, platforms, and shootable terrain"));
}
