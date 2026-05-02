#pragma once

#include <QElapsedTimer>
#include <QOpenGLWidget>
#include <QPoint>
#include <QSet>
#include <QTimer>
#include <QVector3D>

#include <vector>

class GameWidget final : public QOpenGLWidget
{
public:
    explicit GameWidget(QWidget *parent = nullptr);

protected:
    void initializeGL() override;
    void resizeGL(int width, int height) override;
    void paintGL() override;

    void keyPressEvent(QKeyEvent *event) override;
    void keyReleaseEvent(QKeyEvent *event) override;
    void mouseMoveEvent(QMouseEvent *event) override;
    void mousePressEvent(QMouseEvent *event) override;
    void mouseReleaseEvent(QMouseEvent *event) override;
    void focusOutEvent(QFocusEvent *event) override;

public:
    struct BulletMark {
        QVector3D position;
        QVector3D normal;
        float age = 0.0f;
    };

    struct FlyingBullet {
        QVector3D start;
        QVector3D target;
        QVector3D normal;
        float distance = 0.0f;
        float traveled = 0.0f;
        bool willHit = false;
    };

    struct WorldBox {
        QVector3D center;
        QVector3D size;
        QVector3D color;
        bool shootable = true;
    };

    struct SurfaceHit {
        QVector3D position;
        QVector3D normal;
        float distance = 0.0f;
        bool hit = false;
    };

private:
    void buildWorld();
    void updateGame();
    void shoot();
    void resetGame();
    void captureMouse();
    void releaseMouseCapture();
    void centerCapturedMouse();

    QVector3D forwardVector() const;
    QVector3D rightVector() const;
    float currentEyeHeight() const;
    SurfaceHit traceShot(const QVector3D &origin, const QVector3D &direction) const;
    float terrainHeightAt(float x, float z) const;
    QVector3D terrainNormalAt(float x, float z) const;
    float groundHeightAt(float x, float z) const;

    void drawScene();
    void drawSky();
    void drawGround();
    void drawNature();
    void drawGrid();
    void drawWorldGeometry();
    void drawPlayerBody();
    void drawPlayerBodyModel(const QVector3D &position, float yawDegrees, float pitchDegrees, float crouchAmount, bool includeHead, bool includeWeapon);
    void drawMirror();
    void drawFlyingBullets();
    void drawBulletMarks();
    void drawCrosshairAndHud();

    QTimer m_frameTimer;
    QElapsedTimer m_elapsedTimer;
    QSet<int> m_pressedKeys;
    std::vector<WorldBox> m_worldBoxes;
    std::vector<FlyingBullet> m_flyingBullets;
    std::vector<BulletMark> m_bulletMarks;

    QVector3D m_position = QVector3D(0.0f, 1.7f, 74.0f);

    float m_yaw = -90.0f;
    float m_pitch = 0.0f;
    float m_verticalVelocity = 0.0f;
    float m_shotFlash = 0.0f;
    float m_fpsUpdateTimer = 0.0f;
    float m_currentFps = 0.0f;
    float m_walkCycle = 0.0f;
    float m_crouchAmount = 0.0f;
    float m_scopeAmount = 0.0f;
    int m_totalShots = 0;
    int m_framesSinceFpsUpdate = 0;
    bool m_isWalking = false;
    bool m_isGrounded = true;
    bool m_jumpQueued = false;
    bool m_mouseCaptured = false;
    bool m_isScoped = false;

    static constexpr float WorldHalfSize = 150.0f;
    static constexpr float PlayerEyeHeight = 1.7f;
    static constexpr float CrouchEyeHeight = 1.05f;
    static constexpr float MaxClimbHeight = 34.0f;
    static constexpr float MirrorZ = 52.0f;
};
