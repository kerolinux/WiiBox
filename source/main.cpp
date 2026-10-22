// WiiBox - Wii Robotics & Physics Sandbox
// devkitPPC + libogc + GRRLIB + Box2D 2.4.x
//
// Memory policy: every Box2D body is created ONCE at startup. "Spawning" an
// obstacle enables a pooled body, "clearing" disables it. Dragging uses direct
// velocity control (no joints). HUD text uses a built-in bitmap font and a
// static snprintf buffer. Nothing is allocated in the main loop.

#include <grrlib.h>
#include <wiiuse/wpad.h>
#include <ogc/lwp_watchdog.h>
#include <ogc/system.h>
#include <box2d/box2d.h>

#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <cctype>
#include <cstring>

#include "nn.h"
#include "weights.h"

// ------------------------------------------------------------------ config
namespace cfg {
constexpr int   SCREEN_W      = 640;
constexpr int   SCREEN_H      = 480;
constexpr float PPM           = 28.0f;           // pixels per metre (15x11 m -> 420x308 px)
constexpr float CENTER_X      = 412.0f;          // screen position of world origin
constexpr float CENTER_Y      = 232.0f;          // (arena sits right of the side menu)
constexpr float DT            = 1.0f / 60.0f;    // fixed step (NTSC/PAL60 vsync)
constexpr int   VEL_ITERS     = 8;
constexpr int   POS_ITERS     = 3;

// Arena (metres, centred on origin). 15 x 11 m -> 600 x 440 px
constexpr float ARENA_W       = 15.0f;
constexpr float ARENA_H       = 11.0f;
constexpr float WALL_T        = 0.25f;           // wall half-thickness

// Robot
constexpr float ROBOT_HL      = 0.30f;           // half length (x, forward)
constexpr float ROBOT_HW      = 0.20f;           // half width  (y)
constexpr float ROBOT_DENS    = 5.0f;            // kg/m^2  -> 1.2 kg
constexpr float WHEEL_OFFSET  = 0.20f;           // wheel lateral offset from centre
constexpr float MAX_SPEED     = 3.0f;            // m/s at full command
constexpr float MOTOR_KP      = 8.0f;            // N per (m/s) speed error
constexpr float MAX_FORCE     = 12.0f;           // N per wheel
constexpr float WHEEL_GRIP    = 0.8f;            // lateral slip cancellation (0..1)
constexpr float LIN_DAMP      = 0.5f;
constexpr float ANG_DAMP      = 2.0f;             // rotational friction

// Sensors
constexpr int   N_SENS        = 5;
constexpr float SENS_RANGE    = 4.0f;            // metres
constexpr float SENS_ORIGIN_X = 0.31f;           // just in front of nose
constexpr float SENS_DEG[N_SENS] = { -45.0f, -22.5f, 0.0f, 22.5f, 45.0f };

// Props
constexpr int   N_BOXES       = 10;
constexpr int   N_CIRCLES     = 6;
constexpr float DRAG_GAIN     = 12.0f;
constexpr float DRAG_MAX_V    = 15.0f;

// Input
constexpr float STICK_DEADZONE = 0.15f;
} // namespace cfg

static_assert(cfg::N_SENS == weights::N_IN,  "sensor count must match NN inputs");
static_assert(weights::N_OUT == 2,           "NN must have 2 outputs");

// Static, configurable walls: {centre x, centre y, half width, half height}
struct Rect { float cx, cy, hw, hh; };

static const Rect kWalls[] = {
    // perimeter
    {  0.0f,                     cfg::ARENA_H * 0.5f + cfg::WALL_T, cfg::ARENA_W * 0.5f + 2 * cfg::WALL_T, cfg::WALL_T },
    {  0.0f,                    -cfg::ARENA_H * 0.5f - cfg::WALL_T, cfg::ARENA_W * 0.5f + 2 * cfg::WALL_T, cfg::WALL_T },
    { -cfg::ARENA_W * 0.5f - cfg::WALL_T, 0.0f, cfg::WALL_T, cfg::ARENA_H * 0.5f },
    {  cfg::ARENA_W * 0.5f + cfg::WALL_T, 0.0f, cfg::WALL_T, cfg::ARENA_H * 0.5f },
    // (the two default interior walls are paintable grid cells now - see loadDefaultWalls())
};
constexpr int kNumWalls = sizeof(kWalls) / sizeof(kWalls[0]);

// ------------------------------------------------------------------ helpers
static inline float sx(float x) { return cfg::CENTER_X + x * cfg::PPM; }
static inline float sy(float y) { return cfg::CENTER_Y - y * cfg::PPM; }
static inline b2Vec2 toWorld(float px, float py) {
    return b2Vec2((px - cfg::CENTER_X) / cfg::PPM,
                  (cfg::CENTER_Y - py) / cfg::PPM);
}

// ------------------------------------------------------------------ tiny 3x5 font
// Each glyph = 5 rows, 3 bits per row (bit2 = left pixel).
struct Glyph { char c; unsigned char r[5]; };
static const Glyph kFont[] = {
    {'0',{7,5,5,5,7}},{'1',{2,6,2,2,7}},{'2',{7,1,7,4,7}},{'3',{7,1,7,1,7}},{'4',{5,5,7,1,1}},
    {'5',{7,4,7,1,7}},{'6',{7,4,7,5,7}},{'7',{7,1,1,1,1}},{'8',{7,5,7,5,7}},{'9',{7,5,7,1,7}},
    {'A',{2,5,7,5,5}},{'B',{6,5,6,5,6}},{'C',{7,4,4,4,7}},{'D',{6,5,5,5,6}},{'E',{7,4,6,4,7}},
    {'F',{7,4,6,4,4}},{'G',{7,4,5,5,7}},{'H',{5,5,7,5,5}},{'I',{7,2,2,2,7}},{'J',{1,1,1,5,2}},
    {'K',{5,5,6,5,5}},{'L',{4,4,4,4,7}},{'M',{5,7,7,5,5}},{'N',{6,5,5,5,5}},{'O',{2,5,5,5,2}},
    {'P',{7,5,7,4,4}},{'Q',{2,5,5,7,3}},{'R',{6,5,6,5,5}},{'S',{3,4,2,1,6}},{'T',{7,2,2,2,2}},
    {'U',{5,5,5,5,7}},{'V',{5,5,5,5,2}},{'W',{5,5,7,7,5}},{'X',{5,5,2,5,5}},{'Y',{5,5,2,2,2}},
    {'Z',{7,1,2,4,7}},{'.',{0,0,0,0,2}},{'-',{0,0,7,0,0}},{':',{0,2,0,2,0}},{'/',{1,1,2,4,4}},
    {'+',{0,2,7,2,0}},{'(',{1,2,2,2,1}},{')',{4,2,2,2,4}},{'%',{5,1,2,4,5}},
};
constexpr int kFontCount = sizeof(kFont) / sizeof(kFont[0]);

static void drawText(float x, float y, int s, u32 color, const char* str)
{
    for (; *str; ++str, x += 4 * s) {
        const char c = (char)toupper((unsigned char)*str);
        if (c == ' ') continue;
        for (int g = 0; g < kFontCount; ++g) {
            if (kFont[g].c != c) continue;
            for (int row = 0; row < 5; ++row)
                for (int col = 0; col < 3; ++col)
                    if (kFont[g].r[row] & (4 >> col))
                        GRRLIB_Rectangle(x + col * s, y + row * s, (float)s, (float)s, color, true);
            break;
        }
    }
}

// ------------------------------------------------------------------ Box2D callbacks
class ClosestRay : public b2RayCastCallback {
public:
    const b2Body* ignore = nullptr;
    float fraction = 1.0f;
    bool  hit = false;
    float ReportFixture(b2Fixture* f, const b2Vec2&, const b2Vec2&, float frac) override {
        if (f->GetBody() == ignore || f->IsSensor()) return -1.0f;  // skip self
        fraction = frac; hit = true;
        return frac;                                                // clip to closest
    }
};

class PointQuery : public b2QueryCallback {
public:
    b2Vec2   p;
    b2Body*  found = nullptr;
    bool ReportFixture(b2Fixture* f) override {
        b2Body* b = f->GetBody();
        if (b->GetType() == b2_dynamicBody && f->TestPoint(p)) { found = b; return false; }
        return true;
    }
};

// ------------------------------------------------------------------ simulation state
struct Prop { b2Body* body; bool circle; float size; bool active; };

struct Robot {
    b2Body* body = nullptr;
    float sensNorm[cfg::N_SENS];             // 0..1 (NN input)
    float sensDist[cfg::N_SENS];             // metres
    b2Vec2 rayA[cfg::N_SENS], rayB[cfg::N_SENS];  // for drawing (B = hit point)
    float cmd[2]  = {0, 0};                  // left,right command  [-1,1]
    float vel[2]  = {0, 0};                  // measured wheel speed m/s
    bool  manual  = false;
};

static b2World* gWorld = nullptr;           // created once, lives until exit
static Robot    gRobot;
static Prop     gProps[cfg::N_BOXES + cfg::N_CIRCLES];
static int      gNextBox = 0, gNextCircle = 0;
static b2Body*  gDragged = nullptr;

static volatile bool gQuit = false;
static void onReset(u32, void*) { gQuit = true; }
static void onPower() { gQuit = true; }

static void buildArena()
{
    b2BodyDef bd;                            // static body, type defaults to static
    b2Body* walls = gWorld->CreateBody(&bd);
    for (int i = 0; i < kNumWalls; ++i) {
        b2PolygonShape s;
        s.SetAsBox(kWalls[i].hw, kWalls[i].hh, b2Vec2(kWalls[i].cx, kWalls[i].cy), 0.0f);
        walls->CreateFixture(&s, 0.0f);
    }
}

static void buildRobot()
{
    b2BodyDef bd;
    bd.type = b2_dynamicBody;
    bd.position.Set(-5.0f, 0.0f);
    bd.linearDamping  = cfg::LIN_DAMP;
    bd.angularDamping = cfg::ANG_DAMP;
    gRobot.body = gWorld->CreateBody(&bd);

    b2PolygonShape box;
    box.SetAsBox(cfg::ROBOT_HL, cfg::ROBOT_HW);
    b2FixtureDef fd;
    fd.shape = &box;
    fd.density = cfg::ROBOT_DENS;
    fd.friction = 0.3f;
    fd.restitution = 0.1f;
    gRobot.body->CreateFixture(&fd);

    for (int i = 0; i < cfg::N_SENS; ++i) gRobot.sensNorm[i] = 1.0f, gRobot.sensDist[i] = cfg::SENS_RANGE;
}

static void buildPropPool()
{
    int n = 0;
    for (int i = 0; i < cfg::N_BOXES + cfg::N_CIRCLES; ++i, ++n) {
        const bool circle = (i >= cfg::N_BOXES);
        b2BodyDef bd;
        bd.type = b2_dynamicBody;
        bd.enabled = false;                  // pooled, inactive until spawned
        bd.linearDamping  = 3.0f;            // top-down: damping stands in for floor friction
        bd.angularDamping = 3.0f;
        b2Body* b = gWorld->CreateBody(&bd);

        const float size = circle ? 0.25f : 0.25f;
        b2FixtureDef fd;
        fd.density = 1.0f; fd.friction = 0.4f; fd.restitution = 0.2f;
        b2PolygonShape box; b2CircleShape circ;
        if (circle) { circ.m_radius = size; fd.shape = &circ; }
        else        { box.SetAsBox(size, size); fd.shape = &box; }
        b->CreateFixture(&fd);

        gProps[n] = { b, circle, size, false };
    }
}

// ------------------------------------------------------------------ user walls (paintable grid / maze)
// 30 x 22 grid of 0.5 m static cells. All 660 bodies are created once, disabled; painting
// enables a cell, erasing disables it -> no allocation while the sim is running.
namespace grid {
constexpr float CELL = 0.5f;
constexpr int   COLS = 30;                       // 15 m / 0.5
constexpr int   ROWS = 22;                       // 11 m / 0.5
constexpr float X0   = -cfg::ARENA_W * 0.5f;
constexpr float Y0   = -cfg::ARENA_H * 0.5f;
}
static b2Body* gCellBody[grid::ROWS][grid::COLS];
static bool    gCellOn  [grid::ROWS][grid::COLS];
static int     gWallCount = 0;

// goal flag + run statistics
static bool   gGoalSet = false, gSeek = true, gGoalMode = false, gGoalReached = false;
static int    gGoalC = 0, gGoalR = 0, gGoalFrames = 0, gGoalCount = 0, gGoalBanner = 0, gPlanTimer = 0;
static b2Vec2 gGoalPos = b2Vec2(0.0f, 0.0f);
static float  gGoalLastTime = 0.0f;

struct Pose { b2Vec2 pos; float angle; };
static Pose gSpawn = { b2Vec2(-5.0f, 0.0f), 0.0f };

static void buildCellPool()
{
    b2PolygonShape box;
    box.SetAsBox(grid::CELL * 0.5f, grid::CELL * 0.5f);
    for (int r = 0; r < grid::ROWS; ++r)
        for (int c = 0; c < grid::COLS; ++c) {
            b2BodyDef bd;                        // static
            bd.enabled = false;
            bd.position.Set(grid::X0 + (c + 0.5f) * grid::CELL, grid::Y0 + (r + 0.5f) * grid::CELL);
            gCellBody[r][c] = gWorld->CreateBody(&bd);
            gCellBody[r][c]->CreateFixture(&box, 0.0f);
            gCellOn[r][c] = false;
        }
}

static inline bool cellOn(int c, int r)
{
    return c >= 0 && c < grid::COLS && r >= 0 && r < grid::ROWS && gCellOn[r][c];
}

static inline void cellAt(const b2Vec2& w, int& c, int& r)
{
    c = (int)floorf((w.x - grid::X0) / grid::CELL);
    r = (int)floorf((w.y - grid::Y0) / grid::CELL);
}

// returns true if the cell changed. `force` skips the "don't wall in the rover" guard.
static bool setCell(int c, int r, bool on, bool force = false)
{
    if (c < 0 || c >= grid::COLS || r < 0 || r >= grid::ROWS) return false;
    if (gCellOn[r][c] == on) return false;
    if (on && !force) {
        if (gGoalSet && c == gGoalC && r == gGoalR) return false;
        const b2Vec2 d = gRobot.body->GetPosition() - gCellBody[r][c]->GetPosition();
        if (d.Length() < 0.7f) return false;
    }
    gCellBody[r][c]->SetEnabled(on);
    gCellOn[r][c] = on;
    gWallCount += on ? 1 : -1;
    return true;
}

// the two starter walls: erasable like any painted wall
static void loadDefaultWalls()
{
    for (int r = 12; r <= 19; ++r) setCell(13, r, true, true);     // vertical bar  (x -1.0..-0.5 m)
    for (int c = 17; c <= 24; ++c) setCell(c, 5,  true, true);     // horizontal bar (y -3.0..-2.5 m)
}

// Bresenham so fast pointer sweeps leave no gaps
static void paintLine(int c0, int r0, int c1, int r1, bool on)
{
    int dc = abs(c1 - c0), dr = -abs(r1 - r0);
    const int sc = c0 < c1 ? 1 : -1, sr = r0 < r1 ? 1 : -1;
    int err = dc + dr;
    for (;;) {
        setCell(c0, r0, on);
        if (c0 == c1 && r0 == r1) break;
        const int e2 = 2 * err;
        if (e2 >= dr) { err += dr; c0 += sc; }
        if (e2 <= dc) { err += dc; r0 += sr; }
    }
}

static void clearWalls()
{
    for (int r = 0; r < grid::ROWS; ++r)
        for (int c = 0; c < grid::COLS; ++c) setCell(c, r, false, true);
}

// ------------------------------------------------------------------ goal + path planning
// A breadth-first distance field over the wall grid, grown outward from the goal. The rover
// reads the downhill direction a few cells ahead as its steering target. The planner first
// keeps one empty cell of clearance from walls (so the 0.6 x 0.4 m body fits); if that finds
// no route it retries with bare walls. The neural net stays in charge of local avoidance.
static short gDist[grid::ROWS][grid::COLS];

static bool nearWall(int c, int r)
{
    static const int dx[4] = { 1, -1, 0, 0 }, dy[4] = { 0, 0, 1, -1 };
    for (int k = 0; k < 4; ++k) {
        const int nc = c + dx[k], nr = r + dy[k];
        if (nc < 0 || nc >= grid::COLS || nr < 0 || nr >= grid::ROWS) return true;
        if (gCellOn[nr][nc]) return true;
    }
    return false;
}

static void planBfs(bool inflate)
{
    static int qc[grid::ROWS * grid::COLS], qr[grid::ROWS * grid::COLS];
    static const int dx[4] = { 1, -1, 0, 0 }, dy[4] = { 0, 0, 1, -1 };
    for (int r = 0; r < grid::ROWS; ++r)
        for (int c = 0; c < grid::COLS; ++c) gDist[r][c] = -1;
    int h = 0, t = 0;
    gDist[gGoalR][gGoalC] = 0; qc[t] = gGoalC; qr[t] = gGoalR; ++t;
    while (h < t) {
        const int c = qc[h], r = qr[h]; ++h;
        for (int k = 0; k < 4; ++k) {
            const int nc = c + dx[k], nr = r + dy[k];
            if (nc < 0 || nc >= grid::COLS || nr < 0 || nr >= grid::ROWS) continue;
            if (gDist[nr][nc] != -1 || gCellOn[nr][nc]) continue;
            if (inflate && nearWall(nc, nr)) continue;
            gDist[nr][nc] = (short)(gDist[r][c] + 1);
            qc[t] = nc; qr[t] = nr; ++t;
        }
    }
}

static bool robotReachable()
{
    int c, r;
    cellAt(gRobot.body->GetPosition(), c, r);
    for (int dr = -1; dr <= 1; ++dr)
        for (int dc = -1; dc <= 1; ++dc) {
            const int nc = c + dc, nr = r + dr;
            if (nc >= 0 && nc < grid::COLS && nr >= 0 && nr < grid::ROWS && gDist[nr][nc] >= 0) return true;
        }
    return false;
}

static void replan()
{
    if (!gGoalSet) return;
    planBfs(true);
    if (!robotReachable()) planBfs(false);
}

static void setGoal(int c, int r)
{
    if (c < 0 || c >= grid::COLS || r < 0 || r >= grid::ROWS || gCellOn[r][c]) return;
    gGoalC = c; gGoalR = r;
    gGoalPos = b2Vec2(grid::X0 + (c + 0.5f) * grid::CELL, grid::Y0 + (r + 0.5f) * grid::CELL);
    gGoalSet = true; gGoalReached = false; gGoalFrames = 0; gGoalBanner = 0; gPlanTimer = 0;
}

// Next steering target along the downhill path. false = no route known.
static bool navTarget(b2Vec2& target)
{
    static const int dx[4] = { 1, -1, 0, 0 }, dy[4] = { 0, 0, 1, -1 };
    const b2Vec2 p = gRobot.body->GetPosition();
    int c, r;
    cellAt(p, c, r);

    int bc = -1, br = -1; short best = 32767;            // lowest-distance cell around the rover
    for (int dr = -1; dr <= 1; ++dr)
        for (int dc = -1; dc <= 1; ++dc) {
            const int nc = c + dc, nr = r + dr;
            if (nc < 0 || nc >= grid::COLS || nr < 0 || nr >= grid::ROWS) continue;
            const short d = gDist[nr][nc];
            if (d >= 0 && d < best) { best = d; bc = nc; br = nr; }
        }
    if (bc < 0) return false;

    if (best <= 3 && (p - gGoalPos).Length() < 1.2f) { target = gGoalPos; return true; }

    for (int step = 0; step < 3 && best > 0; ++step) {   // look 1.5 m down the path
        int nbc = bc, nbr = br; short nb = best;
        for (int k = 0; k < 4; ++k) {
            const int nc = bc + dx[k], nr = br + dy[k];
            if (nc < 0 || nc >= grid::COLS || nr < 0 || nr >= grid::ROWS) continue;
            const short d = gDist[nr][nc];
            if (d >= 0 && d < nb) { nb = d; nbc = nc; nbr = nr; }
        }
        if (nb == best) break;
        bc = nbc; br = nbr; best = nb;
    }
    target = b2Vec2(grid::X0 + (bc + 0.5f) * grid::CELL, grid::Y0 + (br + 0.5f) * grid::CELL);
    return true;
}

// steer toward the target: turn proportional to heading error, slow down when misaligned
static bool navCommand(float& cl, float& cr)
{
    b2Vec2 target;
    if (!navTarget(target)) return false;
    const b2Vec2 p = gRobot.body->GetPosition();
    const float desired = atan2f(target.y - p.y, target.x - p.x);
    const float e = remainderf(desired - gRobot.body->GetAngle(), 2.0f * b2_pi);   // + = goal is to the left
    const float f = 0.8f * fmaxf(0.1f, cosf(e));
    const float t = fmaxf(-0.8f, fminf(0.8f, 1.2f * e));
    cl = f - t; cr = f + t;
    const float m = fmaxf(1.0f, fmaxf(fabsf(cl), fabsf(cr)));
    cl /= m; cr /= m;
    return true;
}

// ------------------------------------------------------------------ stuck recovery
// If the rover is being told to drive but has hardly moved or turned for 0.75 s,
// back up while swinging away from the wall, then spin toward the clearer side.
struct Unstick {
    enum State { IDLE, REVERSE, TURN, FORWARD };
    State  state = IDLE;
    int    timer = 0, window = 0, cooldown = 0, sinceLast = 9999;
    float  dir = 1.0f, lastDir = 1.0f;       // +1 = swing left (CCW), -1 = right
    float  activity = 0.0f;
    int    turnLen = 40;                     // max frames of spin (timeout)
    int    turnAge = 0;
    int    longWindow = 0;                   // 1.5 s window: catches endless spinning in place
    float  longActivity = 0.0f;
    b2Vec2 longRef = b2Vec2(0.0f, 0.0f);
    b2Vec2 refPos = b2Vec2(0.0f, 0.0f);
    float  refAng = 0.0f;
};
static Unstick gUn;
static u32 gRng = 2463534242u;
static inline u32 rnd32() { gRng ^= gRng << 13; gRng ^= gRng >> 17; gRng ^= gRng << 5; return gRng; }
static inline int rndRange(int a, int b) { return a + (int)(rnd32() % (u32)(b - a + 1)); }

static void unstickReset()
{
    gUn.state = Unstick::IDLE;
    gUn.window = 0; gUn.activity = 0.0f; gUn.cooldown = 30;
    gUn.longWindow = 0; gUn.longActivity = 0.0f;
    gUn.longRef = gRobot.body->GetPosition();
    gUn.refPos = gRobot.body->GetPosition();
    gUn.refAng = gRobot.body->GetAngle();
}

// Modifies the wheel commands in place when recovering. Returns true while overriding.
static bool unstickStep(float& cl, float& cr)
{
    Unstick& u = gUn;
    ++u.sinceLast;

    if (u.state == Unstick::REVERSE) {
        cl = -0.5f - 0.2f * u.dir;           // backing up while swinging the nose toward u.dir
        cr = -0.5f + 0.2f * u.dir;
        if (--u.timer <= 0) { u.state = Unstick::TURN; u.timer = u.turnLen; u.turnAge = 0; }
        return true;
    }
    if (u.state == Unstick::TURN) {
        cl = -0.45f * u.dir + 0.12f;         // gentle spin toward the clear side
        cr =  0.45f * u.dir + 0.12f;
        ++u.turnAge;
        const float side = fminf(gRobot.sensNorm[1], gRobot.sensNorm[3]);
        const bool aligned = u.turnAge > 12 && gRobot.sensNorm[2] > 0.55f && side > 0.35f;   // front open: stop turning
        if (aligned || --u.timer <= 0) { u.state = Unstick::FORWARD; u.timer = rndRange(20, 35); }
        return true;
    }
    if (u.state == Unstick::FORWARD) {       // commit to driving away (gently centred) so it can't re-enter the spin
        const float t = fmaxf(-0.4f, fminf(0.4f, 0.5f * ((gRobot.sensNorm[0] + gRobot.sensNorm[1]) -
                                                         (gRobot.sensNorm[3] + gRobot.sensNorm[4]))));
        cl = 0.55f + t; cr = 0.55f - t;
        if (--u.timer <= 0 || gRobot.sensNorm[2] < 0.2f) { unstickReset(); u.sinceLast = 0; }
        return true;
    }

    if (u.cooldown > 0) {                    // let the rover get going again
        --u.cooldown;
        u.refPos = u.longRef = gRobot.body->GetPosition();
        u.refAng = gRobot.body->GetAngle();
        u.window = u.longWindow = 0; u.activity = u.longActivity = 0.0f;
        return false;
    }

    const float drive = fmaxf(fabsf(cl), fabsf(cr));
    u.activity += drive; u.longActivity += drive;
    const b2Vec2 p = gRobot.body->GetPosition();
    bool stuck = false;

    if (++u.window >= 45) {                  // 0.75 s: pinned against something (not moving, not turning)
        const float a = gRobot.body->GetAngle();
        const float dist = (p - u.refPos).Length();
        const float dAng = fabsf(remainderf(a - u.refAng, 2.0f * b2_pi));
        if ((u.activity / 45.0f) > 0.3f && dist < 0.2f && dAng < 0.5f) stuck = true;
        u.window = 0; u.activity = 0.0f; u.refPos = p; u.refAng = a;
    }
    if (++u.longWindow >= 90) {              // 1.5 s: going nowhere, even if it is spinning
        if ((u.longActivity / 90.0f) > 0.3f && (p - u.longRef).Length() < 0.35f) stuck = true;
        u.longWindow = 0; u.longActivity = 0.0f; u.longRef = p;
    }

    if (stuck) {
        const float leftClear  = gRobot.sensNorm[3] + gRobot.sensNorm[4];
        const float rightClear = gRobot.sensNorm[0] + gRobot.sensNorm[1];
        u.dir = (leftClear >= rightClear) ? 1.0f : -1.0f;
        if (u.sinceLast < 180) u.dir = -u.lastDir;           // stuck again soon: try the other side
        u.lastDir = u.dir;
        u.state = Unstick::REVERSE;
        u.timer = rndRange(18, 30);
        float avg = 0.0f;
        for (int i = 0; i < cfg::N_SENS; ++i) avg += gRobot.sensNorm[i];
        avg /= (float)cfg::N_SENS;
        u.turnLen = (avg < 0.4f) ? rndRange(70, 100)         // dead end: allow a long turn-around
                                 : rndRange(45, 70);
        if (avg < 0.4f) u.timer = rndRange(30, 40);          // dead end: back out a bit further
        cl = -0.5f - 0.2f * u.dir; cr = -0.5f + 0.2f * u.dir;
        return true;
    }
    return false;
}

static void resetRobot()
{
    gRobot.body->SetTransform(gSpawn.pos, gSpawn.angle);
    gRobot.body->SetLinearVelocity(b2Vec2_zero);
    gRobot.body->SetAngularVelocity(0.0f);
    for (int r = 0; r < grid::ROWS; ++r)                     // keep the spawn point free of walls
        for (int c = 0; c < grid::COLS; ++c)
            if (gCellOn[r][c] && (gCellBody[r][c]->GetPosition() - gSpawn.pos).Length() < 0.9f)
                setCell(c, r, false, true);
    gGoalReached = false; gGoalFrames = 0; gGoalBanner = 0; gPlanTimer = 0;
    unstickReset();
}

static void clearProps()
{
    gDragged = (gDragged == gRobot.body) ? gDragged : nullptr;
    for (Prop& p : gProps) { p.body->SetEnabled(false); p.active = false; }
}

static void generateMaze()
{
    // Logical 7 x 5 maze. Each logical cell = 3x3 open grid cells (1.5 m corridors), walls 1 cell thick.
    constexpr int MC = 7, MR = 5, OFFC = 0, P = 4;            // P = pitch (3 open + 1 wall)
    clearProps();
    clearWalls();
    for (int gr = 0; gr <= P * MR; ++gr)
        for (int gc = 0; gc <= P * MC; ++gc)
            if (gc % P == 0 || gr % P == 0) setCell(gc + OFFC, gr, true, true);
    for (int r = 0; r < grid::ROWS; ++r)                      // fill the leftover strip on the top/right edge
        for (int c = 0; c < grid::COLS; ++c)
            if (c > P * MC + OFFC || r > P * MR) setCell(c, r, true, true);

    static const int DX[4] = { 1, -1, 0, 0 }, DY[4] = { 0, 0, 1, -1 };
    bool seen[MR][MC] = {};
    int  stack[MR * MC][2];
    int  sp = 0;
    gRng ^= (u32)gettime(); if (gRng == 0) gRng = 1;         // new maze every time

    seen[0][0] = true; stack[sp][0] = 0; stack[sp][1] = 0; ++sp;
    while (sp > 0) {
        const int ci = stack[sp - 1][0], cj = stack[sp - 1][1];
        int d[4] = { 0, 1, 2, 3 };
        for (int k = 3; k > 0; --k) { const int j = (int)(rnd32() % (u32)(k + 1)); const int t = d[k]; d[k] = d[j]; d[j] = t; }
        bool moved = false;
        for (int k = 0; k < 4 && !moved; ++k) {
            const int ni = ci + DX[d[k]], nj = cj + DY[d[k]];
            if (ni < 0 || ni >= MC || nj < 0 || nj >= MR || seen[nj][ni]) continue;
            if (ni != ci) {                                   // knock out wall column between cells
                const int gc = P * (ni > ci ? ni : ci);
                for (int gr = P * cj + 1; gr <= P * cj + 3; ++gr) setCell(gc + OFFC, gr, false, true);
            } else {                                          // knock out wall row between cells
                const int gr = P * (nj > cj ? nj : cj);
                for (int gc = P * ci + 1; gc <= P * ci + 3; ++gc) setCell(gc + OFFC, gr, false, true);
            }
            seen[nj][ni] = true; stack[sp][0] = ni; stack[sp][1] = nj; ++sp; moved = true;
        }
        if (!moved) --sp;
    }

    // rover starts in the middle of the bottom-left cell, facing an open passage
    gSpawn.pos = b2Vec2(grid::X0 + (2 + OFFC + 0.5f) * grid::CELL, grid::Y0 + (2 + 0.5f) * grid::CELL);
    gSpawn.angle = !cellOn(P + OFFC, 2) ? 0.0f : 0.5f * b2_pi;
    resetRobot();
    setGoal(P * (MC - 1) + 2 + OFFC, P * (MR - 1) + 2);      // flag in the far corner cell
}

static b2Body* spawnProp(const b2Vec2& at)
{
    Prop* p;
    static bool toggle = false;              // alternate box / circle
    toggle = !toggle;
    if (toggle) { p = &gProps[gNextBox];                  gNextBox    = (gNextBox + 1) % cfg::N_BOXES; }
    else        { p = &gProps[cfg::N_BOXES + gNextCircle]; gNextCircle = (gNextCircle + 1) % cfg::N_CIRCLES; }
    p->body->SetEnabled(true);               // (re)enable; recycles oldest if pool is full
    p->body->SetTransform(at, 0.0f);
    p->body->SetLinearVelocity(b2Vec2_zero);
    p->body->SetAngularVelocity(0.0f);
    p->body->SetAwake(true);
    p->active = true;
    return p->body;
}

// ------------------------------------------------------------------ sensors
static void updateSensors()
{
    b2Body* body = gRobot.body;
    ClosestRay cb;
    cb.ignore = body;
    const b2Vec2 origin = body->GetWorldPoint(b2Vec2(cfg::SENS_ORIGIN_X, 0.0f));

    for (int i = 0; i < cfg::N_SENS; ++i) {
        const float a = cfg::SENS_DEG[i] * (b2_pi / 180.0f);
        const b2Vec2 dir = body->GetWorldVector(b2Vec2(cosf(a), sinf(a)));
        const b2Vec2 end = origin + cfg::SENS_RANGE * dir;

        cb.fraction = 1.0f; cb.hit = false;
        gWorld->RayCast(&cb, origin, end);

        gRobot.sensNorm[i] = cb.fraction;
        gRobot.sensDist[i] = cb.fraction * cfg::SENS_RANGE;
        gRobot.rayA[i] = origin;
        gRobot.rayB[i] = origin + gRobot.sensDist[i] * dir;
    }
}

// ------------------------------------------------------------------ drive
// Differential drive: each wheel is a velocity servo along the body's forward axis
// applied at the wheel contact point (produces both thrust and yaw torque), plus a
// lateral-slip impulse that models wheel side-grip.
static void driveWheels(float cmdL, float cmdR)
{
    b2Body* body = gRobot.body;
    const b2Vec2 fwd = body->GetWorldVector(b2Vec2(1.0f, 0.0f));
    const b2Vec2 lat = body->GetWorldVector(b2Vec2(0.0f, 1.0f));
    const float cmds[2]  = { cmdL, cmdR };
    const float sides[2] = { +1.0f, -1.0f };   // left wheel is +y (CCW side)
    const float mass = body->GetMass();

    for (int i = 0; i < 2; ++i) {
        const b2Vec2 p = body->GetWorldPoint(b2Vec2(0.0f, sides[i] * cfg::WHEEL_OFFSET));
        const b2Vec2 v = body->GetLinearVelocityFromWorldPoint(p);
        const float vf = b2Dot(v, fwd);
        const float vl = b2Dot(v, lat);
        gRobot.vel[i] = vf;

        const float target = b2Clamp(cmds[i], -1.0f, 1.0f) * cfg::MAX_SPEED;
        const float F = b2Clamp(cfg::MOTOR_KP * (target - vf), -cfg::MAX_FORCE, cfg::MAX_FORCE);
        body->ApplyForce(F * fwd, p, true);
        body->ApplyLinearImpulse((-cfg::WHEEL_GRIP * 0.5f * mass * vl) * lat, p, true);
    }
}

// ------------------------------------------------------------------ input
struct Input {
    bool  irValid = false;
    float cx = cfg::SCREEN_W * 0.5f, cy = cfg::SCREEN_H * 0.5f;
    u32   down = 0, held = 0, up = 0;
    bool  manual = false;
    float manL = 0, manR = 0;
    bool  nunchuk = false;
    bool  stick = false;                     // Nunchuk stick deflected
};

static void readInput(Input& in)
{
    WPAD_ScanPads();
    in.down = WPAD_ButtonsDown(0);
    in.held = WPAD_ButtonsHeld(0);
    in.up   = WPAD_ButtonsUp(0);
    const WPADData* wd = WPAD_Data(0);

    in.irValid = wd->ir.valid;
    if (in.irValid) { in.cx = wd->ir.x; in.cy = wd->ir.y; }

    // Manual drive: Nunchuk stick (tank mix) or B + D-pad
    float fwd = 0.0f, turn = 0.0f;
    in.nunchuk = (wd->exp.type == WPAD_EXP_NUNCHUK);
    bool stick = false;
    if (in.nunchuk && wd->exp.nunchuk.js.mag > cfg::STICK_DEADZONE) {
        const float ang = wd->exp.nunchuk.js.ang * (b2_pi / 180.0f);   // 0 = up
        const float mag = fminf(wd->exp.nunchuk.js.mag, 1.0f);
        turn = mag * sinf(ang);      // right = +
        fwd  = mag * cosf(ang);
        stick = true;
    }
    const bool bHeld = (in.held & WPAD_BUTTON_B) != 0;
    if (bHeld && !stick) {
        if (in.held & WPAD_BUTTON_UP)    fwd  += 1.0f;
        if (in.held & WPAD_BUTTON_DOWN)  fwd  -= 1.0f;
        if (in.held & WPAD_BUTTON_RIGHT) turn += 1.0f;
        if (in.held & WPAD_BUTTON_LEFT)  turn -= 1.0f;
    }
    in.manual = stick || bHeld;
    in.stick  = stick;
    float l = fwd + turn, r = fwd - turn;          // left faster => turn right
    const float m = fmaxf(1.0f, fmaxf(fabsf(l), fabsf(r)));
    in.manL = l / m; in.manR = r / m;
}

static void handleDrag(const Input& in, bool allowStart)
{
    const b2Vec2 target = toWorld(in.cx, in.cy);

    if ((in.down & WPAD_BUTTON_A) && allowStart) {
        PointQuery q;
        q.p = target;
        b2AABB box;
        box.lowerBound = target - b2Vec2(0.01f, 0.01f);
        box.upperBound = target + b2Vec2(0.01f, 0.01f);
        gWorld->QueryAABB(&q, box);
        gDragged = q.found ? q.found : spawnProp(target);
    }
    if (!(in.held & WPAD_BUTTON_A)) gDragged = nullptr;

    if (gDragged) {
        b2Vec2 v = cfg::DRAG_GAIN * (target - gDragged->GetPosition());
        const float len = v.Length();
        if (len > cfg::DRAG_MAX_V) v *= cfg::DRAG_MAX_V / len;
        gDragged->SetLinearVelocity(v);
        gDragged->SetAwake(true);
    }
}

// ------------------------------------------------------------------ drawing (Wii Sports / early-2000s gel theme)
namespace theme {
constexpr u32 BG_TOP     = 0xF7FBFFFF;   // white
constexpr u32 BG_BOT     = 0xBEDDF3FF;   // sky blue
constexpr u32 WHITE      = 0xFFFFFFFF;
constexpr u32 SHADOW     = 0xC2D7E8FF;   // opaque soft shadow (no alpha overlap artefacts)
constexpr u32 OBJ_SHADOW = 0xB4C8DAFF;
constexpr u32 LINE       = 0xD0E2F0FF;
constexpr u32 CHIP       = 0xEEF5FBFF;
constexpr u32 TRACK      = 0xD6E4F0FF;
constexpr u32 BLUE       = 0x2FA3E6FF;
constexpr u32 BLUE_DK    = 0x1672B0FF;
constexpr u32 NAVY       = 0x145E92FF;
constexpr u32 GREEN      = 0x46C467FF;
constexpr u32 ORANGE     = 0xFF9F1FFF;
constexpr u32 GREY       = 0xA3B5C6FF;
constexpr u32 TEXT       = 0x56677AFF;
constexpr u32 TEXT_DK    = 0x34455AFF;
constexpr u32 WALL       = 0x3B78BDFF;
constexpr u32 WALL_HI    = 0x7DB4E8FF;
constexpr u32 WALL_LO    = 0x22528AFF;
constexpr u32 BOX_DK     = 0xD97A00FF;
constexpr u32 BOX        = 0xFFB032FF;
constexpr u32 BALL_DK    = 0xB8302AFF;
constexpr u32 BALL       = 0xF2584EFF;
constexpr u32 WHEEL      = 0x3C4752FF;
constexpr u32 POS        = 0x2FA85AFF;
constexpr u32 NEG        = 0xE04848FF;
} // namespace theme

static u32  gFrame     = 0;
static bool gShowRays  = true;
static bool gPaused    = false;
static bool gBuild     = false;      // wall-painting mode
static bool gErase     = false;      // paint mode erases instead

static u32 lerpColor(u32 a, u32 b, float t)
{
    u32 out = 0;
    for (int s = 24; s >= 0; s -= 8) {
        const float ca = (float)((a >> s) & 0xFF), cb = (float)((b >> s) & 0xFF);
        out |= ((u32)(ca + (cb - ca) * t) & 0xFF) << s;
    }
    return out;
}

static void drawTextShadow(float x, float y, int s, u32 color, u32 shadow, const char* str)
{
    drawText(x + (s >= 3 ? 2 : 1), y + (s >= 3 ? 2 : 1), s, shadow, str);
    drawText(x, y, s, color, str);
}

static float textW(const char* str, int s) { return (float)strlen(str) * 4.0f * s - (float)s; }

static void roundRect(float x, float y, float w, float h, float r, u32 c)
{
    if (r > h * 0.5f) r = h * 0.5f;
    if (r > w * 0.5f) r = w * 0.5f;
    GRRLIB_Rectangle(x + r, y, w - 2 * r, h, c, true);
    GRRLIB_Rectangle(x, y + r, w, h - 2 * r, c, true);
    GRRLIB_Circle(x + r,     y + r,     r, c, true);
    GRRLIB_Circle(x + w - r, y + r,     r, c, true);
    GRRLIB_Circle(x + r,     y + h - r, r, c, true);
    GRRLIB_Circle(x + w - r, y + h - r, r, c, true);
}

static void card(float x, float y, float w, float h, const char* title)
{
    roundRect(x + 2, y + 4, w, h, 12, theme::SHADOW);
    roundRect(x, y, w, h, 12, theme::WHITE);
    drawText(x + 10, y + 7, 2, theme::BLUE, title);
    GRRLIB_Rectangle(x + 8, y + 22, w - 16, 1.0f, theme::LINE, true);
}

static void pill(float x, float y, float w, float h, u32 color, const char* text, u32 textColor = 0xFFFFFFFF)
{
    roundRect(x, y + 2, w, h, h * 0.5f, theme::SHADOW);
    roundRect(x, y, w, h, h * 0.5f, color);
    GRRLIB_Rectangle(x + h * 0.4f, y + 2, w - h * 0.8f, h * 0.40f, 0xFFFFFF55, true);   // gloss
    drawText(x + (w - textW(text, 2)) * 0.5f, y + (h - 10.0f) * 0.5f, 2, textColor, text);
}

static void meter(float x, float y, float w, float h, float frac, u32 color)
{
    roundRect(x, y, w, h, h * 0.5f, theme::TRACK);
    roundRect(x, y, fmaxf(h, w * frac), h, h * 0.5f, color);
}

// early-2000s "gel" button: white bevel, tinted body, darker lower half, glossy top
static void gelButton(float x, float y, float w, float h, u32 color, bool hover, const char* label)
{
    if (hover) { x -= 3; y -= 3; w += 6; h += 6; color = lerpColor(color, theme::WHITE, 0.22f); }
    roundRect(x + 2, y + 4, w, h, 12, theme::SHADOW);
    roundRect(x, y, w, h, 12, theme::WHITE);
    roundRect(x + 3, y + 3, w - 6, h - 6, 9, color);
    GRRLIB_Rectangle(x + 8, y + h * 0.58f, w - 16, h * 0.34f, 0x00000020, true);        // lower shade
    GRRLIB_Rectangle(x + 9, y + 5, w - 18, h * 0.34f, 0xFFFFFF66, true);                // top gloss
    drawTextShadow(x + (w - textW(label, 2)) * 0.5f, y + (h - 10.0f) * 0.5f + 1, 2,
                   theme::WHITE, theme::NAVY, label);
}

static void drawBackground()
{
    for (int y = 0; y < cfg::SCREEN_H; y += 8)
        GRRLIB_Rectangle(0, (float)y, (float)cfg::SCREEN_W, 8.0f,
                         lerpColor(theme::BG_TOP, theme::BG_BOT, (float)y / cfg::SCREEN_H), true);
    for (int i = -cfg::SCREEN_H; i < cfg::SCREEN_W; i += 22)                      // diagonal pinstripes
        GRRLIB_Line((float)i, 0, (float)(i + cfg::SCREEN_H), (float)cfg::SCREEN_H, 0xFFFFFF35);
    GRRLIB_Circle(120, 420, 110, 0xFFFFFF28, true);                               // soft bokeh
    GRRLIB_Circle(560,  90,  80, 0xFFFFFF28, true);
    GRRLIB_Circle(610, 400,  60, 0xFFFFFF22, true);
}

static u32 rayColor(float norm)
{
    if (norm < 0.25f) return 0xF03A3AFF;     // danger  - red
    if (norm < 0.50f) return 0xFF8A1FFF;     // close   - orange
    if (norm < 0.75f) return 0xF2C200FF;     // near    - yellow
    return 0x2FBF5FFF;                       // clear   - green
}

// ---- world drawing: filled shapes via parallel lines (only GRRLIB_Line needed)
static void fillLocalRect(const b2Body* b, float x0, float x1, float y0, float y1, u32 col,
                          float ox = 0.0f, float oy = 0.0f)
{
    const int n = (int)((x1 - x0) * cfg::PPM * 1.5f) + 1;
    for (int k = 0; k <= n; ++k) {
        const float x = x0 + (x1 - x0) * (float)k / (float)n;
        const b2Vec2 a = b->GetWorldPoint(b2Vec2(x, y0));
        const b2Vec2 c = b->GetWorldPoint(b2Vec2(x, y1));
        GRRLIB_Line(sx(a.x) + ox, sy(a.y) + oy, sx(c.x) + ox, sy(c.y) + oy, col);
    }
}

static inline bool arenaContains(float px, float py)
{
    return px >= sx(-cfg::ARENA_W * 0.5f) && px <= sx(cfg::ARENA_W * 0.5f) &&
           py >= sy( cfg::ARENA_H * 0.5f) && py <= sy(-cfg::ARENA_H * 0.5f);
}

static void drawHand(float x, float y, bool grab)
{
    using namespace theme;
    if (grab) {                                            // closed fist
        GRRLIB_Circle(x + 2, y + 11, 11.0f, SHADOW, true);
        GRRLIB_Circle(x, y + 8, 11.0f, BLUE_DK, true);
        GRRLIB_Circle(x, y + 8,  9.0f, WHITE,   true);
        roundRect(x - 7, y + 1, 14, 5, 2.5f, 0xE0EBF5FF);
        return;
    }
    roundRect(x - 3,  y + 2, 10, 16, 5.0f, SHADOW);        // soft shadow
    roundRect(x - 5,  y - 1, 10, 16, 5.0f, BLUE_DK);       // index finger outline
    GRRLIB_Circle(x + 1, y + 16, 11.0f, BLUE_DK, true);    // palm outline
    GRRLIB_Circle(x - 9, y + 14,  5.5f, BLUE_DK, true);    // thumb outline
    roundRect(x - 3.5f, y + 1, 7, 14, 3.5f, WHITE);        // index finger
    GRRLIB_Circle(x + 1, y + 16, 9.0f, WHITE, true);       // palm
    GRRLIB_Circle(x - 9, y + 14, 3.8f, WHITE, true);       // thumb
}

// bullseye + checkered flag
static void drawGoal(float cx, float cy, bool ghost)
{
    using namespace theme;
    const u32 a = ghost ? 0x70 : 0xFF;
    if (!ghost) GRRLIB_Circle(cx + 3, cy + 4, 15.0f, OBJ_SHADOW, true);
    GRRLIB_Circle(cx, cy, 15.0f, 0x2FBF5F00 | a, true);
    GRRLIB_Circle(cx, cy, 11.0f, 0xFFFFFF00 | a, true);
    GRRLIB_Circle(cx, cy,  7.0f, 0x2FBF5F00 | a, true);
    GRRLIB_Circle(cx, cy,  3.0f, 0xFFFFFF00 | a, true);
    GRRLIB_Line(cx, cy, cx, cy - 26, 0x56677A00 | a);                       // pole
    GRRLIB_Line(cx + 1, cy, cx + 1, cy - 26, 0x56677A00 | a);
    for (int ry = 0; ry < 3; ++ry)                                          // 4 x 3 checker flag
        for (int rx = 0; rx < 4; ++rx)
            GRRLIB_Rectangle(cx + 2 + rx * 3.5f, cy - 26 + ry * 3.5f, 3.5f, 3.5f,
                             ((rx + ry) & 1) ? (0xFFFFFF00 | a) : (0x23303C00 | a), true);
}

static void drawWorld()
{
    using namespace theme;
    drawBackground();

    const float ax = sx(-cfg::ARENA_W * 0.5f), ay = sy(cfg::ARENA_H * 0.5f);

    // checkered floor
    for (int gy = 0; gy < (int)cfg::ARENA_H; ++gy)
        for (int gx = 0; gx < (int)cfg::ARENA_W; ++gx)
            GRRLIB_Rectangle(ax + gx * cfg::PPM, ay + gy * cfg::PPM, cfg::PPM, cfg::PPM,
                             ((gx + gy) & 1) ? 0xE8F2FAFF : 0xF9FCFFFF, true);

    // walls: shadows, then blocks with lit top / dark bottom edge
    for (int i = 0; i < kNumWalls; ++i) {
        const float wx = sx(kWalls[i].cx - kWalls[i].hw), wy = sy(kWalls[i].cy + kWalls[i].hh);
        GRRLIB_Rectangle(wx + 3, wy + 4, 2 * kWalls[i].hw * cfg::PPM, 2 * kWalls[i].hh * cfg::PPM, OBJ_SHADOW, true);
    }
    for (int i = 0; i < kNumWalls; ++i) {
        const float wx = sx(kWalls[i].cx - kWalls[i].hw), wy = sy(kWalls[i].cy + kWalls[i].hh);
        const float ww = 2 * kWalls[i].hw * cfg::PPM,     wh = 2 * kWalls[i].hh * cfg::PPM;
        GRRLIB_Rectangle(wx, wy, ww, wh, WALL, true);
        GRRLIB_Rectangle(wx, wy, ww, 2.0f, WALL_HI, true);
        GRRLIB_Rectangle(wx, wy + wh - 2.0f, ww, 2.0f, WALL_LO, true);
    }

    // user-built walls: shadows (run-merged per row), then blocks with lit top / dark bottom edges
    if (gWallCount > 0) {
        const float cs = grid::CELL * cfg::PPM;
        for (int r = 0; r < grid::ROWS; ++r)
            for (int c = 0; c < grid::COLS; ++c) {
                if (!gCellOn[r][c]) continue;
                int c1 = c; while (c1 + 1 < grid::COLS && gCellOn[r][c1 + 1]) ++c1;
                GRRLIB_Rectangle(sx(grid::X0 + c * grid::CELL) + 3, sy(grid::Y0 + (r + 1) * grid::CELL) + 4,
                                 (c1 - c + 1) * cs, cs, OBJ_SHADOW, true);
                c = c1;
            }
        for (int r = 0; r < grid::ROWS; ++r)
            for (int c = 0; c < grid::COLS; ++c) {
                if (!gCellOn[r][c]) continue;
                int c1 = c; while (c1 + 1 < grid::COLS && gCellOn[r][c1 + 1]) ++c1;
                const float x0 = sx(grid::X0 + c * grid::CELL), y0 = sy(grid::Y0 + (r + 1) * grid::CELL);
                GRRLIB_Rectangle(x0, y0, (c1 - c + 1) * cs, cs, WALL, true);
                for (int k = c; k <= c1; ++k) {
                    const float xk = sx(grid::X0 + k * grid::CELL);
                    if (!cellOn(k, r + 1)) GRRLIB_Rectangle(xk, y0, cs, 2.0f, WALL_HI, true);
                    if (!cellOn(k, r - 1)) GRRLIB_Rectangle(xk, y0 + cs - 2.0f, cs, 2.0f, WALL_LO, true);
                }
                c = c1;
            }
    }

    if (gGoalSet) drawGoal(sx(gGoalPos.x), sy(gGoalPos.y), false);

    // props: glossy orange crates and red balls, each with a drop shadow
    for (const Prop& p : gProps) {
        if (!p.active) continue;
        const b2Vec2 pos = p.body->GetPosition();
        const float cx = sx(pos.x), cy = sy(pos.y);
        if (p.circle) {
            const float r = p.size * cfg::PPM;
            GRRLIB_Circle(cx + 3, cy + 4, r, OBJ_SHADOW, true);
            GRRLIB_Circle(cx, cy, r, BALL_DK, true);
            GRRLIB_Circle(cx, cy, r - 2.0f, BALL, true);
            GRRLIB_Circle(cx - r * 0.3f, cy - r * 0.35f, r * 0.28f, 0xFFFFFFA0, true);   // specular dot
            const b2Vec2 e = p.body->GetWorldPoint(b2Vec2(p.size, 0));
            GRRLIB_Line(cx, cy, sx(e.x), sy(e.y), BALL_DK);                              // spin marker
        } else {
            fillLocalRect(p.body, -p.size, p.size, -p.size, p.size, OBJ_SHADOW, 3, 4);
            fillLocalRect(p.body, -p.size, p.size, -p.size, p.size, BOX_DK);
            const float in = p.size * 0.78f;
            fillLocalRect(p.body, -in, in, -in, in, BOX);
        }
    }

    // sensor rays (double width) + hit dots
    if (gShowRays) {
        for (int i = 0; i < cfg::N_SENS; ++i) {
            const u32 col = rayColor(gRobot.sensNorm[i]);
            const float x0 = sx(gRobot.rayA[i].x), y0 = sy(gRobot.rayA[i].y);
            const float x1 = sx(gRobot.rayB[i].x), y1 = sy(gRobot.rayB[i].y);
            GRRLIB_Line(x0, y0, x1, y1, col);
            GRRLIB_Line(x0 + 1, y0, x1 + 1, y1, col);
            if (gRobot.sensNorm[i] < 1.0f) {
                GRRLIB_Circle(x1, y1, 4.5f, WHITE, true);
                GRRLIB_Circle(x1, y1, 3.0f, col, true);
            }
        }
    }

    // robot: shadow, wheels, bevelled hull, nose bar, cartoon eyes
    b2Body* rb = gRobot.body;
    const float hl = cfg::ROBOT_HL, hw = cfg::ROBOT_HW;
    fillLocalRect(rb, -hl, hl, -hw, hw, OBJ_SHADOW, 3, 4);
    for (int i = 0; i < 2; ++i) {
        const float side = (i == 0) ? 1.0f : -1.0f;
        fillLocalRect(rb, -0.11f, 0.11f, side * hw, side * (hw + 0.06f), gRobot.vel[i] >= -0.05f ? WHEEL : NEG);
    }
    fillLocalRect(rb, -hl, hl, -hw, hw, BLUE_DK);
    fillLocalRect(rb, -hl + 0.04f, hl - 0.04f, -hw + 0.04f, hw - 0.04f, BLUE);
    fillLocalRect(rb, hl - 0.09f, hl - 0.04f, -hw + 0.07f, hw - 0.07f, WHITE);
    for (int i = 0; i < 2; ++i) {
        const float side = (i == 0) ? 1.0f : -1.0f;
        const b2Vec2 e = rb->GetWorldPoint(b2Vec2(0.08f, side * 0.085f));
        const b2Vec2 pu = rb->GetWorldPoint(b2Vec2(0.115f, side * 0.085f));
        GRRLIB_Circle(sx(e.x), sy(e.y), 4.0f, WHITE, true);
        GRRLIB_Circle(sx(pu.x), sy(pu.y), 2.0f, 0x23303CFF, true);
    }

    if (gPaused) pill(sx(0) - 60, sy(0) - 14, 120, 28, ORANGE, "PAUSED");
    if (gGoalBanner > 0) {
        static char gb[24];
        snprintf(gb, sizeof gb, "GOAL! %.1fS", gGoalLastTime);
        pill(sx(0) - 75, sy(cfg::ARENA_H * 0.5f) + 10, 150, 28, GREEN, gb);
    }
}

// ---- side menu buttons (clickable with the Wiimote pointer + A)
struct Btn { float x, y, w, h; };
constexpr int kNumBtns = 10;
static const Btn kBtns[kNumBtns] = {
    {  8, 293, 82, 26 }, { 96, 293, 82, 26 },     // RESET   PAUSE
    {  8, 323, 82, 26 }, { 96, 323, 82, 26 },     // RAYS    CLEAR (props)
    {  8, 353, 82, 26 }, { 96, 353, 82, 26 },     // BUILD   ERASE
    {  8, 383, 82, 26 }, { 96, 383, 82, 26 },     // MAZE    NO WALLS
    {  8, 413, 82, 26 }, { 96, 413, 82, 26 },     // GOAL    SEEK
};

static int uiButtonAt(float px, float py)
{
    for (int i = 0; i < kNumBtns; ++i)
        if (px >= kBtns[i].x && px <= kBtns[i].x + kBtns[i].w &&
            py >= kBtns[i].y && py <= kBtns[i].y + kBtns[i].h) return i;
    return -1;
}

static void uiPress(int i)
{
    switch (i) {
        case 0: resetRobot();                          break;
        case 1: gPaused   = !gPaused;                  break;
        case 2: gShowRays = !gShowRays;                break;
        case 3: clearProps();                          break;
        case 4: gBuild = !gBuild; if (gBuild) gGoalMode = false;                          break;
        case 5: gErase = !gErase; if (gErase) { gBuild = true; gGoalMode = false; }       break;
        case 6: generateMaze();                        break;
        case 7: clearWalls();                          break;
        case 8: gGoalMode = !gGoalMode; if (gGoalMode) { gBuild = false; gErase = false; } break;
        case 9: gSeek = !gSeek;                        break;
    }
}

static void footerItem(float& x, const char* badge, const char* label)
{
    const float bw = (float)strlen(badge) * 8.0f + 14.0f;
    pill(x, 454, bw, 20, theme::BLUE, badge);
    x += bw + 6.0f;
    drawText(x, 459, 2, theme::TEXT, label);
    x += textW(label, 2) + 18.0f;
}

static void drawUI(float fps, const Input& in)
{
    using namespace theme;
    static char buf[32];                     // static: no heap in the loop
    static const char* kSensLabel[cfg::N_SENS] = { "R45", "R22", "FWD", "L22", "L45" };
    const int hover = in.irValid ? uiButtonAt(in.cx, in.cy) : -1;

    // ---- header: glossy blue gel bar
    for (int y = 0; y < 42; y += 3)
        GRRLIB_Rectangle(0, (float)y, (float)cfg::SCREEN_W, 3.0f, lerpColor(0x62CDF8FF, 0x1C8CD4FF, y / 42.0f), true);
    GRRLIB_Rectangle(0, 0, (float)cfg::SCREEN_W, 20.0f, 0xFFFFFF30, true);
    GRRLIB_Rectangle(0, 42, (float)cfg::SCREEN_W, 2.0f, BLUE_DK, true);
    GRRLIB_Rectangle(0, 44, (float)cfg::SCREEN_W, 4.0f, SHADOW, true);
    drawTextShadow(16, 13, 3, WHITE, NAVY, "WIIBOX");
    drawTextShadow(100, 21, 2, 0xE6F6FFFF, NAVY, "ROBOT LAB");
    snprintf(buf, sizeof buf, "%.1f FPS", fps);
    pill(cfg::SCREEN_W - 16 - 100, 9, 100, 24, WHITE, buf, BLUE_DK);
    if (gGoalSet) {
        snprintf(buf, sizeof buf, "GOALS %d  %.1fS", gGoalCount,
                 (gGoalReached ? gGoalLastTime : gGoalFrames / 60.0f));
        pill(245, 9, 150, 24, WHITE, buf, BLUE_DK);
    }

    // ---- side menu: STATUS
    const float px = 8, pw = 170;
    const bool recov = (gUn.state != Unstick::IDLE);
    const char* modeTxt = gRobot.manual ? "MANUAL" : recov ? "BACKING UP"
                        : (gGoalSet && gSeek) ? (gGoalReached ? "GOAL REACHED" : "SEEKING GOAL") : "NEURAL NET";
    const u32   modeCol = gRobot.manual ? ORANGE : (recov ? 0xF2C200FF : GREEN);
    card(px, 54, pw, 50, "STATUS");
    if (in.nunchuk) pill(px + pw - 46, 54 + 5, 34, 15, GREY, "NC");
    pill(px + 10, 54 + 26, pw - 20, 20, modeCol, modeTxt, recov ? NAVY : WHITE);

    // ---- SENSORS
    card(px, 108, pw, 113, "SENSORS (M)");
    for (int i = 0; i < cfg::N_SENS; ++i) {
        const float y = 108 + 27 + i * 17.0f;
        drawText(px + 8, y, 2, TEXT, kSensLabel[i]);
        meter(px + 36, y + 1, 70, 8, gRobot.sensNorm[i], rayColor(gRobot.sensNorm[i]));
        snprintf(buf, sizeof buf, "%.2f", gRobot.sensDist[i]);
        drawText(px + 112, y, 2, TEXT_DK, buf);
    }

    // ---- WHEELS (centred bidirectional bars, m/s)
    card(px, 225, pw, 62, "WHEELS (M/S)");
    for (int i = 0; i < 2; ++i) {
        const float y = 225 + 27 + i * 17.0f;
        const float v = gRobot.vel[i];
        const float frac = fmaxf(-1.0f, fminf(1.0f, v / cfg::MAX_SPEED));
        drawText(px + 8, y, 2, TEXT, i == 0 ? "L" : "R");
        roundRect(px + 24, y + 1, 96, 8, 4, TRACK);
        const float cxm = px + 24 + 48;
        const float len = 48.0f * fabsf(frac);
        GRRLIB_Rectangle(frac >= 0 ? cxm : cxm - len, y + 1, fmaxf(len, 1.0f), 8.0f, frac >= 0 ? POS : NEG, true);
        GRRLIB_Rectangle(cxm, y - 1, 1.0f, 12.0f, TEXT, true);
        snprintf(buf, sizeof buf, "%+.1f", v);
        drawText(px + 126, y, 2, v >= 0 ? POS : NEG, buf);
    }

    // ---- buttons
    const char* lbl[kNumBtns] = { "RESET", gPaused ? "RESUME" : "PAUSE", gShowRays ? "RAYS ON" : "RAYS OFF", "CLEAR",
                                  gBuild ? "BUILD ON" : "BUILD", gErase ? "ERASE ON" : "ERASE", "MAZE", "NO WALLS",
                                  gGoalMode ? "SET GOAL" : "GOAL", gSeek ? "SEEK ON" : "SEEK OFF" };
    const u32 col[kNumBtns]  = { BLUE, gPaused ? GREEN : 0xE8645AFF, gShowRays ? GREEN : GREY, ORANGE,
                                 gBuild ? GREEN : BLUE, gErase ? 0xE8645AFF : GREY, 0x8E6CE0FF, 0x6F7F90FF,
                                 gGoalMode ? GREEN : 0x1FB5A8FF, gSeek ? GREEN : GREY };
    for (int i = 0; i < kNumBtns; ++i)
        gelButton(kBtns[i].x, kBtns[i].y, kBtns[i].w, kBtns[i].h, col[i], hover == i, lbl[i]);

    // ---- ray legend under the arena
    static const struct { u32 col; const char* name; } kLegend[4] = {
        { 0x2FBF5FFF, "CLEAR" }, { 0xF2C200FF, "NEAR" }, { 0xFF8A1FFF, "CLOSE" }, { 0xF03A3AFF, "DANGER" } };
    float lx = sx(-cfg::ARENA_W * 0.5f);
    for (int i = 0; i < 4; ++i) {
        GRRLIB_Circle(lx + 5, 417, 6.0f, WHITE, true);
        GRRLIB_Circle(lx + 5, 417, 4.5f, kLegend[i].col, true);
        drawText(lx + 16, 412, 2, TEXT, kLegend[i].name);
        lx += 16 + textW(kLegend[i].name, 2) + 16;
    }

    // ---- build mode: ghost cell under the pointer + banner
    if (gBuild) {
        if (in.irValid && arenaContains(in.cx, in.cy)) {
            int gc, gr;
            cellAt(toWorld(in.cx, in.cy), gc, gr);
            const float cs = grid::CELL * cfg::PPM;
            const float gx = sx(grid::X0 + gc * grid::CELL), gy = sy(grid::Y0 + (gr + 1) * grid::CELL);
            const bool er = gErase || (in.held & WPAD_BUTTON_B) != 0 || cellOn(gc, gr);
            const u32 gcol = er ? NEG : BLUE;
            GRRLIB_Rectangle(gx, gy, cs, cs, (gcol & 0xFFFFFF00) | 0x60, true);
            GRRLIB_Rectangle(gx, gy, cs, cs, gcol, false);
        }
        pill(516, 405, 114, 22, gErase ? NEG : BLUE, gErase ? "ERASING" : "BUILD MODE");
    }

    // ---- goal placement: ghost flag on the hovered cell + banner
    if (gGoalMode) {
        if (in.irValid && arenaContains(in.cx, in.cy)) {
            int gc, gr;
            cellAt(toWorld(in.cx, in.cy), gc, gr);
            drawGoal(sx(grid::X0 + (gc + 0.5f) * grid::CELL), sy(grid::Y0 + (gr + 0.5f) * grid::CELL), true);
        }
        pill(516, 405, 114, 22, GREEN, "PLACE GOAL");
    }

    // ---- footer: button prompts
    GRRLIB_Rectangle(0, 444, (float)cfg::SCREEN_W, 4.0f, SHADOW, true);
    GRRLIB_Rectangle(0, 448, (float)cfg::SCREEN_W, 32.0f, WHITE, true);
    GRRLIB_Rectangle(0, 448, (float)cfg::SCREEN_W, 1.0f, LINE, true);
    float fx = 12;
    footerItem(fx, "A", gGoalMode ? "SET GOAL" : gBuild ? "PAINT" : "SPAWN/DRAG");
    footerItem(fx, "B", gGoalMode ? "REMOVE" : gBuild ? "ERASE" : "DRIVE");
    footerItem(fx, "1", "RAYS");
    footerItem(fx, "2", "PAUSE");
    footerItem(fx, "HOME", "EXIT");
    drawText(cfg::SCREEN_W - 12 - textW("MLP 5-8-2", 2), 459, 2, GREY, "MLP 5-8-2");

    // ---- pointer on top of everything
    if (in.irValid) drawHand(in.cx, in.cy, (in.held & WPAD_BUTTON_A) != 0);
}

// Click GOAL, then A on the map drops the flag on that cell (mode ends). B removes the flag.
static void handleGoal(const Input& in, bool allowStart)
{
    if (in.down & WPAD_BUTTON_B) { gGoalSet = false; gGoalReached = false; gGoalMode = false; return; }
    if (!allowStart || !(in.down & WPAD_BUTTON_A)) return;
    int c, r;
    cellAt(toWorld(in.cx, in.cy), c, r);
    if (c >= 0 && c < grid::COLS && r >= 0 && r < grid::ROWS && !cellOn(c, r)) {
        setGoal(c, r);
        gGoalMode = false;
    }
}

// ---- command shaping for all automatic driving (net, planner blend)
// Raw net outputs are near full-scale and make the rover swing past the wall it is avoiding, so:
//  * yaw-rate damping brakes the turn as it speeds up (stops the overshoot),
//  * the turn command is capped,
//  * forward speed is limited when something is close ahead (gentle contact),
//  * both are low-pass filtered so corrections are small and smooth.
static float gFwdCmd = 0.0f, gTurnCmd = 0.0f;       // filtered command (+turn = counter-clockwise)

static void shaperSync(float cl, float cr) { gFwdCmd = 0.5f * (cl + cr); gTurnCmd = 0.5f * (cr - cl); }

static void shapeCommand(float& cl, float& cr)
{
    float f = 0.5f * (cl + cr);
    float t = 0.5f * (cr - cl);
    t -= 0.15f * gRobot.body->GetAngularVelocity();                       // yaw-rate damping
    t = fmaxf(-0.55f, fminf(0.55f, t));
    const float front = fminf(gRobot.sensNorm[1], fminf(gRobot.sensNorm[2], gRobot.sensNorm[3]));
    if (front < 0.3f) f = fminf(f, 0.2f + front);                         // ease off near walls
    gFwdCmd  += 0.20f * (f - gFwdCmd);
    gTurnCmd += 0.25f * (t - gTurnCmd);
    cl = fmaxf(-1.0f, fminf(1.0f, gFwdCmd - gTurnCmd));
    cr = fmaxf(-1.0f, fminf(1.0f, gFwdCmd + gTurnCmd));
}

// stuck recovery runs unfiltered (it is already gentle); everything else gets shaped
static void finishCommand(float& cl, float& cr)
{
    if (unstickStep(cl, cr)) shaperSync(cl, cr);
    else                     shapeCommand(cl, cr);
}

// Goal seeking: blend the planner's steering with the neural net's output (the net takes over
// when something is close ahead), stop at the flag, and always keep stuck-recovery active.
static void goalControl(float& cl, float& cr)
{
    if (!(gGoalSet && gSeek)) { finishCommand(cl, cr); return; }
    if (gGoalReached) { cl = cr = 0.0f; shaperSync(0.0f, 0.0f); unstickReset(); return; }

    if ((gRobot.body->GetPosition() - gGoalPos).Length() < 0.6f) {
        gGoalReached = true; ++gGoalCount;
        gGoalLastTime = gGoalFrames / 60.0f; gGoalBanner = 180;
        cl = cr = 0.0f; shaperSync(0.0f, 0.0f);
        return;
    }
    ++gGoalFrames;
    if (--gPlanTimer <= 0) { replan(); gPlanTimer = 10; }

    float nl, nr;
    if (navCommand(nl, nr)) {
        const float m = fminf(gRobot.sensNorm[1], fminf(gRobot.sensNorm[2], gRobot.sensNorm[3]));
        const float avoid = fmaxf(0.0f, fminf(1.0f, (0.40f - m) / 0.25f));   // 0 = path, 1 = net
        cl = (1.0f - avoid) * nl + avoid * cl;
        cr = (1.0f - avoid) * nr + avoid * cr;
    }
    finishCommand(cl, cr);
}

// Paint/erase walls by dragging with A. The stroke type is chosen when A goes down:
// erase if ERASE is on, B is held, or the first cell is already a wall.
static int  gStrokeC = -1, gStrokeR = -1;
static bool gStrokeErase = false;

static void handleBuild(const Input& in, bool allowStart)
{
    if (!(in.held & WPAD_BUTTON_A) || !in.irValid) { gStrokeC = -1; return; }
    int c, r;
    cellAt(toWorld(in.cx, in.cy), c, r);
    if (gStrokeC < 0) {
        if (!allowStart || !(in.down & WPAD_BUTTON_A)) return;
        gStrokeErase = gErase || (in.held & WPAD_BUTTON_B) != 0 || cellOn(c, r);
        gStrokeC = c; gStrokeR = r;
        paintLine(c, r, c, r, !gStrokeErase);
        return;
    }
    if (c != gStrokeC || r != gStrokeR) {
        paintLine(gStrokeC, gStrokeR, c, r, !gStrokeErase);
        gStrokeC = c; gStrokeR = r;
    }
}

// ------------------------------------------------------------------ main
int main(int, char**)
{
    if (GRRLIB_Init() != 0) return 1;
    WPAD_Init();
    WPAD_SetDataFormat(WPAD_CHAN_0, WPAD_FMT_BTNS_ACC_IR);
    WPAD_SetVRes(WPAD_CHAN_0, cfg::SCREEN_W, cfg::SCREEN_H);
    SYS_SetResetCallback(onReset);
    SYS_SetPowerCallback(onPower);

    static b2World world(b2Vec2(0.0f, 0.0f));   // top-down: no gravity
    gWorld = &world;
    buildArena();
    buildRobot();
    buildPropPool();
    buildCellPool();
    loadDefaultWalls();
    unstickReset();

    Input in;
    float fps = 60.0f;
    u64 tLast = gettime();
    int frames = 0;

    while (!gQuit) {
        readInput(in);
        if (in.down & WPAD_BUTTON_HOME)  break;
        if (in.down & WPAD_BUTTON_1)     gShowRays = !gShowRays;
        if (in.down & WPAD_BUTTON_2)     gPaused   = !gPaused;
        if (in.down & WPAD_BUTTON_MINUS) clearProps();
        if (in.down & WPAD_BUTTON_PLUS)  resetRobot();

        if (gBuild || gGoalMode) in.manual = in.stick;          // in build mode B erases instead of driving

        // menu buttons first; a click on the menu must not spawn anything in the arena
        bool uiClick = false;
        if ((in.down & WPAD_BUTTON_A) && in.irValid) {
            const int b = uiButtonAt(in.cx, in.cy);
            if (b >= 0) { uiPress(b); uiClick = true; }
        }
        const bool inside = in.irValid && !uiClick && arenaContains(in.cx, in.cy);
        if (gGoalMode) handleGoal(in, inside);
        else if (gBuild) handleBuild(in, inside);
        else handleDrag(in, inside);

        // ---- perception -> inference -> actuation
        updateSensors();
        gRobot.manual = in.manual;
        if (!gPaused) {
            if (in.manual) {
                gRobot.cmd[0] = in.manL; gRobot.cmd[1] = in.manR;
                shaperSync(in.manL, in.manR);
                unstickReset();
            } else {
                float out[weights::N_OUT];
                nn::forward<weights::N_IN, weights::N_HID, weights::N_OUT>(
                    gRobot.sensNorm, weights::W1, weights::B1, weights::W2, weights::B2, out);
                gRobot.cmd[0] = out[0]; gRobot.cmd[1] = out[1];
                goalControl(gRobot.cmd[0], gRobot.cmd[1]);
            }
            driveWheels(gRobot.cmd[0], gRobot.cmd[1]);
            world.Step(cfg::DT, cfg::VEL_ITERS, cfg::POS_ITERS);
            if (gGoalBanner > 0) --gGoalBanner;
        }

        // ---- render (GRRLIB_Render blocks on vsync => 60 FPS lock)
        drawWorld();
        drawUI(fps, in);
        GRRLIB_Render();
        ++gFrame;

        if (++frames == 30) {
            const u64 now = gettime();
            fps = 30.0f * 1000000.0f / (float)ticks_to_microsecs(now - tLast);
            tLast = now; frames = 0;
        }
    }

    GRRLIB_Exit();
    return 0;
}
