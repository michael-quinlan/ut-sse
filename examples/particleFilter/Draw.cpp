// draws the graphical interface

#include <stdio.h>

#include "GL/glut.h"

#include "pf.h"


//--- COMPILE-TIME OPTIONS ---//

// color scheme, set to 0 (normal) or 1 (inverted)
#define INVERTED_COLORS      0


//--- CONSTANTS ---//

static const int INIT_WIDTH  = 512;
static const int INIT_HEIGHT = 512;

static const int INIT_X_POS = 400;
static const int INIT_Y_POS = 30;

static const char *WINDOW_NAME = "pf";

// particles with a color intensity below this threshold are not drawn,
// if color inversion is enabled this is the threshold pre-inversion
static const float PARTICLE_COLOR_THRESHOLD = 0.01f;

// in RGB
#if !INVERTED_COLORS
// normal colors
static const float BG_COLOR[]      = { 0.0f, 0.0f, 0.0f };		// black
static const float ROBOT_COLOR[]   = { 0.0f, 0.0f, 1.0f };		// blue
static const float REF_OBJ_COLOR[] = { 1.0f, 1.0f, 0.0f };		// yellow
static const float OBS_COLOR[]     = { 0.5f, 0.5f, 0.0f };		// light yellow
static const float GRASS_COLOR[]   = { 1.0f, 1.0f, 1.0f };		// white
static const float ESTPOSE_COLOR[] = { 0.0f, 1.0f, 0.0f };		// green
static const float PARTLEG_COLOR[] = { 1.0f, 1.0f, 1.0f };		// white
static const float TEXT_COLOR[]    = { 0.8f, 0.8f, 0.8f };		// gray
#else
// inverted colors
static const float BG_COLOR[]      = { 1.0f, 1.0f, 1.0f };
static const float ROBOT_COLOR[]   = { 0.0f, 0.0f, 1.0f };
static const float REF_OBJ_COLOR[] = { 0.7f, 0.7f, 0.0f };
static const float OBS_COLOR[]     = { 0.5f, 0.5f, 0.2f };
static const float GRASS_COLOR[]   = { 0.1f, 0.1f, 0.1f };
static const float ESTPOSE_COLOR[] = { 0.0f, 0.7f, 0.0f };
static const float PARTLEG_COLOR[] = { 0.1f, 0.1f, 0.1f };
static const float TEXT_COLOR[]    = { 0.2f, 0.2f, 0.2f };
#endif

// the different ways to display the results of the particle filter
enum SimilarityDisplayMode {
	SD_DISTANCE_AND_BEARING,
	SD_DISTANCE,
	SD_BEARING,
	SD_ALL_PARTICLES,
	NUM_SD_MODES
};

const char *SD_MODE_STRINGS[] = {
	"distance and bearing",
	"distance",
	"bearing",
	"all particles",
	"unknown sd mode"
};


//--- LOADED FROM EXTERNAL MODULE ---//

static Rectangle field;
static Rectangle grass;
static RobotPose actualPose;
static Observation *observations;
static ObservationWindow *obsWindow;
static const Point2D *refObjs;
static int numRefObjs;
static ParticleArray particles;
static ParticleArray_4Wide particles_4Wide;


//--- GLOBALS ---//

// window ID assigned by the window manager
static int windowID = 0;

// window dimensions
static int windowWidth = INIT_WIDTH;
static int windowHeight = INIT_HEIGHT;

// how to display the particles
static SimilarityDisplayMode sdMode = SD_DISTANCE_AND_BEARING;

// ratio of worldspace distance per unit pixel coordinate
static float worldPerWindow = 0.0f;

// estimated pose from the last invocation of the particle filter
static RobotPose estPose;

// fps from the last invocation of the particle filter
static float pfFps;


//--- FUNCTIONS ---//

static forceinline
float alwaysZero(const ProbabilityExponents &pe) {
	(void)pe;
	return 0.0f;
}

// cycles through the display modes round-robin
static
SimilarityDisplayMode getNext(SimilarityDisplayMode mode) {
	return (SimilarityDisplayMode)((mode + 1) % NUM_SD_MODES);
}

static
const char *toString(SimilarityDisplayMode mode) {
	assert(mode >= 0 && mode < NUM_SD_MODES);
	return SD_MODE_STRINGS[mode];
}

// updates all globals that depend on the window size,
// also sets the orthographic projection
static
void updateWindowSizeState(int w, int h) {
	windowWidth = w;
	windowHeight = h;

	// find a square viewing portal that comfortably fits the grass's dimensions
	// 1) grow by 20%
	// 2) cut in half to make it symmetric about the origin
	float maxWorldCoord = max(grass.getWidth(), grass.getHeight()) * (1.2f * 0.5f);
	worldPerWindow = maxWorldCoord / min(windowWidth, windowHeight);

	// visible part of the world
	float x = worldPerWindow * windowWidth;
	float y = worldPerWindow * windowHeight;

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glOrtho(-x, x, -y, y, 0, 1);
}

// perform one-time initialization
static
void oneTimeInit() {
	static bool init = false;

	if (init) {
		return;
	}
	init = true;

	// load all static particle filter state
	field = getField();
	grass = getGrass();
	actualPose = getActualPose();
	observations = getObservations();
	obsWindow = getObservationWindow();
	refObjs = getReferenceObjects();
	numRefObjs = getNumReferenceObjects();
	particles = getParticles();
	particles_4Wide = getParticles_4Wide();

	updateWindowSizeState(windowWidth, windowHeight);

	glDisable(GL_DEPTH);				// no depth buffer needed in 2D

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	glClearColor(BG_COLOR[0], BG_COLOR[1], BG_COLOR[2], 1.0f);
}

// given a length, an x, y that is the vector origin and an angle ang
// that is the orientation of the vector, draws a line representing the vector
static forceinline
void drawVector(float len, float x, float y, AngRad ang, float r, float g, float b) {
	float dx = cosf(ang) * len;
	float dy = sinf(ang) * len;

	glColor3f(r, g, b);
	glBegin(GL_LINES);
		glVertex2f(x, y);
		glVertex2f(x+dx, y+dy);
	glEnd();
}

static forceinline
void drawShortVector(float x, float y, AngRad ang, float r, float g, float b) {
	drawVector(300.0f, x, y, ang, r, g, b);
}

static forceinline
void drawMidVector(float x, float y, AngRad ang, float r, float g, float b) {
	drawVector(400.0f, x, y, ang, r, g, b);
}

static forceinline
void drawLongVector(float x, float y, AngRad ang, float r, float g, float b) {
	drawVector(800.0f, x, y, ang, r, g, b);
}

static forceinline
void drawPoint(float size, float x, float y, float r, float g, float b) {
	glPointSize(size);
	glColor3f(r, g, b);
	glBegin(GL_POINTS);
		glVertex2f(x, y);
	glEnd();
}

static forceinline
void drawBigPoint(float x, float y, float r, float g, float b) {
	drawPoint(5.0f, x, y, r, g, b);
}

static forceinline
void drawSmallPoint(float x, float y, float r, float g, float b) {
	drawPoint(3.0f, x, y, r, g, b);
}

static 
void drawCircle(float x, float y, float rad, float r, float g, float b) {
	static const int NUM_CIRCLE_SEGMENTS = 40;
	static const float ANG_INC = 2.0f * M_PI / NUM_CIRCLE_SEGMENTS;

	glColor3f(r, g, b);
	glBegin(GL_LINE_LOOP);

	for (int i = 0; i < NUM_CIRCLE_SEGMENTS; i++) {
		float ang = i * ANG_INC;
		glVertex2f(rad * cosf(ang) + x, rad * sinf(ang) + y);
	}

	glEnd();
}

static 
void drawEllipse(float x, float y, float x_len, float y_len,
				 float r, float g, float b)
{
	static const int NUM_ELLIPSE_SEGMENTS = 40;
	static const float ANG_INC = 2.0f * M_PI / NUM_ELLIPSE_SEGMENTS;

	glColor3f(r, g, b);
	glBegin(GL_LINE_LOOP);

	for (int i = 0; i < NUM_ELLIPSE_SEGMENTS; i++) {
		float ang = i * ANG_INC;
		glVertex2f(x_len * cosf(ang) + x, y_len * sinf(ang) + y);
	}

	glEnd();
}

// a scissor window is set up around the grass
static
void enableGrassScissor() {
	// assumes the grass is symmetric and centered at the origin
	Point2D p = grass.getTopRight();

	// in pixel coordinates
	int grass_w = (int)(p.x / worldPerWindow);
	int grass_h = (int)(p.y / worldPerWindow);
	int x_off = (int)((windowWidth - grass_w) * 0.5f);
	int y_off = (int)((windowHeight - grass_h) * 0.5f);
	glScissor(x_off, y_off, grass_w, grass_h);

	glEnable(GL_SCISSOR_TEST);
}

// turns off the grass scissor window
static
void disableGrassScissor() {
	glDisable(GL_SCISSOR_TEST);
}

static
void drawGrass() {
	glColor3f(GRASS_COLOR[0], GRASS_COLOR[1], GRASS_COLOR[2]);

	Point2D bl = grass.getBottomLeft();
	Point2D tr = grass.getTopRight();

	glBegin(GL_LINE_LOOP);
		glVertex2f(bl.x, bl.y);
		glVertex2f(tr.x, bl.y);
		glVertex2f(tr.x, tr.y);
		glVertex2f(bl.x, tr.y);
	glEnd();
}

static
void drawParticlesScalar(float (*peFunc)(const ProbabilityExponents &pe)) {
	// draw all of the particle locations as points,
	// draw all of the particle directions as vectors
	for (int i = 0; i < particles.n; i++) {
		float x = peFunc(particles.e[i]);
		float color = exp(x);

		if (color < PARTICLE_COLOR_THRESHOLD) {
			continue;
		}
#if INVERTED_COLORS
		color = 1.0f - color;
#endif

		Particle &p = particles.p[i];
		Point2D pos = p.pos;
		AngRad  ang = p.ang;

		drawMidVector(pos.x, pos.y, ang, color, color, color);
		drawSmallPoint(pos.x, pos.y, color, color, color);
	}
}

static
void drawParticlesSse(float (*peFunc)(const ProbabilityExponents &pe)) {
	for (int i = 0; i < particles_4Wide.n; i++) {		// index of the 4-wide
		Particle_4Wide part4 = particles_4Wide.p[i];
		ProbabilityExponents_4Wide e4 = particles_4Wide.e[i];

		for (int j = 0; j < SSE_WIDTH; j++)  {			// index into the current 4-wide
			float x = peFunc(e4[j]);
			float color = exp(x);

			if (color < PARTICLE_COLOR_THRESHOLD) {
				continue;
			}
#if INVERTED_COLORS
			color = 1.0f - color;
#endif

			Point2D pos = part4.pos[j];
			AngRad  ang = part4.ang[j];

			drawMidVector(pos.x, pos.y, ang, color, color, color);
			drawSmallPoint(pos.x, pos.y, color, color, color);
		}
	}
}

static
void drawParticles() {
	// hook up the probability exponent function pointer based
	// on the sdMode
	float (*peFunc)(const ProbabilityExponents &pe) = NULL;
	switch (sdMode) {
		case SD_DISTANCE_AND_BEARING:
			peFunc = getDistancePlusBearingExponent;
			break;

		case SD_DISTANCE:
			peFunc = getDistanceExponent;
			break;

		case SD_BEARING:
			peFunc = getBearingExponent;
			break;

		case SD_ALL_PARTICLES:
			peFunc = alwaysZero;
			break;

		default:
			error("unknown similarity display mode");
	}

	if (getPfMode() == PF_SCALAR) {
		drawParticlesScalar(peFunc);
	} else {
		drawParticlesSse(peFunc);
	}
}

static
void drawRefObj(int ref_obj_id) {
	const Point2D &p = refObjs[ref_obj_id];
	drawBigPoint(p.x, p.y,
				 REF_OBJ_COLOR[0], REF_OBJ_COLOR[1], REF_OBJ_COLOR[2]);
}

static
void drawRefObjs() {
	for (int i = 0; i < numRefObjs; i++) {
		drawRefObj(i);
	}
}

static
void drawObservations() {
	enableGrassScissor();

	int b = obsWindow->getBase();
	int n = obsWindow->getSize();

	for (int i = 0; i < n; i++) {
		Observation &o = observations[b + i];
		const Point2D &p = refObjs[o.id];

		drawCircle(p.x, p.y, o.d, OBS_COLOR[0], OBS_COLOR[1], OBS_COLOR[2]);
	}

	disableGrassScissor();
}

static
void drawActualPose() {
	Point2D &pos = actualPose.pos_mn;
	AngRad   ang = actualPose.ang_mn;

	drawMidVector(pos.x, pos.y, ang,
				  ROBOT_COLOR[0], ROBOT_COLOR[1], ROBOT_COLOR[2]);
	drawBigPoint(pos.x, pos.y,
				 ROBOT_COLOR[0], ROBOT_COLOR[1], ROBOT_COLOR[2]);
}

static
void drawEstPose() {
	Point2D &pos_mn = estPose.pos_mn;
	Point2D &pos_sd = estPose.pos_sd;

	// draw the estimated position and draw two circles
	// representing one std dev and two std dev
	drawBigPoint(pos_mn.x, pos_mn.y,
				 ESTPOSE_COLOR[0], ESTPOSE_COLOR[1], ESTPOSE_COLOR[2]);

	enableGrassScissor();

	// position one std dev
	drawEllipse(pos_mn.x, pos_mn.y, pos_sd.x, pos_sd.y,
				ESTPOSE_COLOR[0], ESTPOSE_COLOR[1], ESTPOSE_COLOR[2]);

	// position two std dev
	drawEllipse(pos_mn.x, pos_mn.y, 2.0f * pos_sd.x, 2.0f * pos_sd.y,
				ESTPOSE_COLOR[0], ESTPOSE_COLOR[1], ESTPOSE_COLOR[2]);

	disableGrassScissor();

	AngRad   ang_mn = estPose.ang_mn;
	AngRad   ang_sd = estPose.ang_sd;

	// draw the estimated angle using a long vector and draw
	// vector pairs indicating one std dev and two std dev
	drawLongVector(pos_mn.x, pos_mn.y, ang_mn,
				   ESTPOSE_COLOR[0], ESTPOSE_COLOR[1], ESTPOSE_COLOR[2]);

	// bearing one std dev
	drawMidVector(pos_mn.x, pos_mn.y, ang_mn + ang_sd,
				  ESTPOSE_COLOR[0], ESTPOSE_COLOR[1], ESTPOSE_COLOR[2]);
	drawMidVector(pos_mn.x, pos_mn.y, ang_mn - ang_sd,
				  ESTPOSE_COLOR[0], ESTPOSE_COLOR[1], ESTPOSE_COLOR[2]);

	// bearing two std dev
	drawShortVector(pos_mn.x, pos_mn.y, ang_mn + 2.0f * ang_sd,
					ESTPOSE_COLOR[0], ESTPOSE_COLOR[1], ESTPOSE_COLOR[2]);
	drawShortVector(pos_mn.x, pos_mn.y, ang_mn - 2.0f * ang_sd,
					ESTPOSE_COLOR[0], ESTPOSE_COLOR[1], ESTPOSE_COLOR[2]);
}

// x and y should be in world coordinates
static
void drawString(const char *s, float x, float y) {
	glColor3f(TEXT_COLOR[0], TEXT_COLOR[1], TEXT_COLOR[2]);
	glRasterPos2f(x, y);

	for (const char *c = s; *c != '\0'; c++) {
		glutBitmapCharacter(GLUT_BITMAP_HELVETICA_10, *c);
	}
}

static
void drawHud() {
	static const int HUD_STRING_MAX_SIZE = 512;
	char msg[HUD_STRING_MAX_SIZE];

	// in world coordinates
	float spacing    = 5.0f * worldPerWindow;
	float font_h     = 10.0f * worldPerWindow;
	float upper_row1 = 0.93f * windowHeight * worldPerWindow;
	float upper_row2 = upper_row1 - (spacing + font_h)*2.0f;
	float lower_row2 = -0.95f * windowHeight * worldPerWindow;
	float lower_row1 = lower_row2 + (spacing + font_h)*2.0f;

	float left       = -0.95f * windowWidth * worldPerWindow;
	float hmid       = -0.1167f * windowWidth* worldPerWindow;
	float right      =  0.5167f * windowWidth * worldPerWindow;

	// horizaontal adjustment for drawing text
	float horiz_spacer = spacing * 3.0f;

	// vertical adjustment for drawing points
	float vert_spacer = spacing;

	// pf mode and num particles (upper left)
	int n = (getPfMode() == PF_SCALAR) ? particles.n : particles_4Wide.n * SSE_WIDTH;
	sprintf(msg, "%s particle filter [%d particles]", getPfModeString(), n);
	drawString(msg, left, upper_row1);

	// fps (upper left)
	sprintf(msg, "inner loop fps: %.1f", pfFps);
	drawString(msg, left, upper_row2);

	// observation num and window size (lower left)
	sprintf(msg, "[observation %d] window size: %d",
			obsWindow->getBase(), obsWindow->getSize());
	drawString(msg, left, lower_row1);

	// display filter (similarity display mode) (lower left)
	sprintf(msg, "filter: %s", toString(sdMode));
	drawString(msg, left, lower_row2);

	// reference object (lower mid)
	sprintf(msg, "reference object");
	drawString(msg, hmid + horiz_spacer, lower_row1);
	drawBigPoint(hmid, lower_row1 + vert_spacer,
				 REF_OBJ_COLOR[0], REF_OBJ_COLOR[1], REF_OBJ_COLOR[2]);

	// actual pose (lower mid)
	sprintf(msg, "actual pose");
	drawString(msg, hmid + horiz_spacer, lower_row2);
	drawBigPoint(hmid, lower_row2 + vert_spacer,
				 ROBOT_COLOR[0], ROBOT_COLOR[1], ROBOT_COLOR[2]);

	// particle (lower right)
	sprintf(msg, "particle");
	drawString(msg, right + horiz_spacer, lower_row1);
	drawBigPoint(right, lower_row1 + vert_spacer,
				 PARTLEG_COLOR[0], PARTLEG_COLOR[1], PARTLEG_COLOR[2]);

	// estimated pose (lower right)
	sprintf(msg, "estimated pose");
	drawString(msg, right + horiz_spacer, lower_row2);
	drawBigPoint(right, lower_row2 + vert_spacer,
				 ESTPOSE_COLOR[0], ESTPOSE_COLOR[1], ESTPOSE_COLOR[2]);
}

static
void drawScene() {
	drawParticles();
	drawRefObjs();
	drawObservations();
	drawActualPose();
	drawEstPose();
	drawGrass();
	drawHud();
}

static
void drawCb() {
	oneTimeInit();

	// invoke the particle filter
	estPose = runPf();
	pfFps = getLastPfFps();

	glClear(GL_COLOR_BUFFER_BIT);
	drawScene();
	glutSwapBuffers();
}

static
void reshapeCb(int w, int h) {
	if (w == 0 || h == 0) {
		return;
	}

	glViewport(0, 0, w, h);
	updateWindowSizeState(w, h);
}

static
void keyboardCb(unsigned char c, int x, int y) {
	switch (c) {
		// quit
		case 'q':
		case 'Q':
			exit(0);

		// toggle the particle filter mode
		case '`':
		case '~':
			togglePfMode();
			glutPostRedisplay();
			break;

		// change which similarity probability is displayed
		case '\t':
			sdMode = getNext(sdMode);
			glutPostRedisplay();
			break;

		default:
			break;
	}
}

static
void spKeyboardCb(int c, int x, int y) {
	switch (c) {
		// go to next observation
		case GLUT_KEY_RIGHT:
			obsWindow->next();
			glutPostRedisplay();
			break;

		// go to previous observation
		case GLUT_KEY_LEFT:
			obsWindow->prev();
			glutPostRedisplay();
			break;

		// increase the size of the observation window
		case GLUT_KEY_UP:
			obsWindow->grow();
			glutPostRedisplay();
			break;

		// decrease the size of the observation window
		case GLUT_KEY_DOWN:
			obsWindow->shrink();
			glutPostRedisplay();
			break;

		default:
			break;
	}
}

void initWindow(int argc, char **argv) {
	// using these functions to suppress the unreferenced func warning
	(void)glutCreateMenu;

	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA);
	glutInitWindowSize(windowWidth, windowHeight);
	glutInitWindowPosition(INIT_X_POS, INIT_Y_POS);

	windowID = glutCreateWindow(WINDOW_NAME);
	glutDisplayFunc(drawCb);
	glutReshapeFunc(reshapeCb);
	glutKeyboardFunc(keyboardCb);
	glutSpecialFunc(spKeyboardCb);
}

void closeWindow() {
	if (windowID != 0) {
		glutDestroyWindow(windowID);
		windowID = 0;
	}
}

void enterDrawLoop() {
	glutMainLoop();
}

// end of Draw.cpp
