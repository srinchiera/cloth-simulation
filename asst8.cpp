////////////////////////////////////////////////////////////////////////
//
//   Harvard University
//   CS175 : Computer Graphics
//   Professor Steven Gortler
//
////////////////////////////////////////////////////////////////////////

#include <cstddef>
#include <vector>
#include <list>
#include <string>
#include <memory>
#include <map>
#include <fstream>
#include <stdexcept>
#if __GNUG__
#   include <tr1/memory>
#endif

#include <GL/glew.h>
#ifdef __MAC__
#   include <GLUT/glut.h>
#else
#   include <GL/glut.h>
#endif

#include "ppm.h"
#include "cvec.h"
#include "matrix4.h"
#include "rigtform.h"
#include "glsupport.h"
#include "geometrymaker.h"
#include "geometry.h"
#include "arcball.h"
#include "scenegraph.h"
#include "sgutils.h"

#include "asstcommon.h"
#include "drawer.h"
#include "picker.h"
#include "mesh.h"
#include "spring.h"
#include "ball.h"
#include "cloth.h"

using namespace std;
using namespace tr1;

// G L O B A L S ///////////////////////////////////////////////////

// --------- IMPORTANT --------------------------------------------------------
// Before you start working on this assignment, set the following variable
// properly to indicate whether you want to use OpenGL 2.x with GLSL 1.0 or
// OpenGL 3.x+ with GLSL 1.3.
//
// Set g_Gl2Compatible = true to use GLSL 1.0 and g_Gl2Compatible = false to
// use GLSL 1.3. Make sure that your machine supports the version of GLSL you
// are using. In particular, on Mac OS X currently there is no way of using
// OpenGL 3.x with GLSL 1.3 when GLUT is used.
//
// If g_Gl2Compatible=true, shaders with -gl2 suffix will be loaded.
// If g_Gl2Compatible=false, shaders with -gl3 suffix will be loaded.
// To complete the assignment you only need to edit the shader files that get
// loaded
// ----------------------------------------------------------------------------
const bool g_Gl2Compatible = true;


static const float g_frustMinFov = 60.0;  // A minimal of 60 degree field of view
static float g_frustFovY = g_frustMinFov; // FOV in y direction (updated by updateFrustFovY)

static const float g_frustNear = -0.1;    // near plane
static const float g_frustFar = -50.0;    // far plane
static const float g_groundY = -2.0;      // y coordinate of the ground
static const float g_groundSize = 10.0;   // half the ground length

enum SkyMode {WORLD_SKY=0, SKY_SKY=1};

static int g_windowWidth = 512;
static int g_windowHeight = 512;
static bool g_mouseClickDown = false;    // is the mouse button pressed
static bool g_mouseLClickButton, g_mouseRClickButton, g_mouseMClickButton;
static int g_mouseClickX, g_mouseClickY; // coordinates for mouse click event
static int g_activeShader = 0;

static SkyMode g_activeCameraFrame = WORLD_SKY;

static bool g_displayArcball = true;
static double g_arcballScreenRadius = 100; // number of pixels
static double g_arcballScale = 1;

static bool g_pickingMode = false;

static bool g_playingAnimation = false;

// --------- Materials
static shared_ptr<Material> g_redDiffuseMat,
                            g_blueDiffuseMat,
                            g_bumpFloorMat,
                            g_arcballMat,
                            g_pickingMat,
                            g_lightMat,
                            g_shinyMat;

static shared_ptr<Material> g_bunnyMat; // for the bunny
static shared_ptr<Material> g_clothMat; // for the bunny

// wireframe?
static bool g_clothWire = false;

static vector<shared_ptr<Material> > g_bunnyShellMats; // for bunny shells

shared_ptr<Material> g_overridingMaterial;

static bool g_smoothSubdRendering = false;

// --------- Geometry
typedef SgGeometryShapeNode MyShapeNode;

// Vertex buffer and index buffer associated with the ground and cube geometry
static shared_ptr<Geometry> g_ground, g_cube, g_sphere;
static shared_ptr<Geometry> g_sphere2;
static shared_ptr<SimpleGeometryPN> g_clothGeometry;
Cloth g_cloth;
static Cvec3 clothTranslation = Cvec3(-2.5,-.5,-2.5);

static shared_ptr<SimpleGeometryPN> g_meshGeometry;
static Mesh g_mesh;

static int g_subdLevels = 0;

static shared_ptr<SimpleGeometryPN> g_bunnyGeometry;
static Mesh g_bunnyMesh;

// --------- Scene

static shared_ptr<SgRootNode> g_world;
static shared_ptr<SgRbtNode> g_skyNode, g_groundNode, g_light1, g_light2;
static shared_ptr<SgRbtNode> g_currentCameraNode;
static shared_ptr<SgRbtNode> g_currentPickedRbtNode;
static shared_ptr<SgRbtNode> g_sphere2Node;
static shared_ptr<SgRbtNode> g_clothNode;

//static shared_ptr<SgRbtNode> g_clothNode;


static shared_ptr<SgRbtNode> g_bunnyNode;

static int g_msBetweenKeyFrames = 2000; // 2 seconds between keyframes
static int g_animateFramesPerSecond = 60; // frames to render per second during animation playback

// Global variables for used physical simulation
static const Cvec3 g_gravity(0, -0.5, 0);  // gavity vector
static double g_timeStep = 0.02;
static double g_numStepsPerFrame = 10;
static double g_damping = 0.96;
static double g_stiffness = 10;
static int g_simulationsPerSecond = 60;

///////////////// END OF G L O B A L S //////////////////////////////////////////////////


static VertexPN getVertexPN(Mesh& m, const int face, const int vertex);
static VertexPNX getFurVertexPNX(Mesh& m, const int face, const int vertex, int shellNum);

static void initGround() {
  int ibLen, vbLen;
  getPlaneVbIbLen(vbLen, ibLen);

  // Temporary storage for cube Geometry
  vector<VertexPNTBX> vtx(vbLen);
  vector<unsigned short> idx(ibLen);

  makePlane(g_groundSize*2, vtx.begin(), idx.begin());
  g_ground.reset(new SimpleIndexedGeometryPNTBX(&vtx[0], &idx[0], vbLen, ibLen));
}

static void initCubes() {
  int ibLen, vbLen;
  getCubeVbIbLen(vbLen, ibLen);

  // Temporary storage for cube Geometry
  vector<VertexPNTBX> vtx(vbLen);
  vector<unsigned short> idx(ibLen);

  makeCube(1, vtx.begin(), idx.begin());
  g_cube.reset(new SimpleIndexedGeometryPNTBX(&vtx[0], &idx[0], vbLen, ibLen));
}

static void initCloth() {
  g_cloth.loadDimensions(5,5,45,45);

  g_clothGeometry.reset(new SimpleGeometryPN(&(g_cloth.getVertices()[0]), g_cloth.getVertices().size()));
}

static void animateCloth(int dontCare) {

  int vbLen = 45*45*6;
  g_cloth.addForce(Cvec3(0,-0.2,0)*.25);
  g_cloth.timeStep();
  g_cloth.collision((*g_sphere2Node).getRbt().getTranslation()-clothTranslation,1);

  g_clothGeometry->upload(&(g_cloth.getVertices()[0]), vbLen);
  glutTimerFunc(1000/g_animateFramesPerSecond, animateCloth, 0);
  glutPostRedisplay();
}

static void initSphere() {
  int ibLen, vbLen;
  getSphereVbIbLen(20, 10, vbLen, ibLen);

  // Temporary storage for sphere Geometry
  vector<VertexPNTBX> vtx(vbLen);
  vector<unsigned short> idx(ibLen);
  makeSphere(1, 20, 10, vtx.begin(), idx.begin());
  g_sphere.reset(new SimpleIndexedGeometryPNTBX(&vtx[0], &idx[0], vtx.size(), idx.size()));
}

static void initSphere2() {
  int ibLen, vbLen;
  getSphereVbIbLen(20, 10, vbLen, ibLen);

  // Temporary storage for sphere Geometry
  vector<VertexPNTBX> vtx(vbLen);
  vector<unsigned short> idx(ibLen);
  makeSphere(1, 20, 10, vtx.begin(), idx.begin());
  g_sphere2.reset(new SimpleIndexedGeometryPNTBX(&vtx[0], &idx[0], vtx.size(), idx.size()));
}

static void initNormals(Mesh& m) {
  for (int i = 0; i < m.getNumVertices(); ++i) {
    m.getVertex(i).setNormal(Cvec3(0));
  }
  for (int i = 0; i < m.getNumFaces(); ++i) {
    const Mesh::Face f = m.getFace(i);
    const Cvec3 n = f.getNormal();
    for (int j = 0; j < f.getNumVertices(); ++j) {
      f.getVertex(j).setNormal(f.getVertex(j).getNormal() + n);
    }
  }
  for (int i = 0; i < m.getNumVertices(); ++i) {
    Cvec3 n = m.getVertex(i).getNormal();
    if (norm2(n) > CS175_EPS2)
      m.getVertex(i).setNormal(normalize(n));
  }
}

static void subdivide(Mesh& m) {
  for (int i = 0; i < m.getNumFaces(); ++i) {
    const Mesh::Face f = m.getFace(i);
    m.setNewFaceVertex(f, Cvec3(0));
    for (int j = f.getNumVertices()-1; j >= 0; --j) {
      m.setNewFaceVertex(f, m.getNewFaceVertex(f) + f.getVertex(j).getPosition());
    }
    m.setNewFaceVertex(f, m.getNewFaceVertex(f) / f.getNumVertices());
  }
  for (int i = 0; i < m.getNumEdges(); ++i) {
    const Mesh::Edge e = m.getEdge(i);
    m.setNewEdgeVertex(e, (m.getNewFaceVertex(e.getFace(0)) + m.getNewFaceVertex(e.getFace(1)) + e.getVertex(0).getPosition() + e.getVertex(1).getPosition()) * 0.25);
  }

  for (int i = 0; i < m.getNumVertices(); ++i) {
    const Mesh::Vertex v = m.getVertex(i);
    Cvec3 rv = v.getPosition(), re(0), rf(0);
    double n = 0;
    Mesh::VertexIterator it(v.getIterator()), it0(it);
    do {
      re += it.getVertex().getPosition();
      rf += m.getNewFaceVertex(it.getFace());
    }
    while (++n, ++it != it0);
    m.setNewVertexVertex(v, rv * ((n-2)/n) + (re + rf) / (n*n));
  }
  m.subdivide();
}

static VertexPN getVertexPN(Mesh& m, const int face, const int vertex) {
  const Mesh::Face f = m.getFace(face);
  const Cvec3 n = g_smoothSubdRendering ? f.getVertex(vertex).getNormal() : f.getNormal();
  const Cvec3& v = f.getVertex(vertex).getPosition();
  return VertexPN(v[0], v[1], v[2], n[0], n[1], n[2]);
}

// takes a projection matrix and send to the the shaders
static void sendProjectionMatrix(Uniforms& uniforms, const Matrix4& projMatrix) {
  uniforms.put("uProjMatrix", projMatrix);
}

// update g_frustFovY from g_frustMinFov, g_windowWidth, and g_windowHeight
static void updateFrustFovY() {
  if (g_windowWidth >= g_windowHeight)
    g_frustFovY = g_frustMinFov;
  else {
    const double RAD_PER_DEG = 0.5 * CS175_PI/180;
    g_frustFovY = atan2(sin(g_frustMinFov * RAD_PER_DEG) * g_windowHeight / g_windowWidth, cos(g_frustMinFov * RAD_PER_DEG)) / RAD_PER_DEG;
  }
}

static Matrix4 makeProjectionMatrix() {
  return Matrix4::makeProjection(
           g_frustFovY, g_windowWidth / static_cast <double> (g_windowHeight),
           g_frustNear, g_frustFar);
}

enum ManipMode {
  ARCBALL_ON_PICKED,
  ARCBALL_ON_SKY,
  EGO_MOTION
};

static ManipMode getManipMode() {
  // if nothing is picked or the picked transform is the transfrom we are viewing from
  if (g_currentPickedRbtNode == NULL || g_currentPickedRbtNode == g_currentCameraNode) {
    if (g_currentCameraNode == g_skyNode && g_activeCameraFrame == WORLD_SKY)
      return ARCBALL_ON_SKY;
    else
      return EGO_MOTION;
  }
  else
    return ARCBALL_ON_PICKED;
}

static bool shouldUseArcball() {
  return getManipMode() != EGO_MOTION;
}

// The translation part of the aux frame either comes from the current
// active object, or is the identity matrix when
static RigTForm getArcballRbt() {
  switch (getManipMode()) {
  case ARCBALL_ON_PICKED:
    return getPathAccumRbt(g_world, g_currentPickedRbtNode);
  case ARCBALL_ON_SKY:
    return RigTForm();
  case EGO_MOTION:
    return getPathAccumRbt(g_world, g_currentCameraNode);
  default:
    throw runtime_error("Invalid ManipMode");
  }
}

static void updateArcballScale() {
  RigTForm arcballEye = inv(getPathAccumRbt(g_world, g_currentCameraNode)) * getArcballRbt();
  double depth = arcballEye.getTranslation()[2];
  if (depth > -CS175_EPS)
    g_arcballScale = 0.02;
  else
    g_arcballScale = getScreenToEyeScale(depth, g_frustFovY, g_windowHeight);
}

static void drawArcBall(Uniforms& uniforms) {
  RigTForm arcballEye = inv(getPathAccumRbt(g_world, g_currentCameraNode)) * getArcballRbt();
  Matrix4 MVM = rigTFormToMatrix(arcballEye) * Matrix4::makeScale(Cvec3(1, 1, 1) * g_arcballScale * g_arcballScreenRadius);

  sendModelViewNormalMatrix(uniforms, MVM, normalMatrix(MVM));

  g_arcballMat->draw(*g_sphere, uniforms);
}

static void drawStuff(bool picking) {
  g_cloth.collision((*g_sphere2Node).getRbt().getTranslation()-clothTranslation,1);
  g_clothGeometry->upload(&(g_cloth.getVertices()[0]), g_cloth.getVertices().size());

  // if we are not translating, update arcball scale
  if (!(g_mouseMClickButton || (g_mouseLClickButton && g_mouseRClickButton)))
    updateArcballScale();

  Uniforms uniforms;

  // build & send proj. matrix to vshader
  const Matrix4 projmat = makeProjectionMatrix();
  sendProjectionMatrix(uniforms, projmat);

  const RigTForm eyeRbt = getPathAccumRbt(g_world, g_currentCameraNode);
  const RigTForm invEyeRbt = inv(eyeRbt);

  Cvec3 l1 = getPathAccumRbt(g_world, g_light1).getTranslation();
  Cvec3 l2 = getPathAccumRbt(g_world, g_light2).getTranslation();
  uniforms.put("uLight", Cvec3(invEyeRbt * Cvec4(l1, 1)));
  uniforms.put("uLight2", Cvec3(invEyeRbt * Cvec4(l2, 1)));

  if (!picking) {
    Drawer drawer(invEyeRbt, uniforms);
    g_world->accept(drawer);

    if (g_displayArcball && shouldUseArcball())
      drawArcBall(uniforms);
  }
  else {
    Picker picker(invEyeRbt, uniforms);

    g_overridingMaterial = g_pickingMat;
    g_world->accept(picker);
    g_overridingMaterial.reset();

    glFlush();
    g_currentPickedRbtNode = picker.getRbtNodeAtXY(g_mouseClickX, g_mouseClickY);
    if (g_currentPickedRbtNode == g_groundNode)
      g_currentPickedRbtNode.reset(); // set to NULL

    cout << (g_currentPickedRbtNode ? "Part picked" : "No part picked") << endl;
  }



}

static void display() {

  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);


  drawStuff(false);

  glutSwapBuffers();

  checkGlErrors();
}

static void pick() {
  // We need to set the clear color to black, for pick rendering.
  // so let's save the clear color
  GLdouble clearColor[4];
  glGetDoublev(GL_COLOR_CLEAR_VALUE, clearColor);

  glClearColor(0, 0, 0, 0);

  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  drawStuff(true);

  // Uncomment below and comment out the glutPostRedisplay in mouse(...) call back
  // to see result of the pick rendering pass
  // glutSwapBuffers();

  //Now set back the clear color
  glClearColor(clearColor[0], clearColor[1], clearColor[2], clearColor[3]);

  checkGlErrors();
}

static void reshape(const int w, const int h) {
  g_windowWidth = w;
  g_windowHeight = h;
  glViewport(0, 0, w, h);
  cerr << "Size of window is now " << w << "x" << h << endl;
  g_arcballScreenRadius = min(h, w) * 0.25;
  updateFrustFovY();
  glutPostRedisplay();
}

static Cvec3 getArcballDirection(const Cvec2& p, const double r) {
  double n2 = norm2(p);
  if (n2 >= r*r)
    return normalize(Cvec3(p, 0));
  else
    return normalize(Cvec3(p, sqrt(r*r - n2)));
}

static RigTForm moveArcball(const Cvec2& p0, const Cvec2& p1) {
  const Matrix4 projMatrix = makeProjectionMatrix();
  const RigTForm eyeInverse = inv(getPathAccumRbt(g_world, g_currentCameraNode));
  const Cvec3 arcballCenter = getArcballRbt().getTranslation();
  const Cvec3 arcballCenter_ec = Cvec3(eyeInverse * Cvec4(arcballCenter, 1));

  if (arcballCenter_ec[2] > -CS175_EPS)
    return RigTForm();

  Cvec2 ballScreenCenter = getScreenSpaceCoord(arcballCenter_ec,
                                               projMatrix, g_frustNear, g_frustFovY, g_windowWidth, g_windowHeight);
  const Cvec3 v0 = getArcballDirection(p0 - ballScreenCenter, g_arcballScreenRadius);
  const Cvec3 v1 = getArcballDirection(p1 - ballScreenCenter, g_arcballScreenRadius);

  return RigTForm(Quat(0.0, v1[0], v1[1], v1[2]) * Quat(0.0, -v0[0], -v0[1], -v0[2]));
}

static RigTForm doMtoOwrtA(const RigTForm& M, const RigTForm& O, const RigTForm& A) {
  return A * M * inv(A) * O;
}

static RigTForm getMRbt(const double dx, const double dy) {
  RigTForm M;

  if (g_mouseLClickButton && !g_mouseRClickButton) {
    if (shouldUseArcball())
      M = moveArcball(Cvec2(g_mouseClickX, g_mouseClickY), Cvec2(g_mouseClickX + dx, g_mouseClickY + dy));
    else
      M = RigTForm(Quat::makeXRotation(-dy) * Quat::makeYRotation(dx));
  }
  else {
    double movementScale = getManipMode() == EGO_MOTION ? 0.02 : g_arcballScale;
    if (g_mouseRClickButton && !g_mouseLClickButton) {
      M = RigTForm(Cvec3(dx, dy, 0) * movementScale);
    }
    else if (g_mouseMClickButton || (g_mouseLClickButton && g_mouseRClickButton)) {
      M = RigTForm(Cvec3(0, 0, -dy) * movementScale);
    }
  }

  switch (getManipMode()) {
  case ARCBALL_ON_PICKED:
    break;
  case ARCBALL_ON_SKY:
    M = inv(M);
    break;
  case EGO_MOTION:
    if (g_mouseLClickButton && !g_mouseRClickButton) // only invert rotation
      M = inv(M);
    break;
  }
  return M;
}

static RigTForm makeMixedFrame(const RigTForm& objRbt, const RigTForm& eyeRbt) {
  return transFact(objRbt) * linFact(eyeRbt);
}

// l = w X Y Z
// o = l O
// a = w A = l (Z Y X)^1 A = l A'
// o = a (A')^-1 O
//   => a M (A')^-1 O = l A' M (A')^-1 O

static void motion(const int x, const int y) {
  if (!g_mouseClickDown)
    return;

  const double dx = x - g_mouseClickX;
  const double dy = g_windowHeight - y - 1 - g_mouseClickY;

  const RigTForm M = getMRbt(dx, dy);   // the "action" matrix

  // the matrix for the auxiliary frame (the w.r.t.)
  RigTForm A = makeMixedFrame(getArcballRbt(), getPathAccumRbt(g_world, g_currentCameraNode));

  shared_ptr<SgRbtNode> target;
  switch (getManipMode()) {
  case ARCBALL_ON_PICKED:
    target = g_currentPickedRbtNode;
    break;
  case ARCBALL_ON_SKY:
    target = g_skyNode;
    break;
  case EGO_MOTION:
    target = g_currentCameraNode;
    break;
  }

  A = inv(getPathAccumRbt(g_world, target, 1)) * A;

  target->setRbt(doMtoOwrtA(M, target->getRbt(), A));

  g_mouseClickX += dx;
  g_mouseClickY += dy;
  glutPostRedisplay();  // we always redraw if we changed the scene
}

static void mouse(const int button, const int state, const int x, const int y) {
  g_mouseClickX = x;
  g_mouseClickY = g_windowHeight - y - 1;  // conversion from GLUT window-coordinate-system to OpenGL window-coordinate-system

  g_mouseLClickButton |= (button == GLUT_LEFT_BUTTON && state == GLUT_DOWN);
  g_mouseRClickButton |= (button == GLUT_RIGHT_BUTTON && state == GLUT_DOWN);
  g_mouseMClickButton |= (button == GLUT_MIDDLE_BUTTON && state == GLUT_DOWN);

  g_mouseLClickButton &= !(button == GLUT_LEFT_BUTTON && state == GLUT_UP);
  g_mouseRClickButton &= !(button == GLUT_RIGHT_BUTTON && state == GLUT_UP);
  g_mouseMClickButton &= !(button == GLUT_MIDDLE_BUTTON && state == GLUT_UP);

  g_mouseClickDown = g_mouseLClickButton || g_mouseRClickButton || g_mouseMClickButton;

  if (g_pickingMode && button == GLUT_LEFT_BUTTON && state == GLUT_DOWN) {
    pick();
    g_pickingMode = false;
    cerr << "Picking mode is off" << endl;
    glutPostRedisplay(); // request redisplay since the arcball will have moved
  }
  glutPostRedisplay();
}


static void keyboard(const unsigned char key, const int x, const int y) {
  switch (key) {
  case 27:
    exit(0);                                  // ESC
  case 'h':
    cout << " ============== H E L P ==============\n\n"
    << "h\t\thelp menu\n"
    << "s\t\tsave screenshot\n"
    << "p\t\tUse mouse to pick a part to edit\n"
    << "v\t\tCycle view\n"
    << "drag left mouse to rotate\n"
    << "a\t\tToggle display arcball\n"
    << "space\t\tRelease cloth\n"
    << "u\t\tUnfix 2 points on cloth\n"
    << "w\t\tToggle cloth wireframe\n"
    << endl;
    break;
  case 's':
    glFlush();
    writePpmScreenshot(g_windowWidth, g_windowHeight, "out.ppm");
    break;
  case 'v':
  {
    shared_ptr<SgRbtNode> viewers[] = {g_skyNode};
    for (int i = 0; i < 3; ++i) {
      if (g_currentCameraNode == viewers[i]) {
        g_currentCameraNode = viewers[(i+1)%3];
        break;
      }
    }
  }
  break;
  case ' ':
    g_cloth.unfix();
    break;
  case 'p':
    g_pickingMode = !g_pickingMode;
    cerr << "Picking mode is " << (g_pickingMode ? "on" : "off") << endl;
    break;
  case 'm':
    g_activeCameraFrame = SkyMode((g_activeCameraFrame+1) % 2);
    cerr << "Editing sky eye w.r.t. " << (g_activeCameraFrame == WORLD_SKY ? "world-sky frame\n" : "sky-sky frame\n") << endl;
    break;
  case 'a':
    g_displayArcball = !g_displayArcball;
    break;
  case 'u':
    g_cloth.unfix2();
    break;
  case 'r':
    g_cloth = Cloth();
    break;
  case 'w':
    if (g_clothWire) {
        g_clothMat->getRenderStates().polygonMode(GL_FRONT_AND_BACK, GL_FILL);
        g_clothWire = false;
    } else {
        g_clothMat->getRenderStates().polygonMode(GL_FRONT_AND_BACK, GL_LINE);
        g_clothWire = true;
    }
    break;
  }

  glutPostRedisplay();
}

static void initGlutState(int argc, char * argv[]) {
  glutInit(&argc, argv);                                  // initialize Glut based on cmd-line args
  glutInitDisplayMode(GLUT_RGBA|GLUT_DOUBLE|GLUT_DEPTH);  // RGBA pixel channels and double buffering
  glutInitWindowSize(g_windowWidth, g_windowHeight);      // create a window
  glutCreateWindow("Assignment 7");                       // title the window

  glutDisplayFunc(display);                               // display rendering callback
  glutReshapeFunc(reshape);                               // window reshape callback
  glutMotionFunc(motion);                                 // mouse movement callback
  glutMouseFunc(mouse);                                   // mouse click callback
  glutKeyboardFunc(keyboard);
//  glutSpecialFunc(specialKeyboard);                       // special keyboard callback
}

static void initGLState() {
  glClearColor(128./255., 200./255., 255./255., 0.);
  glClearDepth(0.);
  glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
  glPixelStorei(GL_PACK_ALIGNMENT, 1);
  glCullFace(GL_BACK);
  glEnable(GL_CULL_FACE);
  glEnable(GL_DEPTH_TEST);
  glDepthFunc(GL_GREATER);
  glReadBuffer(GL_BACK);
  if (!g_Gl2Compatible)
    glEnable(GL_FRAMEBUFFER_SRGB);
}

static void updateBunnyMesh() {
  Mesh m = g_bunnyMesh;

  initNormals(g_bunnyMesh);

  // dynamic vertex buffer
  vector<VertexPN> verts;
  verts.reserve(m.getNumFaces() * 6); // conservative estimate of num of vertices

  for (int i = 0; i < m.getNumFaces(); ++i) {
    for (int j = 0; j < 3; ++j) {
      verts.push_back(getVertexPN(m, i, j));
    }
    if (m.getFace(i).getNumVertices() == 4) {
      // need another triangle to finish the face
      for (int j = 0; j < 3; ++j) {
        verts.push_back(getVertexPN(m, i, (2 + j) % 4));
      }
    }
  }

  g_bunnyGeometry->upload(&verts[0], verts.size());

}

// New function that loads the bunny mesh and initializes the bunny shell meshes
static void initBunnyMeshes() {
  g_bunnyMesh.load("bunny.mesh");

  g_bunnyGeometry.reset(new SimpleGeometryPN());
  updateBunnyMesh();

}

static void initMaterials() {
  // Create some prototype materials
  Material diffuse("./shaders/basic-gl3.vshader", "./shaders/diffuse-gl3.fshader");
  Material solid("./shaders/basic-gl3.vshader", "./shaders/solid-gl3.fshader");

  // normal mapping
  g_bumpFloorMat.reset(new Material("./shaders/normal-gl3.vshader", "./shaders/normal-gl3.fshader"));
  g_bumpFloorMat->getUniforms().put("uTexColor", shared_ptr<ImageTexture>(new ImageTexture("Fieldstone.ppm", true)));
  g_bumpFloorMat->getUniforms().put("uTexNormal", shared_ptr<ImageTexture>(new ImageTexture("FieldstoneNormal.ppm", false)));

  // copy solid prototype, and set to wireframed rendering
  g_arcballMat.reset(new Material(solid));
  g_arcballMat->getUniforms().put("uColor", Cvec3f(0.27f, 0.82f, 0.35f));
  g_arcballMat->getRenderStates().polygonMode(GL_FRONT_AND_BACK, GL_LINE);

  // copy solid prototype, and set to color white
  g_lightMat.reset(new Material(solid));
  g_lightMat->getUniforms().put("uColor", Cvec3f(1, 1, 1));

  // pick shader
  g_pickingMat.reset(new Material("./shaders/basic-gl3.vshader", "./shaders/pick-gl3.fshader"));

  // shiny material
  g_shinyMat.reset(new Material("./shaders/basic-gl3.vshader", "./shaders/specular-gl3.fshader"));
  g_shinyMat->getUniforms().put("uColor", Cvec3f(.6f, .6f, 0.f));

  // bunny material
  g_bunnyMat.reset(new Material("./shaders/basic-gl3.vshader", "./shaders/bunny-gl3.fshader"));
  g_bunnyMat->getUniforms()
  .put("uColorAmbient", Cvec3f(0.45f, 0.3f, 0.3f))
  .put("uColorDiffuse", Cvec3f(0.2f, 0.2f, 0.2f));

  // copy solid prototype, and set to wireframed rendering
  g_clothMat.reset(new Material("./shaders/basic-gl3.vshader", "./shaders/specular-gl3.fshader"));
  g_clothMat->getUniforms().put("uColor", Cvec3f(102./255., 0./255., 153./255.));
  g_clothMat->getRenderStates().polygonMode(GL_FRONT_AND_BACK, GL_FILL);

};

static void initGeometry() {
  initGround();
  initSphere();
  initBunnyMeshes();
  initCubes();

  initSphere2();
  initCloth();
}

static void initScene() {
  g_world.reset(new SgRootNode());

  g_skyNode.reset(new SgRbtNode(RigTForm(Cvec3(0.0, 0.25, 8.0))));

  g_groundNode.reset(new SgRbtNode(Cvec3(0, g_groundY, 0)));
  g_groundNode->addChild(shared_ptr<MyShapeNode>(
                           new MyShapeNode(g_ground, g_bumpFloorMat)));

  g_light1.reset(new SgRbtNode(RigTForm(Cvec3(4.0, 3.0, 5.0))));
  g_light2.reset(new SgRbtNode(RigTForm(Cvec3(-4, 1.0, -4.0))));
  g_light1->addChild(shared_ptr<MyShapeNode>(
                       new MyShapeNode(g_sphere, g_lightMat, Cvec3(0), Cvec3(0), Cvec3(0.5))));

  g_light2->addChild(shared_ptr<MyShapeNode>(
                       new MyShapeNode(g_sphere, g_lightMat, Cvec3(0), Cvec3(0), Cvec3(0.5))));

  // create a single transform node for both the bunny and the bunny shells
  g_bunnyNode.reset(new SgRbtNode(Cvec3(-3.0,0.0,-4.0)));

  // add bunny as a shape nodes
  g_bunnyNode->addChild(shared_ptr<MyShapeNode>(
                          new MyShapeNode(g_bunnyGeometry, g_bunnyMat)));

  g_sphere2Node.reset(new SgRbtNode());
  g_sphere2Node->addChild(shared_ptr<MyShapeNode>(
                          new MyShapeNode(g_sphere2, g_bunnyMat)));

  g_clothNode.reset(new SgRbtNode(Cvec3(0.0,0.0,0.0)));
  g_clothNode->addChild(shared_ptr<MyShapeNode>(
                          new MyShapeNode(g_clothGeometry, g_clothMat, clothTranslation)));

  g_world->addChild(g_skyNode);
  g_world->addChild(g_groundNode);
  g_world->addChild(g_light1);
  g_world->addChild(g_light2);
  g_world->addChild(g_bunnyNode);

  g_world->addChild(g_sphere2Node);
  g_world->addChild(g_clothNode);

  g_currentCameraNode = g_skyNode;
}

static void initAnimation() {
  animateCloth(0);
}

int main(int argc, char * argv[]) {
  try {
    initGlutState(argc,argv);

    glewInit(); // load the OpenGL extensions

    cout << (g_Gl2Compatible ? "Will use OpenGL 2.x / GLSL 1.0" : "Will use OpenGL 3.x / GLSL 1.3") << endl;
    if ((!g_Gl2Compatible) && !GLEW_VERSION_3_0)
      throw runtime_error("Error: card/driver does not support OpenGL 3.x / GLSL v1.3. Consider setting g_Gl2Compatible to true");
    else if (g_Gl2Compatible && !GLEW_VERSION_2_0)
      throw runtime_error("Error: card/driver does not support OpenGL 2.x / GLSL 1.0");

    initGLState();
    initMaterials();
    initGeometry();
    initScene();
    initAnimation();

    glutMainLoop();
    return 0;
  }
  catch (const runtime_error& e) {
    cout << "Exception caught: " << e.what() << endl;
    return -1;
  }
}
