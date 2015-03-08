/*
* Credit for bullet engine physics tutorial to
*   THECPLUSPLUSGUY @ https://www.youtube.com/watch?v=d7_lJJ_j2NE
*/
#include "main.h"

// Kinect(TM) camera input: 640x480
static int width = 640;
static int height = 480;

static float ceiling = 500.0f;

static bool beg = true;
static bool beg2 = true;
static bool lining = true;
static bool select_prev = false;
static bool snap = false;
static bool snapped = false;
static bool snapfin = false;

static bool m1_down = false; // Left
//static bool cur_select = false;

static double rot_scale = 120.2;

static int cou = 0;
static int ski = 10;
static int f_count = 0;
static int f_skip = 10;
static bool wireg, added;

static btVector3 snappt;
static btVector3 tempsnap;

static float snapdis = 10.0;

btVector3 pos_r, pos_l, start, start_l;
//Vector3 cursor = Vector3(0,50,0);
Physics physics;
btSoftBody* sb;
btRigidBody* rb;

btVector3 pos;
btScalar dis;
btGeneric6DofConstraint* con;

NUI_SKELETON_FRAME frame;

Camera cam = Camera(Vector3(0.0, 150.0, 150.0), Vector3(0.0, 50.0, 0.0), Vector3(0.0, 1.0, 0.0));

Light sl = Light(1);
GLfloat sld[] = { 0.0f, -1.0f, 0.0f };
GLfloat slp[] = { 0.0f, 50.0f, 0.0f };
GLfloat sls[] = { 1.0f, 1.0f, 1.0f, 1.0f };
GLfloat sla[] = { 1.0f, 1.0f, 1.0f, 0.2f };
GLfloat sle = 20.0;

void reshape(int w, int h) {
  width = w;
  height = h;
  glViewport(0, 0, width, height);
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  gluPerspective(60.0, (float)width / height, 1.0, 1000.0);
  glTranslatef(0.0, 0.0, -20.0);
  glMatrixMode(GL_MODELVIEW);
}

void drawSquare(int x, int z, int l) {
  glBegin(GL_QUADS);
  //glNormal3f(0.0f, 1.0f, 0.0f);
  glVertex3f(x, -0.01f, z + l);
  glVertex3f(x + l, -0.01f, z + l);
  glVertex3f(x + l, -0.01f, z);
  glVertex3f(x, -0.01f, z);
  glEnd();
}

void drawCylinder() {
  for(vector<Segment*>::iterator it = Physics::wires.begin(); it != Physics::wires.end(); ++it) {
    //(*it).rb->activate();
    //(wireg) ? (*it).rb->setGravity(btVector3(0, -9.81f, 0)) : (*it).rb->setGravity(btVector3(0, 0, 0));
    if(wireg && !added) {

      //Physics::ddw->addRigidBody((*it).rb);
      (*it)->rb->setGravity(btVector3(0, -9.81f, 0));
      Physics::ddw->removeRigidBody((*it)->rb);

      btVector3 i;
      (*it)->rb->getCollisionShape()->calculateLocalInertia(1.0, i);
      (*it)->rb->setMassProps(1.0f, i);

      //(*it).rb->setLinearVelocity(btVector3(0,-1.0f,0));

      Physics::ddw->addRigidBody((*it)->rb);
      (*it)->rb->activate();
    }
    //cout << "drawinc cylingder" << endl;
      //(*it).rb->setGravity(btVector3(0, -9.81f, 0));
    float m[16];
    btTransform t;
    (*it)->rb->getMotionState()->getWorldTransform(t);
    t.getOpenGLMatrix(m);
    glPushMatrix();
    glMultMatrixf((GLfloat*)m);
    //glRotatef(90, -1, 0, 0);
    glColor3f(1.0f, 1.0f, 0.0f);
    glutSolidCylinder((*it)->cs->getRadius(), (*it)->dis.length(), 5, 5);
    glPopMatrix();
  }
  if(wireg && !added) {
    for(vector<btGeneric6DofConstraint*>::iterator it = Physics::cons.begin(); it != Physics::cons.end(); ++it)
      Physics::ddw->addConstraint((*it),true);

    added = true;
  }
}

void display() {
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glMatrixMode(GL_MODELVIEW);

  glLoadIdentity();
  glLoadMatrixd(cam.getGLMatrix());

  // Plane
  glColor3f(0.0f, 1.0f, 0.0f);
  for(int r = 0; r < 40; r++) {
    for(int c = 0; c < 40; c++) {
      ((r + c) % 2) ? glColor3f(0.6f, 0.6f, 0.6f) : glColor3f(0.3f, 0.3f, 0.3f);
      drawSquare(-1000 + 50 *r, -1000 + 50 *c, 50);
    }
  }
  /*
  glBegin(GL_QUADS);
  //glNormal3f(0.0f, 1.0f, 0.0f);
  glVertex3f(-1000.0f, -0.01f, 1000.0f);
  glVertex3f(1000.0f, -0.01f, 1000.0f);
  glVertex3f(1000.0f, -0.01f, -1000.0f);
  glVertex3f(-1000.0f, -0.01f, -1000.0f);
  glEnd();
  */

  // Cylinder
  drawCylinder();

  // Ball
  //Physics::ballrb[0]->setGravity(btVector3(0,-10,0));
  //Physics::ballrb[0]->setLinearVelocity(btVector3(0,0,0));
  Matrix4d temp;
  temp.makeTranslate(Physics::v_cur.getX(), Physics::v_cur.getY(), Physics::v_cur.getZ());
  temp.transpose();
  //float m[16];
  //btTransform t;
  //Physics::ballrb[0]->getMotionState()->getWorldTransform(t);
  //t.getOpenGLMatrix(m);
  glPushMatrix();
  //glMultMatrixf((float*)m);
  glMultMatrixd(temp.getPointer());
  (Physics::cur_select) ? glColor3f(0.0f, 1.0f, 0.0f) : glColor3f(0.0f, 0.0f, 1.0f);
  glutSolidSphere(Physics::ball->getRadius(), 30, 30);
  glPopMatrix();

  //cout << m[12] << " " << m[13] << " " << m[14] << endl;
  //system("cls");

  // Line
  glDisable(GL_LIGHTING);
  glColor3f(1.0f, 0.8 *Physics::v_cur.getY() / ceiling, 0.8 * Physics::v_cur.getY() / ceiling);
  glBegin(GL_LINES);
  glVertex3f(Physics::v_cur.getX(), 0.0, Physics::v_cur.getZ());
  glVertex3f(Physics::v_cur.getX(), Physics::v_cur.getY(), Physics::v_cur.getZ());
  glEnd();

  // XZ position cue
  temp.makeTranslate(Physics::v_cur.getX(), 0.0, Physics::v_cur.getZ());
  temp.transpose();
  glPushMatrix();
  glMultMatrixd(temp.getPointer());
  glColor3f(0.0f, 1.0f, 1.0f);
  glutSolidSphere(2, 30, 30);
  glPopMatrix();
  glEnable(GL_LIGHTING);

  /*
  glDisable(GL_LIGHTING);
  glBegin(GL_POINTS);
  glColor3f(1.0f, 0.0f, 0.0f);
  glVertex3f(m[12], 1.5f , m[14]);
  glEnd();
  glEnable(GL_LIGHTING);
  */

  // Wall of bricks
  /*
  for (int i = 0; i < 120; i++) {
    Physics::brickrb[i]->getMotionState()->getWorldTransform(t);
    t.getOpenGLMatrix(m);
    glPushMatrix();
    glMultMatrixf(m);
    glColor3f(i % 20, i % 5, i % 10);
    glScalef(1.0, (float)16 / 20, (float)8 / 20);
    glutSolidCube(20);
    glPopMatrix();
  }
  */

  /*
  // String
  for (int i = 0; i < Physics::ddw->getSoftBodyArray().size(); i++) {
    sb = Physics::ddw->getSoftBodyArray()[i];
    glColor3f(1.0f, 1.0f, 1.0f);
    glBegin(GL_TRIANGLES);
    for (int j = 0; j < sb->m_faces.size(); j++) {
      for (int k = 0; k < 3; k++) {
        glVertex3f(sb->m_faces[j].m_n[k]->m_x.x(), sb->m_faces[j].m_n[k]->m_x.y(),
          sb->m_faces[j].m_n[k]->m_x.z());
      }
    }
    glEnd();

    glColor3f(1.0f, 1.0f, 0.0f);
    glBegin(GL_LINES);
    for (int j = 0; j < sb->m_links.size(); j++) {
      for (int k = 0; k < 2; k++) {
        glVertex3f(sb->m_links[j].m_n[k]->m_x.x(), sb->m_links[j].m_n[k]->m_x.y(),
          sb->m_links[j].m_n[k]->m_x.z());
      }
    }
    glEnd();
  }
  */

  glFlush();
  glutSwapBuffers();
}

Vector3 raytrace(int x, int y) {
  int vp[4];
  double mm[16];
  double pm[16];
  float winx, winy, winz;
  double dx, dy, dz;

  glGetDoublev(GL_MODELVIEW_MATRIX, mm);
  glGetDoublev(GL_PROJECTION_MATRIX, pm);
  glGetIntegerv(GL_VIEWPORT, vp);

  winx = (float)x;
  winy = (float)vp[3] - (float)y;
  glReadPixels(x, int(winy), 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &winz);

  gluUnProject(winx, winy, winz, mm, pm, vp, &dx, &dy, &dz);

  return Vector3(dx, dy, dz);
}

void mouse(int k, int s, int x, int y) {
  /*
  m1_down = (s == GLUT_DOWN);
  if (k == GLUT_LEFT_BUTTON && s == GLUT_DOWN) {
    v_prev = raytrace(x, y);
    btVector3 from((btScalar)cam.e.x, (btScalar)cam.e.y, (btScalar)cam.e.z);
    Vector3 v = (v_prev - cam.e);
    v.scale(10000);

    btCollisionWorld::ClosestRayResultCallback rc(btVector3(cam.e.x, cam.e.y, cam.e.z),
      btVector3(v.x, v.y, v.z));
    Physics::ddw->rayTest(btVector3(cam.e.x, cam.e.y, cam.e.z),
      btVector3(v.x, v.y, v.z), rc);

    if (rc.hasHit()) {
      rb = (btRigidBody*)(rc.m_collisionObject);

      if (!(rb->isStaticObject() || rb->isKinematicObject()))// || rb == Physics::ballrb[0])
      {
        //if (rb == Physics::ballrb[0]) cur_select = true;
        pos = rc.m_hitPointWorld;
        rb->setActivationState(DISABLE_DEACTIVATION);

        btVector3 lpiv = rb->getCenterOfMassTransform().inverse() * pos;

        btTransform t;
        t.setIdentity();
        t.setOrigin(lpiv);
        btGeneric6DofConstraint* n_con = new btGeneric6DofConstraint(*rb, t, false);
        n_con->setLinearLowerLimit(btVector3(0, 0, 0));
        n_con->setLinearUpperLimit(btVector3(0, 0, 0));
        n_con->setAngularLowerLimit(btVector3(0, 0, 0));
        n_con->setAngularUpperLimit(btVector3(0, 0, 0));

        Physics::ddw->addConstraint(n_con);
        con = n_con;

        n_con->setParam(BT_CONSTRAINT_STOP_CFM, 0.8f, 0);
        n_con->setParam(BT_CONSTRAINT_STOP_CFM, 0.8f, 1);
        n_con->setParam(BT_CONSTRAINT_STOP_CFM, 0.8f, 2);
        n_con->setParam(BT_CONSTRAINT_STOP_CFM, 0.8f, 3);
        n_con->setParam(BT_CONSTRAINT_STOP_CFM, 0.8f, 4);
        n_con->setParam(BT_CONSTRAINT_STOP_CFM, 0.8f, 5);

        n_con->setParam(BT_CONSTRAINT_STOP_ERP, 0.1f, 0);
        n_con->setParam(BT_CONSTRAINT_STOP_ERP, 0.1f, 1);
        n_con->setParam(BT_CONSTRAINT_STOP_ERP, 0.1f, 2);
        n_con->setParam(BT_CONSTRAINT_STOP_ERP, 0.1f, 3);
        n_con->setParam(BT_CONSTRAINT_STOP_ERP, 0.1f, 4);
        n_con->setParam(BT_CONSTRAINT_STOP_ERP, 0.1f, 5);

        dis = (pos - from).length();
      }
    }
  }
  else {
    if (con != NULL) {
      rb->forceActivationState(ACTIVE_TAG);
      rb->setDeactivationTime(0.0f);
      rb = NULL;

      Physics::ddw->removeConstraint(con);
      delete con;
      con = NULL;
    }
    //cur_select = false;
  }
  */
}

void motion(int x, int y) {
  /*
  v_cur = raytrace(x, y);
  if (m1_down) {
    Vector3 dir;
    btGeneric6DofConstraint* n_con = static_cast<btGeneric6DofConstraint*>(con);
    if (n_con) {
      btVector3 to = btVector3(v_cur.x, v_cur.y, v_cur.z);
      btVector3 from = btVector3(cam.e.x, cam.e.y, cam.e.z);
      btVector3 piv = n_con->getFrameOffsetA().getOrigin();
      btVector3 n_piv;
      btVector3 v = to - from;
      v.normalize();
      v *= dis;
      n_piv = from + v;
      n_con->getFrameOffsetA().setOrigin(n_piv);
    }
  }
  */
}


void keyboard(unsigned char k, int, int) {
  switch (k) {
  case 'w':
    cam.move('w');
    break;
  case 'a':
    cam.move('a');
    break;
  case 's':
    cam.move('s');
    break;
  case 'd':
    cam.move('d');
    break;
  case ' ':
    Physics::v_cur = btVector3(0,50,0);
    cam = Camera(Vector3(0.0, 100.0, 150.0), Vector3(0.0, 50.0, 0.0), Vector3(0.0, 1.0, 0.0));
    physics.init();
    break;
  }
}


void idle() {
  NuiSkeletonGetNextFrame(0, &frame);
  for (int i = 0; i < 6; i++) {
    if (frame.SkeletonData[i].eTrackingState == NUI_SKELETON_TRACKED) {
      pos_r = btVector3(
        frame.SkeletonData[i].SkeletonPositions[NUI_SKELETON_POSITION_HAND_RIGHT].x,
        frame.SkeletonData[i].SkeletonPositions[NUI_SKELETON_POSITION_HAND_RIGHT].y,
        frame.SkeletonData[i].SkeletonPositions[NUI_SKELETON_POSITION_HAND_RIGHT].z);

      pos_l = btVector3(
        frame.SkeletonData[i].SkeletonPositions[NUI_SKELETON_POSITION_HAND_LEFT].x,
        frame.SkeletonData[i].SkeletonPositions[NUI_SKELETON_POSITION_HAND_LEFT].y,
        frame.SkeletonData[i].SkeletonPositions[NUI_SKELETON_POSITION_HAND_LEFT].z);
    
      /*
      Values of x range from approximately -2.2 to +2.2.
      Values of y range from approximately -1.6 to +1.6.
      Values of z range from 0.0 to 4.0.
      */
      double boxhs = 0.20;
      double scale = 5.0;

      // Right Hand
      btVector3 v = pos_r - start;
      if (v.length() > boxhs) {
        bool down = v.getY() < -boxhs;
        bool up = v.getY() > boxhs;
        bool left = v.getX() < -boxhs;
        bool right = v.getX() > boxhs;
        bool front = v.getZ() < -boxhs;
        bool back = v.getZ() > boxhs;

        if(up && Physics::v_cur.getY() < ceiling) 
          Physics::v_cur = Physics::v_cur + btVector3(0, scale*(v.getY() - boxhs), 0);
        else if(down && Physics::v_cur.getY() > Physics::ball->getRadius()) 
          Physics::v_cur = Physics::v_cur + btVector3(0, scale*(v.getY() + boxhs), 0);

        if(left && Physics::v_cur.getX() > -1000)
          Physics::v_cur = Physics::v_cur + btVector3(scale*(v.getX() + boxhs), 0, 0);
        else if(right && Physics::v_cur.getX() < 1000)
          Physics::v_cur = Physics::v_cur + btVector3(scale*(v.getX() - boxhs), 0, 0);

        if(front && Physics::v_cur.getZ() > -1000)
          Physics::v_cur = Physics::v_cur + btVector3(0, 0, scale *scale*(v.getZ() + boxhs));
        else if(back && Physics::v_cur.getZ() < 1000)
          Physics::v_cur = Physics::v_cur + btVector3(0, 0, scale*(v.getZ() - boxhs));

        float frame = 90.0f;
        double x, y, z;
        if(Physics::v_cur.getX() < cam.d.x - frame) x = Physics::v_cur.getX() + frame;
        else if(Physics::v_cur.getX() > cam.d.x + frame) x = Physics::v_cur.getX() - frame;
        else x = cam.d.x;

        if(Physics::v_cur.getY() < cam.d.y - frame) y = Physics::v_cur.getY() + frame;
        else if(Physics::v_cur.getY() > cam.d.y + frame) y = Physics::v_cur.getY() - frame;
        else y = cam.d.y;

        if (Physics::v_cur.getZ() < cam.d.z - (float)frame / 3.0f) z = Physics::v_cur.getZ() + (float)frame / 3.0f;
        else if (Physics::v_cur.getZ() > cam.d.z + (float)frame / 3.0f) z = Physics::v_cur.getZ() - (float)frame / 3.0f;
        else z = cam.d.z;

        cam = Camera(Vector3(x, y + 100.0, z + 150.0), 
          Vector3(x, y, z), Vector3(0.0, 1.0, 0.0));
      }
    
      select_prev = Physics::cur_select;
      // Left Hand
      btVector3 vl = pos_l - start_l;
      if(vl.getX() < -boxhs) {
        Physics::cur_select = true;
      } else {
        Physics::cur_select = false;
      }
      //Physics::gravitize = vl.getY() > boxhs*boxhs;
      if(vl.getY() > boxhs * 2) {
        wireg = true; //?
      }
      else {
        wireg = false;
        added = false;
      }
        //Physics::ddw->setGravity(btVector3(0.0f, -9.81f, 0.0f)) : Physics::ddw->setGravity(btVector3(0.0f, 0.0f, 0.0f));
    
      if(!select_prev && Physics::cur_select) {
        if (beg2 && !beg) {
          beg2 = false;
        }
        if (beg) {
          beg = false;
        }
      }

      //vector<Segment> copy;

      if(Physics::cur_select) {
        if(beg2) {
          lining = true;
        } else {
          float tomax;
          float tomin;
          float closer;
          float closest = 50;
          snappt;
          for (vector<Segment*>::iterator it = Physics::wires.begin(); it != Physics::wires.end(); ++it) {
            tomax = (Physics::v_cur - (*it)->max).length();
            tomin = (Physics::v_cur - (*it)->min).length();
            if(tomin < tomax) {
              closer = tomin;
              tempsnap = (*it)->min;
            } else {
              closer = tomax;
              tempsnap = (*it)->max;
            }

            if(closer > Physics::v_cur.getY() ) {
              closer = Physics::v_cur.getY();
              tempsnap = btVector3(Physics::v_cur.getX(),0,Physics::v_cur.getZ());
            }
            
            if(closer < closest) {
              closest = closer;
              snappt = tempsnap;
            }
            
          }

          if (closest > snapdis)
            snap = false;
          else
            snap = true;

          if (snapped) {
            lining = true;
          }
          else {
            lining = false;
            if (snap) {
              snapped = true;
              Physics::v_cur = Physics::v_prev = snappt;
              // snapfin = true;
              /*for(int i = 0; i < Physics::wires.size(); i++) {
                copy.push_back(Physics::wires[i]);
              }*/
              //lining = true;
            }
          }
        }
      } else {
        if(snapfin) {
          /*float tomax;
          float tomin;
          float closer;
          float closest = 50;
          for (vector<Segment>::iterator it = copy.begin(); it != copy.end(); ++it) {
            tomax = (Physics::v_cur - (*it).max).length();
            tomin = (Physics::v_cur - (*it).min).length();
            if (tomin < tomax) {
              closer = tomin;
              tempsnap = (*it).min;
            }
            else {
              closer = tomax;
              tempsnap = (*it).max;
            }

            if (closer > Physics::v_cur.getY()) {
              closer = Physics::v_cur.getY();
              tempsnap = btVector3(Physics::v_cur.getX(), 0, Physics::v_cur.getZ());
            }

            if (closer < closest) {
              closest = closer;
              snappt = tempsnap;
            }*/


            if(snappt.getX()!=0 && snappt.getY()!=0 && snappt.getZ()!=0) {
              Physics::v_cur = snappt;
            }
          //}
        }
        snap = false;
        snapped = false;
      }



      // Line drawing
      f_count++;
      if (f_count % f_skip == 0) {
        //cout << (vl.getY() > boxhs*boxhs) << " " << Physics::ddw->getGravity().getY() << endl;
        f_count = 0;
        if((Physics::cur_select || snapfin) && lining) Physics::makeLine();
        else {
          Physics::prev_segment = nullptr;
          Physics::v_prev = Physics::v_cur;
          f_count = f_skip -1;
        }
      }
      if(snapfin)
        snapfin = false;
     
     /*
      bool up = (Physics::v_cur.getY() - start.getY()) > boxhs;
      bool left = (Physics::v_cur.getX() - start.getX()) < -boxhs;
      bool right = (Physics::v_cur.getX() - start.getX()) > boxhs;
      bool down = (Physics::v_cur.getY() - start.getY()) < -boxhs;
      bool front = (Physics::v_cur.getZ() - start.getZ()) < -boxhs;
      bool back = (Physics::v_cur.getZ() - start.getZ()) > boxhs;

      if(up)
        cursor = cursor + Vector3(0, (Physics::v_cur.getY() - start.getY()) - boxhs, 0);
      else if (down && cursor.y > Physics::ball->getRadius())
        cursor = cursor + Vector3(0, (Physics::v_cur.getY() - start.getY()) + boxhs, 0);
      */
      //Physics::move(up, left, right, down, front, back);
    }

    /*
    if(cursor.y < ceiling)
    cursor = cursor + Vector3(0.0, .01, 0.0);

    int frame = 90;
    double x, y, z;
    if (cursor.x < cam.d.x - frame) x = cursor.x + frame;
    else if (cursor.x > cam.d.x + frame) x = cursor.x - frame;
    else x = cam.d.x;

    if (cursor.y < cam.d.y - frame) y = cursor.y + frame;
    else if (cursor.y > cam.d.y + frame) y = cursor.y - frame;
    else y = cam.d.y;

    if (cursor.z < cam.d.z - (float)frame / 3.0f) z = cursor.z + (float)frame / 3.0f;
    else if (cursor.z > cam.d.z + (float)frame /3.0f) z = cursor.z - (float)frame /3.0f;
    else z = cam.d.z;

    cam = Camera(Vector3(x, y + 100.0, z + 150.0),
      Vector3(x, y, z), Vector3(0.0, 1.0, 0.0));
      */
  }

  //cout << "Starting position: " << start.getX() << " " << start.getY() << " " << start.getZ() << endl;
  //cout << Physics::v_cur.getX() - start.getX() << " " << Physics::v_cur.getY() - start.getY() << " " << Physics::v_cur.getZ() - start.getZ() << endl;
  //system("cls");

  Physics::ddw->stepSimulation(1 / 100.f, 10);
  display();
}

int main(int argc, char* argv[]) {
  float specular[] = { 1.0, 1.0, 1.0, 1.0 };
  float shininess[] = { 100.0 };
  //float position[] = { 0.0, 100.0, 0.0, 0.0 };  // lightsource position

  glutInit(&argc, argv);
  glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
  glutInitWindowSize(width, height);
  glutCreateWindow("Wire Sculptures");

  glEnable(GL_DEPTH_TEST);
  glClear(GL_DEPTH_BUFFER_BIT);
  glClearColor(0.0, 0.0, 0.0, 0.0);
  glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
  glCullFace(GL_BACK);
  glShadeModel(GL_SMOOTH);
  glMatrixMode(GL_PROJECTION);

  glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, specular);
  glMaterialfv(GL_FRONT_AND_BACK, GL_SHININESS, shininess);
  glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
  glEnable(GL_COLOR_MATERIAL);

  sl.setPosition(slp);
  sl.setSpotDirection(sld);
  sl.setSpecular(sls);
  sl.setSpotExponent(sle);
  sl.setAmbient(sla);

  glEnable(GL_LIGHTING);
  sl.on();
  glEnable(GL_NORMALIZE);

  // Point 
  glEnable(GL_POINT_SMOOTH);
  glHint(GL_POINT_SMOOTH_HINT, GL_NICEST);
  glPointSize(10);

  glutDisplayFunc(display);
  glutReshapeFunc(reshape);
  glutIdleFunc(idle);
  glutKeyboardFunc(keyboard);
  glutMouseFunc(mouse);
  glutMotionFunc(motion);

  Physics::v_cur = btVector3(0,50,0);

  NuiInitialize(NUI_INITIALIZE_FLAG_USES_SKELETON);
  cout << "Please get in a position where the Kinect(TM) can recognize you..." << endl;
  while (frame.SkeletonData[0].eTrackingState != NUI_SKELETON_TRACKED &&
    frame.SkeletonData[1].eTrackingState != NUI_SKELETON_TRACKED &&
    frame.SkeletonData[2].eTrackingState != NUI_SKELETON_TRACKED &&
    frame.SkeletonData[3].eTrackingState != NUI_SKELETON_TRACKED &&
    frame.SkeletonData[4].eTrackingState != NUI_SKELETON_TRACKED &&
    frame.SkeletonData[5].eTrackingState != NUI_SKELETON_TRACKED) {
    NuiSkeletonGetNextFrame(0, &frame);
   // break; // Disable Kinect(TM)
  }

  for(int i = 0; i < 6; i++) {
    if (frame.SkeletonData[i].eTrackingState == NUI_SKELETON_TRACKED) {
      start = btVector3(frame.SkeletonData[i].SkeletonPositions[NUI_SKELETON_POSITION_HAND_RIGHT].x,
        frame.SkeletonData[i].SkeletonPositions[NUI_SKELETON_POSITION_HAND_RIGHT].y,
        frame.SkeletonData[i].SkeletonPositions[NUI_SKELETON_POSITION_HAND_RIGHT].z);

      start_l = btVector3(frame.SkeletonData[i].SkeletonPositions[NUI_SKELETON_POSITION_HAND_LEFT].x,
        frame.SkeletonData[i].SkeletonPositions[NUI_SKELETON_POSITION_HAND_LEFT].y,
        frame.SkeletonData[i].SkeletonPositions[NUI_SKELETON_POSITION_HAND_LEFT].z);
    }
  }
  cout << "Right hand starting position: " << start.getX() << " " << start.getY() << " " << start.getZ() << endl;
  cout << "Left hand starting position: " << start_l.getX() << " " << start_l.getY() << " " << start_l.getZ() << endl;
  Physics::v_prev = start;

  physics.init();

  glutMainLoop();

  NuiShutdown();
  //delete Physics::ball;
  //delete Physics::ballrb;
  delete Physics::floor;
  //delete Physics::brick;
  delete Physics::anchor;
  //delete Physics::rope;
  //delete Physics::roperb;
  //delete Physics::brickrb;
  delete Physics::dsbs;
  delete Physics::sics;
  delete Physics::cd;
  delete Physics::dcc;
  delete Physics::bp;
  delete Physics::ddw;

  return 0;
}