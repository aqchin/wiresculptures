/*
* Credit for material from bullet engine physics tutorial to
*   THECPLUSPLUSGUY @ https://www.youtube.com/watch?v=d7_lJJ_j2NE
*/
#include "Physics.h"

btBroadphaseInterface* Physics::bp;
btDefaultCollisionConfiguration* Physics::dcc;
btCollisionDispatcher* Physics::cd;
btSequentialImpulseConstraintSolver* Physics::sics;
btSoftRigidDynamicsWorld* Physics::ddw;
btDefaultSoftBodySolver* Physics::dsbs;
btSphereShape* Physics::ball;
btRigidBody** Physics::ballrb = new btRigidBody*[1];
btRigidBody** Physics::brickrb = new btRigidBody*[120];
btCollisionShape* Physics::floor;
btBoxShape* Physics::brick;
btBoxShape* Physics::anchor;
btVector3 Physics::v_prev;
btVector3 Physics::v_cur;
bool Physics::cur_select;
Segment* Physics::prev_segment;
vector<Segment*> Physics::wires;
//vector<btFixedConstraint*> Physics::cons;
vector<btGeneric6DofConstraint*> Physics::cons;
//vector<btRigidBody*> Physics::wires;
//btBoxShape* Physics::rope;
//btRigidBody** Physics::roperb = new btRigidBody*[4];

void Physics::init() {
  bp = new btDbvtBroadphase();
  dcc = new btSoftBodyRigidBodyCollisionConfiguration();
  cd = new btCollisionDispatcher(dcc);
  sics = new btSequentialImpulseConstraintSolver();
  dsbs = new btDefaultSoftBodySolver();
  ddw = new btSoftRigidDynamicsWorld(cd, bp, sics, dcc, dsbs);
  ddw->setGravity(btVector3(0, -9.81f, 0));

  floor = new btStaticPlaneShape(btVector3(0, 1, 0), 1);
  ball = new btSphereShape(5);
  brick = new btBoxShape(btVector3(10, 8, 4));
  anchor = new btBoxShape(btVector3(1, 1, 1));
  //rope = new btBoxShape(btVector3(0.5f, 5.0f, 0.5f));
  
  // Floor
  btDefaultMotionState* floorms = new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), 
    btVector3(0, -1, 0)));
  btRigidBody::btRigidBodyConstructionInfo
    floorrbci(0, floorms, floor, btVector3(0, 0, 0));
  floorrbci.m_restitution = 1.0f;
  floorrbci.m_friction = 0.5f;
  btRigidBody* floorrb = new btRigidBody(floorrbci);
  ddw->addRigidBody(floorrb);
  
  // Ball
  btDefaultMotionState* ballms =
    new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), btVector3(0, 50, 0)));
  btScalar mass = 1;
  btVector3 balli(0, 0, 0);
  ball->calculateLocalInertia(mass, balli);
  btRigidBody::btRigidBodyConstructionInfo ballrbci(mass, ballms, ball, balli);
  ballrb[0] = new btRigidBody(ballrbci);
  ddw->addRigidBody(ballrb[0]);

  // Wall of bricks
  brick->setMargin(0.001f);
  float x = -40.0;
  float y = 8.0;
  float z = -50.0;
  btScalar brickm = (btScalar)0.1;
  btVector3 bricki(0, 0, 0);
  brick->calculateLocalInertia(brickm, bricki);
  for (int r = 0; r < 8; r++) {
    y = (float)(8.0 + 16.0 * r);
    for (int c = 0; c < 15; c++) {
      x = (float)(-140.0 + 20.0 * c);
      btDefaultMotionState* brickms = 
        new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), btVector3(x, y, z)));
      btRigidBody::btRigidBodyConstructionInfo brickrbci(brickm, brickms, brick, bricki);
      brickrb[c + 15*r] = new btRigidBody(brickrbci);
      ddw->addRigidBody(brickrb[c+15*r]);
    }
  }
  
  /*
  // String
  btSoftBody* sb = btSoftBodyHelpers::CreateRope(ddw->getWorldInfo(),
    btVector3(0, 150, 0), btVector3(0, 60, 0), 10, 1);
  sb->m_cfg.viterations = 20;
  sb->m_cfg.piterations = 20;
  sb->setTotalMass(.75);
  sb->appendAnchor(sb->m_nodes.size() - 1, ballrb[0]);
  ddw->addSoftBody(sb);
  */

  /*
  // Anchor
  btScalar anchorm = 0;
  btVector3 anchori(0, 0, 0);
  btDefaultMotionState* anchorms = new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1),
    btVector3(0, 100 + 1, 0)));
  btRigidBody::btRigidBodyConstructionInfo
    anchorrbci(0, anchorms, anchor, btVector3(0, 0, 0));
  btRigidBody* anchorrb = new btRigidBody(anchorrbci);
  ddw->addRigidBody(anchorrb);

  // Rope
  btScalar ropem = 1;
  btVector3 ropei(0, 0, 0);
  rope->calculateLocalInertia(ropem, ropei);
  for(int i = 0; i < 4; i++) {
    btDefaultMotionState* ropems =
      new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), btVector3(0, 95 - 10*i, 0)));
    btRigidBody::btRigidBodyConstructionInfo roperbci(ropem, ropems, rope, ropei);
    roperb[i] = new btRigidBody(roperbci);
    ddw->addRigidBody(roperb[i]);
  }

   btTransform fia, fib, fia1, fib1, fia2, fib2, fia3, fib3, fia4, fib4;
   fia = btTransform::getIdentity();
   fib = btTransform::getIdentity();
   fia.setOrigin(btVector3(0, 100, 0)); // anchor 0
   fib.setOrigin(btVector3(0, 100, 0));

   fia1 = btTransform::getIdentity();
   fib1 = btTransform::getIdentity();
   fia1.setOrigin(btVector3(0, 90, 0)); // joint 1
   fib1.setOrigin(btVector3(0, 90, 0));

   fia2 = btTransform::getIdentity();
   fib2 = btTransform::getIdentity();
   fia2.setOrigin(btVector3(0, 80, 0)); // joint 2
   fib2.setOrigin(btVector3(0, 80, 0));

   fia3 = btTransform::getIdentity();
   fib3 = btTransform::getIdentity();
   fia3.setOrigin(btVector3(0, 70, 0)); // joint 3
   fib3.setOrigin(btVector3(0, 70, 0));

   fia4 = btTransform::getIdentity();
   fib4 = btTransform::getIdentity();
   fia4.setOrigin(btVector3(0, 60, 0)); // joint 4
   fib4.setOrigin(btVector3(0, 60, 0));

   btGeneric6DofConstraint* gdc = new btGeneric6DofConstraint(*anchorrb, *roperb[0], fia, fib, false);
   btGeneric6DofConstraint* gdc1 = new btGeneric6DofConstraint(*roperb[0], *roperb[1], fia1, fib1, false);
   btGeneric6DofConstraint* gdc2 = new btGeneric6DofConstraint(*roperb[1], *roperb[2], fia2, fib2, false);
   btGeneric6DofConstraint* gdc3 = new btGeneric6DofConstraint(*roperb[2], *roperb[3], fia3, fib3, false);
   btGeneric6DofConstraint* gdc4 = new btGeneric6DofConstraint(*roperb[3], *ballrb[0], fia4, fib4, false);

   gdc->setAngularLowerLimit(btVector3(-SIMD_PI, 0, 0));
   gdc->setAngularUpperLimit(btVector3(1.5, 0, 0));
   gdc1->setAngularLowerLimit(btVector3(-SIMD_PI, 0, 0));
   gdc1->setAngularUpperLimit(btVector3(1.5, 0, 0));
   gdc2->setAngularLowerLimit(btVector3(-SIMD_PI, 0, 0));
   gdc2->setAngularUpperLimit(btVector3(1.5, 0, 0));
   gdc3->setAngularLowerLimit(btVector3(-SIMD_PI, 0, 0));
   gdc3->setAngularUpperLimit(btVector3(1.5, 0, 0));
   gdc4->setAngularLowerLimit(btVector3(-SIMD_PI, 0, 0));
   gdc4->setAngularUpperLimit(btVector3(1.5, 0, 0));

   ddw->addConstraint(gdc);
   ddw->addConstraint(gdc1);
   ddw->addConstraint(gdc2);
   ddw->addConstraint(gdc3);
   ddw->addConstraint(gdc4);
   */
}

void Physics::makeLine() {
  Segment* s = new Segment;
  //cout << "Making a line" << endl;
  s->dis = v_cur - v_prev;
  s->max = v_cur;
  s->min = v_prev;
  s->cs = new btCylinderShapeZ(btVector3(1.0f, 0.0f, s->dis.length() - s->dis.length()/10));
  s->cs->setMargin(0.01f);
  //s.cs = new btCylinderShape(btVector3(0.5f, 0.5f, s.dis.length()));
  //btBoxShape* wire = new btBoxShape(btVector3(0.05f, dis.length(), 0.05f));

  btScalar mass = 0.0;//(btScalar)0.5;
  //btScalar mass = 0.5;
  btVector3 inertia(0, 0, 0);
  s->cs->calculateLocalInertia(mass, inertia);

  btVector3 y = btVector3(0.0f, 0.0f, 1.0f);
  //btVector3 y = btVector3(0.0f, 0.0f, 1.0f);
  btVector3 v = (y.cross(s->dis));

  float w = sqrt((y.length() * y.length()) * (s->dis.length() * s->dis.length())) + y.dot(s->dis);
  if(w == 0) {
    v.setY(1);
  }
  s->rot = btQuaternion(v.getX(), v.getY(), v.getZ(), w);
  if(w!=0) s->rot.normalize();
  

  //cout << s.rot.getX() << " " << s.rot.getY() << " " << s.rot.getZ() << " " << s.rot.getW() << endl;

  s->frame = btTransform(s->rot, v_prev);
  btDefaultMotionState* ms = new btDefaultMotionState(s->frame);// + (0.5)*s.dis));

  btRigidBody::btRigidBodyConstructionInfo rbci(mass, ms, s->cs, inertia);
  s->rb = new btRigidBody(rbci);
  s->rb->setActivationState(DISABLE_DEACTIVATION);
  ddw->addRigidBody(s->rb);

  //s.rb->setGravity(btVector3(0,0,0));
  //s.rb->setLinearVelocity(btVector3(0,0,0));
  if(s->dis.length() > 0) {
    wires.push_back(s);
    cout << "lollol" << endl;
  }

  if(prev_segment == nullptr) {
    //cout << "Nope" << endl;
  }
  else {
    //cout << "Constraining" << endl;
    //prev_segment->rb->setLinearVelocity(btVector3(0,0,0));
    //btTransform fia = btTransform(btQuaternion(), btVector3(0.0, 0.0, 0.0));
    //btTransform fib = btTransform(btQuaternion(), btVector3(0.0, s.dis.length(), 0.0));
    //btTransform fia = btTransform(s.rot, btVector3(0.0, 0.0, 0.0));
    //btTransform fib = btTransform(prev_segment->rot, btVector3(0.0, s.dis.length()-.25, 0.0));
    //btTransform fia = btTransform(s.rot, v_cur);
    //btTransform fib = btTransform(prev_segment->rot, v_prev);
    /*
    btVector3 v = v_prev.cross(v_cur);
    float wx = sqrt((v_cur.length() * v_cur.length()) * (v_prev.length() * v_prev.length())) + v_cur.dot(v_prev);
    if(wx < 0) v.setY(1);

    btQuaternion q = btQuaternion(v.getX(), v.getY(), v.getZ(), wx);
    if(wx != 0) q.normalize();
    */
    btTransform fia = btTransform::getIdentity();
    fia.setIdentity();
    fia.setOrigin(btVector3(0,0,0));
    //btTransform fib = btTransform(q, btVector3(0,prev_segment->dis.length(),0));
    //btTransform fib = prev_segment->rb->getCenterOfMassTransform().inverse() * s.rb->getCenterOfMassTransform();
    btTransform fib = btTransform::getIdentity();
    fib.setIdentity();
    fib.setOrigin(btVector3(0,0,prev_segment->dis.length()));
  
    //cons.push_back(new btFixedConstraint(*s.rb, *prev_segment->rb, fia, fib));
    //btFixedConstraint* dof = new btFixedConstraint(*s.rb, *prev_segment->rb, fia, fib);
    btGeneric6DofConstraint* dof = new btGeneric6DofConstraint(*s->rb, *prev_segment->rb, fia, fib, true);
    //btGeneric6DofConstraint* dof = new btGeneric6DofConstraint(*s.rb, *prev_segment->rb, btTransform::getIdentity(), btTransform::getIdentity(), true);
    
    cout<< "hiiiii" << endl;

    dof->setAngularLowerLimit(btVector3(0, 0, 0));
    dof->setAngularUpperLimit(btVector3(0, 0, 0));
    dof->setLinearLowerLimit(btVector3(0, 0, 0));
    dof->setLinearUpperLimit(btVector3(0, 0, 0));

    if (prev_segment->dis.length() > 0)
      cons.push_back(dof);
    //ddw->addConstraint(dof, true);
  }

  prev_segment = s;

  v_prev = v_cur;
  //wires.push_back(new btRigidBody(wirerbci));
}

void Physics::move(bool up, bool left, bool right, bool down, bool front, bool back) {
  //cout << "moving" << endl;
  //cout << v_cur.getX() << " " << v_cur.getY() << " " << v_cur.getZ() << endl;
  //cout << (up || left || right || down || front || back) << endl;
  //system("cls");
  float scale = 0.1;
  if (up && front && left)
    ballrb[0]->translate(btVector3(-scale, scale, -scale));
  else if (up && front && right)
    ballrb[0]->translate(btVector3(scale, scale, -scale));
  else if (up && back && left)
    ballrb[0]->translate(btVector3(-scale, scale, scale));
  else if (up && back && right)
    ballrb[0]->translate(btVector3(scale, scale, scale));

  else if (down && front && left)
    ballrb[0]->translate(btVector3(-scale, -scale, -scale));
  else if (down && front && right)
    ballrb[0]->translate(btVector3(scale, -scale, -scale));
  else if (down && back && left)
    ballrb[0]->translate(btVector3(-scale, -scale, scale));
  else if (down && back && right)
    ballrb[0]->translate(btVector3(scale, -scale, scale));

  else if (up && front)
    ballrb[0]->translate(btVector3(0, scale, -scale));
  else if (up && left)
    ballrb[0]->translate(btVector3(-scale, scale, 0));
  else if (up && right)
    ballrb[0]->translate(btVector3(scale, scale, 0));
  else if (up && back)
    ballrb[0]->translate(btVector3(0, scale, scale));

  else if (down && front)
    ballrb[0]->translate(btVector3(0, -scale, -scale));
  else if (down && left)
    ballrb[0]->translate(btVector3(-scale, -scale, 0));
  else if (down && right)
    ballrb[0]->translate(btVector3(scale, -scale, 0));
  else if (down && back)
    ballrb[0]->translate(btVector3(0, -scale, scale));

  else if (front && left)
    ballrb[0]->translate(btVector3(-scale, 0, -scale));
  else if (front && right)
    ballrb[0]->translate(btVector3(scale, 0, -scale));
  else if (back && left)
    ballrb[0]->translate(btVector3(-scale, 0, scale));
  else if (back && right)
    ballrb[0]->translate(btVector3(scale, 0, scale));

  else if (up)
    ballrb[0]->translate(btVector3(0, scale, 0));
  else if (down)
    ballrb[0]->translate(btVector3(0, -scale, 0));
  else if (front)
    ballrb[0]->translate(btVector3(0, 0, -scale));
  else if (back)
    ballrb[0]->translate(btVector3(0, 0, scale));
  else if (left)
    ballrb[0]->translate(btVector3(-scale, 0, 0));
  else if (right)
    ballrb[0]->translate(btVector3(scale, 0, 0));
  else cout << "Nothing happened" << endl;
}

void Physics::deinit() {}