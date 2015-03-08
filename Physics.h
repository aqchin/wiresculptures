#pragma once
#pragma warning(disable : 4305)

#include "btBulletDynamicsCommon.h"
#include "btBulletCollisionCommon.h"

#include "BulletSoftBody/btSoftRigidDynamicsWorld.h"
#include "BulletSoftBody/btDefaultSoftBodySolver.h"
#include "BulletSoftBody/btSoftBodyHelpers.h"
#include "BulletSoftBody/btSoftBodyRigidBodyCollisionConfiguration.h"

#include <iostream>
#include <vector>

using namespace std;

struct Segment {
  btVector3 max, min;
  btCylinderShape* cs;
  btVector3 dis;
  btTransform frame;
  btQuaternion rot;
  btRigidBody* rb;
};

class Physics {
public:

  static btScalar wirem;
  static bool cur_select;
  static Segment* prev_segment;

  static btVector3 v_prev, v_cur;
  static vector<Segment*> wires;

  static btBroadphaseInterface* bp;
  static btDefaultCollisionConfiguration* dcc;
  static btCollisionDispatcher* cd;
  static btSequentialImpulseConstraintSolver* sics;
  static btSoftRigidDynamicsWorld* ddw;
  static btDefaultSoftBodySolver* dsbs;

  static btSphereShape* ball;
  static btRigidBody** ballrb;
  static btCollisionShape* floor;
  static btBoxShape* brick;
  static btBoxShape* anchor;
  //static btBoxShape* rope;
  //static btRigidBody** roperb;
  static btRigidBody** brickrb;
  //static vector<btFixedConstraint*> cons;
  static vector<btGeneric6DofConstraint*> cons;

  static void init();
  static void deinit();
  static void makeLine();
  static void move(bool, bool, bool, bool, bool, bool);
};