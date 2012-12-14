#ifndef BALL_H
#define BALL_H

#include "cvec.h"

static const double AIR_RESISTANCE = 0.05;
static const double TIME = 0.25;

class Ball
{
  float mass;
  Cvec3 accel; //accel
	Cvec3 pos; //current position
	Cvec3 prevPos; //old position
	Cvec3 normal; // an accumulated normal (i.e. non normalized), used for OpenGL soft shading
  bool canMove;

public:
  Ball() {}
	Ball(Cvec3 pos) : mass(1), accel(Cvec3()), pos(pos), prevPos(pos), normal(Cvec3()), canMove(true) {}

	void resetAccel() {
    accel = Cvec3();
  }

  void setAccel(Cvec3 a) {
    accel = a;
  }

	void resetNormal() {
    normal = Cvec3();
  }

  void setNormal(Cvec3 n) {
    normal = n;
  }

	void newForce(Cvec3 force) {
		accel += force/mass;
	}

	void timeStep() {
		if(canMove)
		{
			Cvec3 tmp = pos;
			pos = pos + (pos - prevPos) * (1.0 - AIR_RESISTANCE) + accel * TIME;
			prevPos = tmp;
			resetAccel();
		}
	}

	Cvec3& getPos() {
    return pos;
  }

	void movePos(const Cvec3 v) {
    if(canMove)
      pos += v;
  }

	void fixMovement() {
    canMove = false;
  }

  void unfixMovement() {
    canMove = true;
  }

	void addNormal (Cvec3 normal) {
		normal += normal.normalize();
	}

	Cvec3& getNormal() {
    return normal;
  }
};

#endif	//ball_h
