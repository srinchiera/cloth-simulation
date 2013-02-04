#ifndef BALL_H
#define BALL_H

#include "cvec.h"
#include <iostream>

class Ball
{
  Cvec3 accel; //accel
  Cvec3 pos; //current position
	Cvec3 prevPos; //old position
	Cvec3 normal; // normal
  bool canMove;

public:
  Ball() {}
	Ball(Cvec3 pos) : accel(Cvec3()), pos(pos), prevPos(pos), normal(Cvec3()), canMove(true) {}

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

	void addNormal (Cvec3 n) {
		normal += n.normalize();
	}

    Cvec3& getNormal() {
    return normal;
  }

	void newForce(Cvec3 force) {
		accel += force;
	}

  void minusForce(Cvec3 force) {
		accel -= force;
	}

	Cvec3& getPos() {
    return pos;
  }

	void movePos(const Cvec3 v) {
    if(canMove) 
      pos += v;
  }
	void setPos(const Cvec3 v) {
    pos = v;
  }

	void fixMovement() {
    canMove = false;
  }

  void unfixMovement() {
    canMove = true;
  }

  void timeStep() {
		if(canMove)
		{
			Cvec3 tmp = pos;
      //account for air resistance in calculating distance moved
			pos = accel * 0.25 + pos + (pos - prevPos) * 0.9;
			prevPos = tmp;
			resetAccel();
		}
	}
};

#endif	//ball_h
