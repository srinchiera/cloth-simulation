#ifndef SPRING_H
#define SPRING_H

#include "ball.h"
#include "cvec.h"

class Spring
{
	float len; //length between the two balls

public:
	Ball *ball1;
  Ball *ball2;

	Spring(Ball *ball1, Ball *ball2) :  ball1(ball1),ball2(ball2)
	{
    len = (ball1->getPos() - ball2->getPos()).length();
	}

	void calcSpring()
	{
		Cvec3 vec = ball2->getPos() - ball1->getPos();
		float currLen = vec.length();
		Cvec3 moveLen = (vec * (1 - len/currLen)) / 2;
		ball1->movePos(moveLen);
		ball2->movePos(-moveLen);
	}
};

#endif	//spring_h
