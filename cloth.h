#ifndef CLOTH_H
#define CLOTH_H

#include <iostream>
#include "cvec.h"
#include "geometry.h"

static const int ITERATIONS = 15;

class Cloth
{
    int height;
    int width;
    int ballWidth;
    int ballHeight;
    int numBalls;
    int numVertices;

    std::vector<Ball> balls;
    std::vector<Spring> springs;

    std::vector<VertexPN> vertices;

    void newSpring(Ball *ball1, Ball *ball2) {
        springs.push_back(Spring(ball1,ball2));
    }

    Ball* getBallAt (int x, int y) {
        return &balls[y * ballWidth + x];
    }

    void updateVertices() {
        updateNormals();

        std::vector<VertexPN>::iterator it = vertices.begin();

        for(int i = 0; i < ballWidth -1 ; i++) {
            for(int j = 0; j < ballHeight-1; j++)
            {
                *it = VertexPN(getBallAt(i+1,j)->getPos(), getBallAt(i+1,j)->getNormal().normalize());
                ++it;
                *it = VertexPN(getBallAt(i,j)->getPos(), getBallAt(i,j)->getNormal().normalize());
                ++it;
                *it = VertexPN(getBallAt(i,j+1)->getPos(), getBallAt(i,j+1)->getNormal().normalize());
                ++it;
                *it = VertexPN(getBallAt(i+1,j+1)->getPos(), getBallAt(i+1,j+1)->getNormal().normalize());
                ++it;
                *it = VertexPN(getBallAt(i+1,j)->getPos(), getBallAt(i+1,j)->getNormal().normalize());
                ++it;
                *it = VertexPN(getBallAt(i,j+1)->getPos(), getBallAt(i,j+1)->getNormal().normalize());
                ++it;
            }
        }
    }

  void updateNormals() {
    for (int i = 0; i < ballWidth; ++i) {
        for (int j = 0; j < ballHeight; ++j) {
            Cvec3 normal;

            normal = triNorm(getBallAt(i+1,j), getBallAt(i,j),getBallAt(i,j+1));
            getBallAt(i+1,j)->addNormal(normal);
            getBallAt(i,j)->addNormal(normal);
            getBallAt(i,j+1)->addNormal(normal);

            normal = triNorm(getBallAt(i+1,j+1), getBallAt(i+1,j),getBallAt(i,j+1));
            getBallAt(i+1,j+1)->addNormal(normal);
            getBallAt(i+1,j)->addNormal(normal);
            getBallAt(i,j+1)->addNormal(normal);
        }
    }

  }

    Cvec3 triNorm(Ball *ball1, Ball *ball2, Ball *ball3) {
        double test = cross((ball2->getPos() - ball1->getPos()),(ball3->getPos() - ball1->getPos()))[1];

        return cross((ball2->getPos() - ball1->getPos()),(ball3->getPos() - ball1->getPos()));
    }

public:
  Cloth () {}

  std::vector<VertexPN> getVertices() {
    return vertices;
  }
  void loadDimensions(int w, int h, int bw, int bh) {
        width = w;
        height = h;
        ballWidth = bw;
        ballHeight = bh;

        numBalls = bw*bh;
        numVertices = numBalls * 6;

        balls.resize(numBalls);
        vertices.resize(numVertices * 6);

    for (int x = 0; x < ballWidth; ++x)
        for (int y=0; y < ballHeight; ++y)
      {
            float pos2;
        if (x == 0 || x == ballWidth-1 || y == 0 || y == ballHeight-1)
          pos2 = height / (float)ballHeight / 2;
        else
          pos2 = 0;
        Cvec3 position = Cvec3(width * (x / (float)ballWidth),pos2,height * (y / (float)ballHeight));
            balls[y * ballWidth + x] = Ball(position+Cvec3(0,4,0));
        balls[y * ballWidth + x].setAccel(Cvec3(0.,-0.1,0.));
      }

    for(int i = 0; i < ballWidth; i++)
			for(int j = 0; j < ballHeight; j++)
			{
				if (i < ballWidth - 1)
          newSpring(getBallAt(i,j),getBallAt(i+1,j));
				if (j < ballHeight - 1)
          newSpring(getBallAt(i,j),getBallAt(i,j+1));
				if (i < ballWidth-1 && j < ballHeight-1) 
          newSpring(getBallAt(i,j),getBallAt(i+1,j+1));
				if (i < ballWidth-1 && j < ballHeight-1) 
          newSpring(getBallAt(i+1,j),getBallAt(i,j+1));
			}

		for(int i = 0; i < ballWidth; i++)
			for(int j = 0; j < ballHeight; j++)
			{
				if (i < ballWidth - 2)
          newSpring(getBallAt(i,j),getBallAt(i+2,j));
				if (j < ballHeight - 2)
          newSpring(getBallAt(i,j),getBallAt(i,j+2));
				if (i < ballWidth - 2 && j < ballHeight - 2)
          newSpring(getBallAt(i,j),getBallAt(i+2,j+2));
				if (i < ballWidth - 2 && j < ballHeight - 2)
          newSpring(getBallAt(i+2,j),getBallAt(i,j+2));			
      }

    getBallAt(0,0)->fixMovement();
    getBallAt(ballWidth-1,0)->fixMovement();
    getBallAt(0,ballHeight-1)->fixMovement();
    getBallAt(ballWidth-1,ballHeight-1)->fixMovement();

    updateVertices();


  }

  void unfix()
  {
    getBallAt(0,0)->unfixMovement(); 
    getBallAt(ballWidth-1,0)->unfixMovement();
    getBallAt(0,ballHeight-1)->unfixMovement();
    getBallAt(ballWidth-1,ballHeight-1)->unfixMovement();
  }

  void unfix2()
  {
    getBallAt(0,ballHeight-1)->unfixMovement();
    getBallAt(ballWidth-1,ballHeight-1)->unfixMovement();
  }


  void collision(Cvec3 center, float radius)
	{
   for (int i = 0, n = balls.size(); i < n; i++)
	  {
          float bufferedRadius = radius + .03;
		  Cvec3 toCenter = balls[i].getPos() - center;

		  float l = toCenter.length();

		  if (l < bufferedRadius) {
            balls[i].movePos(toCenter.normalize() * (bufferedRadius-l));
          }
	  }
        updateVertices();
	}

	void timeStep()
	{
		for(int i = 0; i < ITERATIONS; i++)
			for (int j = 0, n = springs.size(); j < n; j++)
				springs[j].calcSpring();

		for (int i = 0, n = balls.size(); i < n; i++)
			balls[i].timeStep();
	}

	void addForce(Cvec3 f)
	{
		for (int i = 0, n = balls.size(); i < n; i++) {
			balls[i].newForce(f);
            balls[i].resetNormal();
        }
	}
};

#endif
