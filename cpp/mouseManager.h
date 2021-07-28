#pragma once
#include <GL/glew.h>
#include <GL/glut.h>
#include <math.h>
#include <cstdio>
#include <iostream>
using namespace std;
extern GLfloat mouseX_, mouseY_;  // posicao inicial do cursor
extern bool rotacionando2;
extern GLfloat m2[];
void mouseAndandoCB2(int PX, int PY);
void mouseCB(int button, int state, int x, int y);