#ifndef ARM_PLANNER_UTIL_H
#define ARM_PLANNER_UTIL_H


#include <math.h>
#include <cmath>
#include <vector>

using namespace std;

#define GETMAPINDEX(X, Y, XSIZE, YSIZE) (Y*XSIZE + X)

#define PI 3.141592654
#define EPSILON PI/8
#define LINKLENGTH_CELLS 10


typedef struct {
	int X1, Y1;
	int X2, Y2;
	int Increment;
	int UsingYIndex;
	int DeltaX, DeltaY;
	int DTerm;
	int IncrE, IncrNE;
	int XIndex, YIndex;
	int Flipped;
} bresenham_param_t;

void get_bresenham_parameters(int p1x, int p1y, int p2x, int p2y, bresenham_param_t *params);

void get_current_point(bresenham_param_t *params, int *x, int *y);

int get_next_point(bresenham_param_t *params);

int iToX(int x_size, int i);

int iToY(int x_size, int i);

int xyToI(int x, int y, int x_size);

bool coordsValid(int x, int y, int x_size, int y_size);

void ContXY2Cell(double x, double y, short unsigned int* pX, short unsigned int *pY, int x_size, int y_size);

double angularDisplacement(double ai, double af);

double configurationDistance(vector<double> a1, vector<double> a2);

double largestAngleDistance(vector<double> a1, vector<double> a2);

std::vector<double> manipulatorPosition(vector<double> angles, int x_size);

int IsValidLineSegment(double x0, double y0, double x1, double y1, double*	map, int x_size, int y_size);

int IsValidArmConfiguration(vector<double> angles, double* map, int x_size, int y_size);



#endif // ARM_PLANNER_UTIL_H




