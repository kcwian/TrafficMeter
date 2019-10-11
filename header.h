#pragma once

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <stdio.h>
#include <opencv2/highgui.hpp>
#include <opencv2/video.hpp>
#include <stdio.h>
#include <iostream>
#include <sstream>
#include <time.h>
#include <iostream>
#include <fstream>

using namespace cv;
using namespace std;

#define CAP_SCALE		    0.8

class CountLine;

void readSequences(std::vector<string> & sequencesToProcess, string filename);
void saveSequenceResult(int a, int b, int c, int d, int e, int f, int g, string filename);
void mouseCallback(int event, int x, int y, int flags, void* userdata);
bool sortRectangle(Rect & rect, std::vector<Rect> & upperLane, std::vector<Rect> & lowerLane, std::vector<Rect> & trackway, std::vector<Rect>  & pavement, int flags);
void processUpperLane(std::vector<Rect> &, Mat &, int &, int &, bool);
void processLowerLane(std::vector<Rect> &, Mat &, int &, int &, bool);
void processPavement(std::vector<Rect> &, Mat &, int &, int &, bool);
void processTrackwayNew(std::vector<Rect> & trackwayRectangles, Mat & frame, int & tramsPassed, bool initalize);


// Pionowe linie
class LineCounter
{

public:

	enum
	{
		LEFT2RIGHT = 1,
		RIGHT2LEFT = 2,
		BOTH = 3,
	};

public:

	LineCounter(int x_, int direction_)
	{
		x = x_;
		direction = direction_;
	}
	~LineCounter() {}

	void checkPoint(Point center)
	{
		// Zlicza punkty, na prawo i na lewo od lini
		if ((center.x >= x) && (center.x <x + (60*CAP_SCALE)))
			actFromRight++;

		if ((center.x < x) && (center.x > x - (60*CAP_SCALE)))
			actFromLeft++;
	}

	
	void updatePassed()
	{
		if (direction == LEFT2RIGHT)
		{
			if ((prevFromLeft != 0) && (actFromRight > 0))
			{
				int a = actFromRight - prevFromRight;
				int b = prevFromLeft - actFromLeft;
				if ((a > 0) || (b > 0))						// Tyle ile przyby這 z lewej lub uby這 z prawej
				{
					if (a > b)
						passed += a;
					else
						passed += b;
				}
			}
		}
		else if (direction == RIGHT2LEFT)
		{
			if (prevFromRight != 0 && (actFromLeft > 0))
			{
				int a = actFromLeft - prevFromLeft;
				int b = prevFromRight - actFromRight;
				if ((a > 0) || (b > 0))						// Tyle ile przyby這 z lewej lub uby這 z prawej
				{			
					if (a > b)
						passed += a;
					else
						passed += b;
				}
			}
		}
		else if (direction == BOTH)
		{
			if ((prevFromRight != 0) || (prevFromLeft !=0))
			{
				int a = actFromLeft - prevFromLeft;
				int b = prevFromRight - actFromRight;
				if ((a > 0) || (b > 0))						// Tyle ile przyby這 z lewej lub uby這 z prawej
				{
					if (a > b)
						passed += a;
					else
						passed += b;
				}
			}
		}

		prevFromLeft = actFromLeft;
		prevFromRight = actFromRight;
		actFromLeft = 0;
		actFromRight = 0;
	}

	int getPassed()
	{
		return passed;
	}

	void reset()
	{
		actFromLeft = 0;
		actFromRight = 0;
		prevFromLeft = 0;
		prevFromRight = 0;
		passed = 0;
	}


private:

	int x;
	int direction = 0;
	int actFromLeft = 0;
	int actFromRight = 0;
	int prevFromLeft = 0;
	int prevFromRight = 0;
	int passed = 0;

};

class CountLine
{

public:

	CountLine(int lineX_)
	{
		lineX = lineX_;
		carsFromLeft = 0;
		carsFromRight = 0;
		trucksFromLeft = 0;
	    trucksFromRight = 0;

		prevCarsFromLeft = 0;
	    prevCarsFromRight = 0;
		prevTrucksFromLeft = 0;
		prevTrucksFromRight = 0;

		pedestrFromRight = 0;
		pedestrFromLeft = 0;
		bikerFromRight = 0;
		bikerFromLeft = 0;

		prevPedestrFromRight = 0;
		prevPedestrFromLeft = 0;
		prevBikerFromRight = 0;
		prevBikerFromLeft = 0;

		passedCars = 0;
		passedTrucks = 0;
		passedBikers = 0;
		passedPedestr = 0;
	}

private:

	int lineX;

	int carsFromLeft = 0;
	int carsFromRight;
	int trucksFromLeft;
	int trucksFromRight;
	int pedestrFromRight;
	int pedestrFromLeft;
	int bikerFromRight;
	int bikerFromLeft;

	int prevCarsFromLeft;
	int prevCarsFromRight;
	int prevTrucksFromLeft;
	int prevTrucksFromRight;
	int prevPedestrFromRight;
	int prevPedestrFromLeft;
	int prevBikerFromRight;
	int prevBikerFromLeft;

	int passedCars;
	int passedTrucks;
	int passedBikers;
	int passedPedestr;

public:

	void checkCar(Point center)
	{
		// Zlicza punkty, na prawo i na lewo od lini
		if ((center.x >= lineX) && (center.x < lineX + 30))
			carsFromRight++;

		if ((center.x < lineX)  && (center.x > lineX - 30))
			carsFromLeft++;
	}

	void checkTruck(Point center)
	{
		// Zlicza punkty, na prawo i na lewo od lini
		if ((center.x >= lineX) && (center.x < lineX + 30))
			trucksFromRight++;

		if ((center.x < lineX) && (center.x > lineX - 30))
			trucksFromLeft++;
	}


	void checkPedestr(Point center)
	{
		// Zlicza punkty, na prawo i na lewo od lini
		if ((center.x >= lineX) && (center.x < lineX + 20))
			pedestrFromRight++;

		if ((center.x < lineX) && (center.x > lineX - 20))
			pedestrFromLeft++;
	}

	void checkBiker(Point center)
	{
		// Zlicza punkty, na prawo i na lewo od lini
		if ((center.x >= lineX) && (center.x < lineX + 20))
			bikerFromRight++;

		if ((center.x < lineX) && (center.x > lineX - 20))
			bikerFromLeft++;
	}

	int getPassedCars()
	{

		if (prevCarsFromRight != 0)
		{
			int a = carsFromLeft - prevCarsFromLeft;
			int b = prevCarsFromRight - carsFromRight;
			if ((a > 0) || (b > 0))
			{
				if (a > b)
					passedCars += a;
				else 
					passedCars += b;
			}
		}

		prevCarsFromLeft = carsFromLeft;
		prevCarsFromRight = carsFromRight;

		carsFromRight = 0;
		carsFromLeft = 0;

		return passedCars;
	}

	int getPassedCarsLeftToRight()
	{
		if (prevCarsFromLeft != 0)
		{
			int prevTotalCars = prevCarsFromLeft + prevCarsFromRight;
			int totalCars = carsFromLeft + carsFromRight;		
			passedCars += (carsFromRight - prevCarsFromRight);

		}
		prevCarsFromLeft = carsFromLeft;
		prevCarsFromRight = carsFromRight;

		carsFromRight = 0;
		carsFromLeft = 0;

		return passedCars;
	}

	int getPassedPedestr()
	{

		if (prevPedestrFromRight != 0)
		{
			int a = pedestrFromLeft - prevPedestrFromLeft;
			int b = prevPedestrFromRight - pedestrFromRight;
			if ((a > 0) || (b > 0))
			{
				if (a > b)
					passedPedestr += a;
				else
					passedPedestr += b;
			}
		}

		prevPedestrFromLeft = pedestrFromLeft;
		prevPedestrFromRight = pedestrFromRight;

		pedestrFromRight = 0;
		pedestrFromLeft = 0;

		return passedPedestr;
	}



	int getPassedTrucks()
	{
		prevTrucksFromLeft = trucksFromLeft;
		prevTrucksFromRight = trucksFromRight;

		return passedTrucks;
	}

	void reset()
	{
		carsFromLeft = 0;
		carsFromRight = 0;
		trucksFromLeft = 0;
		trucksFromRight = 0;

		prevCarsFromLeft = 0;
		prevCarsFromRight = 0;
		prevTrucksFromLeft = 0;
		prevTrucksFromRight = 0;

		passedCars = 0;
		passedTrucks = 0;
	}
};
