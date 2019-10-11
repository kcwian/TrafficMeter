#include "header.h"

using namespace std;
static bool tramCenter = false;

void readSequences(std::vector<string> & sequencesToProcess, string filename)
{
	std::fstream myfile;
	myfile.open(filename);
	if (!myfile.is_open())
	{
		cout << "Nie znaleziono pliku nazwy_sekwencji.txt" << endl;
		return;
	}
	string sequence;

	while (getline(myfile, sequence)) {

		sequencesToProcess.push_back(sequence);
	}
	myfile.close();

}

void saveSequenceResult(int a, int b, int c, int d, int e, int f, int g, string filename)
{
	fstream wynik;
	static bool firstTime = true;
	if (firstTime)
	{
		firstTime = false;
		wynik.open(filename, ios::out);
	}
	else
	{
		wynik.open(filename, ios::app);
	}
	wynik << a << "," << b << "," << c << ","  << d << "," << e << "," << f << "," << g  << endl;
	wynik.close();
}

bool sortRectangle(Rect & rect, std::vector<Rect> & upperLane, std::vector<Rect> & lowerLane, std::vector<Rect> & trackway, std::vector<Rect> & pavement, int flags)
{
	enum
	{
		UPPER_LANE = 1,
		LOWER_LANE = 2,
		TRACKWAY = 3,
		PAVEMENT = 4
	};

	enum FLAGS
	{
		NO_PAVEMENT = 32
	};

	static float const minUpperLaneArea =		  8000  * CAP_SCALE * CAP_SCALE; // OK
	static float const minLowerLaneArea =		  18000 * CAP_SCALE * CAP_SCALE; // 
	static float const minTrackwayArea =		  30000 * CAP_SCALE * CAP_SCALE; // 
	static float const minPavementArea =		  10000 * CAP_SCALE * CAP_SCALE; // 

	static const float laneBordersY[8] = {										// Pozioma linia oddzielaj¹ca górny pas od do³u w górê
											150 * CAP_SCALE, 300 * CAP_SCALE,   // Gorny pas
											300 * CAP_SCALE, 355 * CAP_SCALE,   // Torowisko
											355 * CAP_SCALE, 650 * CAP_SCALE,   // Dolny pas
											700 * CAP_SCALE, 1000 * CAP_SCALE }; // Sciezka 


	cv::Point center = Point(rect.x + (rect.width / 2), rect.y + (rect.height / 2));	// Œrodek prostok¹ta
	int local = 0;
	bool returnValue = false;
	int rectArea = rect.area();


	if (center.y >= laneBordersY[0] && center.y < laneBordersY[1])
		local = UPPER_LANE;

	else if (center.y >= laneBordersY[2] && center.y < laneBordersY[3])
		local = TRACKWAY;

	else if (center.y >= laneBordersY[4] && center.y < laneBordersY[5])
		local = LOWER_LANE;

	else if (center.y >= laneBordersY[6] && center.y < laneBordersY[7])
		local = PAVEMENT;

	switch (local)
	{
	case UPPER_LANE:
	{
		if (rectArea  > minUpperLaneArea)
		{
			if ((center.y > 260 * CAP_SCALE) && (rectArea > 35000 * CAP_SCALE * CAP_SCALE))  // tramwaj na gornej lini
				trackway.push_back(rect);

			else if ((tramCenter == true) && (center.y > 180 * CAP_SCALE) )
			{
				return false;
			}
		
			else
				upperLane.push_back(rect);

			returnValue = true;

		}
	}
	break;

	case LOWER_LANE:
	{
		if ((tramCenter == true) && (flags == 0)) // Wykorzystanie tego, ¿e tym siê róŸni dolne wywo³anie tej funkcji od górnej - jeœli false to górne wywo³anie funkcji z przeprowadzonym  threshold - wtedy tylko dodawac cos do konturu lowerLine
		{
			returnValue = false;
		}
		else if ((tramCenter == true) && (center.y < 420 * CAP_SCALE))
		{
			return false;
		}
		else if (rectArea  > minLowerLaneArea)
		{
			lowerLane.push_back(rect);
			returnValue = true;
		}
	}
	break;

	case PAVEMENT:
	{
		if (flags == FLAGS::NO_PAVEMENT)
			break;

		if (rectArea > minPavementArea)
		{
			pavement.push_back(rect);
			returnValue = true;
		}
	}
	break;

	case TRACKWAY:
	{
		if (rectArea  > minTrackwayArea)
		{
			trackway.push_back(rect);
			returnValue = true;
		}
	}
	break;

	}

	return returnValue;
}

void processUpperLane(std::vector<Rect> & upperLaneRectangles, Mat & frame, int & carsPassed, int & trucksPassed, bool initalize)  // Przetwarza wszystkie prostok¹ty, na których znaleziono pojazdy 
{
	// 
	// Dodac to te¿ w pozosta³ych funkcjach

	static const float carTruckThreshold = 46000 * CAP_SCALE * CAP_SCALE;

	static LineCounter carsLine1(550 * CAP_SCALE, LineCounter::RIGHT2LEFT);
	static LineCounter carsLine2(600 * CAP_SCALE, LineCounter::RIGHT2LEFT);
	static LineCounter carsLine3(650 * CAP_SCALE, LineCounter::RIGHT2LEFT);
	static LineCounter carsLine4(700 * CAP_SCALE, LineCounter::RIGHT2LEFT);

	static LineCounter trucksLine1(600 * CAP_SCALE, LineCounter::RIGHT2LEFT);
	static LineCounter trucksLine2(650 * CAP_SCALE, LineCounter::RIGHT2LEFT);	
	static LineCounter trucksLine3(700 * CAP_SCALE, LineCounter::RIGHT2LEFT);
	static LineCounter trucksLine4(750 * CAP_SCALE, LineCounter::RIGHT2LEFT);

	if (initalize)
	{
		carsLine1.reset();
		carsLine2.reset();
		carsLine3.reset();
		carsLine4.reset();
		
		trucksLine1.reset();
		trucksLine2.reset();
		trucksLine3.reset();
		trucksLine4.reset();
	}

	for (int i = 0; i < upperLaneRectangles.size(); i++)
	{

		Rect rect = upperLaneRectangles.at(i);
		Point center = Point(rect.x + (rect.width / 2), rect.y + (rect.height / 2));

		if (rect.area() < carTruckThreshold)
		{
			carsLine1.checkPoint(center);
			carsLine2.checkPoint(center);
			carsLine3.checkPoint(center);
			carsLine4.checkPoint(center);
			//circle(frame, center, 2, Scalar(255, 0, 0), 2, 8, 0); 
			rectangle(frame, rect, Scalar(0, 0, 255), 2, 8);
		}
		else
		{
			trucksLine1.checkPoint(center);
			trucksLine2.checkPoint(center);
			trucksLine3.checkPoint(center);
			trucksLine4.checkPoint(center);
			//circle(frame, center, 2, Scalar(255, 0, 0), 2, 8, 0);
			rectangle(frame, rect, Scalar(255, 0, 0), 2, 8);
		}

	}
	
	carsLine1.updatePassed();
	carsLine2.updatePassed();
	carsLine3.updatePassed();
	carsLine4.updatePassed();

	int cars1 = carsLine1.getPassed();
	int cars2 = carsLine2.getPassed();
	int cars3 = carsLine3.getPassed();
	int cars4 = carsLine4.getPassed();
	carsPassed = round((float)(cars1 + cars2 + cars3 + cars4) / 4.0);

	//putText(frame, "Upper line cars: " + std::to_string(carsPassed), Point(0, 845), FONT_HERSHEY_TRIPLEX, 1, Scalar(0, 0, 255), 1);
	//putText(frame, std::to_string(carsRightLine.getPassed()), Point(205, 45), FONT_HERSHEY_SIMPLEX, 2, Scalar(255, 255, 0), 2);

	trucksLine1.updatePassed();
	trucksLine2.updatePassed();
	trucksLine3.updatePassed();
	trucksLine4.updatePassed();
	

	int trucks1 = trucksLine1.getPassed();
	int trucks2 = trucksLine2.getPassed();
	int trucks3 = trucksLine3.getPassed();
	int trucks4 = trucksLine4.getPassed();
	trucksPassed = round((float)(trucks1 + trucks2 + trucks3 + trucks4) / 4.0);

	putText(frame, "Cars: " + std::to_string(carsPassed), Point(0, 140), FONT_HERSHEY_TRIPLEX, 1, Scalar(0, 0, 0), 2);
        putText(frame, "Trucks: " + std::to_string(trucksPassed), Point(0, 170), FONT_HERSHEY_TRIPLEX, 1, Scalar(0, 0, 0), 2);	
//putText(frame, std::to_string(trucksPassed), Point(205, 95), FONT_HERSHEY_SIMPLEX, 2, Scalar(0, 255, 255), 2);

}

void processLowerLane(std::vector<Rect> & lowerLaneRectangles, Mat & frame, int & carsPassed, int & trucksPassed, bool initalize)  // Przetwarza wszystkie prostok¹ty, na których znaleziono pojazdy 
{
	static const float carTruckThreshold = 130000 * CAP_SCALE * CAP_SCALE;

	//static LineCounter carsLeftLine(580 * CAP_SCALE, LineCounter::LEFT2RIGHT);
	//static LineCounter carsRightLine(620 * CAP_SCALE, LineCounter::LEFT2RIGHT);
	//static LineCounter trucksLeftLine(580 * CAP_SCALE, LineCounter::LEFT2RIGHT);
	//static LineCounter trucksRightLine(620 * CAP_SCALE, LineCounter::LEFT2RIGHT);

	static LineCounter carsLine1(600 * CAP_SCALE, LineCounter::LEFT2RIGHT);
	static LineCounter carsLine2(650 * CAP_SCALE, LineCounter::LEFT2RIGHT);
	static LineCounter carsLine3(700 * CAP_SCALE, LineCounter::LEFT2RIGHT);
	static LineCounter carsLine4(750 * CAP_SCALE, LineCounter::LEFT2RIGHT);

	static LineCounter trucksLine1(600 * CAP_SCALE, LineCounter::LEFT2RIGHT);
	static LineCounter trucksLine2(650 * CAP_SCALE, LineCounter::LEFT2RIGHT);
	static LineCounter trucksLine3(700 * CAP_SCALE, LineCounter::LEFT2RIGHT);
	static LineCounter trucksLine4(750 * CAP_SCALE, LineCounter::LEFT2RIGHT);
	
	if (initalize)
	{
		carsLine1.reset();
		carsLine2.reset();
		carsLine3.reset();
		carsLine4.reset();

		trucksLine1.reset();
		trucksLine2.reset();
		trucksLine3.reset();
		trucksLine4.reset();
	}

	for (int i = 0; i < lowerLaneRectangles.size(); i++)
	{

		Rect rect = lowerLaneRectangles.at(i);
		Point center = Point(rect.x + (rect.width / 2), rect.y + (rect.height / 2));

		if (rect.area() < carTruckThreshold)
		{
			carsLine1.checkPoint(center);
			carsLine2.checkPoint(center);
			carsLine3.checkPoint(center);
			carsLine4.checkPoint(center);
			rectangle(frame, rect, Scalar(255, 255, 0), 2, 8);
		}
		else
		{
			trucksLine1.checkPoint(center);
			trucksLine2.checkPoint(center);
			trucksLine3.checkPoint(center);
			trucksLine4.checkPoint(center);
	       	 rectangle(frame, rect, Scalar(0, 255, 255), 2, 8);
		}
	}

	carsLine1.updatePassed();
	carsLine2.updatePassed();
	carsLine3.updatePassed();
	carsLine4.updatePassed();

	int cars1 = carsLine1.getPassed();
	carsPassed = cars1;
	int cars2 = carsLine2.getPassed();
	carsPassed = cars2;
	int cars3 = carsLine3.getPassed();
	carsPassed = cars3;
	int cars4 = carsLine4.getPassed();
	carsPassed = cars4;
	carsPassed = round((cars1 + cars2 + cars3 + cars4) / 4.0);
	
	//putText(frame, "Lower line cars: " + std::to_string(carsPassed), Point(45, 845), FONT_HERSHEY_SIMPLEX, 2, Scalar(255, 255, 0), 2);
	//putText(frame, std::to_string(carsRightLine.getPassed()), Point(205, 345), FONT_HERSHEY_SIMPLEX, 2, Scalar(255, 255, 0), 2);

	trucksLine1.updatePassed();
	trucksLine2.updatePassed();
	trucksLine3.updatePassed();
	trucksLine4.updatePassed();


	int trucks1 = trucksLine1.getPassed();
	int trucks2 = trucksLine2.getPassed();
	int trucks3 = trucksLine3.getPassed();
	int trucks4 = trucksLine4.getPassed();
	trucksPassed = round((float)(trucks1 + trucks2 + trucks3 + trucks4) / 4.0);

	putText(frame,"Cars: " +  std::to_string(carsPassed), Point(0, 420), FONT_HERSHEY_TRIPLEX, 1, Scalar(0, 0, 0), 2);
	putText(frame, "Trucks: " +  std::to_string(trucksPassed), Point(0, 450), FONT_HERSHEY_TRIPLEX, 1, Scalar(0, 0, 0), 2);
	//putText(frame, std::to_string(trucksRightLine.getPassed()), Point(205, 395), FONT_HERSHEY_SIMPLEX, 2, Scalar(0, 255, 255), 2);

}

void processPavement(std::vector<Rect> & pavementRectangles, Mat & frame, int & passedPedestrians, int & passedBikers, bool initalize)
{
	float static const minSearchArea = 10000 * CAP_SCALE * CAP_SCALE;
	float static const pedestrBikeThreshold = 30000 * CAP_SCALE * CAP_SCALE;

	static LineCounter bikersLeftLine(480 * CAP_SCALE, LineCounter::BOTH);
	static LineCounter bikersRightLine(1280 * CAP_SCALE, LineCounter::BOTH);
	static LineCounter pedestriansLeftLine(480 * CAP_SCALE, LineCounter::BOTH);
	static LineCounter pedestriansRightLine(1280 * CAP_SCALE, LineCounter::BOTH);

	if (initalize)
	{
		bikersLeftLine.reset();
		bikersRightLine.reset();
		pedestriansLeftLine.reset();
		pedestriansRightLine.reset();
	}

	for (int i = 0; i < pavementRectangles.size(); i++)
	{
		Rect rect = pavementRectangles.at(i);
		Point center = Point(rect.x + (rect.width / 2), rect.y + (rect.height / 2));

		int area = rect.area();

		// Po rozmiarze - dzia³aj¹ce w miarê
		/*if (area <= pedestrBikeThreshold)
		{
			pedestriansLeftLine.checkPoint(center);
			pedestriansRightLine.checkPoint(center);
			rectangle(frame, rect, Scalar(0, 0, 255), 2, 8);
		}
		else
		{
			bikersLeftLine.checkPoint(center);
			bikersRightLine.checkPoint(center);
			rectangle(frame, rect, Scalar(255, 0, 0), 2, 8);
		}*/

		// Po wpsó³rzêdnych góra - dó³
		//circle(frame, center, 2, Scalar(0, 0, 0), 3, 8, 0);

		if (center.y > 770 * CAP_SCALE)
		{
			pedestriansLeftLine.checkPoint(center);
			pedestriansRightLine.checkPoint(center);
			rectangle(frame, rect, Scalar(0, 255, 0), 2, 8);
		}
		else
		{
			bikersLeftLine.checkPoint(center);
			bikersRightLine.checkPoint(center);
			rectangle(frame, rect, Scalar(255, 0, 255), 2, 8);
		}

	}

	pedestriansLeftLine.updatePassed();
	pedestriansRightLine.updatePassed();

	int pedestriansLeftLinePassed = pedestriansLeftLine.getPassed();
	int pedestriansRightLinePassed = pedestriansRightLine.getPassed();
	passedPedestrians = max(pedestriansLeftLinePassed, pedestriansRightLinePassed);

	//putText(frame, std::to_string(pedestriansLeftLine.getPassed()), Point(45, 445), FONT_HERSHEY_SIMPLEX, 2, Scalar(0, 0, 255), 2);
	//putText(frame, std::to_string(pedestriansRightLine.getPassed()), Point(605, 445), FONT_HERSHEY_SIMPLEX, 2, Scalar(0, 0, 255), 2);

	bikersLeftLine.updatePassed();
	bikersRightLine.updatePassed();

	int bikersLeftLinePassed = bikersLeftLine.getPassed();
	int bikersRightLinePassed = bikersLeftLine.getPassed();
	passedBikers = max(bikersLeftLinePassed, bikersRightLinePassed);

	//putText(frame, std::to_string(bikersLeftLine.getPassed()), Point(95, 445), FONT_HERSHEY_SIMPLEX, 2, Scalar(255, 0, 0), 2);
	putText(frame, "Pedstr: " + std::to_string(passedPedestrians), Point(0, 815), FONT_HERSHEY_TRIPLEX, 1, Scalar(0, 0, 0), 2);
	 putText(frame, "Bikers: " + std::to_string(passedBikers) , Point(0, 845), FONT_HERSHEY_TRIPLEX, 1, Scalar(0, 0, 0), 2);
}

void processTrackwayNew(std::vector<Rect> & trackwayRectangles, Mat & frame, int & tramsPassed, bool initalize)  // Przetwarza wszystkie prostok¹ty, na których znaleziono pojazdy 
{
	
	// Tramwaj nie musi przekroczyc lini, a trzeba sprawdzac czy pojawi³ siê prostok¹t
	static bool tramLeft = false;
	static bool tramRight = false;
	
	static const float tramThreshold = 40000 * CAP_SCALE * CAP_SCALE;
	static int tramsPassed_ = 0;

//	static LineCounter tramRightLine(820 * CAP_SCALE, LineCounter::RIGHT2LEFT);
//	static LineCounter tramLeftLine(160 * CAP_SCALE, LineCounter::RIGHT2LEFT);
//	static LineCounter tramCenterLine(420 * CAP_SCALE, LineCounter::RIGHT2LEFT);

	static int tramLeftDownCounter = 100;
	static int tramCenterDownCounter = 100;
	static int tramRightDownCounter = 100;
	static int allDownCounter = 200;
	static bool alreadyAdded = false;

	bool actualFrameTramLeft = false, actualFrameTramRight = false, actualFrameTramCenter = false;
	
	if (initalize)
	{
		tramsPassed_ = 0;
		alreadyAdded = false;
		allDownCounter = 200;
		tramRightDownCounter = 100;
		tramCenterDownCounter = 100;
		tramLeftDownCounter = 100;
		tramLeft = 0;
		tramCenter = 0;
		tramRight = 0;
	}

	for (int i = 0; i < trackwayRectangles.size(); i++)
	{

		Rect rect = trackwayRectangles.at(i);
		Point center = Point(rect.x + (rect.width / 2), rect.y + (rect.height / 2));

		if (rect.area() > tramThreshold)
		{

			if (center.x < 340 * CAP_SCALE)
			{
				actualFrameTramLeft = true;
				tramLeft = true;
				//cout << "Tram z lewej" << endl;
			}
			else if (center.x < 1000 * CAP_SCALE)
			{			
			    actualFrameTramCenter = true;
			    tramCenter = true;
			//cout << "Tram na srodku" << endl;
			}
			else if (center.x > 1400 * CAP_SCALE)	
			{
			   actualFrameTramRight = true;
			   tramRight = true;
			//cout << "Tram z prawej" << endl; 
			}

			rectangle(frame, rect, Scalar(0, 0, 0), 2, 8);
		//	circle(frame, center, 2, Scalar(0, 0, 0), 3, 8, 0);
		}
	}


	if (tramLeft == true)
	{
		tramLeftDownCounter--;
		if (tramLeftDownCounter < 1)
		{
			tramLeftDownCounter = 120;
			tramLeft = false;
		//	cout << "Koniec tramLeft";
		}

	}
	if (tramCenter == true)
	{
		tramCenterDownCounter--;
		if (tramCenterDownCounter < 1)
		{
			tramCenterDownCounter = 120;
			tramCenter = false;
		//	cout << "Koniec tramCenter";
		}
	}
	if (tramRight == true)
	{
		tramRightDownCounter--;
		if (tramRightDownCounter < 1)
		{
			tramRightDownCounter = 120;
			tramRight = false;
			//cout << "Koniec tramRight";
		}


	}

	// Resetowanie - mo¿liwe nowe zliczanie po 3 klatkach braku tramwaju
	static int alreadyAddedResetFrames = 0;
	if ((actualFrameTramLeft == false) && (actualFrameTramCenter == false) && (actualFrameTramRight == false))
	{
		alreadyAddedResetFrames++;
		if (alreadyAddedResetFrames > 30)
		{
			alreadyAdded = false;
			alreadyAddedResetFrames = 0;
			allDownCounter = 200;
			tramCenterDownCounter = 120;
			tramLeftDownCounter = 120;
			tramRightDownCounter = 120;
			tramLeft = false;
			tramRight = false;
			tramCenter = false;
		//	cout << "30 Klatek Reset" << endl;
		}
	}
	else
	{
		alreadyAddedResetFrames = 0;
	}

	if (tramLeft || tramCenter || tramRight)
	{

		if ((tramLeft == true) && (tramCenter == true) && (tramRight == true) && (alreadyAdded == false))
		{
			tramsPassed_ += 1;
			alreadyAdded = true;
		}
		else if ((tramLeft == true) && (tramRight == true) && (tramCenter == false) && (alreadyAdded == false))
		{
			tramsPassed_ += 2;
			alreadyAdded = true;
		}


		tramsPassed = tramsPassed_;
	}

	putText(frame, "Trams: " + std::to_string(tramsPassed), Point(0, 330), FONT_HERSHEY_TRIPLEX, 1, Scalar(0, 0, 0), 2);
	//putText(frame, std::to_string(tramRightLine.getPassed()), Point(550, 550), FONT_HERSHEY_SIMPLEX, 2, Scalar(255, 255, 255), 2);


}
