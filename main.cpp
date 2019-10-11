#include "header.h"

Mat  frame, frameBGS, imageToDisplay;

Ptr< BackgroundSubtractor> pBGS; //MOG Background subtractor 

int main()
{
	// params
	float const capScale = CAP_SCALE;						         	 // Skalowanie obrazu do przetwarzania
	float const minArea = 4000				* CAP_SCALE  * CAP_SCALE;						 // Minimalny obszar dla aut
	float const searchAgainCarArea = 32000  * CAP_SCALE * CAP_SCALE;				 // Obszar powy¿ej którego trzeba przeszukaæ jeszcze raz


	int carsUpperLaneL = 0, trucksUpperLaneL = 0;
	int carsUpperLaneR = 0, trucksUpperLaneR = 0;
	int pedestrPathL = 0, pedestrPathR = 0;


	Mat frameCap;

//	VideoCapture cap("00005.MTS");
	//VideoCapture cap("pedestrians.avi");
	//cap.set(CV_CAP_PROP_POS_FRAMES, 5900); //1500, 1700 - duzo, 0 - pieszy, 6350 - rower, 4400 - tir, 5900- dolny pas,	15000- duzo, 15300 - jeden tramwaj, 2000 - dwa tramwaje
	//cap.set(CV_CAP_PROP_POS_MSEC,  2.5 * 60 * 1000);
//	int capWidth = cap.get(CV_CAP_PROP_FRAME_WIDTH);
//	int capHeight = cap.get(CV_CAP_PROP_FRAME_HEIGHT);
	//cout << capWidth << endl;

	namedWindow("frame",cv::WINDOW_NORMAL);
	cv::setMouseCallback("frame", mouseCallback);

	pBGS = createBackgroundSubtractorMOG2();	//  Stwórz Background Substractor
	//for(int i=0;i <20; i++)
	//pBGS->apply(frame, frameBGS, 0);

	// Pomiar FPS
	TickMeter tickMeter;
	tickMeter.start();

	TickMeter tickMeterTotal;
	tickMeterTotal.start();
	int framesProcessed = 0;
	std::vector<string> sequencesToProcess;
	readSequences(sequencesToProcess, "./sequences_names/sequences_names.txt");
	for (string seq : sequencesToProcess)
	{
		bool initalizeLines = true;
		std::cout << "Przetwarzanie sekwencji: "<< seq << endl;
		int topPassedCars = 0, topPassedTrucks = 0, pedestrians = 0, bikers = 0, lowPassedCars = 0, lowTrucks = 0, trams = 0;
	   VideoCapture cap("./sequences_video/" + seq);
	//cap.set(CV_CAP_PROP_POS_FRAMES, 200);
		int actualFrame = 0;

			while (true)
			{
				if (!cap.read(frameCap))
					break;

				actualFrame++;
				Mat frame = frameCap.clone();
				cv::resize(frame, frame, Size(0, 0), capScale, capScale);
				pBGS->apply(frame, frameBGS, 0.005); // Lepiej dzia³a jak jest duzo aut - 0.005

				medianBlur(frameBGS, frameBGS, 5);

				vector<vector<Point>> contours;
				vector<Vec4i> hierarchy;

				if (actualFrame > 20)
				{
					findContours(frameBGS, contours, hierarchy, cv::RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
				}

				//	Sortowanie konturów
				std::vector<Rect> upperLaneRectangles, lowerLaneRectangles, trackwayRectangles, pavementRectangles;

				for (int i = 0; i < contours.size(); i++)
				{
							
					Rect minRect = boundingRect(contours[i]);	

					if (minRect.area() > minArea)
					{
						Mat roi = frameBGS(minRect).clone();
						Mat roiNoThres = roi.clone();
						threshold(roi, roi, 200, 255, THRESH_BINARY);

						vector<vector<Point> > contoursNew;
						vector<Vec4i> hierarchyNew;
						findContours(roi, contoursNew, hierarchyNew, cv::RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);			// Szuka mniejszych obszarów 

						bool foundMinOneArea = false;

						for (int i = 0; i < contoursNew.size(); i++)
						{
							Rect minRectNew = boundingRect(contoursNew[i]);
							minRectNew.x += minRect.x;												// Dodaje wspó³rzêdne, tak aby wspó³rzêdne pochodzi³y z ca³ego obrazu
							minRectNew.y += minRect.y;

							if (minRectNew.area() > minArea)											// Globalna wartoœæ dla wszystkich obiektów
							{

								if (sortRectangle(minRectNew, upperLaneRectangles, lowerLaneRectangles, trackwayRectangles, pavementRectangles, 32))	// Nie uwzglêdnia œcie¿ki - dla œcie¿ki nie bêdzie thresholdu // Zwraca "True" jeœli znaleziono minimalny obszar
								{
									foundMinOneArea = true;
								}
							}
						}

						if (foundMinOneArea == false)
						{
							sortRectangle(minRect, upperLaneRectangles, lowerLaneRectangles, trackwayRectangles, pavementRectangles, 0);
						}
					}
				}

				// Zliczanie na górnym pasie
				if (initalizeLines == true)
				{
					initalizeLines = false;
					processTrackwayNew(trackwayRectangles, frame, trams, true);
					processUpperLane(upperLaneRectangles, frame, topPassedCars, topPassedTrucks, true);				
					processLowerLane(lowerLaneRectangles, frame, lowPassedCars, lowTrucks, true);				
				    processPavement(pavementRectangles, frame, pedestrians, bikers, true);
				}
				else
				{
			    	processTrackwayNew(trackwayRectangles, frame, trams, false);
					processUpperLane(upperLaneRectangles, frame, topPassedCars, topPassedTrucks, false);
					processLowerLane(lowerLaneRectangles, frame, lowPassedCars, lowTrucks, false);
					processPavement(pavementRectangles, frame, pedestrians, bikers, false);
				
				}
				//	cout << bikers << endl;


				tickMeter.stop();
				framesProcessed++;

				int frameWidth = frame.size().width;
				int frameHeight = frame.size().height;


				// Aktualna klatka
				//putText(frame, std::to_string((int)cap.get(cv::CAP_PROP_POS_FRAMES)), cv::Point(20, frameHeight - 20), FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 255), 1);
				// FPS
				putText(frame, std::to_string((int)(1 / tickMeter.getTimeSec())), cv::Point(frameWidth - 50, frameHeight - 20), FONT_HERSHEY_TRIPLEX, 1, cv::Scalar(0, 0, 255), 1);
				
				tickMeter.reset();
				tickMeter.start();
				imshow("frameBGS", frameBGS);
				imshow("frame", frame);

				int key = waitKey(1);

				//if ((key == 'h'))
				//	waitKey(0);

				/*if ((key == 'q'))
					break;*/
			}
			saveSequenceResult(topPassedCars, lowPassedCars,  topPassedTrucks, lowTrucks, trams, pedestrians, bikers, "./results/results.txt");
}

	tickMeterTotal.stop();
	//destroyAllWindows();
	cout << "Koniec - przetworzono: " << framesProcessed << "Klatek w Czasie: " << tickMeterTotal.getTimeSec() << " FPS:" << framesProcessed / tickMeterTotal.getTimeSec() << endl;
	waitKey(1000);
	return 1;
}


void mouseCallback(int event, int x, int y, int flags, void* userdata)
{
	switch (event)
	{
	case(EVENT_LBUTTONDOWN):
	{
		std::cout << Point(x, y) << std::endl;
	}
	}
}
