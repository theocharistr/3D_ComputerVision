#pragma once

#ifndef __SIMPLE_GAME__
#define __SIMPLE_GAME__

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <random>
#include <chrono>
#include <thread>
#include <ctime>
#include <time.h>

void borders();
void moveBall();
void redraw();
void KeyboardCallback();
void MouseCallBackFunc(int event, int x, int y, int flags, void* userdata);

#define WIDTH			800
#define HEIGHT			600
#define BORDERWIDTH		50
#define BORDERHEIGHT	50
#define WALLETWIDTH		80
#define WALLETHEIGHT	20
#define BALLWIDTH		10
#define BALLHEIGHT		10


#endif // !__SIMPLE_GAME__
