// ConsoleApplication1.cpp : This file contains the 'main' function. Program execution begins and ends there.
/*Bouncing ball.Write a small game.The window itself is the field, it is bordered by three walls(left, right, top).
The fourth side is open, there is a wallet at that side.There is also a ball in the field, its speed is constant.
It bounces back from both the walls and the racket.The racket can be controlled by mouse and/or keyboard.
If the ball falls down, the game exits.
*/

#include "PongGame.h"
using namespace cv;
using namespace std;

#define WIDTH 800
#define HEIGHT 600


Mat image;
int iksz, ipszilon,i;

void redraw() {
   rectangle(image, Point(0, 0), Point(WIDTH, HEIGHT), Scalar(255,255,255), CV_FILLED);
   rectangle(image, Point(iksz,ipszilon), Point(iksz + 20, ipszilon + 20), Scalar(0, 0, 0), CV_FILLED);
   line(image, Point(i, 550), Point(i+50, 550 ), Scalar(0, 0, 0),10);
   imshow("Display window", image);                   // Show our image inside it.
}

void MouseCallBackFunc(int event, int x,int y, int flags, void* userdata)
{ 
    if (event == EVENT_LBUTTONDOWN)
    {
        i= x;
        redraw();
    }

} 

int main(int argc, char** argv)
{
    image = Mat::zeros(HEIGHT, WIDTH, CV_8UC3);
    iksz = +400;
    ipszilon = +400;
    i= +400;
    

    namedWindow("Display window", WINDOW_AUTOSIZE);// Create a window for display.
    setMouseCallback("Display window", MouseCallBackFunc, NULL);



    imshow("Display window", image);                   // Show our image inside it.


    int key;
    bool flag1 = false;//flag1 =false go down
    bool flag2 = false;//flag2=false go right
    while (true) {
        key = cvWaitKey(100);
        
        //cout << key<<" \n";
        //Set borders
        if (flag1 == true)
        {
            ipszilon= ipszilon-5;
        }
        else
        {   
            ipszilon= ipszilon+5;
        }
        if (flag2 == true)
        {
            iksz=iksz-5;
        }
        else
        {
            iksz=iksz+5;
        }
        //cout << flag1 << " " << flag2 << "\n";
        // cout << i << ":i," << i << "\n";
        // cout << iksz<<":x,"<< ipszilon <<":y"<<"\n";
        // (+/-)20 due ti the size of the square
        if (ipszilon ==0)
        {
            ipszilon = ipszilon +20; 
            flag1 = false;
        }
        if (iksz+20 == +800){
            iksz = iksz -20;
            flag2 = true;
        }
        if (iksz == 0)
        {
            iksz = iksz +20;
            flag2 = false;
        }
        //If wallet goes out of bounds
        if (i <= 0 ) {
            i= 0;
        }
        else if( i + 50 >= 800)
        {
            i= 750;
        }
        //550-20=530 from size of ball
        //ball bounces if...
        if ((((i<=iksz && iksz<=i+50)|| (i<= iksz + 20  && iksz+20<=i+50))) && (ipszilon==530))
        {
            flag1 = true;
        }
        else if (ipszilon>=+600)
        {
            break;
        }
        if (key == 27) break;

        switch (key) {
        case 'a':
            key = cvWaitKey(100);
            i=i-10;
            break;
        case 'd':
            key = cvWaitKey(100);
            i=i+10;
            break;
        }
        redraw();
    }


    return 0;
}
