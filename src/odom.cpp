#include "vex.h"
#include "odom.h"


class Point;
class Odometry {
public:
    float currentX = 0.0;
    float currentY = 0.0;
    float currentTheta = 0.0;
    float previousParallel = 0.0;
    float previousPerp = 0.0;
    float previousTheta = 0.0;
    const float dB = 2.5;  // Track width adjustment

    

    void update() {
        float currentParallel = Yaxis.position(rev) * (2 * M_PI);
        float currentPerp = Xaxis.position(rev) * (2 * M_PI);

        float deltaParallel = currentParallel - previousParallel;
        float deltaPerp = currentPerp - previousPerp;
        float currentTheta = inertial19.heading() * (M_PI/180.0);
        float deltaTheta = currentTheta - previousTheta;

        // Local displacement calculation
        float deltaXLocal, deltaYLocal;
        if(fabs(deltaTheta) < 0.01) {  // Threshold for straight movement
            deltaXLocal = deltaPerp;
            deltaYLocal = deltaParallel;
        } else {
            deltaXLocal = 2 * sin(deltaTheta/2) * (deltaPerp/deltaTheta + dB);
            deltaYLocal = 2 * sin(deltaTheta/2) * (deltaParallel/deltaTheta + 4.0);
        }

        // Global conversion
        float avgTheta = previousTheta + deltaTheta/2;
        currentX += deltaYLocal * cos(avgTheta) - deltaXLocal * sin(avgTheta);
        currentY += deltaYLocal * sin(avgTheta) + deltaXLocal * cos(avgTheta);

        previousTheta = currentTheta;
        previousParallel = currentParallel;
        previousPerp = currentPerp;

        
        Brain.Screen.clearScreen();
        Brain.Screen.printAt(10, 20, "X: %.2f Y: %.2f", currentX, currentY);
        Brain.Screen.printAt(10, 40, "Heading: %.1f°", currentTheta * 180/M_PI);
    }
/*
    Point getPosition() {
              return new Point(currentX, currentY, currentTheta);
          }*/
};






