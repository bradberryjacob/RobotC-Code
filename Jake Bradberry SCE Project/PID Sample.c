/*
* This file was written by Jacob Bradberry from the Gwinnett School of Mathematics, Science, and Technology for his 2012 - 2013 Senior Project
* Please give credit where credit is due
* If you have any problems or comments you can reach me at the email address jake1.gsmst@gmail.com
*/

#include "JakeAutonMain.c"

task main()
{
	string drive = "Drive";
	setupPID(.2, .001, .3, 700, drive);
	DrivePID(SensorValue(I2C_1), 800);
}
