//For the world file, you'll have to copy the text generated in both files to the world file.
//Make sure to check the spacing in the file. The XML created goes into the beginning of the line (I accounted for different spacing present in the sdf world file).
//When I copied it in, I made a version without any snow so that I could easily copy/paste.
//Version 1.1 2/2/21 nlemus@umich.edu

#include <iostream>
#include <math.h>
#include <fstream>
using namespace std;

void getXY(int a, float* poseX, float* poseY, int count);

void getXY(int a, float* poseX, float* poseY, int count){

	float offsetL = 0.5; //Left lane
	float offsetC = -0.5; //center lane
	float offsetR = -1.5; //right lane
	float xOffset = -5; //to start the snow right in front of the robot
	*poseX = 0; //so we start in front of the robot each time a new snow lane is created.

	float offsetStepSize = 0.21; //It seemed like the pucks in the Single-I world were about 0.2-0.2245 units apart.

	/*
	These if statements generate the individual lanes of snow.
	I want to update these to work based on any given number of puck elements, but that can wait.
	PoseY:
	This is to count across the columns of each snow lane.
	The %5 comes from the width of the single-I snow.
	PoseX:
	This is to count the rows going up.
	*/

	if (a <= 249) {
		*poseY = offsetL + ((a%5)*offsetStepSize); 
		*poseX = *poseX + (count*offsetStepSize) + xOffset;
	};
	if (a >= 250 && a <= 499) {
		//a = a - 250; //Temporary fix; ex: 42%6 = 0 while not being the 5th element of this section. This will lead to weird stacking along the borders of the snow zones.
		//we don't need these line with modulo 5! yaay. I'll find a workaround for this for a general case say if you use modulo 6 like I was doing earlier.
		*poseY = offsetC + ((a%5)*offsetStepSize);
		*poseX = *poseX + (count * offsetStepSize) + xOffset;
	};
	if (a >= 500 && a <= 750) {
		//a = a - 500; //Temporary fix; see above.
		*poseY = offsetR + ((a%5)*offsetStepSize);
		*poseX = *poseX + (count * offsetStepSize) + xOffset;
	};

}

int main() {
	ofstream myfile;
	ofstream myfile2;
	myfile.open("SnowModels.xml"); //This goes at the top/middle of the world file.
	myfile2.open("FullModelDescription.xml"); //This goes at the bottom of the world file.
	//I want a way to generate one xml file with the whole world in it based on some input values. That can wait for now.

	float poseX, poseY;
	float poseZ = 0.008111;
	//The Z component (0.008111) is taken from the code for single-I snow pucks. 
	//I need to check to see where this places the pucks. I do not want the computer to have to simulate ~600 pucks falling all at once lol.
	int count = 0;

	float ixx = 0.145833; //Taken from Single I world
	float iyy = 0.145833; // ""
	float izz = 0.125; // ""
	float ixy = 0; // ""
	float ixz = 0; // ""
	float iyz = 0; // ""
	float cylinderRadius = 0.0950718; // ""
	float cylinderLength = 0.0162339; // ""
	int maxContacts = 5; // decreased from 10 -> 5. Will this decrease CPU load? I have to find some performance saving loopholes. I'll come back to this.

	for (int i = 0; i < 750; i++) {

		if (i % 5 == 0)
		{
			count++;
		}
		if (i == 0 || i == 250 || i == 500) {
			count = 0; //so we start in front of the robot each time a new snow lane is created.
		};

		getXY(i, &poseX, &poseY, count);

		//This first section has an extra two spaces in front of each line to match the world document I'll send in with this.
		myfile << "      <model name='unit_cylinder_clone_" << i << "'>\n";
		myfile << "        <pose frame=''>" << poseX << " " << poseY << " " << poseZ << " 0 0 0</pose>\n"; 
		myfile << "        <scale>1 1 1</scale>\n";
		myfile << "        <link name='link'>\n";
		myfile << "          <pose frame=''>" << poseX << " " << poseY << " " << poseZ << " 0 0 0</pose>\n";
		myfile << "          <velocity>0 0 0 0 0 0</velocity>\n";
		myfile << "          <acceleration>0 0 -9.8 0 -0 0</acceleration>\n";
		myfile << "          <wrench>0 0 -9.8 0 -0 0</wrench>\n";
		myfile << "        </link>\n";
		myfile << "      </model>\n";

		myfile2 << "    <model name='unit_cylinder_clone_" << i << "'>\n";
		myfile2 << "      <pose frame=''>" << poseX << " " << poseY << " " << poseZ << " 0 0 0</pose>\n";
		myfile2 << "      <link name='link'>\n";
		myfile2 << "        <inertial>\n";
		myfile2 << "          <mass>1</mass>\n";
		myfile2 << "          <inertia>\n";
		myfile2 << "            <ixx>" << ixx << "</ixx>\n";
		myfile2 << "            <ixy>" << ixy << "</ixy>\n";
		myfile2 << "            <ixz>" << ixz << "</ixz>\n";
		myfile2 << "            <iyy>" << iyy << "</iyy>\n";
		myfile2 << "            <iyz>" << iyz << "</iyz>\n";
		myfile2 << "            <izz>" << izz << "</izz>\n";
		myfile2 << "          </inertia>\n";
		myfile2 << "        </inertial>\n";
		myfile2 << "        <collision name='collision'>\n";
		myfile2 << "          <geometry>\n";
		myfile2 << "            <cylinder>\n";
		myfile2 << "              <radius>" << cylinderRadius << "</radius>\n";
		myfile2 << "              <length>" << cylinderLength << "</length>\n";
		myfile2 << "            </cylinder>\n";
		myfile2 << "          </geometry>\n";
		myfile2 << "          <max_contacts>" << maxContacts << "</max_contacts>\n";
		myfile2 << "          <surface>\n";
		myfile2 << "            <contact>\n";
		myfile2 << "              <ode/>\n";
		myfile2 << "            </contact>\n";
		myfile2 << "            <bounce/>\n";
		myfile2 << "            <friction>\n";
		myfile2 << "              <torsional>\n";
		myfile2 << "                <ode/>\n";
		myfile2 << "              </torsional>\n";
		myfile2 << "              <ode/>\n";
		myfile2 << "            </friction>\n";
		myfile2 << "          </surface>\n";
		myfile2 << "        </collision>\n";
		myfile2 << "        <visual name='visual'>\n";
		myfile2 << "          <geometry>\n";
		myfile2 << "            <cylinder>\n";
		myfile2 << "              <radius>" << cylinderRadius << "</radius>\n";
		myfile2 << "              <length>" << cylinderLength << "</length>\n";
		myfile2 << "            </cylinder>\n";
		myfile2 << "          </geometry>\n";
		myfile2 << "          <material>\n";
		myfile2 << "            <script>\n";
		myfile2 << "              <name>Gazebo/Grey</name>\n"; //this one could be a variable too later down the line
		myfile2 << "              <uri>file://media/materials/scripts/gazebo.material</uri>\n"; //could be a variable too later down the line 
		myfile2 << "            </script>\n";
		myfile2 << "          </material>\n";
		myfile2 << "        </visual>\n";
		myfile2 << "        <self_collide>0</self_collide>\n";
		myfile2 << "        <kinematic>0</kinematic>\n";
		myfile2 << "      </link>\n";
		myfile2 << "    </model>\n";
	};

	myfile.close();
	myfile2.close();
	return 0;
}
