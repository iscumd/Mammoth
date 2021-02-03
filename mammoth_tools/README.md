# Snow Generator

## Introduction

Going to use this to make the snow pucks for the simulation worlds. Ideally it will be able to make snow for any size world, but currently it is set up for the triple I.
I reccomend running the no_snow world/launch file.
The snow world needs to be optimized. It runs slowly. If you're using a laptop or older computer, it may not even run at all lol. I'm going to need help cutting down the resources that the world will need to run.

## Building

1. Run the code to generate the xml files that describe each and every snow puck.
2. Copy the text from the two files into an .SDF world file. 
3. Move .SDF world into the worlds folder of the simulation part of the heirarchy. 
4. Add or edit the launch file to reference the world file.
5. Run the install_gazebo_models bash script.
6. Run the launch file.

## Development

Needs to be updated to allow changing of snow variables.

## Documentation

There is some documentation in the source code. I have to add more and update the source code too.
