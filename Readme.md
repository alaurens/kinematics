# Forward Kinematics Library

This repository can be use to compute the forward kinematics of an arbitrary
serial robot.

## Installation, tests and example.

#### Before installing
Before installing please ensure that you have downloaded installed:
- cmake
- Eigen
- Abseil
- google_test and google_mock


#### Installation
To install the library, please `git clone` this repository and build in using
cmake using the following set of commands:

```
git clone something something
mkdir build
cd build 
cmake ../
cmake --build .
```

#### Running the tests and the example.
You should now be able to run the tests using:
```
ctest
```

As well as running the example executable `main.cc` with:
```
./main
```

You can play with the example by modifying the `main.cc`

## Example

In the file `main.cc`, you will an example using the forward kinematics
library applied to the three joint robot described in Fig 4.3 of the 2019
edition of "Modern Robotics Mechanics, Planning and Control." By Kevin M.Lynch
and Frank C.Park.

You can modify the lengths of the links of the robot as well as the joint
configuration the robot is in. You can then build and run the binary:
```
cmake --build .
./main
```

This will print out the solution to the forward kinemtics for the given joint
configuration.

## Using the library for a new serial robot

To use the library to compute the forward kinematics for your own robot you
will need to implement and instance of the `Robot` interface located in
`include/robot.h` The main function you will need to implement is the 
`GetDHParameters` which returns the list of Denavit-Hartenberg parameters 
for each joint of the robot starting with the joint closest to the base.

Once you have implemented the interface you will then be able to use your
new robot class with the `ForwardKinematics` function. Congrats!