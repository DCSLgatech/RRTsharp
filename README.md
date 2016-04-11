RRT# in OMPL (standalone)
===================

This is the repository for RRT# planner development using OMPL.

----------


About
-------------
OMPL takes a long time to generate Python bindings and to compile.  Therefore, planner development is performed "standalone" using a system-installed version of OMPL.  Once development is complete, the planner can be integrated with the OMPL source code.

Note that OMPLapp's visual tools cannot be used without integration.  This code uses an extremely simple OMPL demo and as a program is only useful for qualitative checks for correctness.  A more complex demo and/or benchmark will be integrated at a later date so that future developers can get more appropriate implementation and algorithmic feedback.

The DCSL lab's internal OMPL integration repository is separate from this repository.

----------


Requirements
-------------------

The following must be installed to use this code.

 - OMPL
 - Boost (OMPL prerequisite)
 - CMake (cross-platform build)
 - Python (to use plotpath.py simple visualizer)
	 - MATLAB or any other tool listed at [the OMPL path visualization page](http://ompl.kavrakilab.org/pathVisualization.html) may be used to view the "solution.txt" file instead
 - OpenMP is not used in the current version, but it is found if installed in case future versions are multithreaded

----------


Build and use
-------------------

First, ensure that the required software listed above is installed.  Then:

Clone the repository

    git clone https://github.gatech.edu/ksaigol3/RRTsharp.git

Make a build directory and change to it

    cd RRTsharp
    mkdir build
    cd build

Generate makefiles and build

    cmake ..
    make

This will create an executable called **RRTsharp**.  Run this executable to generate the solution.txt file containing a solution path for a simple 3D rigid body planning problem.
