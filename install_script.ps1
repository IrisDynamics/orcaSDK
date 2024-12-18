## This script is a quick and easy way to install the orcaSDK if you
## do not care about fine-grained control over your build and install.
## This script will likely require administrator privileges to run.


mkdir ./build
cd build

## ----- BUILDING ----- 

## CONFIGURE STEP
cmake ..
## BUILD STEP
cmake --build .
## At this point you can use the code from external projects by adding the path 
## to this directory as a parameter in your project's find_package() call.


## ----- INSTALLING -----

## If you want a custom install location or want to install without admin privileges 
## use the following line. This will also require passing the directory parameter 
## when using find_package().
# cmake --install . --prefix <YOUR INSTALL LOCATION>

## If you want to use the default install location, use the following line. In this
## case you do not require additional parameters when using find_package().
cmake --install .


cd ..