mkdir ./build
cd build
cmake ..
cmake --build . --config Debug
# cmake --install . --prefix ./install_output --config Debug
cmake --install . --config Debug
cd ..