#!/bin/sh

# Compile protobufs to C++ files and build project  
cd proto
mkdir -p generated

echo Delete previously generated header files
find . -name "*.h" -exec rm {}  \;

echo Delete previously generated cpp files
find . -name "*.cpp" -exec rm {}  \;

PROTO_COMPILER=/usr/local/bin/protoc 
echo Using protobuf compiler at $PROTO_COMPILER version $($PROTO_COMPILER --version)
find . -name "*.proto" -exec $PROTO_COMPILER -I=./ --cpp_out=./generated {}  \;

cd ../src
make -j4
