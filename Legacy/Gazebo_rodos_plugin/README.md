# Gazebo Rodos Plugin

Plugin to connect Rodos topics and Gazebo topics on Linux.
This allows "Software in the Loop"-testing for RODOS components
by simulating interaction with simulated hardware in Gazebo.
Gazebo simulates environments, robot hardware and sensors.


## Usage

In your existing RODOS components that you want to test, replace the `Topic` instances
that you want to share with Gazebo with `GazeboTopic` instances. E.g
```c++
Topic<int> exampleTopic(1, "example");
```
would become
```c++
GazeboTopic<int> exampleTopic(1, "example");
```
You can also use a preprocessor macro:
```c++
#define Topic GazeboTopic
```
When a RODOS component writes to that topic,
the value will be published to a Gazebo topic with the same name.

On the other hand, if a RODOS component wants to read a topic from Gazebo,
you need to create a GazeboTopic with the same name and type as the original Gazebo topic
and create a RODOS Subscriber for it.

Finally, for Gazebo to load your code you need to create a shared library
that links to the rodos_plugin and
[include that library in the world file (line 168)](example/world/cessna_demo.world).
See the [example](example/src/example.cpp) for an illustration on how to use the plugin.

You also need something that handles messages and controls the model on the Gazebo side
that were send by your RODOS components. See the
[Gazebo model plugin tutorial](http://gazebosim.org/tutorials?tut=plugins_model&cat=write_plugin)
and the [cessna example plugin](example/src/cessna.cpp) for more information.


## Installation
1. [Install Gazebo 9+](http://gazebosim.org/tutorials?cat=install).
2. Checkout the [Gazebo Rodos Plugin repository](https://github.com/Razrkraken/Gazebo_rodos_plugin)
   as a dependency of your project using git.
3. Download the RODOS dependency with the following command in the cloned repository:
   ```shell script
   git submodule update --init
   ```
4. Modify your CMakeLists.txt according to the
   [provided example CMakeLists.txt](example/CMakeLists.txt).

## Building

1. Create a build folder
   ```shell script
   mkdir build
   ```
2. Change to the build folder
   ```shell script
   cd build
   ```
3. Initialize the cmake project
   ```shell script
   cmake ..
   ```
4. Build the plugins
   ```shell script
   cmake --build .
   ```
   This will build RODOS and the two plugins and place them in the build folder.

## Running the example

Configure cmake to build the example:
```shell script
   cmake -DBUILD_GAZEBO_RODOS_PLUGIN_EXAMPLE=ON ..
   ```
Make sure that the paths to the libraries in the [world file](example/world/cessna_demo.world)
and in the [model file](example/world/cessnaRODOS/model.sdf) are correct.
E.g. you might need to change
```xml
        <plugin name="rodosPlugin" filename="cmake-build-debug/libexample.so"/>
```
to point to the library you built:
```xml
        <plugin name="rodosPlugin" filename="build/libexample.so"/>
```
Then you can use the run_example target in the example to build the example and start Gazebo:
```shell script
cmake --build . --target run_example
```
