### 3. Create and config main.cpp, CMakeLists.txt, package.xml
Example Codes can be found at [here](https://github.com/wookjinAhn/perception_depalletizing_mapping/tree/master/depalletizing_mapping_demos)  
Copy files into your catkin package directory.  
**src/depalletizing_demo.cpp, CMakeLists/CMakeLists.txt, pacakgexml/package.xml**

```bash
cd ~/catkin_ws/src/depalletizing_demo

# copy depalletizing_demo.cpp
cd ~/catkin_ws/src/perception_depalletizing_mapping/depalletizing_mapping_demos/src/
cp depalletizing_demo.cpp ~/catkin_ws/src/depalletizing_demo/

# copy CMakeLists.txt
cd ~/catkin_ws/src/perception_depalletizing_mapping/depalletizing_mapping_demos/CMakeLists/
cp CMakeLists.txt ~/catkin_ws/src/depalletizing_demo/

# copy package.xml
cd ~/catkin_ws/src/perception_depalletizing_mapping/depalletizing_mapping_demos/packagexml/
cp package.xml ~/catkin_ws/src/depalletizing_demo/
```

### 4. Set you Camera Position infomation at "depalletizing_demo.launch"