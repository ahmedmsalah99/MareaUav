# Installation #

## GeographicLib Compile ##
Download this archive:

[GeographicLib-raspberry](https://bitbucket.org/isme_robotics/mar_frankenrotor_ros/downloads/GeographicLib-1.49.tar.gz)

Extract the .tar.gz. file into a temporary directory. Issue the following commands:

```bash
cd build
make
sudo make install
cd ..
cd tools
cd @GEOGRAPHICLIB_DATA@
sudo mkdir /usr/local/share/GeographicLib/
sudo cp -rv * /usr/local/share/GeographicLib/
```

## AprilTAG Compile ##
Download this archive:

[apriltags-raspberry](https://bitbucket.org/isme_robotics/mar_frankenrotor_ros/downloads/apriltags-raspberry.tar.gz)

Extract the .tar.gz file into a temporary directory. Issue the following commands:

```bash
make
sudo make install
```

Create a file named apriltag.pc and put it into `/usr/local/lib/pkgconfig` with this content:

```bash
prefix=/usr/local
exec_prefix=${prefix}
libdir=${exec_prefix}/lib
includedir=${exec_prefix}/include

Name: apriltag
Description: Vision system
Version: 3.0
Libs: -L${libdir} -lapriltag
Cflags: -I${includedir}/apriltag/
```


