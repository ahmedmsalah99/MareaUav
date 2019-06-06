# README #

This README would normally document whatever steps are necessary to get your application up and running.

### What is this repository for? ###

* Quick summary
* Version
* [Learn Markdown](https://bitbucket.org/tutorials/markdowndemo)

### How do I get set up? ###

* Summary of set up
* Configuration
* Dependencies
* Database configuration
* How to run tests
* Deployment instructions

### Contribution guidelines ###

* Writing tests
* Code review
* Other guidelines

### Who do I talk to? ###

* Repo owner or admin
* Other community or team contact

# AprilTAG Compile #
Download this archive:

[apriltags-raspberry](https://bitbucket.org/isme_robotics/mar_frankenrotor_ros/downloads/apriltags-raspberry.tar.gz)

Extract the .tar.gz file into a temporary directory. Issue the following commands:

```bash
make
sudo make install
```

Create a file named apriltag.pc and put it into /usr/local/lib/pkgconfig with this content:

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


