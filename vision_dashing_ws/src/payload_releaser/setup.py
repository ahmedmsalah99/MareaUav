from setuptools import setup

import os

m2m_sim = os.environ['M2M_SIM']

if m2m_sim=="1":
	file_comp = 'payload_releaser = payload_releaser.payload_releaser_sim:main'
elif m2m_sim=="0":
	file_comp = 'payload_releaser = payload_releaser.payload_releaser_rasp:main'
else:
	print("M2M_SIM variable not set; HINT: export M2M_SIM=1 if running simulation, 0 else")
	print("Compiling simulation files by default")
	file_comp = 'payload_releaser = payload_releaser.payload_releaser_sim:main'

package_name = 'payload_releaser'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Alessandro Sperinde',
    author_email='mikael@osrfoundation.org',
    maintainer='Mikael Arguedas',
    maintainer_email='mikael@osrfoundation.org',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='Examples of minimal subscribers using rclpy.',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            file_comp,
        ],
    },
)
