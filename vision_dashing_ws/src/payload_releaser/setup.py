from setuptools import setup

import os

# M2M_MODE: 0 = sim, 1 = mocap, 2 = hw. Mocap and hw both run the real releaser.
m2m_mode = os.environ.get('M2M_MODE', '')

if m2m_mode == "0":
	file_comp = 'payload_releaser = payload_releaser.payload_releaser_sim:main'
elif m2m_mode in ("1", "2"):
	file_comp = 'payload_releaser = payload_releaser.payload_releaser_rasp:main'
else:
	raise SystemExit("M2M_MODE must be set to 0 (sim), 1 (mocap) or 2 (hw); got '" + m2m_mode + "'")

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
