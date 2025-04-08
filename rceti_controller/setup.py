from setuptools import find_packages, setup

# This is a setup script for the rceti_controller package. Equivalent to CMakeLists.txt in CMake.

package_name = 'rceti_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/' + package_name, ['package.xml']),  # Install package.xml
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),  # Marker file
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bturner86239',
    maintainer_email='turner.1893@buckeyemail.osu.edu',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rceti_controller = rceti_controller.rceti_robot_controller:main',
            'test_controller = rceti_controller.test:main',
        ],
    },
)
