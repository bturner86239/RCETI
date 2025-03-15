from setuptools import find_packages, setup

package_name = 'rceti_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
        ],
    },
)
