from setuptools import setup

package_name = 'rceti_AI'  # Replace with your actual package name

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name + '/launch', ['launch/rceti_AI_launch.py']),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='adamelyamani',
    maintainer_email='elyamani.2@osu.edu',
    description='ROS 2 wrapper for real-time servo control using YOLO and live video',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rceti_AI = rceti_AI.rceti_AI:main'
        ],
    },
)