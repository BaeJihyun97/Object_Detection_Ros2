from setuptools import find_packages, setup

package_name = 'ip_camera_display_test'

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
    maintainer='bjh',
    maintainer_email='thdvh5@g.skku.edu',
    description='Package for displaying processed image from object detection node using ROS 2',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'webcam_displayer = ip_camera_display_test.webcam_displayer:main' 
        ],
    },
)
