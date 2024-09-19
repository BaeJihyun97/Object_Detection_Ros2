from setuptools import find_packages, setup
from glob import glob

package_name = 'drone_object_detection'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/conf', glob(f"{package_name}/conf/*.yaml")),
        ('share/' + package_name + '/weights/yolo', glob('weights/yolo/*.pt')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bjh',
    maintainer_email='thdvh5@g.skku.edu',
    description='Package for object detection processing using ROS 2',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'object_detector = drone_object_detection.object_detector:main',
        ],
    },
)
