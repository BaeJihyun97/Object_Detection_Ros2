from setuptools import find_packages, setup
from glob import glob

package_name = 'ip_camera_test'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # config 디렉토리의 모든 YAML 파일을 share/my_package/config 에 복사 
        ('share/' + package_name + '/conf', glob('conf/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bjh',
    maintainer_email='thdvh5@g.skku.edu',
    description='Package for IP WebCam processing using ROS 2',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ip_webcam_publisher = ip_camera_test.ip_webcam_publisher:main'
        ],
    },
)
