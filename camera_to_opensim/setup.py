from setuptools import find_packages, setup

package_name = 'camera_to_opensim'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/convert.launch.py']),
    ],
    install_requires=['setuptools', 'numpy', 'opencv-python'],
    zip_safe=True,
    maintainer='gabri',
    maintainer_email='gabriel.novas@aimen.es',
    description='ROS2 package for Camera to OpenSim conversion',
    license='Alpha',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pose_estimation_node = camera_to_opensim.pose_estimation_node:main',
            'data_processing_node = camera_to_opensim.data_processing_node:main',
            'sports2d_node = camera_to_opensim.sports2d_node:main',
        ],
    },
)
