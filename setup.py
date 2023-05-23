from setuptools import setup

import glob
import os

package_name = 'waam_interpolation'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'model/meshes'), glob.glob('model/meshes/*.dae')),  #glob means
        (os.path.join('share', package_name, 'model'),        glob.glob('model/kuka_robot.urdf')),
        (os.path.join('share', package_name, 'launch'),       glob.glob('launch/kuka_launch.py')),
        (os.path.join('share', package_name, 'rviz'),         glob.glob('rviz/rvizconfig.rviz'))],
    
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='junjie',
    maintainer_email='junjie@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': ['digital_twin = waam_interpolation.kuka_sub:main',
                            'visualization_marker = waam_interpolation.tcp_marker:main',
                            'original_path = waam_interpolation.original_path:main',
                            'interpolated_path = waam_interpolation.interpolated_path:main',
                            
        ],
    },
)
