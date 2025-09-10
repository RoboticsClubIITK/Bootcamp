from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'turtle_pub'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'srv'), glob('srv/*.srv'))
    ],
    install_requires=['setuptools', 'rosidl_default_generators'],
    zip_safe=True,
    maintainer='vivekananda',
    maintainer_email='vivekananda@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
              'draw_circle_service=turtle_pub.draw_circle_service:main',
              'move_circle = turtle_pub.move_circle:main',
        ],
    },
)