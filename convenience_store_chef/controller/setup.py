from setuptools import find_packages, setup
import glob
import os

package_name = 'controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/resource', glob.glob(os.path.join('resource', '*.json'))),
        ('share/' + package_name + '/resource', glob.glob(os.path.join('resource', '*.wav')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='leejinwon',
    maintainer_email='leejinwon@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tm = controller.task_manager:main',
            'motion = controller.motion:main',
        ],
    },
)
