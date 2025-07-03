from glob import glob
from setuptools import find_packages, setup

package_name = 'py_serial_commander'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (f'share/{package_name}/launch', glob('launch/*.launch.py')),  # installs all .launch.py files in launch/
    ],
    install_requires=[
        'setuptools',
    ],
    zip_safe=True,
    maintainer='tobgui',
    maintainer_email='ktobgui@gmail.com',
    description='Python node that listens to cmd_vel and sends serial motor commands',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'serial_commander = py_serial_commander.serial_commander:main',
        ],
    },
)

