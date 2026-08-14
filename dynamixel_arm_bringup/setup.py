import os
from setuptools import find_packages, setup
from glob import glob

package_name = 'dynamixel_arm_bringup'

for script in glob('motion/*'):
    if os.path.isfile(script):
        os.chmod(script, 0o755)

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*')),
        ('share/' + package_name + '/launch', glob('launch/*')),
        ('share/' + package_name + '/motion', glob('motion/*')),
        ('share/' + package_name + '/control_gui', glob('control_gui/*')),

        (os.path.join('lib', package_name), glob('motion/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='markus',
    maintainer_email='11525061@student.vgu.edu.vn',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'control_gui = control_gui.control_gui:main'
        ],
    },
)
