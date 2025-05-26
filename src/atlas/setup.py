from setuptools import setup
from glob import glob

package_name = 'atlas'

launch_files = glob('launch/*.launch')
data_files =[
    ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
]
if launch_files:
    data_files.append(('share/' + package_name + '/launch', glob('launch/*.launch')))

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='albertofinardi',
    maintainer_email='finara@usi.ch',
    description='ATLAS package',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'controller_node = atlas.controller_node:main',
            'line_pid_node = atlas.line_pid_node:main',
            'traffic_detection_node = atlas.traffic_detection_node:main'
        ],
    },
)
