from setuptools import setup
from glob import glob

package_name = 'cube_detection'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yao',
    maintainer_email='yao@example.com',
    description='Cube perception nodes (simple + standard) for orchestration integration',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Perception nodes (no driver control)
            'cube_detector_simple_node = cube_detection.cube_detector_simple_node:main',
            'cube_detector_standard_node = cube_detection.cube_detector_standard_node:main',

            # Optional: keep your chase controller if you still want it for testing
            # (Your file name in the screenshot is cube_chase_standard_node.py)
            'cube_chase_standard_node = cube_detection.cube_chase_standard_node:main',
        ],
    },
)