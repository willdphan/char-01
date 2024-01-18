# setup to try and make predict.py work in launch file

from setuptools import setup
from setuptools import find_packages

package_name = 'ros2_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'predict = scripts.predict:main',
            'control = scripts.control:main'
        ],
    },
)
