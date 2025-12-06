from setuptools import find_packages, setup
from glob import glob

package_name = 'py01_launch'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # launch
        ('share/' + package_name, glob("launch/*.launch.py")),
        ('share/' + package_name, glob("launch/*.launch.xml")),
        ('share/' + package_name, glob("launch/*.launch.yaml"))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gcu68',
    maintainer_email='gcu68@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        ],
    },
)
