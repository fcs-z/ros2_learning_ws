from setuptools import find_packages, setup

package_name = 'py03_tf_broadcaster'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
            'demo01_static_tf_broadcaster_py = py03_tf_broadcaster.demo01_static_tf_broadcaster_py:main',
            'demo02_dynamic_tf_broadcaster_py = py03_tf_broadcaster.demo02_dynamic_tf_broadcaster_py:main',
            'demo03_point_publisher_py = py03_tf_broadcaster.demo03_point_publisher_py:main'
        ],
    },
)
