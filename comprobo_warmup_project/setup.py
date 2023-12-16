from setuptools import find_packages, setup

package_name = 'comprobo_warmup_project'

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
    maintainer='alana',
    maintainer_email='sprout4631@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'marker_publisher = comprobo_warmup_project.marker_publisher:main',
            'odom_marker_publisher = comprobo_warmup_project.odom_marker_publisher:main',
            'teleop = comprobo_warmup_project.teleop:main',
        ],
    },
)
