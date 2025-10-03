from setuptools import find_packages, setup

package_name = 'turtle_circle'

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
    maintainer='egoist',
    maintainer_email='mallaanirudh80@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'turtle_circle_publisher = turtle_circle.turtle_circle_publisher:main',
            'move_circle_client = turtle_circle.move_circle_client:main',
            'move_circle_server = turtle_circle.move_circle_server:main',
        ],
    },
)
