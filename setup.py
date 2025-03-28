from setuptools import find_packages, setup

package_name = 'epuck_planning'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/worlds',  ['worlds/obstacle_arena.wbt']),
        ('share/' + package_name + '/launch', ['launch/robot.launch.py']),
        ('share/' + package_name + '/resource', ['resource/epuck.urdf']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='viswa',
    maintainer_email='vss.viswatejabottu@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'diff_driver = epuck_planning.diff_driver:main',
            'bug_zero = epuck_planning.bug_zero:main' 
        ],
    },
)
