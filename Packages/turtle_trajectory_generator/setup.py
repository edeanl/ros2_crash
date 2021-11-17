from setuptools import setup

package_name = 'turtle_trajectory_generator'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    py_modules=[
        'turtle_trajectory_generator.teleop_twist_keyboard',
        'turtle_trajectory_generator.trajectory_generator_class',
        'turtle_trajectory_generator.applications.twist2cmd',
        'turtle_trajectory_generator.applications.poly_trajectory'
    ],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dean',
    maintainer_email='deane@chalmers.se',
    author='Graylin Trevor Jay, Austin Hendrix, Emmanuel Dean',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: BSD',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='Turtle trajectory functions for ros2 crash course.',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'teleop_twist_keyboard = turtle_trajectory_generator.teleop_twist_keyboard:main',
            'trajectory_generator_class = turtle_trajectory_generator.trajectory_generator_class',
            'twist2cmd = turtle_trajectory_generator.applications.twist2cmd:main',
            'poly_trajectory = turtle_trajectory_generator.applications.poly_trajectory:main'
        ],
    },
)
