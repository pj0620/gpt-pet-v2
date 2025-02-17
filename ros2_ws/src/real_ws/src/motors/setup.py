from setuptools import find_packages, setup

package_name = 'motors'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
      'setuptools',
      'pyserial'
    ],
    zip_safe=True,
    maintainer='pj',
    maintainer_email='pjm7748@g.rit.edu',
    description='motor control package to be run on the client',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
          'service = motors.motor_control:main',
        ],
    },
)
