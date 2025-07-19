
from setuptools import find_packages, setup
import os
from glob import glob
import subprocess


package_name = 'startup'

# Generate URDF from XACRO before install
xacro_path = os.path.join('urdf', 'gptpet.xacro')
urdf_path = os.path.join('urdf', 'gptpet.urdf')
if os.path.exists(xacro_path):
    try:
        subprocess.run(['xacro', xacro_path, '-o', urdf_path], check=True)
    except Exception as e:
        print(f"Warning: Could not generate URDF from XACRO: {e}")

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'urdf'), ['urdf/gptpet.xacro']),
        (os.path.join('share', package_name, 'urdf'), ['urdf/gptpet.urdf'] if os.path.exists('urdf/gptpet.urdf') else []),
    ],
    install_requires=['setuptools'],
    zip_safe=True,  
    maintainer='pj',
    maintainer_email='pjm7748@g.rit.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
