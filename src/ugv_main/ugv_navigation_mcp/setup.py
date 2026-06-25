from glob import glob
import os

from setuptools import find_packages, setup


package_name = 'ugv_navigation_mcp'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Bharat Jain',
    maintainer_email='bharat@todo.todo',
    description='FastMCP tools for occupancy-grid reasoning, Nav2 goals, and depth summaries.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'navigation_mcp_server = ugv_navigation_mcp.navigation_mcp_server:main',
        ],
    },
)
