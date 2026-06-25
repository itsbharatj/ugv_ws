from glob import glob
import os

from setuptools import find_packages, setup


package_name = 'ugv_semantic_navigation'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'data'), glob('data/*.json')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Bharat Jain',
    maintainer_email='bharat@todo.todo',
    description='Semantic mapping, memory retrieval, and object navigation for the UGV rover.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'biased_frontier_explorer = ugv_semantic_navigation.biased_frontier_explorer:main',
            'semantic_camera_observer = ugv_semantic_navigation.semantic_camera_observer:main',
            'semantic_memory_node = ugv_semantic_navigation.semantic_memory_node:main',
            'semantic_nav_node = ugv_semantic_navigation.semantic_nav_node:main',
            'semantic_memory_mcp_server = ugv_semantic_navigation.semantic_memory_mcp_server:main',
        ],
    },
)
