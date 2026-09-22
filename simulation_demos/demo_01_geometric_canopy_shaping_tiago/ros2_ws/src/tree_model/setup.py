from setuptools import find_packages, setup

package_name = 'tree_model'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Maintainer',
    maintainer_email='maintainer@example.com',
    description='Synthetic foliage model and trimming-path visualization for the tree-shaping MVP.',
    license='Proprietary',
    entry_points={
        'console_scripts': [
            'tree_generator = tree_model.tree_generator:main',
            'path_generator = tree_model.path_generator:main',
        ],
    },
)
