from setuptools import find_packages, setup

package_name = 'simple_walk'

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
    maintainer='pi',
    maintainer_email='wwilliamcook@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simple_walk = simple_walk.simple_walk:main',
            'simple_narrow_walk = simple_walk.simple_narrow_walk:main',
            'simple_smooth_walk = simple_walk.simple_smooth_walk:main',
        ],
    },
)
