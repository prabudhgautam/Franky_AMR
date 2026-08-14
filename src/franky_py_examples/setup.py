from setuptools import find_packages, setup

package_name = 'franky_py_examples'

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
    maintainer='prabudh',
    maintainer_email='prabudhrocky2003@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'publisher = franky_py_examples.publisher:main',
            'subscriber = franky_py_examples.subscriber:main',
            'turtle_subscriber = franky_py_examples.turtle_subcriber:main'
        ],
    },
)
