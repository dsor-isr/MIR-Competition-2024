from setuptools import find_packages, setup

package_name = 'glassy_challenge'

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
    maintainer='joaolehodey',
    maintainer_email='joao.lehodey@gmail.com',
    description='This package is used for the MIR 2024 challenge',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'glassy_challenge = glassy_challenge.glassy_challenge:main',
        ],
    },
)

