from setuptools import find_packages, setup

package_name = 'book_examples'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Book Author',
    maintainer_email='author@example.com',
    description='Code examples for the Physical AI & Humanoid Robotics Book',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'talker = book_examples.publisher_member_function:main',
            'listener = book_examples.subscriber_member_function:main',
        ],
    },
)