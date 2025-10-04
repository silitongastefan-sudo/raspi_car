from setuptools import setup

package_name = 'hello_pubsub'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='stefan',
    maintainer_email='stefan@example.com',
    description='Simple publisher and subscriber demo',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hello_publisher = hello_pubsub.publisher_node:main',
            'hello_subscriber = hello_pubsub.subscriber_node:main',
        ],
    },
)

