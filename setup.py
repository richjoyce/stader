from setuptools import setup, find_packages


with open('README.rst') as f:
    readme = f.read()

setup(
    name='stader',
    version='0.0.1',
    description='stability derivatives for aircraft package',
    long_description=readme,
    author='Richard Joyce',
    author_email='rjoyce@ucdavis.edu',
    url='https://github.com/richjoyce/stader',
    license='BSD (3-clause)',
    packages=find_packages(exclude=('tests', 'docs', 'examples')),
    package_data={'stader': ['data/*.json']},
    install_requires=['numpy', 'scipy'],
    classifiers=[
                 'Intended Audience :: Science/Research',
                 'Programming Language :: Python :: 2.7',
                 'Programming Language :: Python :: 3.3',
                 'Programming Language :: Python :: 3.4',
                 'License :: OSI Approved :: BSD License',
                 'Topic :: Scientific/Engineering'
                 ]
)
