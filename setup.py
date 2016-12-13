from setuptools import setup, find_packages


with open('README.rst') as f:
    readme = f.read()

with open('LICENSE') as f:
    license = f.read()

setup(
    name='stader',
    version='0.0.1',
    description='stability derivatives for aircraft package',
    long_description=readme,
    author='Richard Joyce',
    author_email='rjoyce@ucdavis.edu',
    url='https://github.com/richjoyce/stader',
    license=license,
    packages=find_packages(exclude=('tests', 'docs', 'examples'))
)
