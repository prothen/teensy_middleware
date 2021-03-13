"""


Author:     Philipp Rothenhaeusler
            Smart Mobility Lab, KTH
            Stockholm 2020

"""
import os
import sys
import yaml
import datetime


sys.stdout = open('./log/test_hooks.log', 'w')
print('Pre-build scripts executed successfully: {0}'.format(datetime.datetime.now()))
sys.stdout = sys.__stdout__
