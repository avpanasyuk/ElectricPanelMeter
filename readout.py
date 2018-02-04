# -*- coding: utf-8 -*-
"""
Created on Sat Feb  3 16:33:52 2018

@author: panasyuk
"""

import requests
response = requests.get('http://192.168.2.169/read')
response.text

